"""Модуль работы с веб-камерами (USB V4L2 /dev/video*, OpenCV/FFmpeg, MJPEG-стриминг).

Обеспечивает:
1. Захват видеопотоков с четырёх Logitech C930e: передняя левая (id 0),
   задняя левая (id 1), передняя правая (id 2), задняя правая (id 3).
   Порты и углы установки берутся из .env (см. sensor_layout.py); id 0 и 1
   сохранены за старыми «передняя/задняя» для совместимости экранов и API.
2. Раздачу MJPEG-видеопотока (/api/camera/<id>/stream) и отдельных кадров (/api/camera/<id>/frame).
3. Приём кадров от внешних ROS-нод или скриптов (POST /api/camera/<id>/frame).
4. Список доступных камер (/api/cameras) для фронтенда, с ролью и углом установки.
5. Проверку свежести и USB-пути: если кадра нет, не подставляем изображение-заглушку.
"""

from __future__ import annotations

from contextlib import closing
import logging
import os
import shutil
import threading
import time
from typing import Any, Generator, Optional

from .camera_capture import ffmpeg_frames, modes, opencv_frames

log = logging.getLogger("robot_control.camera")


class CameraDevice:
    """Управление одной веб-камерой: фоновый захват кадров и сжатие в JPEG."""

    def __init__(self, cam_id: int, device_path: str, name: str = "", *,
                 role: str = "", spec: Optional[dict] = None, target_provider=None, role_checker=None,
                 revalidate_provider=None):
        self.cam_id = cam_id
        self.device_path = device_path
        self.name = name or f"Камера {cam_id}"
        self.role = role
        self.spec: dict = dict(spec or {})
        self.lock = threading.Lock()
        self.latest_jpeg: Optional[bytes] = None
        self.latest_time: float = 0.0
        self.frame_source = ""
        self.opened_path: Optional[str] = None
        self._opened_fingerprint = None
        self._target_provider = target_provider
        self._revalidate_provider = revalidate_provider
        self._role_checker = role_checker
        self.capture_error = ""
        self.capture_mode = ""
        self.capture_fourcc = None
        self.frames_received = 0
        self.dark_frame = None
        self.width = self.height = self.fps = None  # only report measured frames
        self.is_active = False
        self._stop = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop = False
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop = True
        with self.lock:
            self.is_active = False
            self.opened_path = None
            self._opened_fingerprint = None
            self.latest_jpeg = None
            self.latest_time = 0.0
            self.frame_source = ""

    def feed_frame(self, jpeg_data: bytes) -> None:
        """Ручная загрузка кадра (от внешнего источника / ROS)."""
        with self.lock:
            self.latest_jpeg = jpeg_data
            self.latest_time = time.time()
            self.frame_source = "external"
            self.is_active = True

    def role_verified(self) -> Optional[bool]:
        if self._role_checker is None:
            return None
        try:
            confirmed = self._role_checker()
            if confirmed is None:
                return None
            with self.lock:
                captured = self.frame_source == "capture" and bool(self.opened_path)
                opened = self.opened_path
            return bool(confirmed and captured and opened == self._target())
        except Exception:
            return False

    def _target(self, *, revalidate: bool = False) -> str:
        provider = self._revalidate_provider if revalidate and self._revalidate_provider else self._target_provider
        if provider is None:
            return self.device_path
        try:
            return str(provider() or "")
        except Exception:  # a broken identity check must not open a guessed camera
            log.exception("Камера %s: не удалось проверить USB", self.cam_id)
            return ""

    @staticmethod
    def _fingerprint(path: str):
        try:
            st = os.stat(path)
            return st.st_dev, st.st_ino, st.st_rdev, st.st_ctime_ns
        except OSError:
            return None

    def get_frame(self, max_age_s: float = 2.0) -> Optional[bytes]:
        with self.lock:
            frame, stamp, source = self.latest_jpeg, self.latest_time, self.frame_source
            path, fingerprint = self.opened_path, self._opened_fingerprint
        if not frame or not 0 <= time.time() - stamp < max_age_s:
            return None
        if source == "capture" and (not path or not fingerprint or
                                    self._fingerprint(path) != fingerprint or self._target() != path):
            return None
        return frame

    def _capture_loop(self) -> None:
        # OpenCV's Linux select() otherwise waits 10s on a silent UVC stream.
        os.environ.setdefault("OPENCV_VIDEOIO_V4L_SELECT_TIMEOUT", "2")
        backend = (os.environ.get("RC_CAMERA_BACKEND") or "auto").strip().lower()
        if backend not in ("auto", "opencv", "ffmpeg"):
            self.capture_error = "RC_CAMERA_BACKEND: укажите auto, opencv или ffmpeg"
            return
        cv2 = None
        if backend != "ffmpeg":
            try:
                import cv2
            except (ImportError, OSError, RuntimeError):
                pass
        ffmpeg = shutil.which("ffmpeg") if backend != "opencv" else None
        profiles = modes(cv2 is not None, ffmpeg is not None)
        if not profiles:
            self.capture_error = (f"Нет средства захвата на борту (RC_CAMERA_BACKEND={backend}). "
                                  "Для FFmpeg: sudo apt install ffmpeg; "
                                  "для OpenCV: .venv/bin/pip install opencv-python-headless")
            return
        previous = None
        attempt = 0
        while not self._stop:
            dev_target = self._target()
            # The inventory worker refreshes absent paths. Avoid repeatedly
            # traversing all sysfs from four idle capture threads.
            if not dev_target or not os.path.exists(dev_target):
                self.capture_error = (f"{dev_target}: устройство не подключено" if dev_target else
                                      "Проверьте назначение видеовхода в разделе «Устройства»")
                time.sleep(0.5)
                continue
            # Before each open inspect current sysfs: a replacement at the
            # same /dev/videoN must not inherit the old confirmed role.
            dev_target = self._target(revalidate=True)
            # Never fall back to the numeric cam_id when a path disappears:
            # /dev/videoN may now be a different camera or even the Kinect.
            if not dev_target or not os.path.exists(dev_target):
                self.capture_error = "Видеовход не подтверждён. Проверьте устройство и выбранный порт"
                time.sleep(0.5)
                continue
            fingerprint = self._fingerprint(dev_target)
            if not fingerprint:
                time.sleep(.25)
                continue
            identity = (dev_target, fingerprint)
            if identity != previous:
                previous, attempt = identity, 0
            mode = profiles[attempt]
            self.capture_mode = mode.label

            def current(*, revalidate=False):
                return (not self._stop and self._fingerprint(dev_target) == fingerprint and
                        self._target(revalidate=revalidate) == dev_target)

            try:
                if not os.access(dev_target, os.R_OK | os.W_OK):
                    raise PermissionError("нет доступа; проверьте группу video у бортового процесса")
                reader = (opencv_frames(cv2, dev_target, mode, current) if mode.backend == "OpenCV" else
                          ffmpeg_frames(ffmpeg, dev_target, mode, current))
                first = True
                with closing(reader):
                    for frame in reader:
                        # Also validate *after* the first read: changing format or
                        # starting FFmpeg can reopen a node after the initial check.
                        if not current(revalidate=first):
                            break
                        if first:
                            log.info("Камера %s: первый JPEG %s · %s", self.cam_id, dev_target, mode.label)
                        first = False
                        with self.lock:
                            if self._stop:
                                break
                            self.opened_path = dev_target
                            self._opened_fingerprint = fingerprint
                            self.latest_jpeg = frame.jpeg
                            self.latest_time = time.time()
                            self.frame_source = "capture"
                            self.frames_received += 1
                            self.width, self.height, self.fps = frame.width, frame.height, frame.fps
                            self.capture_fourcc, self.dark_frame = frame.fourcc, frame.dark
                            self.capture_error = ""
                            self.is_active = True
            except Exception as exc:  # driver error, no frames, codec missing, etc.
                self.capture_error = f"{dev_target} · {mode.label}: {exc}"
                log.debug("Камера %s: %s", self.cam_id, self.capture_error)
            finally:
                with self.lock:
                    self.is_active = False
                    self.opened_path = None
                    self._opened_fingerprint = None
                    # Do not serve a previous camera's last frame after a swap.
                    if self.frame_source == "capture":
                        self.latest_jpeg = None
                        self.latest_time = 0.0
                        self.frame_source = ""
                attempt = (attempt + 1) % len(profiles)
                if not self._stop:
                    time.sleep(1.0 if attempt == 0 else .25)


class CameraManager:
    """Управляет четырьмя камерами C930e на борту.

    id 0 — передняя левая, 1 — задняя левая, 2 — передняя правая,
    3 — задняя правая. Первые две сохраняют старые маршруты
    ``/api/camera/front`` и ``/api/camera/rear``.
    """

    _instance: Optional[CameraManager] = None
    _instance_lock = threading.RLock()
    _registry = None

    def __init__(self):
        self.cameras: dict[int, CameraDevice] = {}
        self.lock = threading.Lock()
        self.registry = type(self)._registry
        self._init_devices()

    @classmethod
    def get_instance(cls) -> CameraManager:
        # Flask serves several camera/status requests concurrently.
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = CameraManager()
            return cls._instance

    @classmethod
    def configure_registry(cls, registry) -> None:
        """Set the onboard registry before first capture (and in app tests)."""
        with cls._instance_lock:
            cls._registry = registry
            if cls._instance is not None:
                cls._instance.registry = registry

    def _make_device(self, spec: dict) -> CameraDevice:
        cam_id, path = int(spec["id"]), str(spec["port"])
        role = str(spec.get("role") or "")
        def target():
            return self.registry.camera_target(role, path) if self.registry is not None else path
        def revalidate():
            return self.registry.camera_target(role, path, refresh=True) if self.registry is not None else path
        def role_checker():
            return self.registry.camera_role_confirmed(role) if self.registry is not None else None
        return CameraDevice(cam_id, path, str(spec.get("label") or f"Камера {cam_id}"),
                            role=role, spec=spec, target_provider=target, role_checker=role_checker,
                            revalidate_provider=revalidate)

    @classmethod
    def reset_instance(cls) -> None:
        """Сброс синглтона (для тестов)."""
        with cls._instance_lock:
            if cls._instance is not None:
                for dev in list(cls._instance.cameras.values()):
                    dev.stop()
                cls._instance = None
            cls._registry = None

    def _resolve_device_path(self, cam_id: int, default: str) -> str:
        """Путь камеры из .env (RC_CAMERA_FRONT/REAR) или дефолт."""
        # Ленивый импорт, чтобы избежать цикла
        try:
            from .camera_settings import camera_front_from_env, camera_rear_from_env
        except Exception:
            return default
        if cam_id == 0:
            val = camera_front_from_env()
            return val if val else default
        if cam_id == 1:
            val = camera_rear_from_env()
            return val if val else default
        return default

    @staticmethod
    def _specs() -> list[dict]:
        """Геометрия и ручные назначения; при ошибке не угадывать порт по ID."""
        try:
            from .sensor_layout import camera_specs
            return camera_specs()
        except Exception:  # noqa: BLE001 — настройки не должны ронять экран
            log.exception("Не удалось прочитать настройки камер")
            return [{"id": i, "role": role, "port": ""} for i, role in
                    enumerate(("front_left", "rear_left", "front_right", "rear_right"))]

    def _init_devices(self) -> None:
        """Инициализация четырёх камер с учётом портов из .env."""
        for spec in self._specs():
            dev = self._make_device(spec)
            self.cameras[dev.cam_id] = dev
            # A missing configured path must also reconnect when it appears.
            dev.start()

    def current_settings(self) -> dict[str, str]:
        """Текущие настройки портов из окружения (четыре роли)."""
        try:
            from .camera_settings import current_camera_settings

            return current_camera_settings()
        except Exception:
            return {"front": "", "rear": "", "front_right": "", "rear_right": ""}

    def reconfigure(self, settings: dict[str, str]) -> dict[str, str]:
        """Переключить все четыре камеры на новые порты без перезапуска.

        ``apply_camera_settings`` уже записала значения в ``os.environ``;
        здесь дополнительно подхватываем явно переданный словарь (удобно из
        тестов), после чего читаем геометрию заново.
        """
        try:
            from .camera_settings import ENV_KEYS_BY_ROLE
            for key, value in (settings or {}).items():
                env_keys = ENV_KEYS_BY_ROLE.get(str(key))
                if not env_keys:
                    continue
                cleaned = str(value or "").strip()
                if cleaned:
                    os.environ[env_keys[0]] = cleaned
                else:
                    os.environ.pop(env_keys[0], None)
        except Exception:  # noqa: BLE001 — совместимость со старым словарём
            pass
        specs = self._specs()
        self.refresh(specs)
        return {str(spec.get("role") or spec["id"]): str(spec["port"]) for spec in specs}

    def refresh(self, specs=None) -> None:
        """Discard old frames and re-evaluate paths after an operator hint."""
        specs = specs if specs is not None else self._specs()
        with self.lock:
            old_devices = list(self.cameras.values())
            for dev in old_devices:
                dev.stop()
            for dev in old_devices:
                if dev._thread and dev._thread is not threading.current_thread():
                    dev._thread.join(timeout=0.5)
            self.cameras.clear()
            for spec in specs:
                dev = self._make_device(spec)
                self.cameras[dev.cam_id] = dev
                dev.start()

    def get_camera(self, cam_id: int) -> Optional[CameraDevice]:
        return self.cameras.get(cam_id)

    def feed_frame(self, cam_id: int, jpeg_bytes: bytes) -> bool:
        dev = self.get_camera(cam_id)
        if not dev:
            spec = next((s for s in self._specs() if int(s["id"]) == cam_id), None)
            dev = CameraDevice(cam_id, f"/dev/video{cam_id}", f"Камера {cam_id}",
                               role=str((spec or {}).get("role") or ""), spec=spec)
            self.cameras[cam_id] = dev
        dev.feed_frame(jpeg_bytes)
        return True

    def get_frame(self, cam_id: int, max_age_s: float = 2.0) -> Optional[bytes]:
        dev = self.get_camera(cam_id)
        if dev:
            frame = dev.get_frame(max_age_s)
            if frame:
                return frame
        return None

    def list_cameras(self) -> list[dict[str, Any]]:
        result = []
        for cid, dev in sorted(list(self.cameras.items())):
            frame_present = bool(dev.get_frame(max_age_s=2.0))
            with dev.lock:
                frame_age = max(0.0, time.time() - dev.latest_time) if frame_present and dev.latest_time else None
                frame_source = dev.frame_source if frame_present else ""
                captured = frame_source == "capture"
                opened_path = dev.opened_path if captured else None
                dark, fourcc = (dev.dark_frame, dev.capture_fourcc) if captured else (None, None)
                width, height, fps = (dev.width, dev.height, dev.fps) if captured else (None, None, None)
            result.append(
                {
                    "id": cid,
                    "name": dev.name,
                    "device": dev.device_path,
                    "openedPath": opened_path,
                    "frameSource": frame_source,
                    "frameAgeS": round(frame_age, 3) if frame_age is not None else None,
                    "captureError": dev.capture_error or None,
                    "captureMode": dev.capture_mode or None,
                    "captureFourcc": fourcc,
                    "framesReceived": dev.frames_received,
                    "darkFrame": dark,
                    "exists": os.path.exists(opened_path or dev.device_path),
                    "active": frame_present,
                    "width": width,
                    "height": height,
                    "fps": fps,
                    "role": dev.role,
                    "model": dev.spec.get("model", "Logitech C930e"),
                    "yaw_deg": dev.spec.get("yaw_deg", 0.0),
                    "hfov_deg": dev.spec.get("hfov_deg", 82.0),
                    "stream_url": f"/api/camera/{cid}/stream",
                    "frame_url": f"/api/camera/{cid}/frame",
                }
            )
        return result

    def generate_mjpeg_stream(self, cam_id: int) -> Generator[bytes, None, None]:
        """Генератор multipart/x-mixed-replace для MJPEG стриминга в тег <img>."""
        dev = self.get_camera(cam_id)
        boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"

        while True:
            frame = dev.get_frame() if dev else None
            if frame:
                yield boundary + frame + b"\r\n"
            time.sleep(0.04)  # ~25 fps
