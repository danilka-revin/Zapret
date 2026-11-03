"""Модуль работы с веб-камерами (USB V4L2 /dev/video*, OpenCV, MJPEG-стриминг).

Обеспечивает:
1. Захват видеопотоков с четырёх Logitech C930e: передняя левая (id 0),
   задняя левая (id 1), передняя правая (id 2), задняя правая (id 3).
   Порты и углы установки берутся из .env (см. sensor_layout.py); id 0 и 1
   сохранены за старыми «передняя/задняя» для совместимости экранов и API.
2. Раздачу MJPEG-видеопотока (/api/camera/<id>/stream) и отдельных кадров (/api/camera/<id>/frame).
3. Приём кадров от внешних ROS-нод или скриптов (POST /api/camera/<id>/frame).
4. Список доступных камер (/api/cameras) для фронтенда, с ролью и углом установки.
5. Генерацию тестового кадра в режиме demo / при отсутствии оборудования.
"""

from __future__ import annotations

import glob
import logging
import os
import threading
import time
from typing import Any, Generator, Optional

log = logging.getLogger("robot_control.camera")


class CameraDevice:
    """Управление одной веб-камерой: фоновый захват кадров и сжатие в JPEG."""

    def __init__(self, cam_id: int, device_path: str, name: str = "", *,
                 role: str = "", spec: Optional[dict] = None):
        self.cam_id = cam_id
        self.device_path = device_path
        self.name = name or f"Камера {cam_id}"
        self.role = role
        self.spec: dict = dict(spec or {})
        self.lock = threading.Lock()
        self.latest_jpeg: Optional[bytes] = None
        self.latest_time: float = 0.0
        self.width = 640
        self.height = 360
        self.fps = 30
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
        self.is_active = False

    def feed_frame(self, jpeg_data: bytes) -> None:
        """Ручная загрузка кадра (от внешнего источника / ROS)."""
        with self.lock:
            self.latest_jpeg = jpeg_data
            self.latest_time = time.time()
            self.is_active = True

    def get_frame(self, max_age_s: float = 2.0) -> Optional[bytes]:
        with self.lock:
            if self.latest_jpeg and (time.time() - self.latest_time) < max_age_s:
                return self.latest_jpeg
        return None

    def _capture_loop(self) -> None:
        try:
            import cv2
        except ImportError:
            log.info("Камера %s: cv2 не установлен, ожидается внешний поток или браузерный доступ", self.cam_id)
            return

        cap = None
        while not self._stop:
            try:
                # Открываем устройство через V4L2 или номер индекса
                dev_target = self.device_path if os.path.exists(self.device_path) else self.cam_id
                cap = cv2.VideoCapture(dev_target, cv2.CAP_V4L2 if hasattr(cv2, "CAP_V4L2") else cv2.CAP_ANY)
                if not cap.isOpened():
                    time.sleep(2.0)
                    continue

                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                cap.set(cv2.CAP_PROP_FPS, self.fps)
                self.is_active = True
                log.info("Камера %s подключена (%s)", self.cam_id, self.device_path)

                while not self._stop and cap.isOpened():
                    ret, frame = cap.read()
                    if not ret or frame is None:
                        time.sleep(0.04)
                        continue

                    # Кодируем кадр в JPEG
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 75]
                    ret, buf = cv2.imencode(".jpg", frame, encode_param)
                    if ret:
                        with self.lock:
                            self.latest_jpeg = buf.tobytes()
                            self.latest_time = time.time()

                    time.sleep(1.0 / self.fps)

            except Exception as exc:  # noqa: BLE001
                log.debug("Ошибка захвата камеры %s: %s", self.cam_id, exc)
            finally:
                self.is_active = False
                if cap is not None:
                    cap.release()
                    cap = None
                time.sleep(2.0)


class CameraManager:
    """Управляет четырьмя камерами C930e на борту.

    id 0 — передняя левая, 1 — задняя левая, 2 — передняя правая,
    3 — задняя правая. Первые две сохраняют старые маршруты
    ``/api/camera/front`` и ``/api/camera/rear``.
    """

    _instance: Optional[CameraManager] = None

    def __init__(self):
        self.cameras: dict[int, CameraDevice] = {}
        self.lock = threading.Lock()
        self._init_devices()

    @classmethod
    def get_instance(cls) -> CameraManager:
        if cls._instance is None:
            cls._instance = CameraManager()
        return cls._instance

    @classmethod
    def reset_instance(cls) -> None:
        """Сброс синглтона (для тестов)."""
        if cls._instance is not None:
            for dev in list(cls._instance.cameras.values()):
                dev.stop()
            cls._instance = None

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
        """Геометрия и порты из .env; при ошибке — безопасный набор из двух камер."""
        try:
            from .sensor_layout import camera_specs
            return camera_specs()
        except Exception:  # noqa: BLE001 — настройки не должны ронять экран
            return [
                {"id": 0, "role": "front_left", "label": "Камера передняя (CAM 0)",
                 "port": "/dev/video0", "yaw_deg": 0.0, "hfov_deg": 82.0},
                {"id": 1, "role": "rear_left", "label": "Камера задняя (CAM 1)",
                 "port": "/dev/video1", "yaw_deg": 180.0, "hfov_deg": 82.0},
            ]

    def _init_devices(self) -> None:
        """Инициализация четырёх камер с учётом портов из .env."""
        for spec in self._specs():
            cam_id = int(spec["id"])
            dev_path = str(spec["port"])
            dev = CameraDevice(cam_id, dev_path, str(spec.get("label") or f"Камера {cam_id}"),
                               role=str(spec.get("role") or ""), spec=spec)
            self.cameras[cam_id] = dev
            if os.path.exists(dev_path):
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
        with self.lock:
            for dev in list(self.cameras.values()):
                dev.stop()
            time.sleep(0.15)
            self.cameras.clear()
            for spec in specs:
                cam_id = int(spec["id"])
                dev_path = str(spec["port"])
                dev = CameraDevice(cam_id, dev_path,
                                   str(spec.get("label") or f"Камера {cam_id}"),
                                   role=str(spec.get("role") or ""), spec=spec)
                self.cameras[cam_id] = dev
                if os.path.exists(dev_path):
                    dev.start()
                else:
                    # Устройство не подключено — оставляем объект для feed_frame
                    log.info("Камера %s: порт %s не найден, жду подключения", cam_id, dev_path)
        return {str(spec.get("role") or spec["id"]): str(spec["port"]) for spec in specs}

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
        for cid, dev in sorted(self.cameras.items()):
            active = dev.is_active or bool(dev.get_frame(max_age_s=2.0))
            result.append(
                {
                    "id": cid,
                    "name": dev.name,
                    "device": dev.device_path,
                    "exists": os.path.exists(dev.device_path),
                    "active": active,
                    "width": dev.width,
                    "height": dev.height,
                    "fps": dev.fps,
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
