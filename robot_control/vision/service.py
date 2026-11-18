"""Сервис распознавания: один поток на четыре камеры, состояние для API/экрана.

Сервис не меняет поведение привода: он только публикует рамки, цвет светофора
и отметки на поле. Решение «ехать/стоять» остаётся за навигацией
(``road_traffic.py`` / ``ROAD_NAV_PROPOSAL.md``), а здесь формируются факты —
``action`` и ``signal`` — которые она может использовать.
"""
from __future__ import annotations

import logging
import math
import os
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Optional

from .classes import CLASS_INFO, CLASS_NAMES, TRAFFIC_LIGHT_LABELS
from .detector import (Detection, EnsembleDetector, HeuristicDetector, YoloDetector,
                       cv2_available)
from .traffic import TrafficLightTracker
from .closed_course import ClosedCourseWaitPolicy

log = logging.getLogger("robot_control.vision")

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL = PROJECT_ROOT / "models" / "vision" / "yolo-stop.pt"

#: Подписи на кадре: cv2.putText не умеет кириллицу без внешнего шрифта.
ASCII_TAG = {
    "stop_sign": "STOP",
    "pedestrian_crossing": "PED CROSSING",
    "speed_bump": "SPEED BUMP",
    "traffic_light": "TRAFFIC LIGHT",
    "cone": "CONE",
}
SIGNAL_RGB = {"red": (60, 60, 235), "yellow": (60, 215, 255),
              "green": (80, 210, 90), "unknown": (170, 170, 170)}


def _env_flag(name: str, default: bool = True) -> bool:
    raw = (os.environ.get(name) or "").strip().lower()
    if not raw:
        return default
    return raw in {"1", "true", "yes", "on"}


def _env_float(name: str, default: float, lo: float, hi: float) -> float:
    try:
        value = float((os.environ.get(name) or "").strip() or default)
    except (TypeError, ValueError):
        return default
    return max(lo, min(hi, value)) if math.isfinite(value) else default


def _env_cameras(name: str = "RC_VISION_CAMERAS") -> tuple[int, ...]:
    raw = (os.environ.get(name) or "0,1,2,3").strip()
    ids: list[int] = []
    for chunk in raw.split(","):
        try:
            cam_id = int(chunk)
        except ValueError:
            continue
        if 0 <= cam_id <= 3 and cam_id not in ids:
            ids.append(cam_id)
    return tuple(ids) if ids else (0, 1, 2, 3)


@dataclass
class VisionConfig:
    enabled: bool = True
    model_path: Path = DEFAULT_MODEL
    cameras: tuple[int, ...] = (0, 1, 2, 3)
    interval_s: float = 0.25
    confidence: float = 0.45
    prefer_red: bool = False
    max_field_m: float = 6.0
    traffic_stale_s: float = 2.0
    sign_scale: float = 1.0
    imgsz: int = 416
    device: str = "cpu"
    traffic_mode: str = "legacy"  # verified is explicit bench opt-in, never a new default
    heuristic_enabled: bool = True  # False reproduces the YOLO-only bench profile, no fallback
    closed_course_unknown_timeout: bool = False  # advisory only, NEVER implicit motor permission

    @classmethod
    def from_env(cls) -> "VisionConfig":
        model = (os.environ.get("RC_VISION_MODEL") or "").strip()
        return cls(
            enabled=_env_flag("RC_VISION_ENABLED", True),
            model_path=Path(model) if model else DEFAULT_MODEL,
            cameras=_env_cameras(),
            interval_s=_env_float("RC_VISION_INTERVAL", 0.25, 0.05, 5.0),
            confidence=_env_float("RC_VISION_CONF", 0.45, 0.05, 0.95),
            prefer_red=_env_flag("RC_VISION_PREFER_RED", False),
            max_field_m=_env_float("RC_VISION_MAX_FIELD_M", 6.0, 1.0, 30.0),
            traffic_stale_s=_env_float("RC_VISION_TRAFFIC_STALE_S", 2.0, 0.2, 30.0),
            sign_scale=_env_float("RC_VISION_SIGN_SCALE", 1.0, 0.25, 4.0),
            imgsz=int(_env_float("RC_VISION_IMGSZ", 416, 160, 1280)) // 32 * 32,
            device=(os.environ.get("RC_VISION_DEVICE") or "cpu").strip() or "cpu",
            traffic_mode=(os.environ.get("RC_VISION_TRAFFIC_MODE") or "legacy").strip().lower() or "legacy",
            heuristic_enabled=_env_flag("RC_VISION_HEURISTIC", True),
            closed_course_unknown_timeout=_env_flag("RC_VISION_CLOSED_COURSE_TIMEOUT", False),
        )


@dataclass
class CameraVision:
    """Последний разобранный кадр одной камеры."""

    cam_id: int
    stamp: float = 0.0
    evidence_stamp: float = 0.0
    width: int = 0
    height: int = 0
    detections: list[Detection] = field(default_factory=list)
    jpeg: Optional[bytes] = None
    annotated: Optional[bytes] = None
    error: str = ""

    def to_dict(self, now: float) -> dict[str, Any]:
        return {
            "id": self.cam_id,
            "active": bool(self.jpeg) and now - self.stamp <= 2.0,
            "ageS": round(max(0.0, now - self.stamp), 3) if self.stamp else None,
            "frameStamp": self.evidence_stamp or None,
            "captureAgeS": round(max(0.0, now - self.evidence_stamp), 3) if self.evidence_stamp else None,
            "width": self.width,
            "height": self.height,
            "detections": [d.to_dict() for d in self.detections],
            "error": self.error or None,
        }


class VisionService:
    """Опрашивает кадры ``CameraManager``, детектирует и строит отметки поля.

    ``camera_provider`` — функция, возвращающая менеджер камер. Её можно
    подменить в тестах, чтобы не открывать USB-устройства.
    """

    def __init__(self, camera_provider: Optional[Callable[[], Any]] = None,
                 config: Optional[VisionConfig] = None,
                 layout_provider: Optional[Callable[[], list[dict[str, Any]]]] = None) -> None:
        self.config = config or VisionConfig.from_env()
        self._camera_provider = camera_provider
        self._layout_provider = layout_provider
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._detector: Any = None
        self._backend = "unavailable"
        self._reason = "сервис ещё не инициализирован"
        self._cameras: dict[int, CameraVision] = {cid: CameraVision(cid) for cid in range(4)}
        self._traffic = TrafficLightTracker(stale_s=self.config.traffic_stale_s,
                                            prefer_red=self.config.prefer_red)
        self._closed_course = ClosedCourseWaitPolicy(
            enabled=self.config.closed_course_unknown_timeout, stale_s=self.config.traffic_stale_s)
        self._stamps: dict[int, float] = {}
        self._field: list[dict[str, Any]] = []
        self._frames = 0
        self._last_latency_ms = 0.0

    # ------------------------------------------------------------ инициализация
    def initialize(self) -> str:
        """Выбрать YOLO (если есть веса) или встроенный детектор."""
        with self._lock:
            model = self.config.model_path
            if model.is_file():
                try:
                    yolo = YoloDetector(str(model), confidence=self.config.confidence,
                                        prefer_red=self.config.prefer_red,
                                        imgsz=self.config.imgsz, device=self.config.device,
                                        traffic_mode=self.config.traffic_mode)
                    if self.config.traffic_mode in ("body", "verified"):
                        self._detector = yolo
                        self._backend = "yolo+" + self.config.traffic_mode
                        self._reason = f"модель {model.name}; режим {self.config.traffic_mode}, без эвристического ансамбля"
                    elif cv2_available() and self.config.heuristic_enabled:
                        heuristic = HeuristicDetector(
                            min_confidence=max(0.5, self.config.confidence),
                            prefer_red=self.config.prefer_red)
                        self._detector = EnsembleDetector(yolo, heuristic)
                        self._backend = "yolo+heuristic"
                        self._reason = f"модель {model.name} + встроенный детектор"
                    else:
                        self._detector = yolo
                        self._backend = "yolo"
                        self._reason = f"модель {model.name}"
                    return self._backend
                except Exception as exc:  # noqa: BLE001 - битые веса не роняют экран
                    log.warning("не удалось загрузить YOLO %s: %s", model, exc)
                    self._reason = f"модель не загрузилась: {exc}"
            if self.config.traffic_mode != "legacy" or not self.config.heuristic_enabled:
                # Missing/bad weights or an invalid mode must not silently
                # fall back to unverified HSV/heuristic green.
                # Disabling heuristics disables BOTH extras and fallback.
                self._detector = None
                self._backend = "unavailable"
                self._reason = "требуются корректные веса YOLO; резерв отключён; " + self._reason
                return self._backend
            if cv2_available():
                self._detector = HeuristicDetector(
                    min_confidence=max(0.5, self.config.confidence),
                    prefer_red=self.config.prefer_red)
                self._backend = "heuristic"
                if model.is_file():
                    log.info("резервный детектор знаков: %s", self._reason)
                else:
                    self._reason = "веса YOLO не найдены — работает встроенный детектор"
                return self._backend
            self._detector = None
            self._backend = "unavailable"
            self._reason = "Не установлены OpenCV/NumPy: см. requirements-vision.txt"
            return self._backend

    @property
    def backend(self) -> str:
        return self._backend

    def _manager(self):
        if self._camera_provider is not None:
            return self._camera_provider()
        from ..camera import CameraManager
        return CameraManager.get_instance()

    def _specs(self) -> list[dict[str, Any]]:
        if self._layout_provider is not None:
            return self._layout_provider()
        from ..sensor_layout import camera_specs
        return camera_specs()

    # ------------------------------------------------------------------ поток
    def start(self) -> None:
        """Запустить фоновый разбор кадров (ничего не делает без детектора)."""
        if not self.config.enabled:
            self._reason = "выключено в .env (RC_VISION_ENABLED=0)"
            return
        if self._thread and self._thread.is_alive():
            return
        self.initialize()
        if self._detector is None:
            log.info("распознавание знаков недоступно: %s", self._reason)
            return
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="rc-vision", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread and thread.is_alive():
            thread.join(timeout=1.5)
        self._thread = None
        with self._lock:
            self._closed_course.invalidate("stopped")

    def _loop(self) -> None:
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                manager = self._manager()
                for cam_id in self.config.cameras:
                    if self._stop.is_set():
                        break
                    stamp = self._frame_stamp(manager, cam_id)
                    if stamp is None or stamp <= self._stamps.get(cam_id, 0.0):
                        continue
                    jpeg = manager.get_frame(cam_id, max_age_s=1.0)
                    if not jpeg:
                        continue
                    self.process_frame(cam_id, jpeg, stamp=stamp)
            except Exception as exc:  # noqa: BLE001 - камеры могли быть переподключены
                log.debug("цикл распознавания: %s", exc)
            self._last_latency_ms = (time.monotonic() - started) * 1000.0
            self._stop.wait(max(0.02, self.config.interval_s))

    @staticmethod
    def _frame_stamp(manager, cam_id: int) -> Optional[float]:
        dev = manager.get_camera(cam_id)
        if dev is None:
            return None
        with dev.lock:
            if not dev.latest_jpeg:
                return None
            return float(dev.latest_time)

    # --------------------------------------------------------------- кадр
    def process_frame(self, cam_id: int, jpeg: bytes, *, stamp: Optional[float] = None) -> dict[str, Any]:
        """Разобрать один JPEG. Публичный метод для тестов и внешних кадров."""
        if cam_id not in range(4):
            raise ValueError("id камеры должен быть 0…3")
        # ``frame_stamp`` — время получения кадра (ключ дедупликации); свежесть
        # визуальных отметок считаем уже ПОСЛЕ вывода модели: первый запуск
        # YOLO на CPU может занять больше секунды. Но для разрешающего сигнала
        # отдельно сохраняем время самого кадра, не завершающего inference.
        frame_stamp = time.time() if stamp is None else float(stamp)
        with self._lock:
            detector = self._detector
        if detector is None:
            self.initialize()
            with self._lock:
                detector = self._detector
        state = self._cameras[cam_id]
        if detector is None or not cv2_available():
            finished = time.time()
            state.stamp, state.jpeg, state.detections = finished, jpeg, []
            state.error = self._reason
            self._stamps[cam_id] = frame_stamp
            with self._lock:
                self._closed_course.invalidate("frame_error")
            return state.to_dict(finished)
        import cv2
        import numpy as np

        try:
            buffer = np.frombuffer(jpeg, dtype=np.uint8)
            frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError("кадр не декодируется как JPEG")
            started = time.monotonic()
            detections = detector.detect(frame)
            self._last_latency_ms = (time.monotonic() - started) * 1000.0
            annotated = self._annotate(frame, detections)
        except Exception as exc:  # noqa: BLE001
            finished = time.time()
            state.stamp, state.jpeg, state.detections = finished, jpeg, []
            state.annotated = None
            state.error = str(exc)
            self._stamps[cam_id] = frame_stamp
            with self._lock:
                self._closed_course.invalidate("frame_error")
            return state.to_dict(finished)
        ok, encoded = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
        h, w = frame.shape[:2]
        finished = time.time()
        with self._lock:
            state.stamp, state.jpeg = finished, jpeg
            state.evidence_stamp = frame_stamp
            state.width, state.height = w, h
            state.detections = detections
            state.annotated = encoded.tobytes() if ok else jpeg
            state.error = ""
            self._stamps[cam_id] = frame_stamp
            self._frames += 1
            self._update_traffic(finished)
            self._field = self._build_field()
        return state.to_dict(finished)

    def _annotate(self, frame, detections: list[Detection]):
        """Нарисовать рамки и подписи на копии кадра (для /api/vision/camera/N/frame)."""
        import cv2

        image = frame.copy()
        h, w = image.shape[:2]
        for det in detections:
            x, y, bw, bh = det.box
            p1, p2 = (int(x * w), int(y * h)), (int((x + bw) * w), int((y + bh) * h))
            rgb = SIGNAL_RGB.get(det.color or "", (0, 0, 0)) if det.cls == "traffic_light" else (0, 0, 0)
            info = CLASS_INFO[det.cls]
            if det.cls != "traffic_light":
                hex_color = info["color"].lstrip("#")
                rgb = tuple(int(hex_color[i:i + 2], 16) for i in (4, 2, 0))  # #RRGGBB → BGR
            cv2.rectangle(image, p1, p2, rgb, 2)
            signal = f" {det.color.upper()}" if det.cls == "traffic_light" and det.color and det.color != "unknown" else ""
            text = f"{ASCII_TAG[det.cls]} {det.confidence:.2f}{signal}"
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            ty = max(th + 4, p1[1])
            cv2.rectangle(image, (p1[0], ty - th - 4), (p1[0] + tw + 6, ty + 2), rgb, -1)
            cv2.putText(image, text, (p1[0] + 3, ty - 2), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (255, 255, 255), 1, cv2.LINE_AA)
        return image

    # ------------------------------------------------------- светофор и поле
    def _update_traffic(self, now: float) -> None:
        best: Optional[tuple[float, int, Detection]] = None
        totals = {name: 0 for name in ("red", "yellow", "green")}
        for cam_id, state in self._cameras.items():
            if now - state.stamp > 1.0:
                continue
            for det in state.detections:
                if det.cls != "traffic_light":
                    continue
                for name in totals:
                    totals[name] += int(det.color_counts.get(name, 0))
                score = det.confidence
                if self.config.prefer_red:
                    # Более уверенный green с другой камеры не отменяет red.
                    score += {"red": 3, "yellow": 2, "unknown": 1, "green": 0}.get(det.color or "unknown", 1)
                if best is None or score > best[0]:
                    best = (score, cam_id, det)
        self._traffic.set_counts(totals)
        if best is None:
            self._traffic.update("unknown", 0.0, 0, stamp=now)
        else:
            _, cam_id, det = best
            counts = det.color_counts or {}
            total = sum(counts.values())
            confidence = (counts.get(det.color or "", 0) / total) if total else 0.0
            self._traffic.update(det.color or "unknown", confidence, cam_id, stamp=now,
                                 evidence_stamp=self._cameras[cam_id].evidence_stamp or self._cameras[cam_id].stamp)
        self._update_closed_course(now)

    def _update_closed_course(self, now: float) -> None:
        if not self._closed_course.enabled:
            return
        if not self.config.enabled or self._detector is None:
            self._closed_course.invalidate("unavailable")
            return
        candidates = []
        for cam_id in self.config.cameras:
            state = self._cameras[cam_id]
            observed = state.evidence_stamp
            age = now - observed
            if (state.error or not state.jpeg or observed <= 0 or
                    not math.isfinite(age) or not 0 <= age <= self._closed_course.stale_s or
                    not 0 <= now - state.stamp <= 1.0):
                continue
            for det in state.detections:
                if (det.cls == "traffic_light" and math.isfinite(det.confidence) and
                        det.confidence >= self.config.confidence):
                    candidates.append((det.confidence, cam_id, det, observed, age))
        if not candidates:
            self._closed_course.invalidate("no_fresh_light")
            return
        _, cam_id, det, observed, age = max(candidates, key=lambda item: (item[0], -item[1]))
        self._closed_course.update(
            camera_id=cam_id, box=det.box, frame_stamp=observed, frame_age_s=age,
            signal=det.color or "unknown", confirmed_signal=self._traffic.snapshot(now=now)["state"],
            stop_signal_present=any(item[2].color in ("red", "yellow") for item in candidates))

    def _build_field(self) -> list[dict[str, Any]]:
        """Спроецировать рамки на плоскость поля в системе координат робота."""
        specs = {int(s["id"]): s for s in self._specs()}
        markers: list[dict[str, Any]] = []
        now = time.time()
        for cam_id, state in self._cameras.items():
            if now - state.stamp > 1.5 or not state.width or not state.height:
                continue
            spec = specs.get(cam_id, {})
            yaw = math.radians(float(spec.get("yaw_deg", 0.0)))
            hfov = math.radians(float(spec.get("hfov_deg", 82.0)))
            vfov = math.radians(float(spec.get("vfov_deg", 52.0)))
            pitch = math.radians(float(spec.get("pitch_deg", 0.0)))
            camera_z = float(spec.get("z_m", 0.28))
            focal_px = (state.width / 2.0) / math.tan(hfov / 2.0) if hfov > 0 else state.width
            scale = max(0.25, min(4.0, float(self.config.sign_scale)))
            for index, det in enumerate(state.detections):
                if det.cls == "cone":
                    continue  # конусы — препятствия, на карте ПДД их не отмечаем
                x, y, bw, bh = det.box
                center_x, center_y = x + bw / 2.0, y + bh / 2.0
                bearing = yaw + (0.5 - center_x) * hfov
                info = CLASS_INFO[det.cls]
                # Физический размер знака: и для оценки дальности, и для 3D 1:1.
                height_m = float(info["height_m"]) * scale
                width_m = float(info["width_m"]) * scale
                thickness_m = float(info["thickness_m"]) * scale
                box_px = max(1.0, bh * state.height)
                distance = height_m * focal_px / box_px
                distance = max(0.3, min(self.config.max_field_m, distance))
                px = float(spec.get("x_m", 0.0)) + distance * math.cos(bearing)
                py = float(spec.get("y_m", 0.0)) + distance * math.sin(bearing)
                # Высота центра знака над полом: пробуем вертикальный угол
                # рамки, но доверяем ему только при правдоподобном результате.
                # На учебных фото камера была в руках (~1 м), а на роботе
                # RC_CAMERA_*_Z_M = 0.28 м: без этой проверки знаки «лягут»
                # на пол. Тогда берём типовую высоту макета из CLASS_INFO.
                typical_z = float(info["center_height_m"]) * scale
                min_center = height_m / 2.0 + 0.15
                vertical = max(-1.2, min(1.2, (0.5 - center_y) * vfov - pitch))
                z_m = camera_z + distance * math.tan(vertical)
                z_source = "camera-geometry"
                if (not math.isfinite(z_m) or z_m < min_center or z_m > 2.2 or
                        abs(z_m - typical_z) > 0.6):
                    z_m, z_source = typical_z, "CLASS_INFO"
                z_m = max(0.15, min(2.2, z_m))
                heading = math.degrees(math.atan2(py, px))
                markers.append({
                    "id": f"cam{cam_id}-{det.cls}-{index}",
                    "class": det.cls, "label": info["label"], "short": info["short"],
                    "color": info["color"], "action": info["action"],
                    "confidence": round(det.confidence, 3),
                    "cameraId": cam_id,
                    "x_m": round(px, 3), "y_m": round(py, 3),
                    "z_m": round(z_m, 3), "centerZ_m": round(z_m, 3),
                    "zSource": z_source,
                    "bearingDeg": round(math.degrees(bearing), 1),
                    "headingDeg": round(heading, 1),
                    "distanceM": round(distance, 2),
                    "width_m": round(width_m, 3),
                    "height_m": round(height_m, 3),
                    "thickness_m": round(thickness_m, 3),
                    "shape": info["shape"], "sizeScale": round(scale, 3),
                    "scale1to1": abs(scale - 1.0) < 1e-6,
                    "positionSource": "camera-estimate",
                    "signal": det.color,
                    "ageS": round(max(0.0, now - state.stamp), 3),
                })
        return markers

    # ----------------------------------------------------------------- API
    def state(self) -> dict[str, Any]:
        now = time.time()
        with self._lock:
            cameras = {str(cid): self._cameras[cid].to_dict(now) for cid in range(4)}
            counts = {name: 0 for name in CLASS_NAMES}
            for value in cameras.values():
                for det in value["detections"]:
                    counts[det["class"]] += 1
            traffic = self._traffic.snapshot(now=now)
            if not self.config.enabled or self._detector is None:
                self._closed_course.invalidate("unavailable")
            traffic["closedCourse"] = self._closed_course.snapshot()
            # Отметки поля живут не дольше кадров, на которых они найдены:
            # иначе при потере камеры на карте останутся «призрачные» знаки.
            field = [dict(marker, ageS=round(max(0.0, now - self._cameras[marker["cameraId"]].stamp), 3))
                     for marker in self._field
                     if now - self._cameras[marker["cameraId"]].stamp <= 1.5]
            payload = {
                "ok": self._backend != "unavailable",
                "enabled": bool(self.config.enabled and self._detector is not None),
                "backend": self._backend,
                "model": str(self.config.model_path) if self._backend.startswith("yolo") else None,
                "imgsz": self.config.imgsz,
                "device": self.config.device,
                "trafficMode": self.config.traffic_mode,
                "heuristicEnabled": self._backend in ("heuristic", "yolo+heuristic"),
                "reason": self._reason,
                "cameras": cameras,
                "counts": counts,
                "trafficLight": traffic,
                "trafficLightLabels": TRAFFIC_LIGHT_LABELS,
                "field": field,
                "classes": [{"name": name, **CLASS_INFO[name]} for name in CLASS_NAMES],
                "frames": self._frames,
                "inferenceMs": round(self._last_latency_ms, 1),
                "intervalS": self.config.interval_s,
                "updatedAt": round(now, 3),
                "running": bool(self._thread and self._thread.is_alive()),
            }
        return payload

    def annotated_frame(self, cam_id: int) -> Optional[bytes]:
        with self._lock:
            state = self._cameras.get(cam_id)
            if not state or state.error or not state.jpeg or not 0 <= time.time() - state.evidence_stamp < 2.0:
                return None
            return state.annotated or state.jpeg

    def configure(self, *, enabled: Optional[bool] = None,
                  confidence: Optional[float] = None) -> dict[str, Any]:
        """Изменить настройки на ходу (без перезапуска бортового экрана)."""
        with self._lock:
            if enabled is not None:
                self.config.enabled = bool(enabled)
                if not self.config.enabled:
                    for state in self._cameras.values():
                        state.detections = []
                    self._field = []
                    self._traffic.reset()
                    self._closed_course.invalidate("disabled")
            if confidence is not None:
                self._closed_course.invalidate("configuration_changed")
                self.config.confidence = max(0.05, min(0.95, float(confidence)))
                setter = getattr(self._detector, "set_confidence", None)
                if callable(setter):
                    setter(self.config.confidence)
                elif isinstance(self._detector, HeuristicDetector):
                    self._detector.min_confidence = max(0.5, self.config.confidence)
                elif isinstance(self._detector, YoloDetector):
                    self._detector.confidence = self.config.confidence
            if self.config.enabled and self._detector is None:
                self.initialize()
            if self.config.enabled and not (self._thread and self._thread.is_alive()):
                self.start()
        return self.state()

    def reset(self) -> None:
        with self._lock:
            for cid in range(4):
                self._cameras[cid] = CameraVision(cid)
            self._stamps.clear()
            self._field = []
            self._frames = 0
            self._traffic.reset()
            self._closed_course.reset()


_service: Optional[VisionService] = None
_service_lock = threading.RLock()


def get_vision_service() -> VisionService:
    """Один сервис на процесс: и веб-пульт, и киоск видят одни и те же рамки."""
    global _service
    with _service_lock:
        if _service is None:
            _service = VisionService()
        return _service
