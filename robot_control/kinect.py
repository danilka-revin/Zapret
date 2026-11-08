"""Kinect 360 как второй источник карты: кадр глубины → псевдо-LaserScan.

Драйвер Kinect живёт отдельным процессом (``scripts/kinect_depth_publisher.py``)
и присылает кадр на борт: `POST /api/kinect/depth`. Здесь только математика и
состояние, без usb/freenect/ROS-зависимостей, поэтому модуль работает и в
песочнице, и в тестах.

Кадр глубины (16-битные значения, по умолчанию миллиметры) переводится в
полярные точки `{"a": рад, "r": м}` **в системе координат робота**: ось X —
вперёд, Y — влево, угол 0 — вперёд, как у лидара. Точки пола/потолка и всё,
что ближе/дальше рабочего диапазона, отбрасываются. `SlamMapService` вписывает
эти лучи в ту же occupancy grid, что и лидар, но **не** использует их для
scan matching: позу по-прежнему определяет проверенный 360° лидар.

Поддерживаются три формата входящего сообщения:
* `{"width", "height", "depth": <base64 uint16 LE>, "units": "mm"}` — сырой кадр;
* `{"ranges": [...], "angle_min", "angle_max", "stamp"}` — готовый скан;
* `{"points": [{"a", "r"}], "stamp"}` — точки от внешнего конвертера (ROS 2).
"""
from __future__ import annotations

import array
import base64
import binascii
import importlib.util
import math
import re
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional, Sequence

from .lidar import normalize_angle, scan_from_ranges
from .sensor_layout import kinect_spec

MAX_PIXELS = 640 * 480
MAX_DEPTH_BYTES = 2 * 1024 * 1024
MAX_POINTS = 2000
# Ограничение нагрузки бортового экрана: не более 160×120 отсчётов.
PREVIEW_WIDTH = 160
PREVIEW_HEIGHT = 120


class DepthError(ValueError):
    """Некорректный кадр глубины (не роняет сервер, отдаётся 400)."""


@dataclass(frozen=True)
class DepthFrame:
    width: int
    height: int
    values: Sequence[float]  # метры, 0 = нет данных
    stamp: float
    fx: float = 0.0
    fy: float = 0.0


BACKENDS: tuple[tuple[str, str, str], ...] = (
    ("freenect", "freenect", "pip install freenect (libfreenect + USB)"),
    ("openni", "openni2", "pip install openni2 (OpenNI2 + primesense-ros)"),
    ("ros2", "rclpy", "ROS 2 + /camera/depth/image_rect_raw (requirements-kinect.txt)"),
)


def available_backends() -> dict[str, Any]:
    """Какие драйверы Kinect реально импортируются в этом процессе.

    Диагностика для ``/api/kinect/status``: «кадра нет» и «драйвер не стоит» —
    разные неисправности, и экран должен их различать.
    """
    found: dict[str, bool] = {}
    for name, module, _hint in BACKENDS:
        try:
            found[name] = importlib.util.find_spec(module) is not None
        except (ImportError, ValueError):
            found[name] = False
    return {
        "ok": True,
        "backends": found,
        "installed": [name for name, present in found.items() if present],
        "hints": {name: hint for name, _module, hint in BACKENDS if not found[name]},
    }


def _number(value: Any, name: str, lo: float, hi: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise DepthError(f"{name}: ожидается число") from exc
    if not math.isfinite(result) or not lo <= result <= hi:
        raise DepthError(f"{name}: значение вне допустимого диапазона")
    return result


def decode_depth_payload(payload: Any, *, clock=time.time) -> DepthFrame:
    """Разобрать и проверить сырой кадр глубины из JSON."""
    if not isinstance(payload, dict):
        raise DepthError("ожидается JSON-объект с кадром глубины")
    width = int(_number(payload.get("width"), "width", 2, 640))
    height = int(_number(payload.get("height"), "height", 2, 480))
    if width * height > MAX_PIXELS:
        raise DepthError("кадр глубины слишком большой; пошлите уменьшенный кадр")
    raw = payload.get("depth")
    if isinstance(raw, str):
        try:
            data = base64.b64decode(raw, validate=True)
        except (binascii.Error, ValueError) as exc:
            raise DepthError("depth: некорректный base64") from exc
        if len(data) > MAX_DEPTH_BYTES:
            raise DepthError("кадр глубины слишком большой")
        if len(data) != width * height * 2:
            raise DepthError("depth: размер не совпадает с width × height (uint16)")
        samples = array.array("H")
        samples.frombytes(data)
        if sys.byteorder != "little":
            samples.byteswap()
        scale = _number(payload.get("scale", 0.001), "scale", 1e-6, 10.0)
        values = [float(v) * scale if v else 0.0 for v in samples]
    elif isinstance(raw, list):
        if len(raw) != width * height:
            raise DepthError("depth: длина массива не совпадает с width × height")
        units = str(payload.get("units") or "m").lower()
        scale = {"mm": 0.001, "m": 1.0, "cm": 0.01}.get(units)
        if scale is None:
            raise DepthError("units: допустимо mm, cm или m")
        values = []
        for item in raw:
            try:
                value = float(item) * scale
            except (TypeError, ValueError) as exc:
                raise DepthError("depth: элемент не является числом") from exc
            values.append(value if math.isfinite(value) and value > 0 else 0.0)
    else:
        raise DepthError("depth: ожидается base64-строка uint16 или массив чисел")
    stamp = payload.get("stamp")
    try:
        stamp_value = float(stamp) if stamp is not None else float(clock())
    except (TypeError, ValueError):
        stamp_value = float(clock())
    fx = _number(payload.get("fx") or 0.0, "fx", 0, 100000)
    fy = _number(payload.get("fy") or 0.0, "fy", 0, 100000)
    return DepthFrame(width=width, height=height, values=values,
                      stamp=stamp_value, fx=max(0.0, fx), fy=max(0.0, fy))


def _intrinsics(frame: DepthFrame, spec: dict[str, Any]) -> tuple[float, float, float, float]:
    fx = frame.fx or frame.width / (2.0 * math.tan(math.radians(spec["hfov_deg"]) / 2.0))
    fy = frame.fy or frame.height / (2.0 * math.tan(math.radians(spec["vfov_deg"]) / 2.0))
    return fx, fy, (frame.width - 1) / 2.0, (frame.height - 1) / 2.0


def depth_to_scan(frame: DepthFrame, spec: Optional[dict[str, Any]] = None, *,
                  columns: Optional[int] = None) -> list[dict[str, float]]:
    """Преобразовать кадр глубины в полярные точки в системе робота.

    На каждый угловой сектор берётся ближайшее препятствие подходящей высоты:
    так стена не «пробивается» дальними точками пола за ней. Углы и монтаж
    Kinect (`yaw_deg`, `pitch_deg`, `x_m/y_m/z_m`) учитываются здесь; поза
    робота добавляется позже, в `SlamMapService.ingest_kinect`.
    """
    cfg = dict(spec or kinect_spec())
    bins = max(8, min(MAX_POINTS, int(columns or cfg["scan_columns"])))
    fx, fy, cx, cy = _intrinsics(frame, cfg)
    if fx <= 0 or fy <= 0:
        return []
    hfov = math.radians(float(cfg["hfov_deg"]))
    yaw = math.radians(float(cfg["yaw_deg"]))
    pitch = math.radians(float(cfg["pitch_deg"]))
    cos_p, sin_p = math.cos(pitch), math.sin(pitch)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    min_r = float(cfg["min_range_m"])
    max_r = float(cfg["max_range_m"])
    min_h = float(cfg["min_height_m"])
    max_h = float(cfg["max_height_m"])
    mount_z = float(cfg["z_m"])
    nearest: dict[int, float] = {}
    values = frame.values
    width = frame.width
    for v in range(frame.height):
        row = v * width
        for u in range(width):
            z = float(values[row + u])
            if not math.isfinite(z) or z < min_r or z > max_r:
                continue
            # Камера: X вправо, Y вверх, Z вперёд.
            x_cam = (u - cx) / fx * z
            y_cam = -(v - cy) / fy * z
            # Наклон вниз (pitch > 0) поворачивает луч к полу.
            y_up = y_cam * cos_p - z * sin_p
            z_fwd = y_cam * sin_p + z * cos_p
            height = y_up + mount_z
            if not min_h <= height <= max_h:
                continue
            # Робот: X вперёд, Y влево. Поворот Kinect вокруг вертикали.
            x_r, y_r = z_fwd, -x_cam
            x = x_r * cos_y - y_r * sin_y
            y = x_r * sin_y + y_r * cos_y
            distance = math.hypot(x, y)
            if not min_r <= distance <= max_r:
                continue
            angle = normalize_angle(math.atan2(y, x) - yaw)
            if abs(angle) > hfov / 2.0:
                continue
            index = int((angle + hfov / 2.0) / hfov * bins)
            index = max(0, min(bins - 1, index))
            if index not in nearest or distance < nearest[index]:
                nearest[index] = distance
    points: list[dict[str, float]] = []
    for index in sorted(nearest):
        angle = normalize_angle(yaw - hfov / 2.0 + (index + 0.5) * hfov / bins)
        points.append({"a": round(angle, 4), "r": round(nearest[index], 3)})
    return points


def points_from_payload(payload: dict[str, Any], *, clock=time.time,
                        spec: Optional[dict[str, Any]] = None) -> tuple[list[dict[str, float]], float, str]:
    """Вернуть (точки, метка времени, источник) для любого формата сообщения."""
    if not isinstance(payload, dict):
        raise DepthError("ожидается JSON-объект")
    stamp = payload.get("stamp")
    try:
        stamp_value = float(stamp) if stamp is not None else float(clock())
    except (TypeError, ValueError):
        stamp_value = float(clock())
    if isinstance(payload.get("ranges"), list):
        angle_min = _number(payload.get("angle_min", -math.pi / 4), "angle_min", -math.pi * 2, math.pi * 2)
        angle_max = _number(payload.get("angle_max", math.pi / 4), "angle_max", -math.pi * 2, math.pi * 2)
        if angle_max <= angle_min:
            raise DepthError("angle_max должен быть больше angle_min")
        increment = (angle_max - angle_min) / max(1, len(payload["ranges"]) - 1)
        cfg = spec or kinect_spec()
        points = scan_from_ranges([float(v) for v in payload["ranges"]], angle_min, angle_max,
                                  increment, range_min=float(cfg["min_range_m"]),
                                  range_max=float(cfg["max_range_m"]))[:MAX_POINTS]
        return points, stamp_value, "ranges"
    if isinstance(payload.get("points"), list):
        points: list[dict[str, float]] = []
        for item in payload["points"][:MAX_POINTS]:
            if not isinstance(item, dict):
                continue
            try:
                angle = normalize_angle(float(item["a"]))
                distance = float(item["r"])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(angle) and math.isfinite(distance) and 0.05 <= distance <= 12.0:
                points.append({"a": round(angle, 4), "r": round(distance, 3)})
        return points, stamp_value, "points"
    frame = decode_depth_payload(payload, clock=clock)
    return depth_to_scan(frame, spec), frame.stamp, "depth"


def depth_preview(frame: DepthFrame, spec: dict[str, Any]) -> dict[str, Any]:
    """Один уменьшенный кадр в координатах камеры, не накопленная 3D-карта."""
    stride = max(1, math.ceil(frame.width / PREVIEW_WIDTH),
                 math.ceil(frame.height / PREVIEW_HEIGHT))
    fx, fy, cx, cy = _intrinsics(frame, spec)
    values = array.array("H")
    count = 0
    minimum, maximum = float(spec["min_range_m"]), float(spec["max_range_m"])
    for row in range(0, frame.height, stride):
        for col in range(0, frame.width, stride):
            z = frame.values[row * frame.width + col]
            mm = round(z * 1000) if math.isfinite(z) and minimum <= z <= maximum else 0
            values.append(min(65535, max(0, mm)))
            count += bool(mm)
    if sys.byteorder != "little":
        values.byteswap()
    return {
        "width": math.ceil(frame.width / stride), "height": math.ceil(frame.height / stride),
        "stride": stride, "data": base64.b64encode(values.tobytes()).decode("ascii"),
        "fx": fx, "fy": fy, "cx": cx, "cy": cy, "points": count,
        "sourceWidth": frame.width, "sourceHeight": frame.height,
    }


class KinectDepthService:
    """Последний кадр Kinect и его псевдоскан, потокобезопасно."""

    def __init__(self, *, spec: Optional[dict[str, Any]] = None, clock=time.time):
        self._spec = dict(spec or kinect_spec())
        self._clock = clock
        self._lock = threading.RLock()
        self._points: list[dict[str, float]] = []
        self._stamp = 0.0
        # Свежесть считаем по времени ПРИХОДА кадра, а не по метке драйвера:
        # у ROS 2 / freenect часы могут отличаться от часов приложения.
        self._received_at = 0.0
        self._source = ""
        self._capture_source = "unknown"
        self._capture_port = ""
        self._frames = 0
        self._last_width = 0
        self._last_height = 0
        self._error = ""
        self._consumed_stamp = 0.0
        self._preview: Optional[dict[str, Any]] = None

    @property
    def spec(self) -> dict[str, Any]:
        return dict(self._spec)

    def feed(self, payload: Any) -> dict[str, Any]:
        """Принять кадр; исключение DepthError пробрасывается в HTTP как 400."""
        preview = None
        if isinstance(payload, dict) and not isinstance(payload.get("ranges"), list) and not isinstance(payload.get("points"), list):
            frame = decode_depth_payload(payload, clock=self._clock)
            points, stamp, source = depth_to_scan(frame, self._spec), frame.stamp, "depth"
            preview = depth_preview(frame, self._spec)
        else:
            points, stamp, source = points_from_payload(payload, clock=self._clock, spec=self._spec)
        with self._lock:
            self._points = points
            self._preview = preview
            self._stamp = stamp
            self._received_at = self._clock()
            self._source = source
            capture_source = payload.get("capture_source")
            self._capture_source = (capture_source if isinstance(capture_source, str) and
                                    capture_source in {"demo", "freenect", "openni", "ros2"}
                                    else "unknown")
            capture_port = payload.get("capture_port")
            self._capture_port = (capture_port if isinstance(capture_port, str) and re.fullmatch(
                r"usb:\d+-\d+(?:\.\d+)*", capture_port) else "")
            self._frames += 1
            self._error = ""
            self._last_width = int(payload.get("width") or 0)
            self._last_height = int(payload.get("height") or 0)
        return {"ok": True, "points": len(points), "stamp": stamp, "source": source}

    def consume(self) -> Optional[tuple[list[dict[str, float]], float]]:
        """Отдать кадр в карту ровно один раз (по метке времени)."""
        with self._lock:
            if not self._points or self._stamp == self._consumed_stamp:
                return None
            self._consumed_stamp = self._stamp
            return list(self._points), self._stamp

    def fresh(self, max_age_s: Optional[float] = None) -> bool:
        limit = float(max_age_s if max_age_s is not None else self._spec["max_age_s"])
        with self._lock:
            return bool(self._points) and (self._clock() - self._received_at) <= limit

    def status(self) -> dict[str, Any]:
        with self._lock:
            age = self._clock() - self._received_at if self._received_at else None
            return {
                "ok": True,
                "enabled": True,
                "connected": bool(self._points) and age is not None
                and age <= float(self._spec["max_age_s"]),
                "frames": self._frames,
                "points": len(self._points),
                "lastStamp": self._stamp,
                "ageS": round(age, 3) if age is not None else None,
                "source": self._source,
                "captureSource": self._capture_source,
                "capturePort": self._capture_port,
                "width": self._last_width,
                "height": self._last_height,
                "error": self._error,
                "spec": dict(self._spec),
                "drivers": available_backends(),
            }

    def preview(self) -> dict[str, Any]:
        """Отдать только свежий кадр глубины; 2D-скан не превращается в 3D."""
        with self._lock:
            age = self._clock() - self._received_at if self._received_at else None
            fresh = age is not None and 0 <= age <= float(self._spec["max_age_s"])
            available = bool(fresh and self._preview and self._preview["points"])
            result = {"ok": True, "available": available, "oneFrameOnly": True,
                      "notForNavigation": True, "frameId": self._frames,
                      "ageS": round(age, 3) if age is not None else None,
                      "maxAgeS": float(self._spec["max_age_s"]), "source": self._source,
                      "captureSource": self._capture_source,
                      "capturePort": self._capture_port,
                      "mount": {key: float(self._spec[key]) for key in
                                ("z_m", "yaw_deg", "pitch_deg")}}
            if available:
                result.update(self._preview)
            return result

    def reset(self) -> None:
        with self._lock:
            self._points = []
            self._preview = None
            self._capture_source = "unknown"
            self._capture_port = ""
            self._stamp = 0.0
            self._received_at = 0.0
            self._frames = 0
            self._error = ""
            self._consumed_stamp = 0.0
