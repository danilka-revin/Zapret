"""Автономный проезд трассы с конусами («отдельная вкладка» бортового ПК).

Что делает сервис
-----------------
* едет только через подтверждённый проезд между столбиками: каждый столбик
  запрещён для всего корпуса, лидар независимо проверяет путь торможения;
* у знака «СТОП» снимает тягу, ждёт подтверждения тормоза на всех четырёх
  колёсах и только после этого отсчитывает минимум 3 с;
* знак «неровность» + Kinect: подтверждаем кочку по кадру глубины, заранее
  снижаем скорость, на подъёме по гироскопу/одометрии добавляем тягу порциями,
  пока робот не переедет, после кочки снова держим малую скорость и только
  потом возвращаемся к обычной;
* красный, жёлтый и неясный сигнал — STOP; разрешение только по свежим
  отдельным кадрам зелёного. Повторный красный не скрывается таймером;
* потеря конусов, кадров, глубины, E-STOP, кнопка «Стоп», тайм-аут и предел
  дистанции — всегда STOP.

Это НЕ nav2 и не сертифицированная система безопасности. Тяга включается
только при ``RC_TRACK_ENABLE=1``, свежих камерах, подтверждении осмотра зоны
оператором, снятом E-STOP и готовом шасси. Все команды идут через уже
существующие ``slam.dispatch_drive_command`` / ``dispatch_steering_command``
(защёлка E-STOP, лимиты углов, watchdog прошивки сохраняются).

Факты для решений берутся из ``robot_control.vision`` (публичный контракт
``/api/vision/state``): ``cameras[i].detections[]``, ``field[]`` и
``trafficLight``. Новые классы ``traffic_light_red/yellow/green`` сервис
сводит к ``traffic_light`` + ``signal``, поэтому модуль одинаково работает
со старыми и с дообученными весами. Модуль не открывает USB, не обучает
модель и не подменяет машинное зрение.
"""
from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass, replace
from types import SimpleNamespace
from typing import Any, Callable, Optional

from . import estop as estop_latch
from . import eventlog
from .ground_hazards import GroundHazardTracker, analyze_ground_points, ground_points_from_depth
from .navigation import _scan_coverage
from .slam_map import lidar_mount_offset
from .track_passage import PASSAGE_REASONS, estimate_corridor, passage_fits, post_in_path, swept_hit
from .vision.classes import CLASS_INFO

#: Маркер команд вкладки (как ``exploration``, ``delivery``, ``kinect``).
AUTONOMY_HEADER = "autonomy"
MODULES = ("FL", "FR", "RL", "RR")
FRONT_CAMERAS = (0, 2)
TRACK_SCENARIOS = ("full", "straight", "stop", "bump", "light", "gate", "blocked_gate")
WHEEL_TELEMETRY_MAX_AGE_S = 0.75
BRAKE_TIMEOUT_S = 5.0
STEER_TOLERANCE_DEG = 2.0
MAX_SC_EDGES_PER_S = 6700  # 150 мкс между фронтами в прошивке
WHEEL_CIRCUMFERENCE_M = math.pi * 0.2159  # AOVO Pro 8.5", как в maneuvers.py


# --------------------------------------------------------------------- настройки
def _env_bool(name: str, default: bool) -> bool:
    raw = (os.environ.get(name) or "").strip().lower()
    if not raw:
        return default
    return raw in {"1", "true", "yes", "on", "да"}


def _env_float(name: str, default: float, lo: float, hi: float) -> float:
    try:
        value = float((os.environ.get(name) or "").strip() or default)
    except (TypeError, ValueError):
        return default
    return max(lo, min(hi, value)) if math.isfinite(value) else default


def _env_int(name: str, default: int, lo: int, hi: int) -> int:
    try:
        value = int((os.environ.get(name) or "").strip() or default)
    except (TypeError, ValueError):
        return default
    return max(lo, min(hi, value))


@dataclass(frozen=True)
class TrackConfig:
    """Все пороги трассы в одном месте; лимиты не дают выставить опасное."""

    enabled: bool = False                 # RC_TRACK_ENABLE: явный пуск в .env
    simulation: bool = False              # смотреть камеры, но НЕ трогать тягу
    tick_s: float = 0.20                  # < 650 мс watchdog прошивки
    stale_s: float = 1.5                  # свежесть кадров/знаков
    cruise_percent: float = 8.0           # обычная скорость, % шкалы Servo
    slow_percent: float = 4.0             # перед кочкой/знаком и после кочки
    climb_percent: float = 20.0           # потолок добавки тяги на кочке
    climb_step_percent: float = 3.0       # шаг «добавляй, пока не переедет»
    post_bump_s: float = 0.8              # «потом снова замедли»
    max_steer_deg: float = 22.0           # крабовый ход, не разворот на месте
    steer_deadband_deg: float = 2.5       # руль в движении не перекладываем
    steer_settle_s: float = 0.45          # ждём, пока колёса дойдут до угла
    steer_gain: float = 1.0               # поправка угла краба к центру проезда
    gyro_damping: float = 0.02            # гироскоп гасит рыскание на кочке
    lookahead_m: float = 2.5              # точка прицеливания по коридору
    min_cones: int = 2                    # минимум конусов, чтобы ехать
    min_confidence: float = 0.35          # порог уверенности детектора
    max_lateral_m: float = 4.0
    min_corridor_m: float = 0.9           # уже корпуса + запас → STOP
    max_corridor_m: float = 3.5
    max_offset_m: float = 1.5             # «не сильно далеко от конусов»
    max_nearest_m: float = 4.5            # ближайший конус слишком далеко
    lane_lost_s: float = 1.5              # сколько ждём восстановления проезда без тяги
    stop_wait_s: float = 3.0              # не меньше 3 с после полной остановки
    stop_offset_m: float = 0.45           # продольный допуск «возле знака»
    stop_side_m: float = 1.8              # знак в боковой камере на таком расстоянии
    stop_pass_s: float = 1.2              # знак пропал из передних камер
    sign_cooldown_m: float = 3.0          # защита от повторного срабатывания
    bump_slow_m: float = 1.6              # начать тормозить перед кочкой
    bump_cross_m: float = 0.9             # сколько считать «переездом»
    bump_sign_m: float = 3.0              # дальность знака неровности
    bump_cross_timeout_s: float = 12.0     # предел времени подтверждения/переезда
    stall_s: float = 0.8                  # нет движения → не хватает мощности
    pitch_deg: float = 6.0                # нос задрался — уперлись в кочку
    traffic_lost_s: float = 3.0           # светофор пропал во время ожидания
    max_distance_m: float = 200.0         # жёсткий предохранитель (50 м и более)
    max_duration_s: float = 900.0
    finish_distance_m: float = 0.0        # измеренный финиш, 0 = выключено
    cone_height_m: float = CLASS_INFO["cone"]["height_m"]
    post_radius_m: float = 0.125          # радиус самого широкого основания
    robot_length_m: float = 0.75
    robot_width_m: float = 0.60
    clearance_m: float = 0.12            # запас от края корпуса
    stop_clearance_m: float = 0.8        # проверяемый путь торможения, не измеренный результат
    lidar_scan_age_s: float = 0.75
    max_speed_mps: float = 0.35          # отсечка по SC, не уставка регулятора
    bump_max_height_m: float = 0.12
    require_depth: bool = True
    require_imu: bool = False
    require_blind_check: bool = True
    side_camera_deg: float = 60.0         # |yaw| ≥ 60° → «камера с краю»
    max_field_m: float = 6.0

    @classmethod
    def from_env(cls) -> "TrackConfig":
        return cls(
            enabled=_env_bool("RC_TRACK_ENABLE", False),
            simulation=_env_bool("RC_TRACK_SIMULATION", False),
            tick_s=_env_float("RC_TRACK_TICK_S", 0.20, 0.05, 0.30),
            stale_s=_env_float("RC_TRACK_STALE_S", 1.5, 0.3, 2.0),
            cruise_percent=_env_float("RC_TRACK_CRUISE_PERCENT", 8.0, 1.0, 30.0),
            slow_percent=_env_float("RC_TRACK_SLOW_PERCENT", 4.0, 1.0, 30.0),
            climb_percent=_env_float("RC_TRACK_CLIMB_PERCENT", 20.0, 1.0, 30.0),
            climb_step_percent=_env_float("RC_TRACK_CLIMB_STEP_PERCENT", 3.0, 0.5, 20.0),
            post_bump_s=_env_float("RC_TRACK_POST_BUMP_S", 0.8, 0.0, 10.0),
            max_steer_deg=_env_float("RC_TRACK_MAX_STEER_DEG", 22.0, 1.0, 28.0),
            steer_deadband_deg=_env_float("RC_TRACK_STEER_DEADBAND_DEG", 2.5, 0.5, 15.0),
            steer_settle_s=_env_float("RC_TRACK_STEER_SETTLE_S", 0.45, 0.0, 3.0),
            steer_gain=_env_float("RC_TRACK_STEER_GAIN", 1.0, 0.2, 5.0),
            gyro_damping=_env_float("RC_TRACK_GYRO_DAMPING", 0.02, 0.0, 0.3),
            lookahead_m=_env_float("RC_TRACK_LOOKAHEAD_M", 2.5, 0.5, 8.0),
            min_cones=_env_int("RC_TRACK_MIN_CONES", 2, 1, 12),
            min_confidence=_env_float("RC_TRACK_MIN_CONFIDENCE", 0.35, 0.05, 0.95),
            max_lateral_m=_env_float("RC_TRACK_MAX_LATERAL_M", 4.0, 0.5, 12.0),
            min_corridor_m=_env_float("RC_TRACK_MIN_CORRIDOR_M", 0.9, 0.4, 3.0),
            max_corridor_m=_env_float("RC_TRACK_MAX_CORRIDOR_M", 3.5, 1.0, 10.0),
            max_offset_m=_env_float("RC_TRACK_MAX_OFFSET_M", 1.5, 0.3, 5.0),
            max_nearest_m=_env_float("RC_TRACK_MAX_NEAREST_M", 4.5, 0.5, 12.0),
            lane_lost_s=_env_float("RC_TRACK_LANE_LOST_S", 1.5, 0.2, 10.0),
            stop_wait_s=_env_float("RC_TRACK_STOP_WAIT_S", 3.0, 3.0, 30.0),
            stop_offset_m=_env_float("RC_TRACK_STOP_OFFSET_M", 0.45, 0.0, 3.0),
            stop_side_m=_env_float("RC_TRACK_STOP_SIDE_M", 1.8, 0.2, 6.0),
            stop_pass_s=_env_float("RC_TRACK_STOP_PASS_S", 1.2, 0.2, 6.0),
            sign_cooldown_m=_env_float("RC_TRACK_SIGN_COOLDOWN_M", 3.0, 0.0, 20.0),
            bump_slow_m=_env_float("RC_TRACK_BUMP_SLOW_M", 1.6, 0.2, 8.0),
            bump_cross_m=_env_float("RC_TRACK_BUMP_CROSS_M", 0.9, 0.2, 5.0),
            bump_sign_m=_env_float("RC_TRACK_BUMP_SIGN_M", 3.0, 0.5, 10.0),
            bump_cross_timeout_s=_env_float("RC_TRACK_BUMP_CROSS_TIMEOUT_S", 12.0, 1.0, 60.0),
            stall_s=_env_float("RC_TRACK_STALL_S", 0.8, 0.2, 5.0),
            pitch_deg=_env_float("RC_TRACK_PITCH_DEG", 6.0, 0.5, 45.0),
            traffic_lost_s=_env_float("RC_TRACK_TRAFFIC_LOST_S", 3.0, 0.5, 20.0),
            max_distance_m=_env_float("RC_TRACK_MAX_DISTANCE_M", 200.0, 10.0, 5000.0),
            max_duration_s=_env_float("RC_TRACK_MAX_DURATION_S", 900.0, 30.0, 7200.0),
            finish_distance_m=_env_float("RC_TRACK_FINISH_DISTANCE_M", 0.0, 0.0, 5000.0),
            cone_height_m=_env_float("RC_TRACK_POST_HEIGHT_M", CLASS_INFO["cone"]["height_m"], 0.1, 2.0),
            post_radius_m=_env_float("RC_TRACK_POST_RADIUS_M", 0.125, 0.02, 0.5),
            robot_length_m=_env_float("RC_ROBOT_LENGTH_M", 0.75, 0.3, 2.5),
            robot_width_m=_env_float("RC_ROBOT_WIDTH_M", 0.60, 0.25, 2.0),
            clearance_m=_env_float("RC_TRACK_CLEARANCE_M", 0.12, 0.08, 1.0),
            stop_clearance_m=_env_float("RC_TRACK_STOP_CLEARANCE_M", 0.8, 0.5, 4.0),
            lidar_scan_age_s=_env_float("RC_TRACK_LIDAR_MAX_AGE_S", 0.75, 0.3, 1.0),
            max_speed_mps=_env_float("RC_TRACK_MAX_SPEED_MPS", 0.35, 0.05, 0.6),
            bump_max_height_m=_env_float("RC_TRACK_BUMP_MAX_HEIGHT_M", 0.12, 0.04, 0.2),
            require_depth=_env_bool("RC_TRACK_REQUIRE_DEPTH", True),
            require_imu=_env_bool("RC_TRACK_REQUIRE_IMU", False),
            require_blind_check=_env_bool("RC_TRACK_REQUIRE_BLIND_CHECK", True),
            side_camera_deg=_env_float("RC_TRACK_SIDE_CAMERA_DEG", 60.0, 10.0, 170.0),
            max_field_m=_env_float("RC_TRACK_MAX_FIELD_M", 6.0, 1.0, 30.0),
        )

    def public(self) -> dict[str, Any]:
        """Настройки для экрана (все ключи — объяснимые числа)."""
        return {key: getattr(self, key) for key in (
            "enabled", "simulation", "tick_s", "stale_s", "cruise_percent",
            "slow_percent", "climb_percent", "climb_step_percent", "post_bump_s",
            "max_steer_deg", "lookahead_m", "min_cones", "min_corridor_m",
            "max_corridor_m", "max_offset_m", "lane_lost_s", "stop_wait_s",
            "stop_offset_m", "stop_side_m", "bump_slow_m", "bump_cross_m",
            "stall_s", "pitch_deg", "max_distance_m", "max_duration_s", "finish_distance_m",
            "robot_length_m", "robot_width_m", "clearance_m", "post_radius_m",
            "stop_clearance_m", "max_speed_mps", "bump_max_height_m", "require_depth",
            "require_imu", "require_blind_check", "side_camera_deg")}


class TrackAutonomyError(ValueError):
    def __init__(self, code: str, message: str, status: int = 409):
        super().__init__(message)
        self.code = code
        self.message = message
        self.status = status

    def payload(self) -> dict[str, Any]:
        return {"ok": False, "code": self.code, "error": self.message}


# ------------------------------------------------------------------ восприятие
def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def camera_specs(layout: dict[str, Any]) -> dict[int, dict[str, float]]:
    """id камеры → геометрия монтажа из ``/api/sensors/layout``."""
    specs: dict[int, dict[str, float]] = {}
    for cam in (layout or {}).get("cameras") or []:
        try:
            cid = int(cam["id"])
        except (KeyError, TypeError, ValueError):
            continue
        try:
            specs[cid] = {
                "yaw_deg": float(cam.get("yaw_deg", 0.0)),
                "hfov_deg": float(cam.get("hfov_deg", 82.0)),
                "x_m": float(cam.get("x_m", 0.30)),
                "y_m": float(cam.get("y_m", 0.0)),
            }
        except (TypeError, ValueError):
            continue
    return specs


def _number(value: Any) -> float | None:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return None
    return float(value) if math.isfinite(value) else None


def _camera_fresh(vision: dict[str, Any], cid: int, cfg: TrackConfig) -> dict[str, Any] | None:
    cam = ((vision.get("cameras") or {}).get(str(cid)) or {})
    age = _number(cam.get("captureAgeS", cam.get("ageS")))
    width, height = _number(cam.get("width")), _number(cam.get("height"))
    if cam.get("error") or not width or not height or age is None or not 0 <= age <= cfg.stale_s:
        return None
    return cam


def _vision_readiness(vision: dict[str, Any], cfg: TrackConfig) -> tuple[int, bool]:
    fresh = {cid for cid in range(4) if _camera_fresh(vision, cid, cfg) is not None}
    return len(fresh), set(FRONT_CAMERAS) <= fresh


def cone_points(vision: dict[str, Any], layout: dict[str, Any],
                cfg: TrackConfig) -> list[dict[str, Any]]:
    """Рамки конусов → точки (X вперёд, Y влево) в системе робота.

    Дальность оцениваем по высоте рамки и физической высоте конуса — так же,
    как ``VisionService._build_field`` считает знаки. Конусы в ``field``
    намеренно не публикуются, поэтому читаем именно detections.
    """
    specs = camera_specs(layout)
    height = float(cfg.cone_height_m)
    cones: list[dict[str, Any]] = []
    for cid in range(4):
        cam = _camera_fresh(vision, cid, cfg)
        if cam is None:
            continue
        spec = specs.get(cid, {"yaw_deg": 0.0, "hfov_deg": 82.0, "x_m": 0.30, "y_m": 0.0})
        hfov = math.radians(max(10.0, float(spec["hfov_deg"])))
        yaw = math.radians(float(spec["yaw_deg"]))
        focal_px = (float(cam["width"]) / 2.0) / math.tan(hfov / 2.0)
        frame_h = max(1.0, float(cam.get("height") or 360))
        for det in cam.get("detections") or []:
            if det.get("class") != "cone" or (_number(det.get("confidence")) or 0) < cfg.min_confidence:
                continue
            box = det.get("box") or {}
            try:
                bw, bh = float(box["w"]), float(box["h"])
                cx = float(box["x"]) + bw / 2.0
            except (KeyError, TypeError, ValueError):
                continue
            if not (0 < bw <= 1 and 0 < bh <= 1 and 0 <= cx <= 1):
                continue
            distance = _clamp(height * focal_px / max(1.0, bh * frame_h), 0.3, cfg.max_field_m)
            bearing = yaw + (0.5 - cx) * hfov
            cones.append({
                "x": round(float(spec["x_m"]) + distance * math.cos(bearing), 3),
                "y": round(float(spec["y_m"]) + distance * math.sin(bearing), 3),
                "distance": round(distance, 3),
                "cameraId": cid,
                "confidence": round(float(det.get("confidence") or 0), 3),
            })
    return cones


def sign_marks(vision: dict[str, Any], layout: dict[str, Any],
               cfg: TrackConfig) -> list[dict[str, Any]]:
    """Знаки/светофоры из ``field`` (или напрямую из рамок, если field пуст).

    ``side`` = «передняя» или «краевая» камера — по |yaw|. Именно появление
    СТОП/неровности/светофора в краевой камере означает «робот возле знака».
    """
    specs = camera_specs(layout)
    wanted = {"stop_sign", "speed_bump", "traffic_light"}
    marks: list[dict[str, Any]] = []
    for item in vision.get("field") or []:
        if item.get("class") not in wanted:
            continue
        age = _number(item.get("ageS"))
        if age is None or not 0 <= age <= cfg.stale_s:
            continue
        if float(item.get("confidence") or 0) < cfg.min_confidence:
            continue
        try:
            cid = int(item.get("cameraId"))
            x = float(item["x_m"])
            y = float(item["y_m"])
            distance = float(item["distanceM"])
        except (KeyError, TypeError, ValueError):
            continue
        if _camera_fresh(vision, cid, cfg) is None or not all(map(math.isfinite, (x, y, distance))):
            continue
        yaw = float(specs.get(cid, {}).get("yaw_deg", 0.0))
        marks.append({
            "class": item["class"], "x": round(x, 3), "y": round(y, 3),
            "distance": round(distance, 3), "cameraId": cid,
            "side": "side" if abs(yaw) >= cfg.side_camera_deg else "front",
            "yawDeg": yaw, "signal": item.get("signal"),
            "confidence": round(float(item.get("confidence") or 0), 3),
        })
    represented = {(mark["cameraId"], mark["class"]) for mark in marks}
    # Резерв по каждой камере и классу, а не только при полностью пустом field.
    for cid in range(4):
        cam = _camera_fresh(vision, cid, cfg)
        if cam is None:
            continue
        spec = specs.get(cid, {"yaw_deg": 0.0, "hfov_deg": 82.0, "x_m": 0.30, "y_m": 0.0})
        hfov = math.radians(max(10.0, float(spec["hfov_deg"])))
        yaw_deg = float(spec["yaw_deg"])
        yaw = math.radians(yaw_deg)
        focal_px = (float(cam["width"]) / 2.0) / math.tan(hfov / 2.0)
        frame_h = max(1.0, float(cam.get("height") or 360))
        for det in cam.get("detections") or []:
            name = det.get("class")
            if name not in wanted or (cid, name) in represented:
                continue
            if float(det.get("confidence") or 0) < cfg.min_confidence:
                continue
            box = det.get("box") or {}
            try:
                bw, bh = float(box["w"]), float(box["h"])
                cx = float(box["x"]) + bw / 2.0
            except (KeyError, TypeError, ValueError):
                continue
            if bh <= 0:
                continue
            if not (0 < bw <= 1 and 0 < bh <= 1 and 0 <= cx <= 1):
                continue
            distance = _clamp(float(CLASS_INFO[name]["height_m"]) * focal_px /
                              max(1.0, bh * frame_h), 0.3, cfg.max_field_m)
            bearing = yaw + (0.5 - cx) * hfov
            marks.append({
                "class": name,
                "x": round(float(spec["x_m"]) + distance * math.cos(bearing), 3),
                "y": round(float(spec["y_m"]) + distance * math.sin(bearing), 3),
                "distance": round(distance, 3), "cameraId": cid,
                "side": "side" if abs(yaw_deg) >= cfg.side_camera_deg else "front",
                "yawDeg": yaw_deg, "signal": det.get("signal"),
                "confidence": round(float(det.get("confidence") or 0), 3),
            })
    return marks


def traffic_decision(vision: dict[str, Any], marks: list[dict[str, Any]],
                     cfg: TrackConfig) -> dict[str, Any]:
    """Use current camera evidence as well as the smoothed display signal.

    In particular a remembered green, or the closed-course unknown advisory,
    cannot grant traction. Fresh red (including red pixels) wins immediately.
    """
    tracked = vision.get("trafficLight") or {}
    age = _number(tracked.get("ageS"))
    fresh = tracked.get("fresh") is True and age is not None and 0 <= age <= cfg.stale_s
    state = tracked.get("state") if fresh else "unknown"
    lights, frames = [], {}
    for cid in range(4):
        cam = _camera_fresh(vision, cid, cfg)
        if cam:
            stamp = _number(cam.get("frameStamp"))
            if stamp is not None and stamp > 0:
                frames[cid] = stamp
            lights.extend((cid, stamp, det)
                          for det in cam.get("detections") or []
                          if det.get("class") == "traffic_light" and
                          (_number(det.get("confidence")) or 0) >= cfg.min_confidence)
    light_marks = [m for m in marks if m["class"] == "traffic_light"]
    visible = bool(lights or light_marks)
    counts = {color: sum(int((det.get("signalCounts") or {}).get(color) or 0) for _, _, det in lights)
              for color in ("red", "yellow", "green")}
    signals = [det.get("signal") for _, _, det in lights] + [m.get("signal") for m in light_marks]
    green = None
    if "red" in signals or counts["red"] or state == "red":
        state = "red"
    elif "yellow" in signals or state == "yellow":
        state = "yellow"
    elif state == "green" and lights and all(signal == "green" for signal in signals):
        # Prefer a fixed camera while it sees the head, not whichever had
        # a marginally higher softmax score on the latest loop iteration.
        green = next(((cid, stamp) for cid, stamp, det in lights
                      if stamp is not None and stamp > 0 and (det.get("signalCounts") or {}).get("green", 0) > 0), None)
        if green is None:
            state = "unknown"
    else:
        state = "unknown"
    return {"visible": visible, "state": state if visible else "none",
            "fresh": bool(lights), "hold": visible and green is None,
            "confidence": tracked.get("confidence"), "counts": counts, "frames": frames,
            "cameraId": green[0] if green else None, "frameStamp": green[1] if green else None}


def lidar_clearance(info: dict[str, Any], angle_deg: float, cfg: TrackConfig) -> dict[str, Any]:
    """Independent scan-only veto along the protected braking envelope."""
    points = info.get("points") if info.get("trusted") is True else None
    ready, count, _ = _scan_coverage(points)
    if not ready:
        return {"ready": False, "blocked": False, "points": count, "nearestM": None}
    offset = lidar_mount_offset()
    nearest = None
    for point in points:
        try:
            r, a = float(point["r"]), float(point["a"])
        except (KeyError, TypeError, ValueError):
            continue
        if not (math.isfinite(r) and math.isfinite(a) and r > 0):
            continue
        x, y = offset + r * math.cos(a), r * math.sin(a)
        if swept_hit(x, y, angle_deg, cfg.stop_clearance_m,
                     cfg.robot_length_m / 2 + cfg.clearance_m,
                     cfg.robot_width_m / 2 + cfg.clearance_m):
            nearest = r if nearest is None else min(nearest, r)
    return {"ready": True, "blocked": nearest is not None, "points": count,
            "nearestM": round(nearest, 2) if nearest is not None else None}


_EMPTY_GROUND: dict[str, Any] = {"ok": False, "depthFresh": False, "nearestBumpM": None,
                                 "status": "unknown", "reason": "no_depth", "hazards": []}


def _downsample_depth_frame(frame: Any, spec: dict[str, Any]) -> Any:
    """Полный raw-depth кадр → быстрый навигационный кадр ≤ 120×90.

    Важно: это НЕ ``KinectDepthService.preview()`` с ``notForNavigation``.
    Берётся исходный кадр того же frameId и прореживается здесь, внутри
    владельца тяги; оптика делится на stride, чтобы XYZ остались в метрах.
    Ограничение размера держит шаг цикла короче watchdog 650 мс.
    """
    try:
        width, height = int(frame.width), int(frame.height)
        values = frame.values
        if width < 2 or height < 2 or len(values) != width * height:
            return None
        stride = max(1, math.ceil(width / 120.0), math.ceil(height / 90.0))
        fx = float(getattr(frame, "fx", 0) or 0)
        fy = float(getattr(frame, "fy", 0) or 0)
        if fx <= 0:
            fx = width / (2.0 * math.tan(math.radians(float(spec["hfov_deg"])) / 2.0))
        if fy <= 0:
            fy = height / (2.0 * math.tan(math.radians(float(spec["vfov_deg"])) / 2.0))
        sampled = ([float(value) for value in values] if stride == 1 else
                   [float(values[v * width + u])
                    for v in range(0, height, stride)
                    for u in range(0, width, stride)])
        return SimpleNamespace(width=math.ceil(width / stride),
                               height=math.ceil(height / stride),
                               values=sampled, fx=fx / stride, fy=fy / stride,
                               stamp=getattr(frame, "stamp", 0.0))
    except (AttributeError, KeyError, TypeError, ValueError, ZeroDivisionError):
        return None


def kinect_navigation_ready(kinect: Any, cfg: TrackConfig) -> bool:
    """Свежий ПОЛНЫЙ кадр глубины, пригодный для навигации по рельефу.

    ``preview()`` у Kinect помечен ``notForNavigation`` и уменьшен; ``fresh()``
    может быть True и для «points/ranges», где плоского кадра вовсе нет. Поэтому
    безопасный путь — отдельный ``navigation_fresh``/``navigation_frame``.
    """
    if kinect is None:
        return False
    checker = getattr(kinect, "navigation_fresh", None)
    if not callable(checker):
        return False
    try:
        return bool(checker(cfg.stale_s, allow_demo=cfg.simulation))
    except Exception:  # noqa: BLE001 — сбой драйвера = данных нет
        return False


class BumpDetector:
    """Kinect 360 → подтверждённая кочка впереди, м.

    Один полный кадр глубины = один анализ: иначе неизменный кадр за несколько
    тактов набрал бы ``confirm_frames`` и «подтвердил» кочку без измерений.
    Пустой/засвеченный кадр не становится ровной дорогой (``unknown``).
    """

    def __init__(self) -> None:
        self.tracker = GroundHazardTracker()
        self.last_frame_id: Any = None
        self.last_result = dict(_EMPTY_GROUND)

    def reset(self) -> None:
        self.tracker.reset()
        self.last_frame_id = None
        self.last_result = dict(_EMPTY_GROUND)

    def update(self, kinect: Any, imu: dict[str, Any], stamp: float,
               cfg: TrackConfig) -> dict[str, Any]:
        if not kinect_navigation_ready(kinect, cfg):
            return dict(_EMPTY_GROUND, reason="no_navigation_depth")
        try:
            snapshot = kinect.navigation_frame() or {}
            frame = snapshot.get("frame")
            spec = snapshot.get("spec") or kinect.spec
            frame_id = snapshot.get("frameId")
            frame_age = _number(snapshot.get("ageS", 0))
            if (frame_age is None or not 0 <= frame_age <= cfg.stale_s or
                    (not cfg.simulation and snapshot.get("captureSource") == "demo")):
                return dict(_EMPTY_GROUND, reason="stale_navigation_depth")
        except Exception:  # noqa: BLE001 — отсутствие глубины не должно ронять цикл
            return dict(_EMPTY_GROUND, reason="no_navigation_depth")
        if frame is None:
            return dict(_EMPTY_GROUND, reason="no_navigation_depth")
        if frame_id is not None and frame_id == self.last_frame_id:
            return dict(self.last_result)
        fast_frame = _downsample_depth_frame(frame, spec)
        if fast_frame is None:
            return dict(_EMPTY_GROUND, reason="bad_navigation_frame")
        try:
            points = ground_points_from_depth(
                fast_frame, spec,
                body_pitch_deg=float(imu.get("pitch") or 0.0) if imu.get("fresh") else 0.0,
                body_roll_deg=float(imu.get("roll") or 0.0) if imu.get("fresh") else 0.0)
            settings = self.tracker.config
            analysis = analyze_ground_points(points, settings)
            tracked = self.tracker.update(analysis, max(0.001, float(stamp)))
        except Exception as exc:  # noqa: BLE001 — битый кадр = «данных нет»
            return dict(_EMPTY_GROUND, reason=str(exc)[:120])
        hazards = tracked.get("hazards") or []
        bumps = [item for item in hazards if item.get("kind") == "bump"
                 and 0 < float(item.get("severityM") or 0) <= cfg.bump_max_height_m
                 and abs(float(item["center"][1])) <= cfg.robot_width_m / 2 + cfg.clearance_m]
        nearest = min(bumps, key=lambda item: item["bbox"][0], default=None)
        result = {
            "ok": bool(tracked.get("status") == "ok"),
            "depthFresh": tracked.get("validCells", 0) >= settings.min_valid_cells,
            "nearestBumpM": nearest["bbox"][0] if nearest else None,
            "bumpFarM": nearest["bbox"][2] if nearest else None,
            "status": tracked.get("status"), "reason": tracked.get("reason") or "",
            "hazards": hazards, "validPoints": tracked.get("validPoints", 0),
        }
        self.last_frame_id = frame_id
        self.last_result = dict(result)
        return result


def ground_bump_distance(kinect: Any, imu: dict[str, Any], tracker: GroundHazardTracker,
                         stamp: float, cfg: TrackConfig) -> dict[str, Any]:
    """Одноразовый анализ кадра (для тестов); сервис держит ``BumpDetector``."""
    detector = BumpDetector()
    detector.tracker = tracker
    return detector.update(kinect, imu, stamp, cfg)


def imu_snapshot(raw: Any) -> dict[str, Any]:
    state = raw if isinstance(raw, dict) else {}
    fresh = bool(state.get("ok")) and not state.get("stale") and _number(state.get("pitch")) is not None

    def num(key: str) -> float | None:
        value = state.get(key)
        return float(value) if isinstance(value, (int, float)) and math.isfinite(float(value)) else None

    return {"fresh": fresh, "pitch": num("pitch") or 0.0, "roll": num("roll") or 0.0,
            "gyroZ": num("gyroZ") or 0.0, "heading": num("heading"), "ageMs": state.get("ageMs")}


# ---------------------------------------------------------------------- привод
def chassis_feedback(telemetry: dict[str, Any]) -> dict[str, Any]:
    """Confirm the four *distinct* physical wheel boards, not an average RPM."""
    motors = {m.get("id"): m for m in telemetry.get("motors") or []
              if isinstance(m, dict) and m.get("id") in MODULES and m.get("online") is True}
    ready = set(motors) == set(MODULES) and telemetry.get("linkOk") is True
    speed = _number(telemetry.get("wheelSpeedMps"))
    diameter_mm = _number(((telemetry.get("maneuverConfig") or {}).get("chassis") or {}).get("wheelDiameterMm"))
    circumference = math.pi * diameter_mm / 1000 if diameter_mm and diameter_mm > 0 else WHEEL_CIRCUMFERENCE_M
    peak_speed = 0.0
    stopped = ready
    angles, brake_ids, drive_percents, counts = {}, {}, {}, {}
    for name, motor in motors.items():
        age = _number(motor.get("telemetryAgeS"))
        rpm = _number(motor.get("scRpm"))
        firmware = motor.get("firmware")
        brake = motor.get("brakeId")
        count, ppr = motor.get("scCount"), motor.get("scPpr")
        angle = _number(motor.get("angle"))
        if (age is None or not 0 <= age < WHEEL_TELEMETRY_MAX_AGE_S or
                type(firmware) is not int or firmware < 19 or type(brake) is not int or brake < 0 or
                rpm is None or rpm < 0 or type(motor.get("scRecent")) is not bool or
                type(count) is not int or not 0 <= count <= 0xFFFFFFFF or type(ppr) is not int or ppr <= 0 or
                angle is None or motor.get("homed") is not True or motor.get("fullZeroFault") is True or
                motor.get("neutralLocked") is True or motor.get("calibrating") or motor.get("cycling")):
            ready = False
        if rpm is not None:
            peak_speed = max(peak_speed, rpm * circumference / 60)
        if rpm is None or rpm > 1.0 or motor.get("scRecent") is not False or motor.get("drivePercent") != 0:
            stopped = False
        if angle is not None and motor.get("moving") is False:
            angles[name] = angle
        brake_ids[name] = brake
        counts[name] = count
        drive_percents[name] = motor.get("drivePercent")
    return {"ready": ready and speed is not None and speed >= 0, "stopped": ready and stopped and len(angles) == len(MODULES),
            "angles": angles, "brakeIds": brake_ids, "wheelSpeedMps": speed,
            "peakWheelSpeedMps": peak_speed, "drivePercents": drive_percents, "counts": counts,
            "linkOk": bool(telemetry.get("linkOk")), "onlineWheels": len(motors)}


class TrackMotion:
    """Brake ACK, then steering confirmation, then a bounded traction heartbeat."""

    def __init__(self, source_provider: Callable[[], Any],
                 drive: Callable[[Any, dict], tuple],
                 steer: Callable[[Any, dict], tuple],
                 cfg: TrackConfig, *, simulation: bool = False,
                 clock: Callable[[], float] = time.monotonic):
        self._source = source_provider
        self._drive = drive
        self._steer = steer
        self.cfg = cfg
        self.simulation = simulation
        self.clock = clock
        self.steer_deg = 0.0
        self.percent = 0.0
        self.settle_until = 0.0
        self.brake_id: int | None = None
        self.brake_at = 0.0
        self.steering_at: float | None = None
        self.command_count = 0
        self.failures = 0

    def _record(self, result: Any, code: str, message: str) -> None:
        self.command_count += 1
        if not isinstance(result, dict) or not result.get("ok"):
            self.failures += 1
            why = result.get("reason") if isinstance(result, dict) else None
            raise TrackAutonomyError(code, f"{message}: {why or 'нет ответа'}", 503)

    def _send_drive(self, percent: float) -> None:
        result = {"ok": True} if self.simulation else self._drive(
            self._source(), {"percent": round(percent, 2)})[0]
        self._record(result, "drive_refused", "Шасси отклонило тягу")
        self.percent = percent
        if percent:
            self.brake_id = None

    def _send_steer(self, target: float) -> None:
        if self.simulation:
            self.command_count += 1
            return
        source = self._source()
        for module in MODULES:
            result, _status = self._steer(source, {"module": module, "angle": target})
            self._record(result, "steer_refused", f"Руль {module} отклонил угол")

    def stop(self) -> None:
        self.percent = 0.0
        self.steering_at = None
        self.settle_until = 0.0
        self.brake_id = None
        self.brake_at = self.clock()
        if self.simulation:
            self._send_drive(0)
            return
        brake = getattr(self._source(), "brake_for_keyboard", None)
        result = brake() if callable(brake) else {}
        brake_id = result.get("brakeId") if isinstance(result, dict) else None
        if not isinstance(result, dict) or not result.get("ok") or type(brake_id) is not int or brake_id <= 0:
            self._send_drive(0)  # best effort if tagged STOP could not be issued
            self.failures += 1
            raise TrackAutonomyError("brake_unconfirmed", "Не удалось отправить подтверждаемый STOP", 503)
        self.command_count += 1
        self.brake_id = brake_id

    def hold(self) -> None:
        # Do not invalidate an outstanding STOP ID or a three-second dwell on
        # every tick. Do cancel a pending steering command on a safety hold.
        if self.percent or self.steering_at is not None or (not self.simulation and self.brake_id is None):
            self.stop()

    def stopped(self, feedback: dict[str, Any]) -> bool:
        if self.simulation:
            speed = _number(feedback.get("wheelSpeedMps"))
            return self.percent == 0 and speed is not None and 0 <= speed <= 0.01
        acknowledged = (feedback.get("ready") and feedback.get("stopped") and self.brake_id is not None and
                        all(feedback["brakeIds"].get(m) == self.brake_id for m in MODULES))
        if not acknowledged and self.brake_id is not None and self.clock() - self.brake_at > BRAKE_TIMEOUT_S:
            raise TrackAutonomyError("brake_unconfirmed", "Колёса не подтвердили полную остановку", 503)
        return bool(acknowledged)

    def probe_angle(self, steer_left_deg: float) -> float:
        """Actual requested trajectory after quantization and steering deadband."""
        target = round(_clamp(-steer_left_deg, -self.cfg.max_steer_deg, self.cfg.max_steer_deg), 1)
        return self.steer_deg if abs(target - self.steer_deg) <= self.cfg.steer_deadband_deg else target

    @staticmethod
    def _aligned(feedback: dict[str, Any], target: float) -> bool:
        angles = feedback.get("angles") or {}
        return set(angles) == set(MODULES) and all(abs(a - target) <= STEER_TOLERANCE_DEG for a in angles.values())

    def command(self, percent: float, steer_left_deg: float, feedback: dict[str, Any]) -> bool:
        """Return True only once nonzero power is sent."""
        target = self.probe_angle(steer_left_deg)
        now = self.clock()
        if not self.simulation and not feedback.get("ready"):
            raise TrackAutonomyError("chassis_feedback", "Нет свежей телеметрии четырёх колёс", 503)
        if self.steering_at is not None:
            aligned = self.simulation or self._aligned(feedback, self.steer_deg)
            if now < self.settle_until or not aligned:
                if now - self.steering_at > BRAKE_TIMEOUT_S:
                    raise TrackAutonomyError("steer_unconfirmed", "Четыре колеса не подтвердили угол", 503)
                return False
            self.steering_at = None
        aligned = self.simulation or self._aligned(feedback, target)
        if target != self.steer_deg or not aligned or not self.percent:
            if not self.simulation and not self.stopped(feedback):
                self.hold()
                return False
            if (self.simulation and target != self.steer_deg) or not aligned:
                if self.simulation:
                    self.stop()
                self._send_steer(target)
                self.steer_deg = target
                self.steering_at = now
                self.settle_until = now + self.cfg.steer_settle_s
                return False
        self.steer_deg = target
        self._send_drive(percent)
        return percent > 0


class Odometer:
    """Пройденный путь по счётчикам SC колёс; без телеметрии не выдумываем метры."""

    def __init__(self, telemetry_provider: Callable[[], Any]):
        self._telemetry = telemetry_provider
        self.distance = 0.0
        self.delta = 0.0
        self.source = "none"
        self._last: dict[str, int] | None = None

    def _read(self) -> dict[str, Any]:
        try:
            value = self._telemetry()
        except Exception:  # noqa: BLE001 — отсутствие телеметрии не исключение для цикла
            return {}
        return value if isinstance(value, dict) else {}

    def update(self, dt: float, telemetry: dict[str, Any] | None = None) -> float:
        telemetry = self._read() if telemetry is None else telemetry
        motors = [m for m in telemetry.get("motors") or [] if isinstance(m, dict) and m.get("online")]
        counts: dict[str, int] = {}
        ppr: dict[str, float] = {}
        for motor in motors:
            name = str(motor.get("id") or "")
            count, pulses = motor.get("scCount"), motor.get("scPpr")
            if (name in MODULES and type(count) is int and isinstance(pulses, int) and pulses > 0):
                counts[name] = count
                ppr[name] = float(pulses)
        self.delta = 0.0
        if len(counts) == 4 and self._last is not None:
            deltas = []
            for name, value in counts.items():
                step = (value - self._last.get(name, value)) & 0xFFFFFFFF
                if step > MAX_SC_EDGES_PER_S * (dt + 0.5) + 2:
                    # Reset/discontinuity isn't odometry. Real traction also
                    # checks the per-wheel samples below before using distance.
                    step = 0
                deltas.append(step / ppr[name] * WHEEL_CIRCUMFERENCE_M)
            self.delta = max(0.0, sum(deltas) / len(deltas))
            self.source = "sc"
        else:
            speed = telemetry.get("wheelSpeedMps")
            if isinstance(speed, (int, float)) and math.isfinite(float(speed)) and speed > 0 and dt > 0:
                self.delta = float(speed) * dt
                self.source = "wheel"
            else:
                self.source = "none"
        self._last = counts if len(counts) == 4 else None
        self.distance += self.delta
        return self.delta

    def telemetry_snapshot(self) -> dict[str, Any]:
        return chassis_feedback(self._read())


# --------------------------------------------------------------------- сервис
class TrackAutonomyService:
    """Фоновый цикл трассы + статус/предполётные проверки для вкладки."""

    def __init__(self, *, config: Optional[TrackConfig] = None,
                 vision_provider: Optional[Callable[[], Any]] = None,
                 layout_provider: Optional[Callable[[], Any]] = None,
                 kinect_provider: Optional[Callable[[], Any]] = None,
                 lidar_provider: Optional[Callable[[float], Any]] = None,
                 imu_provider: Optional[Callable[[], Any]] = None,
                 telemetry_provider: Optional[Callable[[], Any]] = None,
                 perception_provider: Optional[Callable[[float], dict[str, Any]]] = None,
                 source_provider: Optional[Callable[[], Any]] = None,
                 rivals_provider: Optional[Callable[[], list[str]]] = None,
                 preempt: Optional[Callable[[], None]] = None,
                 drive: Optional[Callable[[Any, dict], tuple]] = None,
                 steer: Optional[Callable[[Any, dict], tuple]] = None,
                 estop: Optional[Callable[[], Any]] = None,
                 clock: Callable[[], float] = time.monotonic,
                 log_events: bool = True):
        self.cfg = config or TrackConfig.from_env()
        self._vision = vision_provider or (lambda: {})
        self._layout = layout_provider or (lambda: {})
        self._kinect = kinect_provider or (lambda: None)
        self._lidar = lidar_provider or (lambda _max_age: {})
        self._imu = imu_provider or (lambda: {})
        self._telemetry = telemetry_provider or (lambda: {})
        self._perception = perception_provider
        self._source = source_provider or (lambda: None)
        self._rivals = rivals_provider or (lambda: [])
        self._preempt = preempt or (lambda: None)
        self._estop = estop or estop_latch.latched
        self.clock = clock
        self.log_events = log_events
        self._bump_detector = BumpDetector()
        self._odometer = Odometer(self._telemetry)
        self.motion = TrackMotion(
            self._source,
            drive or self._noop_dispatch,
            steer or self._noop_dispatch,
            self.cfg, simulation=self.cfg.simulation or self._perception is not None,
            clock=clock,
        )
        self._lock = threading.RLock()
        self._motion_lock = threading.RLock()
        self._cancel = threading.Event()
        self._stop_epoch = 0
        self._worker: Optional[threading.Thread] = None
        self._session: Optional[dict[str, Any]] = None
        self._state: dict[str, Any] = {
            "phase": "idle", "message": "Автономность не запущена",
            "distanceM": 0.0, "percent": 0.0, "steerLeftDeg": 0.0,
            "boostPercent": 0.0, "events": [],
            "counters": {"stop": 0, "bump": 0, "traffic": 0, "powerBoosts": 0},
            "perception": {}, "corridor": {}, "traffic": {}, "ground": {},
            "imu": {}, "telemetry": {}, "signs": [], "cones": [],
        }

    @staticmethod
    def _noop_dispatch(_source: Any, _body: dict) -> tuple[dict, int]:
        return {"ok": False, "reason": "control_not_configured"}, 409

    # ------------------------------------------------------------ состояние ---
    @property
    def active(self) -> bool:
        with self._lock:
            worker = self._worker
            return bool(worker and worker.is_alive())

    def _set(self, **fields: Any) -> None:
        with self._lock:
            self._state.update(fields)

    def _is_estopped(self) -> bool:
        try:
            return self._estop() is not None
        except Exception:  # noqa: BLE001 — fail-safe: сбой проверки = стоим
            return True

    # ---------------------------------------------------------- предполёт ---
    def _vision_snapshot(self) -> dict[str, Any]:
        try:
            value = self._vision()
        except Exception:  # noqa: BLE001
            return {}
        return value if isinstance(value, dict) else {}

    def _layout_snapshot(self) -> dict[str, Any]:
        try:
            value = self._layout()
        except Exception:  # noqa: BLE001
            return {}
        return value if isinstance(value, dict) else {}

    def _scan_info(self) -> dict[str, Any]:
        try:
            info = self._lidar(self.cfg.lidar_scan_age_s)
            return info if isinstance(info, dict) else {}
        except Exception:  # noqa: BLE001 — fail closed on scan loss
            return {}

    def _rival_modes(self) -> list[str]:
        try:
            return [str(item) for item in (self._rivals() or []) if item]
        except Exception:  # noqa: BLE001
            return ["нет статуса владельца тяги"]

    def preflight(self, *, blind_zone_checked: bool = False,
                  for_start: bool = False) -> dict[str, Any]:
        blockers: list[str] = []
        warnings: list[str] = []
        checks: dict[str, bool] = {}
        vision = self._vision_snapshot()
        layout = self._layout_snapshot()
        source = None
        try:
            source = self._source()
        except Exception:  # noqa: BLE001
            pass
        fresh_cameras, front_fresh = _vision_readiness(vision, self.cfg)
        cones = cone_points(vision, layout, self.cfg)
        corridor = estimate_corridor(cones, self.cfg)
        lidar = lidar_clearance(self._scan_info(), corridor["steerLeftDeg"], self.cfg)
        real = not self.motion.simulation
        require_blind = real or self.cfg.require_blind_check
        estop_clear = not self._is_estopped()
        vision_ok = bool(vision.get("ok") and vision.get("enabled"))
        checks.update({
            "enabled": self.cfg.enabled,
            "estopClear": estop_clear,
            "visionOk": vision_ok,
            "visionRunning": bool(vision.get("running")),
            "camerasFresh": front_fresh,
            "conesVisible": len(cones) >= self.cfg.min_cones,
            "corridorOk": bool(corridor.get("ok")),
            "geometryCalibrated": bool(layout.get("calibrated")) and not layout.get("duplicatePorts"),
            "lidarReady": bool(lidar["ready"]), "lidarClear": bool(lidar["ready"] and not lidar["blocked"]),
            "driveReady": bool(getattr(source, "supports_drive", False)),
            "steerReady": bool(getattr(source, "supports_individual_steer", False)),
            "brakeFeedback": callable(getattr(source, "brake_for_keyboard", None)),
            "depthFresh": False,
            "imuFresh": False,
            "rivalsIdle": True,
            "blindZone": bool(blind_zone_checked),
            "standstill": True,
        })
        try:
            checks["depthFresh"] = kinect_navigation_ready(self._kinect(), self.cfg)
        except Exception:  # noqa: BLE001
            pass
        checks["imuFresh"] = bool(imu_snapshot(self._imu()).get("fresh"))
        rivals = self._rival_modes()
        checks["rivalsIdle"] = not rivals
        telemetry = self._odometer.telemetry_snapshot()
        online_wheels = int(telemetry.get("onlineWheels") or 0)
        checks["chassisLink"] = bool(telemetry.get("linkOk"))
        checks["wheelsOnline"] = online_wheels >= len(MODULES)
        checks["wheelFeedback"] = bool(telemetry["ready"])
        checks["standstill"] = (telemetry["stopped"] and len(telemetry["angles"]) == len(MODULES)
                                if real else (telemetry.get("wheelSpeedMps") or 0) <= 0.05)
        checks["powerLimits"] = self.cfg.slow_percent <= self.cfg.cruise_percent <= self.cfg.climb_percent
        drive_limit = getattr(source, "drive_limit_percent", None)
        if real and callable(drive_limit):
            try:
                limit = _number(drive_limit())
            except Exception:  # noqa: BLE001 — no confirmed power budget
                limit = None
            checks["powerLimits"] = checks["powerLimits"] and limit is not None and self.cfg.climb_percent <= limit

        if not checks["enabled"]:
            blockers.append("Режим выключен: задайте RC_TRACK_ENABLE=1 в .env и перезапустите киоск")
        if not checks["estopClear"]:
            blockers.append("Взведён E-STOP: снимите аварийную остановку")
        if not checks["visionOk"] or not checks["visionRunning"]:
            blockers.append("Машинное зрение не работает: проверьте /api/vision/state и камеры")
        if not checks["camerasFresh"]:
            blockers.append("Нужны свежие кадры обеих передних камер (%.1f с)" % self.cfg.stale_s)
        if not checks["conesVisible"]:
            blockers.append("Конусы не видны: поставьте робота в коридор и проверьте разметку кадров")
        if not checks["corridorOk"]:
            blockers.append(PASSAGE_REASONS.get(corridor["reason"], "Проезд не подтверждён"))
        if rivals:
            blockers.append("Другой режим уже владеет приводом: " + ", ".join(rivals))
        if not checks["standstill"]:
            blockers.append("Не подтверждена полная остановка колёс")
        if not checks["powerLimits"]:
            blockers.append("Проверьте тягу: малая ≤ обычная ≤ предел на кочке и не выше лимита шасси")
        if (real or self.cfg.require_depth) and not checks["depthFresh"]:
            blockers.append("Нет свежего навигационного кадра Kinect 360: "
                            "кочку подтверждать нечем (preview не для навигации)")
        if self.cfg.require_imu and not checks["imuFresh"]:
            blockers.append("Нет свежего гироскопа/наклона: подъём на кочке не контролируется")
        if real:
            if not checks["geometryCalibrated"]:
                blockers.append("Откалибруйте геометрию камер и Kinect; проверьте порты камер")
            if not checks["lidarReady"]:
                blockers.append("Нет свежего полного физического скана лидара")
            elif not checks["lidarClear"]:
                blockers.append("Лидар видит препятствие на пути корпуса")
            if not checks["wheelFeedback"] or not checks["brakeFeedback"]:
                blockers.append("Нужны подтверждаемый STOP и свежая телеметрия SC всех колёс")
            if not checks["driveReady"]:
                blockers.append("Шасси не готово к тяге: нужен RC_SLAM_SOURCE=master")
            if not checks["steerReady"]:
                blockers.append("Шасси не умеет адресное руление: поворот по конусам недоступен")
            if not checks["chassisLink"]:
                blockers.append("Нет свежей связи с платами колёс (@TLM): проверьте USB и питание")
            if not checks["wheelsOnline"]:
                blockers.append("Не все четыре колеса онлайн: %d из %d" %
                                (online_wheels, len(MODULES)))
        # Галочка осмотра — действие оператора в конкретном пуске, а не свойство
        # робота: status() не должен из-за неё вечно показывать canStart=False.
        if require_blind and not checks["blindZone"]:
            if for_start:
                blockers.append("Осмотрите зону вокруг робота и поставьте галочку подтверждения")
            else:
                warnings.append("Перед пуском осмотрите зону и поставьте галочку подтверждения")
        if not real:
            warnings.append("Симуляция: команды тяге НЕ отправляются")
            if not checks["geometryCalibrated"]:
                warnings.append("Геометрия камер не откалибрована: углы взяты по умолчанию")
            if not checks["lidarReady"]:
                warnings.append("Нет физического скана лидара: прогон не проверяет препятствия")
        if not checks["depthFresh"]:
            warnings.append("Нет полного кадра Kinect: перед кочкой режим встанет, "
                            "используйте raw depth, а не points/ranges")
        if not checks["imuFresh"]:
            warnings.append("Гироскоп без свежих данных: переезд кочки заблокирован")
        if self.active:
            blockers.append("Автономность уже идёт")
        return {
            "ok": True, "blockers": blockers, "warnings": warnings,
            "checks": checks, "freshCameras": fresh_cameras,
            "conesVisible": len(cones), "canStart": not blockers,
            "simulation": not real,
            "requireBlindCheck": require_blind,
            "forStart": for_start,
            "preview": {"corridor": corridor, "lidar": lidar, "cones": cones[:24],
                        "signs": sign_marks(vision, layout, self.cfg)[:12]},
        }

    def status(self) -> dict[str, Any]:
        with self._lock:
            state = dict(self._state)
            session = dict(self._session) if self._session else None
        preflight = self.preflight()
        payload = {
            "ok": True, "active": self.active,
            "enabled": self.cfg.enabled, "simulation": self.cfg.simulation,
            "config": self.cfg.public(),
            "updatedAt": round(time.time(), 3),
            "elapsedS": round(max(0.0, self.clock() - session["started"]), 2) if session else 0.0,
            "drive": {"percent": self.motion.percent, "steerDeg": round(self.motion.steer_deg, 2),
                      "commands": self.motion.command_count, "failures": self.motion.failures,
                      "simulated": self.motion.simulation},
            "odometry": {"source": self._odometer.source, "distanceM": round(self._odometer.distance, 3)},
        }
        payload.update(state)
        preview = preflight.pop("preview", {})
        if not payload["active"]:
            payload.update(preview)
        payload.update(preflight)
        return payload

    # -------------------------------------------------------------- пуск/стоп
    def _new_session(self, now: float) -> dict[str, Any]:
        return {
            "started": now, "last": now, "finished": False,
            "lane_lost_since": None, "vision_lost_since": None,
            "stop_target": None, "stop_seen_at": None, "stop_braking": False,
            "stop_wait_until": None, "stop_served": None,
            "light_target": None, "light_seen_at": None, "light_waiting": False,
            "light_passed": False, "green_camera": None, "green_stamp": None, "green_frames": 0,
            "light_frames": {},
            "bump_armed": False, "bump_expected_at": None, "bump_active": False, "bump_cross_start": 0.0,
            "bump_cross_since": None, "bump_cross_distance": 0.0,
            "bump_post_until": 0.0, "bump_consumed_until": 0.0, "bump_wait_since": None,
            "boost": 0.0, "max_boost": 0.0, "stall_since": None,
            "drive_sample": None, "pulse_times": {},
            "events": [],
        }

    def start_ticket(self) -> int:
        with self._lock:
            return self._stop_epoch

    def start(self, body: Any = None, *, ticket: int | None = None) -> tuple[dict[str, Any], int]:
        ticket = self.start_ticket() if ticket is None else ticket
        payload = body if isinstance(body, dict) else {}
        blind = payload.get("blindZoneChecked") is True
        checks = self.preflight(blind_zone_checked=blind, for_start=True)
        if checks["blockers"]:
            raise TrackAutonomyError("not_ready", "; ".join(checks["blockers"]), 409)
        with self._motion_lock, self._lock:
            if ticket != self._stop_epoch:
                raise TrackAutonomyError("start_interrupted", "Пуск отменён командой «Стоп»")
            if self.active:
                raise TrackAutonomyError("already_running", "Автономность уже идёт", 409)
            if self._is_estopped():
                raise TrackAutonomyError("not_ready", "Взведён E-STOP", 409)
            if not self.motion.simulation:
                self._preempt()
            self.motion.stop()
            self._bump_detector.reset()
            self._odometer = Odometer(self._telemetry)
            self._cancel = threading.Event()
            self._session = self._new_session(self.clock())
            self._state = {
                "phase": "starting", "message": "Проверка камер, проезда и датчиков…",
                "distanceM": 0.0, "percent": 0.0, "steerLeftDeg": 0.0,
                "boostPercent": 0.0, "events": [],
                "counters": {"stop": 0, "bump": 0, "traffic": 0, "powerBoosts": 0},
                "perception": {}, "corridor": {}, "traffic": {}, "ground": {}, "lidar": {},
                "imu": {}, "telemetry": {}, "signs": [], "cones": [],
            }
            self._event(self._session, "starting", "Автономность запущена оператором", "ok")
            worker = threading.Thread(target=self._run, args=(self._cancel,),
                                      name="robot-track-autonomy", daemon=True)
            self._worker = worker
            worker.start()
        return {"ok": True, "status": self.status()}, 202

    def interrupt(self, reason: str = "Остановлено оператором") -> None:
        """Retire the track before competing commands may take the chassis."""
        # Also invalidate a Start that has entered preflight but not armed the worker.
        with self._lock:
            self._stop_epoch += 1
            self._cancel.set()
        with self._motion_lock:
            session = self._session
            if session is not None and not session["finished"]:
                self._finish(session, "cancelled", reason, "warn")

    def stop(self, reason: str = "Остановлено оператором") -> dict[str, Any]:
        self.interrupt(reason)
        return {"ok": True, "status": self.status()}

    # ------------------------------------------------------------- восприятие
    def _perceive(self, now: float) -> dict[str, Any]:
        if self._perception is not None:
            return self._perception(now) or {}
        vision = self._vision_snapshot()
        layout = self._layout_snapshot()
        cones = cone_points(vision, layout, self.cfg)
        signs = sign_marks(vision, layout, self.cfg)
        traffic = traffic_decision(vision, signs, self.cfg)
        imu = imu_snapshot(self._imu())
        ground = self._bump_detector.update(self._kinect(), imu, now, self.cfg)
        fresh_cameras, front_fresh = _vision_readiness(vision, self.cfg)
        return {
            "ok": bool(vision.get("ok") and vision.get("enabled") and vision.get("running")),
            "fresh": front_fresh, "freshCameras": fresh_cameras, "cones": cones, "signs": signs,
            "traffic": traffic, "ground": ground, "imu": imu,
            "geometryCalibrated": bool(layout.get("calibrated")) and not layout.get("duplicatePorts"),
        }

    # ---------------------------------------------------------------- журнал --
    def _event(self, session: dict[str, Any], phase: str, message: str,
               tone: str = "info", *, record: bool = True) -> None:
        entry = {"atS": round(max(0.0, self.clock() - session["started"]), 2),
                 "distanceM": round(self._odometer.distance, 2),
                 "phase": phase, "message": message, "tone": tone}
        session["events"] = (session["events"] + [entry])[-120:]
        self._set(events=list(session["events"]))
        if record and self.log_events:
            level = {"ok": "ok", "warn": "warn", "stop": "warn", "err": "err"}.get(tone, "info")
            eventlog.record(level, "автономность: %s" % message, source="трасса")

    def _finish(self, session: dict[str, Any], phase: str, message: str,
                tone: str = "info") -> None:
        with self._motion_lock:
            if session["finished"]:
                return
            session["finished"] = True
            try:
                self.motion.stop()
            except Exception:  # noqa: BLE001 — watchdog is the final fallback
                message += " STOP не подтверждён; проверьте физический E-STOP."
            self._event(session, phase, message, tone)
            self._set(phase=phase, message=message, percent=0.0, boostPercent=0.0)

    def _hold(self, session: dict[str, Any], phase: str, message: str) -> None:
        with self._motion_lock:
            if not session["finished"] and not self._cancel.is_set():
                self.motion.hold()
                self._set(phase=phase, message=message, percent=0.0, boostPercent=0.0)

    def _count(self, key: str) -> None:
        counters = dict(self._state["counters"])
        counters[key] += 1
        self._set(counters=counters)

    def _stop_sign(self, session: dict[str, Any], signs: list[dict[str, Any]],
                   now: float, distance: float, feedback: dict[str, Any]) -> tuple[bool, bool]:
        """Return (must hold, approach slowly), with dwell measured after brake ACK."""
        threshold = self.cfg.stop_offset_m + self.cfg.robot_length_m / 2
        if not session["stop_braking"]:
            served = session["stop_served"]
            marks = [m for m in signs if m["class"] == "stop_sign" and m["x"] >= -0.5
                     and abs(m["y"]) <= self.cfg.max_lateral_m and
                     (served is None or math.hypot(distance + m["x"] - served[0],
                                                  m["y"] - served[1]) > 0.8)]
            if marks:
                mark = min(marks, key=lambda m: m["x"])
                session["stop_target"] = (distance + mark["x"], mark["y"])
                session["stop_seen_at"] = now
                session["stop_braking"] = (mark["x"] <= threshold or
                    (mark["side"] == "side" and mark["distance"] <= self.cfg.stop_side_m
                     and mark["x"] <= threshold + 0.30))
            elif session["stop_target"] is not None:
                if session["stop_target"][0] - distance <= threshold + self.cfg.stop_clearance_m:
                    session["stop_braking"] = True
                elif now - session["stop_seen_at"] > self.cfg.stop_pass_s:
                    raise TrackAutonomyError("stop_sign_lost", "Знак СТОП потерян до подъезда")
                else:
                    self._hold(session, "stop_braking", "Знак СТОП пропал из кадра — жду его снова.")
                    return True, False
        if not session["stop_braking"]:
            target = session["stop_target"]
            return False, target is not None and target[0] - distance <= max(1.6, threshold + self.cfg.stop_clearance_m)

        if self.motion.steering_at is not None or not self.motion.stopped(feedback):
            session["stop_wait_until"] = None
            self._hold(session, "stop_braking", "СТОП: жду подтверждения полной остановки всех колёс.")
            return True, False
        if session["stop_wait_until"] is None:
            seconds = max(3.0, self.cfg.stop_wait_s)
            session["stop_wait_until"] = now + seconds
            self._event(session, "stop_wait", f"Полная остановка подтверждена: жду {seconds:g} с.", "stop")
        remaining = session["stop_wait_until"] - now
        if remaining > 0:
            self._hold(session, "stop_wait", f"СТОП: осталось {remaining:.1f} с.")
            return True, False
        session["stop_braking"] = False
        session["stop_wait_until"] = None
        session["stop_served"] = session["stop_target"]
        session["stop_target"] = None
        self._count("stop")
        self._event(session, "following", "Выдержка у знака СТОП завершена.", "ok")
        return False, False

    def _traffic(self, session: dict[str, Any], traffic: dict[str, Any],
                 signs: list[dict[str, Any]], now: float, distance: float) -> bool:
        """Latch the light until it is passed; every red/yellow reasserts STOP."""
        marks = [mark for mark in signs if mark["class"] == "traffic_light"]
        target = session["light_target"]
        if target is not None:
            marks = [m for m in marks if math.hypot(distance + m["x"] - target[0], m["y"] - target[1]) <= 1.5]
        if session["light_passed"]:
            # Only an observed passage can release a lost/occluded light.
            # A new signal ahead starts another gate, not a distance cooldown.
            ahead = [m for m in signs if m["class"] == "traffic_light" and m["x"] >= 0.0]
            if not ahead:
                return False
            session.update(light_passed=False, light_target=None, light_seen_at=None,
                           green_frames=0, green_camera=None, green_stamp=None)
            target, marks = None, ahead
        visible = bool(traffic.get("visible") and (marks or target is None))
        if visible:
            if marks and target is None:
                mark = min(marks, key=lambda m: abs(m["x"]))
                session["light_target"] = (distance + mark["x"], mark["y"])
            session["light_seen_at"] = now
        seen = session["light_seen_at"]
        if seen is None:
            return False
        if not visible and now - seen > self.cfg.traffic_lost_s:
            raise TrackAutonomyError("signal_lost", "Светофор потерян до подтверждённого проезда")
        cam, stamp = traffic.get("cameraId"), _number(traffic.get("frameStamp"))
        green = (visible and traffic.get("fresh") is True and traffic.get("state") == "green" and
                 not traffic.get("hold") and type(cam) is int and stamp is not None and stamp > 0 and
                 any(m.get("cameraId") == cam and m.get("signal") == "green" for m in marks))
        # Remember evidence even while stopped or when the light briefly drops
        # out of view. Replayed earlier green frames cannot undo a newer red.
        history = session["light_frames"]
        if green:
            if cam != session["green_camera"]:
                session.update(green_camera=cam, green_stamp=None, green_frames=0)
            if stamp > history.get(cam, 0.0):
                session["green_stamp"] = stamp
                session["green_frames"] = min(2, session["green_frames"] + 1)
            elif stamp != session["green_stamp"] or stamp < history[cam]:
                session.update(green_stamp=None, green_frames=0)
        else:
            session.update(green_camera=None, green_stamp=None, green_frames=0)
        frames = dict(traffic.get("frames") or {})
        if type(cam) is int and stamp is not None:
            frames[cam] = stamp
        for cid, frame in frames.items():
            previous = history.get(cid, 0.0)
            value = _number(frame)
            if value is not None and value > previous:
                history[cid] = value
        if not green or session["green_frames"] < 2:
            if not session["light_waiting"]:
                session["light_waiting"] = True
                self._event(session, "traffic_wait", "Светофор: стою до свежего подтверждённого зелёного.", "stop")
            return True
        if session["light_waiting"]:
            session["light_waiting"] = False
            self._event(session, "following", "Светофор: зелёный подтверждён на двух кадрах.", "ok")
        if marks and max(m["x"] for m in marks) <= -self.cfg.robot_length_m / 2:
            session["light_passed"] = True
            self._count("traffic")
        return False

    def _bump_speed(self, session: dict[str, Any], signs: list[dict[str, Any]], ground: dict[str, Any],
                    imu: dict[str, Any], now: float, distance: float, delta: float,
                    angle: float) -> tuple[str, float, str]:
        cfg = self.cfg
        bump = _number(ground.get("nearestBumpM")) if ground.get("ok") else None
        sign = min((m["x"] for m in signs if m["class"] == "speed_bump" and m["x"] >= -0.35
                    and abs(m["y"]) <= cfg.max_lateral_m), default=None)
        if distance >= session["bump_consumed_until"] and sign is not None and sign <= cfg.bump_sign_m:
            session["bump_armed"] = True
            session["bump_expected_at"] = distance + sign
        for hazard in ground.get("hazards") or []:
            height = _number(hazard.get("severityM"))
            box = hazard.get("bbox")
            if box and (hazard["kind"] == "pothole" or height is not None and height > cfg.bump_max_height_m):
                x0, y0, x1, y1 = box
                if swept_hit((x0 + x1) / 2, (y0 + y1) / 2, angle, cfg.stop_clearance_m,
                             (cfg.robot_length_m + x1 - x0) / 2 + cfg.clearance_m,
                             (cfg.robot_width_m + y1 - y0) / 2 + cfg.clearance_m):
                    raise TrackAutonomyError("unsafe_ground", "Kinect: яма или слишком высокая неровность на пути")

        entry_m = cfg.robot_length_m / 2 + 0.45
        if session["bump_active"]:
            if now - session["bump_cross_since"] > cfg.bump_cross_timeout_s:
                raise TrackAutonomyError("bump_timeout", "Не подтверждён переезд кочки за отведённое время")
            if not ground.get("ok") or not imu.get("fresh"):
                raise TrackAutonomyError("bump_feedback", "На кочке потеряна глубина или данные о наклоне")
            if distance - session["bump_cross_start"] >= session["bump_cross_distance"]:
                session.update(bump_active=False, bump_armed=False, bump_post_until=now + cfg.post_bump_s,
                               bump_consumed_until=distance + cfg.sign_cooldown_m,
                               boost=0.0, stall_since=None)
                self._count("bump")
                self._event(session, "bump_exit", "Переезд кочки подтверждён — держу малую тягу.", "ok")
                return "bump_exit", cfg.slow_percent, "Кочка пройдена, продолжаю медленно."

        if session["bump_armed"] and not session["bump_active"]:
            expected = session["bump_expected_at"] - distance
            if bump is not None and 0 < bump <= entry_m and imu.get("fresh"):
                far = max(bump, _number(ground.get("bumpFarM")) or bump)
                # SC measures distance along the ray, not forward projection.
                crossing = (far + cfg.robot_length_m / 2 + 0.10) / math.cos(math.radians(cfg.max_steer_deg))
                session.update(bump_active=True, bump_cross_start=distance, bump_cross_since=now,
                               bump_cross_distance=max(cfg.bump_cross_m, crossing),
                               bump_wait_since=None, boost=0.0, stall_since=None)
                self._event(session, "bump_climb", "Кочка подтверждена Kinect, начинаю переезд.", "warn")
            elif expected <= entry_m or (bump is not None and bump <= entry_m):
                if session["bump_wait_since"] is None:
                    session["bump_wait_since"] = now
                if now - session["bump_wait_since"] > cfg.bump_cross_timeout_s:
                    raise TrackAutonomyError("bump_unconfirmed", "Перед кочкой нет подтверждения глубины или наклона")
                return "bump_wait", 0.0, "Стою: жду подтверждённую Kinect кочку и свежий наклон."
            elif min(expected, bump if bump is not None else expected) <= cfg.bump_slow_m:
                return "bump_slow", cfg.slow_percent, "Подъезжаю к кочке на малой тяге."
        elif not session["bump_active"] and now >= session["bump_post_until"] and bump is not None and bump <= entry_m:
            if distance >= session["bump_consumed_until"]:
                raise TrackAutonomyError("unmarked_bump", "Неровность на пути без распознанного знака — STOP")

        if session["bump_active"]:
            # Boost only while actually commanding traction on a measured incline,
            # never while braking/aligning or waiting for a light.
            if delta > 0.002 or self.motion.percent <= 0:
                session["stall_since"] = None
            elif session["stall_since"] is None:
                session["stall_since"] = now
            stall = session["stall_since"]
            if stall is not None and now - stall >= cfg.stall_s and abs(imu["pitch"]) >= cfg.pitch_deg:
                added = min(cfg.climb_step_percent, cfg.climb_percent - cfg.slow_percent - session["boost"])
                if added <= 0:
                    raise TrackAutonomyError("bump_stall", "Кочка не пройдена на предельной тяге — STOP")
                session["boost"] += added
                session["max_boost"] = max(session["max_boost"], session["boost"])
                session["stall_since"] = now
                self._count("powerBoosts")
                self._event(session, "bump_climb", f"Нет прогресса на подъёме: добавка тяги +{added:g}%.", "warn")
            return "bump_climb", cfg.slow_percent + session["boost"], "Переезжаю подтверждённую кочку."
        if now < session["bump_post_until"]:
            return "bump_exit", cfg.slow_percent, "После кочки держу малую тягу."
        return "following", cfg.cruise_percent, "Двигаюсь через подтверждённый проезд."

    def _check_progress(self, session: dict[str, Any], feedback: dict[str, Any], now: float) -> None:
        """Reject missing traction acknowledgments and an unresponsive wheel."""
        sample = session["drive_sample"]
        commanded = int(round(self.motion.percent))
        if not commanded:
            session.update(drive_sample=None, pulse_times={})
            return
        counts = feedback["counts"]
        if sample is None:
            session["pulse_times"] = dict.fromkeys(MODULES, now)
        else:
            for module in MODULES:
                edges = (counts[module] - sample["counts"][module]) & 0xFFFFFFFF
                if edges > MAX_SC_EDGES_PER_S * (max(0.0, now - sample["time"]) + 0.5) + 2:
                    raise TrackAutonomyError("sc_changed", f"Счётчик SC колеса {module} сброшен или повреждён")
                if edges:
                    session["pulse_times"][module] = now
                if not session["bump_active"] and now - session["pulse_times"][module] >= 3.0:
                    raise TrackAutonomyError("no_progress", f"Колесо {module} не подтвердило движение; проверьте калибровку тяги")
        since = sample["since"] if sample is not None and sample["percent"] == commanded else now
        if now - since > WHEEL_TELEMETRY_MAX_AGE_S and any(
                feedback["drivePercents"][m] != commanded for m in MODULES):
            raise TrackAutonomyError("drive_unconfirmed", "Колёса не подтвердили команду тяги")
        session["drive_sample"] = {"counts": counts, "time": now, "since": since, "percent": commanded}

    def _step(self, now: float) -> None:
        session = self._session
        if session is None or session["finished"] or self._cancel.is_set():
            return
        dt = max(0.0, min(1.0, now - session["last"]))
        session["last"] = now
        try:
            self._step_checked(session, now, dt)
        except TrackAutonomyError as exc:
            self._finish(session, "error", exc.message, "err")

    def _step_checked(self, session: dict[str, Any], now: float, dt: float) -> None:
        cfg = self.cfg
        if self._is_estopped():
            raise TrackAutonomyError("estop", "E-STOP взведён — тяга снята.")
        if now - session["started"] > cfg.max_duration_s:
            raise TrackAutonomyError("track_timeout", "Достигнут лимит времени; финиш не подтверждён.")
        rivals = self._rival_modes()
        if rivals:
            raise TrackAutonomyError("drive_owned", "Режим тяги изменился: " + ", ".join(rivals))
        snap = self._perceive(now)
        if self._cancel.is_set():
            return
        raw = self._odometer._read()
        real = not self.motion.simulation
        feedback = chassis_feedback(raw) if real else snap.get("telemetry", raw)
        if real:
            if not feedback["ready"]:
                raise TrackAutonomyError("chassis_feedback", "Потеряна свежая телеметрия четырёх колёс")
            if max(feedback["wheelSpeedMps"], feedback["peakWheelSpeedMps"]) > cfg.max_speed_mps:
                raise TrackAutonomyError("overspeed", "Превышен предел скорости по телеметрии колёс")
            if not snap["geometryCalibrated"]:
                raise TrackAutonomyError("geometry", "Нет подтверждённой геометрии камер и Kinect")
            self._check_progress(session, feedback, now)
        delta = self._odometer.update(dt, raw)
        distance = self._odometer.distance
        if distance >= cfg.max_distance_m:
            raise TrackAutonomyError("distance_limit", "Достигнут предохранительный предел дистанции")
        if cfg.finish_distance_m > 0 and distance >= cfg.finish_distance_m:
            self._finish(session, "done", "Достигнута настроенная дистанция финиша.", "ok")
            return

        cones, signs, traffic, ground, imu = (snap[key] for key in ("cones", "signs", "traffic", "ground", "imu"))
        corridor = estimate_corridor(cones, cfg)
        steer = corridor["steerLeftDeg"]
        if imu.get("fresh"):
            steer -= float(imu.get("gyroZ") or 0) * cfg.gyro_damping
        steer = _clamp(steer, -cfg.max_steer_deg, cfg.max_steer_deg)
        angle = -self.motion.probe_angle(steer)
        scan = self._scan_info()
        lidar = lidar_clearance(scan, angle, cfg)
        if self.motion.percent > 0 and angle != -self.motion.steer_deg and lidar["ready"] and not lidar["blocked"]:
            lidar = lidar_clearance(scan, -self.motion.steer_deg, cfg)
        self._set(distanceM=round(distance, 3), perception={"ok": snap["ok"],
                  "freshCameras": snap["freshCameras"], "cones": len(cones), "signs": len(signs)},
                  cones=cones[:24], signs=signs[:12], corridor=corridor, lidar=lidar,
                  traffic=traffic, ground={k: ground.get(k) for k in ("ok", "nearestBumpM", "status", "reason")},
                  imu=imu, telemetry={k: feedback.get(k) for k in ("linkOk", "onlineWheels", "wheelSpeedMps")})
        if lidar["blocked"]:
            self._finish(session, "blocked", "Лидар видит препятствие на пути корпуса — STOP.", "stop")
            return
        if real and not lidar["ready"]:
            raise TrackAutonomyError("lidar_lost", "Потерян свежий полный физический скан лидара")
        if real and not ground.get("depthFresh"):
            raise TrackAutonomyError("depth_lost", "Потерян навигационный кадр глубины Kinect")
        if cfg.require_imu and not imu.get("fresh"):
            raise TrackAutonomyError("imu_lost", "Потеряны данные о наклоне")

        # Keep the traffic latch even during a sign dwell or a temporary lane stop.
        vision_fresh = snap["ok"] and snap["fresh"]
        wait_light = self._traffic(session, traffic if vision_fresh else {}, signs if vision_fresh else [], now, distance)
        if not vision_fresh or not corridor["ok"]:
            session["stop_wait_until"] = None  # don't count time without confirmed feedback
            lost = "vision_lost_since" if not vision_fresh else "lane_lost_since"
            message = ("Нет свежих кадров передних камер" if not vision_fresh else
                       PASSAGE_REASONS.get(corridor["reason"], "Проезд не подтверждён"))
            if session[lost] is None:
                session[lost] = now
            if now - session[lost] > cfg.lane_lost_s:
                self._finish(session, "blocked", message + " — STOP.", "stop")
            else:
                self._hold(session, "lane_lost", message + " — стою.")
            return
        session["vision_lost_since"] = session["lane_lost_since"] = None
        if (not passage_fits(corridor, angle, cfg) or post_in_path(cones, angle, corridor["travelM"], cfg)
                or self.motion.percent > 0 and post_in_path(cones, -self.motion.steer_deg, cfg.stop_clearance_m, cfg)):
            self._finish(session, "blocked", "Угол колёс не оставляет безопасного прохода между столбиками.", "stop")
            return

        wait_stop, slow_stop = self._stop_sign(session, signs, now, distance, feedback)
        if wait_stop:
            return
        if wait_light:
            self._hold(session, "traffic_wait", "Светофор: стою до свежего подтверждённого зелёного.")
            return
        phase, speed, message = self._bump_speed(session, signs, ground, imu, now, distance, delta, angle)
        if not speed:
            self._hold(session, phase, message)
            return
        if slow_stop:
            speed = min(speed, cfg.slow_percent)
            if phase == "following":
                phase, message = "stop_approach", "Подъезжаю к знаку СТОП на малой тяге."
        with self._motion_lock:
            if self._cancel.is_set() or session["finished"]:
                return
            if self._is_estopped():
                raise TrackAutonomyError("estop", "E-STOP взведён — тяга снята.")
            applied = self.motion.command(speed, steer, feedback)
            self._set(phase=phase if applied else "aligning", message=message if applied else
                      "Подтверждаю остановку и угол всех колёс.", percent=round(speed, 2) if applied else 0.0,
                      steerLeftDeg=angle, boostPercent=round(session["boost"], 2))

    def _run(self, cancel: threading.Event) -> None:
        session = self._session
        try:
            while not cancel.is_set() and session is not None and not session["finished"]:
                tick = self.clock()
                self._step(tick)
                cancel.wait(max(0.0, self.cfg.tick_s - (self.clock() - tick)))
        except Exception as exc:  # noqa: BLE001 — барьер остановки и понятный статус
            if session is not None:
                self._finish(session, "error", "Сбой автономности: %s" % str(exc)[:160], "err")
        finally:
            if session is not None and not session["finished"]:
                self._finish(session, "cancelled", "Автономность остановлена оператором.", "warn")
            with self._lock:
                self._worker = None

    # ------------------------------------------------------------- сухой прогон
    def simulate(self, scenario: str = "full") -> dict[str, Any]:
        """Прогнать тот же автомат на скриптованной трассе без железа и тяги.

        Возвращает события и сводку. Ни один провайдер реального робота не
        вызывается,``RC_TRACK_ENABLE`` не нужен, команды не уходят в шасси.
        """
        world = ScriptedWorld(scenario)
        config = replace(TrackConfig(), enabled=True, simulation=True,
                         require_depth=False, require_imu=False,
                         require_blind_check=False,
                         finish_distance_m=44.0,
                         max_duration_s=240.0, max_distance_m=200.0)
        sim = TrackAutonomyService(
            config=config, perception_provider=world.snapshot,
            telemetry_provider=world.telemetry, lidar_provider=world.scan,
            source_provider=lambda: None,
            rivals_provider=lambda: [], estop=lambda: None,
            clock=world.clock, log_events=False,
        )
        sim._session = sim._new_session(world.clock())
        sim._state["phase"] = "starting"
        steps = 0
        max_steps = int(config.max_duration_s / config.tick_s) + 10
        while steps < max_steps and not sim._session["finished"]:
            sim._step(world.clock())
            world.advance(config.tick_s, sim.motion.percent, -sim.motion.steer_deg)
            steps += 1
        session = sim._session
        counters = dict(sim._state.get("counters") or {})
        phase = sim._state["phase"]
        stopped_at_gate = (phase == "blocked" and sim._odometer.distance == 0.0 and
                           sim._state["corridor"].get("reason") == "corridor_narrow")
        required = {"full": ("stop", "bump", "traffic"), "stop": ("stop",),
                    "bump": ("bump",), "light": ("traffic",)}.get(world.scenario, ())
        passed = (stopped_at_gate if world.scenario == "blocked_gate" else
                  phase == "done" and all(counters[key] >= 1 for key in required))
        return {
            "ok": True, "passed": passed, "expectedStop": world.scenario == "blocked_gate",
            "scenario": world.scenario, "steps": steps, "phase": phase,
            "message": sim._state.get("message"),
            "distanceM": round(sim._odometer.distance, 2),
            "elapsedS": round(world.clock() - session["started"], 2),
            "counters": counters,
            "events": list(session["events"]),
            "commands": sim.motion.command_count,
            "maxBoostPercent": round(float(session.get("max_boost") or 0.0), 2),
        }


# ------------------------------------------------------- скриптованная трасса
class ScriptedWorld:
    """Синтетические камеры/датчики для сухого прогона и юнит-тестов.

    Трасса: прямая → плавный левый поворот → правый поворот → СТОП (12 м) →
    кочка (22 м, для проезда нужна добавка тяги) → светофор (32 м, красный
    4 с, потом зелёный) → финиш 44 м. Конусы выдаются сразу в координатах
    робота (как после ``cone_points``), поэтому геометрия камер не участвует.
    """

    def __init__(self, scenario: str = "full") -> None:
        self.scenario = scenario if scenario in TRACK_SCENARIOS else "full"
        self.t = 0.0
        self.s = 0.0  # продольная координата
        self.y = 0.0
        self.travel = 0.0
        self.percent = 0.0
        self.pitch = 0.0
        self.speed = 0.0
        self.green_at: float | None = None
        self._ppr = 90

    def clock(self) -> float:
        return self.t

    def _center(self, distance: float) -> float:
        if self.scenario == "gate":
            return 0.40  # смещённый створ: на одном нулевом угле не проедем
        if distance < 8.0:
            return 0.0
        if distance < 16.0:
            return 0.9 * (distance - 8.0) / 8.0           # уходим влево
        if distance < 26.0:
            return 0.9 - 1.5 * (distance - 16.0) / 10.0   # и вправо
        return -0.6

    def advance(self, dt: float, percent: float, steer_left_deg: float = 0.0) -> None:
        self.percent = percent
        on_bump = (self.scenario in {"full", "bump"} and 21.8 <= self.s <= 22.0)
        # На кочке малой тяге не хватает мощности: только после добавки едем.
        self.pitch = 8.0 if on_bump else 0.0
        self.speed = min(0.33, 0.30 * (percent / 8.0)) if percent > 0 else 0.0
        if on_bump and percent < 11.0:
            self.speed = 0.0
        ds = self.speed * dt
        angle = math.radians(steer_left_deg)
        self.travel += ds
        self.s += ds * math.cos(angle)
        self.y += ds * math.sin(angle)
        self.t += dt

    def telemetry(self) -> dict[str, Any]:
        count = int(self.travel / WHEEL_CIRCUMFERENCE_M * self._ppr)
        motors = [{"id": name, "online": True, "scCount": count, "scPpr": self._ppr,
                   "scRpm": 0 if self.percent <= 0 else 60, "homed": True}
                  for name in MODULES]
        return {"linkOk": True, "motors": motors,
                "wheelSpeedMps": self.speed}

    @staticmethod
    def scan(_max_age: float) -> dict[str, Any]:
        # This object exists exclusively inside the dry-run executor. The
        # actual service never marks synthetic/API scans as trusted.
        points = [{"a": i * math.tau / 90, "r": 7.0} for i in range(90)]
        return {"trusted": True, "points": points}

    def snapshot(self, _now: float) -> dict[str, Any]:
        cones: list[dict[str, Any]] = []
        for step in range(-1, 7):
            ahead = (math.floor(self.s / 0.9) + step + 1) * 0.9
            if self.scenario == "gate" and ahead < 2.7:
                continue
            x = ahead - self.s
            lateral = self._center(ahead) - self.y
            half = 0.8
            if self.scenario == "blocked_gate" and step == 1:
                half = 0.40
            elif self.scenario in {"full", "gate"} and abs(ahead - 5.4) < 1e-6:
                half = 0.66   # real gap between the two occupied posts
            for side in (-1.0, 1.0):
                cones.append({"x": round(x, 3), "y": round(lateral + side * half, 3),
                              "distance": round(abs(x), 3), "cameraId": 0 if side > 0 else 2,
                              "confidence": 0.9})
        signs: list[dict[str, Any]] = []
        traffic: dict[str, Any] = {"visible": False, "state": "none", "hold": False,
                                   "fresh": True, "counts": {"red": 0, "yellow": 0, "green": 0}}
        ground: dict[str, Any] = {"ok": True, "depthFresh": True, "nearestBumpM": None, "status": "ok"}
        want = self.scenario
        # Знак СТОП: сначала передние камеры, у самого знака — краевая.
        if want in {"full", "stop"} and 8.0 <= self.s <= 12.6:
            ahead = 12.0 - self.s
            side = "side" if ahead <= 0.55 else "front"
            signs.append({"class": "stop_sign", "x": round(ahead, 3), "y": round(1.1 - self.y, 3),
                          "distance": round(math.hypot(ahead, 1.1 - self.y), 3),
                          "cameraId": 1 if side == "side" else 0,
                          "side": side, "signal": None, "confidence": 0.95})
        # Кочка: знак + подтверждение Kinect, когда подъехали ближе 3 м.
        if want in {"full", "bump"} and 18.5 <= self.s <= 22.8:
            ahead = 22.0 - self.s
            side = "side" if ahead <= 0.3 else "front"
            signs.append({"class": "speed_bump", "x": round(ahead, 3), "y": round(1.0 - self.y, 3),
                          "distance": round(math.hypot(ahead, 1.0 - self.y), 3),
                          "cameraId": 1 if side == "side" else 0,
                          "side": side, "signal": None, "confidence": 0.92})
            if 0.10 <= ahead <= 2.0:
                ground.update(nearestBumpM=round(ahead, 3), bumpFarM=round(ahead + 0.30, 3))
        # Светофор: красный, пока не простояли 4 с, потом зелёный.
        if want in {"full", "light"} and 28.0 <= self.s <= 33.0:
            # Красный загорается при подъезде и через 4 с ожидания сменяется
            # зелёным — проверяем именно «стоял, дождался, поехал».
            if self.green_at is None:
                self.green_at = self.t + 4.0
            state = "green" if self.green_at is not None and self.t >= self.green_at else "red"
            signs.append({"class": "traffic_light", "x": round(32.0 - self.s, 3),
                          "y": round(1.2 - self.y, 3), "distance": 1.5, "cameraId": 1, "side": "side",
                          "signal": state, "confidence": 0.94})
            traffic = {"visible": True, "state": state, "hold": state != "green",
                       "fresh": True, "cameraId": 1, "frameStamp": self.t,
                       "counts": {"red": 40 if state == "red" else 0,
                                                  "yellow": 0, "green": 40 if state == "green" else 0}}
        return {
            "ok": True, "fresh": True, "freshCameras": 4,
            "cones": cones, "signs": signs, "traffic": traffic,
            "ground": ground,
            "imu": {"fresh": True, "pitch": self.pitch, "roll": 0.0, "gyroZ": 0.0},
            "telemetry": self.telemetry(),
        }


def simulate_track(scenario: str = "full") -> dict[str, Any]:
    """Удобная точка входа для CLI и HTTP: сухой прогон без приложения."""
    return TrackAutonomyService(
        config=replace(TrackConfig(), simulation=True),
        estop=lambda: None, clock=lambda: 0.0, log_events=False,
    ).simulate(scenario)
