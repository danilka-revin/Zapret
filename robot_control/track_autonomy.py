"""Автономный проезд трассы с конусами («отдельная вкладка» бортового ПК).

Что делает сервис
-----------------
* держится по середине коридора из конусов: конусы — граница, а не цель;
  коридор может уходить и влево, и вправо, трасса 50 м и длиннее;
* знак «СТОП» виден сбоку и сверху: едем, пока его видят передние камеры,
  останавливаемся, когда знак появляется в боковых камерах (робот примерно
  напротив знака), ждём 3 с и едем дальше;
* знак «неровность» + Kinect: подтверждаем кочку по кадру глубины, заранее
  снижаем скорость, на подъёме по гироскопу/одометрии добавляем тягу порциями,
  пока робот не переедет, после кочки снова держим малую скорость и только
  потом возвращаемся к обычной;
* светофор сбоку: красный и жёлтый (и «не определён») — стоим, едем только на
  зелёный;
* потеря конусов, кадров, глубины, E-STOP, кнопка «Стоп», тайм-аут и предел
  дистанции — всегда STOP.

Это НЕ nav2 и не сертифицированная система безопасности. Тяга включается
только при ``RC_TRACK_ENABLE=1``, свежих камерах, подтверждении осмотра зоны
оператором, снятом E-STOP и готовом шасси. Все команды идут через уже
существующие ``slam.dispatch_drive_command`` / ``dispatch_steering_command``
(защёлка E-STOP, лимиты углов, watchdog прошивки сохраняются).

Факты для решений берутся из ``robot_control.vision`` — доработанного и
влитого из ``main`` / PR #158 машинного зрения (тот же публичный контракт
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
from .ground_hazards import (GroundConfig, GroundHazardTracker,
                             analyze_ground_points, ground_points_from_depth)
from .vision.classes import CLASS_INFO

#: Маркер команд вкладки (как ``exploration``, ``delivery``, ``kinect``).
AUTONOMY_HEADER = "autonomy"
MODULES = ("FL", "FR", "RL", "RR")
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
    steer_gain: float = 1.4               # усиление pure-pursuit по центру
    gyro_damping: float = 0.02            # гироскоп гасит рыскание на кочке
    lookahead_m: float = 2.5              # точка прицеливания по коридору
    min_cones: int = 2                    # минимум конусов, чтобы ехать
    min_confidence: float = 0.35          # порог уверенности детектора
    max_lateral_m: float = 4.0
    min_corridor_m: float = 0.9           # уже корпуса + запас → STOP
    max_corridor_m: float = 3.5
    max_offset_m: float = 1.5             # «не сильно далеко от конусов»
    max_nearest_m: float = 4.5            # ближайший конус слишком далеко
    lane_lost_s: float = 1.5              # сколько терпим потерю коридора
    stop_wait_s: float = 3.0              # ровно 3 с у знака СТОП
    stop_offset_m: float = 0.45           # продольный допуск «возле знака»
    stop_side_m: float = 1.8              # знак в боковой камере на таком расстоянии
    stop_pass_s: float = 1.2              # знак пропал из передних камер
    sign_cooldown_m: float = 3.0          # защита от повторного срабатывания
    bump_slow_m: float = 1.6              # начать тормозить перед кочкой
    bump_cross_m: float = 0.9             # сколько считать «переездом»
    bump_sign_m: float = 3.0              # дальность знака неровности
    bump_cross_timeout_s: float = 6.0     # запас, если нет одометрии
    stall_s: float = 0.8                  # нет движения → не хватает мощности
    pitch_deg: float = 6.0                # нос задрался — уперлись в кочку
    traffic_lost_s: float = 3.0           # светофор пропал во время ожидания
    max_distance_m: float = 200.0         # жёсткий предохранитель (50 м и более)
    max_duration_s: float = 900.0
    finish_distance_m: float = 0.0        # только сухой прогон, 0 = выключено
    cone_height_m: float = CLASS_INFO["cone"]["height_m"]
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
            tick_s=_env_float("RC_TRACK_TICK_S", 0.20, 0.05, 0.60),
            stale_s=_env_float("RC_TRACK_STALE_S", 1.5, 0.3, 10.0),
            cruise_percent=_env_float("RC_TRACK_CRUISE_PERCENT", 8.0, 0.0, 60.0),
            slow_percent=_env_float("RC_TRACK_SLOW_PERCENT", 4.0, 0.0, 40.0),
            climb_percent=_env_float("RC_TRACK_CLIMB_PERCENT", 20.0, 1.0, 100.0),
            climb_step_percent=_env_float("RC_TRACK_CLIMB_STEP_PERCENT", 3.0, 0.5, 20.0),
            post_bump_s=_env_float("RC_TRACK_POST_BUMP_S", 0.8, 0.0, 10.0),
            max_steer_deg=_env_float("RC_TRACK_MAX_STEER_DEG", 22.0, 1.0, 45.0),
            steer_deadband_deg=_env_float("RC_TRACK_STEER_DEADBAND_DEG", 2.5, 0.5, 15.0),
            steer_settle_s=_env_float("RC_TRACK_STEER_SETTLE_S", 0.45, 0.0, 3.0),
            steer_gain=_env_float("RC_TRACK_STEER_GAIN", 1.4, 0.2, 5.0),
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
            stop_wait_s=_env_float("RC_TRACK_STOP_WAIT_S", 3.0, 0.0, 30.0),
            stop_offset_m=_env_float("RC_TRACK_STOP_OFFSET_M", 0.45, 0.0, 3.0),
            stop_side_m=_env_float("RC_TRACK_STOP_SIDE_M", 1.8, 0.2, 6.0),
            stop_pass_s=_env_float("RC_TRACK_STOP_PASS_S", 1.2, 0.2, 6.0),
            sign_cooldown_m=_env_float("RC_TRACK_SIGN_COOLDOWN_M", 3.0, 0.0, 20.0),
            bump_slow_m=_env_float("RC_TRACK_BUMP_SLOW_M", 1.6, 0.2, 8.0),
            bump_cross_m=_env_float("RC_TRACK_BUMP_CROSS_M", 0.9, 0.2, 5.0),
            bump_sign_m=_env_float("RC_TRACK_BUMP_SIGN_M", 3.0, 0.5, 10.0),
            bump_cross_timeout_s=_env_float("RC_TRACK_BUMP_CROSS_TIMEOUT_S", 6.0, 1.0, 60.0),
            stall_s=_env_float("RC_TRACK_STALL_S", 0.8, 0.2, 5.0),
            pitch_deg=_env_float("RC_TRACK_PITCH_DEG", 6.0, 0.5, 45.0),
            traffic_lost_s=_env_float("RC_TRACK_TRAFFIC_LOST_S", 3.0, 0.5, 20.0),
            max_distance_m=_env_float("RC_TRACK_MAX_DISTANCE_M", 200.0, 10.0, 5000.0),
            max_duration_s=_env_float("RC_TRACK_MAX_DURATION_S", 900.0, 30.0, 7200.0),
            finish_distance_m=_env_float("RC_TRACK_FINISH_DISTANCE_M", 0.0, 0.0, 5000.0),
            require_depth=_env_bool("RC_TRACK_REQUIRE_DEPTH", True),
            require_imu=_env_bool("RC_TRACK_REQUIRE_IMU", False),
            require_blind_check=_env_bool("RC_TRACK_REQUIRE_BLIND_CHECK", True),
            side_camera_deg=_env_float("RC_TRACK_SIDE_CAMERA_DEG", 60.0, 10.0, 170.0),
            max_field_m=_env_float("RC_TRACK_MAX_FIELD_M", 6.0, 1.0, 30.0),
        )

    def public(self) -> dict[str, Any]:
        """Настройки для экрана (бсе ключи — объяснимые числа)."""
        return {key: getattr(self, key) for key in (
            "enabled", "simulation", "tick_s", "stale_s", "cruise_percent",
            "slow_percent", "climb_percent", "climb_step_percent", "post_bump_s",
            "max_steer_deg", "lookahead_m", "min_cones", "min_corridor_m",
            "max_corridor_m", "max_offset_m", "lane_lost_s", "stop_wait_s",
            "stop_offset_m", "stop_side_m", "bump_slow_m", "bump_cross_m",
            "stall_s", "pitch_deg", "max_distance_m", "max_duration_s",
            "require_depth", "require_imu", "require_blind_check", "side_camera_deg")}


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


def _camera_fresh(vision: dict[str, Any], cid: int, cfg: TrackConfig) -> dict[str, Any] | None:
    cam = ((vision.get("cameras") or {}).get(str(cid)) or {})
    age = cam.get("ageS")
    if not cam.get("width") or age is None or float(age) > cfg.stale_s:
        return None
    return cam


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
            if det.get("class") != "cone" or float(det.get("confidence") or 0) < cfg.min_confidence:
                continue
            box = det.get("box") or {}
            try:
                bw, bh = float(box["w"]), float(box["h"])
                cx = float(box["x"]) + bw / 2.0
            except (KeyError, TypeError, ValueError):
                continue
            if bh <= 0:
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
        if float(item.get("ageS") or 99.0) > cfg.stale_s:
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
        yaw = float(specs.get(cid, {}).get("yaw_deg", 0.0))
        marks.append({
            "class": item["class"], "x": round(x, 3), "y": round(y, 3),
            "distance": round(distance, 3), "cameraId": cid,
            "side": "side" if abs(yaw) >= cfg.side_camera_deg else "front",
            "yawDeg": yaw, "signal": item.get("signal"),
            "confidence": round(float(item.get("confidence") or 0), 3),
        })
    if marks:
        return marks
    # Резерв: ветка с машинным зрением может публиковать только рамки.
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
            if name not in wanted or name not in CLASS_INFO:
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
    """Едем только на зелёный; красный, жёлтый и «не определён» — стоим."""
    traffic = vision.get("trafficLight") or {}
    fresh = bool(traffic.get("fresh"))
    state = str(traffic.get("state") or "unknown") if fresh else "unknown"
    counts = traffic.get("counts") or {}
    red_pixels = int(counts.get("red") or 0)
    visible = (any(m["class"] == "traffic_light" for m in marks) or
               state != "unknown" or red_pixels > 0 or
               any(int(counts.get(name) or 0) > 0 for name in ("red", "yellow", "green")))
    # Светодиодная матрица обновляется построчно: малейший красный — стоим.
    if visible and red_pixels > 0 and state == "green":
        state = "red"
    hold = visible and state != "green"
    return {
        "visible": visible, "state": state if visible else "none",
        "fresh": fresh, "hold": hold, "confidence": traffic.get("confidence"),
        "counts": {name: int(counts.get(name) or 0) for name in ("red", "yellow", "green")},
        "cameraId": traffic.get("cameraId"),
    }


def _fit_line(points: list[dict[str, Any]]) -> tuple[float, float]:
    """y = a + b·x по МНК; одной точки достаточно (горизонтальная граница)."""
    xs = [float(p["x"]) for p in points]
    ys = [float(p["y"]) for p in points]
    n = len(xs)
    mean_x, mean_y = sum(xs) / n, sum(ys) / n
    denom = sum((x - mean_x) ** 2 for x in xs)
    b = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / denom if denom > 1e-6 else 0.0
    return mean_y - b * mean_x, _clamp(b, -2.0, 2.0)


def estimate_corridor(cones: list[dict[str, Any]], cfg: TrackConfig) -> dict[str, Any]:
    """Оценка центра коридора, его ширины и угла крабового хода.

    Границы ищем не по знаку Y (на повороте обе границы могут быть слева), а по
    самому большому разрыву между поперечными координатами ближних конусов.
    Положительный ``steerLeftDeg`` — влево; прошивке нужен угол с обратным
    знаком (см. ``TrackMotion``).
    """
    forward = [c for c in cones
               if 0.10 <= float(c["x"]) <= max(cfg.lookahead_m * 1.8,
                                                cfg.max_nearest_m * 1.5)
               and abs(float(c["y"])) <= cfg.max_lateral_m]
    near = [c for c in forward if float(c["x"]) <= cfg.lookahead_m * 1.8]
    base = {"ok": False, "count": len(near), "cones": near, "steerLeftDeg": 0.0,
            "centerM": None, "widthM": None, "headingDeg": None,
            "nearestM": None, "degraded": False, "reason": ""}
    # Отличаем «конусы уже слишком далеко» от «кадр пустой»: это разные
    # соображения безопасности, но в обоих случаях тягу не включаем.
    if forward and min(float(c["x"]) for c in forward) > cfg.max_nearest_m:
        base["nearestM"] = round(min(float(c["x"]) for c in forward), 2)
        base["reason"] = "cones_far"
        return base
    if len(near) < cfg.min_cones:
        base["reason"] = "cones_missing"
        return base
    nearest = min(float(c["x"]) for c in near)
    base["nearestM"] = round(nearest, 2)
    if nearest > cfg.max_nearest_m:
        base["reason"] = "cones_far"
        return base
    ordered = sorted(near, key=lambda c: float(c["y"]))
    split, gap = None, 0.0
    for index in range(len(ordered) - 1):
        delta = float(ordered[index + 1]["y"]) - float(ordered[index]["y"])
        if delta > gap:
            split, gap = index + 1, delta
    if split is not None and gap >= cfg.min_corridor_m * 0.55 and 0 < split < len(ordered):
        right, left = ordered[:split], ordered[split:]
    else:
        middle = sum(float(c["y"]) for c in ordered) / len(ordered)
        right = [c for c in ordered if float(c["y"]) < middle]
        left = [c for c in ordered if float(c["y"]) >= middle]
        if not right or not left:
            only = ordered
            side = 1.0 if float(only[0]["y"]) >= 0 else -1.0
            a, b = _fit_line(only)
            nominal = _clamp(cfg.min_corridor_m + 0.6, 0.6, cfg.max_corridor_m)
            center0 = a - side * nominal / 2.0
            target = center0 + b * cfg.lookahead_m
            heading = math.degrees(math.atan2(target, cfg.lookahead_m))
            steer = _clamp(heading * cfg.steer_gain, -cfg.max_steer_deg, cfg.max_steer_deg)
            base.update({"ok": True, "degraded": True, "reason": "one_side",
                         "centerM": round(center0, 3), "widthM": None,
                         "headingDeg": round(heading, 2), "steerLeftDeg": round(steer, 2),
                         "leftCount": len(left), "rightCount": len(right)})
            return base
    a_r, b_r = _fit_line(right)
    a_l, b_l = _fit_line(left)
    center0 = (a_l + a_r) / 2.0
    slope = (b_l + b_r) / 2.0
    width = a_l - a_r
    target = center0 + slope * cfg.lookahead_m
    if width < cfg.min_corridor_m:
        base.update({"reason": "corridor_narrow", "widthM": round(width, 3)})
        return base
    if abs(center0) > cfg.max_offset_m:
        base.update({"reason": "too_far_from_cones", "centerM": round(center0, 3)})
        return base
    heading = math.degrees(math.atan2(target, cfg.lookahead_m))
    steer = _clamp(heading * cfg.steer_gain, -cfg.max_steer_deg, cfg.max_steer_deg)
    base.update({
        "ok": True, "centerM": round(center0, 3), "centerAheadM": round(target, 3),
        "widthM": round(width, 3), "headingDeg": round(heading, 2),
        "slope": round(slope, 3), "steerLeftDeg": round(steer, 2),
        "leftCount": len(left), "rightCount": len(right),
        "wide": width > cfg.max_corridor_m,
    })
    return base


_EMPTY_GROUND: dict[str, Any] = {"ok": False, "nearestBumpM": None,
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
        return bool(checker(cfg.stale_s))
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
                body_pitch_deg=float(imu.get("pitch") or 0.0),
                body_roll_deg=float(imu.get("roll") or 0.0))
            analysis = analyze_ground_points(points, GroundConfig())
            tracked = self.tracker.update(analysis, max(0.001, float(stamp)))
        except Exception as exc:  # noqa: BLE001 — битый кадр = «данных нет»
            return dict(_EMPTY_GROUND, reason=str(exc)[:120])
        bumps = [item for item in tracked.get("hazards") or [] if item.get("kind") == "bump"]
        nearest = min((float(item["distanceM"]) for item in bumps), default=None)
        result = {
            "ok": bool(tracked.get("status") == "ok"),
            "nearestBumpM": round(nearest, 3) if nearest is not None else None,
            "status": tracked.get("status"), "reason": tracked.get("reason") or "",
            "hazards": bumps[:4], "validPoints": tracked.get("validPoints", 0),
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
    fresh = bool(state.get("ok")) and not state.get("stale")

    def num(key: str) -> float | None:
        value = state.get(key)
        return float(value) if isinstance(value, (int, float)) and math.isfinite(float(value)) else None

    return {"fresh": fresh, "pitch": num("pitch") or 0.0, "roll": num("roll") or 0.0,
            "gyroZ": num("gyroZ") or 0.0, "heading": num("heading"), "ageMs": state.get("ageMs")}


# ---------------------------------------------------------------------- привод
class TrackMotion:
    """Единый владелец тяги и руля внутри режима.

    Ограничения прошивки (см. rus_bridge.py): руль перекладывается только на
    стоящем роботе, углы всех четырёх колёс одинаковы (краб), команду тяги надо
    повторять чаще 650 мс. Положительный угол прошивки — вправо, а положительная
    Y у нас влево, поэтому знак меняем здесь и только здесь.
    """

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
        self.commands: list[dict[str, Any]] = []
        self.failures = 0

    def _send_drive(self, percent: float) -> None:
        if self.simulation:
            self.commands.append({"kind": "drive", "percent": round(percent, 2)})
            return
        result, _status = self._drive(self._source(), {"percent": round(percent, 2)})
        if not isinstance(result, dict) or not result.get("ok"):
            self.failures += 1
            raise TrackAutonomyError("drive_refused", "Шасси отклонило тягу: %s" %
                                     str((result or {}).get("reason") or "нет ответа"), 503)
        self.commands.append({"kind": "drive", "percent": round(percent, 2)})

    def _send_steer(self, target: float) -> None:
        if self.simulation:
            self.commands.append({"kind": "steer", "angle": round(target, 2)})
            return
        source = self._source()
        for module in MODULES:
            result, _status = self._steer(source, {"module": module, "angle": round(target, 1)})
            if not isinstance(result, dict) or not result.get("ok"):
                self.failures += 1
                raise TrackAutonomyError("steer_refused", "Руль %s отклонил угол: %s" %
                                         (module, str((result or {}).get("reason") or "нет ответа")), 503)

    def command(self, percent: float, steer_left_deg: float) -> bool:
        """Вернуть True, если тяга реально подана в этом такте."""
        target = _clamp(-float(steer_left_deg), -self.cfg.max_steer_deg, self.cfg.max_steer_deg)
        if abs(target - self.steer_deg) > self.cfg.steer_deadband_deg:
            self._send_drive(0)  # сначала тормоз: руль в движении не переводится
            self._send_steer(target)
            self.steer_deg = target
            self.percent = 0.0
            self.settle_until = self.clock() + self.cfg.steer_settle_s
            return False
        if self.clock() < self.settle_until:
            self._send_drive(0)
            return False
        self._send_drive(percent)
        self.percent = percent
        return True

    def stop(self) -> None:
        self._send_drive(0)
        self.percent = 0.0
        self.settle_until = 0.0


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

    def update(self, dt: float) -> float:
        telemetry = self._read()
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
                step = value - self._last.get(name, value)
                if step < 0:  # переполнение счётчика — не будем «прыгать» вперёд
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
        telemetry = self._read()
        motors = telemetry.get("motors") or []
        return {"linkOk": bool(telemetry.get("linkOk")),
                "onlineWheels": sum(1 for m in motors if isinstance(m, dict) and m.get("online")),
                "wheelSpeedMps": telemetry.get("wheelSpeedMps")}


# --------------------------------------------------------------------- сервис
class TrackAutonomyService:
    """Фоновый цикл трассы + статус/предполётные проверки для вкладки."""

    def __init__(self, *, config: Optional[TrackConfig] = None,
                 vision_provider: Optional[Callable[[], Any]] = None,
                 layout_provider: Optional[Callable[[], Any]] = None,
                 kinect_provider: Optional[Callable[[], Any]] = None,
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
        self._cancel = threading.Event()
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
        fresh_cameras = sum(1 for cid in range(4) if _camera_fresh(vision, cid, self.cfg) is not None)
        cones = cone_points(vision, layout, self.cfg)
        corridor = estimate_corridor(cones, self.cfg)
        estop_clear = not self._is_estopped()
        vision_ok = bool(vision.get("ok") and vision.get("enabled"))
        checks.update({
            "enabled": self.cfg.enabled,
            "estopClear": estop_clear,
            "visionOk": vision_ok,
            "visionRunning": bool(vision.get("running")),
            "camerasFresh": fresh_cameras >= 2,
            "conesVisible": len(cones) >= self.cfg.min_cones,
            "corridorOk": bool(corridor.get("ok")),
            "geometryCalibrated": bool(layout.get("calibrated")),
            "driveReady": bool(getattr(source, "supports_drive", False)),
            "steerReady": bool(getattr(source, "supports_individual_steer", False)),
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
        try:
            rivals = [str(item) for item in (self._rivals() or []) if item]
            checks["rivalsIdle"] = not rivals
        except Exception:  # noqa: BLE001
            rivals = []
        telemetry = self._odometer.telemetry_snapshot()
        online_wheels = int(telemetry.get("onlineWheels") or 0)
        checks["chassisLink"] = bool(telemetry.get("linkOk"))
        checks["wheelsOnline"] = online_wheels >= len(MODULES)
        checks["standstill"] = not (float(telemetry.get("wheelSpeedMps") or 0) > 0.05)

        if not checks["enabled"]:
            blockers.append("Режим выключен: задайте RC_TRACK_ENABLE=1 в .env и перезапустите киоск")
        if not checks["estopClear"]:
            blockers.append("Взведён E-STOP: снимите аварийную остановку")
        if not checks["visionOk"] or not checks["visionRunning"]:
            blockers.append("Машинное зрение не работает: проверьте /api/vision/state и камеры")
        if not checks["camerasFresh"]:
            blockers.append("Нет свежих кадров минимум с двух камер (%.1f с)" % self.cfg.stale_s)
        if not checks["conesVisible"]:
            blockers.append("Конусы не видны: поставьте робота в коридор и проверьте разметку кадров")
        if not checks["corridorOk"]:
            blockers.append("Коридор не распознан: %s" % (corridor.get("reason") or "мало конусов"))
        if rivals:
            blockers.append("Другой режим уже владеет приводом: " + ", ".join(rivals))
        if not checks["standstill"]:
            blockers.append("Робот ещё катится: дождитесь полной остановки")
        if self.cfg.require_depth and not checks["depthFresh"]:
            blockers.append("Нет свежего навигационного кадра Kinect 360: "
                            "кочку подтверждать нечем (preview не для навигации)")
        if self.cfg.require_imu and not checks["imuFresh"]:
            blockers.append("Нет свежего гироскопа/наклона: подъём на кочке не контролируется")
        if not self.cfg.simulation:
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
        if self.cfg.require_blind_check and not checks["blindZone"]:
            if for_start:
                blockers.append("Осмотрите зону вокруг робота и поставьте галочку подтверждения")
            else:
                warnings.append("Перед пуском осмотрите зону и поставьте галочку подтверждения")
        if not checks["geometryCalibrated"]:
            warnings.append("Геометрия камер не откалибрована: углы взяты по умолчанию")
        if self.cfg.simulation:
            warnings.append("RC_TRACK_SIMULATION=1: камеры читаются, команды тяге НЕ отправляются")
        if not checks["depthFresh"]:
            warnings.append("Нет полного кадра Kinect: перед кочкой режим встанет, "
                            "используйте raw depth, а не points/ranges")
        if not checks["imuFresh"]:
            warnings.append("Гироскоп без свежих данных: добавка тяги будет только по одометрии")
        if corridor.get("degraded"):
            warnings.append("Видна только одна граница конусов: держимся на заданной полуширине")
        if self.active:
            blockers.append("Автономность уже идёт")
        return {
            "ok": True, "blockers": blockers, "warnings": warnings,
            "checks": checks, "freshCameras": fresh_cameras,
            "conesVisible": len(cones), "canStart": not blockers,
            "simulation": self.cfg.simulation,
            "requireBlindCheck": self.cfg.require_blind_check,
            "forStart": for_start,
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
                      "commands": len(self.motion.commands), "failures": self.motion.failures,
                      "simulated": self.motion.simulation},
            "odometry": {"source": self._odometer.source, "distanceM": round(self._odometer.distance, 3)},
        }
        payload.update(state)
        payload.update(preflight)
        return payload

    # -------------------------------------------------------------- пуск/стоп
    def _new_session(self, now: float) -> dict[str, Any]:
        return {
            "started": now, "last": now, "finished": False,
            "lane_lost_since": None, "vision_lost_since": None,
            "depth_lost_since": None,
            "stop_armed": False, "stop_wait_until": None, "stop_last_seen": None,
            "stop_last_distance": None, "stop_consumed_until": 0.0,
            "traffic_waiting": False, "traffic_since": None, "traffic_visible_at": None,
            "traffic_consumed_until": 0.0,
            "bump_armed": False, "bump_active": False, "bump_cross_start": 0.0,
            "bump_cross_since": None, "bump_post_until": 0.0, "bump_consumed_until": 0.0,
            "boost": 0.0, "max_boost": 0.0, "stall_since": None,
            "events": [],
        }

    def start(self, body: Any = None) -> tuple[dict[str, Any], int]:
        payload = body if isinstance(body, dict) else {}
        blind = payload.get("blindZoneChecked") is True
        checks = self.preflight(blind_zone_checked=blind, for_start=True)
        if checks["blockers"]:
            raise TrackAutonomyError("not_ready", "; ".join(checks["blockers"]), 409)
        with self._lock:
            if self.active:
                raise TrackAutonomyError("already_running", "Автономность уже идёт", 409)
            try:
                self._preempt()
            except Exception:  # noqa: BLE001 — отзыв чужого режима не должен ломать старт
                pass
            self.motion.stop()
            self._bump_detector.reset()
            self._odometer = Odometer(self._telemetry)
            self._cancel = threading.Event()
            self._session = self._new_session(self.clock())
            self._state = {
                "phase": "starting", "message": "Проверка камер, конусов и датчиков…",
                "distanceM": 0.0, "percent": 0.0, "steerLeftDeg": 0.0,
                "boostPercent": 0.0, "events": [],
                "counters": {"stop": 0, "bump": 0, "traffic": 0, "powerBoosts": 0},
                "perception": {}, "corridor": {}, "traffic": {}, "ground": {},
                "imu": {}, "telemetry": {}, "signs": [], "cones": [],
            }
            self._event(self._session, "starting", "Автономность запущена оператором", "ok")
            worker = threading.Thread(target=self._run, args=(self._cancel,),
                                      name="robot-track-autonomy", daemon=True)
            self._worker = worker
            worker.start()
        return {"ok": True, "status": self.status()}, 202

    def stop(self, reason: str = "Остановлено оператором") -> dict[str, Any]:
        with self._lock:
            self._cancel.set()
            session = self._session
        try:
            self.motion.stop()
        except TrackAutonomyError:
            pass
        if session is not None:
            self._finish(session, "cancelled", reason, "warn")
        return {"ok": True, "status": self.status()}

    # ------------------------------------------------------------- восприятие
    def _perceive(self, now: float) -> dict[str, Any]:
        if self._perception is not None:
            data = self._perception(now) or {}
            result = {
                "ok": bool(data.get("ok", True)), "fresh": bool(data.get("fresh", True)),
                "freshCameras": int(data.get("freshCameras") or 4),
                "cones": list(data.get("cones") or []),
                "signs": list(data.get("signs") or []),
                "traffic": dict(data.get("traffic") or {"visible": False, "state": "none", "hold": False}),
                "ground": dict(data.get("ground") or {"ok": False, "nearestBumpM": None}),
                "imu": dict(data.get("imu") or {}),
                "telemetry": dict(data.get("telemetry") or {}),
                "vision": {},
            }
            return result
        vision = self._vision_snapshot()
        layout = self._layout_snapshot()
        cones = cone_points(vision, layout, self.cfg)
        signs = sign_marks(vision, layout, self.cfg)
        traffic = traffic_decision(vision, signs, self.cfg)
        imu = imu_snapshot(self._imu())
        ground = self._bump_detector.update(self._kinect(), imu, now, self.cfg)
        fresh_cameras = sum(1 for cid in range(4) if _camera_fresh(vision, cid, self.cfg) is not None)
        return {
            "ok": bool(vision.get("ok") and vision.get("enabled")),
            "fresh": fresh_cameras >= 2, "freshCameras": fresh_cameras,
            "cones": cones, "signs": signs, "traffic": traffic,
            "ground": ground, "imu": imu,
            "telemetry": self._odometer.telemetry_snapshot(), "vision": vision,
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
        if session["finished"]:
            return
        session["finished"] = True
        try:
            self.motion.stop()
        except TrackAutonomyError:
            pass
        self._event(session, phase, message, tone)
        self._set(phase=phase, message=message, percent=0.0)

    # --------------------------------------------------------------- один такт
    def _step(self, now: float) -> None:
        session = self._session
        if session is None or session["finished"]:
            return
        dt = max(0.0, min(1.0, now - session["last"]))
        session["last"] = now
        if self._is_estopped():
            self._finish(session, "error", "E-STOP взведён — тяга снята.", "err")
            return
        if now - session["started"] > self.cfg.max_duration_s:
            self._finish(session, "done", "Достигнут предел времени трассы.", "ok")
            return
        delta = self._odometer.update(dt)
        distance = self._odometer.distance
        if distance >= self.cfg.max_distance_m:
            self._finish(session, "done", "Достигнут предел дистанции %.0f м — стою." %
                         self.cfg.max_distance_m, "ok")
            return
        if self.cfg.finish_distance_m > 0 and distance >= self.cfg.finish_distance_m:
            self._finish(session, "done", "Трасса пройдена: %.1f м." % distance, "ok")
            return
        snap = self._perceive(now)
        cones, signs = snap["cones"], snap["signs"]
        traffic, ground, imu = snap["traffic"], snap["ground"], snap["imu"]
        corridor = estimate_corridor(cones, self.cfg)

        self._set(distanceM=round(distance, 3), perception={
            "ok": snap["ok"], "freshCameras": snap["freshCameras"],
            "cones": len(cones), "signs": len(signs),
        }, cones=cones[:24], signs=signs[:12], corridor=corridor,
            traffic=traffic, ground={"ok": ground.get("ok"),
                                     "nearestBumpM": ground.get("nearestBumpM"),
                                     "status": ground.get("status"),
                                     "reason": ground.get("reason")},
            imu={key: imu.get(key) for key in ("fresh", "pitch", "roll", "gyroZ")},
            telemetry=snap["telemetry"])

        # 1. Кадры и конусы: без коридора робот не должен «угадывать» дорогу.
        if not snap["ok"] or not snap["fresh"]:
            if session["vision_lost_since"] is None:
                session["vision_lost_since"] = now
            if now - session["vision_lost_since"] > self.cfg.lane_lost_s:
                self._finish(session, "error", "Кадры камер устарели — стою, коридор не виден.", "err")
                return
            self.motion.stop()
            self._set(phase="lane_lost", message="Нет свежих кадров — жду камеры.", percent=0.0)
            return
        session["vision_lost_since"] = None
        if not corridor["ok"]:
            if session["lane_lost_since"] is None:
                session["lane_lost_since"] = now
            if now - session["lane_lost_since"] > self.cfg.lane_lost_s:
                self._finish(session, "error",
                             "Коридор из конусов потерян (%s) — стою." %
                             (corridor.get("reason") or "причина не указана"), "err")
                return
            self.motion.stop()
            self._set(phase="lane_lost", message="Коридор не виден (%s), жду конусы." %
                      (corridor.get("reason") or "мало конусов"), percent=0.0)
            return
        session["lane_lost_since"] = None

        # 2. Знак СТОП: едем, пока видят передние; стоп — когда знак с краю.
        stops = [m for m in signs if m["class"] == "stop_sign"]
        front_stops = [m for m in stops if m["side"] == "front"]
        side_stops = [m for m in stops if m["side"] == "side"]
        cooldown_ok = distance >= session["stop_consumed_until"]
        if stops:
            session["stop_last_seen"] = now
            session["stop_last_distance"] = min(float(m["distance"]) for m in stops)
            if front_stops and cooldown_ok:
                session["stop_armed"] = True
        stop_now = bool(session["stop_armed"] and cooldown_ok and (
            any(float(m["distance"]) <= self.cfg.stop_side_m for m in side_stops) or
            any(float(m["x"]) <= self.cfg.stop_offset_m for m in stops) or
            (not front_stops and session["stop_last_seen"] is not None and
             now - session["stop_last_seen"] <= self.cfg.stop_pass_s and
             (session["stop_last_distance"] or 99) <= 2.5)))
        if stop_now and session["stop_wait_until"] is None:
            session["stop_wait_until"] = now + self.cfg.stop_wait_s
            session["stop_consumed_until"] = distance + self.cfg.sign_cooldown_m
            session["stop_armed"] = False
            session["stop_count"] = session.get("stop_count", 0) + 1
            counters = dict(self._state.get("counters") or {})
            counters["stop"] = counters.get("stop", 0) + 1
            self._set(counters=counters)
            self._event(session, "stop_wait", "Знак СТОП: остановился возле знака, жду %.0f с." %
                        self.cfg.stop_wait_s, "stop")
        if session["stop_wait_until"] is not None:
            if now < session["stop_wait_until"]:
                self.motion.stop()
                self._set(phase="stop_wait", percent=0.0,
                          message="СТОП: жду %.1f с." % (session["stop_wait_until"] - now))
                return
            session["stop_wait_until"] = None
            self._event(session, "following", "СТОП выдержан 3 с — продолжаю трассу.", "ok")

        # 3. Светофор сбоку: красный/жёлтый/неизвестный — стоим, зелёный — едем.
        light_visible = bool(traffic.get("visible"))
        if light_visible:
            session["traffic_visible_at"] = now
        gate_ok = distance >= session["traffic_consumed_until"]
        if (gate_ok and light_visible and traffic.get("hold") and
                not session["traffic_waiting"]):
            session["traffic_waiting"] = True
            session["traffic_since"] = now
            counters = dict(self._state.get("counters") or {})
            counters["traffic"] = counters.get("traffic", 0) + 1
            self._set(counters=counters)
            self._event(session, "traffic_wait", "Светофор: %s — стою, жду зелёный." %
                        str(traffic.get("state") or "не определён").upper(), "stop")
        if session["traffic_waiting"]:
            if light_visible and not traffic.get("hold") and traffic.get("fresh"):
                session["traffic_waiting"] = False
                session["traffic_consumed_until"] = distance + self.cfg.sign_cooldown_m
                self._event(session, "following", "Светофор: зелёный — еду.", "ok")
            elif (not light_visible and session["traffic_visible_at"] is not None and
                  now - session["traffic_visible_at"] > self.cfg.traffic_lost_s):
                self._finish(session, "error", "Светофор потерян во время ожидания — стою.", "err")
                return
            else:
                self.motion.stop()
                self._set(phase="traffic_wait", percent=0.0,
                          message="Светофор %s — стою до зелёного." %
                          str(traffic.get("state") or "не определён").upper())
                return

        # 4. Кочка: знак + Kinect, малая скорость, добавка тяги по гироскопу/SC.
        bump_signs = [m for m in signs if m["class"] == "speed_bump"]
        distances = [float(m["x"]) for m in bump_signs if float(m["x"]) > 0]
        distances += [float(m["distance"]) for m in bump_signs if
                      m["side"] == "side" and float(m["x"]) <= 0]
        if ground.get("nearestBumpM") is not None:
            distances.append(float(ground["nearestBumpM"]))
        bump_distance = min(distances) if distances else None
        bump_cooldown = distance >= session["bump_consumed_until"]
        if bump_signs and bump_cooldown and bump_distance is not None and \
                bump_distance <= self.cfg.bump_sign_m:
            session["bump_armed"] = True
        if ground.get("ok"):
            session["depth_lost_since"] = None
        else:
            if session["depth_lost_since"] is None:
                session["depth_lost_since"] = now
            if (self.cfg.require_depth and session["bump_armed"] and
                    now - session["depth_lost_since"] > self.cfg.lane_lost_s):
                self._finish(session, "error", "Kinect потерян перед кочкой — стою.", "err")
                return

        phase, speed, message = "following", self.cfg.cruise_percent, "Держу середину коридора."
        if session["bump_armed"] and bump_distance is not None and \
                bump_distance <= self.cfg.bump_slow_m:
            phase, speed, message = "bump_slow", self.cfg.slow_percent, "Кочка впереди — снижаю скорость."
            if not session["bump_active"] and (bump_distance <= self.cfg.bump_cross_m or
                                               any(float(m["x"]) <= 0.2 for m in bump_signs)):
                session["bump_active"] = True
                session["bump_cross_start"] = distance
                session["bump_cross_since"] = now
                session["boost"] = 0.0
                counters = dict(self._state.get("counters") or {})
                counters["bump"] = counters.get("bump", 0) + 1
                self._set(counters=counters)
                self._event(session, "bump_climb", "Переезжаю кочку на малой скорости.", "warn")
        if session["bump_active"]:
            crossed = (distance - session["bump_cross_start"] >= self.cfg.bump_cross_m or
                       (self._odometer.source == "none" and session["bump_cross_since"] is not None and
                        now - session["bump_cross_since"] >= self.cfg.bump_cross_timeout_s))
            if crossed:
                session["bump_active"] = False
                session["bump_armed"] = False
                session["bump_post_until"] = now + self.cfg.post_bump_s
                session["bump_consumed_until"] = distance + self.cfg.sign_cooldown_m
                session["boost"] = 0.0
                session["stall_since"] = None
                # В этот же такт уже не «подъём»: сбрасываем добавку и держим
                # малую скорость заданное время, и только потом cruise_percent.
                phase, speed = "bump_exit", self.cfg.slow_percent
                message = "Кочка пройдена — снова держу малую скорость."
                self._event(session, "bump_exit", message, "ok")
            else:
                phase = "bump_climb"
                incline = abs(float(imu.get("pitch") or 0.0)) >= self.cfg.pitch_deg
                no_progress = delta <= 0.002
                if no_progress:
                    if session["stall_since"] is None:
                        session["stall_since"] = now
                else:
                    session["stall_since"] = None
                stuck = (session["stall_since"] is not None and
                         now - session["stall_since"] >= self.cfg.stall_s) or (incline and no_progress)
                if stuck:
                    room = self.cfg.climb_percent - (self.cfg.slow_percent + session["boost"])
                    if room > 0:
                        added = min(self.cfg.climb_step_percent, room)
                        session["boost"] += added
                        session["max_boost"] = max(session["max_boost"], session["boost"])
                        counters = dict(self._state.get("counters") or {})
                        counters["powerBoosts"] = counters.get("powerBoosts", 0) + 1
                        self._set(counters=counters)
                        self._event(session, "bump_climb",
                                    "Не хватает мощности забраться (наклон %.1f°): тяга +%.1f %% → %.1f %%." %
                                    (float(imu.get("pitch") or 0.0), added,
                                     self.cfg.slow_percent + session["boost"]), "warn")
                    session["stall_since"] = now
                speed = self.cfg.slow_percent + session["boost"]
                message = "Кочка: добавляю тягу, пока не перееду (%.1f %%)." % speed
        elif now < session["bump_post_until"]:
            phase, speed = "bump_exit", self.cfg.slow_percent
            message = "После кочки держу малую скорость %.1f с." % max(0.0, session["bump_post_until"] - now)

        # 5. Крабовый угол к центру + небольшое демпфирование гироскопом.
        steer_left = float(corridor.get("steerLeftDeg") or 0.0)
        if imu.get("fresh"):
            steer_left -= float(imu.get("gyroZ") or 0.0) * self.cfg.gyro_damping
        steer_left = _clamp(steer_left, -self.cfg.max_steer_deg, self.cfg.max_steer_deg)
        try:
            applied = self.motion.command(speed, steer_left)
        except TrackAutonomyError as exc:
            self._finish(session, "error", "Привод отказал: %s" % exc.message, "err")
            return
        self._set(phase=phase, message=message,
                  percent=round(speed if applied else 0.0, 2),
                  steerLeftDeg=round(steer_left, 2),
                  boostPercent=round(session["boost"], 2))

    def _run(self, cancel: threading.Event) -> None:
        session = self._session
        try:
            while not cancel.is_set() and session is not None and not session["finished"]:
                self._step(self.clock())
                cancel.wait(self.cfg.tick_s)
        except Exception as exc:  # noqa: BLE001 — поток обязан завершиться с понятным статусом
            if session is not None:
                self._finish(session, "error", "Сбой автономности: %s" % str(exc)[:160], "err")
        finally:
            try:
                self.motion.stop()
            except TrackAutonomyError:
                pass
            if cancel.is_set() and session is not None and not session["finished"]:
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
        config = replace(self.cfg, enabled=True, simulation=True,
                         require_depth=False, require_imu=False,
                         require_blind_check=False,
                         finish_distance_m=44.0,
                         max_duration_s=240.0, max_distance_m=200.0)
        sim = TrackAutonomyService(
            config=config, perception_provider=world.snapshot,
            telemetry_provider=world.telemetry,
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
            world.advance(config.tick_s, sim.motion.percent)
            steps += 1
        session = sim._session
        counters = dict(sim._state.get("counters") or {})
        return {
            "ok": True, "scenario": world.scenario, "steps": steps,
            "phase": sim._state.get("phase"),
            "message": sim._state.get("message"),
            "distanceM": round(sim._odometer.distance, 2),
            "elapsedS": round(world.clock() - session["started"], 2),
            "counters": counters,
            "events": list(session["events"]),
            "commands": len(sim.motion.commands),
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
        self.scenario = scenario if scenario in {"full", "straight", "stop", "bump", "light"} else "full"
        self.t = 0.0
        self.s = 0.0
        self.percent = 0.0
        self.pitch = 0.0
        self.green_at: float | None = None
        self._ppr = 90

    def clock(self) -> float:
        return self.t

    def _center(self, distance: float) -> float:
        if distance < 8.0:
            return 0.0
        if distance < 16.0:
            return 0.9 * (distance - 8.0) / 8.0           # уходим влево
        if distance < 26.0:
            return 0.9 - 1.5 * (distance - 16.0) / 10.0   # и вправо
        return -0.6

    def advance(self, dt: float, percent: float) -> None:
        self.percent = percent
        on_bump = (self.scenario in {"full", "bump"} and 21.8 <= self.s <= 22.0)
        # На кочке малой тяге не хватает мощности: только после добавки едем.
        self.pitch = 8.0 if on_bump else 0.0
        speed = 0.60 * (percent / 8.0) if percent > 0 else 0.0
        if on_bump and percent < 11.0:
            speed = 0.0
        self.s += speed * dt
        self.t += dt

    def telemetry(self) -> dict[str, Any]:
        count = int(self.s / WHEEL_CIRCUMFERENCE_M * self._ppr)
        motors = [{"id": name, "online": True, "scCount": count, "scPpr": self._ppr,
                   "scRpm": 0 if self.percent <= 0 else 60, "homed": True}
                  for name in MODULES]
        return {"linkOk": True, "motors": motors,
                "wheelSpeedMps": 0.42 * self.percent / 8.0 if self.percent > 0 else 0.0}

    def snapshot(self, _now: float) -> dict[str, Any]:
        center = self._center(self.s)
        cones: list[dict[str, Any]] = []
        for step in range(1, 8):
            ahead = self.s + step * 0.9
            lateral = self._center(ahead) - center
            for side in (-1.0, 1.0):
                cones.append({"x": round(step * 0.9, 3),
                              "y": round(lateral + side * 0.8, 3),
                              "distance": round(step * 0.9, 3),
                              "cameraId": 0 if side > 0 else 2,
                              "confidence": 0.9})
        signs: list[dict[str, Any]] = []
        traffic: dict[str, Any] = {"visible": False, "state": "none", "hold": False,
                                   "fresh": True, "counts": {"red": 0, "yellow": 0, "green": 0}}
        ground: dict[str, Any] = {"ok": False, "nearestBumpM": None, "status": "unknown"}
        want = self.scenario
        # Знак СТОП: сначала передние камеры, у самого знака — краевая.
        if want in {"full", "stop"} and 8.0 <= self.s <= 12.6:
            ahead = 12.0 - self.s
            side = "side" if ahead <= 0.55 else "front"
            signs.append({"class": "stop_sign", "x": round(ahead, 3), "y": 1.1,
                          "distance": round(math.hypot(ahead, 1.1), 3),
                          "cameraId": 1 if side == "side" else 0,
                          "side": side, "signal": None, "confidence": 0.95})
        # Кочка: знак + подтверждение Kinect, когда подъехали ближе 3 м.
        if want in {"full", "bump"} and 18.5 <= self.s <= 22.8:
            ahead = 22.0 - self.s
            side = "side" if ahead <= 0.3 else "front"
            signs.append({"class": "speed_bump", "x": round(ahead, 3), "y": 1.0,
                          "distance": round(math.hypot(ahead, 1.0), 3),
                          "cameraId": 1 if side == "side" else 0,
                          "side": side, "signal": None, "confidence": 0.92})
            if ahead <= 2.0:
                ground = {"ok": True, "nearestBumpM": round(max(0.05, ahead), 3),
                          "status": "ok", "hazards": [{"kind": "bump", "distanceM": max(0.05, ahead)}]}
        # Светофор: красный, пока не простояли 4 с, потом зелёный.
        if want in {"full", "light"} and 28.0 <= self.s <= 33.0:
            # Красный загорается при подъезде и через 4 с ожидания сменяется
            # зелёным — проверяем именно «стоял, дождался, поехал».
            if self.green_at is None:
                self.green_at = self.t + 4.0
            state = "green" if self.green_at is not None and self.t >= self.green_at else "red"
            signs.append({"class": "traffic_light", "x": round(max(0.0, 32.0 - self.s), 3),
                          "y": 1.2, "distance": 1.5, "cameraId": 1, "side": "side",
                          "signal": state, "confidence": 0.94})
            traffic = {"visible": True, "state": state, "hold": state != "green",
                       "fresh": True, "counts": {"red": 40 if state == "red" else 0,
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
