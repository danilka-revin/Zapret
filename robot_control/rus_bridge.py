"""Мост: команды шасси из ``rus_core`` → мастер-Arduino Zapret (``MasterSource``).

Нижний уровень — ``slam_gui/backend.py``: мастер по USB-UART переводит ``w FL <deg>``
(поворот одного модуля) и ``t N`` (тяга всех колёс, ±20 %) в I²C-модули
``motor_node``. Ограничения прошивки, которые мост обязан соблюдать:

* тяга одна на все четыре колеса: углы и скорости должны совпадать; разворот
  на месте и поворот с разными углами колёс на этом железе не выполняются;
* руль в движении не переводится (``module_busy``): перед поворотом колёс
  тяга снимается тормозом, колёса доходят до цели, и только потом снова тяга;
* угол модуля ограничен ±100° (``STEER_LIMIT_DEG`` — та же константа, что в
  прошивке и в ``MasterSource.MAX_STEER_DEG``): крабовый ход на 45° и чистый
  боковой ход на 90° доступны, больше — отказ ``steer_limit``;
* heartbeat тяги не реже 650 мс: пока едем, команду ``t N`` надо повторять;
* скорость в м/с переводится в проценты по калибровке ``mps_per_percent``.
  Без калибровки мост тягу не даёт — честнее стоять, чем ехать с выдуманной скоростью.

Мост не знает про HTTP и потоки: ``MasterBridge.step(vx, vy, wz)`` — один шаг
управления, его вызывает тот, кто решает, куда ехать (пульт, миссия, тест).
E-STOP проверяется на каждом шаге через ``robot_control.estop``.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional

from robot_control import estop as estop_latch
from robot_control.rus_core.kinematics import SwerveKinematics

#: Имена модулей, как в прошивке мастера (``MODULE_TITLES`` в backend.py).
MODULE_NAMES = {1: "FL", 2: "FR", 3: "RL", 4: "RR"}

#: Предел абсолютного угла руля. Ровно столько же в прошивке
#: (``STEER_LIMIT_DEG``) и в ``slam_gui/backend.py``: мост не должен обещать
#: угол, который плата отрежет.
STEER_LIMIT_DEG = 100.0

#: Допуск, при котором углы модулей считаются одинаковыми (одна тяга на все колёса).
SAME_ANGLE_DEG = 0.5
#: Порог, с которого новый угол руля отправляется мастеру.
STEER_SEND_DEG = 0.5


@dataclass
class Plan:
    """Результат расчёта по скорости корпуса (без обращения к железу)."""
    ok: bool
    reason: str = ""                         # причина отказа, пусто при ok
    steer_deg: Dict[str, float] = field(default_factory=dict)
    percent: int = 0                         # тяга, %; 0 — стоянка


def plan_motion(kin: SwerveKinematics, vx: float, vy: float, wz: float, *,
                mps_per_percent: Optional[float],
                max_steer_deg: float = STEER_LIMIT_DEG,
                max_drive_percent: int = 20) -> Plan:
    """Перевести скорость корпуса в углы руля и тягу, которые умеет прошивка.

    Угол модуля сворачивается в диапазон ±100°: если угол больше, колесо
    разворачивается на 180°, а скорость меняет знак (вектор тот же).
    """
    cmds = kin.inverse(vx, vy, wz)
    if not any(c.enable for c in cmds):
        return Plan(ok=True, percent=0)

    folded = []
    for c in cmds:
        angle, speed = c.angle_rad, c.speed_mps
        if angle > math.pi / 2:
            angle, speed = angle - math.pi, -speed
        elif angle < -math.pi / 2:
            angle, speed = angle + math.pi, -speed
        folded.append((MODULE_NAMES[c.module_id], math.degrees(angle), speed))

    angles = [a for _, a, _ in folded]
    speeds = [s for _, _, s in folded]
    if max(angles) - min(angles) > SAME_ANGLE_DEG:
        return Plan(ok=False, reason="unsupported_motion")
    scale = max(1.0, max(abs(s) for s in speeds))
    if max(speeds) - min(speeds) > 1e-6 * scale:
        return Plan(ok=False, reason="unsupported_motion")

    common_deg = sum(angles) / len(angles)
    if abs(common_deg) > max_steer_deg:
        return Plan(ok=False, reason="steer_limit")
    if mps_per_percent is None or mps_per_percent <= 0:
        return Plan(ok=False, reason="speed_not_calibrated")

    percent = int(round((sum(speeds) / len(speeds)) / mps_per_percent))
    if abs(percent) > max_drive_percent:
        return Plan(ok=False, reason="drive_limit")
    steer = {name: round(common_deg, 1) for name, _, _ in folded}
    return Plan(ok=True, steer_deg=steer, percent=percent)


class MasterBridge:
    """Один шаг управления шасси через мастер-Arduino.

    Состояние: последний отправленный угол руля, включена ли тяга, момент начала
    выравнивания. Все ответы — словари с ``ok``, ``phase`` и ``reason``.
    """

    def __init__(self, source: Any, kin: Optional[SwerveKinematics] = None, *,
                 mps_per_percent: Optional[float] = None,
                 max_steer_deg: float = STEER_LIMIT_DEG,
                 align_deg: float = 2.5,
                 align_timeout_s: float = 3.0,
                 max_drive_percent: int = 20,
                 estop_check: Callable[[], Any] = estop_latch.latched):
        self.source = source
        self.kin = kin or SwerveKinematics()
        self.mps_per_percent = mps_per_percent
        self.max_steer_deg = max_steer_deg
        self.align_deg = align_deg
        self.align_timeout_s = align_timeout_s
        self.max_drive_percent = max_drive_percent
        self.estop_check = estop_check
        self.steer_target: Dict[str, float] = {}
        self.percent = 0
        self._align_since: Optional[float] = None
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ #
    def step(self, vx: float, vy: float, wz: float, now: Optional[float] = None) -> Dict[str, Any]:
        """Один шаг: проверить E-STOP, посчитать план и продвинуть состояние."""
        now = time.time() if now is None else now
        with self._lock:
            if self.estop_check() is not None:
                return self._stop("estop", now, force=True)

            plan = plan_motion(self.kin, vx, vy, wz,
                               mps_per_percent=self.mps_per_percent,
                               max_steer_deg=self.max_steer_deg,
                               max_drive_percent=self.max_drive_percent)
            if not plan.ok:
                return self._stop(plan.reason, now)
            if plan.percent == 0:
                return self._stop("idle", now, ok=True)

            if self._steer_changed(plan.steer_deg):
                if self.percent != 0:
                    # Руль в движении мастер не переводит: сначала тормоз.
                    return self._stop("steer_change", now, phase="braking")
                return self._send_steer(plan.steer_deg, now)

            if self.percent != 0 and (plan.percent > 0) != (self.percent > 0):
                # Смена направления тяги — тоже через тормоз.
                return self._stop("direction_change", now, phase="braking")

            aligned = self._aligned(now)
            if aligned is not True:
                if aligned is None:
                    return self._stop("module_offline", now)
                if self._align_since is None:
                    self._align_since = now
                if now - self._align_since > self.align_timeout_s:
                    # Руль не дошёл: забыть цель, чтобы следующий шаг заново отправил поворот.
                    result = self._stop("steer_timeout", now)
                    self.steer_target = {}
                    return result
                return {"ok": True, "phase": "aligning", "reason": "", "percent": 0,
                        "steer": dict(self.steer_target)}

            res = self.source.drive(plan.percent, now=now)
            if not res.get("ok"):
                return self._stop(str(res.get("reason") or "drive_refused"), now)
            self.percent = plan.percent
            return {"ok": True, "phase": "driving", "reason": "", "percent": plan.percent,
                    "steer": dict(self.steer_target)}

    def brake(self, reason: str = "manual", now: Optional[float] = None) -> Dict[str, Any]:
        """Снять тягу тормозом (для вызова извне, например при уходе пульта)."""
        with self._lock:
            return self._stop(reason, time.time() if now is None else now)

    # ------------------------------------------------------------------ #
    def _steer_changed(self, target: Dict[str, float]) -> bool:
        if set(target) != set(self.steer_target):
            return True
        return any(abs(target[k] - self.steer_target.get(k, 1e9)) > STEER_SEND_DEG for k in target)

    def _send_steer(self, target: Dict[str, float], now: float) -> Dict[str, Any]:
        for name, deg in target.items():
            res = self.source.steer_module(name, deg, now=now)
            if not res.get("ok"):
                return self._stop(str(res.get("reason") or "steer_refused"), now)
        self.steer_target = dict(target)
        self._align_since = now
        return {"ok": True, "phase": "steering", "reason": "", "percent": 0,
                "steer": dict(target)}

    def _aligned(self, now: float) -> Optional[bool]:
        """True — все модули на цели и стоят; False — ждём; None — телеметрии нет."""
        lock = getattr(self.source, "lock", None)
        stale = float(getattr(self.source, "STALE_S", 1.0))
        frames = getattr(self.source, "frames", {})
        if lock is not None:
            with lock:
                snapshot = dict(frames)
        else:
            snapshot = dict(frames)
        for name, target in self.steer_target.items():
            entry = snapshot.get(name)
            if entry is None or now - entry[1] >= stale:
                return None
            frame = entry[0]
            if not frame.get("cal") or frame.get("moving") or frame.get("calib"):
                return False
            if abs(float(frame["deg"]) - target) > self.align_deg:
                return False
        return True

    def _stop(self, reason: str, now: float, *, ok: bool = False,
              phase: str = "stopped", force: bool = False) -> Dict[str, Any]:
        # force: тормоз шлётся даже если мост считает тягу выключенной (E-STOP).
        if self.percent != 0 or force:
            self.source.drive(0, now=now)
            self.percent = 0
        self._align_since = None
        return {"ok": ok, "phase": phase, "reason": "" if ok else reason,
                "percent": 0, "steer": dict(self.steer_target)}
