"""Автоматические сигналы шасси: стоп-сигнал при торможении и тревога у препятствия.

Модуль не открывает портов и не даёт команд движению. Он читает уже готовые
снимки — телеметрию шасси (тяга, скорость, ошибки) и свежий скан лидара в
системе робота — и переключает реле/зуммер платы датчиков. Тягу, замок и карты
он не трогает: отказ платы виден в ``error`` и не влияет на торможение.

Стоп-сигнал
-----------
Включается, когда:
* команда тяги сменилась на ноль (STOP, отпущенная клавиша, конец манёвра);
* робот ещё катится (обороты или скорость) при нулевой команде — торможение;
* сработал E-STOP;
* скорость заметно упала, пока тяга ещё есть (плавное торможение);
* мост шасси сообщает фазу ``braking``.

Горит всё время торможения и короткий хвост ``tail_s`` (по умолчанию 1,2 с),
чтобы сзади идущий успел увидеть. Ручной стоп-сигнал оператора автоматика не
гасит: она помнит только свои биты и снимает только их.

Тревога «слишком близко»
------------------------
Свежий скан лидара (не старше ``scan_age_s``), полоса по направлению движения
шириной корпуса. Препятствие ближе ``near_m`` (по умолчанию 0,5 м) при
ненулевой тяге включает передние фары, **мигающие** поворотники и прерывистый
зуммер. Тревога не даёт тяги и не заменяет тормоз — только предупреждает. Без
свежего скана тревога не выдумывается, причина видна в ``reason``.

Пока идёт остановка (``hold``), предупреждение не гаснет, хотя тяги уже нет:
робот стоит у препятствия, и оператор должен видеть, почему. После возврата
управления всё возвращается к обычному правилу «тревога про движение».

Направление берётся из знака команды: вперёд — сектор 0 ± полширины, назад —
сектор π. Стоящий робот тревогу не включает: она про движение. Пока тяга не
снята, порог один для любой скорости: «ползком подъехать в стену» — не режим.

Остановка у препятствия (умный круиз-контроль)
---------------------------------------------
Та же тревога даёт «мягкую» остановку на ``hold_s`` (по умолчанию 5 с): пока
робот едет, тяга снимается, робот стоит и ждёт, затем управление возвращается
само. Это не E-STOP и не блокировка: следующая команда с клавиатуры или из
режима «Скан» принимается сразу, если препятствие уже не ближе ``near_m``;
новая попытка тронуться при всё ещё близком препятствии снова упирается в
остановку. Без свежего скана остановка не выдумывается — работает только
предупреждение, причина видна в ``reason``.
"""
from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

#: Сектор обзора вокруг направления движения: корпус 75 × 60 см, на 0,5 м
#: полширины 0,35 м — это ±35°, берём 45° с запасом на поворот.
DEFAULT_HALF_ANGLE_DEG = 45.0
#: Ширина полосы по курсу, м: боковая стена не считается препятствием впереди.
DEFAULT_HALF_WIDTH_M = 0.35
#: Порог тревоги, м. «Полметра» из постановки; настраивается RC_SAFETY_NEAR_M.
DEFAULT_NEAR_M = 0.5
#: Период опроса. Телеметрия платы идёт 5 Гц, лидар — 10 Гц.
DEFAULT_TICK_S = 0.1
#: Возраст скана, после которого тревога не выдумывается.
DEFAULT_SCAN_AGE_S = 1.0
#: Хвост стоп-сигнала после прекращения торможения.
DEFAULT_TAIL_S = 1.2
#: Остановка у препятствия: сколько секунд стоять, прежде чем вернуть управление.
DEFAULT_HOLD_S = 5.0
#: Снижение скорости, которое считаем торможением, м/с за тик.
DEFAULT_DECEL_MPS = 0.15
#: Прерывистый зуммер: 0,4 с звук, 0,4 с тишина.
BUZZ_PERIOD_S = 0.8
BUZZ_ON_S = 0.4
#: Мигание поворотников при тревоге: 0,35 с горят, 0,35 с погашены. Симметрично,
#: чтобы «аварийка» читалась, а не сливалась в постоянный свет.
BLINK_ON_S = 0.35
#: Длина посылки зуммера на плате; тон короче периода, поэтому он «дробится».
BUZZ_ALERT_MS = 380
#: Пока автоматика владеет битом, его состояние подтверждается заново не чаще
#: этого: команды идут только по изменению, а не 10 раз в секунду.
RESEND_S = 2.0

#: Биты протокола платы (не номера реле: стопы — D5, правый — D3).
LIGHT_BITS = {"head": 1, "stop": 2, "left": 4, "right": 8}


def _env_float(name: str, default: float, lo: float, hi: float) -> float:
    try:
        value = float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default
    if not math.isfinite(value):
        return default
    return max(lo, min(hi, value))


def _env_flag(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return default
    return raw.strip().lower() not in {"0", "false", "no", "off"}


@dataclass(frozen=True)
class SignalConfig:
    """Пороги и периоды. Значения проверяются при сборке, а не на ходу."""

    near_m: float = DEFAULT_NEAR_M
    half_angle_deg: float = DEFAULT_HALF_ANGLE_DEG
    half_width_m: float = DEFAULT_HALF_WIDTH_M
    scan_age_s: float = DEFAULT_SCAN_AGE_S
    tail_s: float = DEFAULT_TAIL_S
    tick_s: float = DEFAULT_TICK_S
    decel_mps: float = DEFAULT_DECEL_MPS
    enabled: bool = True
    #: Зуммер тревоги можно отключить отдельно (площадка, ночь).
    buzzer: bool = True
    #: Остановка у препятствия и её длительность; 0 — только предупреждать.
    hold: bool = True
    hold_s: float = DEFAULT_HOLD_S
    #: Мигать поворотниками, а не держать их постоянно.
    blink: bool = True

    @classmethod
    def from_env(cls) -> "SignalConfig":
        return cls(
            near_m=_env_float("RC_SAFETY_NEAR_M", DEFAULT_NEAR_M, 0.15, 2.0),
            half_angle_deg=_env_float("RC_SAFETY_HALF_ANGLE_DEG", DEFAULT_HALF_ANGLE_DEG, 10.0, 90.0),
            half_width_m=_env_float("RC_SAFETY_HALF_WIDTH_M", DEFAULT_HALF_WIDTH_M, 0.1, 1.0),
            scan_age_s=_env_float("RC_SAFETY_SCAN_AGE_S", DEFAULT_SCAN_AGE_S, 0.2, 5.0),
            tail_s=_env_float("RC_BRAKE_LIGHT_TAIL_S", DEFAULT_TAIL_S, 0.0, 10.0),
            tick_s=_env_float("RC_SIGNALS_TICK_S", DEFAULT_TICK_S, 0.02, 1.0),
            decel_mps=_env_float("RC_BRAKE_DECEL_MPS", DEFAULT_DECEL_MPS, 0.02, 2.0),
            enabled=_env_flag("RC_SIGNALS_ENABLED", True),
            buzzer=_env_flag("RC_SAFETY_BUZZER", True),
            hold=_env_flag("RC_SAFETY_OBSTACLE_HOLD", True),
            hold_s=_env_float("RC_SAFETY_HOLD_S", DEFAULT_HOLD_S, 0.0, 60.0),
            blink=_env_flag("RC_SAFETY_BLINK", True),
        )


@dataclass
class DriveFacts:
    """Только нужное для сигналов; отсутствие данных — честный ноль, не догадка."""

    percent: Optional[int] = None
    #: Знак команды: 1 вперёд, -1 назад, 0 тяги нет.
    direction: int = 0
    moving: bool = False
    speed_mps: Optional[float] = None
    estop: bool = False
    braking: bool = False
    error: str = ""


def facts_from_telemetry(telemetry: Any, *, estop_latched: bool = False) -> DriveFacts:
    """Разобрать снимок ``read()`` шасси (MasterSource, RosSource, NoSource).

    ``estop_latched`` передаёт защёлку E-STOP явным аргументом: в телеметрии
    шасси её может не быть, а сигнал обязан гореть и при взведённом останове.
    """
    if not isinstance(telemetry, dict):
        facts = DriveFacts(error="нет телеметрии шасси")
        facts.estop = bool(estop_latched)
        return facts
    drive = telemetry.get("drive")
    if not isinstance(drive, dict):
        return DriveFacts(error="нет поля drive в телеметрии")
    facts = DriveFacts()
    try:
        commanded = drive.get("commandedPercent")
        facts.percent = int(commanded) if commanded is not None else None
    except (TypeError, ValueError):
        facts.percent = None
    if facts.percent:
        facts.direction = 1 if facts.percent > 0 else -1
    facts.moving = bool(drive.get("active"))
    try:
        speed = float(telemetry.get("speedMps"))
        if math.isfinite(speed):
            facts.speed_mps = speed
            if abs(speed) > 0.03:
                facts.moving = True
    except (TypeError, ValueError):
        pass
    moving_rpm = False
    for motor in telemetry.get("motors") or []:
        if not isinstance(motor, dict) or not motor.get("online"):
            continue
        try:
            rpm = float(motor.get("rpm"))
        except (TypeError, ValueError):
            continue
        if math.isfinite(rpm) and abs(rpm) > 1.0:
            moving_rpm = True
    if moving_rpm:
        facts.moving = True
    phase = str(drive.get("phase") or "").lower()
    reason = str(drive.get("reason") or "").lower()
    facts.braking = phase in {"braking", "stopping"} or reason in {"operator_stop", "estop"}
    estop = telemetry.get("estop")
    if isinstance(estop, dict):
        facts.estop = bool(estop.get("latched") or estop.get("active") or estop.get("engaged"))
    if estop_latched:
        facts.estop = True
    return facts


def nearest_ahead(points: Any, *, direction: int, config: SignalConfig,
                  offset_m: float = 0.0) -> Optional[float]:
    """Ближайшая точка в полосе по курсу, м. Лидар стоит не в центре корпуса."""
    if direction == 0 or not points:
        return None
    base = 0.0 if direction > 0 else math.pi
    half = math.radians(config.half_angle_deg)
    nearest = None
    for point in points:
        try:
            angle = float(point["a"])
            distance = float(point["r"])
        except (KeyError, TypeError, ValueError):
            continue
        if not math.isfinite(angle) or not math.isfinite(distance) or distance <= 0:
            continue
        delta = abs((angle - base + math.pi) % (2 * math.pi) - math.pi)
        if delta > half:
            continue
        along = (offset_m + distance * math.cos(angle)) * (1 if direction > 0 else -1)
        cross = abs(distance * math.sin(angle))
        if along < 0 or cross > config.half_width_m:
            continue
        nearest = along if nearest is None else min(nearest, along)
    return nearest


class SignalController:
    """Состояние стоп-сигнала и тревоги: чистая логика, без потоков и портов."""

    def __init__(self, config: Optional[SignalConfig] = None, *,
                 clock: Callable[[], float] = time.monotonic):
        self.config = config or SignalConfig.from_env()
        self._clock = clock
        self._stop_until = 0.0
        #: Биты, которые включила автоматика; только их она и снимает.
        self._owned = 0
        self._alert = False
        #: Свет тревоги горит (мигающая фаза) — то, что реально уходит на плату.
        self._alert_lights = False
        self._blink_on = False
        self._blink_at = 0.0
        self._buzz_on = False
        self._buzz_at = 0.0
        self._buzz_sent = float("-inf")
        self._last_percent: Optional[int] = None
        self._previous_speed: Optional[float] = None
        self._last_sent: dict[str, float] = {}
        #: Состояние, рассчитанное последним ``update`` (в т.ч. стоп-сигнал).
        self._stop_on = False
        self.distance_m: Optional[float] = None
        #: Пока идёт остановка у препятствия: до какого времени робот стоит и
        #: чем она вызвана. Тягу робот снимает сам — этот модуль только решает.
        self._hold_until = 0.0
        self._hold_since: Optional[float] = None
        self._hold_trigger_distance: Optional[float] = None
        self._hold_count = 0
        self._hold_direction = 0
        #: Последний свежий скан: нужен проверке «тронуться в стену?».
        self._points: Any = None
        self._scan_age: Optional[float] = None
        self._scan_at = 0.0
        self._offset_m = 0.0
        #: Защита от состояния, посчитанного из двух снимков сразу.
        self._lock = threading.RLock()
        self.reason = "ещё не опрашивался"
        self.error = ""
        self.facts = DriveFacts()

    # ------------------------------------------------------------- решение --
    def update(self, facts: DriveFacts, *, points: Any = None, scan_age: Optional[float] = None,
               offset_m: float = 0.0) -> dict[str, Any]:
        with self._lock:
            return self._update(facts, points=points, scan_age=scan_age, offset_m=offset_m)

    def _update(self, facts: DriveFacts, *, points: Any = None,
                scan_age: Optional[float] = None, offset_m: float = 0.0) -> dict[str, Any]:
        now = self._clock()
        if scan_age is not None and points:
            # Скан запоминаем для проверки перед выдачей тяги: она приходит
            # между тактами фоновой нити и должна видеть ту же обстановку.
            self._points, self._scan_age = points, float(scan_age)
            self._scan_at, self._offset_m = now, float(offset_m or 0.0)
        self.facts = facts
        cfg = self.config
        if not cfg.enabled:
            self._alert = False
            self._stop_until = 0.0
            self._release_hold(now, "автосигналы выключены (RC_SIGNALS_ENABLED=0)")
            return self.snapshot()

        # --- стоп-сигнал: реальное торможение, а не каждая пауза
        released = self._last_percent not in (None, 0) and facts.percent == 0
        rolling = facts.percent in (None, 0) and facts.moving
        slowing = False
        if facts.speed_mps is not None and self._previous_speed is not None:
            # Тяга ещё есть, но скорость падает — это тоже торможение.
            slowing = facts.speed_mps > 0.02 and self._previous_speed - facts.speed_mps >= cfg.decel_mps
            if facts.speed_mps <= 0.02:
                slowing = False
        if facts.speed_mps is not None:
            self._previous_speed = facts.speed_mps
        braking = bool(facts.braking or facts.estop or released or rolling or slowing)
        if braking:
            self._stop_until = now + cfg.tail_s
        if facts.percent and not facts.braking and not facts.estop:
            # Снова едем: хвост не должен гореть поверх тяги, но уже включённый
            # стоп-сигнал гаснет не мгновенно — иначе он мигал бы на разгоне.
            self._stop_until = min(self._stop_until, now + min(cfg.tail_s, 0.4))
        stop_on = braking or now < self._stop_until
        self._stop_on = stop_on

        # --- тревога: по свежему скану, в движении или во время самой паузы
        self.distance_m = None
        alert = False
        holding = self._hold_since is not None
        if not facts.direction and not holding:
            self.reason = "нет тяги — тревога не нужна"
        elif scan_age is None:
            self.reason = "нет скана лидара — тревога не выдумывается"
        elif scan_age > cfg.scan_age_s:
            self.reason = "скан устарел (%.1f с)" % scan_age
        else:
            # Во время паузы тяги нет, но направление последнего хода помним:
            # иначе предупреждение погасло бы именно тогда, когда робот стоит
            # у препятствия и оператор ждёт объяснения.
            direction = facts.direction or (self._hold_direction if holding else 0)
            near = nearest_ahead(points, direction=direction, config=cfg, offset_m=offset_m)
            self.distance_m = near
            if near is not None and near <= cfg.near_m:
                alert = True
                self.reason = "препятствие %.2f м по курсу" % near
            elif near is not None:
                self.reason = "по курсу свободно: %.2f м" % near
            else:
                self.reason = "по курсу точек нет"
        self._alert = alert
        was_holding = self._hold_since is not None
        triggered = self._update_hold(now, facts, alert)
        if was_holding and self._hold_since is None and not facts.direction:
            # Пауза кончилась, а тяги нет: предупреждение снова про движение и
            # не должно остаться гореть на стоянке из-за прошлого хода.
            self._alert = False
        # Предупреждение не гаснет во время паузы: робот стоит у препятствия.
        lights_on = bool(self._alert or self._hold_since is not None) and scan_age is not None
        self._blink_on = self._blink_phase(now, lights_on, cfg)
        self._alert_lights = lights_on and self._blink_on

        # --- зуммер: прерывистый, чтобы не сливаться в постоянный вой
        if (alert or self._hold_since is not None) and cfg.buzzer:
            if not self._buzz_on and now - self._buzz_at >= BUZZ_PERIOD_S - BUZZ_ON_S:
                self._buzz_on, self._buzz_at = True, now
            elif self._buzz_on and now - self._buzz_at >= BUZZ_ON_S:
                self._buzz_on, self._buzz_at = False, now
        else:
            self._buzz_on = False

        self._last_percent = facts.percent
        return self.snapshot(stop_on=stop_on, triggered=triggered)

    def _blink_phase(self, now: float, active: bool, cfg: SignalConfig) -> bool:
        """Мигание «аварийки»: 0,35 с горит, 0,35 с погашено.

        Мигает сам контроллер, поэтому реле переключаются с такта опроса
        (10 Гц), а не от каждого кадра лидара. Без ``blink`` свет горит ровно.
        """
        if not active:
            self._blink_at = 0.0
            return False
        if not cfg.blink:
            return True
        if self._blink_at <= 0.0:
            self._blink_at = now
            return True
        elapsed = now - self._blink_at
        if elapsed >= 2 * BLINK_ON_S:
            self._blink_at = now + (elapsed % (2 * BLINK_ON_S)) - BLINK_ON_S
            elapsed = now - self._blink_at
        return elapsed < BLINK_ON_S

    # ------------------------------------------------- остановка у препятствия --
    def _start_hold(self, now: float, distance: Optional[float], direction: int = 0) -> bool:
        cfg = self.config
        self._hold_since = now
        if direction:
            self._hold_direction = 1 if direction > 0 else -1
        # Пауза не продлевается сама: иначе близкая стена держала бы робота
        # вечно, и «вернуть управление» было бы нечем.
        self._hold_until = now + max(0.1, cfg.hold_s)
        self._hold_trigger_distance = distance
        self._hold_count += 1
        self.reason = "препятствие %s м по курсу — остановка на %.0f с" % (
            "%.2f" % distance if distance is not None else "?", cfg.hold_s)
        self._alert = True
        return True

    def _update_hold(self, now: float, facts: DriveFacts, alert: bool) -> bool:
        """Решить, нужна ли остановка у препятствия. ``True`` — она началась сейчас."""
        cfg = self.config
        if not cfg.enabled or not cfg.hold:
            self._release_hold(now, self.reason)
            return False
        if self._hold_since is not None:
            if now >= self._hold_until:
                # Пауза вышла: управление возвращается независимо от того, что
                # впереди. Следующая попытка тяги проверит обстановку заново.
                self._release_hold(now, "остановка окончена — управление вернулось")
            return False
        if facts.direction and alert:
            return self._start_hold(now, self.distance_m, facts.direction)
        return False

    def obstacle(self, direction: int, *, now: Optional[float] = None) -> Optional[float]:
        """Ближайшая точка по курсу из последнего свежего скана, м (или ``None``)."""
        if not direction or not self._points:
            return None
        now = self._clock() if now is None else float(now)
        age = self._scan_age
        if age is not None:
            age += max(0.0, now - self._scan_at)
        if age is None or age > self.config.scan_age_s:
            return None
        return nearest_ahead(self._points, direction=direction, config=self.config,
                             offset_m=self._offset_m)

    def request_hold(self, direction: int, *, now: Optional[float] = None) -> dict[str, Any]:
        """Проверка перед выдачей тяги: не начинать ход в близкое препятствие.

        Вызывается ручным управлением (клавиатура, режим «Скан») до того, как
        тяга уйдёт на платы, поэтому рывка в сторону стены не бывает даже на
        десятые доли секунды. Если препятствие близко, пауза начинается здесь
        же; если нет — она не начинается вовсе.
        """
        now = self._clock() if now is None else float(now)
        with self._lock:
            if self._hold_since is not None and now < self._hold_until:
                return self.snapshot(now=now)
            if self._hold_since is not None:
                self._release_hold(now, "остановка окончена — управление вернулось")
            if not self.config.enabled or not self.config.hold or not direction:
                return self.snapshot(now=now, triggered=False)
            near = self.obstacle(direction, now=now)
            if near is not None and near <= self.config.near_m:
                self.distance_m = near
                triggered = self._start_hold(now, near, direction)
                return self.snapshot(now=now, triggered=triggered)
            return self.snapshot(now=now, triggered=False)

    def _release_hold(self, now: float, reason: str) -> None:
        if self._hold_since is not None:
            self.reason = reason
        self._hold_since = None
        self._hold_until = 0.0
        self._hold_trigger_distance = None

    def clear_hold(self, reason: str = "оператор снял паузу") -> None:
        """Снять паузу досрочно: тяга снова разрешена (E-STOP снимают отдельно)."""
        with self._lock:
            self._release_hold(self._clock(), reason)

    def snapshot(self, *, stop_on: Optional[bool] = None,
                 triggered: Optional[bool] = None,
                 now: Optional[float] = None) -> dict[str, Any]:
        brake = self._stop_on if stop_on is None else stop_on
        now = self._clock() if now is None else float(now)
        holding = self._hold_since is not None
        remaining = 0.0
        if holding:
            remaining = round(max(0.0, self._hold_until - now), 1)
        return {
            "enabled": self.config.enabled,
            "brakeLight": brake,
            "alert": self._alert,
            "buzz": bool(self._buzz_on),
            #: Предупреждение включено (не гаснет во время паузы).
            "lightsOn": self._alert_lights,
            "blink": self.config.blink,
            "distanceM": None if self.distance_m is None else round(self.distance_m, 3),
            "nearM": self.config.near_m,
            "hold": holding,
            "holdRemainingS": remaining,
            "holdTriggered": bool(triggered) if triggered is not None else None,
            "holdCount": self._hold_count,
            "holdS": self.config.hold_s if self.config.hold else 0.0,
            "reason": self.reason,
            "error": self.error,
        }

    def desired_lights(self, *, stop_on: Optional[bool] = None) -> dict[str, bool]:
        """Какие биты света автоматика держит прямо сейчас."""
        wanted: dict[str, bool] = {}
        if stop_on is None:
            stop_on = self._stop_on
        if stop_on:
            wanted["stop"] = True
        if self._alert:
            # Фары горят всё время тревоги, поворотники — по фазе мигания.
            wanted["head"] = True
            if self._alert_lights:
                wanted.update(left=True, right=True)
        return wanted

    # -------------------------------------------------------------- вывод --
    def apply(self, module: Any, *, manual: Any = None,
              stop_on: Optional[bool] = None) -> dict[str, Any]:
        """Привести реле и зуммер платы к нужному состоянию.

        ``manual`` — последние известные биты оператора. Если бит включил он,
        автоматика его не трогает; при отпускании тревоги снимаются только
        биты, которые включила она сама.
        """
        state = self.snapshot(stop_on=stop_on)
        if module is None:
            return state
        now = self._clock()
        manual_lights = manual if isinstance(manual, dict) else {}
        try:
            for name, on in self.desired_lights(stop_on=stop_on).items():
                bit = LIGHT_BITS[name]
                if manual_lights.get(name) and not (self._owned & bit):
                    continue  # включил человек — это его свет
                if not on:
                    continue
                sent = self._last_sent.get(name)
                if sent is None or now - sent >= RESEND_S:
                    module.light(name, True)
                    self._last_sent[name] = now
                self._owned |= bit
            for name in ("head", "stop", "left", "right"):
                bit = LIGHT_BITS[name]
                if not (self._owned & bit) or name in self.desired_lights(stop_on=stop_on):
                    continue
                if manual_lights.get(name):
                    # Человек включил тот же свет, пока автоматика им владела:
                    # отдаём бит ему и больше не гасим.
                    self._owned &= ~bit
                    self._last_sent.pop(name, None)
                    continue
                module.light(name, False)
                self._owned &= ~bit
                self._last_sent.pop(name, None)
            if state["buzz"] and self.config.buzzer and now - self._buzz_sent >= BUZZ_ON_S:
                # Одна посылка на импульс: тон на плате ограничен по времени,
                # повторять его каждый такт незачем.
                module.buzz()
                self._buzz_sent = now
            self.error = ""
        except Exception as exc:  # noqa: BLE001 — плата не должна ронять сервер
            self.error = str(exc) or type(exc).__name__
        state["error"] = self.error
        return state

    def release(self, module: Any) -> None:
        """Погасить всё, чем владела автоматика (остановка сервера, смена режима)."""
        if module is None:
            return
        try:
            for name in ("head", "stop", "left", "right"):
                bit = LIGHT_BITS[name]
                if self._owned & bit:
                    module.light(name, False)
            self._owned = 0
            self._last_sent.clear()
        except Exception as exc:  # noqa: BLE001
            self.error = str(exc) or type(exc).__name__
        self._alert = False
        self._alert_lights = False
        self._blink_on = False
        self._blink_at = 0.0
        self._buzz_on = False
        self._stop_until = 0.0
        self._stop_on = False
        self._hold_since = None
        self._hold_until = 0.0
        self._hold_trigger_distance = None


class DriveSignalService:
    """Фоновая нить: снимки шасси и лидара → ``SignalController`` → плата.

    Провайдеры ленивые и вызываются с обработкой ошибок: неподключённое шасси
    или отсутствующий лидар дают пустые данные, а не падение сервера.
    """

    def __init__(self, controller: SignalController, *,
                 telemetry_provider: Callable[[], Any],
                 lidar_provider: Optional[Callable[[], Any]] = None,
                 module_provider: Callable[[], Any],
                 offset_provider: Optional[Callable[[], float]] = None,
                 estop_provider: Optional[Callable[[], Any]] = None):
        self.controller = controller
        self._estop = estop_provider
        self._telemetry = telemetry_provider
        self._lidar = lidar_provider
        self._module = module_provider
        self._offset = offset_provider or (lambda: 0.0)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self.last_state = controller.snapshot()

    # ------------------------------------------------------------- опрос ---
    def step(self) -> dict[str, Any]:
        """Один такт. Ручной вызов используется тестами и первым кадром."""
        try:
            telemetry = self._telemetry()
        except Exception as exc:  # noqa: BLE001
            telemetry = None
            self.controller.error = str(exc) or type(exc).__name__
        try:
            estop_latched = bool(self._estop()) if self._estop is not None else False
        except Exception:  # noqa: BLE001 — защёлка читает файл; ошибка = не взведена явно
            estop_latched = False
        facts = facts_from_telemetry(telemetry, estop_latched=estop_latched)
        points, age = None, None
        if self._lidar is not None and facts.direction:
            try:
                info = self._lidar() or {}
                points = info.get("points")
                stamp = float(info.get("stamp") or 0.0)
                age = time.time() - stamp if stamp > 0 else None
            except Exception as exc:  # noqa: BLE001
                points, age = None, None
                self.controller.error = str(exc) or type(exc).__name__
        try:
            offset = float(self._offset())
        except (TypeError, ValueError):
            offset = 0.0
        state = self.controller.update(facts, points=points, scan_age=age, offset_m=offset)
        module = None
        try:
            module = self._module()
        except Exception as exc:  # noqa: BLE001
            self.controller.error = str(exc) or type(exc).__name__
        manual = {}
        if module is not None:
            try:
                manual = (module.state() or {}).get("lights") or {}
            except Exception:  # noqa: BLE001 — без телеметрии считаем, что ручного нет
                manual = {}
        state = self.controller.apply(module, manual=manual, stop_on=state["brakeLight"])
        with self._lock:
            self.last_state = state
        return state

    def state(self) -> dict[str, Any]:
        with self._lock:
            return dict(self.last_state)

    # -------------------------------------------------------------- поток --
    def start(self) -> "DriveSignalService":
        if self._thread is not None and self._thread.is_alive():
            return self
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="drive-signals", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=2.0)
        self.controller.release(self._module())

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self.step()
            except Exception as exc:  # noqa: BLE001 — сигналы не должны ронять экран
                self.controller.error = str(exc) or type(exc).__name__
            self._stop.wait(self.controller.config.tick_s)
