# -*- coding: utf-8 -*-
"""Манёвры тяги: «метр вперёд», «развернуться на 90°» и другие пресеты.

Пресет здесь — это не «держать 12 % две секунды», а *цель*: расстояние в
метрах или угол в градусах. Оператор нажимает кнопку, сервер сам:

1. ставит колёса в нужные углы и ждёт, пока модули доедут;
2. плавно (рампа) набирает рабочий процент тяги;
3. считает пройденный путь по тахометру — обороты каждого колеса за тик,
   умноженные на длину окружности;
4. гасит тягу заранее (тормозной участок) и останавливает моторы на цели.

Длина окружности берётся из диаметра колеса, а угол разворота — из геометрии
шасси: при вращении вокруг центра каждое колесо проходит дугу
``R · θ``, где ``R = hypot(база/2, колея/2)``. Поэтому «90°» — это не
«сколько-то секунд», а измеренный угол.

Гироскопа в шасси нет: единственный датчик движения — тахометр на D19.
Если его нет (или он молчит), раннер честно переходит в режим **по времени**:
берёт скорость с прошлого удачного прогона (`learned`), а если прогонов ещё
не было — осторожную заводскую оценку. В обоих случаях экран пишет, чем
именно меряли, — «метр» на глазок не выдаётся за метр.

Модуль ничего не отправляет сам: он получает кадры телеметрии и две функции
(«повернуть колесо», «дать тягу»). Так его можно проверить без железа —
``tests/test_maneuvers.py``.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path

MODULES = ("FL", "FR", "RL", "RR")

#: Куда поворачивать колёса для бокового хода: как у пресетов руления.
SIDE_DEG = 90.0

#: Заводские параметры шасси. Мотор AOVO Pro 350 Вт — колесо 8,5″ (216 мм);
#: база и колея — типичные для такого шасси, их надо померить на своём роботе.
DEFAULT_CHASSIS = {
    "wheelDiameterMm": 216.0,
    "wheelbaseMm": 500.0,
    "trackMm": 450.0,
    "rampSeconds": 0.5,
}

#: Заводские оценки скорости для режима «по времени» (пока нет своего прогона).
#: Это именно оценка: после первого удачного прогона по тахометру сервер
#: запоминает настоящую скорость (`learned`) и дальше берёт её.
DEFAULT_NOMINAL = {"mps": 0.35, "dps": 45.0}

#: Пресеты. `kind` — что меряем: метры (`distanceM`) или градусы (`angleDeg`).
DEFAULT_PRESETS = {
    "forward": {
        "title": "Вперёд", "kind": "distance", "distanceM": 1.0,
        "percent": 12, "toleranceM": 0.05,
    },
    "backward": {
        "title": "Назад", "kind": "distance", "distanceM": 0.5,
        "percent": 10, "toleranceM": 0.05,
    },
    "left": {
        "title": "Боком влево", "kind": "distance", "distanceM": 0.5,
        "percent": 10, "toleranceM": 0.05,
    },
    "right": {
        "title": "Боком вправо", "kind": "distance", "distanceM": 0.5,
        "percent": 10, "toleranceM": 0.05,
    },
    "spin_left": {
        "title": "Разворот влево", "kind": "angle", "angleDeg": 90.0,
        "percent": 10, "toleranceDeg": 3.0,
    },
    "spin_right": {
        "title": "Разворот вправо", "kind": "angle", "angleDeg": 90.0,
        "percent": 10, "toleranceDeg": 3.0,
    },
}

MANEUVER_IDS = tuple(DEFAULT_PRESETS)

#: Разумные границы того, что вообще имеет смысл вводить.
LIMITS = {
    "distanceM": (0.05, 20.0),
    "angleDeg": (5.0, 360.0),
    "percent": (1.0, 100.0),
    "toleranceM": (0.01, 1.0),
    "toleranceDeg": (0.5, 30.0),
    "wheelDiameterMm": (50.0, 400.0),
    "wheelbaseMm": (100.0, 3000.0),
    "trackMm": (100.0, 3000.0),
    "rampSeconds": (0.1, 3.0),
    "mps": (0.01, 5.0),
    "dps": (1.0, 360.0),
}


# --------------------------------------------------------------- геометрия # #
def wheel_circumference_m(diameter_mm):
    """Длина окружности колеса в метрах: один оборот тахометра = этот путь."""
    return math.pi * float(diameter_mm) / 1000.0


def turn_radius_m(chassis):
    """Радиус, по которому едет каждое колесо при вращении вокруг центра."""
    return math.hypot(float(chassis["wheelbaseMm"]) / 2000.0,
                      float(chassis["trackMm"]) / 2000.0)


def pivot_angle_deg(chassis):
    """Угол колеса для разворота на месте: касательная к окружности поворота.

    Колесо на углу корпуса стоит от центра на ``R = hypot(база/2, колея/2)``,
    а ехать ему нужно поперёк радиуса: ``atan2(база/2, колея/2)``. Для
    типичного шасси это 45…50° — всегда меньше 90°, то есть внутри хода руля
    ±100°. Настоящая касательная (135°) не нужна: колесо, повёрнутое на +48° и
    пущенное назад, толкает корпус ровно туда же, куда толкала бы касательная.
    """
    return math.degrees(math.atan2(float(chassis["wheelbaseMm"]) / 2.0,
                                   float(chassis["trackMm"]) / 2.0))


def steering_targets(name, chassis=None):
    """Куда повернуть колёса и в какую сторону крутить каждое.

    Знаки — как в прошивке: положительный угол поворачивает колесо вправо.
    Направление тяги ``+1`` — «вперёд» для этого колеса (с учётом его
    зеркальности, которую задаёт ``m dir``), ``-1`` — назад.
    """
    if name not in DEFAULT_PRESETS:
        raise ValueError("неизвестный манёвр: %s" % name)
    chassis = chassis or DEFAULT_CHASSIS
    if name in ("forward", "backward"):
        return ({m: 0.0 for m in MODULES},
                {m: 1 if name == "forward" else -1 for m in MODULES})
    if name in ("left", "right"):
        angle = -SIDE_DEG if name == "left" else SIDE_DEG
        return {m: angle for m in MODULES}, {m: 1 for m in MODULES}
    # Разворот против часовой: углы зеркальные, борта едут в разные стороны.
    pivot = pivot_angle_deg(chassis)
    ccw = {"FL": (pivot, -1), "FR": (-pivot, 1),
           "RL": (-pivot, -1), "RR": (pivot, 1)}
    if name == "spin_right":
        ccw = {m: (angle, -sign) for m, (angle, sign) in ccw.items()}
    return ({m: angle for m, (angle, _) in ccw.items()},
            {m: sign for m, (_, sign) in ccw.items()})


def _angle_error(current, target):
    """Разница углов без сюрпризов на переходе через ±180°."""
    diff = (float(current) - float(target)) % 360.0
    return abs(diff - 360.0) if diff > 180.0 else abs(diff)


# ------------------------------------------------------------------ конфиг # #
def default_config():
    return {
        "chassis": dict(DEFAULT_CHASSIS),
        "nominal": dict(DEFAULT_NOMINAL),
        "presets": {name: dict(values) for name, values in DEFAULT_PRESETS.items()},
        #: Скорость, измеренная на удачном прогоне: манёвр → {"rate", "percent"}.
        "learned": {},
    }


def _clamp_number(value, limits):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    low, high = limits
    if number < low or number > high:
        return None
    return number


def normalize_config(raw):
    """Привести файл настроек к рабочему виду: чужие поля и мусор отбрасываем."""
    config = default_config()
    if not isinstance(raw, dict):
        return config
    for group in ("chassis", "nominal"):
        values = raw.get(group)
        if not isinstance(values, dict):
            continue
        for key in config[group]:
            limits = LIMITS.get(key)
            if limits is None:
                continue
            number = _clamp_number(values.get(key), limits)
            if number is not None:
                config[group][key] = number
    presets = raw.get("presets")
    if isinstance(presets, dict):
        for name, values in presets.items():
            if name not in config["presets"] or not isinstance(values, dict):
                continue
            merged = config["presets"][name]
            for key in ("distanceM", "angleDeg", "percent", "toleranceM", "toleranceDeg"):
                if key not in merged:
                    continue
                number = _clamp_number(values.get(key), LIMITS[key])
                if number is not None:
                    merged[key] = number
    learned = raw.get("learned")
    if isinstance(learned, dict):
        for name, values in learned.items():
            if name not in config["presets"] or not isinstance(values, dict):
                continue
            rate = _clamp_number(values.get("rate"), (0.001, 1000.0))
            percent = _clamp_number(values.get("percent"), LIMITS["percent"])
            if rate is not None and percent is not None:
                config["learned"][name] = {"rate": rate, "percent": percent}
    return config


def apply_preset_update(config, name, values):
    """Обновить один пресет (или шасси) из формы экрана.

    Возвращает ``(config, error)``; при ошибке конфиг не меняется. Числа
    проверяются по :data:`LIMITS`, лишние поля молча отбрасываются — экран
    может прислать весь снимок формы, а не одно поле.
    """
    if not isinstance(values, dict):
        return config, "invalid_value"
    updated = json.loads(json.dumps(config))          # глубокая копия
    if name == "chassis":
        target = updated["chassis"]
    elif name == "nominal":
        target = updated["nominal"]
    elif name in updated["presets"]:
        target = updated["presets"][name]
    else:
        return config, "invalid_maneuver"
    changed = False
    for key, value in values.items():
        if key not in target:
            continue
        limits = LIMITS.get(key)
        if limits is None:
            continue
        number = _clamp_number(value, limits)
        if number is None:
            return config, "invalid_value"
        target[key] = number
        changed = True
    if not changed:
        return config, "invalid_value"
    return normalize_config(updated), None


def config_path(path=None):
    """Файл настроек манёвров. ``data/`` — локальные данные, в git не попадает."""
    if path is not None:
        return Path(path)
    env = os.environ.get("RC_MANEUVER_CONFIG", "").strip()
    if env:
        return Path(env)
    root = Path(__file__).resolve().parent.parent
    return root / "data" / "maneuvers.json"


def load_config(path=None):
    target = config_path(path)
    try:
        raw = json.loads(target.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return default_config()
    return normalize_config(raw)


def save_config(config, path=None):
    target = config_path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(tmp, target)
    return target


def preset_target(preset):
    """Цель манёвра в «своих» единицах: метры или градусы."""
    return float(preset["distanceM"] if preset["kind"] == "distance" else preset["angleDeg"])


def preset_tolerance(preset):
    return float(preset["toleranceM"] if preset["kind"] == "distance" else preset["toleranceDeg"])


# ------------------------------------------------------------------ раннер # #
class ManeuverRunner:
    """Один манёвр за раз: руление → рампа → измерение → остановка на цели.

    Раннер не знает, откуда берутся кадры и куда уходят команды: он получает
    ``frames`` (модуль → кадр телеметрии) и две функции-отправки. Всё
    состояние — под замком, потому что тикает он в потоке порта, а читает его
    HTTP-запрос экрана.
    """

    IDLE = "idle"
    STEERING = "steering"
    DRIVING = "driving"
    DONE = "done"
    FAILED = "failed"
    CANCELLED = "cancelled"
    ACTIVE = (STEERING, DRIVING)

    STEER_TOLERANCE_DEG = 1.5
    STEER_RESEND_S = 0.4
    STEER_TIMEOUT_S = 10.0
    SEND_INTERVAL_S = 0.2
    #: Сколько ждать обороты, прежде чем признать тахометр молчащим.
    TACH_GRACE_S = 1.5
    #: Сколько секунд результат манёвра остаётся на экране.
    RESULT_S = 20.0

    def __init__(self, config=None, clock=time.monotonic, brake=None):
        self.lock = threading.RLock()
        self.clock = clock
        #: Чем гасить тягу при остановке манёвра (у источника — «b» всем колёсам).
        self.brake = brake or (lambda: None)
        self.config = normalize_config(config if config is not None
                                       else default_config())
        self._clear()

    # ------------------------------------------------------------- состояние #
    def _clear(self):
        self.name = None
        self.state = self.IDLE
        self.reason = None
        self.message = ""
        self.mode = "odometry"
        self.target = 0.0
        self.value = 0.0
        self.progress = 0.0
        self.percent = 0
        self.work_percent = 0
        self.started_at = None
        self.finished_at = None
        self.stopped_at = None
        self.learned = None
        self._level = 0.0
        self._decel = False
        self._last_tick = None
        self._elapsed = 0.0
        self._travel = {m: 0.0 for m in MODULES}
        self._sent = {}                  # модуль → последний отправленный процент
        self._sent_at = 0.0
        self._steer_at = -1e9             # первая команда рулю — сразу, без паузы
        self._deadline = 0.0
        self._rate = 0.0
        self._speed = 0.0                # измеренная скорость, м/с или °/с
        self._last_value = 0.0

    def status(self):
        with self.lock:
            preset = self.config["presets"].get(self.name) if self.name else None
            return {
                "active": self.state in self.ACTIVE,
                "state": self.state,
                "name": self.name,
                "title": (preset or {}).get("title"),
                "kind": (preset or {}).get("kind"),
                "mode": self.mode,
                "target": round(self.target, 3),
                "value": round(self.value, 3),
                "progress": round(self.progress, 3),
                "percent": self.percent,
                "workPercent": self.work_percent,
                "reason": self.reason,
                "message": self.message,
                "learned": dict(self.learned) if self.learned else None,
            }

    def config_payload(self):
        """То, что видит экран: пресеты, шасси, пределы и запомненные скорости."""
        with self.lock:
            return {
                "presets": {name: dict(values)
                            for name, values in self.config["presets"].items()},
                "chassis": dict(self.config["chassis"]),
                "nominal": dict(self.config["nominal"]),
                "learned": {name: dict(values)
                            for name, values in self.config["learned"].items()},
                "limits": {key: list(bounds) for key, bounds in LIMITS.items()},
            }

    def update_config(self, name, values):
        """Обновить настройки из формы экрана.

        Под работающим мотором нельзя: смена процента — рывок, смена диаметра
        колеса — другая одометрия прямо посреди манёвра.
        """
        with self.lock:
            if self.state in self.ACTIVE:
                return {"ok": False, "reason": "module_busy"}
            before = self.config["presets"].get(name, {}).get("percent")
            self.config, error = apply_preset_update(self.config, name, values)
            if error:
                return {"ok": False, "reason": error}
            # Запомненная скорость измерена на другом проценте — сбиваем её,
            # чтобы «по времени» не считало по устаревшей цифре.
            if name in self.config["presets"] and \
                    self.config["presets"][name]["percent"] != before:
                self.config["learned"].pop(name, None)
            return {"ok": True, "config": self.config_payload()}

    # ------------------------------------------------------------------ старт #
    def start(self, name, now, limit_percent=None):
        """Начать манёвр. ``now`` — монотонное время (по нему же идут тики)."""
        with self.lock:
            if name not in self.config["presets"]:
                return {"ok": False, "reason": "invalid_maneuver"}
            if self.state in self.ACTIVE:
                return {"ok": False, "reason": "maneuver_busy"}
            preset = self.config["presets"][name]
            work = int(round(float(preset["percent"])))
            if limit_percent is not None:
                work = min(work, max(1, int(limit_percent)))
            if work < 1:
                return {"ok": False, "reason": "invalid_value"}
            self._clear()
            self.name = name
            self.state = self.STEERING
            self.target = preset_target(preset)
            self.work_percent = work
            self.started_at = now
            self._last_tick = now
            self._deadline = now + self.STEER_TIMEOUT_S + self._expected_seconds() * 2.5 + 5.0
            self.message = "Колёса встают в углы манёвра…"
            return {"ok": True, "maneuver": self.status()}

    def cancel(self, now=None, reason="cancelled"):
        """Остановить манёвр (кнопка СТОП, E-STOP, ручная тяга, потеря колеса)."""
        with self.lock:
            if self.state not in self.ACTIVE:
                return self.status()
            self._stop(reason, now)
            return self.status()

    # ------------------------------------------------------------------- тик  #
    def tick(self, now, frames, steer, drive):
        """Продвинуть манёвр на один шаг.

        ``frames`` — словарь «модуль → кадр телеметрии» (``deg``, ``moving``,
        ``rpm``, ``tach``), ``steer(module, angle)`` и
        ``drive(module, percent)`` — отправка команд (``module=None`` —
        всем колёсам сразу).
        """
        with self.lock:
            if self.state not in self.ACTIVE:
                return self.status()
            dt = max(0.0, now - (self._last_tick if self._last_tick is not None else now))
            self._last_tick = now
            if now > self._deadline:
                self._stop("timeout", now)
                return self.status()
            if self.state == self.STEERING:
                self._steer_phase(now, frames, steer)
            if self.state == self.DRIVING:
                self._drive_phase(now, frames, drive, dt)
            return self.status()

    def _steer_phase(self, now, frames, steer):
        targets, _ = steering_targets(self.name, self.config["chassis"])
        offline = [m for m in MODULES
                   if not frames.get(m) or not frames[m].get("cal")]
        if offline:
            self._stop("not_homed" if all(frames.get(m) for m in offline)
                       else "module_offline", now)
            return
        if now - self.started_at > self.STEER_TIMEOUT_S:
            self._stop("steering_timeout", now)
            return
        arrived = all(_angle_error(frames[m].get("deg") or 0.0, targets[m])
                      <= self.STEER_TOLERANCE_DEG and not frames[m].get("moving")
                      for m in MODULES)
        if not arrived:
            if now - self._steer_at >= self.STEER_RESEND_S:
                self._steer_at = now
                for module in MODULES:
                    steer(module, targets[module])
            return
        # Колёса на месте — начинаем движение с нуля измерений.
        self.state = self.DRIVING
        self._elapsed = 0.0
        self._travel = {m: 0.0 for m in MODULES}
        self._sent = {}
        self.mode = "odometry" if any(frames[m].get("tach") for m in MODULES) else "time"
        self._rate = self._rate_estimate()
        self._speed = 0.0
        self._last_value = 0.0
        self._deadline = now + self._expected_seconds() * 2.5 + 5.0
        self.message = "Едем." if self.mode == "odometry" else "Едем; тахометр не подключён — считаю по времени."

    def _drive_phase(self, now, frames, drive, dt):
        self._elapsed += dt
        measured = self._measure(dt, frames)
        if dt > 0:
            # Мгновенная скорость с лёгким сглаживанием: по ней считаем, где
            # начинать тормозной участок.
            instant = (measured - self._last_value) / dt
            self._speed = instant if self._speed <= 0 else 0.7 * self._speed + 0.3 * instant
        self._last_value = measured
        self.value = measured
        self.progress = 1.0 if self.target <= 0 else min(1.0, measured / self.target)
        remaining = max(0.0, self.target - measured)
        preset = self.config["presets"][self.name]
        tolerance = preset_tolerance(preset)
        ramp = float(self.config["chassis"]["rampSeconds"])
        speed = self._current_speed()
        # Тормозной участок: рампа гасит тягу за ramp секунд, а средняя
        # скорость на ней вдвое ниже рабочей — столько и останется проехать.
        if not self._decel and remaining <= 0.5 * speed * ramp + 0.25 * tolerance:
            self._decel = True
        step = max(0.5, float(self.work_percent) * dt / max(0.05, ramp))
        if self._decel:
            self._level = max(0.0, self._level - step)
        else:
            self._level = min(float(self.work_percent), self._level + step)
        if self._level <= 0.0 and self._decel:
            # Рампа догасила тягу: доехали или не дотянули — это разные ответы.
            self._stop(self._arrival_reason(tolerance), now)
            return
        if self.value >= self.target + tolerance:
            self._stop(self._arrival_reason(tolerance), now)
            return
        self._apply(now, drive)

    def _arrival_reason(self, tolerance):
        error = self.value - self.target
        if abs(error) <= tolerance:
            return "reached"
        return "underrun" if error < 0 else "overshoot"

    # ------------------------------------------------------------- измерение #
    def _measure(self, dt, frames):
        circumference = wheel_circumference_m(self.config["chassis"]["wheelDiameterMm"])
        for module in MODULES:
            frame = frames.get(module) or {}
            rpm = frame.get("rpm")
            if rpm is None:
                continue
            self._travel[module] += (float(rpm) / 60.0) * circumference * dt
        if self.mode == "odometry":
            travel = max(abs(self._travel[m]) for m in MODULES)
            if self._elapsed > self.TACH_GRACE_S and travel < 0.01:
                # Тахометр «есть», а оборотов нет: не ждём у моря погоды.
                self.mode = "time"
                self._rate = self._rate_estimate()
                self.message = "Тахометр молчит — считаю по времени."
        if self.mode == "time":
            return self._elapsed * self._rate
        return self._odometry_value()

    def _odometry_value(self):
        preset = self.config["presets"][self.name]
        travel = [abs(self._travel[m]) for m in MODULES]
        if not travel:
            return 0.0
        mean = sum(travel) / len(travel)
        if preset["kind"] == "distance":
            return mean
        radius = turn_radius_m(self.config["chassis"])
        if radius <= 0:
            return 0.0
        return math.degrees(mean / radius)

    def _current_speed(self):
        """Оценка скорости в единицах манёвра: м/с или °/с."""
        preset = self.config["presets"][self.name]
        if self.mode == "odometry" and self._elapsed > 0.35 and self.value > 0.005:
            measured = self.value / self._elapsed
            if measured > 0:
                return measured
        return self._rate_estimate()

    def _rate_estimate(self):
        preset = self.config["presets"][self.name]
        percent = max(1.0, float(self.work_percent))
        learned = self.config["learned"].get(self.name)
        if learned and learned.get("percent"):
            return float(learned["rate"]) * percent / float(learned["percent"])
        if preset["kind"] == "distance":
            return float(self.config["nominal"]["mps"]) * percent / 100.0
        return float(self.config["nominal"]["dps"]) * percent / 100.0

    def _expected_seconds(self):
        rate = self._rate_estimate()
        if rate <= 0:
            return 5.0
        return max(0.5, self.target / rate)

    # ------------------------------------------------------------ отправка # #
    def _apply(self, now, drive):
        _, signs = steering_targets(self.name, self.config["chassis"])
        level = int(round(self._level))
        if level <= 0 and not self._decel:
            return                       # рампа ещё не вышла из нуля
        targets = {module: int(signs[module] * level) for module in MODULES}
        heartbeat = now - self._sent_at >= self.SEND_INTERVAL_S
        # Сначала — общая команда, если всем колёсам ехать в одну сторону:
        # одна строка в порт вместо четырёх.
        if len(set(targets.values())) == 1 and heartbeat:
            percent = next(iter(targets.values()))
            self._sent = {m: percent for m in MODULES}
            self._sent_at = now
            self.percent = percent
            drive(None, percent)
            return
        changed = [m for m in MODULES if self._sent.get(m) != targets[m]]
        if not changed and not heartbeat:
            return
        self._sent_at = now
        for module in changed or list(MODULES):
            self.percent = targets[module]
            self._sent[module] = targets[module]
            drive(module, targets[module])

    def _stop(self, reason, now):
        """Погасить тягу и запомнить результат. Команда «b» — тормоз всем."""
        if now is None:
            now = self.clock()
        was = self.state
        self.stopped_at = now
        self.state = self.DONE if reason == "reached" else (
            self.CANCELLED if reason in ("cancelled", "estop", "manual") else self.FAILED)
        self.reason = reason
        self.finished_at = now
        self.percent = 0
        self._level = 0.0
        self._decel = False
        if reason == "reached":
            self.progress = 1.0
            self.message = "Готово: %.2f из %.2f." % (self.value, self.target)
            if self.mode == "odometry" and self._elapsed > 0.3 and self.value > 0.01:
                rate = self.value / self._elapsed
                preset = self.config["presets"][self.name]
                self.learned = {"rate": rate, "percent": float(self.work_percent)}
                self.config["learned"][self.name] = dict(self.learned)
        elif reason == "timeout":
            self.message = "Не доехал: стоп по таймауту."
        elif reason in ("cancelled", "estop", "manual"):
            self.message = "Манёвр остановлен."
        elif reason in ("underrun", "overshoot"):
            self.message = "Остановился не на цели: %.2f из %.2f — проверьте настройки." % (
                self.value, self.target)
        elif reason == "steering_timeout":
            self.message = "Колёса не успели встать в углы манёвра."
        elif reason == "not_homed":
            self.message = "У колеса нет нуля — сначала калибровка."
        elif reason == "module_offline":
            self.message = "Колесо не на связи."
        else:
            self.message = "Манёвр прерван."
        if was in self.ACTIVE:
            # Тормоз — всегда всем колёсам: манёвр мог ехать бортами в разные
            # стороны, и «остановить только левые» здесь означало бы разворот.
            self.brake()
