# -*- coding: utf-8 -*-
"""Манёвры тяги: «метр вперёд», «развернуться на 90°» и другие пресеты.

Пресет здесь — это не «держать 12 % две секунды», а *цель*: расстояние в
метрах или угол в градусах. Оператор нажимает кнопку, сервер сам:

1. ставит колёса в нужные углы и ждёт, пока модули доедут;
2. плавно (рампа) набирает рабочий процент тяги;
3. считает путь каждого колеса по переходам SC (счётчик прошивки v17);
4. гасит тягу заранее (тормозной участок) и останавливает моторы на цели.

Путь — переходы / PPR × окружность. Угол разворота оценивается из средней
дуги колёс и геометрии ``R = hypot(база/2, колея/2)``. Это колёсная
одометрия, не измерение движения корпуса: SC не показывает направление,
его предполагаем по команде, а скольжение и инерцию датчик не обнаруживает.
Все четыре счётчика и свежая телеметрия обязательны. Потеря или молчание
одного колеса останавливают манёвр; перехода к движению по времени нет.

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

#: AOVO Pro 350 W 36 V: номинальная шина 8,5″ = 215,9 мм. Эффективный
#: диаметр под нагрузкой нужно откалибровать; база и колея — только примеры.
#: https://www.aovofix.co.uk/products/aovo-pro-m365-es80-motor-v2
DEFAULT_CHASSIS = {
    "wheelDiameterMm": 215.9,
    "wheelbaseMm": 500.0,
    "trackMm": 450.0,
    "rampSeconds": 0.5,
}

#: Грубые оценки только для таймаута и начальной рампы, не для измерения пути.
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


def odometry_error(frames):
    """Вернуть причину отказа, если хотя бы один SC-счётчик недоступен.

    Телеметрия должна быть предварительно отфильтрована по свежести. Ноль RPM
    не доказывает исправность провода: молчание дополнительно контролируется
    уже после начала тяги.
    """
    for module in MODULES:
        frame = frames.get(module)
        if not frame:
            return "module_offline"
        if type(frame.get("firmware")) is not int or frame["firmware"] < 17:
            return "sc_firmware_required"
        count, ppr, uptime, rpm = (frame.get("scCount"), frame.get("scPpr"),
                                   frame.get("uptimeMs"), frame.get("scRpm"))
        if (type(count) is not int or not 0 <= count <= 0xFFFFFFFF or
                type(ppr) is not int or not 1 <= ppr <= 1000 or
                type(uptime) is not int or not 0 <= uptime <= 0xFFFFFFFF or
                not isinstance(rpm, (int, float)) or isinstance(rpm, bool) or
                not math.isfinite(rpm) or not 0 <= rpm <= 404000.0 / ppr + 0.1):
            return "sc_invalid"
    return None


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
    #: Переходы могут быть редкими на малом ходу, но ждать бесконечно нельзя.
    TACH_GRACE_S = 3.0
    #: Аппаратный фильтр 150 мкс (≈6667 переходов/с); запас на задержку кадра.
    MAX_EDGE_HZ = 6700.0
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
        self._sc = {}                    # последний счётчик, PPR и uptime каждого колеса
        self._brake_id = None           # точный ACK начального STOP всех четырёх плат
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
                "sensor": "sc_count",
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
            if name == "chassis":
                self.config["learned"].clear()
            return {"ok": True, "config": self.config_payload()}

    # ------------------------------------------------------------------ старт #
    def start(self, name, now, limit_percent=None, brake_id=None):
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
            self._brake_id = brake_id
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
        error = odometry_error(frames)
        if error:
            self._stop(error, now)
            return
        if self._brake_id is not None:
            if any(type(frames[m].get("brakeId")) is not int for m in MODULES):
                self._stop("brake_firmware_required", now)
                return
            if any(frames[m]["brakeId"] != self._brake_id for m in MODULES):
                if now - self.started_at > self.STEER_TIMEOUT_S:
                    self._stop("brake_unconfirmed", now)
                return
            if any(frames[m].get("drivePercent") != 0 for m in MODULES):
                self._stop("drive_mismatch", now)
                return
        if now - self.started_at > self.STEER_TIMEOUT_S:
            self._stop("steering_timeout", now)
            return
        # Нельзя рулить, пока SC ещё показывает выбег. Ноль SC сам по себе
        # не подтверждает провод: это контролируется при движении.
        if any(frames[m]["scRpm"] > 1.0 for m in MODULES):
            return
        arrived = all(_angle_error(frames[m].get("deg") or 0.0, targets[m])
                      <= self.STEER_TOLERANCE_DEG and not frames[m].get("moving")
                      for m in MODULES)
        if not arrived:
            if now - self._steer_at >= self.STEER_RESEND_S:
                self._steer_at = now
                for module in MODULES:
                    if frames[module].get("moving"):
                        continue
                    try:
                        result = steer(module, targets[module])
                    except Exception:
                        result = False
                    if result is False or (isinstance(result, dict) and not result.get("ok")
                                           and result.get("reason") != "module_busy"):
                        self._stop("command_failed", now)
                        return
            return
        # Колёса на месте — начинаем движение с нуля измерений.
        self.state = self.DRIVING
        self._elapsed = 0.0
        self._travel = {m: 0.0 for m in MODULES}
        self._sc = {m: {"count": frames[m]["scCount"], "ppr": frames[m]["scPpr"],
                        "uptime": frames[m]["uptimeMs"], "sampleAt": now,
                        "pulseAt": now} for m in MODULES}
        self._sent = {}
        self.mode = "odometry"
        self._rate = self._rate_estimate()
        self._speed = 0.0
        self._last_value = 0.0
        self._deadline = now + self._expected_seconds() * 2.5 + 5.0
        self.message = "Едем по счётчикам SC; направление предполагается по команде."

    def _drive_phase(self, now, frames, drive, dt):
        self._elapsed += dt
        error = odometry_error(frames)
        if error:
            self._stop(error, now)
            return
        targets, signs = steering_targets(self.name, self.config["chassis"])
        for module in MODULES:
            frame = frames[module]
            if not frame.get("cal") or frame.get("moving") or frame.get("calib") or frame.get("cycle"):
                self._stop("steering_changed", now)
                return
            angle = frame.get("deg")
            if not isinstance(angle, (int, float)) or not math.isfinite(angle) or \
                    _angle_error(angle, targets[module]) > self.STEER_TOLERANCE_DEG:
                self._stop("steering_changed", now)
                return
            reported = frame.get("drivePercent")
            if reported is not None and reported * signs[module] < 0:
                self._stop("drive_mismatch", now)
                return
        measured, error = self._measure(now, frames)
        if error:
            self._stop(error, now)
            return
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
        try:
            self._apply(now, drive)
        except Exception:  # отказ отправки не должен оставлять предыдущую тягу
            self._stop("command_failed", now)

    def _target_wheel_path(self):
        preset = self.config["presets"][self.name]
        return (self.target if preset["kind"] == "distance" else
                math.radians(self.target) * turn_radius_m(self.config["chassis"]))

    def _arrival_reason(self, tolerance):
        # Среднее не должно скрыть полностью неработающий датчик на одном колесе.
        if min(self._travel.values()) < self._target_wheel_path() * 0.5:
            return "sc_mismatch"
        error = self.value - self.target
        if abs(error) <= tolerance:
            return "reached"
        return "underrun" if error < 0 else "overshoot"

    # ------------------------------------------------------------- измерение #
    def _measure(self, now, frames):
        circumference = wheel_circumference_m(self.config["chassis"]["wheelDiameterMm"])
        for module in MODULES:
            frame, previous = frames[module], self._sc[module]
            count, uptime = frame["scCount"], frame["uptimeMs"]
            if frame["scPpr"] != previous["ppr"]:
                return None, "sc_changed"
            elapsed = max(0.0, now - previous["sampleAt"])
            dt_ms = (uptime - previous["uptime"]) & 0xFFFFFFFF
            delta = (count - previous["count"]) & 0xFFFFFFFF
            # Перезагрузка платы или порванный/переставленный счётчик нельзя
            # принять за километры пути. Оба счётчика допускают переполнение.
            if ((not dt_ms and delta) or dt_ms > elapsed * 1000.0 + 1250.0 or
                    delta > self.MAX_EDGE_HZ * (elapsed + 0.5) + 2):
                return None, "sc_changed"
            if dt_ms:
                previous.update(count=count, uptime=uptime, sampleAt=now)
                if delta:
                    self._travel[module] += circumference * delta / previous["ppr"]
                    previous["pulseAt"] = now
            if now - previous["pulseAt"] >= self.TACH_GRACE_S:
                return None, "sc_stalled"
        # Если цель уже пройдена одними колёсами, не продолжаем ждать другие.
        paths = list(self._travel.values())
        if max(paths) >= self._target_wheel_path() and min(paths) < max(paths) * 0.35:
            return None, "sc_mismatch"
        return self._odometry_value(), None

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
            result = drive(None, percent)
            if result is False or (isinstance(result, dict) and not result.get("ok")):
                self._stop("command_failed", now)
            return
        changed = [m for m in MODULES if self._sent.get(m) != targets[m]]
        if not changed and not heartbeat:
            return
        self._sent_at = now
        for module in changed or list(MODULES):
            self.percent = targets[module]
            self._sent[module] = targets[module]
            result = drive(module, targets[module])
            if result is False or (isinstance(result, dict) and not result.get("ok")):
                self._stop("command_failed", now)
                return

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
            self.message = "Потеряна свежая телеметрия колеса — тяга снята."
        elif reason == "sc_firmware_required":
            self.message = "Для одометрии нужна прошивка v17 на всех четырёх платах."
        elif reason in ("brake_firmware_required", "brake_unconfirmed"):
            self.message = "Нет подтверждения тормоза от четырёх плат — движение не начато."
        elif reason in ("sc_invalid", "sc_changed"):
            self.message = "Счётчик SC неверен или плата перезапущена — тяга снята."
        elif reason == "sc_stalled":
            self.message = "Нет переходов SC одного из колёс — тяга снята."
        elif reason == "sc_mismatch":
            self.message = "Пути колёс слишком различаются — тяга снята, цель не подтверждена."
        elif reason in ("steering_changed", "drive_mismatch", "command_failed"):
            self.message = "Руль или команда тяги не подтверждены — тяга снята."
        else:
            self.message = "Манёвр прерван."
        if was in self.ACTIVE:
            # Тормоз — всегда всем колёсам: манёвр мог ехать бортами в разные
            # стороны, и «остановить только левые» здесь означало бы разворот.
            self.brake()
