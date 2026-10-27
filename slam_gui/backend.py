#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
backend.py — сервер борта RUS SLAM: отдаёт экраны и данные любому браузеру.

Ничего не открывает и не показывает сам: робот лишь отвечает по сети, а
страницы смотрят в браузере — на дисплее робота, планшете или ноутбуке.
Один процесс отдаёт оба экрана и данные к ним:

    /                     → основной экран робота (main.html, киоск)
    /index.html, /console → инженерный пульт (index.html)
    /api/state            → состояние: двигатели, АКБ, груз, замок, связь
    /api/lock/open        → открыть грузовой отсек по PIN-коду
    /api/lock/close       → закрыть отсек
    /api/lock/pin         → сменить PIN-код
    /api/audit            → журнал доступа (аудит с хеш-цепочкой)
    /api/health           → проверка живости

Источники данных (выбираются автоматически или флагом):
    --source none     робот не подключён: все значения пустые (null), без демо;
    --source serial   реальные модули: кадры телеметрии 16 Б по /dev/ttyUSB*
                      (протокол: docs/SERIAL_PROTOCOL.md, CRC16/Modbus);
    --source master   четыре колеса по одному USB каждое: @TLM/@ID опознают
                      какое колесо на каком порту, поворот адресный (w FL 12.5);
    --source ros      ROS 2 (rclpy): /modules/state, /battery, /cargo/lock.

Замок: PIN-код хранится как SHA-256(salt + pin) в gui/state/lock.json,
5 неудачных попыток → блокировка 30 с, каждое событие — в журнал аудита.
Реальный замок вызывается внешней командой:

    RUS_SLAM_LOCK_CMD='gpio write 7 1'   (open)   /  RUS_SLAM_LOCK_CMD_CLOSE  (close)

Примеры запуска:
    python3 gui/backend.py                          # только сервер (штатный режим)
    python3 gui/backend.py --source serial --ports /dev/ttyUSB0,/dev/ttyUSB1,/dev/ttyUSB2,/dev/ttyUSB3
    python3 gui/backend.py --source ros --pin 7232
    python3 gui/backend.py --kiosk                   # опция: дисплей на борту, браузер без рамок
    python3 gui/backend.py --open                    # опция: открыть страницу в браузере робота

Кто где смотрит:
    на роботе     http://127.0.0.1:8080/            основной экран
    планшет/ПК    http://<ip-робота>:8080/          основной экран (оператор, получатель)
    инженер       http://<ip-робота>:8080/console   инженерный пульт, удалённо
"""

import argparse
import hashlib
import json
import math
import os
import queue
import re
import secrets
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
import webbrowser
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

ROOT = os.path.dirname(os.path.abspath(__file__))
STATE_DIR = os.path.join(ROOT, "state")
LOCK_FILE = os.path.join(STATE_DIR, "lock.json")
MAIN_PAGE = "/main.html"
CONSOLE_PAGE = "/index.html"

DEFAULT_PIN = "7232"
MAX_ATTEMPTS = 5
LOCK_MS = 30_000

# --- Протокол (docs/SERIAL_PROTOCOL.md) -------------------------------------
CMD_SYNC = (0xAA, 0x55)
TLM_SYNC = (0xBB, 0x44)
CMD_LEN, TLM_LEN = 10, 16
STATUS_ENABLED, STATUS_HOMED, STATUS_FAULT = 0x01, 0x02, 0x04
MODULE_NAMES = {1: "FL", 2: "FR", 3: "RL", 4: "RR"}
MODULE_TITLES = {
    "FL": "передний левый", "FR": "передний правый",
    "RL": "задний левый", "RR": "задний правый",
}
WHEEL_R_M = 0.127            # радиус колеса Xiaomi M365 Pro, м
MAX_DRIVE_PERCENT = 20       # заводской предел; плата может прислать свой (dmax=)
#: Жёсткий потолок сервера: выше него тягу не пропустит ни экран, ни API, даже
#: если плата разрешила себе больше. Заводской предел 20 % остаётся настройкой
#: по умолчанию — поднимать его можно только осознанно, на вывешенных колёсах.
DRIVE_ABS_MAX_PERCENT = 100
DRIVE_ANGLE_ALIGNMENT_DEG = 2.5
#: Выход на регулятор скорости: ШИМ (процент = скважность) или 0–5 В
#: (analogWrite + фильтр/ЦАП). Совпадает с drive_mode в прошивке.
DRIVE_OUTPUT_MODES = ("pwm", "volt")
DRIVE_MV_FULL = 5000

# Предел абсолютной команды поворота «w <модуль> <угол>». Одно число на весь
# стек: столько же в прошивке (STEER_LIMIT_DEG в firmware/motor_wheel),
# в robot_control/slam.py (STEERING_ANGLE_LIMIT_DEG) и в экранах
# (slam_gui/steer_presets.js → LIMIT_DEG). Механика позволяет больше — упор
# потенциометра ±130°, программный предел хода STEER_TRAVEL_LIMIT_DEG = 125°,
# поэтому 100° безопасно: чистый боковой ход — это 90°, а дальше идёт запас
# на диагонали. Рассинхрон этих чисел и давал «кривое» руление: экран
# отправлял угол, который сервер или плата резали.
STEER_LIMIT_DEG = 100.0

# --- АКБ 12S3P LiFePO4 (совпадает с gui/console-core.js) --------------------
PACK = {
    "series": 12, "parallel": 3, "cellAh": 6.2,
    "nominalV": 38.4, "fullV": 43.8, "emptyV": 30.0,
    "capacityAh": 18.6, "capacityWh": 714.2,
    "internalR": 0.075, "bmsA": 60, "fuseA": 25,
    "lowV": 35.5, "criticalV": 33.5, "tempWarnC": 45, "tempMaxC": 60,
    "ocv": [(2.50, 0), (2.80, 3), (3.00, 7), (3.10, 10), (3.20, 15), (3.25, 30),
            (3.30, 50), (3.33, 70), (3.35, 85), (3.40, 95), (3.45, 98), (3.65, 100)],
}


# ============================================================================
# 1. Протокол обмена
# ============================================================================
def crc16(data):
    """CRC16/Modbus (0xA001, init 0xFFFF). Эталон: b'123456789' → 0x4B37."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte & 0xFF
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def build_command(module_id, steer_cdeg=0, pwm=0, enable=False, home=False):
    """Кадр команды ПК → модуль, 10 байт."""
    steer = max(-32767, min(32767, int(round(steer_cdeg))))
    duty = max(-1000, min(1000, int(round(pwm))))
    flags = (0x01 if enable else 0) | (0x02 if home else 0)
    body = bytes([CMD_SYNC[0], CMD_SYNC[1], module_id & 0xFF]) + \
        struct.pack(">hh", steer, duty) + bytes([flags])
    crc = crc16(body)
    return body + struct.pack(">H", crc)


def parse_telemetry(frame):
    """Кадр телеметрии 16 Б → dict, либо None (длина/синхро/CRC)."""
    if len(frame) < TLM_LEN:
        return None
    if frame[0] != TLM_SYNC[0] or frame[1] != TLM_SYNC[1]:
        return None
    if crc16(frame[:14]) != struct.unpack(">H", frame[14:16])[0]:
        return None
    steer_cdeg, enc_delta, pwm_actual = struct.unpack(">hhh", frame[4:10])
    vbat_cv, current_ma = struct.unpack(">HH", frame[10:14])
    status = frame[3]
    return {
        "moduleId": frame[2],
        "id": MODULE_NAMES.get(frame[2], "M%d" % frame[2]),
        "status": status,
        "enabled": bool(status & STATUS_ENABLED),
        "homed": bool(status & STATUS_HOMED),
        "fault": bool(status & STATUS_FAULT),
        "steerDeg": steer_cdeg / 100.0,
        "encDelta": enc_delta,
        "pwm": pwm_actual,
        "volts": vbat_cv / 100.0,
        "amps": current_ma / 1000.0,
    }


def pick_telemetry(buf):
    """Ищет в буфере корректный кадр. → (кадр | None, остаток буфера)."""
    while len(buf) >= TLM_LEN:
        start = buf.find(bytes(TLM_SYNC))
        if start < 0:
            return None, buf[-1:]          # синхробайтов нет — держим хвост
        if start:
            buf = buf[start:]
        if len(buf) < TLM_LEN:
            return None, buf
        frame = buf[:TLM_LEN]
        parsed = parse_telemetry(frame)
        if parsed:
            return parsed, buf[TLM_LEN:]
        buf = buf[1:]                      # CRC не сошёлся — сдвигаемся на байт
    return None, buf


# ============================================================================
# 2. Модель АКБ 12S3P
# ============================================================================
def _interp(table, x, col):
    key, val = (0, 1) if col == 0 else (1, 0)
    if x <= table[0][key]:
        return table[0][val]
    if x >= table[-1][key]:
        return table[-1][val]
    for i in range(1, len(table)):
        a, b = table[i - 1], table[i]
        if x <= b[key]:
            k = (x - a[key]) / (b[key] - a[key] or 1)
            return a[val] + k * (b[val] - a[val])
    return table[-1][val]


def soc_from_voltage(pack_v):
    """SOC, % — по кривой OCV на элемент."""
    return max(0.0, min(100.0, _interp(PACK["ocv"], (pack_v or 0) / PACK["series"], 0)))


def voltage_from_soc(soc):
    return _interp(PACK["ocv"], max(0.0, min(100.0, soc)), 1) * PACK["series"]


def pack_state(pack_v, pack_a, soc=None, temp_c=28.0):
    """Сводка по АКБ для экрана: %, напряжение, ток, запас хода, состояние."""
    soc = soc_from_voltage(pack_v) if soc is None else max(0.0, min(100.0, soc))
    remaining_wh = PACK["capacityWh"] * soc / 100.0
    wh_per_km = PACK["capacityWh"] / 22.0          # паспортные ~22 км на полный заряд
    if pack_v >= PACK["lowV"]:
        level = "НОРМА"
    elif pack_v >= PACK["criticalV"]:
        level = "НИЗКИЙ"
    else:
        level = "КРИТИЧЕСКИЙ"
    return {
        "soc": round(soc, 1),
        "volts": round(pack_v, 2),
        "amps": round(abs(pack_a), 2),
        "watts": round(pack_v * abs(pack_a), 1),
        "remainingWh": round(remaining_wh, 1),
        "rangeKm": round(remaining_wh / wh_per_km, 1),
        "state": "заряд" if pack_a < -0.2 else ("разряд" if pack_a > 0.2 else "покой"),
        "level": level,
        "tempC": round(temp_c, 1),
        "cellMin": round(pack_v / PACK["series"] - 0.011, 3),
        "cellMax": round(pack_v / PACK["series"] + 0.013, 3),
        "thresholds": {"lowV": PACK["lowV"], "criticalV": PACK["criticalV"]},
    }


# ============================================================================
# 3. Источники данных
# ============================================================================
class NoSource:
    """Робот не подключён: ни одного выдуманного значения, все поля пустые (null)."""

    name = "none"

    def read(self):
        motors = [{"id": mid, "title": MODULE_TITLES[mid], "angle": None, "rpm": None,
                   "temp": None, "homed": None, "online": False, "fault": None, "pot": None}
                  for mid in MODULE_NAMES.values()]
        battery = {"soc": None, "volts": None, "amps": None, "watts": None,
                   "remainingWh": None, "rangeKm": None, "state": None, "level": None,
                   "tempC": None, "cellMin": None, "cellMax": None,
                   "thresholds": {"lowV": PACK["lowV"], "criticalV": PACK["criticalV"]}}
        return {"motors": motors, "battery": battery, "cargo": {},
                "mode": None, "route": None, "speedMps": None, "powerKw": None,
                "drive": {"available": False, "canDrive": False, "reason": "source_unavailable",
                          "maxPercent": MAX_DRIVE_PERCENT, "commandedPercent": 0,
                          "commandedModules": {}, "individual": False, "perModule": {},
                          "active": False, "reportedPercent": {},
                          "config": {"supported": False, "outputMode": None, "tach": None,
                                     "maxPercent": None, "minMv": None, "maxMv": None,
                                     "outputMv": None, "brake": None, "dirReverse": None}},
                "linkOk": False}


TLM_PREFIX = "@TLM "


#: Откуда у модуля ноль (поле home= прошивки v2): датчик D18, вручную или нет.
HOME_SOURCES = ("opto", "manual", "none")
#: Коды ошибок калибровки прошивки (поле err=) → текст для экрана и журнала.
CALIB_ERRORS = {
    1: "датчик не найден в ±135° от старта",
    2: "датчик не освобождается (залип или флажок шире 30°)",
    3: "кромка не найдена при медленном подходе",
    4: "калибровка прервана командой стоп",
    5: "калибровка не уложилась по времени",
    6: "кромки расходятся между подходами",
}


def parse_master_line(line, module=None):
    """Строка монитора мастера "@TLM mod=FL deg=-9.25 moving=1 ..." → dict.

    Остальные строки монитора (текст для человека) возвращают None.
    """
    line = (line or "").strip()
    if not line.startswith(TLM_PREFIX):
        return None
    fields = {}
    for token in line[len(TLM_PREFIX):].split():
        key, sep, value = token.partition("=")
        if sep:
            fields[key] = value
    try:
        module = module or fields["mod"].upper()
        angle = float(fields["deg"])
        target = float(fields.get("tgt", "0"))
        rpm = float(fields["rpm"]) if "rpm" in fields else None
        drive_percent = int(fields["drive"]) if "drive" in fields else None
        firmware = int(fields["fw"]) if "fw" in fields else None
        calib_error = int(fields["err"]) if "err" in fields else None
        home = fields.get("home")
        # Гибкая настройка тяги (прошивка с командой m): режим выхода, тахометр,
        # предел, мёртвая зона и фактическое напряжение выхода.
        drive_mode = {"0": "pwm", "1": "volt"}.get(fields.get("dm")) if "dm" in fields else None
        tach = {"0": False, "1": True}.get(fields.get("tach")) if "tach" in fields else None
        drive_max = int(fields["dmax"]) if "dmax" in fields else None
        try:
            drive_min_mv = int(fields["vmin"]) if "vmin" in fields else None
            drive_max_mv = int(fields["vmax"]) if "vmax" in fields else None
            drive_output_mv = int(fields["out"]) if "out" in fields else None
        except ValueError:
            return None
        brake = {"0": False, "1": True}.get(fields.get("brk")) if "brk" in fields else None
        dir_reverse = {"0": False, "1": True}.get(fields.get("dir")) if "dir" in fields else None
        for key, value in (("dm", drive_mode), ("tach", tach), ("brk", brake), ("dir", dir_reverse)):
            if key in fields and value is None:
                return None
        # Обороты и тягу проверяем по тому же пределу, что разрешила плата:
        # старая прошивка (без dmax=) не могла превысить заводские 20 %.
        drive_ceiling = drive_max if drive_max is not None else MAX_DRIVE_PERCENT
        if (module not in MODULE_TITLES or not math.isfinite(angle) or
                not math.isfinite(target) or (rpm is not None and (not math.isfinite(rpm) or rpm < 0)) or
                (drive_percent is not None and abs(drive_percent) > drive_ceiling) or
                (calib_error is not None and not 0 <= calib_error <= 15) or
                (home is not None and home not in HOME_SOURCES) or
                (drive_max is not None and not 1 <= drive_max <= DRIVE_ABS_MAX_PERCENT) or
                (drive_min_mv is not None and not 0 <= drive_min_mv <= DRIVE_MV_FULL) or
                (drive_max_mv is not None and not 1 <= drive_max_mv <= DRIVE_MV_FULL) or
                (drive_min_mv is not None and drive_max_mv is not None and drive_min_mv > drive_max_mv) or
                (drive_output_mv is not None and not 0 <= drive_output_mv <= DRIVE_MV_FULL)):
            return None
        zeroed = fields.get("cal") == "1"
        if home is None:
            # Прошивка без поля home= знала только ручной программный ноль.
            home = "manual" if zeroed else "none"
        return {
            "mod": module,
            "deg": angle,
            "tgt": target,
            "moving": fields.get("moving") == "1",
            # cal — у модуля есть ноль: по оптодатчику D18 (k / «Калибровка»)
            # или вручную («Задать 0°»). После перезапуска платы ноля нет.
            "cal": zeroed,
            "home": home if zeroed else "none",
            "manualZero": zeroed and home == "manual",
            "rpm": rpm,
            "drivePercent": drive_percent,
            "cycle": fields.get("cycle") == "1",
            "calib": fields.get("calib") == "1",
            "opto": fields.get("opto") == "1",
            "calibError": calib_error,
            "firmware": firmware,
            # pot=1 — потенциометр A0 откалиброван (q), доводка положения включена.
            # Поле появилось в прошивке v2 после руля; старые кадры его не содержат.
            "pot": fields.get("pot") == "1",
            "potMode": fields.get("pm") == "1" if fields.get("pm") in {"0", "1"} else None,
            # Настройка тяги: None у прошивки без команды m (до гибкого привода).
            "driveMode": drive_mode,
            "tach": tach,
            "driveMaxPercent": drive_max,
            "driveMinMv": drive_min_mv,
            "driveMaxMv": drive_max_mv,
            "driveOutputMv": drive_output_mv,
            "brake": brake,
            "dirReverse": dir_reverse,
        }
    except (KeyError, ValueError):
        return None


class _WheelLink:
    """Открытый USB-порт колеса: очередь команд и привязанное имя модуля."""

    def __init__(self, path):
        self.path = path
        self.device = os.path.realpath(path)
        self.rx = bytearray()
        self.rx_discard = False
        self.module = None               # опознанное колесо (FL/FR/RL/RR)
        self.commands = queue.Queue(maxsize=16)


#: Порт не занимался межпроцессным замком: callback не передан (отдельный
#: запуск без пульта) — ведём себя как раньше, без замков.
_UNCLAIMED = object()


class MasterSource:
    """Четыре USB-колеса, только ручное назначение ``module_ports``.

    Пустой порт отключает колесо. Не выбранные оператором устройства не
    открываются, имена плат и ответы @ID не определяют назначение порта.
    Свежая телеметрия @TLM всё ещё обязательна для команд; межпроцессный
    замок, остановка тяги и watchdog не зависят от имени колеса в прошивке.
    """

    name = "master"
    supports_individual_steer = True
    supports_drive = True
    #: Каждое колесо — своё USB и своя плата, поэтому тягу можно давать одному
    #: колесу («t FL 8»), не трогая остальные: так делают разворот на месте и
    #: проверку одного мотора.
    supports_individual_drive = True
    supports_calibration = True
    STALE_S = 1.0
    DRIVE_STALE_S = 0.75
    MAX_STEER_DEG = STEER_LIMIT_DEG
    MAX_DRIVE_PERCENT = 20
    MESSAGE_LIMIT = 40
    #: Сколько ждать строку @TLM после открытия порта (плата перезагружается).
    PROBE_S = 5.0
    #: Пауза перед повторным открытием порта, с которого ничего не пришло.
    RETRY_S = 2.0
    # Recover a wedged USB stream, not an ordinary missed 250 ms frame.
    TELEMETRY_TIMEOUT_S = 10.0
    MAX_LINE_BYTES = 1024
    #: Как часто менеджер пересматривает список портов-кандидатов.
    RESCAN_S = 2.0
    #: Как часто обновляем отметку «здесь колесо» (её срок жизни — 60 с).
    TAG_REFRESH_S = 15.0

    def __init__(self, port=None, baud=115200, on_message=None, on_status=None,
                 port_provider=None, probe=None, claim=None, release=None,
                 yield_to_lidar=None, tag_module=None, untag_module=None,
                 module_ports=None):
        # Legacy positional ports/providers are deliberately NOT candidates.
        # Without a wheel-to-port assignment, never open any device.
        self.module_ports = {}
        seen = set()
        for module, path in (module_ports or {}).items():
            if not path:
                continue
            if module not in MODULE_TITLES:
                raise ValueError("Неизвестное колесо: %s" % module)
            real = os.path.realpath(path)
            if real in seen:
                raise ValueError("Один USB-порт нельзя назначить двум колёсам")
            seen.add(real)
            self.module_ports[module] = path
        ports = list(self.module_ports.values())
        self.configured_port = ports[0] if ports else None
        self.port = None                 # первый опознанный порт (для экрана)
        self.ports = ports
        self.candidates = [p for p in ports if os.path.exists(p)]
        self.baud = baud
        self.lock = threading.Lock()
        self.frames = {}                 # модуль → (последний кадр, время)
        self.modules = {}                # модуль → порт (кто где опознан)
        self.links = {}                  # порт → _WheelLink открытого порта
        self.drive_percent = 0
        #: Тяга отдельных колёс: модуль → последний отправленный процент.
        #: Пусто, пока едет «вся телега» общей командой t N.
        self.drive_modules = {}
        self.drive_last_command = 0.0
        self.commands = queue.Queue(maxsize=16)
        self.serial_ready = threading.Event()
        self.messages = []               # строки «# FL: …» прошивки: (время, текст)
        self.on_message = on_message     # их же — в журнал экрана
        self.on_status = on_status       # «порт не найден / колёса на связи»
        #: Межпроцессный замок порта (см. docstring класса): ``claim(port)`` →
        #: владение или ``None`` (порт занят лидаром/вторым процессом),
        #: ``release(владение)`` — снять.
        self.claim = claim
        self.release = release
        #: ``yield_to_lidar(port)`` → True, пока порт нужен лидару.
        self.yield_to_lidar = yield_to_lidar
        #: Отметка «на этом порту подтвердилось колесо» (её видит лидар).
        self.tag_module = tag_module
        self.untag_module = untag_module
        self.last_error = ""
        self._last_status = None
        self._stop = False
        self._serial = None
        self._spawning = set()
        self._cooldown = {}              # порт → время следующей попытки открыть
        threading.Thread(target=self._reader, daemon=True).start()

    @staticmethod
    def _port_error_hint(exc):
        """Понятная причина ошибки pyserial — прямо в журнал экрана."""
        low = str(exc or "").lower()
        if "permission" in low or "access is denied" in low:
            return " — нет прав: sudo usermod -aG dialout $USER, затем войдите заново"
        if ("exclusively lock" in low or "resource temporarily unavailable" in low
                or "busy" in low):
            return " — порт занят другой программой (монитор порта или второй процесс)"
        return ""

    def _note_status(self, level, text):
        """Сообщить о порте один раз, а не каждые две секунды."""
        text = str(text or "").strip()
        if not text:
            return
        with self.lock:
            if text == self._last_status:
                return
            self._last_status = text
        callback = getattr(self, "on_status", None)
        if callback is not None:
            try:
                callback(level, text)
            except Exception:                 # noqa: BLE001 — журнал не ломает чтение
                pass

    def feed(self, line, now=None, module=None):
        """Принимает строку монитора; True, если это телеметрия.

        Строки «# FL: …» — сообщения прошивки для человека (калибровка, ошибки
        датчика): они сохраняются и уходят в журнал.
        """
        now = time.time() if now is None else now
        frame = parse_master_line(line, module=module)
        if frame is None:
            text = (line or "").strip()
            if text.startswith("# ") and len(text) > 2:
                self._note_message(text[2:], now)
            return False
        with self.lock:
            self.frames[frame["mod"]] = (frame, now)
        return True

    def _note_message(self, text, now):
        messages = getattr(self, "messages", None)
        if messages is None:
            return
        with self.lock:
            messages.append((now, text))
            del messages[:-self.MESSAGE_LIMIT]
        callback = getattr(self, "on_message", None)
        if callback is not None:
            try:
                callback(text)
            except Exception:                 # noqa: BLE001 — журнал не ломает чтение порта
                pass

    def recent_messages(self, limit=10):
        with self.lock:
            items = list(getattr(self, "messages", None) or [])[-limit:]
        return [{"ts": round(ts, 3), "text": text} for ts, text in items]

    # --------------------------------------------------------------- команды #
    @staticmethod
    def _command_module(command):
        """Какому колесу адресована команда: «w FL 12.5» или «t FL 8» → ``FL``.

        Прошивка колеса и так игнорирует чужое имя, но сервер предпочитает
        не слать лишние байты: адресная команда уходит на свой порт. Имя
        снимается перед отправкой (см. :meth:`_fanout_commands`), потому что
        прошивка ждёт «w 12.5» и «t 8».
        """
        try:
            text = bytes(command).decode("ascii", "ignore").strip()
        except Exception:                     # noqa: BLE001 — битые байты не команда
            return None
        parts = text.split()
        if len(parts) >= 2 and parts[0].lower() in ("w", "t", "m"):
            token = parts[1].upper()
            if token in MODULE_TITLES:
                return token
        return None

    def _module_drive_active(self, module=None):
        """Идёт ли тяга: общая (все колёса) или конкретного колеса."""
        if self.drive_percent != 0:
            return True
        values = getattr(self, "drive_modules", None) or {}
        if module is None:
            return any(percent != 0 for percent in values.values())
        return values.get(module, 0) != 0

    def set_pot_mode(self, action, now=None):
        """Режим потенциометра всех четырёх плат: pm 0/1 или калибровка q."""
        if not self.serial_ready.is_set():
            return {"ok": False, "reason": "serial_unavailable"}
        now = time.time() if now is None else now
        with self.lock:
            frames = dict(self.frames)
            drive_active = self._module_drive_active()
        if any(mid not in frames or now - frames[mid][1] >= self.STALE_S for mid in MODULE_TITLES):
            return {"ok": False, "reason": "module_offline"}
        values = [frames[mid][0] for mid in MODULE_TITLES]
        if any(f.get("potMode") is None for f in values):
            return {"ok": False, "reason": "firmware_update_required"}
        if drive_active or any(f.get("moving") or f.get("calib") or f.get("cycle") or f.get("drivePercent") for f in values):
            return {"ok": False, "reason": "module_busy"}
        if action == "calibrate" and any(not f.get("potMode") or not f.get("cal") for f in values):
            return {"ok": False, "reason": "zero_required"}
        command = {"on": b"pm 1\n", "off": b"pm 0\n", "calibrate": b"q\n"}.get(action)
        if command is None:
            return {"ok": False, "reason": "invalid_action"}
        try:
            self.commands.put_nowait(command)
        except queue.Full:
            return {"ok": False, "reason": "command_queue_full"}
        return {"ok": True, "queued": True}

    def calibrate_module(self, module, now=None):
        """Калибровка по оптодатчику одного модуля (FL/FR/RL/RR) или всех (ALL).

        Колесо ищет флажок в пределах ±125° от текущего положения (ход руля
        270°): сначала вправо, затем влево. Команда принимается только при
        выключенной тяге и стоящем модуле. После включения Arduino колёса
        выставляют прямо руками: прошивка не знает, где колесо стоит.
        """
        module = str(module or "").upper()
        if module not in MODULE_TITLES and module != "ALL":
            return {"ok": False, "reason": "invalid_module"}
        if not self.serial_ready.is_set():
            return {"ok": False, "reason": "serial_unavailable"}
        now = time.time() if now is None else now
        with self.lock:
            frames = dict(self.frames)
            drive_active = self._module_drive_active()
        if drive_active:
            return {"ok": False, "reason": "module_busy"}
        targets = list(MODULE_TITLES) if module == "ALL" else [module]
        fresh = {mid: frames[mid][0] for mid in targets
                 if mid in frames and now - frames[mid][1] < self.STALE_S}
        if not fresh:
            return {"ok": False, "reason": "module_offline"}
        if module != "ALL" and module not in fresh:
            return {"ok": False, "reason": "module_offline"}
        if any(frame.get("calib") or frame.get("moving") or frame.get("cycle")
               or (frame.get("drivePercent") or 0) != 0 for frame in fresh.values()):
            return {"ok": False, "reason": "module_busy"}
        command = b"k\n" if module == "ALL" else ("w %s calib\n" % module).encode("ascii")
        try:
            self.commands.put_nowait(command)
        except queue.Full:
            return {"ok": False, "reason": "command_queue_full"}
        return {"ok": True, "queued": True, "module": module,
                "offline": sorted(set(targets) - set(fresh))}

    def steer_module(self, module, angle, now=None):
        """Поставить в очередь только поворот одного откалиброванного модуля."""
        module = str(module or "").upper()
        try:
            angle = float(angle)
        except (TypeError, ValueError):
            return {"ok": False, "reason": "invalid_angle"}
        if module not in MODULE_TITLES:
            return {"ok": False, "reason": "invalid_module"}
        if not math.isfinite(angle) or abs(angle) > self.MAX_STEER_DEG:
            # Предел возвращается в ответе: экран показывает настоящее число,
            # а не зашитое в подписи кнопки.
            return {"ok": False, "reason": "angle_limit", "limit": self.MAX_STEER_DEG}
        if not self.serial_ready.is_set():
            return {"ok": False, "reason": "serial_unavailable"}

        now = time.time() if now is None else now
        with self.lock:
            entry = self.frames.get(module)
            # Крутить руль можно только у стоящего колеса: прошивка тоже это
            # запрещает («руль заблокирован: включена тяга»), но экран должен
            # получить понятный отказ, а не молчание.
            drive_active = self._module_drive_active(module)
        if drive_active:
            return {"ok": False, "reason": "module_busy"}
        if entry is None or now - entry[1] >= self.STALE_S:
            return {"ok": False, "reason": "module_offline"}
        frame = entry[0]
        if not frame.get("cal"):
            return {"ok": False, "reason": "not_homed"}
        if frame.get("moving") or frame.get("calib") or frame.get("cycle"):
            return {"ok": False, "reason": "module_busy"}

        # Отдельная команда поворота одного модуля, без команды тяги.
        command = ("w %s %.1f\n" % (module, angle)).encode("ascii")
        try:
            self.commands.put_nowait(command)
        except queue.Full:
            return {"ok": False, "reason": "command_queue_full"}
        return {"ok": True, "queued": True, "module": module, "angle": round(angle, 1)}

    def zero_module(self, module, now=None):
        """Назначить текущую позицию программным нулём; мотор не двигается."""
        module = str(module or "").upper()
        if module not in MODULE_TITLES:
            return {"ok": False, "reason": "invalid_module"}
        if not self.serial_ready.is_set():
            return {"ok": False, "reason": "serial_unavailable"}

        now = time.time() if now is None else now
        with self.lock:
            entry = self.frames.get(module)
            drive_active = self._module_drive_active(module)
        if drive_active:
            return {"ok": False, "reason": "module_busy"}
        if entry is None or now - entry[1] >= self.STALE_S:
            return {"ok": False, "reason": "module_offline"}
        frame = entry[0]
        if frame.get("moving") or frame.get("calib") or frame.get("cycle"):
            return {"ok": False, "reason": "module_busy"}

        # Прошивка принимает zero только на стоящем модуле. Оператор сначала
        # механически выставляет колесо прямо; сервер не запускает калибровочный ход.
        try:
            self.commands.put_nowait(("w %s zero\n" % module).encode("ascii"))
        except queue.Full:
            return {"ok": False, "reason": "command_queue_full"}
        return {"ok": True, "queued": True, "module": module}

    def drive_config(self, now=None):
        """Настройка привода тяги по телеметрии всех четырёх плат.

        Платы должны присылать одинаковую настройку: если одна управляется
        ШИМ, а другая 0–5 В, «общего режима» нет, и экран честно пишет
        «различаются» вместо того, чтобы показать настройку одной платы.
        """
        now = time.time() if now is None else now
        with self.lock:
            frames = dict(self.frames)
        fresh = {}
        for module in MODULE_TITLES:
            entry = frames.get(module)
            if entry is not None and now - entry[1] < self.STALE_S:
                fresh[module] = entry[0]
        values = list(fresh.values())

        def common(key):
            if len(values) != len(MODULE_TITLES):
                return None
            first = values[0].get(key)
            if any(frame.get(key) != first for frame in values):
                return None
            return first

        supported = bool(values) and all(frame.get("driveMode") is not None for frame in values)
        output_mode = common("driveMode")
        tach = common("tach")
        max_percent = common("driveMaxPercent")
        # Даже если платы не сошлись в пределе, осторожная из них задаёт общий:
        # сервер не должен обещать экрану больше, чем подтвердила самая узкая.
        limits = [frame.get("driveMaxPercent") for frame in values
                  if frame.get("driveMaxPercent") is not None]
        limit_percent = min(limits) if len(limits) == len(MODULE_TITLES) else None
        min_mv = common("driveMinMv")
        max_mv = common("driveMaxMv")
        output_mv = common("driveOutputMv")
        partial = {
            "supported": supported,
            "confirmed": supported and all(frame.get(field) is not None
                                           for frame in values
                                           for field in ("driveMaxPercent", "tach", "driveMode")),
            "outputMode": output_mode,
            "tach": tach,
            "maxPercent": max_percent,
            "limitPercent": limit_percent,
            "minMv": min_mv,
            "maxMv": max_mv,
            "outputMv": output_mv,
            "brake": common("brake"),
            "dirReverse": common("dirReverse"),
            "perModule": {module: {
                "outputMode": frame.get("driveMode"),
                "tach": frame.get("tach"),
                "maxPercent": frame.get("driveMaxPercent"),
                "minMv": frame.get("driveMinMv"),
                "maxMv": frame.get("driveMaxMv"),
                "outputMv": frame.get("driveOutputMv"),
                "dirReverse": frame.get("dirReverse"),
            } for module, frame in fresh.items()},
        }
        partial["mixed"] = supported and (
            output_mode is None or tach is None or max_percent is None)
        return partial

    def drive_limit_percent(self, now=None):
        """Действующий предел тяги: минимум из присланных платами, не выше сервера.

        Платы останавливают мотор по своему пределу; сервер не должен обещать
        экрану больше, чем подтвердила самая осторожная из четырёх.
        """
        config = self.drive_config(now)
        reported = config.get("limitPercent")
        limit = reported if isinstance(reported, int) and reported > 0 else self.MAX_DRIVE_PERCENT
        return max(1, min(int(limit), DRIVE_ABS_MAX_PERCENT))

    def drive_status(self, now=None):
        """Готовность тяги: общей (все колёса) и каждого колеса по отдельности.

        Общая тяга требует выровненных колёс; одиночная — только своего колеса:
        разворот на месте и проверка одного мотора как раз едут вразнобой.
        """
        now = time.time() if now is None else now
        with self.lock:
            frames = dict(self.frames)
            commanded = self.drive_percent
            commanded_modules = dict(getattr(self, "drive_modules", None) or {})
        motors = {}
        reason = ""
        fresh_frames = {}
        for module in MODULE_TITLES:
            entry = frames.get(module)
            if entry is not None and now - entry[1] < self.STALE_S:
                fresh_frames[module] = entry[0]
                motors[module] = entry[0].get("drivePercent")
            else:
                motors[module] = None

        if not self.serial_ready.is_set():
            reason = "serial_unavailable"
        elif len(fresh_frames) != len(MODULE_TITLES):
            reason = "module_offline"
        elif any(frame.get("drivePercent") is None for frame in fresh_frames.values()):
            reason = "drive_firmware_required"
        elif any(not frame.get("cal") for frame in fresh_frames.values()):
            reason = "not_homed"
        elif any(frame.get("moving") or frame.get("calib") or frame.get("cycle")
                 for frame in fresh_frames.values()):
            reason = "module_busy"
        else:
            angles = [float(frame["deg"]) for frame in fresh_frames.values()]
            if max(angles) - min(angles) > DRIVE_ANGLE_ALIGNMENT_DEG:
                reason = "steering_misaligned"

        # По колесу: своё «можно/нельзя» и понятная причина. Общая ошибка
        # (рассинхрон углов, чужое колесо не на связи) одиночную тягу этого
        # колеса не запрещает — иначе кнопки «вперёд/назад» в карточке стояли
        # бы серыми, пока соседнее колесо не выставлено.
        per_module = {}
        for module in MODULE_TITLES:
            frame = fresh_frames.get(module)
            why = ""
            if not self.serial_ready.is_set():
                why = "serial_unavailable"
            elif frame is None:
                why = "module_offline"
            elif frame.get("drivePercent") is None:
                why = "drive_firmware_required"
            elif not frame.get("cal"):
                why = "not_homed"
            elif frame.get("calib") or frame.get("cycle") or frame.get("moving"):
                why = "module_busy"
            # Сторону вращения меняют только у стоящего колеса: реверс DIR под
            # тягой — рывок. Мёртвый ноль и чужие колёса здесь не мешают —
            # настройка живёт в EEPROM именно этой платы.
            why_dir = ""
            if not self.serial_ready.is_set():
                why_dir = "serial_unavailable"
            elif frame is None:
                why_dir = "module_offline"
            elif frame.get("driveMode") is None:
                why_dir = "firmware_update_required"
            elif (commanded != 0 or commanded_modules.get(module, 0) or
                  frame.get("calib") or frame.get("cycle") or frame.get("moving") or
                  frame.get("drivePercent")):
                why_dir = "module_busy"
            per_module[module] = {
                "canDrive": not why,
                "reason": why or None,
                "percent": motors.get(module),
                "commandedPercent": commanded_modules.get(module, 0),
                "outputMv": frame.get("driveOutputMv") if frame else None,
                "rpm": frame.get("rpm") if frame else None,
                "tach": frame.get("tach") if frame else None,
                # Сторона вращения этого колеса (m dir 0|1 в прошивке):
                # «вперёд» на зеркальном колесе — обратный DIR.
                "dirReverse": frame.get("dirReverse") if frame else None,
                "canSetDirection": not why_dir,
                "directionReason": why_dir or None,
            }

        config = self.drive_config(now)
        return {
            "available": self.serial_ready.is_set(),
            "canDrive": not reason,
            "reason": reason or None,
            "maxPercent": self.drive_limit_percent(now),
            "commandedPercent": commanded,
            "commandedModules": commanded_modules,
            "individual": bool(getattr(self, "supports_individual_drive", False)),
            "perModule": per_module,
            "active": any(value not in (None, 0) for value in motors.values()),
            "reportedPercent": motors,
            "reportedOutputMv": {module: frame.get("driveOutputMv")
                                 for module, frame in fresh_frames.items()},
            "config": config,
            "alignmentToleranceDeg": DRIVE_ANGLE_ALIGNMENT_DEG,
        }

    def drive(self, percent, now=None):
        try:
            percent = float(percent)
        except (TypeError, ValueError):
            return {"ok": False, "reason": "invalid_drive"}
        limit = self.drive_limit_percent(now)
        if not math.isfinite(percent) or abs(percent) > limit:
            return {"ok": False, "reason": "drive_limit", "limit": limit}
        if not self.serial_ready.is_set():
            return {"ok": False, "reason": "serial_unavailable"}

        now = time.time() if now is None else now
        target = int(round(percent))
        if target == 0:
            command = b"b\n"
        else:
            status = self.drive_status(now)
            if not status["canDrive"]:
                return {"ok": False, "reason": status["reason"] or "drive_not_ready"}
            # «t» — тяга; «d» в прошивке — поворот на угол, как в проверенном скетче.
            command = ("t %d\n" % target).encode("ascii")
        if target == 0:
            # STOP always jumps ahead of queued steering/heartbeat commands,
            # not only when the bounded queue is already full.
            self._discard_pending_commands()
        try:
            self.commands.put_nowait(command)
        except queue.Full:
            if target != 0:
                return {"ok": False, "reason": "command_queue_full"}
            self._discard_pending_commands()
            try:
                self.commands.put_nowait(command)
            except queue.Full:
                return {"ok": False, "reason": "command_queue_full"}
        with self.lock:
            self.drive_percent = target
            # Общая команда адресована всем колёсам: прежние одиночные
            # значения больше ничего не значат.
            self.drive_modules = {}
            self.drive_last_command = now
        return {"ok": True, "queued": True, "percent": target,
                "maxPercent": limit}

    def drive_module(self, module, percent, now=None):
        """Тяга одного колеса: «t FL 8» уходит только на порт этого колеса.

        В отличие от общей тяги, здесь не требуется выравнивание всех колёс:
        смысл команды — крутить одно колесо (проверка мотора, разворот на
        месте, когда FL/RL едут вперёд, а FR/RR назад). Своё колесо всё равно
        должно быть на связи, с заданным нулём и стоять.
        """
        module = str(module or "").upper()
        if module not in MODULE_TITLES:
            return {"ok": False, "reason": "invalid_module"}
        try:
            percent = float(percent)
        except (TypeError, ValueError):
            return {"ok": False, "reason": "invalid_drive"}
        if not math.isfinite(percent):
            return {"ok": False, "reason": "invalid_drive"}
        limit = self.drive_limit_percent(now)
        if abs(percent) > limit:
            return {"ok": False, "reason": "drive_limit", "limit": limit}
        if not self.serial_ready.is_set():
            return {"ok": False, "reason": "serial_unavailable"}

        now = time.time() if now is None else now
        target = int(round(percent))
        if target != 0:
            with self.lock:
                entry = self.frames.get(module)
            if entry is None or now - entry[1] >= self.DRIVE_STALE_S:
                return {"ok": False, "reason": "module_offline"}
            frame = entry[0]
            if frame.get("drivePercent") is None:
                return {"ok": False, "reason": "drive_firmware_required"}
            if not frame.get("cal"):
                return {"ok": False, "reason": "not_homed"}
            if frame.get("calib") or frame.get("cycle") or frame.get("moving"):
                return {"ok": False, "reason": "module_busy"}

        # Имя колеса в команде: прошивка берёт только своё (чужое молча
        # пропускает), а сервер всё равно шлёт её лишь на этот порт.
        command = ("t %s %d\n" % (module, target)).encode("ascii")
        try:
            self.commands.put_nowait(command)
        except queue.Full:
            return {"ok": False, "reason": "command_queue_full"}
        with self.lock:
            modules = dict(getattr(self, "drive_modules", None) or {})
            modules[module] = target
            self.drive_modules = modules
            self.drive_last_command = now
        return {"ok": True, "queued": True, "module": module,
                "percent": target, "maxPercent": limit}

    def stop_drive(self, now=None):
        return self.drive(0, now)

    #: Что можно менять командой m. Каждое действие — один токен настройки.
    DRIVE_CONFIG_ACTIONS = ("output", "tach", "brake", "limit", "voltage", "defaults",
                            "direction")

    def set_drive_config(self, action, value=None, module=None, now=None):
        """Настройка привода тяги на всех четырёх платах (команда m).

        Смена режима выхода или предела под работающим мотором — это рывок,
        поэтому требуется покой: тяга выключена, колёса стоят, калибровка не
        идёт. Перед командой проверяем, что все четыре платы на связи, свежи и
        вообще поддерживают настройку (в @TLM есть dm/tach/dmax).
        """
        if action not in self.DRIVE_CONFIG_ACTIONS:
            return {"ok": False, "reason": "invalid_action"}
        if not self.serial_ready.is_set():
            return {"ok": False, "reason": "serial_unavailable"}
        if action == "direction":
            # Направление — свойство конкретного колеса, поэтому и команда
            # адресная: «m FL dir 1». Остальные колёса не трогаются вовсе.
            return self._set_module_direction(value, module, now)
        now = time.time() if now is None else now
        with self.lock:
            frames = dict(self.frames)
            drive_active = self._module_drive_active()
        if any(mid not in frames or now - frames[mid][1] >= self.STALE_S for mid in MODULE_TITLES):
            return {"ok": False, "reason": "module_offline"}
        values = [frames[mid][0] for mid in MODULE_TITLES]
        if any(f.get("driveMode") is None for f in values):
            return {"ok": False, "reason": "firmware_update_required"}
        if drive_active or any(f.get("moving") or f.get("calib") or f.get("cycle") or
                               f.get("drivePercent") for f in values):
            return {"ok": False, "reason": "module_busy"}
        commands = self._drive_config_commands(action, value, values)
        if isinstance(commands, dict):
            return commands
        for command in commands:
            try:
                self.commands.put_nowait(command)
            except queue.Full:
                return {"ok": False, "reason": "command_queue_full"}
        return {"ok": True, "queued": True, "action": action,
                "commands": [c.decode("ascii").strip() for c in commands]}

    def _set_module_direction(self, value, module, now=None):
        """Сторона вращения одного колеса: ``m FL dir 0|1`` только на его порт.

        Физически колёса стоят зеркально, поэтому «вперёд» у левого и правого
        борта — противоположные DIR. Здесь оператор говорит плате, что считать
        «вперёд»; логика тяги не меняется, меняется только уровень DIR, и
        выбор живёт в EEPROM платы.
        """
        module = str(module or "").upper()
        if module not in MODULE_TITLES:
            return {"ok": False, "reason": "invalid_module"}
        if isinstance(value, bool):
            reverse = value
        elif isinstance(value, str):
            reverse = {"reverse": True, "normal": False}.get(value.lower())
            if reverse is None:
                return {"ok": False, "reason": "invalid_value"}
        else:
            return {"ok": False, "reason": "invalid_value"}
        now = time.time() if now is None else now
        with self.lock:
            entry = self.frames.get(module)
            drive_active = self._module_drive_active(module)
        if entry is None or now - entry[1] >= self.STALE_S:
            return {"ok": False, "reason": "module_offline"}
        frame = entry[0]
        if frame.get("driveMode") is None:
            return {"ok": False, "reason": "firmware_update_required"}
        if (drive_active or frame.get("moving") or frame.get("calib") or
                frame.get("cycle") or frame.get("drivePercent")):
            return {"ok": False, "reason": "module_busy"}
        command = ("m %s dir %d\n" % (module, 1 if reverse else 0)).encode("ascii")
        try:
            self.commands.put_nowait(command)
        except queue.Full:
            return {"ok": False, "reason": "command_queue_full"}
        return {"ok": True, "queued": True, "action": "direction", "module": module,
                "reverse": reverse,
                "commands": [command.decode("ascii").strip()]}

    def _drive_config_commands(self, action, value, values):
        """Действие экрана → команды прошивке или отказ с причиной."""
        if action == "output":
            if value not in DRIVE_OUTPUT_MODES:
                return {"ok": False, "reason": "invalid_value"}
            return [b"m volt\n" if value == "volt" else b"m pwm\n"]
        if action in ("tach", "brake"):
            if value not in ("on", "off"):
                return {"ok": False, "reason": "invalid_value"}
            return [("m %s %d\n" % (action, 1 if value == "on" else 0)).encode("ascii")]
        if action == "limit":
            try:
                limit = int(value)
            except (TypeError, ValueError):
                return {"ok": False, "reason": "invalid_value"}
            if not 1 <= limit <= DRIVE_ABS_MAX_PERCENT:
                return {"ok": False, "reason": "invalid_value"}
            return [("m max %d\n" % limit).encode("ascii")]
        if action == "voltage":
            if not isinstance(value, dict) or set(value) != {"minMv", "maxMv"}:
                return {"ok": False, "reason": "invalid_value"}
            try:
                min_mv, max_mv = int(value["minMv"]), int(value["maxMv"])
            except (TypeError, ValueError):
                return {"ok": False, "reason": "invalid_value"}
            if not (0 <= min_mv <= max_mv <= DRIVE_MV_FULL) or max_mv < 1:
                return {"ok": False, "reason": "invalid_value"}
            return [("m vmin %d\n" % min_mv).encode("ascii"),
                    ("m vmax %d\n" % max_mv).encode("ascii")]
        if action == "defaults":
            return [b"m reset\n"]
        return {"ok": False, "reason": "invalid_action"}

    def _discard_pending_commands(self):
        while True:
            try:
                self.commands.get_nowait()
            except queue.Empty:
                break
            else:
                self.commands.task_done()
        with self.lock:
            links = list((getattr(self, "links", None) or {}).values())
        for link in links:
            while True:
                try:
                    link.commands.get_nowait()
                except queue.Empty:
                    break

    def _brake_all(self):
        """Тормоз на все открытые порты: один пропавший порт гасит всю тягу."""
        with self.lock:
            links = list((getattr(self, "links", None) or {}).values())
            self.drive_percent = 0
            self.drive_modules = {}
            self.drive_last_command = 0.0
        for link in links:
            try:
                link.commands.put_nowait(b"b\n")
            except queue.Full:
                pass

    def _fanout_commands(self):
        """Раздать команды из общей очереди по открытым портам."""
        while True:
            try:
                command = self.commands.get_nowait()
            except queue.Empty:
                return
            else:
                self.commands.task_done()
            target = self._command_module(command)
            with self.lock:
                links = list((getattr(self, "links", None) or {}).values())
            if target and not any(link.module == target for link in links):
                self._note_message(
                    "%s: команда «%s» не доставлена — порт колеса не на связи"
                    % (target, bytes(command).decode("ascii", "ignore").strip()),
                    time.time())
                continue
            for link in links:
                if not link.module:
                    # Порт ещё (или уже) не опознан как колесо: писать в него
                    # нельзя — на том же USB может быть лидар, а команды уходят
                    # прямо в его UART (см. _link_worker).
                    continue
                if target and link.module != target:
                    continue
                try:
                    routed = command
                    if target and self._assigned_module(link.path):
                        # Порт уже определяет колесо: имя в команде лишнее, а
                        # прошивка ждёт «w 12.5» / «t 8» без него.
                        parts = bytes(command).decode("ascii").split()
                        routed = (" ".join([parts[0]] + parts[2:]) + "\n").encode("ascii")
                    link.commands.put_nowait(routed)
                except queue.Full:
                    pass

    def _link_write(self, link, dev):
        while True:
            try:
                command = link.commands.get_nowait()
            except queue.Empty:
                return
            try:
                dev.write(command)
                dev.flush()
            finally:
                link.commands.task_done()

    def _assigned_module(self, path):
        real = os.path.realpath(path)
        return next((m for m, p in getattr(self, "module_ports", {}).items()
                     if os.path.realpath(p) == real), None)

    def _link_line(self, link, line):
        module = self._assigned_module(link.path)
        if module is None:
            return False
        # Role comes from the selected port, never from mod= or @ID.
        # Validate actual telemetry before enabling commands to that port.
        if not self.feed(line, module=module):
            return False
        with self.lock:
            fresh = link.module is None
            self.modules[module] = link.path
            link.module = module
            if self.port is None:
                self.port = link.path
        self.serial_ready.set()
        if fresh:
            self._note_status("ok", "мастер на связи: %s · колесо %s (ручной порт)" % (link.path, module))
        return True

    def _link_bytes(self, link, raw):
        """UART timeouts may split a line; only a newline completes a frame.

        Bound memory on binary/noisy ports and discard the *whole* oversized
        line, rather than interpreting a valid-looking suffix as telemetry.
        """
        telemetry = False
        for byte in raw:
            if byte == 10:
                if not link.rx_discard and link.rx:
                    telemetry = self._link_line(
                        link, link.rx.decode("utf-8", "replace")) or telemetry
                link.rx.clear()
                link.rx_discard = False
            elif not link.rx_discard:
                link.rx.append(byte)
                if len(link.rx) > self.MAX_LINE_BYTES:
                    link.rx.clear()
                    link.rx_discard = True
        return telemetry

    # ------------------------------------------------------------ потоки портов #
    def _candidate_ports(self):
        """Проверить наличие только вручную назначенных портов, без USB-поиска."""
        ports = list(self.module_ports.values())
        out, seen = [], set()
        for path in ports:
            path = str(path or "").strip()
            if not path or not os.path.exists(path):
                continue
            try:
                real = os.path.realpath(path)
            except OSError:
                real = path
            if real in seen:
                continue
            seen.add(real)
            out.append(path)
        with self.lock:
            self.candidates = list(out)
        return out

    def _lidar_needs(self, port):
        """Порт сейчас нужен лидару (он ищет свой или уже читает этот)."""
        guard = getattr(self, "yield_to_lidar", None)
        if guard is None:
            return False
        try:
            return bool(guard(port))
        except Exception:                 # noqa: BLE001 — лидар не ломает колёса
            return False

    def _claim(self, port):
        """Занять межпроцессный замок порта.

        Возвращает владение, ``None`` (порт занят лидаром или вторым процессом
        робота) или :data:`_UNCLAIMED`, если замков нет вовсе.
        """
        claim = getattr(self, "claim", None)
        if claim is None:
            return _UNCLAIMED
        try:
            return claim(port)
        except Exception:                 # noqa: BLE001 — без замка живём как раньше
            return _UNCLAIMED

    def _unclaim(self, token):
        if token is _UNCLAIMED or token is None:
            return
        release = getattr(self, "release", None)
        if release is None:
            return
        try:
            release(token)
        except Exception:                 # noqa: BLE001 — замок уже не наш
            pass

    def _tag(self, port, on=True):
        callback = getattr(self, "tag_module" if on else "untag_module", None)
        if callback is None:
            return
        try:
            callback(port)
        except Exception:                 # noqa: BLE001 — отметка не критична
            pass

    def _open_new_links(self):
        """Открыть порты, которых ещё нет в работе: каждый — своё колесо."""
        candidates = self._candidate_ports()
        now = time.monotonic()
        if not candidates and not self.links:
            self.serial_ready.clear()
            self._note_status(
                "warn",
                "колёса: назначенные USB-порты недоступны — проверьте ручной выбор портов")
            return
        waiting_for_lidar = False
        for candidate in candidates:
            with self.lock:
                # Candidate aliases can change between rescans. Opening the
                # same tty again can toggle DTR and reset an Arduino even when
                # the subsequent pyserial exclusive lock fails.
                device = os.path.realpath(candidate)
                busy = {link.device for link in self.links.values()}
                busy.update(os.path.realpath(p) for p in self._spawning)
                if device in busy:
                    continue
                if self._cooldown.get(candidate, 0.0) > now:
                    continue
            if self._lidar_needs(candidate):
                # Лидар ищет свой порт (или уже читает этот): колёса ждут, иначе
                # его перебор упирается в порты, которые держим мы.
                waiting_for_lidar = True
                continue
            with self.lock:
                if candidate in self.links or candidate in self._spawning:
                    continue
                self._spawning.add(candidate)
            link = _WheelLink(candidate)
            threading.Thread(target=self._link_worker, args=(link,), daemon=True).start()
        if waiting_for_lidar and not self._spawning and not self.links:
            self._note_status("warn", "колёса: лидар ищет свой порт — жду")

    def _link_worker(self, link):
        """Читать только назначенный порт; готовность — по свежей @TLM."""
        serial = self._serial
        dev = None
        token = self._claim(link.path)
        identified = False
        try:
            if token is None:
                # Замок держит лидар или второй процесс робота: порт не наш,
                # ждём молча (сообщение «лидар ищет порт» уже сказано выше).
                with self.lock:
                    self._cooldown[link.path] = time.monotonic() + self.RETRY_S
                return
            # exclusive: второй процесс не может открыть тот же UART и читать
            # его вперемешку (строки телеметрии рвутся, команды теряются).
            dev = serial.Serial(link.path, self.baud, timeout=0.2, exclusive=True)
            with self.lock:
                self.links[link.path] = link
            # Ждём свежую телеметрию после возможного сброса при открытии.
            deadline = time.monotonic() + self.PROBE_S
            while not self._stop and link.module is None and time.monotonic() < deadline:
                raw = dev.readline()
                if raw:
                    self._link_bytes(link, raw)
            if link.module is None:
                self._note_status("warn", "порт %s: нет @TLM — проверю снова; проверьте питание USB и прошивку" % link.path)
                return
            identified = True
            # Отметка «здесь колесо»: лидар такой порт не перебирает — его
            # открытие сбрасывает плату и рвёт телеметрию.
            self._tag(link.path)
            # Keep the shared real-device lock until close. pyserial's tty
            # lock alone cannot prevent another subsystem opening/resetting
            # the board before it attempts its own exclusive lock.
            last_telemetry = time.monotonic()
            next_tag = last_telemetry + self.TAG_REFRESH_S
            while not self._stop:
                raw = dev.readline()
                if raw and self._link_bytes(link, raw):
                    last_telemetry = time.monotonic()
                if time.monotonic() - last_telemetry >= self.TELEMETRY_TIMEOUT_S:
                    raise TimeoutError(
                        "нет телеметрии @TLM %.0f с; переподключаю USB — проверьте питание разветвителя, кабель и прошивку"
                        % self.TELEMETRY_TIMEOUT_S)
                self._link_write(link, dev)
                if time.monotonic() >= next_tag:
                    self._tag(link.path)
                    next_tag = time.monotonic() + self.TAG_REFRESH_S
        except Exception as exc:          # noqa: BLE001 — порт может пропасть
            self.last_error = str(exc)
            if not self._stop:
                self._note_status(
                    "warn",
                    "колесо %s (%s): %s%s — жду переподключения"
                    % (link.module or "не опознано", link.path, exc, self._port_error_hint(exc)))
        finally:
            if dev is not None:
                try:
                    dev.close()
                except Exception:         # noqa: BLE001 — порт уже мёртв
                    pass
            lost_module = None
            with self.lock:
                self.links.pop(link.path, None)
                self._spawning.discard(link.path)
                self._cooldown[link.path] = time.monotonic() + self.RETRY_S
                if link.module and self.modules.get(link.module) == link.path:
                    self.modules.pop(link.module, None)
                    self.frames.pop(link.module, None)
                    lost_module = link.module
                if not self.modules:
                    self.serial_ready.clear()
                    self.port = None
            self._unclaim(token)
            if identified and not self._stop:
                # Порт потерян: снимаем отметку, иначе лидар считал бы его чужим,
                # даже если на место колеса воткнули лидар.
                self._tag(link.path, on=False)
            if lost_module and not self._stop:
                self._note_status("warn", "колесо %s: порт %s пропал" % (lost_module, link.path))
                # Fail closed: потеря колеса в движении гасит тягу везде —
                # и общую, и одиночную (колесо могло ехать одно).
                if self._module_drive_active():
                    self._brake_all()

    def _reader(self):
        try:
            import serial                # pyserial, только для реального железа
        except ImportError:
            print("! master: pyserial не установлен (pip install pyserial) — порты колёс выключены")
            self._note_status("error", "мастер: pyserial не установлен — порты колёс выключены")
            return
        self._serial = serial
        next_scan = 0.0
        while not self._stop:
            if time.monotonic() >= next_scan:
                next_scan = time.monotonic() + self.RESCAN_S
                self._open_new_links()
            self._fanout_commands()
            time.sleep(0.05 if (self.links or self._spawning) else self.RETRY_S)

    # -------------------------------------------------------------- телеметрия #
    def read(self, now=None):
        now = time.time() if now is None else now
        with self.lock:
            frames = dict(self.frames)
        motors = []
        link = False
        for mid in MODULE_NAMES.values():
            entry = frames.get(mid)
            fresh = entry is not None and (now - entry[1]) < self.STALE_S
            if fresh:
                link = True
                frame = entry[0]
                error = frame.get("calibError")
                motors.append({"id": mid, "title": MODULE_TITLES[mid],
                               "angle": round(frame["deg"], 2),
                               "targetAngle": round(frame["tgt"], 2),
                               "rpm": frame.get("rpm"), "temp": None,
                               "drivePercent": frame.get("drivePercent"),
                               "homed": frame["cal"], "manualZero": frame.get("manualZero", frame["cal"]),
                               "homeSource": frame.get("home") or ("manual" if frame["cal"] else "none"),
                               "opto": frame.get("opto"), "calibError": error or 0,
                               "calibErrorText": CALIB_ERRORS.get(error) if error else None,
                               "firmware": frame.get("firmware"),
                               "pot": frame.get("pot", False),
                               "potMode": frame.get("potMode"),
                               # Гибкая настройка тяги своей платы: экран не
                               # выдаёт «0 об/мин» за измерение, если тахометра
                               # нет, и показывает напряжение выхода.
                               "tach": frame.get("tach"),
                               "driveMode": frame.get("driveMode"),
                               "driveMaxPercent": frame.get("driveMaxPercent"),
                               "driveOutputMv": frame.get("driveOutputMv"),
                               "online": True, "fault": None,
                               "moving": frame["moving"], "calibrating": frame["calib"],
                               "cycling": frame.get("cycle", False)})
            else:
                motors.append({"id": mid, "title": MODULE_TITLES[mid], "angle": None,
                               "targetAngle": None, "rpm": None, "temp": None,
                               "drivePercent": None, "homed": None, "online": False, "fault": None,
                               "pot": None, "tach": None, "driveMode": None,
                               "driveMaxPercent": None, "driveOutputMv": None,
                               "moving": False, "calibrating": False})
        out = NoSource().read()              # АКБ и маршрут пока не передаются
        out["motors"] = motors
        out["drive"] = self.drive_status(now)
        out["mode"] = "МАСТЕР НА СВЯЗИ" if link else None
        out["linkOk"] = link
        out["masterMessages"] = self.recent_messages(8) if hasattr(self, "messages") else []
        # Какие порты открыты и за каким колесом какой закреплён: экран
        # объясняет «нет связи» конкретным портом, а не советом «проверьте кабель».
        with self.lock:
            modules = dict(getattr(self, "modules", None) or {})
            open_ports = list(getattr(self, "links", None) or {})
            port = getattr(self, "port", None)
            ready = getattr(self, "serial_ready", None)
            active = port if ready is not None and ready.is_set() else None
            candidates = list(getattr(self, "candidates", None) or
                              getattr(self, "ports", None) or ([port] if port else []))
            out["masterPort"] = {
                "configured": getattr(self, "configured_port", port),
                "active": active,
                "candidates": candidates,
                "error": getattr(self, "last_error", "") or None,
                "modules": modules,
                "open": open_ports,
            }
        return out

class MasterPeerSource:
    """Пульт (8080) без собственного UART: телеметрия берётся с бортового экрана.

    Порт мастера один, и владеет им бортовой экран (8090). Если открыть его ещё
    раз из пульта, два процесса читают одни и те же байты, строки рвутся, а
    команды отбрасываются. Этот источник только опрашивает телеметрию.
    Веб-вкладка /controls посылает команды через отдельный авторизованный
    HTTP-прокси /api/control/* на борт, не вызывая методы этого источника.
    """

    name = "master-peer"
    supports_individual_steer = False
    supports_drive = False
    POLL_S = 0.2
    STALE_S = 1.5

    def __init__(self, peer_url, fetch=None):
        self.peer_url = peer_url
        self.lock = threading.Lock()
        self.snapshot_data = None
        self.snapshot_time = 0.0
        self.error = "ещё не опрошен"
        self._fetch = fetch or self._http_get
        threading.Thread(target=self._poll, daemon=True).start()

    def _http_get(self):
        import urllib.request
        with urllib.request.urlopen(self.peer_url, timeout=1.0) as response:
            return json.loads(response.read().decode("utf-8", "replace"))

    def _poll(self):
        while True:
            try:
                body = self._fetch()
                data = body.get("data") if isinstance(body, dict) else None
                if not isinstance(data, dict):
                    raise ValueError("экран не вернул телеметрию")
                with self.lock:
                    self.snapshot_data = data
                    self.snapshot_time = time.time()
                    self.error = ""
            except Exception as exc:  # noqa: BLE001 — экран может быть выключен
                with self.lock:
                    self.error = str(exc) or type(exc).__name__
            time.sleep(self.POLL_S)

    def read(self, now=None):
        now = time.time() if now is None else now
        with self.lock:
            data = self.snapshot_data
            fresh = data is not None and now - self.snapshot_time < self.STALE_S
            error = self.error
        out = NoSource().read()
        if not fresh:
            # Не выдаём потерю связи за гарантированно снятый E-STOP: UI
            # должен консервативно считать состояние борта неизвестным/запертым.
            out["estop"] = {"unknown": True}
            out["peer"] = {"url": self.peer_url, "error": error or "нет связи с бортовым экраном"}
            return out
        out["motors"] = data.get("motors") or out["motors"]
        out["drive"] = data.get("drive") or out["drive"]
        out["masterMessages"] = data.get("masterMessages") or []
        out["masterPort"] = data.get("masterPort")
        out["linkOk"] = bool(data.get("linkOk"))
        out["mode"] = data.get("mode")
        out["estop"] = data.get("estop")
        out["peer"] = {"url": self.peer_url, "error": ""}
        return out

    def steer_module(self, module, angle, now=None):
        return {"ok": False, "reason": "control_not_supported"}

    def zero_module(self, module, now=None):
        return {"ok": False, "reason": "control_not_supported"}

    def calibrate_module(self, module, now=None):
        return {"ok": False, "reason": "control_not_supported"}

    def drive(self, percent, now=None):
        return {"ok": False, "reason": "control_not_supported"}

    def stop_drive(self, now=None):
        return {"ok": False, "reason": "control_not_supported"}


class SerialSource:
    """Реальные модули: 4 UART, кадры телеметрии 16 Б (20 Гц).

    Порт открывается только на время проверки: если за ``PROBE_WINDOW_S`` от
    него не пришло ни одного корректного кадра телеметрии, порт освобождается и
    следующая попытка откладывается (2 → 4 → … → 30 с). Без этого источник
    данных шасси в режиме ``auto`` находил все ``/dev/ttyUSB*`` и держал их
    открытыми навсегда — включая порт USB-лидара, из которого он вычитывал
    байты: лидар в веб-интерфейсе «не отвечал», хотя ``scripts/lidar_check.py``
    (его запускают при остановленном пульте) работал.

    Замок порта (``claim``/``release``): пока порт не доказал, что он — модуль
    шасси, его читает только один процесс робота. ``claim(port)`` возвращает
    признак владения или ``None``, если порт занят лидаром (тот берёт порт
    монопольно, ``exclusive`` в pyserial) либо вторым процессом робота;
    ``release`` снимает замок. Без этого пульт (8080) и бортовой экран (8090)
    читали один и тот же USB-порт вдвоём: байты доставались тому, кто успел, а
    второй получал от pyserial «device reports readiness to read but returned no
    data (… multiple access on port?)». Лидару же порт не доставался вовсе: его
    перебор упирался в «занят», хотя порт держало это же приложение, — и на
    экране появлялся совет «проверь питание и кабель лидара».

    Как только на порту подтвердилась телеметрия, замок снимается, а порт
    читается дальше как раньше — в том числе вторым процессом робота: оба экрана
    обязаны показывать одни и те же модули. Для лидара такой порт — просто «нет
    данных»: он переберёт его вместе с остальными USB и отпустит, поделив эти
    секунды с экранами.

    Из замка видно только «занят / свободен», а на роботе с двумя адаптерами
    этого мало: пока лидар искал свой порт, модули успевали взять оба под свою
    проверку, и лидар писал «все 2 порт(ов) заняты другой программой» — то есть
    винил чужую программу там, где порты держало это же приложение. Поэтому есть
    два необязательных механизма (приходят из ``robot_control.lidar`` через
    ``robot_control/slam.py``; в отдельном запуске ``backend.py`` их нет и
    поведение прежнее):

      * ``yield_to_lidar(port)`` — «лидар ещё ищет порт (или читает этот), не
        занимай его сейчас»: поток ждёт и не открывает порт, пока лидар не нашёл
        свой. Порт, уже подтверждённый кадрами телеметрии, не уступается никогда:
        иначе телеметрия пропадала бы с обоих экранов на каждом круге поиска
        лидара (``PROBE_WINDOW_S``);
      * ``tag_module(port)`` / ``untag_module(port)`` — отметка в ``data/``, что
        на порту подтвердились модули шасси. Лидар такие порты не перебирает.
        Отметка снимается, как только окно проверки кончилось без кадров: если
        адаптер модулей выдернули и на его место воткнули лидар, порт не остаётся
        «занятым» навсегда.
    """

    name = "serial"

    #: Сколько секунд ждём кадр телеметрии, прежде чем освободить порт.
    PROBE_WINDOW_S = 2.0
    #: Пауза между попытками на «чужом» порту: растёт, чтобы не дёргать его.
    BACKOFF_START_S = 2.0
    BACKOFF_MAX_S = 30.0
    #: Как часто обновляем отметку «здесь модули шасси», пока идут кадры: чаще
    #: незачем (отметку читает лидар), реже — рискуем, что она протухнет.
    TAG_REFRESH_S = 5.0
    #: Пауза, пока лидар ищет свой порт: короткая, чтобы модули вернулись к
    #: своим портам сразу, как только поиск кончился (или окно приоритета истекло).
    YIELD_SLEEP_S = 0.5

    def __init__(self, ports, baud=115200, claim=None, release=None,
                 yield_to_lidar=None, tag_module=None, untag_module=None):
        self.ports = ports
        self.baud = baud
        #: Межпроцессный замок порта (см. docstring класса): ``claim(port)`` →
        #: владение или ``None``, ``release(владение)`` — снять. ``None`` —
        #: замков нет (отдельный запуск ``slam_gui/backend.py`` без пульта).
        self.claim = claim
        self.release = release or (lambda token: None)
        #: ``yield_to_lidar(port)`` → True, если порт сейчас нужен лидару.
        self.yield_to_lidar = yield_to_lidar
        #: Отметка порта, на котором подтвердилась телеметрия модулей.
        self.tag_module = tag_module
        self.untag_module = untag_module
        self.lock = threading.Lock()
        self.frames = {}                 # id модуля → последний кадр
        self.cargo = {"kg": 80, "closed": True}
        self.seen = 0.0
        self._stop = False
        for idx, port in enumerate(ports):
            threading.Thread(target=self._reader, args=(port, idx + 1), daemon=True).start()

    def _reader(self, port, default_id):
        try:
            import serial                    # pyserial, только для реального железа
        except ImportError:
            print("! serial: pyserial не установлен (pip install pyserial) — модуль %s выключен" % port)
            return
        buf = b""
        backoff = self.BACKOFF_START_S
        reported_busy = False
        reported_yield = False
        announced = False
        tagged_at = 0.0
        last_frame = 0.0                 # когда на этом порту видели кадр телеметрии
        while not self._stop:
            confirmed = False
            token = None
            if self.yield_to_lidar is not None and self.yield_to_lidar(port):
                # Лидар ещё ищет свой порт (см. docstring): подождём, не занимая
                # порты своей проверкой. Иначе его перебор упирается в «занят», и
                # он перестаёт видеть лидар, хотя порты держит это же приложение.
                if not reported_yield:
                    reported_yield = True
                    print("· serial %s: лидар ищет свой порт — жду" % port)
                time.sleep(self.YIELD_SLEEP_S)
                continue
            reported_yield = False
            if self.claim is not None:
                token = self.claim(port)
                if token is None:
                    # Порт занят: его держит лидар (в том числе пока ищет свой
                    # порт) или второй процесс робота. Ждём молча — не забиваем
                    # журнал повторами и не отбираем у лидара байты.
                    if not reported_busy:
                        reported_busy = True
                        self._note_busy(port, backoff)
                    time.sleep(backoff)
                    backoff = min(backoff * 2, self.BACKOFF_MAX_S)
                    continue
            try:
                with serial.Serial(port, self.baud, timeout=0.2) as dev:
                    if not announced:
                        # «открыт» печатаем один раз на порт, а не на каждом круге
                        # перебора: иначе журнал пульта целиком состоит из этих
                        # строк и по нему не видно, что вообще происходит.
                        announced = True
                        print("· serial: %s открыт (модуль %d)" % (port, default_id))
                    reported_busy = False
                    deadline = time.monotonic() + self.PROBE_WINDOW_S
                    while not self._stop:
                        chunk = dev.read(64)
                        if chunk:
                            buf += chunk
                            while True:
                                frame, buf = pick_telemetry(buf)
                                if not frame:
                                    break
                                confirmed = True
                                mid = frame.get("moduleId") or default_id
                                with self.lock:
                                    self.frames[mid] = frame
                                    self.seen = time.time()
                                last_frame = time.time()
                                if (self.tag_module is not None
                                        and time.monotonic() - tagged_at >= self.TAG_REFRESH_S):
                                    # Отметка «здесь модули шасси»: лидар такие порты
                                    # не перебирает (см. docstring класса).
                                    tagged_at = time.monotonic()
                                    self.tag_module(port)
                        if confirmed:
                            if time.time() - last_frame > self.PROBE_WINDOW_S:
                                # Кадры кончились: держать порт открытым «на всякий
                                # случай» нельзя — модули могли выдернуть, а на их
                                # место встать лидар; отметка «здесь модули» уйдёт
                                # вслед за окном проверки (см. ниже).
                                break
                            # Модуль говорит протоколом шасси — порт наш, держим
                            # его открытым. Замок при этом снимаем: второй процесс
                            # робота должен читать модули как раньше (см. docstring).
                            if token is not None:
                                self.release(token)
                                token = None
                            backoff = self.BACKOFF_START_S
                            deadline = time.monotonic() + self.PROBE_WINDOW_S
                            continue
                        if time.monotonic() >= deadline:
                            break
            except Exception as exc:          # noqa: BLE001 — порт может пропасть
                text = str(exc).lower()
                if "busy" in text or "resource" in text:
                    # Порт занят: его читает лидар (``exclusive``) или второй
                    # процесс робота. Молча ждём — не забиваем журнал повторами.
                    if not reported_busy:
                        reported_busy = True
                        self._note_busy(port, backoff)
                else:
                    print("! serial %s: %s — повтор через %d с" % (port, exc, int(backoff)))
            finally:
                if token is not None:
                    # Порт не подтвердился как модуль шасси — отпускаем его:
                    # следующая попытка позже, а лидар успеет забрать его себе.
                    self.release(token)
                    token = None
            if self.untag_module is not None and (
                    not confirmed or time.time() - last_frame > self.PROBE_WINDOW_S):
                # За целое окно проверки кадров не было: порт молчит или его
                # забрал лидар. Снимаем отметку «здесь модули шасси», иначе лидар
                # считал бы порт чужим и не открыл бы на нём лидара, если
                # адаптеры поменяли местами (или модули просто выдернули).
                self.untag_module(port)
            if self._stop:
                return
            time.sleep(backoff)
            if not confirmed:
                backoff = min(backoff * 2, self.BACKOFF_MAX_S)

    @staticmethod
    def _note_busy(port, backoff):
        print("· serial %s: порт занят (лидар или второй процесс) — жду %d с"
              % (port, int(backoff)))

    def read(self):
        with self.lock:
            frames = dict(self.frames)
            seen = self.seen
        motors = []
        volts_all, amps_all = [], []
        for num, name in MODULE_NAMES.items():
            f = frames.get(num)
            if not f:
                motors.append({"id": name, "title": MODULE_TITLES[name], "angle": 0.0,
                               "rpm": 0.0, "temp": 0.0, "homed": False, "online": False})
                continue
            rpm = f["encDelta"] * 6 * 60 / 360.0 * 10            # 6 имп/об·10 → об/мин (при 10 Гц опросе)
            motors.append({"id": name, "title": MODULE_TITLES[name], "angle": f["steerDeg"],
                           "rpm": rpm, "temp": 0.0, "homed": f["homed"], "online": True,
                           "fault": f["fault"], "amps": f["amps"], "pwm": f["pwm"]})
            volts_all.append(f["volts"])
            amps_all.append(f["amps"])
        volts = sum(volts_all) / len(volts_all) if volts_all else 0.0
        amps = sum(amps_all) if amps_all else 0.0
        link = (time.time() - seen) < 1.0 if seen else False
        battery = pack_state(volts or PACK["nominalV"], amps)
        return {
            "motors": motors,
            "battery": battery,
            "cargo": dict(self.cargo),
            "mode": "АВТОНОМНЫЙ РЕЖИМ" if link else "НЕТ СВЯЗИ С МОДУЛЯМИ",
            "route": "—",
            "speedMps": round(sum(abs(m.get("rpm", 0)) for m in motors) / 4 * 2 * math.pi * WHEEL_R_M / 60, 2),
            "powerKw": round(battery["watts"] / 1000.0, 2),
            "linkOk": link,
        }


LIDAR_MAX_AGE_S = 1.0     # скан старше — на экран не отдаём (карта пустая)


def scan_points(msg):
    """sensor_msgs/LaserScan -> [{a, r}]: a в радианах из [-pi, pi] (0 — вперёд, против часовой),
    r в метрах. Недостоверные лучи (inf, nan, вне range_min..range_max) пропускаются."""
    points = []
    lo = float(msg.range_min)
    hi = float(msg.range_max)
    inc = float(msg.angle_increment)
    for i, r in enumerate(msg.ranges):
        r = float(r)
        if not math.isfinite(r) or r < lo or r > hi:
            continue
        a = float(msg.angle_min) + i * inc
        a = math.atan2(math.sin(a), math.cos(a))       # в [-pi, pi]
        points.append({"a": round(a, 4), "r": round(r, 3)})
    return points


class RosSource:
    """ROS 2 (rclpy): /modules/state, /battery, /screen/motors и скан лидара.

    Пути к системному ROS 2 добавляются сами (``robot_control/ros_env.py``):
    экран работает из ``.venv`` и без этого не видел ``rclpy`` вовсе. Если ROS 2
    нет — источник не выдумывает данные: ``linkOk`` остаётся ложным, а причина
    видна в ``self.reason``. Демо-симуляции в приложении нет.
    """

    name = "ros"

    def __init__(self):
        self.ok = False
        self.data = {}
        self.scan = []
        self.scan_at = 0.0
        # ROS 2 стоит системно (/opt/ros/<дистрибутив>), а экран запускается из
        # .venv проекта: без подготовки путей rclpy в нём не виден, и источник
        # молча падал в симуляцию (см. robot_control/ros_env.py).
        try:
            from robot_control import ros_env
            ros_env.ensure_python_path()
            # Сигналы приложения (stop.sh, systemd, киоск) не перехватываем:
            # иначе rclpy.init() в фоновом потоке подменяет SIGTERM всего
            # процесса, и экран не может штатно завершиться.
            ros_env.init_rclpy()
            import rclpy                                            # noqa: F401
            from rclpy.node import Node
        except Exception as exc:  # noqa: BLE001 — нет ROS 2, сломан rclpy и т. п.
            print("! ros: rclpy не найден (%s) — источник не подключён." % exc)
            print("       поставь ROS 2 и пакеты лидара: ./scripts/ros2_setup.sh")
            self.fallback = None
            self.reason = "ROS 2 недоступен: rclpy не импортируется в этом интерпретаторе"
            return
        self.fallback = None
        self.reason = ""
        threading.Thread(target=self._spin_node, daemon=True).start()

    def _spin_node(self):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32, String
        from sensor_msgs.msg import LaserScan
        from rclpy.qos import qos_profile_sensor_data

        source = self
        # Топик скана: по умолчанию /scan, как у драйверов лидаров (RC_LIDAR_ROS_TOPIC).
        scan_topic = os.environ.get("RC_LIDAR_ROS_TOPIC") or "/scan"

        class Bridge(Node):
            def __init__(self):
                super().__init__("rus_slam_screen_bridge")
                self.create_subscription(String, "/screen/state", source._on_state, 10)
                self.create_subscription(Float32, "/battery/voltage", source._on_volts, 10)
                self.create_subscription(Float32, "/battery/current", source._on_amps, 10)
                self.create_subscription(String, "/screen/motors", source._on_motors, 10)
                self.create_subscription(LaserScan, scan_topic, source._on_scan,
                                         qos_profile_sensor_data)

        try:
            from robot_control import ros_env
            ros_env.init_rclpy(rclpy)
        except Exception as exc:                                     # noqa: BLE001
            self.reason = "rclpy не инициализировался: %s" % exc
            print("! ros: %s" % self.reason)
            return
        try:
            node = Bridge()
        except Exception as exc:                                     # noqa: BLE001
            self.reason = "узел ROS 2 не создался: %s" % exc
            print("! ros: %s" % self.reason)
            return
        self.ok = True
        print("· ros: мост экрана запущен (/screen/state, /battery/*, /screen/motors, %s)"
              % scan_topic)
        # Свой executor: rclpy.spin() берёт глобальный executor процесса, а в
        # нём одновременно крутится мост лидара из веб-пульта. Два потока в
        # одном executor'е — гонка на его очередях и потерянные сканы.
        try:
            from rclpy.executors import SingleThreadedExecutor
        except ImportError:                                          # pragma: no cover
            executor = None
        try:
            if executor is None:
                rclpy.spin(node)
            else:
                executor = SingleThreadedExecutor()
                executor.add_node(node)
                try:
                    executor.spin()
                finally:
                    try:
                        executor.remove_node(node)
                    finally:
                        executor.shutdown()
        except Exception:                                            # noqa: BLE001
            self.ok = False

    def _on_state(self, msg):
        try:
            self.data.update(json.loads(msg.data))
        except Exception:                                            # noqa: BLE001
            pass

    def _on_volts(self, msg):
        self.data.setdefault("battery", {})["volts"] = float(msg.data)

    def _on_amps(self, msg):
        self.data.setdefault("battery", {})["amps"] = float(msg.data)

    def _on_motors(self, msg):
        try:
            self.data["motors"] = json.loads(msg.data)
        except Exception:                                            # noqa: BLE001
            pass

    def _on_scan(self, msg):
        try:
            self.scan = scan_points(msg)
            self.scan_at = time.time()
            try:
                from robot_control.lidar import LidarManager
                LidarManager.get_instance().feed_scan_msg(
                    ranges=list(msg.ranges),
                    angle_min=float(msg.angle_min),
                    angle_max=float(msg.angle_max),
                    angle_increment=float(msg.angle_increment),
                    range_min=float(msg.range_min),
                    range_max=float(msg.range_max),
                    source_label="ros",
                )
            except Exception:
                pass
        except Exception:                                            # noqa: BLE001
            pass

    def read(self):
        if self.fallback:
            return self.fallback.read()
        d = self.data
        volts = float(d.get("battery", {}).get("volts") or 0.0)
        amps = float(d.get("battery", {}).get("amps") or 0.0)
        return {
            "motors": d.get("motors") or [],
            "battery": pack_state(volts or PACK["nominalV"], amps),
            "cargo": d.get("cargo") or {"kg": 0, "closed": True},
            "mode": d.get("mode") or ("АВТОНОМНЫЙ РЕЖИМ" if self.ok else "ОЖИДАНИЕ ROS"),
            "route": d.get("route") or "—",
            "speedMps": d.get("speedMps") or 0.0,
            "powerKw": round((volts * abs(amps)) / 1000.0, 2),
            "linkOk": self.ok,
            # лидар в реальном времени: свежий скан или None (карта пустая)
            "lidar": self.scan if self.scan and time.time() - self.scan_at < LIDAR_MAX_AGE_S else None,
        }


# ============================================================================
# 4. Замок грузового отсека (PIN, блокировки, аудит)
# ============================================================================
class Vault:
    """PIN-хранилище: SHA-256 + соль, счётчик попыток, аудит с хеш-цепочкой."""

    def __init__(self, path=LOCK_FILE, default_pin=DEFAULT_PIN,
                 max_attempts=MAX_ATTEMPTS, lock_ms=LOCK_MS, clock=time.time):
        self.path = path
        self.default_pin = default_pin
        self.max_attempts = max_attempts
        self.lock_ms = lock_ms
        self.clock = clock
        self.lock = threading.Lock()
        self.data = self._load()

    # --- файл ---
    def _load(self):
        try:
            with open(self.path, "r", encoding="utf-8") as fh:
                raw = json.load(fh)
            if isinstance(raw, dict) and raw.get("pinHash"):
                raw.setdefault("audit", [])
                raw.setdefault("fails", 0)
                raw.setdefault("lockUntil", 0)
                return raw
        except (OSError, ValueError):
            pass
        return self._fresh()

    def _fresh(self):
        salt = secrets.token_hex(16)
        return {
            "version": 1, "salt": salt,
            "pinHash": self._hash(salt, self.default_pin),
            "fails": 0, "lockUntil": 0, "audit": [], "changedAt": time.time(),
        }

    def _persist(self):
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            tmp = self.path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump(self.data, fh, ensure_ascii=False, indent=2)
            os.replace(tmp, self.path)
        except OSError as exc:
            print("! не удалось сохранить %s: %s" % (self.path, exc))

    # --- крипто ---
    @staticmethod
    def _hash(salt, pin):
        return hashlib.sha256((salt + ":" + str(pin)).encode("utf-8")).hexdigest()

    def _audit(self, action, ok, detail=""):
        prev = self.data["audit"][-1]["hash"] if self.data["audit"] else "genesis"
        ts = self.clock()
        payload = "%s|%.0f|%s|%d|%s" % (prev, ts * 1000, action, 1 if ok else 0, detail)
        entry = {"ts": ts, "action": action, "ok": bool(ok), "detail": detail,
                 "prev": prev, "hash": hashlib.sha256(payload.encode("utf-8")).hexdigest()}
        self.data["audit"].append(entry)
        del self.data["audit"][:-200]

    def verify_audit(self):
        prev = "genesis"
        for i, e in enumerate(self.data["audit"]):
            if e.get("prev") != prev:
                return {"ok": False, "brokenAt": i}
            payload = "%s|%.0f|%s|%d|%s" % (prev, e["ts"] * 1000, e["action"],
                                            1 if e["ok"] else 0, e.get("detail", ""))
            if hashlib.sha256(payload.encode("utf-8")).hexdigest() != e.get("hash"):
                return {"ok": False, "brokenAt": i}
            prev = e["hash"]
        return {"ok": True, "brokenAt": -1}

    # --- логика ---
    def lock_remaining_ms(self, now=None):
        now = self.clock() if now is None else now
        return max(0, int((self.data.get("lockUntil", 0) - now) * 1000))

    def state(self, open_flag=False):
        rem = self.lock_remaining_ms()
        return {
            "open": bool(open_flag),
            "blocked": rem > 0,
            "remainingMs": rem,
            "attemptsLeft": max(0, self.max_attempts - int(self.data.get("fails", 0))),
            "changedAt": self.data.get("changedAt"),
        }

    def verify(self, pin):
        with self.lock:
            now = self.clock()
            if self.lock_remaining_ms(now) > 0:
                self._audit("unlock", False, "ввод заблокирован")
                self._persist()
                return {"ok": False, "reason": "blocked",
                        "remainingMs": self.lock_remaining_ms(now),
                        "attemptsLeft": 0}
            pin = re.sub(r"\D", "", str(pin or ""))
            if not 4 <= len(pin) <= 8:
                return {"ok": False, "reason": "format", "attemptsLeft": self.state()["attemptsLeft"]}
            if self._hash(self.data["salt"], pin) == self.data["pinHash"]:
                self.data["fails"] = 0
                self.data["lockUntil"] = 0
                self.data["lastOpenAt"] = now
                self._audit("unlock", True, "доступ разрешён")
                self._persist()
                return {"ok": True, "reason": "ok", "attemptsLeft": self.max_attempts}
            if self.data.get("lockUntil") and self.data["lockUntil"] <= now:
                self.data["fails"] = 0
                self.data["lockUntil"] = 0
            self.data["fails"] = int(self.data.get("fails", 0)) + 1
            if self.data["fails"] >= self.max_attempts:
                self.data["fails"] = 0
                self.data["lockUntil"] = now + self.lock_ms / 1000.0
                self._audit("unlock", False, "блокировка после %d попыток" % self.max_attempts)
                self._persist()
                return {"ok": False, "reason": "blocked", "remainingMs": self.lock_ms, "attemptsLeft": 0}
            self._audit("unlock", False, "неверный PIN")
            self._persist()
            return {"ok": False, "reason": "wrong",
                    "attemptsLeft": max(0, self.max_attempts - self.data["fails"])}

    def set_pin(self, current, new):
        res = self.verify(current)
        if not res.get("ok"):
            return {"ok": False, "reason": res.get("reason"), "detail": "текущий PIN неверен"}
        new = re.sub(r"\D", "", str(new or ""))
        if not 4 <= len(new) <= 8:
            return {"ok": False, "reason": "format", "detail": "новый PIN — 4…8 цифр"}
        with self.lock:
            self.data["salt"] = secrets.token_hex(16)
            self.data["pinHash"] = self._hash(self.data["salt"], new)
            self.data["changedAt"] = self.clock()
            self._audit("pin_change", True, "PIN изменён")
            self._persist()
        return {"ok": True}

    def audit(self, limit=20):
        items = self.data["audit"][-int(limit):][::-1]
        return [{k: v for k, v in e.items() if k != "hash"} for e in items]


def hardware_lock(action):
    """Реальный замок: внешняя команда из окружения (реле/соленоид/GPIO)."""
    cmd = os.environ.get("RUS_SLAM_LOCK_CMD_CLOSE" if action == "close" else "RUS_SLAM_LOCK_CMD")
    if not cmd:
        return "software"
    try:
        subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return "hardware"
    except OSError as exc:
        print("! замок: команда не выполнена (%s)" % exc)
        return "error"


# ============================================================================
# 5. Состояние приложения
# ============================================================================
class App:
    def __init__(self, source, vault, cargo_nominal_kg=80):
        self.source = source
        self.vault = vault
        self.cargo_nominal_kg = cargo_nominal_kg
        self.lock_open = False
        self.started = time.time()

    def snapshot(self):
        data = self.source.read()
        lock_state = self.vault.state(self.lock_open)
        cargo = dict(data.get("cargo") or {})
        cargo["closed"] = not self.lock_open
        # Вес груза — только если источник его реально передал (датчика пока нет):
        # номинальные 80 кг раньше подменяли отсутствующее значение.
        out = dict(data)
        out["cargo"] = cargo
        out["lock"] = lock_state
        out["source"] = getattr(self.source, "name", "unknown")
        out["uptimeSec"] = int(time.time() - self.started)
        out["ts"] = time.time()
        mgr_scan = None
        try:
            from robot_control.lidar import LidarManager
            mgr_scan = LidarManager.get_instance().get_scan()
        except Exception:
            pass
        if mgr_scan is not None:
            out["lidar"] = mgr_scan
            # Реальный скан (USB, ROS 2, HTTP). Демо-скана в системе нет.
            out["lidarDemo"] = False
        return out

    def open_lock(self, pin):
        res = self.vault.verify(pin)
        if res.get("ok"):
            self.lock_open = True
            res["hardware"] = hardware_lock("open")
            res["lock"] = self.vault.state(True)
        else:
            res["lock"] = self.vault.state(self.lock_open)
        return res

    def close_lock(self):
        if not self.lock_open:
            return {"ok": True, "lock": self.vault.state(False)}
        self.lock_open = False
        return {"ok": True, "hardware": hardware_lock("close"), "lock": self.vault.state(False)}


# ============================================================================
# 6. HTTP: статика + JSON API
# ============================================================================
class ApiHandler(SimpleHTTPRequestHandler):
    app = None                      # проставляется в main()

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=ROOT, **kwargs)

    # --- утилиты ---
    def _json(self, payload, code=200):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        try:
            length = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(length) if length else b"{}"
            return json.loads(raw.decode("utf-8") or "{}")
        except (ValueError, UnicodeDecodeError, OSError):
            return {}

    # --- маршруты ---
    def do_GET(self):                                            # noqa: N802
        path = self.path.split("?", 1)[0]
        if path == "/api/state":
            return self._json({"ok": True, "data": self.app.snapshot()})
        if path == "/api/audit":
            limit = 20
            m = re.search(r"[?&]limit=(\d+)", self.path)
            if m:
                limit = max(1, min(200, int(m.group(1))))
            return self._json({"ok": True, "audit": self.app.vault.audit(limit),
                               "integrity": self.app.vault.verify_audit()})
        if path == "/api/health":
            return self._json({"ok": True, "source": getattr(self.app.source, "name", "?"),
                               "uptimeSec": int(time.time() - self.app.started),
                               "pinDefault": self.app.vault.default_pin})
        if path in ("/", "/index.htm", "/default.html"):
            self.send_response(302)
            self.send_header("Location", MAIN_PAGE)
            self.end_headers()
            return
        if path == "/console":
            self.send_response(302)
            self.send_header("Location", CONSOLE_PAGE)
            self.end_headers()
            return
        super().do_GET()

    def do_POST(self):                                           # noqa: N802
        path = self.path.split("?", 1)[0]
        body = self._body()
        if path == "/api/lock/open":
            res = self.app.open_lock(body.get("pin", ""))
            return self._json(res, 200 if res.get("ok") else 403)
        if path == "/api/lock/close":
            return self._json(self.app.close_lock())
        if path == "/api/lock/pin":
            res = self.app.vault.set_pin(body.get("current", ""), body.get("new", ""))
            if res.get("ok"):
                self.app.close_lock()
            return self._json(res, 200 if res.get("ok") else 403)
        return self._json({"ok": False, "error": "неизвестный метод"}, 404)

    # --- служебное ---
    def end_headers(self):
        self.send_header("Cache-Control", "no-store, must-revalidate")
        super().end_headers()

    def log_message(self, fmt, *args):
        status = str(args[1]) if len(args) > 1 else ""
        if status.startswith(("4", "5")):
            super().log_message(fmt, *args)


# ============================================================================
# 7. Запуск
# ============================================================================
KIOSK_CANDIDATES = ["chromium-browser", "chromium", "google-chrome",
                    "google-chrome-stable", "microsoft-edge", "firefox"]


def local_ips():
    try:
        return [ip for ip in socket.gethostbyname_ex(socket.gethostname())[2]
                if not ip.startswith("127.")]
    except OSError:
        return []


def launch_kiosk(url):
    exe = next((shutil.which(c) for c in KIOSK_CANDIDATES if shutil.which(c)), None)
    if not exe:
        print("! браузер не найден — откройте вручную:", url)
        return
    base = os.path.basename(exe)
    args = ([exe, "--kiosk", url] if "firefox" in base
            else [exe, "--kiosk", "--app=" + url, "--noerrdialogs", "--disable-infobars",
                  "--autoplay-policy=no-user-gesture-required"])
    try:
        subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("· браузер открыт в киоск-режиме:", base)
    except OSError as exc:
        print("! не удалось открыть браузер:", exc)


def make_source(kind, ports):
    if kind == "serial":
        return SerialSource(ports or ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"])
    if kind == "ros":
        return RosSource()
    if kind == "auto":
        return RosSource()
    return NoSource()


def build_app(args):
    source = make_source(args.source, args.ports)
    vault = Vault(args.lock_file or LOCK_FILE, args.pin or DEFAULT_PIN,
                  max_attempts=args.max_attempts, lock_ms=int(args.lock_sec * 1000))
    return App(source, vault)


def main(argv=None):
    ap = argparse.ArgumentParser(description="RUS SLAM: основной экран робота и API")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--source", choices=["none", "serial", "ros", "auto"], default="auto",
                    help="источник данных; без робота значения пустые (демо-режима нет)")
    ap.add_argument("--ports", default="", help="UART-порты для --source serial, через запятую")
    ap.add_argument("--pin", default=DEFAULT_PIN, help="заводской PIN отсека (по умолчанию 7232)")
    ap.add_argument("--lock-file", default="", help="файл состояния замка (по умолчанию gui/state/lock.json)")
    ap.add_argument("--max-attempts", type=int, default=MAX_ATTEMPTS)
    ap.add_argument("--lock-sec", type=float, default=LOCK_MS / 1000.0)
    ap.add_argument("--kiosk", action="store_true",
                    help="опция: открыть основной экран браузером на дисплее робота (без рамок)")
    ap.add_argument("--open", action="store_true",
                    help="опция: открыть основной экран в браузере робота (с рамками)")
    ap.add_argument("--delay", type=float, default=1.2)
    ap.add_argument("--reset-pin", action="store_true", help="сбросить PIN к заводскому")
    args = ap.parse_args(argv)

    lock_path = args.lock_file or LOCK_FILE
    if args.reset_pin:
        try:
            os.remove(lock_path)
            print("· PIN сброшен к заводскому (%s)" % args.pin)
        except OSError:
            pass

    app = build_app(args)
    if not os.path.exists(lock_path):
        app.vault._persist()                                  # noqa: SLF001 — первичная запись
    ApiHandler.app = app

    server = ThreadingHTTPServer((args.host, args.port),
                                 partial(ApiHandler))
    url = "http://127.0.0.1:%d%s" % (args.port, MAIN_PAGE)

    print("RUS SLAM · сервер борта: страницы и API (сам окон не открывает)")
    print("  источник данных : %s" % getattr(app.source, "name", "?"))
    print("  на роботе       : %s" % url)
    for ip in local_ips():
        print("  с планшета/ПК   : http://%s:%d%s" % (ip, args.port, MAIN_PAGE))
    print("  пульт (удалённо): http://127.0.0.1:%d%s" % (args.port, CONSOLE_PAGE))
    print("  API             : /api/state · /api/lock/open · /api/lock/close · /api/audit · /api/health")
    print("  дисплей робота  : страницу показывает deploy/rus-slam-display.service")
    print("                    (разово — флаг --kiosk); сервер только отдаёт данные")
    print("  Ctrl+C — остановить\n")

    if args.kiosk or args.open:
        def opener():
            time.sleep(max(0.0, args.delay))
            launch_kiosk(url) if args.kiosk else webbrowser.open(url)
        threading.Thread(target=opener, daemon=True).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n· остановлено оператором")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
