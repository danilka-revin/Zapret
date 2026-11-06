"""Модуль датчиков робота: разбор кадров и USB-клиент бортового экрана.

Плата (``firmware/sensor_module``) отвечает за свет, замок двери, концевик,
карты доступа и компас. Сервер ничего из этого не вычисляет — он только
показывает состояние и отправляет команды, а безопасное состояние держит
прошивка (после сброса реле выключены, замок закрыт).

Протокол — ASCII-строки 115200 с CRC-16/Modbus в конце, как у колёс:

* ``@S``    — телеметрия, 5 Гц;
* ``@CARD`` — приложили карту (пускать или нет);
* ``@EVT``  — дверь/замок;
* ``@ACK``  — ответ на команду;
* ``#``     — сообщение для человека, в журнал.

Строка без верного CRC отбрасывается: порча линии не должна превращаться в
«дверь открыта» или «свет включён». CRC не является защитой от подмены —
это только контроль целостности.
"""

from __future__ import annotations

import re
import threading
import time
from typing import Any, Callable, Iterable

#: Версия протокола платы; сервер понимает ровно её (см. поле ``fw=``).
SENSOR_PROTOCOL = 1

TELEMETRY_PREFIX = "@S "
CARD_PREFIX = "@CARD "
EVENT_PREFIX = "@EVT "
ACK_PREFIX = "@ACK "
NOTE_PREFIX = "#"

#: Биты ошибок в поле ``err=`` — те же значения, что в прошивке.
ERROR_BITS: tuple[tuple[int, str], ...] = (
    (1 << 0, "MPU6050"),
    (1 << 1, "HMC5883L"),
    (1 << 2, "BMP180"),
    (1 << 3, "RC522"),
    (1 << 4, "EEPROM"),
    (1 << 5, "замок"),
    (1 << 6, "шина I²C"),
)

#: Свет: имя → бит поля ``light=``. Реле: head, stop, left, right; dim — лента.
LIGHT_BITS: dict[str, int] = {
    "head": 1 << 0,
    "stop": 1 << 1,
    "left": 1 << 2,
    "right": 1 << 3,
    "dim": 1 << 4,
}

#: Последнее событие RFID (поле ``rfid=``).
RFID_STATES: dict[int, str] = {
    0: "none",
    1: "granted",
    2: "denied",
    3: "enrolled",
    4: "deleted",
}

UID_RE = re.compile(r"^[0-9A-Fa-f]{8}$")
KEY_RE = re.compile(r"([a-z]+)=(\S+)")
TELEMETRY_FIELDS = (
    "fw", "hdg", "pitch", "roll", "gz", "press", "temp", "alt",
    "door", "lock", "light", "dim", "rfid", "cards", "link", "err", "t",
)

#: Пределы, за которыми кадр считается испорченным, а не просто странным.
HEADING_RANGE = (-36000, 36000)
TILT_RANGE = (-9001, 9001)
STALE_AFTER_S = 2.0


class SensorFrameError(ValueError):
    """Кадр не разобран: битый CRC, не тот набор полей или значения вне пределов."""


def crc16_modbus(text: str) -> int:
    """CRC-16/Modbus по ASCII-хвосту кадра — тот же расчёт, что на плате."""
    crc = 0xFFFF
    for byte in text.encode("ascii", "replace"):
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def split_checked(line: str) -> str:
    """Отделить полезную часть кадра от ``crc=`` и проверить CRC."""
    text = line.strip()
    if not text:
        raise SensorFrameError("пустая строка")
    head, _, crc_text = text.rpartition(" crc=")
    if not head or len(crc_text) != 4:
        raise SensorFrameError("нет поля crc=")
    try:
        expected = int(crc_text, 16)
    except ValueError as exc:
        raise SensorFrameError("crc не шестнадцатеричный") from exc
    actual = crc16_modbus(head)
    if actual != expected:
        raise SensorFrameError("CRC не сходится: %04X != %04X" % (expected, actual))
    return head


def _fields(head: str) -> dict[str, str]:
    return {key: value for key, value in KEY_RE.findall(head)}


def _int(fields: dict[str, str], key: str) -> int:
    value = fields.get(key)
    if value is None:
        raise SensorFrameError("нет поля %s=" % key)
    try:
        return int(value, 10)
    except ValueError as exc:
        raise SensorFrameError("поле %s= не число: %r" % (key, value)) from exc


def parse_telemetry(line: str) -> dict[str, Any]:
    """Разобрать кадр ``@S`` в словарь состояния (углы — в градусах)."""
    head = split_checked(line)
    if not head.startswith(TELEMETRY_PREFIX):
        raise SensorFrameError("это не телеметрия модуля датчиков")
    fields = _fields(head[len(TELEMETRY_PREFIX):])
    missing = [name for name in TELEMETRY_FIELDS if name not in fields]
    if missing:
        raise SensorFrameError("нет полей: %s" % ", ".join(missing))
    protocol = _int(fields, "fw")
    if protocol != SENSOR_PROTOCOL:
        raise SensorFrameError("версия протокола %d, сервер ждёт %d" % (protocol, SENSOR_PROTOCOL))
    heading = _int(fields, "hdg")
    pitch = _int(fields, "pitch")
    roll = _int(fields, "roll")
    if not HEADING_RANGE[0] <= heading <= HEADING_RANGE[1]:
        raise SensorFrameError("hdg вне пределов: %d" % heading)
    if not TILT_RANGE[0] <= pitch <= TILT_RANGE[1] or not TILT_RANGE[0] <= roll <= TILT_RANGE[1]:
        raise SensorFrameError("pitch/roll вне пределов: %d/%d" % (pitch, roll))
    mask = _int(fields, "light")
    if not 0 <= mask <= 0x1F:
        raise SensorFrameError("light вне пределов: %d" % mask)
    dim = _int(fields, "dim")
    if not 0 <= dim <= 255:
        raise SensorFrameError("dim вне пределов: %d" % dim)
    cards = _int(fields, "cards")
    if not 0 <= cards <= 12:
        raise SensorFrameError("cards вне пределов: %d" % cards)
    rfid_code = _int(fields, "rfid")
    if rfid_code not in RFID_STATES:
        raise SensorFrameError("rfid=%d неизвестен" % rfid_code)
    return {
        "protocol": protocol,
        "headingCd": heading,
        "heading": round(heading / 100.0, 2),
        "pitch": round(pitch / 100.0, 2),
        "roll": round(roll / 100.0, 2),
        "gyroZ": round(_int(fields, "gz") / 10.0, 1),
        "pressurePa": _int(fields, "press"),
        "pressureHpa": round(_int(fields, "press") / 100.0, 1),
        "temperatureC": round(_int(fields, "temp") / 10.0, 1),
        "altitudeM": round(_int(fields, "alt") / 100.0, 2),
        "doorOpen": _int(fields, "door") == 1,
        "lockOpen": _int(fields, "lock") == 1,
        "lightMask": mask,
        "lights": {name: bool(mask & bit) for name, bit in LIGHT_BITS.items()},
        "dim": dim,
        "rfid": RFID_STATES[rfid_code],
        "cards": cards,
        "linkOk": _int(fields, "link") == 1,
        "errorsMask": _int(fields, "err"),
        "errors": [name for bit, name in ERROR_BITS if _int(fields, "err") & bit],
        "uptimeMs": _int(fields, "t"),
    }


def parse_card(line: str) -> dict[str, Any]:
    """Разобрать кадр ``@CARD``: какую карту приложили и пустили ли её."""
    head = split_checked(line)
    if not head.startswith(CARD_PREFIX):
        raise SensorFrameError("это не событие карты")
    fields = _fields(head[len(CARD_PREFIX):])
    uid = fields.get("uid", "")
    if not UID_RE.match(uid):
        raise SensorFrameError("uid не 8 hex-знаков: %r" % uid)
    verdict = _int(fields, "ok")
    if verdict not in (0, 1):
        raise SensorFrameError("ok=%d вне пределов" % verdict)
    return {
        "uid": uid.upper(),
        "granted": verdict == 1,
        "cards": _int(fields, "cards"),
        "atMs": _int(fields, "t"),
    }


def parse_event(line: str) -> dict[str, Any]:
    """Разобрать кадр ``@EVT`` (дверь или замок)."""
    head = split_checked(line)
    if not head.startswith(EVENT_PREFIX):
        raise SensorFrameError("это не событие платы")
    fields = _fields(head[len(EVENT_PREFIX):])
    kind = fields.get("kind", "")
    if kind not in ("door", "lock"):
        raise SensorFrameError("неизвестное событие: %r" % kind)
    opened = _int(fields, "open")
    if opened not in (0, 1):
        raise SensorFrameError("open=%d вне пределов" % opened)
    return {"event": kind, "open": opened == 1, "atMs": _int(fields, "t")}


def parse_ack(line: str) -> dict[str, Any]:
    """Разобрать кадр ``@ACK`` — подтверждение команды."""
    head = split_checked(line)
    if not head.startswith(ACK_PREFIX):
        raise SensorFrameError("это не подтверждение команды")
    fields = _fields(head[len(ACK_PREFIX):])
    name = fields.get("cmd", "")
    if not name:
        raise SensorFrameError("в подтверждении нет cmd=")
    verdict = _int(fields, "ok")
    if verdict not in (0, 1):
        raise SensorFrameError("ok=%d вне пределов" % verdict)
    return {
        "id": _int(fields, "id"),
        "cmd": name,
        "ok": verdict == 1,
        "atMs": _int(fields, "t"),
    }


def parse_line(line: str) -> dict[str, Any] | None:
    """Разобрать любую строку платы; ``None`` — это сообщение человеку."""
    text = line.strip()
    if not text:
        return None
    if text.startswith(NOTE_PREFIX):
        return {"kind": "note", "text": text[len(NOTE_PREFIX):].strip()}
    if text.startswith(TELEMETRY_PREFIX):
        return {"kind": "telemetry", **parse_telemetry(text)}
    if text.startswith(CARD_PREFIX):
        return {"kind": "card", **parse_card(text)}
    if text.startswith(EVENT_PREFIX):
        return {"kind": "event", **parse_event(text)}
    if text.startswith(ACK_PREFIX):
        return {"kind": "ack", **parse_ack(text)}
    raise SensorFrameError("неизвестный кадр: %r" % text[:32])


# ------------------------------- команды ------------------------------------
def command_light(name: str, on: bool) -> str:
    """``light head on`` и т.п. Имена проверяются: опечатка не должна дойти."""
    key = str(name).strip().lower()
    if key not in LIGHT_BITS:
        raise ValueError("неизвестный свет: %r" % name)
    return "light %s %s" % (key, "on" if on else "off")


def command_lights_all(on: bool) -> str:
    return "light all %s" % ("on" if on else "off")


def command_dim(level: int) -> str:
    value = int(level)
    if not 0 <= value <= 255:
        raise ValueError("яркость вне 0…255: %r" % level)
    return "dim %d" % value


def command_lock(open_: bool, seconds: float | None = None) -> str:
    if not open_:
        return "lock close"
    if seconds is None:
        return "lock open"
    value = int(round(float(seconds)))
    if not 1 <= value <= 10:
        raise ValueError("замок открывается на 1…10 с, а не %r" % seconds)
    return "lock open %d" % value


def command_card(action: str, uid: str | None = None) -> str:
    key = str(action).strip().lower()
    if key == "list":
        return "card list"
    if key == "learn":
        return "card learn"
    if key == "clear":
        return "card clear"
    if key not in ("add", "del"):
        raise ValueError("неизвестное действие с картами: %r" % action)
    text = str(uid or "").strip()
    if not UID_RE.match(text):
        raise ValueError("UID карты — 8 hex-знаков: %r" % uid)
    return "card %s %s" % (key, text.upper())


def command_mag(action: str) -> str:
    key = str(action).strip().lower()
    if key not in ("start", "stop", "reset"):
        raise ValueError("неизвестная калибровка компаса: %r" % action)
    return "mag cal %s" % key


# -------------------------------- клиент ------------------------------------
class SensorModule:
    """Чтение платы по USB и очередь команд. Поток — демон, закрывается stop()."""

    def __init__(self, port: str, baudrate: int = 115200,
                 serial_factory: Callable[..., Any] | None = None,
                 clock: Callable[[], float] = time.monotonic,
                 log: Callable[[str], None] | None = None) -> None:
        self.port = port
        self.baudrate = baudrate
        self._serial_factory = serial_factory
        self._clock = clock
        self._log = log or (lambda message: None)
        self._serial = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._state: dict[str, Any] | None = None
        self._state_at: float = 0.0
        self._events: list[dict[str, Any]] = []
        self._frames = 0
        self._bad_frames = 0
        self._error: str | None = None
        self._last_ack: dict[str, Any] | None = None
        self._last_card: dict[str, Any] | None = None

    # --- жизненный цикл ---
    def open(self) -> None:
        if self._serial_factory is None:
            import serial  # noqa: PLC0415 — pyserial нужен только на железе

            self._serial_factory = serial.Serial
        with self._lock:
            if self._serial is not None:
                return
            self._serial = self._serial_factory(
                self.port, self.baudrate, timeout=0.2,
            )
            self._error = None

    def start(self) -> "SensorModule":
        if self._thread is not None and self._thread.is_alive():
            return self
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="sensor-module", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=2.0)
        with self._lock:
            serial_port, self._serial = self._serial, None
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:  # noqa: BLE001 — порт мог отвалиться сам
                pass

    # --- поток ---
    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self.open()
                port = self._serial
                if port is None:
                    self._stop.wait(0.5)
                    continue
                line = port.readline()
                if line:
                    self.feed(line.decode("ascii", "replace"))
                    continue
                self._stop.wait(0.01)
            except Exception as exc:  # noqa: BLE001 — обрыв USB не должен ронять экран
                self._error = str(exc) or exc.__class__.__name__
                self._log("модуль датчиков: %s" % self._error)
                with self._lock:
                    dead, self._serial = self._serial, None
                if dead is not None:
                    try:
                        dead.close()
                    except Exception:  # noqa: BLE001
                        pass
                self._stop.wait(1.0)

    def feed(self, line: str) -> dict[str, Any] | None:
        """Обработать одну строку платы. Возвращает разобранный кадр или None."""
        try:
            frame = parse_line(line)
        except SensorFrameError as exc:
            with self._lock:
                self._bad_frames += 1
                self._error = str(exc)
            return None
        if frame is None:
            return None
        with self._lock:
            self._frames += 1
            now = self._clock()
            if frame["kind"] == "telemetry":
                self._state = frame
                self._state_at = now
            elif frame["kind"] == "card":
                self._last_card = frame
                self._push_event(frame)
            elif frame["kind"] == "event":
                self._push_event(frame)
            elif frame["kind"] == "ack":
                self._last_ack = frame
                self._push_event(frame)
            else:
                self._push_event(frame)
        return frame

    def _push_event(self, frame: dict[str, Any]) -> None:
        self._events.append(frame)
        del self._events[:-50]

    # --- команды ---
    def send(self, command: str) -> None:
        text = command.strip()
        if not text:
            raise ValueError("пустая команда")
        self.open()
        with self._lock:
            port = self._serial
            if port is None:
                raise RuntimeError("порт модуля датчиков не открыт")
            port.write((text + "\r\n").encode("ascii"))
            port.flush()

    def light(self, name: str, on: bool) -> None:
        self.send(command_light(name, on))

    def lights_all(self, on: bool) -> None:
        self.send(command_lights_all(on))

    def set_dim(self, level: int) -> None:
        self.send(command_dim(level))

    def lock(self, open_: bool, seconds: float | None = None) -> None:
        self.send(command_lock(open_, seconds))

    def cards(self) -> None:
        self.send(command_card("list"))

    def card(self, action: str, uid: str | None = None) -> None:
        self.send(command_card(action, uid))

    def mag_calibration(self, action: str) -> None:
        self.send(command_mag(action))

    # --- состояние ---
    def state(self) -> dict[str, Any]:
        """Снимок для экрана и API; безопасен, пока порт молчит."""
        with self._lock:
            frame = self._state
            age = (self._clock() - self._state_at) if frame else None
            snapshot = {
                "ok": frame is not None and age is not None and age <= STALE_AFTER_S,
                "configured": True,
                "port": self.port,
                "protocol": frame["protocol"] if frame else SENSOR_PROTOCOL,
                "frames": self._frames,
                "badFrames": self._bad_frames,
                "error": self._error,
                "ageMs": int(age * 1000) if age is not None else None,
                "stale": age is not None and age > STALE_AFTER_S,
                "lastAck": dict(self._last_ack) if self._last_ack else None,
                "lastCard": dict(self._last_card) if self._last_card else None,
                "events": [dict(item) for item in self._events[-10:]],
            }
            if frame is None:
                snapshot.update({
                    "heading": None, "headingCd": None, "pitch": None, "roll": None,
                    "gyroZ": None, "pressurePa": None, "pressureHpa": None,
                    "temperatureC": None, "altitudeM": None, "doorOpen": None,
                    "lockOpen": None, "lightMask": 0, "lights": {}, "dim": None,
                    "rfid": "none", "cards": 0, "linkOk": False, "errors": [],
                    "errorsMask": 0, "uptimeMs": None,
                })
            else:
                snapshot.update({key: value for key, value in frame.items() if key != "kind"})
            return snapshot

    def compass(self) -> dict[str, Any]:
        """Короткий снимок для карты лидара: курс и то, что видно на двери."""
        state = self.state()
        return {
            "ok": state["ok"],
            "heading": state["heading"],
            "pitch": state["pitch"],
            "roll": state["roll"],
            "port": state["port"],
            "ageMs": state["ageMs"],
            "stale": state["stale"],
            "doorOpen": state["doorOpen"],
            "lockOpen": state["lockOpen"],
            "errors": state["errors"],
        }


def sensor_port(env: dict[str, str] | None = None) -> str | None:
    """Порт модуля датчиков задаётся вручную: автопоиск USB выключен по правилам."""
    source = env if env is not None else __import__("os").environ
    value = (source.get("RC_SENSOR_PORT") or "").strip()
    return value or None


def open_sensor_module(env: dict[str, str] | None = None, *, clock: Callable[[], float] = time.monotonic,
                       serial_factory: Callable[..., Any] | None = None,
                       log: Callable[[str], None] | None = None) -> SensorModule | None:
    """Собрать клиент по ``RC_SENSOR_PORT``; без порта — ``None`` (демо нет)."""
    port = sensor_port(env)
    if not port:
        return None
    return SensorModule(port, serial_factory=serial_factory, clock=clock, log=log)


def note_texts(events: Iterable[dict[str, Any]]) -> list[str]:
    """Тексты сообщений платы из снимка — для журнала бортового экрана."""
    return [item["text"] for item in events if item.get("kind") == "note" and item.get("text")]
