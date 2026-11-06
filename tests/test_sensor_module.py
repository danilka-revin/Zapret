"""Модуль датчиков: прошивка, протокол USB и его место в бортовом экране.

Что проверяем:

* распиновка и правила безопасности в скетче ``firmware/sensor_module`` —
  они заданы разводкой платы, а не кодом сервера;
* хост-симуляция `tests/firmware_sim/sensor_sim.cpp`: GY-87 на I²C, RC522 на SPI
  с картой в поле, лента, реле, серво замка и концевик. Сценарии идут в
  отдельных процессах, как после сброса платы;
* разбор кадров ``@S``/``@CARD``/``@EVT``/``@ACK`` сервером: те же числа, что
  печатает прошивка, включая CRC-16/Modbus;
* маршруты ``/api/sensor_module*`` и курс на карте лидара.

Сборка настоящим avr-gcc проверяется отдельно и пропускается, если нет
``AVR_ROOT``/``CORE_ROOT`` и библиотеки Adafruit NeoPixel (см.
``firmware/tools/avr_build.py``).
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest

from robot_control import sensor_module as sm
from robot_control.config import Config
from robot_control.kiosk_app import create_kiosk_app

ROOT = Path(__file__).resolve().parent.parent
SKETCH = ROOT / "firmware" / "sensor_module" / "sensor_module.ino"
SIM_DIR = ROOT / "tests" / "firmware_sim"


def _text() -> str:
    return SKETCH.read_text(encoding="utf-8")


def _const(name: str) -> str:
    match = re.search(r"\bconst\s+[\w: ]+?\b%s\s*=\s*([^;]+);" % re.escape(name), _text())
    assert match, name
    return match.group(1).strip()


def frame(payload: str) -> str:
    """Дописать CRC так, как это делает плата."""
    return "%s crc=%04X" % (payload, sm.crc16_modbus(payload))


def telemetry(**overrides) -> str:
    fields = {
        "fw": sm.SENSOR_PROTOCOL, "hdg": 0, "pitch": 0, "roll": 0, "gz": 0,
        "press": 69964, "temp": 150, "alt": 301674, "door": 0, "lock": 0,
        "light": 0, "dim": 80, "strip": 0, "rfid": 0, "cards": 0, "link": 1, "err": 0,
        "t": 1234,
    }
    fields.update(overrides)
    body = " ".join("%s=%s" % (key, value) for key, value in fields.items())
    return frame("@S " + body)


# ============================ прошивка: пины и правила ====================== #
def test_pinout_is_exactly_the_wired_one():
    """Распиновка задана разводкой: ошибка в пине — это сгоревший RC522."""
    text = _text()
    assert _const("RC522_SS_PIN") == "53"          # SPI Mega: SS
    assert _const("RC522_RST_PIN") == "9"
    assert _const("STRIP_PIN") == "6"
    assert _const("LOCK_SERVO_PIN") == "7"
    assert _const("DOOR_SWITCH_PIN") == "8"
    assert re.search(r"RELAY_PINS\[4\]\s*=\s*\{2,\s*3,\s*4,\s*5\}", text)
    # GY-87 на аппаратном I²C Mega: SDA = 20, SCL = 21 (Wire.begin() без аргументов).
    assert "Wire.begin()" in text and "Wire.begin(" not in text.replace("Wire.begin()", "")
    assert "SDA — 20" in text and "SCL — 21" in text


def test_rc522_is_fed_from_3v3_and_documented_as_such():
    """RC522 питается только от 3.3 В — 5 В на нём сжигает вход."""
    text = _text()
    assert "3.3 В" in text or "3.3V" in text
    assert "3.3 В!" in text or "3.3 В" in text
    # Никакого питания RC522 от пинов Mega в коде нет: только сигналы.
    assert "RC522_VCC" not in text


def test_relays_and_strip_are_off_after_reset_and_watchdog_runs():
    """После сброса свет выключен, замок закрыт; watchdog включён и кормится."""
    text = _text()
    setup = text[text.index("void setup()"):text.index("void loop()")]
    assert "relayWrite(i, false)" in setup      # реле выключены до датчиков
    assert "lockSet(false, 0)" in setup         # замок закрыт
    assert "strip.clear()" in setup
    assert "wdt_enable(WDTO_2S)" in setup
    loop = text[text.index("void loop()"):]
    assert "wdt_reset()" in loop


def test_sketch_avoids_heap_and_blocking_prints():
    """Как и в прошивке колёс: без String и без Serial.print в обход буфера."""
    text = _text()
    assert "String " not in text
    assert "Serial.print" not in text
    assert "malloc(" not in text and "new " not in text
    # Обмен с картой неблокирующий: ожидание ответа идёт через rcPoll, а не
    # циклом do/while внутри обмена.
    assert "rcStart(" in text and "rcPoll(" in text
    assert "do {" not in text.split("void rcPoll")[0].split("bool rcTransceive")[-1]


def test_sketch_version_matches_server_protocol():
    # 2 — в кадр добавлено поле strip: разбор на сервере обязан идти в ногу.
    assert int(_const("SENSOR_PROTOCOL")) == sm.SENSOR_PROTOCOL == 2


def test_led_strip_is_off_until_it_is_actually_wired():
    """Ленты пока нет: прошивка не должна зависеть от неё ни в чём, кроме света."""
    text = _text()
    # По умолчанию лента выключена и включается одной строкой или флагом сборки.
    assert re.search(r"#ifndef SENSOR_STRIP_ENABLED\s*\n#define SENSOR_STRIP_ENABLED 0", text)
    assert '"-DSENSOR_STRIP_ENABLED=1"' in text or "-DSENSOR_STRIP_ENABLED=1" in text
    assert re.search(r"static const bool STRIP_ENABLED = SENSOR_STRIP_ENABLED != 0;", text)
    # Без ленты пиксели не пишутся, а линия данных прижата к земле.
    strip_fn = text[text.index("void applyStrip"):text.index("void lightsRefresh")]
    assert strip_fn.index("if (!STRIP_ENABLED) return;") < strip_fn.index("if (!strip_auto) return;")
    setup = text[text.index("void setup()"):text.index("void loop()")]
    assert "if (STRIP_ENABLED) {" in setup and "digitalWrite(STRIP_PIN, LOW)" in setup
    # Команда strip принимается и без ленты: сервер не должен получать отказ.
    assert 'strcmp_P(word, PSTR("strip"))' in text
    # Признак ленты уезжает в телеметрию — экран не обещает габаритов зря.
    assert "strip=%d" in text


# ============================== хост-симуляция ============================== #
def _compiler() -> str | None:
    return shutil.which(os.environ.get("CXX", "g++")) or shutil.which("clang++")


def _build(binary: Path, *, strip: bool = False) -> Path:
    """Собрать хост-симуляцию: по умолчанию без ленты, с флагом — с лентой."""
    compiler = _compiler()
    if not compiler:
        pytest.skip("нет g++/clang++ — симуляция модуля датчиков пропущена")
    command = [compiler, "-std=c++17", "-O1", "-w"]
    if strip:
        command.append("-DSENSOR_STRIP_ENABLED=1")
    command += ["-I", str(SIM_DIR / "stubs"), str(SIM_DIR / "sensor_sim.cpp"),
                "-o", str(binary)]
    result = subprocess.run(command, capture_output=True, text=True, timeout=300)
    assert result.returncode == 0, result.stderr[-4000:]
    return binary


@pytest.fixture(scope="module")
def simulator(tmp_path_factory):
    return _build(tmp_path_factory.mktemp("sensor-sim") / "sensor_sim")


@pytest.fixture(scope="module")
def simulator_with_strip(tmp_path_factory):
    return _build(tmp_path_factory.mktemp("sensor-sim-strip") / "sensor_sim", strip=True)


def test_firmware_simulation_scenarios(simulator):
    result = subprocess.run([str(simulator)], capture_output=True, text=True, timeout=900)
    report = result.stdout + "\n" + result.stderr[-6000:]
    assert result.returncode == 0, report
    summary = re.search(r"(\d+)/(\d+) сценариев прошли", result.stdout)
    assert summary and summary.group(1) == summary.group(2), report
    # Свет, замок, карты, компас и отказы датчиков — все сценарии на месте.
    assert int(summary.group(2)) >= 14, report


def test_firmware_with_the_strip_wired_still_paints_it(simulator_with_strip):
    """Ленту подключат позже: тот же скетч с флагом сборки должен её красить."""
    result = subprocess.run([str(simulator_with_strip)], capture_output=True, text=True,
                            timeout=900)
    report = result.stdout + "\n" + result.stderr[-6000:]
    assert result.returncode == 0, report
    summary = re.search(r"(\d+)/(\d+) сценариев прошли", result.stdout)
    assert summary and summary.group(1) == summary.group(2), report


def test_simulated_frames_parse_with_the_server_parser(simulator):
    """Кадры платы разбираются тем же кодом, что и на экране."""
    result = subprocess.run([str(simulator), "dump"], capture_output=True, text=True, timeout=300)
    assert result.returncode == 0, result.stderr[-2000:]
    kinds: dict[str, int] = {}
    cards: list[dict] = []
    acks: list[dict] = []
    telemetry_frames: list[dict] = []
    for line in result.stdout.splitlines():
        if not line.strip():
            continue
        parsed = sm.parse_line(line)
        assert parsed is not None, line
        kinds[parsed["kind"]] = kinds.get(parsed["kind"], 0) + 1
        if parsed["kind"] == "telemetry":
            telemetry_frames.append(parsed)
        elif parsed["kind"] == "card":
            cards.append(parsed)
        elif parsed["kind"] == "ack":
            acks.append(parsed)
    assert telemetry_frames, result.stdout[-2000:]
    assert kinds.get("note", 0) >= 1
    assert acks and all(item["cmd"] for item in acks)
    granted = [item for item in cards if item["granted"]]
    denied = [item for item in cards if not item["granted"]]
    assert granted and granted[0]["uid"] == "A1B2C3D4"
    assert denied and denied[0]["uid"] == "11223344"
    # Барометр опрашивается 2 раза в секунду: в первых кадрах давления ещё нет.
    measured = [item for item in telemetry_frames if item["pressurePa"]]
    assert measured, "ни одного замера барометра за сценарий"
    for item in telemetry_frames:
        assert item["protocol"] == sm.SENSOR_PROTOCOL
        assert -360 <= item["heading"] <= 360
        if item["pressurePa"]:
            # Пример замера из даташита BMP180: UT = 27898, UP = 23843.
            assert item["pressurePa"] == 69964
            assert item["temperatureC"] == 15.0


def test_avr_build_when_toolchain_available():
    """Сборка avr-gcc — только если есть ядро Arduino и библиотека ленты."""
    if not (os.environ.get("AVR_ROOT") and os.environ.get("CORE_ROOT")):
        pytest.skip("нет AVR_ROOT/CORE_ROOT — сборка avr-gcc пропущена")
    if not os.environ.get("NEOPIXEL_ROOT"):
        pytest.skip("нет NEOPIXEL_ROOT (Adafruit_NeoPixel) — сборка avr-gcc пропущена")
    result = subprocess.run(
        ["python3", str(ROOT / "firmware" / "tools" / "avr_build.py"), str(SKETCH)],
        capture_output=True, text=True, timeout=600,
    )
    assert result.returncode == 0, result.stdout[-4000:] + result.stderr[-2000:]


# ============================== разбор кадров =============================== #
def test_telemetry_parses_into_degrees_and_flags():
    parsed = sm.parse_telemetry(telemetry(hdg=12345, pitch=-120, roll=35, gz=12,
                                          door=1, lock=1, light=0b00101, dim=128,
                                          rfid=1, cards=2, err=0b1000, t=98765))
    assert parsed["heading"] == 123.45
    assert parsed["pitch"] == -1.2 and parsed["roll"] == 0.35
    assert parsed["gyroZ"] == 1.2
    assert parsed["pressureHpa"] == 699.6
    assert parsed["altitudeM"] == 3016.74
    assert parsed["doorOpen"] is True and parsed["lockOpen"] is True
    assert parsed["lights"] == {"head": True, "stop": False, "left": True,
                                "right": False, "dim": False}
    assert parsed["stripEnabled"] is False        # лента пока не подключена
    assert parsed["rfid"] == "granted" and parsed["cards"] == 2
    assert parsed["errors"] == ["RC522"]
    assert parsed["uptimeMs"] == 98765


def test_broken_crc_is_rejected():
    line = telemetry()
    broken = line[:-1] + ("0" if line[-1] != "0" else "1")
    with pytest.raises(sm.SensorFrameError):
        sm.parse_telemetry(broken)


def test_truncated_or_alien_frames_are_rejected():
    good = telemetry()
    without_door = good.replace(" door=0", "")
    with pytest.raises(sm.SensorFrameError):
        sm.parse_telemetry(without_door)          # поле пропало — кадр не наш
    # Плата новее сервера: та же форма кадра, но другая версия протокола.
    alien = good.replace("fw=%d" % sm.SENSOR_PROTOCOL, "fw=%d" % (sm.SENSOR_PROTOCOL + 1))
    assert alien != good
    with pytest.raises(sm.SensorFrameError):
        sm.parse_telemetry(alien)
    with pytest.raises(sm.SensorFrameError):
        sm.parse_line("hello world")
    with pytest.raises(sm.SensorFrameError):
        sm.parse_telemetry("")
    assert sm.parse_line("   ") is None
    assert sm.parse_line("# SENSORS: карта записана") == {
        "kind": "note", "text": "SENSORS: карта записана"}


def test_out_of_range_values_are_rejected():
    with pytest.raises(sm.SensorFrameError):
        sm.parse_telemetry(telemetry(hdg=40000))
    with pytest.raises(sm.SensorFrameError):
        sm.parse_telemetry(telemetry(dim=300))
    with pytest.raises(sm.SensorFrameError):
        sm.parse_telemetry(telemetry(cards=40))
    with pytest.raises(sm.SensorFrameError):
        sm.parse_telemetry(telemetry(rfid=9))
    with pytest.raises(sm.SensorFrameError):
        sm.parse_telemetry(telemetry(light=64))


def test_card_event_and_uid_validation():
    parsed = sm.parse_card(frame("@CARD uid=a1b2c3d4 ok=1 cards=2 t=1000"))
    assert parsed == {"uid": "A1B2C3D4", "granted": True, "cards": 2, "atMs": 1000}
    with pytest.raises(sm.SensorFrameError):
        sm.parse_card(frame("@CARD uid=XYZ ok=1 cards=0 t=1"))


def test_event_and_ack_frames():
    event = sm.parse_event(frame("@EVT kind=door open=1 t=42"))
    assert event == {"event": "door", "open": True, "atMs": 42}
    assert sm.parse_line(frame("@EVT kind=door open=1 t=42")) == {
        "kind": "event", "event": "door", "open": True, "atMs": 42}
    ack = sm.parse_ack(frame("@ACK id=7 cmd=lock ok=1 t=99"))
    assert ack == {"id": 7, "cmd": "lock", "ok": True, "atMs": 99}
    with pytest.raises(sm.SensorFrameError):
        sm.parse_event(frame("@EVT kind=lidar open=1 t=1"))


# ============================== команды ==================================== #
def test_commands_are_validated_before_they_reach_the_board():
    assert sm.command_light("head", True) == "light head on"
    assert sm.command_light("LEFT", False) == "light left off"
    assert sm.command_lights_all(False) == "light all off"
    assert sm.command_dim(120) == "dim 120"
    assert sm.command_lock(True) == "lock open"
    assert sm.command_lock(True, 3) == "lock open 3"
    assert sm.command_lock(False) == "lock close"
    assert sm.command_card("add", "a1b2c3d4") == "card add A1B2C3D4"
    assert sm.command_card("list") == "card list"
    assert sm.command_mag("start") == "mag cal start"
    for bad in (lambda: sm.command_light("brake", True),
                lambda: sm.command_dim(256),
                lambda: sm.command_lock(True, 30),
                lambda: sm.command_card("add", "zz"),
                lambda: sm.command_card("steal"),
                lambda: sm.command_mag("go")):
        with pytest.raises(ValueError):
            bad()


# ============================== клиент платы =============================== #
class FakeSerial:
    def __init__(self):
        self.written: list[bytes] = []
        self.incoming: list[bytes] = []
        self.closed = False
        self.fail = False

    def readline(self):
        if self.fail:
            raise OSError("USB отвалился")
        return self.incoming.pop(0) if self.incoming else b""

    def write(self, data):
        self.written.append(data)
        return len(data)

    def flush(self):
        pass

    def close(self):
        self.closed = True


class FakeClock:
    def __init__(self, now: float = 0.0):
        self.now = now

    def __call__(self) -> float:
        return self.now


def make_module(serial_port: FakeSerial | None = None, clock: FakeClock | None = None):
    port = serial_port or FakeSerial()
    clock = clock or FakeClock()
    module = sm.SensorModule("/dev/ttyACM2", serial_factory=lambda *a, **k: port, clock=clock)
    module.open()
    return module, port, clock


def test_state_is_built_from_telemetry_and_goes_stale():
    module, serial_port, clock = make_module()
    clock.now = 10.0
    module.feed(telemetry(hdg=9000, cards=1, link=1))
    state = module.state()
    assert state["ok"] is True and state["stale"] is False
    assert state["heading"] == 90.0 and state["cards"] == 1
    assert state["port"] == "/dev/ttyACM2" and state["configured"] is True
    clock.now = 14.0
    stale = module.state()
    assert stale["stale"] is True and stale["ok"] is False
    assert stale["ageMs"] == 4000


def test_state_without_any_frame_is_not_an_error_but_not_ok():
    module, _, _ = make_module()
    state = module.state()
    assert state["ok"] is False and state["heading"] is None
    assert state["configured"] is True and state["errors"] == []


def test_bad_frames_are_counted_and_do_not_change_state():
    module, _, clock = make_module()
    module.feed(telemetry(hdg=18000))
    before = module.state()["heading"]
    assert module.feed(telemetry(hdg=0)[:-1] + "!") is None
    state = module.state()
    assert state["heading"] == before
    assert state["badFrames"] == 1


def test_cards_acks_and_events_are_kept_for_the_screen():
    module, _, clock = make_module()
    module.feed(frame("@CARD uid=A1B2C3D4 ok=1 cards=1 t=5"))
    module.feed(frame("@EVT kind=lock open=1 t=6"))
    module.feed(frame("@ACK id=3 cmd=lock ok=1 t=7"))
    state = module.state()
    assert state["lastCard"]["uid"] == "A1B2C3D4" and state["lastCard"]["granted"] is True
    assert state["lastAck"]["cmd"] == "lock" and state["lastAck"]["ok"] is True
    assert [item["kind"] for item in state["events"]] == ["card", "event", "ack"]


def test_commands_go_out_as_crlf_lines():
    module, serial_port, _ = make_module()
    module.light("head", True)
    module.lock(True, 2)
    module.card("list")
    module.set_dim(200)
    module.mag_calibration("stop")
    assert serial_port.written == [
        b"light head on\r\n", b"lock open 2\r\n", b"card list\r\n",
        b"dim 200\r\n", b"mag cal stop\r\n",
    ]


def test_compass_snapshot_for_the_map():
    module, _, clock = make_module()
    clock.now = 5.0
    module.feed(telemetry(hdg=27000, door=0, lock=1, err=0))
    compass = module.compass()
    assert compass["ok"] is True and compass["heading"] == 270.0
    assert compass["lockOpen"] is True and compass["doorOpen"] is False
    assert compass["errors"] == [] and compass["port"] == "/dev/ttyACM2"


def test_module_without_frames_reports_not_ok_in_compass():
    module, _, _ = make_module()
    compass = module.compass()
    assert compass["ok"] is False and compass["heading"] is None


def test_port_comes_only_from_the_environment():
    assert sm.sensor_port({}) is None
    assert sm.sensor_port({"RC_SENSOR_PORT": "  "}) is None
    assert sm.sensor_port({"RC_SENSOR_PORT": "/dev/ttyACM3"}) == "/dev/ttyACM3"
    assert sm.open_sensor_module({}) is None
    module = sm.open_sensor_module({"RC_SENSOR_PORT": "/dev/ttyACM3"})
    assert module is not None and module.port == "/dev/ttyACM3"


def test_reader_thread_reconnects_after_usb_failure():
    port = FakeSerial()
    port.incoming.append((telemetry(hdg=100) + "\r\n").encode("ascii"))
    module = sm.SensorModule("/dev/ttyACM1", serial_factory=lambda *a, **k: port,
                             clock=FakeClock())
    module.start()
    for _ in range(200):
        if module.state()["heading"] is not None:
            break
        import time as _time
        _time.sleep(0.01)
    assert module.state()["heading"] == 1.0
    module.stop()
    assert port.closed is True


# ============================== бортовой экран ============================== #
@pytest.fixture
def kiosk_app(config: Config):
    application = create_kiosk_app(config)
    application.config.update(TESTING=True)
    yield application
    module = application.extensions.get("rc_sensor_module")
    if module is not None:
        module.stop()


class FakeVault:
    def __init__(self, pin="1234"):
        self.pin = pin
        self.failures = 0

    def verify(self, pin):
        if str(pin) == self.pin:
            return {"ok": True, "lock": {"locked": True}}
        self.failures += 1
        return {"ok": False, "reason": "wrong", "lock": {"locked": True}}


class FakeModule:
    """Модуль без порта: запоминает команды и отдаёт заданное состояние."""

    def __init__(self):
        self.calls: list[tuple] = []
        self.frame = sm.parse_telemetry(telemetry(hdg=12345, cards=1, light=1))

    def state(self):
        return {"configured": True, "ok": True, "port": "/dev/ttyACM9",
                **{key: value for key, value in self.frame.items() if key != "kind"}}

    def compass(self):
        return {"ok": True, "heading": 123.45, "port": "/dev/ttyACM9", "stale": False,
                "doorOpen": False, "lockOpen": False, "errors": []}

    def light(self, name, on):
        sm.command_light(name, on)          # валидация — как у настоящего клиента
        self.calls.append(("light", name, on))

    def lights_all(self, on):
        self.calls.append(("lights_all", on))

    def set_dim(self, level):
        sm.command_dim(level)
        self.calls.append(("dim", level))

    def lock(self, open_, seconds=None):
        sm.command_lock(open_, seconds)
        self.calls.append(("lock", open_, seconds))

    def card(self, action, uid=None):
        sm.command_card(action, uid)
        self.calls.append(("card", action, uid))

    def mag_calibration(self, action):
        sm.command_mag(action)
        self.calls.append(("mag", action))

    def stop(self):
        pass


def test_sensor_endpoint_reports_missing_port(kiosk_app):
    client = kiosk_app.test_client()
    response = client.get("/api/sensor_module")
    assert response.status_code == 200
    data = response.get_json()["data"]
    assert data["configured"] is False and data["ok"] is False
    assert "RC_SENSOR_PORT" in data["error"]


def test_sensor_endpoint_returns_heading_and_lights(kiosk_app):
    module = FakeModule()
    kiosk_app.extensions["rc_sensor_module"] = module
    data = kiosk_app.test_client().get("/api/sensor_module").get_json()["data"]
    assert data["ok"] is True and data["heading"] == 123.45
    assert data["lights"]["head"] is True and data["cards"] == 1


def test_lights_take_bool_and_reject_unknown_names(kiosk_app):
    module = FakeModule()
    kiosk_app.extensions["rc_sensor_module"] = module
    client = kiosk_app.test_client()
    assert client.post("/api/sensor_module/lights",
                       json={"light": "head", "on": True}).status_code == 200
    assert module.calls[-1] == ("light", "head", True)
    assert client.post("/api/sensor_module/lights",
                       json={"all": True, "on": False}).status_code == 200
    assert module.calls[-1] == ("lights_all", False)
    assert client.post("/api/sensor_module/lights", json={"dim": 200}).status_code == 200
    assert module.calls[-1] == ("dim", 200)
    bad = client.post("/api/sensor_module/lights", json={"light": "brake", "on": True})
    assert bad.status_code == 400 and "свет" in bad.get_json()["error"]
    assert client.post("/api/sensor_module/lights", json={"dim": 999}).status_code == 400


def test_door_lock_needs_the_case_pin(kiosk_app):
    module = FakeModule()
    kiosk_app.extensions["rc_sensor_module"] = module
    client = kiosk_app.test_client()
    # Без PIN-хранилища открыть дверь нельзя: 403, команда не уходит.
    refused = client.post("/api/sensor_module/lock", json={"open": True})
    assert refused.status_code == 403
    assert not [call for call in module.calls if call[0] == "lock"]

    kiosk_app.extensions["rc_vault"] = FakeVault("1234")
    assert client.post("/api/sensor_module/lock",
                       json={"open": True, "pin": "0000"}).status_code == 403
    ok = client.post("/api/sensor_module/lock", json={"open": True, "pin": "1234", "seconds": 3})
    assert ok.status_code == 200
    assert module.calls[-1] == ("lock", True, 3.0)
    # Закрытие свободно: замок должен уметь закрыть любой, это безопасное действие.
    assert client.post("/api/sensor_module/lock", json={"open": False}).status_code == 200
    assert module.calls[-1] == ("lock", False, None)


def test_cards_change_only_with_pin(kiosk_app):
    module = FakeModule()
    kiosk_app.extensions["rc_sensor_module"] = module
    client = kiosk_app.test_client()
    assert client.post("/api/sensor_module/cards", json={"action": "list"}).status_code == 200
    assert module.calls[-1] == ("card", "list", None)

    denied = client.post("/api/sensor_module/cards",
                         json={"action": "add", "uid": "A1B2C3D4"})
    assert denied.status_code == 403

    kiosk_app.extensions["rc_vault"] = FakeVault("1234")
    granted = client.post("/api/sensor_module/cards",
                          json={"action": "add", "uid": "a1b2c3d4", "pin": "1234"})
    assert granted.status_code == 200
    assert module.calls[-1] == ("card", "add", "a1b2c3d4")
    assert client.post("/api/sensor_module/cards",
                       json={"action": "add", "uid": "zz", "pin": "1234"}).status_code == 400


def test_mag_calibration_endpoint(kiosk_app):
    module = FakeModule()
    kiosk_app.extensions["rc_sensor_module"] = module
    client = kiosk_app.test_client()
    assert client.post("/api/sensor_module/mag", json={"action": "start"}).status_code == 200
    assert module.calls[-1] == ("mag", "start")
    assert client.post("/api/sensor_module/mag", json={"action": "spin"}).status_code == 400


def test_commands_without_module_are_refused(kiosk_app):
    client = kiosk_app.test_client()
    response = client.post("/api/sensor_module/lights", json={"light": "head", "on": True})
    assert response.status_code == 503
    assert "не подключён" in response.get_json()["error"]


def test_map_gets_the_compass_next_to_the_lidar_pose(kiosk_app):
    module = FakeModule()
    kiosk_app.extensions["rc_sensor_module"] = module
    payload = kiosk_app.test_client().get("/api/kiosk_map").get_json()
    assert payload["compass"]["heading"] == 123.45
    assert payload["compass"]["ok"] is True


def test_map_without_module_still_works(kiosk_app):
    payload = kiosk_app.test_client().get("/api/kiosk_map").get_json()
    assert "compass" in payload and payload["compass"] is None


def test_kiosk_state_carries_the_sensor_block():
    """Поле sensorModule в /api/kiosk_state — контракт для экрана."""
    source = (ROOT / "robot_control" / "kiosk_app.py").read_text(encoding="utf-8")
    assert '"sensorModule": sensor_state_payload()' in source
