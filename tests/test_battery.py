"""JBD protocol, all-bank reports, port isolation and public read-only APIs."""
from __future__ import annotations

import csv
import io
import os
import shutil
import subprocess
import threading
import time
from pathlib import Path

import pytest

from robot_control.battery import (
    BatteryManager, ProtocolError, extract_frames, parse_snapshot, read_request,
)


def response(reg, data, status=0, with_status=True):
    payload = bytes([status, len(data)]) + data
    checksum = (-sum(payload if with_status else payload[1:])) & 0xFFFF
    return bytes([0xDD, reg]) + payload + checksum.to_bytes(2, "big") + b"\x77"


def payloads(count=32, received=None):
    basic = bytearray(27)
    for offset, value in ((0, 5120), (2, (-1250) & 65535), (4, 2500), (6, 5000),
                          (8, 123), (12, 1), (14, 0x8000), (16, 1 << 2),
                          (23, 2981), (25, 2991)):
        basic[offset:offset + 2] = value.to_bytes(2, "big")
    basic[18:23] = bytes([0x21, 50, 3, count, 2])
    cells = b"".join((3200 + i).to_bytes(2, "big") for i in range(count if received is None else received))
    return bytes(basic), cells


class FakeSerial:
    def __init__(self, basic=None, cells=None):
        self.basic, self.cells = payloads() if basic is None else (basic, cells)
        self.writes = []
        self.pending = b""
        self.closed = False

    def reset_input_buffer(self):
        self.pending = b""

    def write(self, data):
        self.writes.append(data)
        reg = data[2]
        self.pending = b"noise" + response(reg, self.basic if reg == 3 else self.cells)

    def read(self, size):
        out, self.pending = self.pending[:3], self.pending[3:]
        return out

    def close(self):
        self.closed = True


def test_exact_read_requests_no_write_commands():
    assert read_request(3).hex() == "dda50300fffd77"
    assert read_request(4).hex() == "dda50400fffc77"
    with pytest.raises(ProtocolError):
        read_request(0xE1)


def test_frames_checksum_noise_fragmentation_and_status_variants():
    valid = response(3, b"abc")
    corrupt = bytearray(valid)
    corrupt[-2] ^= 1
    frames, tail = extract_frames(b"junk" + corrupt + valid[:5])
    assert not frames
    assert tail == valid[:5]
    frames, tail = extract_frames(tail + valid[5:] + response(4, b"", status=0x80, with_status=False))
    assert frames == [(3, 0, b"abc"), (4, 0x80, b"")]
    assert not tail


def test_parse_all_32_banks_signed_current_and_high_balance_mask():
    sample = parse_snapshot(*payloads(), now=100)
    assert sample["updatedAt"] == 100
    assert sample["amps"] == -12.5
    assert sample["volts"] == 51.2
    assert sample["watts"] == -640
    assert sample["temperatures"] == [25, 26]
    assert sample["cycles"] == 123
    assert sample["remainingAh"] == 25
    assert sample["fullAh"] == 50
    assert sample["chargeOn"] and sample["dischargeOn"]
    assert sample["faults"] == ["Перенапряжение АКБ"]
    assert len(sample["cells"]) == 32
    assert sample["cells"][-1]["number"] == 32
    assert sample["cells"][-1]["mv"] == 3231
    assert [cell["number"] for cell in sample["cells"] if cell["balancing"]] == [1, 32]
    assert sample["minMv"] == 3200
    assert sample["maxMv"] == 3231
    assert sample["deltaMv"] == 31
    assert sample["averageMv"] == 3215.5
    assert sample["complete"]


@pytest.mark.parametrize("count,received", [(16, 15), (16, 17)])
def test_missing_or_extra_banks_are_not_silently_dropped(count, received):
    sample = parse_snapshot(*payloads(count, received))
    assert not sample["complete"]
    assert len(sample["cells"]) == max(count, received)
    assert sample["receivedCount"] == received
    if received < count:
        assert sample["cells"][-1]["mv"] is None


@pytest.mark.parametrize("bad", [b"", b"\x01", b"\x01\x02\x03"])
def test_reject_truncated_cells(bad):
    with pytest.raises(ProtocolError):
        parse_snapshot(payloads()[0], bad)


def test_reject_truncated_temperature_data():
    basic, cells = payloads()
    with pytest.raises(ProtocolError):
        parse_snapshot(basic[:-1], cells)


def test_manager_poll_and_stale_snapshot():
    manager = BatteryManager("/dev/fake")
    serial = FakeSerial()
    manager.poll(serial)
    assert serial.writes == [read_request(3), read_request(4)]
    sample = manager.snapshot()
    assert sample["connected"] and sample["complete"]
    stale = manager.snapshot(now=sample["data"]["updatedAt"] + 6)
    assert not stale["connected"]
    assert "устарели" in stale["reason"]
    assert len(stale["data"]["cells"]) == 32
    sample["data"]["cells"].clear()
    assert len(manager.snapshot()["data"]["cells"]) == 32


def test_no_bms_configuration_is_not_simulation():
    manager = BatteryManager()
    manager.start()
    assert manager._thread is None
    snap = manager.snapshot()
    assert not snap["connected"] and not snap["configured"]
    assert snap["data"] is None
    assert "RC_BMS_PORT" in snap["reason"]


def test_device_error_and_timeout_do_not_publish_a_partial_sample():
    manager = BatteryManager("/dev/fake")
    class ErrorSerial(FakeSerial):
        def write(self, data):
            self.pending = response(data[2], b"", status=0x80)
    with pytest.raises(ProtocolError, match="статус 0x80"):
        manager.poll(ErrorSerial())
    assert manager.snapshot()["data"] is None
    class SilentSerial(FakeSerial):
        def write(self, data):
            self.pending = b""
        def read(self, size):
            time.sleep(.01)
            return b""
    with pytest.raises(ProtocolError, match="не ответила"):
        manager.poll(SilentSerial())
    assert not manager.snapshot()["connected"]


@pytest.fixture
def battery_app(config, monkeypatch):
    from robot_control.kiosk_app import create_kiosk_app
    monkeypatch.setenv("RC_BMS_PORT", "/dev/fake")
    app = create_kiosk_app(config)
    app.config["TESTING"] = True
    return app


def test_public_page_api_and_all_bank_csv(battery_app):
    client = battery_app.test_client()
    manager = battery_app.extensions["rc_battery"]
    assert manager._thread is None  # tests never open real UART ports
    assert client.get("/api/battery/report.csv").status_code == 503
    manager.poll(FakeSerial())
    body = client.get("/api/battery").get_json()
    assert body["ok"] and body["battery"]["connected"]
    assert len(body["battery"]["data"]["cells"]) == 32
    report = client.get("/api/battery/report.csv")
    assert report.status_code == 200
    assert report.data.startswith(b"\xef\xbb\xbf")
    rows = list(csv.reader(io.StringIO(report.data.decode("utf-8-sig")), delimiter=";"))
    banks = rows[rows.index(["Банка", "Напряжение, мВ", "Отклонение от среднего, мВ", "Балансировка", "Данные"]) + 1:]
    assert len(banks) == 32
    assert banks[0] == ["1", "3200", "-15.5", "Да", "Прочитано"]
    assert banks[-1] == ["32", "3231", "15.5", "Да", "Прочитано"]
    page = client.get("/kiosk").text
    assert 'id="swipe-battery-view"' in page
    assert 'id="battery-open"' in page
    assert 'href="api/battery/report.csv"' in page
    assert client.get("/battery.js").status_code == 200
    assert client.post("/api/battery", json={"chargeOn": False}).status_code == 405


def test_incomplete_report_includes_missing_bank(battery_app):
    manager = battery_app.extensions["rc_battery"]
    manager.poll(FakeSerial(*payloads(16, 15)))
    client = battery_app.test_client()
    snap = client.get("/api/battery").get_json()["battery"]
    assert snap["connected"] and not snap["complete"]
    assert "15 из 16" in snap["reason"]
    report = client.get("/api/battery/report.csv").data.decode("utf-8-sig")
    assert "16;;;Нет;Нет данных" in report


def test_stale_or_failed_link_cannot_be_exported(battery_app):
    manager = battery_app.extensions["rc_battery"]
    manager.poll(FakeSerial())
    manager._sample["updatedAt"] -= 10
    client = battery_app.test_client()
    assert not client.get("/api/battery").get_json()["battery"]["connected"]
    assert client.get("/api/battery/report.csv").status_code == 503
    manager._sample["updatedAt"] = time.time()
    manager._error = "USB отключён"
    assert client.get("/api/battery/report.csv").status_code == 503


def test_operator_server_relays_bms_without_opening_uart(client, monkeypatch):
    from robot_control import battery
    manager = BatteryManager("/dev/fake")
    manager.poll(FakeSerial())
    monkeypatch.setattr(battery, "kiosk_snapshot", manager.snapshot)
    assert client.get("/api/battery").get_json()["battery"]["connected"]
    assert client.get("/api/battery/report.csv").status_code == 200
    assert client.get("/battery.js").status_code == 200


def test_operator_relay_failure_is_explicit(monkeypatch):
    from robot_control.battery import kiosk_snapshot
    import urllib.request
    def fail(*args, **kwargs):
        raise OSError("offline")
    monkeypatch.setattr(urllib.request, "urlopen", fail)
    snap = kiosk_snapshot()
    assert not snap["connected"]
    assert snap["data"] is None
    assert "бортовой сервер" in snap["reason"]


def test_bms_port_and_symlinks_excluded_from_chassis_and_lidar(tmp_path, monkeypatch):
    from robot_control.slam import chassis_ports
    from robot_control.lidar import discover_ports
    port = tmp_path / "ttyUSB0"
    alias = tmp_path / "bms-by-id"
    other = tmp_path / "ttyUSB1"
    port.touch()
    other.touch()
    alias.symlink_to(port)
    monkeypatch.setenv("RC_BMS_PORT", str(alias))
    assert chassis_ports([str(port), str(alias), str(other)]) == [str(other)]
    assert discover_ports(configured=f"{port},{alias},{other}", autoprobe=False) == [str(other)]


@pytest.mark.skipif(os.name != "posix", reason="PTY integration requires POSIX")
def test_real_pyserial_exchange_over_pty():
    """Real serial transport and fragmented responses, not just parser mocks."""
    import pty
    import select
    from robot_control.lidar import try_lock_port, unlock_port
    master, slave = pty.openpty()
    manager = BatteryManager(os.ttyname(slave))
    seen = []
    stop = threading.Event()
    def emulate():
        buffer = b""
        while not stop.is_set():
            ready, _, _ = select.select([master], [], [], .05)
            if not ready:
                continue
            buffer += os.read(master, 1024)
            while len(buffer) >= 7:
                frame, buffer = buffer[:7], buffer[7:]
                seen.append(frame)
                reg = frame[2]
                data = payloads()[0 if reg == 3 else 1]
                reply = response(reg, data)
                os.write(master, reply[:5])
                time.sleep(.01)
                os.write(master, reply[5:])
    emulator = threading.Thread(target=emulate, daemon=True)
    emulator.start()
    manager.start()
    try:
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and not manager.snapshot()["connected"]:
            time.sleep(.02)
        snap = manager.snapshot()
        assert snap["connected"], snap["reason"]
        assert len(snap["data"]["cells"]) == 32
        assert seen[:2] == [read_request(3), read_request(4)]
        assert try_lock_port(manager.port) is None
    finally:
        manager.stop()
        stop.set()
        emulator.join(timeout=1)
        os.close(slave)
        os.close(master)
    fd = try_lock_port(manager.port)
    assert fd is not None
    unlock_port(fd)


def test_battery_page_javascript():
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run([node, str(root / "tests/js/battery_screen.js")], cwd=root,
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr


def run_kiosk_js(tmp_path, payload):
    import json
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    root = Path(__file__).resolve().parents[1]
    fixture = tmp_path / "battery-payload.json"
    fixture.write_text(json.dumps(payload), encoding="utf-8")
    result = subprocess.run([node, str(root / "tests/js/lidar_screen.js"), "robot", str(fixture)],
                            cwd=root, capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
    body = json.loads(result.stdout)
    assert not body["renderErrors"]
    return body


def test_fourth_page_navigation_preserves_other_pages(tmp_path):
    body = run_kiosk_js(tmp_path, {"ok": True, "data": {}, "batteryNavigation": True})
    states = body["navigation"]
    assert [s["battery"] for s in states] == [True, False, True, True, False]
    assert [s["motors"] for s in states] == [False, True, False, False, False]
    assert [s["cameras"] for s in states] == [False, False, False, False, True]
    assert [s["open"] for s in states] == [True, True, True, False, True]
    assert states[0]["title"] == "4 · Аккумулятор"


@pytest.mark.parametrize("connected", [True, False])
def test_main_kiosk_summary_uses_real_bms_without_simulation_fallback(tmp_path, connected):
    measured = parse_snapshot(*payloads())
    body = run_kiosk_js(tmp_path, {
        "ok": True, "data": {"battery": {"soc": 99, "volts": 40}},
        "bms": {"configured": True, "connected": connected, "reason": "Все банки прочитаны", "data": measured},
    })
    assert body["soc"] == (50 if connected else '—')
    assert body["volts"] == ('51.2' if connected else '—')
    assert "JBD BMS" in body["batteryState"]


def test_explicit_chassis_sources_cannot_open_bms_port(monkeypatch):
    from robot_control.slam import make_source, chassis_source
    from types import SimpleNamespace
    monkeypatch.setenv("RC_BMS_PORT", "/dev/fake-bms")
    monkeypatch.setenv("RC_SLAM_PORT", "/dev/fake-bms")
    source = make_source("master")
    assert source.name == "none"
    mod = SimpleNamespace(SerialSource=lambda ports, **kwargs: ports)
    assert chassis_source(mod, ["/dev/fake-bms", "/dev/fake-module"]) == ["/dev/fake-module"]
