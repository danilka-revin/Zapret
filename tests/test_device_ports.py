"""Ручное назначение портов: без автопоиска, с уникальностью и .env."""
from __future__ import annotations

import os
from pathlib import Path

import pytest

from robot_control import device_ports as ports

SERIAL_A = "/dev/ttyUSB0"
SERIAL_B = "/dev/ttyUSB1"
VIDEO_A = "/dev/video0"
VIDEO_B = "/dev/video2"


@pytest.fixture(autouse=True)
def clean_port_env(monkeypatch):
    keys = [ports.WHEEL_ENV_KEY, *[keys[0] for keys in ports.ROLE_ENV.values()]]
    for key in keys:
        monkeypatch.delenv(key, raising=False)
    yield
    # save_* пишут не только файл, но и os.environ (живое применение): убираем.
    for key in keys:
        os.environ.pop(key, None)


def test_wheel_ports_are_position_agnostic_and_deduplicated(monkeypatch):
    monkeypatch.setenv(ports.WHEEL_ENV_KEY, f"{SERIAL_A},{SERIAL_B},,{SERIAL_A}")
    assert ports.wheel_ports() == [SERIAL_A, SERIAL_B, "", ""]


def test_validate_rejects_wrong_kind_and_duplicate_ports(monkeypatch):
    with pytest.raises(ports.PortError):
        ports.validate("camera_FL", SERIAL_A)      # для камеры нужен видеопорт
    with pytest.raises(ports.PortError):
        ports.validate("lidar", VIDEO_A)           # для лидара нужен UART
    monkeypatch.setenv("RC_CAMERA_FRONT", VIDEO_A)
    with pytest.raises(ports.PortError):
        ports.validate("camera_FR", VIDEO_A)       # один порт — одно устройство
    with pytest.raises(ports.PortError):
        ports.validate("wheels", SERIAL_A)


def test_wheels_cannot_repeat_and_cannot_steal_a_camera(monkeypatch):
    monkeypatch.setenv("RC_CAMERA_FRONT", VIDEO_A)
    with pytest.raises(ports.PortError):
        ports.validate_wheels([SERIAL_A, SERIAL_A, "", ""])
    assert ports.validate_wheels([SERIAL_A, SERIAL_B, "", ""]) == [SERIAL_A, SERIAL_B, "", ""]


def test_save_wheels_writes_one_env_key_and_keeps_other_lines(tmp_path, monkeypatch):
    env = tmp_path / ".env"
    env.write_text("RC_SECRET_KEY=secret\nRC_LIDAR_PORT=/dev/ttyUSB9\n", encoding="utf-8")
    saved = ports.save_wheels([SERIAL_A, SERIAL_B, "", ""], env_path=env)
    assert saved == [SERIAL_A, SERIAL_B, "", ""]
    text = env.read_text(encoding="utf-8")
    assert "RC_WHEEL_PORTS=/dev/ttyUSB0,/dev/ttyUSB1" in text
    assert "RC_SECRET_KEY=secret" in text and "RC_LIDAR_PORT=/dev/ttyUSB9" in text
    assert os.environ[ports.WHEEL_ENV_KEY] == f"{SERIAL_A},{SERIAL_B}"


def test_payload_lists_wheels_and_eight_roles(monkeypatch):
    monkeypatch.setenv(ports.WHEEL_ENV_KEY, SERIAL_A)
    monkeypatch.setenv("RC_CAMERA_FRONT", VIDEO_A)
    payload = ports.payload()
    assert payload["ok"] and payload["manual"] is True
    assert payload["wheels"]["slots"] == [SERIAL_A, "", "", ""]
    assert payload["wheels"]["envKey"] == ports.WHEEL_ENV_KEY
    assert set(payload["roles"]) == set(ports.ROLE_ENV)
    assert len(payload["roles"]) == 8
    assert payload["roles"]["camera_FL"]["port"] == VIDEO_A
    assert payload["roles"]["camera_FL"]["envKey"] == "RC_CAMERA_FRONT"
    assert {"serial", "video", "usb"} <= set(payload["options"])


def test_kiosk_ports_api_shows_manual_assignments_without_auth(kiosk_client):
    response = kiosk_client.get("/api/devices/ports")
    assert response.status_code == 200
    body = response.get_json()
    assert body["ok"] and body["manual"] is True and body["canEdit"] is True
    assert body["wheels"]["slots"] == ["", "", "", ""]
    assert len(body["roles"]) == 8
    assert response.headers["Cache-Control"] == "no-store"


def test_kiosk_ports_save_requires_the_devices_header(kiosk_client):
    response = kiosk_client.post("/api/devices/ports", json={"role": "wheels", "ports": []})
    assert response.status_code == 400


def test_kiosk_ports_save_writes_wheel_ports(kiosk_client, monkeypatch):
    monkeypatch.setenv("RC_DEVICE_PORTS_FILE", str(Path(os.environ["RC_DEVICE_PORTS_FILE"])))
    response = kiosk_client.post(
        "/api/devices/ports",
        json={"role": "wheels", "ports": [SERIAL_A, SERIAL_B, "", ""]},
        headers={"X-Robot-Control": "devices"},
    )
    assert response.status_code == 200, response.get_json()
    body = response.get_json()
    assert body["ok"] and "телеметрия" in body["message"]
    assert body["ports"]["wheels"]["slots"] == [SERIAL_A, SERIAL_B, "", ""]
    env = Path(os.environ["RC_DEVICE_PORTS_FILE"])
    assert "RC_WHEEL_PORTS=/dev/ttyUSB0,/dev/ttyUSB1" in env.read_text(encoding="utf-8")


def test_kiosk_ports_save_rejects_duplicate_wheel_ports(kiosk_client):
    response = kiosk_client.post(
        "/api/devices/ports",
        json={"role": "wheels", "ports": [SERIAL_A, SERIAL_A, "", ""]},
        headers={"X-Robot-Control": "devices"},
    )
    assert response.status_code == 400
    assert "дважды" in response.get_json()["error"]


def test_kiosk_camera_assignment_writes_env_and_reports_roles(kiosk_client, monkeypatch):
    from robot_control import camera_settings
    saved = {}

    def fake_apply(settings):
        saved.update(settings)
        if settings.get("front"):
            monkeypatch.setenv("RC_CAMERA_FRONT", settings["front"])
        return {"ok": True}

    monkeypatch.setattr(camera_settings, "apply_camera_settings", fake_apply)
    response = kiosk_client.post(
        "/api/devices/ports",
        json={"role": "camera_FL", "port": VIDEO_A},
        headers={"X-Robot-Control": "devices"},
    )
    assert response.status_code == 200, response.get_json()
    assert saved["front"] == VIDEO_A
    assert response.get_json()["ports"]["roles"]["camera_FL"]["port"] == VIDEO_A


def test_device_page_offers_manual_ports_and_no_camera_modal(kiosk_client):
    page = kiosk_client.get("/devices").get_data(as_text=True)
    assert 'id="ports-panel"' in page and 'id="wheel-slots"' in page
    assert "Автопоиск портов отключён" in page
    screen = kiosk_client.get("/kiosk").get_data(as_text=True)
    assert "cam-settings-modal" not in screen
    assert "/devices" in screen  # ссылка на единственное место настройки


# ------------------- список портов: реальные USB-порты, без дублей ------------------- #
def node(path, kind, aliases=(), index=""):
    return {"path": path, "kind": kind, "aliases": list(aliases), "interface": "1.0",
            "index": index, "name": ""}


def usb(topology, vidpid, nodes=(), product="USB device", manufacturer="Maker", hub=False):
    return {"topology": topology, "vidPid": vidpid, "serial": "", "manufacturer": manufacturer,
            "product": product, "class": "09" if hub else "00", "interfaces": [],
            "busPath": "", "nodes": list(nodes), "connection": "через хаб · 1-2",
            "parent": "1-2", "isHub": hub}


class FakeRegistry:
    """Реестр устройств, у которого есть только снимок USB-топологии."""

    def __init__(self, devices):
        self.devices = devices
        self.calls = 0

    def usb(self, refresh=False):
        self.calls += 1
        return {"available": True, "devices": self.devices, "collectedAt": 1000}


def no_pyserial(monkeypatch):
    """Убрать pyserial из списка: проверяем именно топологию, а не /dev машины."""
    import serial.tools.list_ports as list_ports
    monkeypatch.setattr(list_ports, "comports", lambda: [])


def two_ports_topology():
    return FakeRegistry([
        usb("1-2.1", "1a86:7523", [node("/dev/ttyUSB0", "serial", [
            "/dev/serial/by-id/usb-1a86_A-if00-port0",
            "/dev/serial/by-path/pci-0:1.2-port0"])], product="CH340"),
        usb("1-2.2", "1a86:7523", [node("/dev/ttyUSB1", "serial", [
            "/dev/serial/by-id/usb-1a86_B-if00-port0"])], product="CH340"),
        usb("1-4", "046d:0843", [node("/dev/video0", "video",
                                     ["/dev/v4l/by-id/usb-Cam"], index="0")], product="Camera"),
        usb("1-6", "045e:02ae", product="Kinect"),
    ])


def test_serial_list_has_one_row_per_physical_port(monkeypatch):
    """by-id/by-path — псевдонимы одного порта, а не отдельные строки списка."""
    no_pyserial(monkeypatch)
    monkeypatch.setenv(ports.WHEEL_ENV_KEY, "/dev/ttyUSB0,/dev/serial/by-path/pci-0:1.2-port0")
    options = ports._serial_options(ports._topology(two_ports_topology()))
    assert [option["device"] for option in options] == ["/dev/ttyUSB0", "/dev/ttyUSB1"]
    first = options[0]
    assert first["port"] == "/dev/serial/by-id/usb-1a86_A-if00-port0"  # путь для .env
    assert first["usbPort"] == "1-2.1"                                 # реальный USB-порт
    assert first["byPath"] == "/dev/serial/by-path/pci-0:1.2-port0"
    assert first["name"] == "Maker · CH340"
    # Назначенный by-path не добавил вторую строку того же порта.
    assert len({option["port"] for option in options}) == 2


def test_offboard_uart_ports_are_not_offered(monkeypatch):
    """/dev/ttyS0…31 живут на плате и к USB не относятся — их в списке нет."""
    import types
    import serial.tools.list_ports as list_ports

    def port(device, hwid, description="n/a", manufacturer=None, product=None,
             location=""):
        return types.SimpleNamespace(device=device, hwid=hwid, description=description,
                                     manufacturer=manufacturer, product=product,
                                     location=location, interface=None)

    def fake_comports():
        return [
            port("/dev/ttyS0", ""),                       # встроенный UART платы
            port("/dev/ttyS4", "PNP0501"),
            port("/dev/ttyUSB5", "USB VID:PID=1A86:7523", description="USB Serial",
                 manufacturer="Maker", product="CH340", location="1-3:1.0"),
        ]

    monkeypatch.setattr(list_ports, "comports", fake_comports)
    options = ports._serial_options({"available": True, "devices": []})
    assert [option["device"] for option in options] == ["/dev/ttyUSB5"]
    assert options[0]["usbPort"] == "1-3"
    assert options[0]["vidPid"] == "1a86:7523"


def test_usb_only_list_does_not_repeat_devices_with_a_node(monkeypatch):
    """Колесо уже есть как tty-порт: второй раз как usb:1-2 оно только путает."""
    no_pyserial(monkeypatch)
    registry = two_ports_topology()
    usb_options = ports._usb_options(ports._topology(registry))
    assert [option["port"] for option in usb_options] == ["usb:1-6"]
    assert usb_options[0]["kinect"] is True


def test_camera_assignment_does_not_leak_into_the_serial_list(monkeypatch):
    """Видеопорт камеры раньше попадал в список последовательных портов."""
    no_pyserial(monkeypatch)
    monkeypatch.setenv("RC_CAMERA_FRONT", "/dev/video0")
    options = ports._serial_options(ports._topology(two_ports_topology()))
    assert all(option["device"].startswith("/dev/tty") for option in options)
    assert "/dev/video0" not in [option["port"] for option in options]


def test_ports_are_sorted_by_physical_usb_port(monkeypatch):
    no_pyserial(monkeypatch)
    registry = FakeRegistry([
        usb("1-10", "1a86:7523", [node("/dev/ttyUSB2", "serial")], product="CH340"),
        usb("1-9", "1a86:7523", [node("/dev/ttyUSB1", "serial")], product="CH340"),
        usb("2-1", "1a86:7523", [node("/dev/ttyUSB0", "serial")], product="CH340"),
    ])
    options = ports._serial_options(ports._topology(registry))
    assert [option["usbPort"] for option in options] == ["1-9", "1-10", "2-1"]


def test_disconnected_assignment_stays_visible_but_marked(monkeypatch):
    no_pyserial(monkeypatch)
    monkeypatch.setenv("RC_LIDAR_PORT", "/dev/ttyUSB7")
    options = ports._serial_options({"available": True, "devices": []})
    assert [option["port"] for option in options] == ["/dev/ttyUSB7"]
    assert options[0]["exists"] is False


def test_payload_reads_the_topology_once(monkeypatch):
    """Три списка — один снимок sysfs: страница обновляется каждые 2,5 с."""
    no_pyserial(monkeypatch)
    registry = two_ports_topology()
    payload = ports.payload(registry=registry)
    assert registry.calls == 1
    assert len(payload["options"]["serial"]) == 2
    assert set(payload["options"]) == {"serial", "video", "usb"}  # ключи из devices.js
