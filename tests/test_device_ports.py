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
