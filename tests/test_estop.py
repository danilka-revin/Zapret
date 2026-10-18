"""E-STOP: защёлка на диске, блокировка тяги/руления, маршруты с PIN."""

import json

import pytest

from conftest import login
from robot_control import estop
from robot_control.slam import dispatch_drive_command, dispatch_steering_command


@pytest.fixture(autouse=True)
def latch_file(tmp_path, monkeypatch):
    path = tmp_path / "estop.json"
    monkeypatch.setenv("RC_ESTOP_FILE", str(path))
    return path


def control_headers(client):
    from robot_control.slam import DRIVE_COMMAND_HEADER_VALUE, STEERING_COMMAND_HEADER

    token = client.get("/api/navigation/status").get_json()["csrfToken"]
    return {STEERING_COMMAND_HEADER: DRIVE_COMMAND_HEADER_VALUE, "X-CSRF-Token": token}


class RecordingSource:
    supports_drive = True
    name = "test"

    def __init__(self):
        self.drives = []

    def drive(self, percent):
        self.drives.append(percent)
        return {"ok": True, "percent": percent}

    def steer_module(self, module, angle):
        return {"ok": True}


def test_latch_persists_and_releases(latch_file):
    assert estop.latched() is None
    info = estop.latch("тест")
    assert info["reason"] == "тест"
    assert latch_file.exists()
    assert estop.latched()["reason"] == "тест"
    estop.release()
    assert estop.latched() is None


def test_corrupt_latch_file_counts_as_latched(latch_file):
    latch_file.write_text("{not json", encoding="utf-8")
    info = estop.latched()
    assert info is not None and info["corrupt"] is True


def test_latch_file_without_flag_is_not_latched(latch_file):
    latch_file.write_text(json.dumps({"latched": False}), encoding="utf-8")
    assert estop.latched() is None


def test_drive_refused_while_latched_but_stop_allowed(latch_file):
    src = RecordingSource()
    estop.latch()
    result, status = dispatch_drive_command(src, {"percent": 10})
    assert status == 409 and result["reason"] == "estop"
    assert src.drives == []
    result, status = dispatch_drive_command(src, {"percent": 0})
    assert status == 200 and src.drives == [0]


def test_steering_refused_while_latched():
    src = RecordingSource()
    estop.latch()
    result, status = dispatch_steering_command(src, {"module": 1, "angle": 5})
    assert status == 409 and result["reason"] == "estop"


def test_drive_works_after_release():
    src = RecordingSource()
    estop.latch()
    estop.release()
    result, status = dispatch_drive_command(src, {"percent": 10})
    assert status == 200 and src.drives == [10]


def test_estop_route_needs_header_and_csrf_and_latches(client, operator):
    login(client)
    assert client.post("/api/estop").status_code == 400
    from robot_control.slam import DRIVE_COMMAND_HEADER_VALUE, STEERING_COMMAND_HEADER
    drive_header = {STEERING_COMMAND_HEADER: DRIVE_COMMAND_HEADER_VALUE}
    assert client.post("/api/estop", headers=drive_header).json["reason"] == "csrf_required"
    resp = client.post("/api/estop", headers=control_headers(client))
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["ok"] is True and body["estop"] is True
    assert estop.latched() is not None
    state = client.get("/api/state").get_json()["data"]
    assert state["estop"] is not None


def test_release_route_requires_pin(client, operator):
    login(client)
    headers = control_headers(client)
    client.post("/api/estop", headers=headers)
    resp = client.post("/api/estop/release", headers=headers, json={"pin": "0000"})
    assert resp.status_code == 403
    assert estop.latched() is not None
    pin = client.get("/api/health").get_json()["pinDefault"]
    resp = client.post("/api/estop/release", headers=headers, json={"pin": pin})
    assert resp.status_code == 200
    assert estop.latched() is None


def test_estop_route_requires_login(client, operator):
    """Без входа пульта (8080) E-STOP не взводится даже с правильным заголовком."""
    from robot_control.slam import DRIVE_COMMAND_HEADER_VALUE, STEERING_COMMAND_HEADER
    headers = {STEERING_COMMAND_HEADER: DRIVE_COMMAND_HEADER_VALUE}
    resp = client.post("/api/estop", headers=headers)
    assert resp.status_code in (302, 401, 403)
    assert estop.latched() is None
