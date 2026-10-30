"""Safety checks for the real-robot diagnostic (no hardware involved)."""
from __future__ import annotations

import copy

import pytest

from scripts import measure_wheel_latency as cli


def idle_state():
    return {
        "linkOk": True, "estop": None,
        "neutral": {"phase": "idle", "locked": False},
        "drive": {"available": True, "active": False, "commandedPercent": 0,
                  "commandedModules": {}, "maneuver": {"active": False}},
        "motors": [{"id": module, "online": True, "telemetryAgeS": 0.05, "firmware": 21,
                    "brakeId": 0, "drivePercent": 0, "driveServoValue": 0,
                    "moving": False, "calibrating": False, "cycling": False,
                    "scRpm": 0.0, "scRecent": False, "neutralId": 0, "neutralLocked": False}
                   for module in cli.MODULES],
    }


def test_cli_never_contacts_robot_without_explicit_stop_confirmation(monkeypatch):
    monkeypatch.setattr(cli, "request_json", lambda *_args, **_kwargs: pytest.fail("network must not be used"))
    with pytest.raises(SystemExit):
        cli.main([])


def test_preflight_rejects_motion_neutral_lost_or_unknown_state():
    assert set(cli.validate_state(idle_state())) == set(cli.MODULES)
    for field, bad in (("online", False), ("telemetryAgeS", 0.75), ("firmware", 16),
                       ("brakeId", None), ("drivePercent", 1), ("driveServoValue", 2),
                       ("moving", True), ("calibrating", True), ("cycling", True),
                       ("scRpm", None), ("scRpm", 2), ("scRecent", None), ("scRecent", True),
                       ("neutralId", 1), ("neutralLocked", True)):
        state = idle_state()
        state["motors"][2][field] = bad
        with pytest.raises(cli.MeasurementError, match="RL"):
            cli.validate_state(state)
    for field, value in (("linkOk", False), ("estop", {"latched": True}),
                         ("neutral", {"phase": "neutral", "locked": True}),
                         ("drive", {"available": True, "active": True})):
        state = idle_state()
        state[field] = value
        with pytest.raises(cli.MeasurementError):
            cli.validate_state(state)


def test_diagnostic_sends_only_begin_stop_and_end_and_uses_each_new_ack(monkeypatch):
    state, requests = idle_state(), []
    current_id = 0

    def fake_request(_url, path, body=None):
        nonlocal current_id
        requests.append((path, copy.deepcopy(body)))
        if path == "/api/kiosk_wheel_state":
            return {"ok": True, "data": copy.deepcopy(state)}
        if path == "/api/navigation/status":
            return {"ok": True, "phase": "idle"}
        if body["action"] == "begin":
            return {"ok": True, "session": "test-session"}
        if body["action"] == "stop":
            current_id += 1
            for motor in state["motors"]:
                motor["brakeId"] = current_id
            return {"ok": True, "brakeId": current_id}
        assert body["action"] == "end"
        return {"ok": True}

    monkeypatch.setattr(cli, "request_json", fake_request)
    monkeypatch.setattr(cli.time, "sleep", lambda _delay: None)
    result = cli.measure("http://robot", 3)
    assert all(len(result[m]) == 3 for m in (*cli.MODULES, "all"))
    commands = [body for _, body in requests if body is not None]
    assert [body["action"] for body in commands] == ["begin", "stop", "stop", "stop", "end"]
    assert [body.get("sequence") for body in commands[1:]] == [1, 2, 3, 4]
    assert all("percent" not in body and "angles" not in body for body in commands)


def test_diagnostic_ends_session_if_snapshot_becomes_unsafe(monkeypatch):
    state, actions = idle_state(), []

    def fake_request(_url, path, body=None):
        if path == "/api/kiosk_wheel_state":
            return {"ok": True, "data": copy.deepcopy(state)}
        if path == "/api/navigation/status":
            return {"ok": True, "phase": "idle"}
        actions.append(body["action"])
        if body["action"] == "begin":
            return {"ok": True, "session": "test-session"}
        if body["action"] == "stop":
            state["motors"][0]["online"] = False
            return {"ok": True, "brakeId": 42}
        return {"ok": True}

    monkeypatch.setattr(cli, "request_json", fake_request)
    with pytest.raises(cli.MeasurementError):
        cli.measure("http://robot", 1)
    assert actions == ["begin", "stop", "end"]
