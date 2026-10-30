"""Neutral lease, four-board interlocks and STOP priority (not physical tests)."""
from __future__ import annotations

import copy
import time

import pytest

from robot_control.neutral import MODULES, NeutralControl


class Clock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now


class Wheels:
    name = "master"
    supports_drive = supports_individual_steer = True
    MAX_STEER_DEG = 100

    def __init__(self):
        self.inhibited = False
        self.brake_id = 100
        self.commands = []
        self.motors = {m: {"id": m, "online": True, "firmware": 18, "neutralId": 0,
                           "neutralLocked": False, "brakeId": 0, "scRpm": 0.0, "telemetryAgeS": 0.0,
                           "drivePercent": 0, "moving": False, "cycling": False,
                           "calibrating": False, "homed": True, "angle": 0.0}
                       for m in MODULES}
        self.active = False

    def read(self):
        return {"linkOk": True, "motors": copy.deepcopy(list(self.motors.values())),
                "drive": {"available": True, "active": self.active, "canDrive": True,
                          "commandedPercent": 0, "maneuver": {"active": False}}}

    def reserve_neutral(self):
        self.inhibited = True
        return self.brake_for_control()

    def brake_for_control(self):
        self.brake_id += 1
        self.commands.append(("brake", self.brake_id))
        return {"ok": True, "brakeId": self.brake_id}

    def ack_brake(self, *modules):
        for m in modules or MODULES:
            self.motors[m].update(brakeId=self.brake_id, neutralId=0, neutralLocked=False, drivePercent=0)

    def neutral_heartbeat(self, brake_id, deadline):
        self.commands.append(("neutral", brake_id))
        return {"ok": True}

    def ack_neutral(self, *modules):
        for m in modules or MODULES:
            self.motors[m].update(neutralId=self.brake_id, neutralLocked=True, brakeId=0)

    def _brake_all(self):
        self.commands.append(("stop",))

    def finish_neutral(self):
        self.inhibited = False

    def drive(self, percent):
        if percent and self.inhibited:
            return {"ok": False, "reason": "neutral_locked"}
        self.commands.append(("drive", percent))
        return {"ok": True}

    def steer_module(self, module, angle):
        if self.inhibited:
            return {"ok": False, "reason": "neutral_locked"}
        self.commands.append(("steer", module, angle))
        return {"ok": True}


def make_control(source=None):
    source, clock = source or Wheels(), Clock()
    return source, clock, NeutralControl(lambda: source, lambda: None, lambda: False, clock)


def engage(source, control):
    result, status = control.begin()
    assert status == 200 and source.inhibited
    token = result["session"]
    # Neither old/partial STOP nor old neutral reports release the brakes.
    source.ack_brake("FL", "FR", "RL")
    assert control.heartbeat({"action": "heartbeat", "session": token, "sequence": 1})[0]["phase"] == "braking"
    assert all(cmd[0] != "neutral" for cmd in source.commands)
    source.ack_brake("RR")
    assert control.heartbeat({"action": "heartbeat", "session": token, "sequence": 2})[0]["phase"] == "engaging"
    source.ack_neutral("FL", "FR", "RL")
    assert control.heartbeat({"action": "heartbeat", "session": token, "sequence": 3})[0]["phase"] == "engaging"
    assert not control.status()["active"]
    source.ack_neutral("RR")
    assert control.heartbeat({"action": "heartbeat", "session": token, "sequence": 4})[0]["phase"] == "neutral"
    assert control.status()["active"]
    return token


def test_neutral_confirms_all_four_and_allows_manual_rotation_until_restored():
    source, clock, control = make_control()
    token = engage(source, control)
    for motor in source.motors.values():
        motor["scRpm"] = 40.0  # hand rolling is allowed after engagement
    clock.now += 0.2
    assert control.heartbeat({"action": "heartbeat", "session": token, "sequence": 5})[0]["ok"]
    assert source.drive(20)["reason"] == "neutral_locked"
    assert source.steer_module("FL", 20)["reason"] == "neutral_locked"
    assert control.restore()[0]["phase"] == "restoring"
    source.ack_brake("FL", "FR", "RL")
    assert control.status()["locked"] and source.inhibited
    source.ack_brake("RR")
    assert control.status() == {"phase": "idle", "locked": False, "active": False, "reason": ""}
    assert not source.inhibited
    assert control.heartbeat({"action": "heartbeat", "session": token, "sequence": 6})[0]["reason"] == "neutral_session"


def test_timeout_stop_and_stale_heartbeat_cannot_restart():
    source, clock, control = make_control()
    token = engage(source, control)
    count = len(source.commands)
    clock.now += 0.61
    result, status = control.heartbeat({"action": "heartbeat", "session": token, "sequence": 5})
    assert status == 409 and result["reason"] == "neutral_session"
    assert source.commands[count:] == [("stop",)]
    assert control.status()["phase"] == "fault" and source.inhibited
    assert control.restore()[0]["phase"] == "restoring"
    assert control.status()["locked"]
    source.ack_brake()
    assert not control.status()["locked"] and not source.inhibited
    assert control.heartbeat({"action": "heartbeat", "session": token, "sequence": 999})[1] == 409


def test_lost_or_reset_wheel_brakes_all_and_keeps_interlock():
    source, _, control = make_control()
    token = engage(source, control)
    source.motors["RR"]["neutralId"] = 0
    reply, status = control.heartbeat({"action": "heartbeat", "session": token, "sequence": 5})
    assert status == 409 and reply["reason"] == "neutral_unconfirmed"
    assert source.commands[-1] == ("stop",) and source.inhibited
    # Restoration cannot report success when even one acknowledgement is old.
    control.restore()
    source.ack_brake("FL", "FR", "RL")
    assert control.status()["locked"]


@pytest.mark.parametrize("change,reason", [
    (lambda s: s.motors["FL"].update(firmware=17, neutralId=None), "neutral_firmware_required"),
    (lambda s: s.motors["FR"].update(online=False), "module_offline"),
    (lambda s: s.motors["RL"].update(telemetryAgeS=0.7), "module_offline"),
    (lambda s: s.motors["RR"].update(scRpm=None), "sc_invalid"),
    (lambda s: s.motors["FL"].update(scRpm=1.1), "wheel_moving"),
    (lambda s: s.motors["FR"].update(moving=True), "module_busy"),
    (lambda s: s.motors["RL"].update(drivePercent=8), "module_busy"),
    (lambda s: setattr(s, "active", True), "module_busy"),
])
def test_begin_refuses_missing_firmware_connection_or_running_task(change, reason):
    source, _, control = make_control()
    change(source)
    result, status = control.begin()
    assert status == 409 and result["reason"] == reason
    assert not source.inhibited and source.commands == []


def test_kiosk_blocks_motion_during_neutral_but_always_accepts_stop(kiosk_client):
    source = Wheels()
    kiosk_client.application.extensions["rc_slam"].source = source
    headers = {"X-Robot-Control": "drive", "X-Robot-Command-Deadline": str(time.time() + 0.5)}
    reply = kiosk_client.post("/api/kiosk_neutral", json={"action": "begin"}, headers=headers)
    assert reply.status_code == 200
    token = reply.json["session"]
    source.ack_brake()
    beat = {"action": "heartbeat", "session": token, "sequence": 1}
    assert kiosk_client.post("/api/kiosk_neutral", json=beat, headers=headers).json["phase"] == "engaging"
    source.ack_neutral()
    beat["sequence"] = 2
    assert kiosk_client.post("/api/kiosk_neutral", json=beat, headers=headers).json["phase"] == "neutral"
    for path, body, marker in [
        ("/api/kiosk_drive", {"value": 40}, "drive"),
        ("/api/kiosk_drive", {"value": 40, "percent": 0}, "drive"),
        ("/api/kiosk_steer", {"module": "FL", "angle": 20}, "steering"),
        ("/api/kiosk_calibrate", {"module": "ALL"}, "steering"),
        ("/api/kiosk_keyboard", {"action": "begin"}, "drive"),
        ("/api/kiosk_maneuver", {"action": "start", "name": "forward"}, "steering"),
        ("/api/navigation/start", {}, "navigation"),
    ]:
        result = kiosk_client.post(path, json=body, headers={**headers, "X-Robot-Control": marker})
        assert result.status_code == 409 and result.json["reason"] == "neutral_locked", (path, result.json)
    stop = kiosk_client.post("/api/kiosk_drive", json={"percent": 0}, headers=headers)
    assert stop.status_code == 200 and source.commands[-2:] == [("stop",), ("drive", 0)]
    beat["sequence"] = 3
    assert kiosk_client.post("/api/kiosk_neutral", json=beat, headers=headers).status_code == 409
    assert source.inhibited
    assert kiosk_client.post("/api/kiosk_neutral", json={"action": "restore"}, headers=headers).status_code == 200
    source.ack_brake()
    assert not kiosk_client.application.extensions["rc_neutral"].status()["locked"]


def test_expired_or_out_of_order_requests_do_not_refresh_neutral():
    source, clock, control = make_control()
    token = engage(source, control)
    before = list(source.commands)
    result, status = control.heartbeat({"action": "heartbeat", "session": token, "sequence": 4})
    assert status == 409 and result["reason"] == "stale_command" and source.commands == before
    clock.now += 0.2
    original = source.read

    def slow_read():
        clock.now += 0.5
        return original()

    source.read = slow_read
    result, status = control.heartbeat({"action": "heartbeat", "session": token, "sequence": 5}, deadline=clock.now + 0.3)
    assert status == 408 and result["reason"] == "command_expired"
    assert source.commands[-1] == ("stop",) and source.inhibited
    assert control.status()["locked"]


def test_navigation_and_estop_prevent_neutral():
    source, clock = Wheels(), Clock()
    for estop, navigation, reason in [(lambda: {"reason": "test"}, lambda: False, "estop"),
                                      (lambda: None, lambda: True, "navigation_active")]:
        control = NeutralControl(lambda: source, estop, navigation, clock)
        result, status = control.begin()
        assert status == 409 and result["reason"] == reason
        assert not source.inhibited and not source.commands


def test_queued_neutral_heartbeat_expires_before_serial_write():
    from robot_control import slam
    backend = slam.load_backend()
    source = object.__new__(backend.MasterSource)
    link = backend._WheelLink("/tmp/test-wheel")
    link.commands.put(backend._ExpiringCommand(b"n 123\n", time.monotonic() - 1))
    link.commands.put(b"b\n")

    class Port:
        def __init__(self):
            self.writes = []

        def write(self, value):
            self.writes.append(value)
            return len(value)

        def flush(self):
            pass

    port = Port()
    source._link_write(link, port)
    assert port.writes == [b"b\n"] and link.commands.empty()
    assert link.commands.unfinished_tasks == 0


def test_server_restart_detects_orphaned_neutral_and_requires_fresh_brake():
    source, _, control = make_control()
    source.motors["FL"]["neutralId"] = 123
    source.observed_neutral = lambda: any(m["neutralId"] for m in source.motors.values())
    source.inhibit_neutral_fault = lambda: (setattr(source, "inhibited", True), source._brake_all())
    status = control.status()
    assert status["phase"] == "fault" and status["locked"] and source.inhibited
    assert source.commands == [("stop",)]
    assert control.restore()[0]["phase"] == "restoring"
    source.ack_brake()
    assert control.status()["phase"] == "idle" and not source.inhibited


def test_neutral_begin_waits_for_motion_request_already_in_flight(kiosk_client):
    import threading

    source = Wheels()
    entered, release = threading.Event(), threading.Event()

    def slow_steer(module, angle):
        entered.set()
        assert release.wait(timeout=3)
        source.motors[module]["moving"] = True
        return {"ok": True, "angle": angle}

    source.steer_module = slow_steer
    app = kiosk_client.application
    app.extensions["rc_slam"].source = source
    responses = {}

    def steer():
        with app.test_client() as client:
            responses["steer"] = client.post("/api/kiosk_steer", json={"module": "FL", "angle": 20},
                                              headers={"X-Robot-Control": "steering"})

    def begin():
        with app.test_client() as client:
            responses["begin"] = client.post("/api/kiosk_neutral", json={"action": "begin"},
                                              headers={"X-Robot-Control": "drive",
                                                       "X-Robot-Command-Deadline": str(time.time() + 0.5)})

    t1, t2 = threading.Thread(target=steer), threading.Thread(target=begin)
    t1.start()
    assert entered.wait(timeout=2)
    t2.start()
    assert t2.is_alive()  # reservation cannot overtake a request already executing
    release.set()
    t1.join(timeout=3)
    t2.join(timeout=3)
    assert not t1.is_alive() and not t2.is_alive()
    assert responses["steer"].status_code == 200
    assert responses["begin"].json["reason"] == "module_busy"
    assert not source.inhibited and not source.commands


def test_command_arriving_during_neutral_cannot_execute_after_restore(kiosk_client):
    import threading

    source = Wheels()
    app = kiosk_client.application
    app.extensions["rc_slam"].source = source
    control = app.extensions["rc_neutral"]
    engage(source, control)
    entered, release = threading.Event(), threading.Event()
    original = control.restore

    def delayed_restore():
        entered.set()
        assert release.wait(timeout=3)
        result = original()
        source.ack_brake()
        assert control.status()["phase"] == "idle"
        return result

    control.restore = delayed_restore
    responses = {}

    def restore():
        with app.test_client() as client:
            responses["restore"] = client.post("/api/kiosk_neutral", json={"action": "restore"},
                                                headers={"X-Robot-Control": "drive"})

    def steer():
        with app.test_client() as client:
            responses["steer"] = client.post("/api/kiosk_steer", json={"module": "FL", "angle": 15},
                                              headers={"X-Robot-Control": "steering"})

    t1, t2 = threading.Thread(target=restore), threading.Thread(target=steer)
    t1.start()
    assert entered.wait(timeout=2)
    t2.start()
    assert t2.is_alive()
    release.set()
    t1.join(timeout=3)
    t2.join(timeout=3)
    assert not t1.is_alive() and not t2.is_alive()
    assert responses["restore"].status_code == 200
    assert responses["steer"].status_code == 409
    assert responses["steer"].json["reason"] == "neutral_locked"
    assert not any(call[0] == "steer" for call in source.commands)


def test_global_stop_sends_brake_before_inflight_motion_request_finishes(kiosk_client):
    import threading

    source = Wheels()
    entered, release, braked = threading.Event(), threading.Event(), threading.Event()

    def slow_steer(module, angle):
        entered.set()
        assert release.wait(timeout=3)
        return {"ok": True, "angle": angle}

    original_brake = source._brake_all

    def early_brake():
        original_brake()
        braked.set()

    source.steer_module, source._brake_all = slow_steer, early_brake
    app = kiosk_client.application
    app.extensions["rc_slam"].source = source

    def steer():
        with app.test_client() as client:
            client.post("/api/kiosk_steer", json={"module": "FL", "angle": 20},
                        headers={"X-Robot-Control": "steering"})

    def stop():
        with app.test_client() as client:
            client.post("/api/kiosk_drive", json={"percent": 0}, headers={"X-Robot-Control": "drive"})

    t1, t2 = threading.Thread(target=steer), threading.Thread(target=stop)
    t1.start()
    assert entered.wait(timeout=2)
    t2.start()
    assert braked.wait(timeout=1), "STOP must brake before the slow motion request returns"
    assert t1.is_alive()
    release.set()
    t1.join(timeout=3)
    t2.join(timeout=3)
    assert not t1.is_alive() and not t2.is_alive()
    assert source.commands[-1] == ("drive", 0)


def test_v19_neutral_refuses_recent_or_unknown_sc_even_if_rpm_is_zero():
    source, _, control = make_control()
    for motor in source.motors.values():
        motor.update(firmware=19, scRecent=False)
    source.motors["FL"]["scRecent"] = True
    result, status = control.begin()
    assert status == 409 and result["reason"] == "wheel_moving" and not source.inhibited
    source.motors["FL"]["scRecent"] = None
    result, status = control.begin()
    assert status == 409 and result["reason"] == "sc_invalid" and not source.inhibited
    source.motors["FL"]["scRecent"] = False
    assert control.begin()[0]["ok"] and source.inhibited
