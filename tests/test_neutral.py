"""Removed freewheel and safe recovery of legacy board state (no hardware)."""
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
        # fw18 represents an already-flashed legacy board, not new fw25.
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

    def observed_neutral(self):
        return any(m["firmware"] >= 18 and (m["neutralId"] != 0 or m["neutralLocked"] is not False)
                   for m in self.motors.values())

    def inhibit_neutral_fault(self):
        self.inhibited = True
        self._brake_all()

    def brake_for_control(self):
        self.brake_id += 1
        self.commands.append(("brake", self.brake_id))
        return {"ok": True, "brakeId": self.brake_id}

    def ack_brake(self, *modules):
        for m in modules or MODULES:
            self.motors[m].update(brakeId=self.brake_id, neutralId=0, neutralLocked=False, drivePercent=0)

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


def make_control(source=None, interrupt_keyboard=None):
    source, clock = source or Wheels(), Clock()
    return source, clock, NeutralControl(lambda: source, clock, interrupt_keyboard)


def orphan(source, control):
    source.motors["FL"].update(neutralId=123, neutralLocked=True)
    assert control.status()["phase"] == "fault"
    assert source.inhibited


@pytest.mark.parametrize("legacy_state", [
    {"neutralId": 123, "neutralLocked": True},
    {"neutralId": 0, "neutralLocked": True},  # watchdog already applied the brake
    {"neutralId": 0, "neutralLocked": None},  # unknown is not a confirmation
])
def test_legacy_state_brakes_and_interrupts_keyboard(legacy_state):
    interrupted = []
    source, _, control = make_control(interrupt_keyboard=lambda: interrupted.append(True))
    source.motors["FL"].update(**legacy_state)
    status = control.status()
    assert status == {"phase": "fault", "locked": True, "active": False, "reason": "neutral_unconfirmed"}
    assert source.inhibited and source.commands == [("stop",)] and interrupted == [True]
    assert control.ticket() == (1, True)
    assert control.status()["locked"] and interrupted == [True]  # not repeated on every poll


def test_restore_before_first_poll_detects_orphan_and_requires_four_new_acks():
    source, _, control = make_control()
    source.motors["FL"].update(neutralId=123, neutralLocked=True)
    assert control.restore() == ({"ok": True, "phase": "restoring"}, 200)
    assert source.inhibited and source.commands == [("stop",), ("brake", 101)]
    # Neither old acknowledgements, nor three new ones, nor an unknown lock
    # can authorize motion. Only fresh telemetry for this exact id can.
    source.ack_brake("FL", "FR", "RL")
    assert control.status()["locked"] and source.inhibited
    source.ack_brake("RR")
    source.motors["RR"]["neutralLocked"] = None
    assert control.status()["locked"] and source.inhibited
    source.motors["RR"]["neutralLocked"] = False
    assert control.status() == {"phase": "idle", "locked": False, "active": False, "reason": ""}
    assert control.ticket() == (1, False) and not source.inhibited


def test_wrong_brake_id_times_out_and_retry_needs_a_different_ack():
    source, clock, control = make_control()
    orphan(source, control)
    control.restore()
    source.ack_brake()
    source.motors["RR"]["brakeId"] -= 1
    assert control.status()["locked"]
    clock.now += 2.1
    assert control.status()["reason"] == "brake_unconfirmed" and source.inhibited
    assert control.restore()[0]["phase"] == "restoring"
    old_id = source.brake_id - 1
    for motor in source.motors.values():
        motor["brakeId"] = old_id
    assert control.status()["locked"]
    source.ack_brake()
    assert not control.status()["locked"] and not source.inhibited


@pytest.mark.parametrize("change,reason", [
    (lambda s: s.motors["FL"].update(firmware=17), "neutral_firmware_required"),
    (lambda s: s.motors["FR"].update(online=False), "module_offline"),
    (lambda s: s.motors["RL"].update(telemetryAgeS=0.7), "module_offline"),
    (lambda s: s.motors["RR"].update(scRpm=None), "sc_invalid"),
    (lambda s: s.motors["FL"].update(firmware=19, scRecent=None), "sc_invalid"),
    (lambda s: s.motors["FR"].update(moving=True), "module_busy"),
    (lambda s: s.motors["RL"].update(drivePercent=8), "module_busy"),
    (lambda s: setattr(s, "active", True), "module_busy"),
])
def test_recovery_never_unlocks_with_missing_stale_or_busy_telemetry(change, reason):
    source, clock, control = make_control()
    orphan(source, control)
    control.restore()
    source.ack_brake()
    change(source)
    clock.now += 2.1
    status = control.status()
    assert status["phase"] == "fault" and status["locked"] and status["reason"] == reason
    assert source.inhibited


def test_stop_interrupts_pending_recovery_and_stale_ack_cannot_unlock():
    source, _, control = make_control()
    orphan(source, control)
    control.restore()
    source.ack_brake()
    control.interrupt("stopped")
    assert control.status()["phase"] == "fault" and source.inhibited
    control.restore()
    assert control.status()["locked"]  # old id was not an ACK for this brake
    source.ack_brake()
    assert not control.status()["locked"]


@pytest.mark.parametrize("payload", [
    {"action": "begin"},
    {"action": "heartbeat", "session": "old", "sequence": 9999},
    {"action": "begin", "extra": "anything"},
])
def test_old_kiosk_release_requests_are_rejected_even_without_deadline(kiosk_client, payload):
    source = Wheels()
    kiosk_client.application.extensions["rc_slam"].source = source
    response = kiosk_client.post("/api/kiosk_neutral", json=payload, headers={"X-Robot-Control": "drive"})
    assert response.status_code == 410 and response.json["reason"] == "neutral_disabled"
    assert source.commands == [] and not source.inhibited
    from robot_control import slam
    backend = slam.load_backend()
    assert not hasattr(backend.MasterSource, "reserve_neutral")
    assert not hasattr(backend.MasterSource, "neutral_heartbeat")


def test_rejected_heartbeat_still_brakes_an_observed_old_state(kiosk_client):
    source = Wheels()
    source.motors["FL"].update(neutralId=42, neutralLocked=True)
    kiosk_client.application.extensions["rc_slam"].source = source
    response = kiosk_client.post("/api/kiosk_neutral", json={"action": "heartbeat", "session": "old", "sequence": 1},
                                 headers={"X-Robot-Control": "drive"})
    assert response.status_code == 410 and source.inhibited
    assert source.commands == [("stop",)]


def test_kiosk_blocks_motion_for_legacy_state_but_always_accepts_stop(kiosk_client):
    source = Wheels()
    kiosk_client.application.extensions["rc_slam"].source = source
    control = kiosk_client.application.extensions["rc_neutral"]
    orphan(source, control)
    headers = {"X-Robot-Control": "drive", "X-Robot-Command-Deadline": str(time.time() + 0.5)}
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
    assert stop.status_code == 200 and source.commands[-1] == ("drive", 0)
    assert source.inhibited
    assert kiosk_client.post("/api/kiosk_neutral", json={"action": "restore"}, headers=headers).status_code == 200
    source.ack_brake()
    assert not control.status()["locked"]
    assert kiosk_client.post("/api/kiosk_drive", json={"percent": 8}, headers=headers).status_code == 200


def test_queued_old_neutral_heartbeat_expires_before_serial_write():
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

    port = Port()
    source._link_write(link, port)
    assert port.writes == [b"b\n"] and link.commands.empty()
    assert link.commands.unfinished_tasks == 0


def test_motion_arriving_during_recovery_cannot_execute_after_ack(kiosk_client):
    import threading

    source = Wheels()
    app = kiosk_client.application
    app.extensions["rc_slam"].source = source
    control = app.extensions["rc_neutral"]
    orphan(source, control)
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
    assert responses["steer"].status_code == 409 and responses["steer"].json["reason"] == "neutral_locked"
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
