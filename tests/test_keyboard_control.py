"""Keyboard lease, interlocks, stale commands and real kiosk/peer transport."""
from __future__ import annotations

import copy
import threading
import time

from robot_control.keyboard import KeyboardControl, MODULES
from robot_control import slam


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
        self.commanded = 0
        self.calls = []
        self.brake_id = 0
        self.motors = {m: {"id": m, "online": True, "homed": True, "moving": False,
                           "calibrating": False, "cycling": False, "angle": 0.0,
                           "scRpm": 0.0, "scPpr": 90, "scCount": 0, "brakeId": 0, "firmware": 17,
                           "drivePercent": 0, "telemetryAgeS": 0.0} for m in MODULES}

    def read(self):
        return {"linkOk": True, "motors": copy.deepcopy(list(self.motors.values())),
                "drive": {"available": True, "canDrive": True, "active": bool(self.commanded),
                          "commandedPercent": self.commanded}}

    def drive(self, percent):
        percent = int(percent)
        self.commanded = percent
        self.calls.append(("drive", percent))
        for motor in self.motors.values():
            motor["drivePercent"] = percent
        return {"ok": True, "percent": percent}

    def brake_for_control(self):
        self.brake_id += 1
        for motor in self.motors.values():
            motor["brakeId"] = self.brake_id
        return {"ok": True, "brakeId": self.brake_id}

    def steer_module(self, module, angle):
        self.calls.append(("steer", module, angle))
        self.motors[module]["angle"] = angle
        return {"ok": True, "angle": angle}

    def pulses(self, count=3):
        for motor in self.motors.values():
            motor["scCount"] = (motor["scCount"] + count) % 2**32
            motor["scRpm"] = 12.0


def controller(source, clock):
    def drive(src, payload):
        return src.drive(payload["percent"]), 200

    def steer(src, payload):
        return src.steer_module(payload["module"], payload["angle"]), 200

    return KeyboardControl(lambda: source, drive, steer, lambda: None, clock)


def begin(control):
    result, status = control.handle({"action": "begin"})
    assert status == 200 and result["ok"]
    return result["session"]


def send(control, token, seq, action, **values):
    if action == "drive":
        values.setdefault("permit", "")
    return control.handle({"action": action, "session": token, "sequence": seq, **values})


def straight():
    return {m: 0 for m in MODULES}


def start_drive(control, token, seq, percent, angles):
    result, status = send(control, token, seq, "arm", direction=1 if percent > 0 else -1, angles=angles)
    assert status == 200 and result["ok"], result
    return send(control, token, seq + 1, "drive", percent=percent, angles=angles, permit=result["permit"])


def test_stop_sequence_rejects_late_heartbeats_and_old_sessions():
    clock, source = Clock(), Wheels()
    ctl = controller(source, clock)
    token = begin(ctl)
    assert start_drive(ctl, token, 1, 8, straight())[0]["ok"]
    assert source.commanded == 8
    assert send(ctl, token, 4, "stop")[0]["ok"]
    assert source.commanded == 0
    result, _ = send(ctl, token, 3, "drive", percent=8, angles=straight())
    assert result["reason"] == "stale_command" and source.commanded == 0
    new_token = begin(ctl)
    assert new_token != token
    result, _ = send(ctl, token, 100, "drive", percent=8, angles=straight())
    assert result["reason"] == "keyboard_session" and source.commanded == 0


def test_heartbeat_after_watchdog_window_never_restarts():
    clock, source = Clock(), Wheels()
    ctl = controller(source, clock)
    token = begin(ctl)
    assert start_drive(ctl, token, 1, 8, straight())[0]["ok"]
    clock.now += 0.61
    result, _ = send(ctl, token, 3, "drive", percent=8, angles=straight())
    assert result["reason"] == "keyboard_expired"
    assert source.commanded == 0
    result, _ = send(ctl, token, 4, "drive", percent=8, angles=straight())
    assert result["reason"] == "keyboard_session" and source.commanded == 0


def test_stale_or_wrong_angles_missing_sensor_and_estop_fail_closed():
    for damage, reason in [
        (lambda s: s.motors["FR"].update(angle=5), "steering_changed"),
        (lambda s: s.motors["RL"].update(telemetryAgeS=0.8), "module_offline"),
        (lambda s: s.motors["FL"].update(scRpm=None), "sc_invalid"),
        (lambda s: s.motors["RR"].update(homed=False), "not_homed"),
    ]:
        clock, source = Clock(), Wheels()
        ctl = controller(source, clock)
        token = begin(ctl)
        assert start_drive(ctl, token, 1, 8, straight())[0]["ok"]
        damage(source)
        clock.now += 0.2
        result, _ = send(ctl, token, 3, "drive", percent=8, angles=straight())
        assert result["reason"] == reason and source.commanded == 0


def test_each_silent_sensor_stops_all_wheels_and_cannot_be_hidden_by_others():
    for quiet in MODULES:
        clock, source = Clock(), Wheels()
        ctl = controller(source, clock)
        token = begin(ctl)
        assert start_drive(ctl, token, 1, 8, straight())[0]["ok"]
        for seq in range(3, 19):
            clock.now += 0.2
            for module in MODULES:
                if module != quiet:
                    source.motors[module]["scCount"] += 3
                    source.motors[module]["scRpm"] = 20
            result, _ = send(ctl, token, seq, "drive", percent=8, angles=straight())
            if not result["ok"]:
                break
        assert result["reason"] == "sc_stalled"
        assert source.commanded == 0


def test_counter_rollover_allowed_but_reboot_or_direction_change_stops():
    clock, source = Clock(), Wheels()
    for motor in source.motors.values():
        motor["scCount"] = 2**32 - 2
    ctl = controller(source, clock)
    token = begin(ctl)
    assert start_drive(ctl, token, 1, -8, straight())[0]["ok"]
    clock.now += 0.2
    source.pulses(4)
    assert send(ctl, token, 3, "drive", percent=-8, angles=straight())[0]["ok"]
    result, _ = send(ctl, token, 4, "drive", percent=8, angles=straight())
    assert result["reason"] == "module_busy" and source.commanded == 0

    token = begin(ctl)
    for motor in source.motors.values():
        motor.update(scRpm=0, scCount=100)
    assert start_drive(ctl, token, 1, 8, straight())[0]["ok"]
    source.motors["FR"]["scCount"] = 0
    clock.now += 0.2
    result, _ = send(ctl, token, 3, "drive", percent=8, angles=straight())
    assert result["reason"] == "sc_changed" and source.commanded == 0


def test_steering_requires_stop_and_coasting_wheel_must_finish():
    clock, source = Clock(), Wheels()
    ctl = controller(source, clock)
    token = begin(ctl)
    source.motors["FL"]["scRpm"] = 10
    result, _ = send(ctl, token, 1, "steer", module="FR", angle=25)
    assert result["reason"] == "wheel_moving" and source.commanded == 0
    source.motors["FL"]["scRpm"] = 0
    token = begin(ctl)
    assert send(ctl, token, 1, "steer", module="FR", angle=25)[0]["ok"]
    angles = straight() | {"FR": 25}
    assert start_drive(ctl, token, 2, 8, angles)[0]["ok"]
    result, _ = send(ctl, token, 4, "steer", module="FL", angle=25)
    assert result["reason"] == "module_busy" and source.commanded == 0


def test_manual_override_revokes_lease():
    clock, source = Clock(), Wheels()
    ctl = controller(source, clock)
    token = begin(ctl)
    assert start_drive(ctl, token, 1, 8, straight())[0]["ok"]
    ctl.interrupt()
    assert source.commanded == 0
    result, _ = send(ctl, token, 4, "drive", percent=8, angles=straight())
    assert result["reason"] == "keyboard_session" and source.commanded == 0


def test_stop_waits_for_inflight_drive_then_revokes_older_sequence():
    source, clock = Wheels(), Clock()
    entered, release = threading.Event(), threading.Event()

    def blocking_drive(src, body):
        if body["percent"]:
            entered.set()
            assert release.wait(2)
        return src.drive(body["percent"]), 200

    ctl = KeyboardControl(lambda: source, blocking_drive,
                          lambda src, body: (src.steer_module(**body), 200), lambda: None, clock)
    token = begin(ctl)
    permit = send(ctl, token, 1, "arm", direction=1, angles=straight())[0]["permit"]
    replies = []
    drive = threading.Thread(target=lambda: replies.append(send(ctl, token, 2, "drive", percent=8, angles=straight(), permit=permit)))
    stop = threading.Thread(target=lambda: replies.append(send(ctl, token, 3, "stop")))
    drive.start()
    assert entered.wait(1)
    stop.start()
    release.set()
    drive.join(2)
    stop.join(2)
    assert not drive.is_alive() and not stop.is_alive()
    assert source.calls[-1] == ("drive", 0) and source.commanded == 0
    result, _ = send(ctl, token, 2, "drive", percent=8, angles=straight())
    assert result["reason"] == "stale_command" and source.commanded == 0


def test_kiosk_manual_stop_revokes_keyboard_lease(kiosk_client):
    source = Wheels()
    kiosk_client.application.extensions["rc_slam"].source = source
    headers = {"X-Robot-Control": "drive", "X-Robot-Command-Deadline": str(time.time() + 0.5)}
    begin_reply = kiosk_client.post("/api/kiosk_keyboard", json={"action": "begin"}, headers=headers)
    token = begin_reply.json["session"]
    arm = {"action": "arm", "session": token, "sequence": 1, "direction": 1, "angles": straight()}
    permit = kiosk_client.post("/api/kiosk_keyboard", json=arm, headers=headers).json["permit"]
    drive = {"action": "drive", "session": token, "sequence": 2,
             "percent": 8, "angles": straight(), "permit": permit}
    response = kiosk_client.post("/api/kiosk_keyboard", json=drive, headers=headers)
    assert response.json["ok"], response.json
    assert kiosk_client.post("/api/kiosk_drive", json={"percent": 0}, headers=headers).json["ok"]
    drive["sequence"] = 3
    assert kiosk_client.post("/api/kiosk_keyboard", json=drive, headers=headers).json["reason"] == "keyboard_session"
    assert source.commanded == 0


def test_kiosk_wasd_route_requires_command_marker(kiosk_client):
    assert kiosk_client.post("/api/kiosk_keyboard", json={"action": "begin"}).json["reason"] == "request_header_required"


def test_first_drive_requires_recent_single_use_permit_even_if_stop_packet_is_lost():
    clock, source = Clock(), Wheels()
    ctl = controller(source, clock)
    token = begin(ctl)
    arm = send(ctl, token, 1, "arm", direction=1, angles=straight())[0]
    clock.now += 0.51
    result, _ = send(ctl, token, 2, "drive", percent=8, angles=straight(), permit=arm["permit"])
    assert result["reason"] == "keyboard_expired"
    assert source.commanded == 0


def test_arm_requires_confirmed_angles_not_just_an_idle_lease():
    clock, source = Clock(), Wheels()
    ctl = controller(source, clock)
    token = begin(ctl)
    source.motors["FL"]["angle"] = 25
    result, _ = send(ctl, token, 1, "arm", direction=1, angles=straight())
    assert result["reason"] == "steering_changed" and source.commanded == 0


def test_neither_stale_zero_nor_old_brake_ack_permits_a_new_start():
    source, clock = Wheels(), Clock()
    # Simulate an undelivered brake while all cached telemetry still says 0.
    def delayed_brake():
        source.brake_id += 1
        return {"ok": True, "brakeId": source.brake_id}

    source.brake_for_control = delayed_brake
    ctl = controller(source, clock)
    token = begin(ctl)
    # Even an ACK for an older brake arriving later is not this STOP's ACK.
    for motor in source.motors.values():
        motor["brakeId"] = 0
        assert motor["drivePercent"] == 0 and motor["scRpm"] == 0
    reply, _ = send(ctl, token, 1, "arm", direction=1, angles=straight())
    assert reply["reason"] == "brake_unconfirmed"
    assert source.commanded == 0 and not [c for c in source.calls if c[0] == "steer"]


def test_missing_brake_ack_field_fails_closed():
    source, clock = Wheels(), Clock()
    ctl = controller(source, clock)
    token = begin(ctl)
    source.motors["RL"]["brakeId"] = None
    reply, _ = send(ctl, token, 1, "arm", direction=1, angles=straight())
    assert reply["reason"] == "brake_firmware_required"
    assert source.commanded == 0


def test_expired_keyboard_request_cannot_acquire_or_restart_lease(kiosk_client):
    source = Wheels()
    kiosk_client.application.extensions["rc_slam"].source = source
    headers = {"X-Robot-Control": "drive", "X-Robot-Command-Deadline": str(time.time() - 1)}
    response = kiosk_client.post("/api/kiosk_keyboard", json={"action": "begin"}, headers=headers)
    assert response.status_code == 408 and response.json["reason"] == "command_expired"
    assert source.commanded == 0
    headers["X-Robot-Command-Deadline"] = str(time.time() + 0.5)
    token = kiosk_client.post("/api/kiosk_keyboard", json={"action": "begin"}, headers=headers).json["session"]
    headers["X-Robot-Command-Deadline"] = str(time.time() - 1)
    stale_arm = {"action": "arm", "session": token, "sequence": 1, "direction": 1, "angles": straight()}
    assert kiosk_client.post("/api/kiosk_keyboard", json=stale_arm, headers=headers).status_code == 408
    headers["X-Robot-Command-Deadline"] = str(time.time() + 0.5)
    assert kiosk_client.post("/api/kiosk_keyboard", json=stale_arm, headers=headers).json["reason"] == "keyboard_session"
    assert source.commanded == 0


def test_keyboard_plans_in_javascript():
    import shutil
    import subprocess
    from pathlib import Path
    import pytest
    if not shutil.which('node'):
        pytest.skip('node needed to test the browser planner')
    result = subprocess.run([shutil.which('node'), str(Path(__file__).parent / 'js/keyboard_plan.js')],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr


def test_kiosk_estop_revokes_keyboard_without_restart(kiosk_client):
    source = Wheels()
    kiosk_client.application.extensions['rc_slam'].source = source
    headers = {'X-Robot-Control': 'drive', 'X-Robot-Command-Deadline': str(time.time() + 0.5)}
    token = kiosk_client.post('/api/kiosk_keyboard', json={'action': 'begin'}, headers=headers).json['session']
    arm = {'action': 'arm', 'session': token, 'sequence': 1, 'direction': 1, 'angles': straight()}
    permit = kiosk_client.post('/api/kiosk_keyboard', json=arm, headers=headers).json['permit']
    drive = {'action': 'drive', 'session': token, 'sequence': 2,
             'percent': 8, 'angles': straight(), 'permit': permit}
    assert kiosk_client.post('/api/kiosk_keyboard', json=drive, headers=headers).json['ok']
    assert source.commanded == 8
    assert kiosk_client.post('/api/estop', json={}, headers=headers).json['ok']
    assert source.commanded == 0
    drive['sequence'] = 3
    assert kiosk_client.post('/api/kiosk_keyboard', json=drive, headers=headers).json['reason'] == 'keyboard_session'
    assert source.commanded == 0
