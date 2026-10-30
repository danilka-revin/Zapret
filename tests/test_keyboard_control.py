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
    if action in {"drive", "live"}:
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


def test_batch_steering_is_one_sequenced_request_and_still_needs_confirmed_angles():
    source, clock = Wheels(), Clock()
    ctl = controller(source, clock)
    token = begin(ctl)
    target = {"FL": 45, "FR": 45, "RL": 45, "RR": 45}
    source.motors["RR"]["brakeId"] = 0  # one board has not acknowledged this STOP
    result, _ = send(ctl, token, 1, "steer_all", angles=target)
    assert result["reason"] == "brake_unconfirmed"
    assert not any(call[0] == "steer" for call in source.calls)

    token = begin(ctl)
    previous = len(source.calls)
    result, status = send(ctl, token, 1, "steer_all", angles=target)
    assert status == 200 and result["sent"] == list(MODULES)
    assert source.calls[previous:] == [("steer", module, 45) for module in MODULES]
    source.motors["RR"]["angle"] = 0  # server must not arm on an unconfirmed wheel
    result, _ = send(ctl, token, 2, "arm", direction=1, angles=target)
    assert result["reason"] == "steering_changed"
    assert source.commanded == 0


def test_fast_wasd_retries_coasting_only_without_steering_or_traction():
    for action in ("steer_all", "arm"):
        source, clock = Wheels(), Clock()
        ctl = controller(source, clock)
        token = begin(ctl)
        source.motors["FL"]["scRpm"] = 10
        payload = ({"angles": {m: 25 for m in MODULES}} if action == "steer_all"
                   else {"direction": 1, "angles": straight()})
        before = list(source.calls)
        result, status = send(ctl, token, 1, action, **payload)
        assert status == 409 and result == {"ok": False, "reason": "wheel_moving", "retryable": True}
        assert source.calls == before and source.commanded == 0
        # Still the same lease and a *new* sequence, with no permit issued early.
        assert send(ctl, token, 1, action, **payload)[0]["reason"] == "stale_command"
        source.motors["FL"]["scRpm"] = 0
        result, status = send(ctl, token, 2, action, **payload)
        assert status == 200 and result["ok"] and source.commanded == 0


def test_retryable_coasting_does_not_survive_manual_stop():
    source, clock = Wheels(), Clock()
    ctl = controller(source, clock)
    token = begin(ctl)
    source.motors["RR"]["scRpm"] = 15
    assert send(ctl, token, 1, "arm", direction=1, angles=straight())[0]["retryable"]
    ctl.interrupt()
    source.motors["RR"]["scRpm"] = 0
    assert send(ctl, token, 2, "arm", direction=1, angles=straight())[0]["reason"] == "keyboard_session"
    assert source.commanded == 0


def test_v19_recent_sc_prevents_start_even_when_averaged_rpm_is_zero():
    source, clock = Wheels(), Clock()
    for motor in source.motors.values():
        motor.update(firmware=19, scRecent=False)
    ctl = controller(source, clock)
    token = begin(ctl)
    source.motors["FR"]["scRecent"] = True
    result, status = send(ctl, token, 1, "arm", direction=1, angles=straight())
    assert status == 409 and result["reason"] == "wheel_moving" and result["retryable"]
    assert source.commanded == 0
    source.motors["FR"]["scRecent"] = False
    assert send(ctl, token, 2, "arm", direction=1, angles=straight())[0]["ok"]
    # Missing field from a v19 board is unknown, never a stopped wheel.
    source.motors["FR"]["scRecent"] = None
    result, _ = send(ctl, token, 3, "arm", direction=1, angles=straight())
    assert result["reason"] == "sc_invalid" and source.commanded == 0


def test_fast_wheel_state_is_current_and_omits_lidar_payload(kiosk_client):
    from robot_control import estop
    source = Wheels()
    kiosk_client.application.extensions["rc_slam"].source = source
    response = kiosk_client.get("/api/kiosk_wheel_state")
    assert response.status_code == 200 and response.headers["Cache-Control"] == "no-store"
    data = response.json["data"]
    assert set(data) == {"motors", "drive", "linkOk", "estop", "neutral", "steeringControl"}
    assert "lidar" not in data and len(data["motors"]) == 4
    assert data["estop"] is None and data["neutral"]["phase"] == "idle"
    estop.latch("fast-state-test")
    try:
        assert kiosk_client.get("/api/kiosk_wheel_state").json["data"]["estop"] is not None
    finally:
        estop.release()


class LiveWheels(Wheels):
    """Fake v20 boards, with optional missed ACK or stalled steering."""
    def __init__(self):
        super().__init__()
        self.quiet_ack = set()
        self.stuck_steering = set()
        for motor in self.motors.values():
            motor.update(firmware=20, scRecent=False, liveId=0, liveSeq=0)

    def drive(self, percent):
        result = super().drive(percent)
        if percent == 0:
            for motor in self.motors.values():
                motor.update(liveId=0, liveSeq=0, moving=False)
        return result

    def keyboard_live(self, brake_id, sequence, percent, angles, deadline):
        assert deadline > time.monotonic()
        self.commanded = percent
        self.calls.append(("live", brake_id, sequence, percent, dict(angles)))
        for module, motor in self.motors.items():
            if module not in self.quiet_ack:
                motor.update(liveId=brake_id, liveSeq=sequence, drivePercent=percent)
            if module in self.stuck_steering:
                motor["moving"] = True
            else:
                motor.update(angle=angles[module], moving=False)
        return {"ok": True, "percent": percent}


def start_live(control, token, seq=1, percent=20, angles=None):
    angles = angles or straight()
    result, status = send(control, token, seq, "arm", direction=1 if percent > 0 else -1, angles=angles)
    assert status == 200 and result["ok"], result
    return send(control, token, seq + 1, "live", percent=percent, angles=angles, permit=result["permit"])


def test_v20_live_turn_and_center_without_braking_or_reversing():
    source, clock = LiveWheels(), Clock()
    ctl = controller(source, clock)
    token = begin(ctl)
    assert start_live(ctl, token)[0]["ok"]
    before = len(source.calls)
    clock.now += 0.2
    source.pulses()
    turn = {"FL": 25, "FR": 25, "RL": -25, "RR": -25}
    assert send(ctl, token, 3, "live", percent=20, angles=turn)[0]["ok"]
    clock.now += 0.2
    source.pulses()
    assert send(ctl, token, 4, "live", percent=20, angles=straight())[0]["ok"]
    assert [call[0] for call in source.calls[before:]] == ["live", "live"]
    assert source.commanded == 20 and all(m["angle"] == 0 for m in source.motors.values())


def test_live_requires_v20_permit_and_never_accepts_a_running_reverse():
    source, clock = LiveWheels(), Clock()
    ctl = controller(source, clock)
    token = begin(ctl)
    result, _ = send(ctl, token, 1, "live", percent=20, angles=straight())
    assert result["reason"] == "keyboard_expired" and source.commanded == 0
    token = begin(ctl)
    source.motors["FR"]["firmware"] = 19
    arm, _ = send(ctl, token, 1, "arm", direction=1, angles=straight())
    result, _ = send(ctl, token, 2, "live", percent=20, angles=straight(), permit=arm["permit"])
    assert result["reason"] == "live_firmware_required" and source.commanded == 0
    source.motors["FR"]["firmware"] = 20
    token = begin(ctl)
    assert start_live(ctl, token)[0]["ok"]
    result, _ = send(ctl, token, 3, "live", percent=-20, angles=straight())
    assert result["reason"] == "module_busy" and source.commanded == 0
    assert send(ctl, token, 4, "live", percent=20, angles=straight())[0]["reason"] == "keyboard_session"


def test_live_unconfirmed_board_or_stalled_steering_brakes_everything():
    for problem in ("no_ack", "stalled"):
        source, clock = LiveWheels(), Clock()
        ctl = controller(source, clock)
        token = begin(ctl)
        assert start_live(ctl, token)[0]["ok"]
        if problem == "no_ack":
            source.quiet_ack.add("FR")
        else:
            source.stuck_steering.add("FR")
        target = {"FL": 25, "FR": 25, "RL": -25, "RR": -25}
        for sequence in range(3, 50):
            clock.now += 0.2
            source.pulses()
            result, _ = send(ctl, token, sequence, "live", percent=20, angles=target)
            if not result["ok"]:
                break
        assert result["reason"] == ("live_unconfirmed" if problem == "no_ack" else "steering_changed")
        assert source.commanded == 0


def test_live_timeout_stop_bad_telemetry_and_manual_override_cannot_restart():
    for fault, expected in [("timeout", "keyboard_expired"), ("sc", "sc_invalid"),
                            ("identity", "live_firmware_required"), ("manual", "manual_override")]:
        source, clock = LiveWheels(), Clock()
        ctl = controller(source, clock)
        token = begin(ctl)
        assert start_live(ctl, token)[0]["ok"]
        if fault == "timeout":
            clock.now += 0.61
        elif fault == "sc":
            source.motors["FL"]["scRecent"] = None
        elif fault == "identity":
            source.motors["FL"]["liveSeq"] = None
        else:
            source.commanded = 12
        result, _ = send(ctl, token, 3, "live", percent=20, angles=straight())
        assert result["reason"] == expected and source.commanded == 0
        assert send(ctl, token, 4, "live", percent=20, angles=straight())[0]["reason"] == "keyboard_session"

    source, clock = LiveWheels(), Clock()
    ctl = controller(source, clock)
    token = begin(ctl)
    assert start_live(ctl, token)[0]["ok"]
    assert send(ctl, token, 5, "stop")[0]["ok"]
    assert send(ctl, token, 4, "live", percent=20, angles=straight())[0]["reason"] == "stale_command"
    assert source.commanded == 0


def test_estop_revokes_a_live_session_on_the_real_kiosk_route(kiosk_client):
    source = LiveWheels()
    kiosk_client.application.extensions['rc_slam'].source = source
    headers = {'X-Robot-Control': 'drive', 'X-Robot-Command-Deadline': str(time.time() + 0.5)}
    token = kiosk_client.post('/api/kiosk_keyboard', json={'action': 'begin'}, headers=headers).json['session']
    arm = {'action': 'arm', 'session': token, 'sequence': 1, 'direction': 1, 'angles': straight()}
    permit = kiosk_client.post('/api/kiosk_keyboard', json=arm, headers=headers).json['permit']
    live = {'action': 'live', 'session': token, 'sequence': 2, 'percent': 20,
            'angles': straight(), 'permit': permit}
    assert kiosk_client.post('/api/kiosk_keyboard', json=live, headers=headers).json['ok']
    assert source.commanded == 20
    assert kiosk_client.post('/api/estop', json={}, headers=headers).json['ok']
    assert source.commanded == 0
    live['sequence'] = 3
    assert kiosk_client.post('/api/kiosk_keyboard', json=live, headers=headers).json['reason'] == 'keyboard_session'
    assert source.commanded == 0


def test_global_system_stop_retires_live_session_even_when_shutdown_fails(kiosk_client, monkeypatch):
    from robot_control import shutdown
    source = LiveWheels()
    kiosk_client.application.extensions['rc_slam'].source = source
    monkeypatch.setattr(shutdown, 'stop_all', lambda _path: {'ok': False, 'error': 'test'})
    headers = {'X-Robot-Control': 'drive', 'X-Robot-Command-Deadline': str(time.time() + 0.5)}
    token = kiosk_client.post('/api/kiosk_keyboard', json={'action': 'begin'}, headers=headers).json['session']
    permit = kiosk_client.post('/api/kiosk_keyboard', headers=headers, json={
        'action': 'arm', 'session': token, 'sequence': 1, 'direction': 1, 'angles': straight()}).json['permit']
    packet = {'action': 'live', 'session': token, 'sequence': 2, 'percent': 20,
              'angles': straight(), 'permit': permit}
    assert kiosk_client.post('/api/kiosk_keyboard', headers=headers, json=packet).json['ok']
    assert source.commanded == 20
    assert kiosk_client.post('/api/system/stop', headers={'X-Robot-Control': 'stop'}, json={}).status_code == 500
    assert source.commanded == 0
    packet['sequence'] = 3
    assert kiosk_client.post('/api/kiosk_keyboard', headers=headers, json=packet).json['reason'] == 'keyboard_session'


def test_global_stop_finishes_after_an_inflight_live_request(kiosk_client, monkeypatch):
    import threading
    from robot_control import shutdown

    class RacingWheels(LiveWheels):
        def __init__(self):
            super().__init__()
            self.in_live = threading.Event()
            self.release_live = threading.Event()
            self.early_brake = threading.Event()

        def keyboard_live(self, *args, **kwargs):
            self.in_live.set()
            assert self.release_live.wait(2)
            return super().keyboard_live(*args, **kwargs)

        def _brake_all(self):
            self.commanded = 0
            self.early_brake.set()

    app = kiosk_client.application
    source = RacingWheels()
    app.extensions['rc_slam'].source = source
    monkeypatch.setattr(shutdown, 'stop_all', lambda _path: {'ok': False, 'error': 'test'})
    headers = {'X-Robot-Control': 'drive', 'X-Robot-Command-Deadline': str(time.time() + 0.5)}
    token = kiosk_client.post('/api/kiosk_keyboard', json={'action': 'begin'}, headers=headers).json['session']
    permit = kiosk_client.post('/api/kiosk_keyboard', headers=headers, json={
        'action': 'arm', 'session': token, 'sequence': 1, 'direction': 1, 'angles': straight()}).json['permit']
    results = {}

    def live_request():
        with app.test_client() as client:
            results['live'] = client.post('/api/kiosk_keyboard', headers=headers, json={
                'action': 'live', 'session': token, 'sequence': 2, 'percent': 20,
                'angles': straight(), 'permit': permit}).json

    def stop_request():
        with app.test_client() as client:
            results['stop'] = client.post('/api/system/stop', headers={'X-Robot-Control': 'stop'}, json={}).status_code

    live_thread = threading.Thread(target=live_request)
    stop_thread = threading.Thread(target=stop_request)
    live_thread.start()
    try:
        assert source.in_live.wait(1)
        stop_thread.start()
        assert source.early_brake.wait(1), 'STOP must not wait for an in-flight request'
    finally:
        source.release_live.set()
        live_thread.join(2)
        if stop_thread.ident is not None:
            stop_thread.join(2)
    assert not live_thread.is_alive() and not stop_thread.is_alive()
    assert results['live']['ok'] and results['stop'] == 500
    assert source.commanded == 0, 'final STOP under the gate must follow the in-flight live packet'
    assert app.extensions['rc_keyboard']._token is None
