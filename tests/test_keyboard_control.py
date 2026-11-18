"""Keyboard lease, interlocks, stale commands and real kiosk/peer transport."""
from __future__ import annotations

import copy
import threading
import time
from pathlib import Path

import pytest

from robot_control.keyboard import KeyboardControl, MODULES
from robot_control import maneuvers, slam


class Clock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now


class Wheels:
    name = "master"
    supports_drive = supports_individual_steer = supports_individual_drive = True
    MAX_STEER_DEG = 100

    def __init__(self):
        self.commanded = 0
        self.commanded_modules = {}
        self.chassis = dict(maneuvers.DEFAULT_CHASSIS)
        self.calls = []
        self.brake_id = 0
        self.motors = {m: {"id": m, "online": True, "homed": True, "moving": False,
                           "calibrating": False, "cycling": False, "angle": 0.0,
                           "scRpm": 0.0, "scPpr": 90, "scCount": 0, "brakeId": 0, "firmware": 17,
                           "drivePercent": 0, "telemetryAgeS": 0.0} for m in MODULES}

    def read(self):
        return {"linkOk": True, "motors": copy.deepcopy(list(self.motors.values())),
                "drive": {"available": True, "canDrive": True,
                          "active": bool(self.commanded) or bool(self.commanded_modules),
                          "commandedPercent": self.commanded,
                          "commandedModules": dict(self.commanded_modules)}}

    def maneuver_config(self):
        return {"supported": True, "chassis": dict(self.chassis)}

    def drive(self, percent):
        percent = int(percent)
        self.commanded = percent
        self.commanded_modules = {}
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


def test_kiosk_keyboard_route_accepts_full_scale_and_zero(kiosk_client):
    """HTTP-путь WASD пропускает 100 % и трактует 0 % как STOP."""
    source = Wheels()
    kiosk_client.application.extensions["rc_slam"].source = source
    headers = {"X-Robot-Control": "drive",
               "X-Robot-Command-Deadline": str(time.time() + 0.5)}
    begin_reply = kiosk_client.post("/api/kiosk_keyboard",
                                    json={"action": "begin"}, headers=headers)
    assert begin_reply.json["maxPercent"] == 100
    token = begin_reply.json["session"]
    arm = {"action": "arm", "session": token, "sequence": 1,
           "direction": 1, "angles": straight()}
    permit = kiosk_client.post("/api/kiosk_keyboard", json=arm,
                               headers=headers).json["permit"]
    drive = {"action": "drive", "session": token, "sequence": 2,
             "percent": 100, "angles": straight(), "permit": permit}
    assert kiosk_client.post("/api/kiosk_keyboard", json=drive,
                             headers=headers).json["ok"]
    assert source.commanded == 100
    zero = {"action": "live", "session": token, "sequence": 3,
            "percent": 0, "angles": straight(), "permit": ""}
    assert kiosk_client.post("/api/kiosk_keyboard", json=zero,
                             headers=headers).json["ok"]
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


def test_keyboard_runtime_in_javascript():
    import shutil
    import subprocess
    from pathlib import Path
    import pytest
    if not shutil.which('node'):
        pytest.skip('node needed to test the browser state machine')
    result = subprocess.run([shutil.which('node'), str(Path(__file__).parent / 'js/keyboard_runtime.js')],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr


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
        individual = isinstance(percent, dict)
        self.commanded = 0 if individual else percent
        self.commanded_modules = dict(percent) if individual else {}
        self.calls.append(("live", brake_id, sequence, percent, dict(angles)))
        for module, motor in self.motors.items():
            value = percent[module] if individual else percent
            if module not in self.quiet_ack:
                motor.update(liveId=brake_id, liveSeq=sequence, drivePercent=value)
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


def test_keyboard_stop_preempts_an_inflight_heartbeat_without_losing_session(kiosk_client):
    """The early brake must not make the superseded heartbeat revoke the lease."""
    class RacingWheels(LiveWheels):
        def __init__(self):
            super().__init__()
            self.hold_snapshot = False
            self.in_snapshot = threading.Event()
            self.release_snapshot = threading.Event()
            self.early_brake = threading.Event()

        def read(self):
            if self.hold_snapshot:
                self.in_snapshot.set()
                assert self.release_snapshot.wait(2)
                self.hold_snapshot = False
            return super().read()

        def _brake_all(self):
            self.commanded = 0
            self.early_brake.set()

    app = kiosk_client.application
    source = RacingWheels()
    app.extensions['rc_slam'].source = source
    headers = {'X-Robot-Control': 'drive', 'X-Robot-Command-Deadline': str(time.time() + 0.5)}
    token = kiosk_client.post('/api/kiosk_keyboard', json={'action': 'begin'}, headers=headers).json['session']
    permit = kiosk_client.post('/api/kiosk_keyboard', headers=headers, json={
        'action': 'arm', 'session': token, 'sequence': 1, 'direction': 1, 'angles': straight()}).json['permit']
    packet = {'action': 'live', 'session': token, 'sequence': 2, 'percent': 20,
              'angles': straight(), 'permit': permit}
    assert kiosk_client.post('/api/kiosk_keyboard', headers=headers, json=packet).json['ok']
    source.pulses()
    source.hold_snapshot = True
    results = {}

    def live_request():
        with app.test_client() as client:
            results['live'] = client.post('/api/kiosk_keyboard', headers=headers,
                                          json={**packet, 'sequence': 3, 'permit': ''}).json

    def stop_request():
        with app.test_client() as client:
            results['stop'] = client.post('/api/kiosk_keyboard', headers=headers,
                                          json={'action': 'stop', 'session': token, 'sequence': 4}).json

    live_thread = threading.Thread(target=live_request)
    stop_thread = threading.Thread(target=stop_request)
    live_thread.start()
    try:
        assert source.in_snapshot.wait(1)
        stop_thread.start()
        assert source.early_brake.wait(1), 'keyboard STOP must brake before the in-flight heartbeat finishes'
    finally:
        source.release_snapshot.set()
        live_thread.join(2)
        if stop_thread.ident is not None:
            stop_thread.join(2)
    assert not live_thread.is_alive() and not stop_thread.is_alive()
    assert results['live']['reason'] == 'stale_command', results
    assert results['stop']['ok'] and results['stop']['brakeId'] > 0, results
    assert source.commanded == 0
    # A new permit is still required; a stale packet cannot restart traction.
    result = kiosk_client.post('/api/kiosk_keyboard', headers=headers,
                               json={**packet, 'sequence': 3, 'permit': ''}).json
    assert result['reason'] == 'stale_command' and source.commanded == 0
    arm = kiosk_client.post('/api/kiosk_keyboard', headers=headers, json={
        'action': 'arm', 'session': token, 'sequence': 5, 'direction': 1, 'angles': straight()}).json
    assert arm['reason'] == 'wheel_moving' and source.commanded == 0, arm
    for motor in source.motors.values():
        motor.update(scRpm=0, scRecent=False)
    arm = kiosk_client.post('/api/kiosk_keyboard', headers=headers, json={
        'action': 'arm', 'session': token, 'sequence': 6, 'direction': -1, 'angles': straight()}).json
    assert arm['ok'], arm
    result = kiosk_client.post('/api/kiosk_keyboard', headers=headers,
                               json={**packet, 'sequence': 7, 'percent': -20, 'permit': arm['permit']}).json
    assert result['ok'] and source.commanded == -20, result
    result = kiosk_client.post('/api/kiosk_keyboard', headers=headers,
                               json={'action': 'stop', 'session': token, 'sequence': 8}).json
    assert result['ok'] and source.commanded == 0, result


def test_stop_fence_blocks_overtaking_commands_only_for_its_own_session():
    source, clock = LiveWheels(), Clock()
    ctl = controller(source, clock)
    token = begin(ctl)
    assert start_live(ctl, token)[0]['ok']
    stop = {'action': 'stop', 'session': token, 'sequence': 4}
    for invalid in (None, {}, {**stop, 'session': 'not-the-owner'},
                    {**stop, 'sequence': True}, {**stop, 'sequence': 2},
                    {**stop, 'action': []}, {**stop, 'extra': 1}):
        assert not ctl.preempt_stop(invalid)
    assert ctl.preempt_stop(stop)
    source.drive(0)  # emulate the route's immediate best-effort brake
    # Even newer commands may not get ahead of the pending final STOP.
    for sequence in (3, 4, 5):
        result, _ = send(ctl, token, sequence, 'live', percent=20, angles=straight())
        assert result['reason'] == 'stale_command' and source.commanded == 0
    assert ctl.handle(stop)[0]['ok']
    ctl.finish_preempted_stop(stop)
    assert send(ctl, token, 3, 'live', percent=20, angles=straight())[0]['reason'] == 'stale_command'
    assert send(ctl, token, 5, 'arm', direction=1, angles=straight())[0]['ok']
    # Old fences and old-session STOPs cannot affect a newly opened session.
    new_token = begin(ctl)
    assert not ctl.preempt_stop({**stop, 'sequence': 999})
    assert send(ctl, new_token, 1, 'arm', direction=1, angles=straight())[0]['ok']


def test_stop_fence_does_not_override_estop_or_expired_new_commands(kiosk_client):
    from robot_control import estop
    source = LiveWheels()
    app = kiosk_client.application
    app.extensions['rc_slam'].source = source
    hdr = {'X-Robot-Control': 'drive', 'X-Robot-Command-Deadline': str(time.time() + 0.5)}
    token = kiosk_client.post('/api/kiosk_keyboard', json={'action': 'begin'}, headers=hdr).json['session']
    controller = app.extensions['rc_keyboard']
    stop = {'action': 'stop', 'session': token, 'sequence': 2}
    assert controller.preempt_stop(stop)
    expired = {**hdr, 'X-Robot-Command-Deadline': str(time.time() - 1)}
    old = {'action': 'arm', 'session': token, 'sequence': 1, 'direction': 1, 'angles': straight()}
    reply = kiosk_client.post('/api/kiosk_keyboard', json=old, headers=expired)
    assert reply.status_code == 409 and reply.json['reason'] == 'stale_command'
    assert kiosk_client.post('/api/kiosk_keyboard', json=stop, headers=hdr).json['ok']
    controller.finish_preempted_stop(stop)
    # The STOP never permits starting through a latched E-STOP.
    estop.latch('keyboard-stop-fence-test')
    try:
        result = kiosk_client.post('/api/kiosk_keyboard', headers=hdr, json={**old, 'sequence': 3}).json
        assert result['reason'] == 'estop' and source.commanded == 0
        assert kiosk_client.post('/api/kiosk_keyboard', headers=hdr,
                                 json={**old, 'sequence': 4}).json['reason'] == 'keyboard_session'
    finally:
        estop.release()
    token = kiosk_client.post('/api/kiosk_keyboard', json={'action': 'begin'}, headers=hdr).json['session']
    arm = {**old, 'session': token, 'sequence': 1}
    assert kiosk_client.post('/api/kiosk_keyboard', json=arm, headers=expired).status_code == 408
    assert kiosk_client.post('/api/kiosk_keyboard', json=arm, headers=hdr).json['reason'] == 'keyboard_session'
    assert source.commanded == 0


def test_stop_preempts_remaining_wheels_of_inflight_steer_all(kiosk_client):
    class RacingSteer(LiveWheels):
        def __init__(self):
            super().__init__()
            self.started = threading.Event()
            self.release = threading.Event()
            self.early_brake = threading.Event()

        def steer_module(self, module, angle):
            result = super().steer_module(module, angle)
            if module == 'FL':
                self.started.set()
                assert self.release.wait(2)
            return result

        def _brake_all(self):
            self.commanded = 0
            self.early_brake.set()

    app, source = kiosk_client.application, RacingSteer()
    app.extensions['rc_slam'].source = source
    hdr = {'X-Robot-Control': 'drive', 'X-Robot-Command-Deadline': str(time.time() + 0.5)}
    token = kiosk_client.post('/api/kiosk_keyboard', json={'action': 'begin'}, headers=hdr).json['session']
    results = {}

    def steer():
        with app.test_client() as client:
            results['steer'] = client.post('/api/kiosk_keyboard', headers=hdr, json={
                'action': 'steer_all', 'session': token, 'sequence': 1,
                'angles': {m: 25 for m in MODULES}}).json

    def stop():
        with app.test_client() as client:
            results['stop'] = client.post('/api/kiosk_keyboard', headers=hdr, json={
                'action': 'stop', 'session': token, 'sequence': 2}).json

    a, b = threading.Thread(target=steer), threading.Thread(target=stop)
    a.start()
    try:
        assert source.started.wait(1)
        b.start()
        assert source.early_brake.wait(1)
    finally:
        source.release.set()
        a.join(2)
        if b.ident is not None:
            b.join(2)
    assert not a.is_alive() and not b.is_alive()
    assert results['steer']['reason'] == 'stale_command' and results['stop']['ok'], results
    assert [call[1] for call in source.calls if call[0] == 'steer'] == ['FL']
    assert source.commanded == 0


def test_overlapping_stop_fences_do_not_release_early():
    source, clock = LiveWheels(), Clock()
    ctl = controller(source, clock)
    token = begin(ctl)
    first = {'action': 'stop', 'session': token, 'sequence': 1}
    latest = {'action': 'stop', 'session': token, 'sequence': 3}
    assert ctl.preempt_stop(first) and ctl.preempt_stop(latest)
    assert ctl.handle(first)[0]['ok']
    ctl.finish_preempted_stop(first)
    assert send(ctl, token, 4, 'arm', direction=1, angles=straight())[0]['reason'] == 'stale_command'
    assert ctl.handle(latest)[0]['ok']
    ctl.finish_preempted_stop(latest)
    assert send(ctl, token, 4, 'arm', direction=1, angles=straight())[0]['ok']


def test_stop_registration_is_atomic_with_newer_sequence_acceptance():
    # If the newer command wins, STOP was already stale. If STOP wins, no
    # newer sequence may overtake it and make the final tagged brake stale.
    for _ in range(30):
        source, clock = LiveWheels(), Clock()
        ctl = controller(source, clock)
        token = begin(ctl)
        stop = {'action': 'stop', 'session': token, 'sequence': 4}
        ready_a, ready_b = threading.Event(), threading.Event()
        results = {}

        def register_stop():
            ready_a.set()
            results['stop'] = ctl.preempt_stop(stop)

        def accept_newer():
            ready_b.set()
            results['ping'] = send(ctl, token, 5, 'ping')[0]

        ctl._stop_lock.acquire()
        a, b = threading.Thread(target=register_stop), threading.Thread(target=accept_newer)
        try:
            a.start()
            b.start()
            assert ready_a.wait(1) and ready_b.wait(1)
        finally:
            ctl._stop_lock.release()
            a.join(2)
            b.join(2)
        assert not a.is_alive() and not b.is_alive()
        assert results['stop'] is not results['ping']['ok'], results
        if results['stop']:
            assert results['ping']['reason'] == 'stale_command'
            assert ctl.handle(stop)[0]['ok']
            ctl.finish_preempted_stop(stop)


def test_keyboard_traction_uses_full_zero_to_hundred_range():
    """По умолчанию WASD отдаёт 0…100 %, а 0 % — это STOP без потери сессии."""
    clock, source = Clock(), Wheels()
    ctl = controller(source, clock)
    assert ctl.max_percent == 100
    result, status = ctl.handle({"action": "begin"})
    assert status == 200 and result["maxPercent"] == 100
    token = result["session"]
    armed, status = send(ctl, token, 1, "arm", direction=1, angles=straight())
    assert status == 200 and armed["ok"]
    full, status = send(ctl, token, 2, "drive", percent=100, angles=straight(),
                        permit=armed["permit"])
    assert status == 200 and full["ok"] and source.commanded == 100
    # 0 % останавливает тягу через общий тормоз, сессия остаётся активной.
    stopped, status = send(ctl, token, 3, "live", percent=0,
                           angles=straight(), permit="")
    assert status == 200 and stopped["ok"] and source.commanded == 0
    assert send(ctl, token, 4, "ping")[0]["ok"]
    # И после нуля доступна полная тяга в обе стороны.
    armed, status = send(ctl, token, 5, "arm", direction=-1, angles=straight())
    assert status == 200 and armed["ok"]
    reverse, status = send(ctl, token, 6, "drive", percent=-100, angles=straight(),
                           permit=armed["permit"])
    assert status == 200 and reverse["ok"] and source.commanded == -100


def test_keyboard_configurable_cap_is_still_enforced(monkeypatch):
    """Оператор может сузить диапазон; проверка остаётся на борту."""
    monkeypatch.setenv("RC_KEYBOARD_MAX_PERCENT", "50")
    clock, source = Clock(), Wheels()
    ctl = controller(source, clock)
    assert ctl.max_percent == 50
    result, status = ctl.handle({"action": "begin"})
    assert status == 200 and result["maxPercent"] == 50
    token = result["session"]
    armed, _ = send(ctl, token, 1, "arm", direction=1, angles=straight())
    above, status = send(ctl, token, 2, "drive", percent=51, angles=straight(),
                         permit=armed["permit"])
    assert status == 400 and above["reason"] == "speed_limit"
    assert source.commanded == 0
    # Отказ снимает сессию; предел одинаков и для живой тяги на ходу.
    token = begin(ctl)
    armed, _ = send(ctl, token, 1, "arm", direction=1, angles=straight())
    above, status = send(ctl, token, 2, "live", percent=51,
                         angles=straight(), permit=armed["permit"])
    assert status == 400 and above["reason"] == "speed_limit"
    assert source.commanded == 0


def test_keyboard_limit_follows_environment(monkeypatch):
    monkeypatch.setenv("RC_KEYBOARD_MAX_PERCENT", "12")
    clock, source = Clock(), Wheels()
    ctl = controller(source, clock)
    assert ctl.max_percent == 12
    assert ctl.handle({"action": "begin"})[0]["maxPercent"] == 12
    monkeypatch.setenv("RC_KEYBOARD_MAX_PERCENT", "0")
    assert controller(source, clock).max_percent == 0
    # Некорректное значение возвращает полный диапазон и не роняет борт.
    monkeypatch.setenv("RC_KEYBOARD_MAX_PERCENT", "не число")
    assert controller(source, clock).max_percent == 100


def test_wasd_slider_has_full_percent_range():
    html = (Path(__file__).resolve().parents[1] / "slam_gui/robot.html").read_text()
    assert 'id="keyboard-speed" type="range" min="0" max="100" step="1" value="20"' in html
    assert 'id="keyboard-speed-max">100<' in html


# --------------------------------------------------------------- ограничения
class Guard:
    """Подставной ограничитель: состояние задаёт тест, как это делает борт."""

    def __init__(self, **state):
        self.state = {"blocked": False, "blockedReason": None, "bump": False,
                      "speedCapPercent": 100, "slowdown": 0.0, **state}

    def snapshot(self, **_):
        return dict(self.state)


def guarded(source, clock, guard):
    def drive(src, payload):
        return src.drive(payload["percent"]), 200

    def steer(src, payload):
        return src.steer_module(payload["module"], payload["angle"]), 200

    return KeyboardControl(lambda: source, drive, steer, lambda: None, clock, guard=guard)


def test_red_light_blocks_traction_but_not_braking_or_steering():
    clock, source = Clock(), Wheels()
    guard = Guard()
    ctl = guarded(source, clock, guard)
    token = begin(ctl)
    assert start_drive(ctl, token, 1, 8, straight())[0]["ok"]
    assert source.commanded == 8

    # Красный подтвердился на ходу: тяга снимается, сессия остаётся живой.
    guard.state = {"blocked": True, "blockedReason": "красный (CAM 0)", "bump": False,
                   "speedCapPercent": 100, "slowdown": 0.0}
    result, status = send(ctl, token, 3, "drive", percent=8, angles=straight())
    assert status == 409 and result["reason"] == "red_light_block"
    assert source.commanded == 0

    # Повторная попытка и новое разрешение тоже не дают тяги.
    result, _ = send(ctl, token, 4, "arm", direction=1, angles=straight())
    assert result["reason"] == "red_light_block"
    result, _ = send(ctl, token, 5, "live", percent=8, angles=straight())
    assert result["reason"] == "red_light_block"
    assert source.commanded == 0
    # Руль и STOP остаются доступны: стоящий робот должен уметь тормозить и
    # выставлять колёса, чтобы уехать сразу по зелёному.
    assert send(ctl, token, 6, "stop")[0]["ok"]
    result, _ = send(ctl, token, 7, "steer", module="FR", angle=10)
    assert result["ok"] and source.motors["FR"]["angle"] == 10
    assert send(ctl, token, 8, "steer", module="FR", angle=0)[0]["ok"]

    # Зелёный снял блокировку — тяга снова разрешена без перезапуска.
    guard.state["blocked"] = False
    token = begin(ctl)
    assert start_drive(ctl, token, 1, 8, straight())[0]["ok"]
    assert source.commanded == 8


def test_broken_guard_fails_closed_instead_of_allowing_drive():
    clock, source = Clock(), Wheels()

    class Broken:
        def snapshot(self, **_):
            raise RuntimeError("камера отвалилась")

    ctl = guarded(source, clock, Broken())
    token = begin(ctl)
    result, _ = send(ctl, token, 1, "arm", direction=1, angles=straight())
    assert result["reason"] == "red_light_block" and source.commanded == 0
    assert ctl.guard_state()["blocked"] is True


def test_bump_sign_caps_keyboard_traction_by_twenty_percent():
    clock, source = Clock(), LiveWheels()
    guard = Guard()
    ctl = guarded(source, clock, guard)
    token = begin(ctl)
    assert start_live(ctl, token, percent=40)[0]["ok"]
    assert source.commanded == 40

    # «Кочка» в кадре: потолок 80 % — запрошенные 40 % не урезаются.
    guard.state.update(bump=True, speedCapPercent=80, slowdown=0.2)
    clock.now += 0.2
    source.pulses()
    assert send(ctl, token, 3, "live", percent=40, angles=straight())[0]["ok"]
    assert source.commanded == 40

    # Полная шкала урезается до 80 %: снижение на 20 % делает борт, а не экран.
    clock.now += 0.2
    source.pulses()
    result, status = send(ctl, token, 4, "live", percent=100, angles=straight())
    assert status == 200 and result["ok"]
    assert source.commanded == 80
    assert send(ctl, token, 5, "live", percent=0, angles=straight())[0]["ok"]
    assert source.commanded == 0

    # Реверс урезается по модулю, знак сохраняется.
    for motor in source.motors.values():
        motor.update(scRpm=0.0, moving=False)
    token = begin(ctl)
    result, _ = send(ctl, token, 1, "arm", direction=-1, angles=straight())
    assert result["ok"]
    assert send(ctl, token, 2, "drive", percent=-90, angles=straight(),
                permit=result["permit"])[0]["ok"]
    assert source.commanded == -80


def test_guard_state_is_reported_on_begin_and_in_snapshot():
    clock, source = Clock(), Wheels()
    guard = Guard()
    ctl = guarded(source, clock, guard)
    result, status = ctl.handle({"action": "begin"})
    assert status == 200 and result["guard"]["blocked"] is False
    assert result["guard"]["speedCapPercent"] == 100
    assert "blocked" in ctl.guard_state()
    # Без ограничителя (стенд, плата без камер) тяга не ограничена.
    plain = controller(Wheels(), Clock())
    assert plain.guard_state() == {"blocked": False, "speedCapPercent": 100,
                                   "bump": False, "reason": "ограничения по знакам не подключены"}


def test_sign_appearing_between_permit_and_first_packet_still_caps_speed():
    """«Кочка» между ARM и первым пакетом: разрешение не даёт полную тягу."""
    clock, source = Clock(), LiveWheels()
    guard = Guard()
    ctl = guarded(source, clock, guard)
    token = begin(ctl)
    arm, status = send(ctl, token, 1, "arm", direction=1, angles=straight())
    assert status == 200 and arm["ok"]
    guard.state.update(bump=True, speedCapPercent=80, slowdown=0.2)
    # Пакет урезается ещё до сверки с разрешением: предел действует на любую
    # команду тяги, а не только на продолжение уже начатого хода.
    assert send(ctl, token, 2, "live", percent=100, angles=straight(),
                permit=arm["permit"])[0]["ok"]
    assert source.commanded == 80


# ------------------------------- умный круиз-контроль: пауза у препятствия
class Safety:
    """Подставной контроллер препятствий: пауза и близкое препятствие."""

    def __init__(self, near=None, hold_s=5.0, clock=None):
        self.near = near
        self.hold_s = hold_s
        self.until = 0.0
        self.calls = []
        # Свой снимок паузы считается по тем же часам, что и у клавиатуры.
        self.clock = clock

    def _now(self):
        return float(self.clock()) if self.clock is not None else getattr(self, "now", 0.0)

    def request_hold(self, direction, now=None):
        self.calls.append((direction, now))
        now = getattr(self, "now", 0.0) if now is None else now
        self.now = now
        if now < self.until:
            return {"hold": True, "holdRemainingS": round(self.until - now, 1)}
        if self.near is not None and direction and self.near <= 0.5:
            self.until = now + self.hold_s
            return {"hold": True, "holdRemainingS": self.hold_s, "holdTriggered": True}
        return {"hold": False, "holdRemainingS": 0.0}

    def snapshot(self, **_):
        now = self._now()
        return {"hold": self.until > now, "holdRemainingS": max(0.0, self.until - now)}

    def state(self):
        return self.snapshot()

    def clear_hold(self, reason=""):
        # Как настоящий контроллер: оператор снимает паузу досрочно.
        self.calls.append(("clear", reason))
        self.until = 0.0


def guarded_safety(source, clock, safety):
    def drive(src, payload):
        return src.drive(payload["percent"]), 200

    def steer(src, payload):
        return src.steer_module(payload["module"], payload["angle"]), 200

    return KeyboardControl(lambda: source, drive, steer, lambda: None, clock,
                           safety_provider=lambda: safety)


def test_obstacle_hold_stops_traction_and_refuses_to_restart_into_the_wall():
    clock, source = Clock(), Wheels()
    safety = Safety(near=0.4, clock=clock)
    ctl = guarded_safety(source, clock, safety)
    token = begin(ctl)

    # Тронуться в стену нельзя: пауза начинается до выдачи тяги, тяга не уходит.
    result, status = send(ctl, token, 1, "arm", direction=1, angles=straight())
    assert status == 409 and result["reason"] == "obstacle_hold"
    assert result["holdRemainingS"] == pytest.approx(5.0, abs=0.1)
    assert source.commanded == 0
    assert ctl.hold_snapshot()["hold"] is True

    # Паузу нельзя обойти удержанием клавиши: каждая попытка — отказ.
    for seq in (2, 3, 4):
        result, _ = send(ctl, token, seq, "arm", direction=1, angles=straight())
        assert result["reason"] == "obstacle_hold"
        assert source.commanded == 0

    # Пауза вышла, но стена всё ещё близко — новая попытка снова упирается.
    clock.now += 6.0
    result, _ = send(ctl, token, 5, "arm", direction=1, angles=straight())
    assert result["reason"] == "obstacle_hold" and source.commanded == 0

    # Путь свободен — ход разрешён без перезапуска режима.
    safety.near = 2.0
    clock.now += 6.0
    assert start_drive(ctl, token, 6, 8, straight())[0]["ok"]
    assert source.commanded == 8


def test_obstacle_hold_never_blocks_braking_or_steering():
    clock, source = Clock(), Wheels()
    safety = Safety(near=0.3, clock=clock)
    ctl = guarded_safety(source, clock, safety)
    token = begin(ctl)
    result, _ = send(ctl, token, 1, "arm", direction=1, angles=straight())
    assert result["reason"] == "obstacle_hold"
    assert send(ctl, token, 2, "stop")[0]["ok"]
    assert send(ctl, token, 3, "steer", module="FR", angle=0)[0]["ok"]
    # Явный STOP и 0 % проходят всегда: пауза — про тягу, не про тормоз.
    assert send(ctl, token, 4, "live", percent=0, angles=straight())[0]["ok"] is True


def test_dead_safety_controller_does_not_allow_drive():
    clock, source = Clock(), Wheels()

    class Broken:
        def state(self):
            return self.snapshot()

        def snapshot(self, **_):
            raise RuntimeError("лидар отвалился")

        def request_hold(self, direction, now=None):
            raise RuntimeError("лидар отвалился")

    ctl = guarded_safety(source, clock, Broken())
    token = begin(ctl)
    result, _ = send(ctl, token, 1, "arm", direction=1, angles=straight())
    assert result["reason"] == "obstacle_hold" and source.commanded == 0
    assert ctl.hold_snapshot()["hold"] is True


def test_driving_into_an_object_pauses_five_seconds_then_driving_returns_by_itself():
    """Главный сценарий: ехали, объект ближе 0,5 м — стоп на 5 с и возврат хода."""
    clock, source = Clock(), LiveWheels()
    safety = Safety(near=None, clock=clock)
    ctl = guarded_safety(source, clock, safety)
    token = begin(ctl)
    assert start_live(ctl, token, percent=20)[0]["ok"]
    assert source.commanded == 20

    # На ходу объект оказался ближе полуметра: тяга снимается сразу, ещё до
    # отправки следующего пакета на платы.
    safety.near = 0.42
    clock.now += 0.2
    source.pulses()
    result, status = send(ctl, token, 3, "live", percent=20, angles=straight())
    assert status == 409 and result["reason"] == "obstacle_hold"
    assert result["holdRemainingS"] == pytest.approx(5.0, abs=0.1)
    assert result["retryable"] is False
    assert source.commanded == 0

    # Удержание клавиши паузу не пробивает: каждый пакет получает отказ.
    for seq in (4, 5, 6):
        clock.now += 0.2
        source.pulses()
        result, _ = send(ctl, token, seq, "live", percent=20, angles=straight())
        assert result["reason"] == "obstacle_hold" and source.commanded == 0
    assert ctl.hold_snapshot()["hold"] is True

    # Пять секунд вышло, объект убрали: ход возвращается сам, без перезапуска.
    # Браузер присылает следующий пакет с новым номером, а экран уже показывает,
    # что пауза кончилась.
    safety.near = None
    clock.now += 5.2
    for motor in source.motors.values():
        motor.update(angle=0.0, moving=False, scRpm=0.0, scRecent=False)
    assert ctl.hold_snapshot()["hold"] is False
    arm, status = send(ctl, token, 7, "arm", direction=1, angles=straight())
    assert status == 200 and arm["ok"]
    assert send(ctl, token, 8, "live", percent=20, angles=straight(),
                permit=arm["permit"])[0]["ok"]
    assert source.commanded == 20


def test_stop_and_steering_stay_available_during_the_obstacle_pause():
    clock, source = Clock(), Wheels()
    safety = Safety(near=0.3, clock=clock)
    ctl = guarded_safety(source, clock, safety)
    token = begin(ctl)
    assert send(ctl, token, 1, "arm", direction=1, angles=straight())[0]["reason"] == "obstacle_hold"
    # Пауза — про тягу: тормоз, руль и 0 % доступны, сессия не закрывается.
    assert send(ctl, token, 2, "stop")[0]["ok"]
    assert send(ctl, token, 3, "steer", module="FR", angle=12)[0]["ok"]
    assert source.motors["FR"]["angle"] == 12
    assert send(ctl, token, 4, "live", percent=0, angles=straight())[0]["ok"]


def test_obstacle_pause_can_be_cancelled_early_by_the_operator():
    clock, source = Clock(), Wheels()
    safety = Safety(near=None, hold_s=5.0, clock=clock)
    ctl = guarded_safety(source, clock, safety)
    token = begin(ctl)
    safety.near = 0.3
    assert send(ctl, token, 1, "arm", direction=1, angles=straight())[0]["reason"] == "obstacle_hold"
    safety.near = None
    ctl.clear_hold("оператор")
    assert ctl.hold_snapshot()["hold"] is False
    assert start_drive(ctl, token, 2, 8, straight())[0]["ok"]
