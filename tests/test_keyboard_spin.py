"""Hold-to-turn keyboard pivot: geometry, per-board signs and fail-closed fences."""
from __future__ import annotations

import math
import time

import pytest

from robot_control import maneuvers
from test_keyboard_control import Clock, LiveWheels, controller, send, start_live, straight

MODULES = ("FL", "FR", "RL", "RR")


def session(ctl):
    result, status = ctl.handle({"action": "begin"})
    assert status == 200 and result["ok"]
    return result["session"], result["spin"]


def prepare(ctl, token, geometry, direction=-1, sequence=1):
    angles = geometry["angles"]
    result, status = send(ctl, token, sequence, "steer_all", pattern="spin", angles=angles)
    assert status == 200 and result["ok"], result
    result, status = send(ctl, token, sequence + 1, "arm", pattern="spin",
                          direction=direction, angles=angles)
    assert status == 200 and result["ok"], result
    return result["permit"]


def launch(ctl, token, geometry, direction=-1, sequence=1, speed=20):
    permit = prepare(ctl, token, geometry, direction, sequence)
    return send(ctl, token, sequence + 2, "live", pattern="spin", percent=direction * speed,
                angles=geometry["angles"], permit=permit)


def test_geometry_is_from_saved_axes_and_cross_checked_against_maneuvers():
    source = LiveWheels()
    ctl = controller(source, Clock())
    token, geometry = session(ctl)
    assert geometry["wheelbaseMm"] == 500 and geometry["trackMm"] == 380
    assert geometry["angleDeg"] == pytest.approx(52.76516601842533, abs=1e-12)
    angles, signs = maneuvers.steering_targets("spin_left", source.chassis)
    assert geometry["angles"] == angles
    assert signs == {"FL": -1, "FR": 1, "RL": -1, "RR": 1}
    assert launch(ctl, token, geometry)[0]["ok"]
    assert source.commanded_modules == {"FL": -20, "FR": 20, "RL": -20, "RR": 20}

    # Test the tangential velocity vectors, not just a table of signs. The
    # physical positive steering angle is clockwise in this firmware.
    positions = {"FL": (250, 190), "FR": (250, -190),
                 "RL": (-250, 190), "RR": (-250, -190)}
    for module, (x, y) in positions.items():
        theta = math.radians(source.motors[module]["angle"])
        sign = source.commanded_modules[module] / 20
        vx, vy = sign * math.cos(theta), -sign * math.sin(theta)
        assert x * vx + y * vy == pytest.approx(0, abs=1e-10)
        assert x * vy - y * vx > 0  # left = counter-clockwise


def test_saved_dimensions_not_hard_coded():
    source = LiveWheels()
    source.chassis.update(wheelbaseMm=500, trackMm=450)
    ctl = controller(source, Clock())
    token, geometry = session(ctl)
    assert geometry["angleDeg"] == pytest.approx(math.degrees(math.atan2(500, 450)))
    assert geometry["angleDeg"] != pytest.approx(52.76516601842533)
    assert launch(ctl, token, geometry, direction=1)[0]["ok"]
    assert source.commanded_modules == {"FL": 20, "FR": -20, "RL": 20, "RR": -20}


def test_spin_stop_reverse_and_stale_heartbeat():
    source, clock = LiveWheels(), Clock()
    ctl = controller(source, clock)
    token, geometry = session(ctl)
    assert launch(ctl, token, geometry)[0]["ok"]
    source.pulses()
    clock.now += 0.12
    assert send(ctl, token, 4, "live", pattern="spin", percent=-35,
                angles=geometry["angles"])[0]["ok"]
    assert source.commanded_modules == {"FL": -35, "FR": 35, "RL": -35, "RR": 35}
    assert send(ctl, token, 6, "stop")[0]["ok"]
    assert not source.commanded_modules
    old = send(ctl, token, 5, "live", pattern="spin", percent=-20, angles=geometry["angles"])[0]
    assert old["reason"] == "stale_command" and not source.commanded_modules
    for motor in source.motors.values():
        motor.update(scRpm=0, scRecent=False)
    permit = prepare(ctl, token, geometry, direction=1, sequence=7)
    assert send(ctl, token, 9, "live", pattern="spin", percent=20,
                angles=geometry["angles"], permit=permit)[0]["ok"]
    assert source.commanded_modules == {"FL": 20, "FR": -20, "RL": 20, "RR": -20}
    assert send(ctl, token, 10, "stop")[0]["ok"]
    assert not source.commanded_modules


@pytest.mark.parametrize("fault,expected", [
    ("reverse", "module_busy"), ("pattern", "module_busy"),
    ("geometry", "spin_geometry_changed"), ("angle", "invalid_angle"),
    ("bad_sign", "live_unconfirmed"), ("sc", "sc_invalid"), ("stale", "module_offline"),
])
def test_spin_fault_brakes_all_and_revokes_session(fault, expected):
    source, clock = LiveWheels(), Clock()
    ctl = controller(source, clock)
    token, geometry = session(ctl)
    assert launch(ctl, token, geometry)[0]["ok"]
    clock.now += 0.1
    source.pulses()
    payload = {"pattern": "spin", "percent": -20, "angles": dict(geometry["angles"])}
    if fault == "reverse":
        payload["percent"] = 20
    elif fault == "pattern":
        del payload["pattern"]  # same scalar sign must not become ordinary driving
    elif fault == "geometry":
        source.chassis["trackMm"] = 450
    elif fault == "angle":
        payload["angles"]["FR"] += 1
    elif fault == "bad_sign":
        source.motors["FR"]["drivePercent"] = -20
    elif fault == "sc":
        source.motors["FR"]["scRecent"] = None
    else:
        source.motors["RR"]["telemetryAgeS"] = 0.8
    result, _ = send(ctl, token, 4, "live", **payload)
    assert result["reason"] == expected
    assert source.commanded == 0 and source.commanded_modules == {}
    assert all(m["drivePercent"] == 0 for m in source.motors.values())
    assert send(ctl, token, 5, "live", pattern="spin", percent=-20,
                angles=geometry["angles"])[0]["reason"] == "keyboard_session"


def test_linear_to_spin_same_sign_requires_stop_and_fresh_permit():
    source, clock = LiveWheels(), Clock()
    ctl = controller(source, clock)
    token, geometry = session(ctl)
    assert start_live(ctl, token, percent=-20)[0]["ok"]
    result, _ = send(ctl, token, 3, "live", pattern="spin", percent=-20, angles=geometry["angles"])
    assert result["reason"] == "module_busy"
    assert source.commanded == 0 and not source.commanded_modules


def test_spin_requires_exact_plan_close_actual_angles_and_the_right_permit():
    source, clock = LiveWheels(), Clock()
    ctl = controller(source, clock)
    token, geometry = session(ctl)
    angles = geometry["angles"]
    wrong = dict(angles, FR=angles["FR"] + 0.1)
    result, _ = send(ctl, token, 1, "steer_all", pattern="spin", angles=wrong)
    assert result["reason"] == "invalid_angle" and not source.commanded_modules

    token, geometry = session(ctl)
    angles = geometry["angles"]
    assert send(ctl, token, 1, "steer_all", pattern="spin", angles=angles)[0]["ok"]
    source.motors["RR"]["angle"] += 1.01  # still reject a wheel outside the 1° bound
    result, _ = send(ctl, token, 2, "arm", pattern="spin", direction=-1, angles=angles)
    assert result["reason"] == "steering_changed" and not source.commanded_modules

    token, geometry = session(ctl)
    permit = prepare(ctl, token, geometry)
    # A spin permit cannot be used to energize all boards in the same direction.
    result, _ = send(ctl, token, 3, "live", percent=-20, angles=geometry["angles"], permit=permit)
    assert result["reason"] == "keyboard_expired" and not source.commanded_modules


def test_pivot_accepts_realistic_potentiometer_residual_but_still_requires_all_wheels():
    source, clock = LiveWheels(), Clock()
    ctl = controller(source, clock)
    token, geometry = session(ctl)
    angles = geometry["angles"]
    assert send(ctl, token, 1, "steer_all", pattern="spin", angles=angles)[0]["ok"]
    for m in source.motors.values():
        m["angle"] += 0.6
    armed, _ = send(ctl, token, 2, "arm", pattern="spin", direction=1, angles=angles)
    assert armed["ok"], armed
    assert send(ctl, token, 3, "live", pattern="spin", percent=20, angles=angles,
                permit=armed["permit"])[0]["ok"]
    source.pulses()
    clock.now += 0.1
    assert send(ctl, token, 4, "live", pattern="spin", percent=20, angles=angles)[0]["ok"]
    assert send(ctl, token, 5, "stop")[0]["ok"]
    assert not source.commanded_modules


def test_missing_geometry_or_individual_drive_disables_only_spin():
    for bad in (None, 0, float("nan"), True, 4000):
        source, clock = LiveWheels(), Clock()
        source.chassis["wheelbaseMm"] = bad
        ctl = controller(source, clock)
        token, geometry = session(ctl)
        assert geometry is None
        result, _ = send(ctl, token, 1, "arm", pattern="spin", direction=-1, angles=straight())
        assert result["reason"] == "spin_geometry_required" and not source.commanded_modules
    source, clock = LiveWheels(), Clock()
    source.supports_individual_drive = False
    ctl = controller(source, clock)
    token, geometry = session(ctl)
    assert geometry is None
    assert start_live(ctl, token)[0]["ok"]  # ordinary keyboard remains compatible


def test_missing_one_spin_ack_stops_every_wheel():
    source, clock = LiveWheels(), Clock()
    ctl = controller(source, clock)
    token, geometry = session(ctl)
    source.quiet_ack.add("RL")
    assert launch(ctl, token, geometry)[0]["ok"]
    for seq in range(4, 9):
        clock.now += 0.12
        source.pulses()
        result, _ = send(ctl, token, seq, "live", pattern="spin", percent=-20,
                         angles=geometry["angles"])
        if not result["ok"]:
            break
    assert result["reason"] == "live_unconfirmed"
    assert source.commanded == 0 and source.commanded_modules == {}


def test_configuration_update_interrupts_armed_keyboard(kiosk_client):
    source = LiveWheels()
    def set_config(name, values):
        assert name == "chassis"
        source.chassis.update(values)
        return {"ok": True}
    source.set_maneuver_config = set_config
    app = kiosk_client.application
    app.extensions["rc_slam"].source = source
    drive = {"X-Robot-Control": "drive", "X-Robot-Command-Deadline": str(time.time() + 0.5)}
    start = kiosk_client.post("/api/kiosk_keyboard", headers=drive, json={"action": "begin"}).json
    token, angles = start["session"], start["spin"]["angles"]
    assert kiosk_client.post("/api/kiosk_keyboard", headers=drive, json={
        "action": "steer_all", "session": token, "sequence": 1, "pattern": "spin", "angles": angles}).json["ok"]
    permit = kiosk_client.post("/api/kiosk_keyboard", headers=drive, json={
        "action": "arm", "session": token, "sequence": 2, "pattern": "spin",
        "direction": -1, "angles": angles}).json["permit"]
    packet = {"action": "live", "session": token, "sequence": 3, "pattern": "spin",
              "percent": -20, "angles": angles, "permit": permit}
    assert kiosk_client.post("/api/kiosk_keyboard", headers=drive, json=packet).json["ok"]
    changed = kiosk_client.post("/api/kiosk_maneuver_config", headers={"X-Robot-Control": "steering"},
                                json={"name": "chassis", "values": {"trackMm": 450}})
    assert changed.json["ok"] and not source.commanded_modules
    packet.update(sequence=4, permit="")
    assert kiosk_client.post("/api/kiosk_keyboard", headers=drive, json=packet).json["reason"] == "keyboard_session"


def test_spin_requires_each_sensor_to_keep_advancing():
    for quiet in MODULES:
        source, clock = LiveWheels(), Clock()
        ctl = controller(source, clock)
        token, geometry = session(ctl)
        assert launch(ctl, token, geometry)[0]["ok"]
        for sequence in range(4, 25):
            clock.now += 0.2
            for module in MODULES:
                if module != quiet:
                    source.motors[module]["scCount"] += 3
                    source.motors[module]["scRpm"] = 12
            result, _ = send(ctl, token, sequence, "live", pattern="spin", percent=-20,
                             angles=geometry["angles"])
            if not result["ok"]:
                break
        assert result["reason"] == "sc_stalled"
        assert not source.commanded_modules and all(m["drivePercent"] == 0 for m in source.motors.values())


def test_spin_requires_fw20_and_does_not_resume_after_heartbeat_timeout():
    source, clock = LiveWheels(), Clock()
    ctl = controller(source, clock)
    token, geometry = session(ctl)
    permit = prepare(ctl, token, geometry)
    source.motors["FR"]["firmware"] = 19
    reply, _ = send(ctl, token, 3, "live", pattern="spin", percent=-20,
                    angles=geometry["angles"], permit=permit)
    assert reply["reason"] == "live_firmware_required" and not source.commanded_modules

    source.motors["FR"]["firmware"] = 20
    token, geometry = session(ctl)
    assert launch(ctl, token, geometry)[0]["ok"]
    clock.now += 0.61
    reply, _ = send(ctl, token, 4, "live", pattern="spin", percent=-20, angles=geometry["angles"])
    assert reply["reason"] == "keyboard_expired" and not source.commanded_modules
    assert send(ctl, token, 5, "live", pattern="spin", percent=-20,
                angles=geometry["angles"])[0]["reason"] == "keyboard_session"


def test_spin_stops_if_confirmed_angle_drifts_beyond_tolerance():
    source, clock = LiveWheels(), Clock()
    ctl = controller(source, clock)
    token, geometry = session(ctl)
    assert launch(ctl, token, geometry)[0]["ok"]
    clock.now += 0.1
    source.pulses()
    source.motors["RR"]["angle"] += 1.01
    reply, _ = send(ctl, token, 4, "live", pattern="spin", percent=-20, angles=geometry["angles"])
    assert reply["reason"] == "steering_changed" and not source.commanded_modules


def test_begin_geometry_reply_cannot_mutate_the_server_session_plan():
    source, clock = LiveWheels(), Clock()
    ctl = controller(source, clock)
    token, geometry = session(ctl)
    geometry["angles"]["FL"] = 0
    reply, _ = send(ctl, token, 1, "steer_all", pattern="spin", angles=geometry["angles"])
    assert reply["reason"] == "invalid_angle" and not source.commanded_modules
