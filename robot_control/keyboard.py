"""Sequenced hold-to-run keyboard control, owned by the onboard process.

The browser plans steering; the server independently verifies fresh wheel
telemetry and the requested angles before every traction heartbeat. A lease,
monotonic sequence and short drive deadline prevent delayed requests from
restarting motion after a release, timeout or manual override. Firmware's
650 ms watchdog is the final backstop if either process or the network dies.
"""
from __future__ import annotations

import math
import secrets
import threading
import time

MODULES = ("FL", "FR", "RL", "RR")


class KeyboardControl:
    IDLE_TTL = 10.0
    # Shorter than the firmware watchdog: a late heartbeat cannot resume it.
    DRIVE_TTL = 0.60
    # A first traction packet may be delayed before it ever reaches the robot.
    # It must use a freshly issued, single-use permit, not an idle lease.
    START_PERMIT_TTL = 0.50
    TELEMETRY_AGE_S = 0.75
    SC_SILENCE_S = 3.0
    ANGLE_TOLERANCE_DEG = 1.5

    def __init__(self, source, drive, steer, estop, clock=time.monotonic):
        self._source = source
        self._drive = drive
        self._steer = steer
        self._estop = estop
        self._clock = clock
        self._lock = threading.RLock()
        self._token = None
        self._sequence = 0
        self._last_activity = 0.0
        self._last_drive = 0.0
        self._running = False
        self._direction = 0
        self._percent = 0
        self._angles = None
        self._permit = None
        self._wheels = {}
        self._brake_id = None

    def _brake(self):
        self._brake_id = None
        try:
            source = self._source()
            # Preserve the normal STOP path and its override semantics.
            result, status = self._drive(source, {"percent": 0})
            if not result.get("ok"):
                return result, status
            barrier = getattr(source, "brake_for_control", None)
            if not callable(barrier):
                return {"ok": False, "reason": "brake_firmware_required"}, 409
            result = barrier()
            brake_id = result.get("brakeId")
            if not result.get("ok") or type(brake_id) is not int or not 0 < brake_id <= 0x7FFFFFFF:
                return {"ok": False, "reason": result.get("reason", "command_failed")}, 503
            self._brake_id = brake_id
            return {"ok": True, "brakeId": brake_id}, 200
        except Exception:  # hardware watchdog remains the backstop
            return {"ok": False, "reason": "command_failed"}, 503

    def _clear(self):
        self._token = None
        self._running = False
        self._direction = self._percent = 0
        self._angles = None
        self._permit = None
        self._wheels = {}

    def interrupt(self):
        """Another operator/mode takes priority, including an ordinary STOP."""
        with self._lock:
            if self._token is not None:
                self._clear()
                self._brake()

    def _fail(self, reason, status=409):
        self._clear()
        self._brake()
        return {"ok": False, "reason": reason}, status

    @staticmethod
    def _number(value):
        return (isinstance(value, (int, float)) and not isinstance(value, bool)
                and math.isfinite(value))

    def _snapshot(self):
        try:
            snap = self._source().read()
        except Exception:
            return None, "module_offline"
        if not isinstance(snap, dict) or snap.get("linkOk") is False:
            return None, "module_offline"
        drive = snap.get("drive") or {}
        if not drive.get("available"):
            return None, "serial_unavailable"
        motors = {m.get("id"): m for m in (snap.get("motors") or []) if isinstance(m, dict)}
        for module in MODULES:
            motor = motors.get(module) or {}
            age, rpm = motor.get("telemetryAgeS"), motor.get("scRpm")
            if motor.get("online") is not True or not self._number(age) or not 0 <= age < self.TELEMETRY_AGE_S:
                return None, "module_offline"
            if motor.get("homed") is not True:
                return None, "not_homed"
            if motor.get("calibrating") or motor.get("cycling"):
                return None, "module_busy"
            if not self._number(rpm) or rpm < 0:
                return None, "sc_invalid"
            brake_id = motor.get("brakeId")
            if type(brake_id) is not int or not 0 <= brake_id <= 0x7FFFFFFF:
                return None, "brake_firmware_required"
            if not self._number(motor.get("angle")) or type(motor.get("drivePercent")) is not int:
                return None, "module_offline"
        return (drive, motors), None

    def _steering_ready(self, snapshot):
        drive, motors = snapshot
        if self._brake_id is None or any(motors[m]["brakeId"] != self._brake_id for m in MODULES):
            return "brake_unconfirmed"
        if drive.get("active") or any(motors[m]["drivePercent"] != 0 for m in MODULES):
            return "module_busy"
        if any(motors[m]["scRpm"] > 1.0 for m in MODULES):
            return "wheel_moving"
        return None

    def _target_ready(self, snapshot, angles, first):
        drive, motors = snapshot
        if not drive.get("canDrive"):
            return drive.get("reason") or "drive_not_ready"
        if first:
            error = self._steering_ready(snapshot)
            if error:
                return error
        elif drive.get("commandedPercent") != self._percent:
            # A different controller changed or stopped traction; do not resume.
            return "manual_override"
        for module in MODULES:
            motor = motors[module]
            if motor.get("moving") or abs(motor["angle"] - angles[module]) > self.ANGLE_TOLERANCE_DEG:
                return "steering_changed"
        return None

    def _sensor_progress(self, motors, now, first):
        for module in MODULES:
            motor = motors[module]
            count = motor.get("scCount")
            v17 = type(motor.get("firmware")) is int and motor["firmware"] >= 17
            if v17 and (type(count) is not int or not 0 <= count <= 0xFFFFFFFF):
                return "sc_invalid"
            if first:
                self._wheels[module] = {"count": count, "sampleAt": now, "pulseAt": now}
                continue
            previous = self._wheels[module]
            if v17:
                if type(previous["count"]) is not int:
                    return "sc_changed"
                delta = (count - previous["count"]) & 0xFFFFFFFF
                if delta > 6700 * (max(0.0, now - previous["sampleAt"]) + 0.5) + 2:
                    return "sc_changed"
                if delta:
                    previous["pulseAt"] = now
                previous.update(count=count, sampleAt=now)
            elif motor["scRpm"] > 0:
                # v16 lacks a counter; zero also means a disconnected SC wire.
                previous["pulseAt"] = now
            if now - previous["pulseAt"] >= self.SC_SILENCE_S:
                return "sc_stalled"
        return None

    def handle(self, body):
        if not isinstance(body, dict) or not isinstance(body.get("action"), str):
            return {"ok": False, "reason": "payload"}, 400
        action = body["action"]
        with self._lock:
            now = self._clock()
            if action == "begin":
                if set(body) != {"action"}:
                    return {"ok": False, "reason": "payload"}, 400
                self._clear()
                result, status = self._brake()
                if not result.get("ok"):
                    return result, status
                if self._estop():
                    return {"ok": False, "reason": "estop"}, 409
                source = self._source()
                if not getattr(source, "supports_drive", False) or not getattr(source, "supports_individual_steer", False):
                    return {"ok": False, "reason": "control_not_supported"}, 409
                self._token = secrets.token_urlsafe(24)
                self._sequence = 0
                self._last_activity = now
                return {"ok": True, "session": self._token}, 200

            if action not in {"stop", "end", "ping", "steer", "arm", "drive"}:
                return {"ok": False, "reason": "invalid_action"}, 400
            if not isinstance(body.get("session"), str) or body["session"] != self._token:
                return {"ok": False, "reason": "keyboard_session"}, 409
            sequence = body.get("sequence")
            if type(sequence) is not int or sequence <= 0:
                return {"ok": False, "reason": "payload"}, 400
            if sequence <= self._sequence:
                return {"ok": False, "reason": "stale_command"}, 409
            self._sequence = sequence

            if action in {"stop", "end"}:
                if set(body) != {"action", "session", "sequence"}:
                    return self._fail("payload", 400)
                self._running = False
                self._angles = None
                self._permit = None
                self._wheels = {}
                self._direction = self._percent = 0
                self._last_activity = now
                result, status = self._brake()
                if action == "end" or not result.get("ok"):
                    self._clear()
                return result, status

            if self._estop():
                return self._fail("estop")
            if now - self._last_activity > self.IDLE_TTL or (self._running and now - self._last_drive > self.DRIVE_TTL):
                return self._fail("keyboard_expired")
            self._last_activity = now
            if action == "ping":
                if set(body) != {"action", "session", "sequence"}:
                    return self._fail("payload", 400)
                return {"ok": True}, 200

            snapshot, error = self._snapshot()
            if error:
                return self._fail(error)
            if action == "steer":
                if set(body) != {"action", "session", "sequence", "module", "angle"}:
                    return self._fail("payload", 400)
                module, angle = body.get("module"), body.get("angle")
                if module not in MODULES or not self._number(angle) or abs(angle) > 100:
                    return self._fail("invalid_angle", 400)
                if self._running:
                    return self._fail("module_busy")
                self._permit = None
                error = self._steering_ready(snapshot)
                if error:
                    return self._fail(error)
                result, status = self._steer(self._source(), {"module": module, "angle": angle})
                if not result.get("ok"):
                    return self._fail(result.get("reason", "command_failed"), status)
                return result, status

            if action == "arm":
                if set(body) != {"action", "session", "sequence", "direction", "angles"}:
                    return self._fail("payload", 400)
                angles, direction = body.get("angles"), body.get("direction")
                if (self._running or type(direction) is not int or direction not in (-1, 1) or
                        not isinstance(angles, dict) or set(angles) != set(MODULES) or
                        not all(self._number(angles[m]) and abs(angles[m]) <= 100 for m in MODULES)):
                    return self._fail("invalid_angle", 400)
                error = self._target_ready(snapshot, angles, first=True)
                if error:
                    return self._fail(error)
                nonce = secrets.token_urlsafe(18)
                self._permit = {"value": nonce, "expires": now + self.START_PERMIT_TTL,
                                "angles": dict(angles), "direction": direction}
                return {"ok": True, "permit": nonce}, 200

            if set(body) != {"action", "session", "sequence", "percent", "angles", "permit"}:
                return self._fail("payload", 400)
            percent, angles = body.get("percent"), body.get("angles")
            if type(percent) is not int or not 1 <= abs(percent) <= 100 or not isinstance(angles, dict) or set(angles) != set(MODULES):
                return self._fail("invalid_drive", 400)
            if not all(self._number(angles[m]) and abs(angles[m]) <= 100 for m in MODULES):
                return self._fail("invalid_angle", 400)
            direction = 1 if percent > 0 else -1
            if self._running and (direction != self._direction or angles != self._angles):
                return self._fail("module_busy")
            if not self._running:
                permit = self._permit
                if (not permit or not isinstance(body.get("permit"), str) or
                        body["permit"] != permit["value"] or now > permit["expires"] or
                        direction != permit["direction"] or angles != permit["angles"]):
                    return self._fail("keyboard_expired")
            error = self._target_ready(snapshot, angles, first=not self._running)
            if not error:
                error = self._sensor_progress(snapshot[1], now, first=not self._running)
            if error:
                return self._fail(error)
            result, status = self._drive(self._source(), {"percent": percent})
            if not result.get("ok"):
                return self._fail(result.get("reason", "command_failed"), status)
            self._running = True
            self._permit = None
            self._direction = direction
            self._percent = percent
            self._angles = dict(angles)
            self._last_drive = now
            return result, status
