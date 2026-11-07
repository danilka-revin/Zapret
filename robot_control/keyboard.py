"""Sequenced hold-to-run keyboard control, owned by the onboard process.

The browser plans steering; the server independently verifies fresh wheel
telemetry and the requested angles before every traction heartbeat. A lease,
monotonic sequence and short drive deadline prevent delayed requests from
restarting motion after a release, timeout or manual override. Firmware's
650 ms watchdog is the final backstop if either process or the network dies.
"""
from __future__ import annotations

import math
import os
import secrets
import threading
import time

from . import maneuvers

MODULES = ("FL", "FR", "RL", "RR")

#: Потолок тяги режима WASD, % шкалы Servo. Клавиатура не даёт оператору
#: обратной связи по скорости (нет ни ползунка под рукой, ни плавного
#: наращивания), поэтому предел ниже общего предела привода. Ограничение
#: проверяется на борту: браузер присылает только процент из своего ползунка.
DEFAULT_MAX_PERCENT = 30


def env_max_percent() -> int:
    """Потолок тяги WASD из ``RC_KEYBOARD_MAX_PERCENT`` (1…100)."""
    try:
        limit = int(os.environ.get("RC_KEYBOARD_MAX_PERCENT", DEFAULT_MAX_PERCENT))
    except (TypeError, ValueError):
        limit = DEFAULT_MAX_PERCENT
    return max(1, min(100, limit))


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
    # The target remains the calculated tangent, but real potentiometer
    # feedback has a 0.7° deadband (and ADC quantisation). A 0.15° gate
    # could never arm even when the firmware had correctly finished steering.
    # Still reject any stationary wheel over 1° away, including while moving.
    SPIN_ANGLE_TOLERANCE_DEG = 1.0
    # A missing board ACK must not be hidden by heartbeats to the other three.
    LIVE_ACK_TTL = 0.45
    LIVE_STEER_TTL = 6.0

    def __init__(self, source, drive, steer, estop, clock=time.monotonic,
                 max_percent: int | None = None):
        self._source = source
        self._drive = drive
        self._steer = steer
        self._estop = estop
        self._clock = clock
        self.max_percent = env_max_percent() if max_percent is None else max(1, min(100, int(max_percent)))
        self._lock = threading.RLock()
        # STOP must be advertised without waiting for the motion gate or the
        # potentially blocked request holding _lock. The short fence lock is
        # never held while acquiring _lock or while talking to hardware.
        self._stop_lock = threading.Lock()
        self._stop_fence = None
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
        self._live = False
        self._live_pending = []
        self._live_latest_seq = 0
        self._live_angles_since = None
        self._pattern = None
        self._wheel_percents = {}
        self._spin_geometry = None
        self._spin_source = None

    def _brake(self):
        self._brake_id = None
        try:
            source = self._source()
            # Preserve the normal STOP path and its override semantics.
            result, status = self._drive(source, {"percent": 0})
            if not result.get("ok"):
                return result, status
            # Keyboard STOP must also interrupt a partially completed steering
            # move, so releasing a key can promptly select a new angle.
            barrier = getattr(source, "brake_for_keyboard", None)
            if not callable(barrier):
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
        with self._stop_lock:
            self._token = None
        self._running = False
        self._direction = self._percent = 0
        self._angles = None
        self._permit = None
        self._wheels = {}
        self._live = False
        self._live_pending = []
        self._live_latest_seq = 0
        self._live_angles_since = None
        self._pattern = None
        self._wheel_percents = {}
        self._spin_geometry = None
        self._spin_source = None

    def interrupt(self):
        """Another operator/mode takes priority, including an ordinary STOP."""
        with self._lock:
            if self._token is not None:
                self._clear()
                self._brake()

    def preempt_stop(self, body):
        """Fence a validated STOP before its immediate, out-of-gate brake.

        A heartbeat already holding the motion gate can otherwise see that
        brake as a manual override and revoke the session before STOP arrives.
        The final tagged STOP still executes under the gate. This does not
        weaken global STOP, E-STOP, deadlines, or stale-sequence checks.
        """
        if (not isinstance(body, dict) or set(body) != {"action", "session", "sequence"} or
                not isinstance(body.get("action"), str) or body["action"] not in {"stop", "end"} or
                not isinstance(body.get("session"), str) or
                type(body.get("sequence")) is not int or body["sequence"] <= 0):
            return False
        token, sequence = body["session"], body["sequence"]
        with self._stop_lock:
            # Session/sequence updates also use this short lock. Never wait
            # behind the motion command that STOP is preempting.
            if token != self._token or sequence <= self._sequence:
                return False
            old = self._stop_fence
            if old is None or old[0] != token or sequence > old[1]:
                self._stop_fence = (token, sequence, True)
                return True
        return False

    def finish_preempted_stop(self, body):
        if not isinstance(body, dict):
            return
        with self._stop_lock:
            fence = self._stop_fence
            if fence and (body.get("session"), body.get("sequence")) == fence[:2]:
                self._stop_fence = (fence[0], fence[1], False)

    def _preempted_locked(self, body):
        fence = self._stop_fence
        return bool(fence and body["session"] == fence[0] and
                    (fence[2] or body["sequence"] <= fence[1]))

    def is_preempted(self, body):
        if (not isinstance(body, dict) or not isinstance(body.get("action"), str) or
                body["action"] in {"stop", "end", "begin"} or not isinstance(body.get("session"), str) or
                type(body.get("sequence")) is not int):
            return False
        with self._stop_lock:
            return self._preempted_locked(body)

    def _fail(self, reason, status=409, *, body=None):
        if self.is_preempted(body):
            return {"ok": False, "reason": "stale_command"}, 409
        self._clear()
        self._brake()
        return {"ok": False, "reason": reason}, status

    @staticmethod
    def _number(value):
        return (isinstance(value, (int, float)) and not isinstance(value, bool)
                and math.isfinite(value))

    @staticmethod
    def _pattern_for(body, keys):
        """Keep the existing keyboard protocol unchanged for ordinary driving."""
        if set(body) == keys:
            return "drive"
        if set(body) == keys | {"pattern"} and body.get("pattern") == "spin":
            return "spin"
        return None

    def _geometry(self, source):
        """One authoritative snapshot, shared with the odometry manoeuvres."""
        if getattr(source, "supports_individual_drive", False) is not True:
            return None
        try:
            config = source.maneuver_config()
            if not isinstance(config, dict) or config.get("supported") is not True:
                return None
            chassis = config.get("chassis")
            if not isinstance(chassis, dict):
                return None
            dimensions = {}
            for key in ("wheelbaseMm", "trackMm"):
                value = chassis.get(key)
                low, high = maneuvers.LIMITS[key]
                if not self._number(value) or not low <= value <= high:
                    return None
                dimensions[key] = float(value)
            angle = maneuvers.pivot_angle_deg(dimensions)
            angles, _ = maneuvers.steering_targets("spin_left", dimensions)
            limit = getattr(source, "MAX_STEER_DEG", None)
            if not self._number(limit) or any(abs(value) > limit for value in angles.values()):
                return None
            return {**dimensions, "angleDeg": angle, "angles": angles}
        except Exception:  # noqa: BLE001 — never invent a geometry on failure
            return None

    def _spin_error(self, angles):
        if self._spin_geometry is None:
            return "spin_geometry_required"
        try:
            source = self._source()
        except Exception:  # noqa: BLE001 — a lost owner must stop the spin
            return "spin_geometry_changed"
        if source is not self._spin_source or self._geometry(source) != self._spin_geometry:
            return "spin_geometry_changed"
        if angles != self._spin_geometry["angles"]:
            return "invalid_angle"
        return None

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
            if (not self._number(rpm) or rpm < 0 or
                    (type(motor.get("firmware")) is int and motor["firmware"] >= 19 and
                     type(motor.get("scRecent")) is not bool)):
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
        if any(motors[m]["scRpm"] > 1.0 or motors[m].get("scRecent") is True for m in MODULES):
            return "wheel_moving"
        return None

    def _target_ready(self, snapshot, angles, first, tolerance=None):
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
        tolerance = self.ANGLE_TOLERANCE_DEG if tolerance is None else tolerance
        for module in MODULES:
            motor = motors[module]
            if motor.get("moving") or abs(motor["angle"] - angles[module]) > tolerance:
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

    def _live_progress(self, snapshot, now):
        """Confirm each board has applied recent heartbeats and reaches targets."""
        drive, motors = snapshot
        matching = all(motors[m].get("liveId") == self._brake_id for m in MODULES)
        if matching:
            confirmed = min(motors[m]["liveSeq"] for m in MODULES)
            self._live_pending = [(seq, sent) for seq, sent in self._live_pending if seq > confirmed]
            if confirmed >= self._live_latest_seq:
                if (not drive.get("active") or any(motors[m]["drivePercent"] != self._wheel_percents[m] for m in MODULES)):
                    return "live_unconfirmed"
                tolerance = (self.SPIN_ANGLE_TOLERANCE_DEG if self._pattern == "spin"
                             else self.ANGLE_TOLERANCE_DEG)
                arrived = all(not motors[m].get("moving") and
                              abs(motors[m]["angle"] - self._angles[m]) <= tolerance
                              for m in MODULES)
                if arrived:
                    self._live_angles_since = None
                elif self._live_angles_since is None:
                    return "steering_changed"
                elif now - self._live_angles_since > 0.3 and any(
                        not motors[m].get("moving") and
                        abs(motors[m]["angle"] - self._angles[m]) > tolerance
                        for m in MODULES):
                    return "steering_changed"
        if self._live_pending and now - self._live_pending[0][1] > self.LIVE_ACK_TTL:
            return "live_unconfirmed"
        if self._live_angles_since is not None and now - self._live_angles_since > self.LIVE_STEER_TTL:
            return "steering_changed"
        return None

    def _handle_live(self, body, snapshot, now, deadline):
        pattern = self._pattern_for(body, {"action", "session", "sequence", "percent", "angles", "permit"})
        if pattern is None:
            return self._fail("payload", 400, body=body)
        percent, angles, sequence = body.get("percent"), body.get("angles"), body["sequence"]
        if (type(percent) is not int or not 1 <= abs(percent) <= 100 or
                not isinstance(angles, dict) or set(angles) != set(MODULES) or
                not all(self._number(angles[m]) and abs(angles[m]) <= 100 for m in MODULES) or
                sequence >= 0x7FFFFFFF):
            return self._fail("invalid_drive", 400, body=body)
        if abs(percent) > self.max_percent:
            # Браузер уже ограничивает ползунок, но предел обязан жить на
            # борту: устаревшая вкладка или ручной запрос не должны дать больше.
            return self._fail("speed_limit", 400, body=body)
        if pattern == "spin":
            error = self._spin_error(angles)
            if error:
                return self._fail(error, body=body)
        drive, motors = snapshot
        if any(type(motors[m].get("firmware")) is not int or motors[m]["firmware"] < 20 or
               type(motors[m].get("liveId")) is not int or not 0 <= motors[m]["liveId"] < 0x7FFFFFFF or
               type(motors[m].get("liveSeq")) is not int or not 0 <= motors[m]["liveSeq"] < 0x7FFFFFFF
               for m in MODULES):
            return self._fail("live_firmware_required", body=body)
        direction = 1 if percent > 0 else -1
        first = not self._running
        if first:
            permit = self._permit
            if (not permit or not isinstance(body.get("permit"), str) or
                    body["permit"] != permit["value"] or now > permit["expires"] or
                    direction != permit["direction"] or angles != permit["angles"] or
                    pattern != permit["pattern"]):
                return self._fail("keyboard_expired", body=body)
            tolerance = self.SPIN_ANGLE_TOLERANCE_DEG if pattern == "spin" else None
            error = self._target_ready(snapshot, angles, first=True, tolerance=tolerance)
        else:
            if not self._live or pattern != self._pattern or direction != self._direction:
                return self._fail("module_busy", body=body)
            if (pattern == "spin" and
                    (drive.get("commandedPercent") != 0 or drive.get("commandedModules") != self._wheel_percents)):
                return self._fail("manual_override", body=body)
            if pattern == "drive" and drive.get("commandedPercent") != self._percent:
                return self._fail("manual_override", body=body)
            error = self._live_progress(snapshot, now)
        if not error:
            error = self._sensor_progress(motors, now, first=first)
        if error:
            return self._fail(error, body=body)
        sender = getattr(self._source(), "keyboard_live", None)
        if not callable(sender):
            return self._fail("live_firmware_required", body=body)
        # No delayed UART heartbeat may outlive its HTTP command/lease.
        deadline = min(deadline if deadline is not None else time.monotonic() + 0.5,
                       time.monotonic() + 0.5)
        if deadline <= time.monotonic():
            return self._fail("command_expired", 408, body=body)
        if self.is_preempted(body):
            return {"ok": False, "reason": "stale_command"}, 409
        # Left/right arrows use opposite wheel directions. Negative percent
        # means counter-clockwise in the model, matching manoeuvres.steering_targets.
        # Physical directions still need checking with the wheels off the floor.
        wheel_percents = ({m: percent if m in ("FL", "RL") else -percent for m in MODULES}
                          if pattern == "spin" else {m: percent for m in MODULES})
        try:
            result = sender(self._brake_id, sequence,
                            wheel_percents if pattern == "spin" else percent, angles, deadline)
        except Exception:  # noqa: BLE001 — stop on broken transport
            return self._fail("command_failed", 503, body=body)
        if not result.get("ok"):
            return self._fail(result.get("reason", "command_failed"), 503, body=body)
        if not first and angles != self._angles:
            self._live_angles_since = now
        self._running = self._live = True
        self._permit = None
        self._direction, self._percent = direction, percent
        self._pattern, self._wheel_percents = pattern, wheel_percents
        self._angles = dict(angles)
        self._live_latest_seq = sequence
        self._live_pending.append((sequence, now))
        self._last_drive = now
        return result, 200

    def handle(self, body, deadline=None):
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
                self._spin_source = source
                self._spin_geometry = self._geometry(source)
                with self._stop_lock:
                    self._token = secrets.token_urlsafe(24)
                    self._sequence = 0
                self._last_activity = now
                # The browser can reuse this freshly issued STOP instead of
                # sending another one on the first arrow press. It must still
                # wait for all four matching brake ACKs and request a permit.
                spin = self._spin_geometry
                if spin is not None:
                    spin = {**spin, "angles": dict(spin["angles"])}
                # ``maxPercent`` — предел тяги WASD с борта: браузер ставит по
                # нему максимум ползунка, а не угадывает его в разметке.
                return {"ok": True, "session": self._token, "brakeId": self._brake_id,
                        "spin": spin, "maxPercent": self.max_percent}, 200

            if action not in {"stop", "end", "ping", "steer", "steer_all", "arm", "drive", "live"}:
                return {"ok": False, "reason": "invalid_action"}, 400
            if not isinstance(body.get("session"), str) or body["session"] != self._token:
                return {"ok": False, "reason": "keyboard_session"}, 409
            sequence = body.get("sequence")
            if type(sequence) is not int or sequence <= 0:
                return {"ok": False, "reason": "payload"}, 400
            # Accepting a new sequence and registering a STOP fence must be
            # atomic; otherwise a newer packet can overtake the final STOP.
            with self._stop_lock:
                if (sequence <= self._sequence or
                        (action not in {"stop", "end"} and self._preempted_locked(body))):
                    return {"ok": False, "reason": "stale_command"}, 409
                self._sequence = sequence

            if action in {"stop", "end"}:
                if set(body) != {"action", "session", "sequence"}:
                    return self._fail("payload", 400, body=body)
                self._running = False
                self._angles = None
                self._permit = None
                self._wheels = {}
                self._direction = self._percent = 0
                self._live = False
                self._live_pending = []
                self._live_latest_seq = 0
                self._live_angles_since = None
                self._pattern = None
                self._wheel_percents = {}
                self._last_activity = now
                result, status = self._brake()
                if action == "end" or not result.get("ok"):
                    self._clear()
                return result, status

            if self._estop():
                return self._fail("estop", body=body)
            if now - self._last_activity > self.IDLE_TTL or (self._running and now - self._last_drive > self.DRIVE_TTL):
                return self._fail("keyboard_expired", body=body)
            self._last_activity = now
            if action == "ping":
                if set(body) != {"action", "session", "sequence"}:
                    return self._fail("payload", 400, body=body)
                return {"ok": True}, 200

            snapshot, error = self._snapshot()
            if error:
                return self._fail(error, body=body)
            if self.is_preempted(body):
                return {"ok": False, "reason": "stale_command"}, 409
            if action == "steer_all":
                pattern = self._pattern_for(body, {"action", "session", "sequence", "angles"})
                if pattern is None:
                    return self._fail("payload", 400, body=body)
                angles = body.get("angles")
                if (not isinstance(angles, dict) or set(angles) != set(MODULES) or
                        not all(self._number(angles[m]) and abs(angles[m]) <= 100 for m in MODULES)):
                    return self._fail("invalid_angle", 400, body=body)
                if pattern == "spin":
                    error = self._spin_error(angles)
                    if error:
                        return self._fail(error, body=body)
                if self._running or any(snapshot[1][m].get("moving") for m in MODULES):
                    return self._fail("module_busy", body=body)
                self._permit = None
                error = self._steering_ready(snapshot)
                if error == "wheel_moving":
                    # A faster browser can observe an earlier zero before the
                    # next SC sample reports coasting. Stay braked and allow a
                    # fresh sequenced retry; never issue a partial steer here.
                    return {"ok": False, "reason": error, "retryable": True}, 409
                if error:
                    return self._fail(error, body=body)
                sent = []
                # One sequenced request avoids four serial network round-trips;
                # each wheel is still checked and independently acknowledged.
                for module in MODULES:
                    if self.is_preempted(body):
                        return {"ok": False, "reason": "stale_command"}, 409
                    tolerance = self.SPIN_ANGLE_TOLERANCE_DEG if pattern == "spin" else 1.0
                    if abs(snapshot[1][module]["angle"] - angles[module]) <= tolerance:
                        continue
                    result, status = self._steer(self._source(), {"module": module, "angle": angles[module]})
                    if not result.get("ok"):
                        return self._fail(result.get("reason", "command_failed"), status, body=body)
                    sent.append(module)
                return {"ok": True, "sent": sent}, 200

            if action == "steer":
                if set(body) != {"action", "session", "sequence", "module", "angle"}:
                    return self._fail("payload", 400, body=body)
                module, angle = body.get("module"), body.get("angle")
                if module not in MODULES or not self._number(angle) or abs(angle) > 100:
                    return self._fail("invalid_angle", 400, body=body)
                if self._running:
                    return self._fail("module_busy", body=body)
                self._permit = None
                error = self._steering_ready(snapshot)
                if error:
                    return self._fail(error, body=body)
                if self.is_preempted(body):
                    return {"ok": False, "reason": "stale_command"}, 409
                result, status = self._steer(self._source(), {"module": module, "angle": angle})
                if not result.get("ok"):
                    return self._fail(result.get("reason", "command_failed"), status, body=body)
                return result, status

            if action == "arm":
                pattern = self._pattern_for(body, {"action", "session", "sequence", "direction", "angles"})
                if pattern is None:
                    return self._fail("payload", 400, body=body)
                angles, direction = body.get("angles"), body.get("direction")
                if (self._running or type(direction) is not int or direction not in (-1, 1) or
                        not isinstance(angles, dict) or set(angles) != set(MODULES) or
                        not all(self._number(angles[m]) and abs(angles[m]) <= 100 for m in MODULES)):
                    return self._fail("invalid_angle", 400, body=body)
                if pattern == "spin":
                    error = self._spin_error(angles)
                    if error:
                        return self._fail(error, body=body)
                tolerance = self.SPIN_ANGLE_TOLERANCE_DEG if pattern == "spin" else None
                error = self._target_ready(snapshot, angles, first=True, tolerance=tolerance)
                if error == "wheel_moving":
                    self._permit = None
                    return {"ok": False, "reason": error, "retryable": True}, 409
                if error:
                    return self._fail(error, body=body)
                if self.is_preempted(body):
                    return {"ok": False, "reason": "stale_command"}, 409
                nonce = secrets.token_urlsafe(18)
                self._permit = {"value": nonce, "expires": now + self.START_PERMIT_TTL,
                                "angles": dict(angles), "direction": direction, "pattern": pattern}
                return {"ok": True, "permit": nonce}, 200

            if action == "live":
                return self._handle_live(body, snapshot, now, deadline)
            if self._live:
                return self._fail("module_busy", body=body)
            if set(body) != {"action", "session", "sequence", "percent", "angles", "permit"}:
                return self._fail("payload", 400, body=body)
            percent, angles = body.get("percent"), body.get("angles")
            if type(percent) is not int or not 1 <= abs(percent) <= 100 or not isinstance(angles, dict) or set(angles) != set(MODULES):
                return self._fail("invalid_drive", 400, body=body)
            if abs(percent) > self.max_percent:
                return self._fail("speed_limit", 400, body=body)
            if not all(self._number(angles[m]) and abs(angles[m]) <= 100 for m in MODULES):
                return self._fail("invalid_angle", 400, body=body)
            direction = 1 if percent > 0 else -1
            if self._running and (direction != self._direction or angles != self._angles):
                return self._fail("module_busy", body=body)
            if not self._running:
                permit = self._permit
                if (not permit or not isinstance(body.get("permit"), str) or
                        body["permit"] != permit["value"] or now > permit["expires"] or
                        direction != permit["direction"] or angles != permit["angles"] or
                        permit["pattern"] != "drive"):
                    return self._fail("keyboard_expired", body=body)
            error = self._target_ready(snapshot, angles, first=not self._running)
            if not error:
                error = self._sensor_progress(snapshot[1], now, first=not self._running)
            if error:
                return self._fail(error, body=body)
            if self.is_preempted(body):
                return {"ok": False, "reason": "stale_command"}, 409
            result, status = self._drive(self._source(), {"percent": percent})
            if not result.get("ok"):
                return self._fail(result.get("reason", "command_failed"), status, body=body)
            self._running = True
            self._permit = None
            self._direction = direction
            self._percent = percent
            self._angles = dict(angles)
            self._last_drive = now
            return result, status
