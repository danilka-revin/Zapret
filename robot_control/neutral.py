"""Manual freewheel lease, owned by the process that owns the wheel ports.

The browser must keep the lease alive. Each board independently applies its
650 ms watchdog and will not accept an old neutral heartbeat after braking.
Motion remains inhibited on the server until a *new* brake is acknowledged
by all four boards. Telemetry describes commanded pins, not mechanical grip.
"""
from __future__ import annotations

import math
import secrets
import threading
import time

MODULES = ("FL", "FR", "RL", "RR")


class NeutralControl:
    LEASE_S = 0.60
    TELEMETRY_AGE_S = 0.60
    STAGE_TIMEOUT_S = 2.0
    ENGAGE_TIMEOUT_S = 1.0

    def __init__(self, source, estop, navigation_active, clock=time.monotonic, interrupt_keyboard=None):
        self._source = source
        self._estop = estop
        self._navigation_active = navigation_active
        self._clock = clock
        self._interrupt_keyboard = interrupt_keyboard or (lambda: None)
        self._lock = threading.RLock()
        self._phase = "idle"
        # A single immutable value makes arrival checks atomic even when a
        # request waits while neutral is engaged and restored in another thread.
        self._motion_ticket = (0, False)
        self._reason = ""
        self._token = None
        self._sequence = 0
        self._brake_id = 0
        self._last = 0.0
        self._stage_at = 0.0
        self._owner = None

    @staticmethod
    def _number(value):
        return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)

    def _snapshot(self, stopped=False):
        try:
            source = self._owner or self._source()
            snap = source.read()
        except Exception:  # noqa: BLE001 — communication failure must fail closed
            return None, "module_offline"
        if not isinstance(snap, dict) or snap.get("linkOk") is False:
            return None, "module_offline"
        drive = snap.get("drive") or {}
        if not drive.get("available"):
            return None, "serial_unavailable"
        motors = {item.get("id"): item for item in snap.get("motors", []) if isinstance(item, dict)}
        for module in MODULES:
            motor = motors.get(module) or {}
            age = motor.get("telemetryAgeS")
            if motor.get("online") is not True or not self._number(age) or not 0 <= age < self.TELEMETRY_AGE_S:
                return None, "module_offline"
            if (type(motor.get("firmware")) is not int or motor["firmware"] < 18 or
                    type(motor.get("neutralId")) is not int or type(motor.get("brakeId")) is not int or
                    type(motor.get("neutralLocked")) is not bool):
                return None, "neutral_firmware_required"
            rpm = motor.get("scRpm")
            if (not self._number(rpm) or rpm < 0 or
                    (motor["firmware"] >= 19 and type(motor.get("scRecent")) is not bool)):
                return None, "sc_invalid"
            if motor.get("moving") or motor.get("calibrating") or motor.get("cycling"):
                return None, "module_busy"
            if type(motor.get("drivePercent")) is not int or motor["drivePercent"] != 0:
                return None, "module_busy"
            if stopped and (rpm > 1.0 or motor.get("scRecent") is True):
                return None, "wheel_moving"
        if (drive.get("active") or drive.get("commandedPercent") not in (None, 0) or
                any(value not in (None, 0) for value in (drive.get("commandedModules") or {}).values()) or
                (drive.get("maneuver") or {}).get("active")):
            return None, "module_busy"
        return motors, None

    def _fault(self, reason):
        # Do not unlock: the operator must request and confirm a fresh brake.
        self._token = None
        self._phase = "fault"
        self._reason = reason
        try:
            self._owner._brake_all()
        except Exception:  # noqa: BLE001 — hardware watchdog is the final backstop
            pass

    def _advance(self):
        now = self._clock()
        if self._phase == "idle":
            # If this process restarted while a board was in neutral, do not
            # report a confirmed brake or permit movement on stale telemetry.
            source = self._source()
            observed = getattr(source, "observed_neutral", None)
            if callable(observed) and observed():
                self._owner = source
                self._motion_ticket = (self._motion_ticket[0] + 1, True)
                self._phase, self._reason = "fault", "neutral_unconfirmed"
                try:
                    source.inhibit_neutral_fault()
                except Exception:  # noqa: BLE001
                    self._fault("command_failed")
        if self._phase in {"braking", "engaging", "neutral"}:
            if now - self._last > self.LEASE_S:
                self._fault("neutral_expired")
            elif self._estop():
                self._fault("estop")
            elif ((self._phase == "braking" and now - self._stage_at > self.STAGE_TIMEOUT_S) or
                  (self._phase == "engaging" and now - self._stage_at > self.ENGAGE_TIMEOUT_S)):
                self._fault("neutral_unconfirmed")
        elif self._phase == "restoring":
            motors, error = self._snapshot()
            if not error and all(motors[m]["brakeId"] == self._brake_id and motors[m]["neutralId"] == 0 and
                                 motors[m]["neutralLocked"] is False for m in MODULES):
                try:
                    self._owner.finish_neutral()
                except Exception:  # noqa: BLE001
                    self._phase, self._reason = "fault", "command_failed"
                    return
                self._phase, self._reason, self._owner = "idle", "", None
                self._motion_ticket = (self._motion_ticket[0], False)
                self._brake_id = 0
            elif now - self._stage_at > self.STAGE_TIMEOUT_S:
                self._phase, self._reason = "fault", error or "brake_unconfirmed"

    def ticket(self):
        # Read before waiting for the HTTP motion gate. A request that arrived
        # during (or before) a neutral transition must not execute afterwards.
        return self._motion_ticket

    def status(self):
        with self._lock:
            self._advance()
            return {"phase": self._phase, "locked": self._phase != "idle",
                    "active": self._phase == "neutral", "reason": self._reason}

    def begin(self, deadline=None):
        with self._lock:
            self._advance()
            if self._phase != "idle":
                return {"ok": False, "reason": "neutral_locked"}, 409
            if self._estop():
                return {"ok": False, "reason": "estop"}, 409
            if self._navigation_active():
                return {"ok": False, "reason": "navigation_active"}, 409
            source = self._source()
            if not all(callable(getattr(source, name, None)) for name in
                       ("reserve_neutral", "neutral_heartbeat", "finish_neutral", "brake_for_control")):
                return {"ok": False, "reason": "control_not_supported"}, 409
            self._owner = source
            motors, error = self._snapshot(stopped=True)
            if not error and any(motors[m]["neutralId"] != 0 or motors[m]["neutralLocked"] for m in MODULES):
                error = "neutral_unconfirmed"
            if error:
                self._owner = None
                return {"ok": False, "reason": error}, 409
            # An idle WASD lease must not later wake up after neutral is engaged.
            self._interrupt_keyboard()
            if deadline is not None and self._clock() >= deadline:
                self._owner = None
                return {"ok": False, "reason": "command_expired"}, 408
            self._motion_ticket = (self._motion_ticket[0] + 1, True)
            try:
                result = source.reserve_neutral()
            except Exception:  # noqa: BLE001 — reservation may already have happened
                self._fault("command_failed")
                return {"ok": False, "reason": "command_failed"}, 503
            brake_id = result.get("brakeId")
            if not result.get("ok"):
                self._owner = None
                self._motion_ticket = (self._motion_ticket[0], False)
                return {"ok": False, "reason": result.get("reason", "command_failed")}, 409
            if type(brake_id) is not int or not 0 < brake_id < 0x7FFFFFFF:
                self._fault("command_failed")
                return {"ok": False, "reason": "command_failed"}, 503
            self._token = secrets.token_urlsafe(24)
            self._sequence = 0
            self._brake_id = brake_id
            self._last = self._stage_at = self._clock()
            self._phase, self._reason = "braking", ""
            return {"ok": True, "session": self._token, "phase": self._phase}, 200

    def heartbeat(self, body, deadline=None):
        with self._lock:
            self._advance()
            if not isinstance(body, dict) or set(body) != {"action", "session", "sequence"}:
                return {"ok": False, "reason": "payload"}, 400
            if not self._token or body.get("session") != self._token:
                return {"ok": False, "reason": "neutral_session"}, 409
            sequence = body.get("sequence")
            if type(sequence) is not int or sequence <= 0:
                return {"ok": False, "reason": "payload"}, 400
            if sequence <= self._sequence:
                return {"ok": False, "reason": "stale_command"}, 409
            self._sequence = sequence
            received = self._clock()
            expiry = min(deadline if deadline is not None else received + 0.5, received + 0.5)
            previous_expiry = self._last + self.LEASE_S
            motors, error = self._snapshot(stopped=self._phase == "braking")
            if error:
                self._fault(error)
                return {"ok": False, "reason": error}, 409
            # A stalled snapshot or queued HTTP request may not revive neutral.
            if self._clock() >= min(expiry, previous_expiry):
                self._fault("command_expired")
                return {"ok": False, "reason": "command_expired"}, 408
            self._last = received
            if self._phase == "braking":
                if all(motors[m]["brakeId"] == self._brake_id and motors[m]["neutralId"] == 0 and
                       motors[m]["neutralLocked"] is False for m in MODULES):
                    self._phase, self._stage_at = "engaging", self._clock()
                else:
                    return {"ok": True, "phase": self._phase}, 200
            if self._phase == "engaging":
                if all(motors[m]["neutralId"] == self._brake_id and motors[m]["neutralLocked"] is True for m in MODULES):
                    self._phase = "neutral"
                elif any(motors[m]["neutralId"] not in (0, self._brake_id) or
                         (motors[m]["neutralId"] == self._brake_id and motors[m]["neutralLocked"] is not True) or
                         (motors[m]["neutralId"] == 0 and
                          (motors[m]["brakeId"] != self._brake_id or motors[m]["neutralLocked"] is not False))
                         for m in MODULES):
                    self._fault("neutral_unconfirmed")
                    return {"ok": False, "reason": "neutral_unconfirmed"}, 409
            elif self._phase == "neutral" and any(motors[m]["neutralId"] != self._brake_id or
                                                    motors[m]["neutralLocked"] is not True for m in MODULES):
                self._fault("neutral_unconfirmed")
                return {"ok": False, "reason": "neutral_unconfirmed"}, 409
            try:
                result = self._owner.neutral_heartbeat(self._brake_id, expiry)
            except Exception:  # noqa: BLE001
                result = {"ok": False, "reason": "command_failed"}
            if not result.get("ok"):
                error = result.get("reason", "command_failed")
                self._fault(error)
                return {"ok": False, "reason": error}, 409
            return {"ok": True, "phase": self._phase}, 200

    def interrupt(self, reason="stopped"):
        """STOP/E-STOP immediately revoke the lease and retain the interlock."""
        with self._lock:
            if self._phase != "idle":
                self._fault(reason)

    def restore(self):
        """Queue a fresh brake; release the interlock only after four ACKs."""
        with self._lock:
            self._token = None
            if self._phase == "idle":
                return {"ok": True, "phase": "idle"}, 200
            try:
                result = self._owner.brake_for_control()
            except Exception:  # noqa: BLE001
                result = {"ok": False, "reason": "command_failed"}
            brake_id = result.get("brakeId")
            if not result.get("ok") or type(brake_id) is not int or not 0 < brake_id < 0x7FFFFFFF:
                self._phase, self._reason = "fault", result.get("reason", "command_failed")
                return {"ok": False, "reason": self._reason}, 409
            self._brake_id = brake_id
            self._phase, self._reason, self._stage_at = "restoring", "", self._clock()
            return {"ok": True, "phase": self._phase}, 200
