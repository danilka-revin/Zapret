"""Fail-closed brake recovery for wheels running legacy firmware.

The application no longer starts or maintains a freewheel session. A board
with older firmware can still report an orphaned neutral/neutral-lock after a
restart, so motion stays inhibited until a *new* tagged brake is acknowledged
by all four boards. Telemetry describes commanded pins, not mechanical grip.
"""
from __future__ import annotations

import math
import threading
import time

MODULES = ("FL", "FR", "RL", "RR")


class NeutralControl:
    TELEMETRY_AGE_S = 0.60
    STAGE_TIMEOUT_S = 2.0

    def __init__(self, source, clock=time.monotonic, interrupt_keyboard=None):
        self._source = source
        self._clock = clock
        self._interrupt_keyboard = interrupt_keyboard or (lambda: None)
        self._lock = threading.RLock()
        self._phase = "idle"
        # Read before the HTTP motion gate. Commands arriving during recovery
        # must not execute after it completes, even if they were waiting on it.
        self._motion_ticket = (0, False)
        self._reason = ""
        self._brake_id = 0
        self._stage_at = 0.0
        self._owner = None

    @staticmethod
    def _number(value):
        return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)

    def _snapshot(self):
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
        if (drive.get("active") or drive.get("commandedPercent") not in (None, 0) or
                any(value not in (None, 0) for value in (drive.get("commandedModules") or {}).values()) or
                (drive.get("maneuver") or {}).get("active")):
            return None, "module_busy"
        return motors, None

    def _fault(self, reason):
        # Do not unlock: a fresh brake must be requested and confirmed.
        self._phase, self._reason = "fault", reason
        try:
            self._owner._brake_all()
        except Exception:  # noqa: BLE001 — hardware watchdog is the final backstop
            pass

    def _advance(self):
        now = self._clock()
        if self._phase == "idle":
            # Older wheel firmware can still be in neutral after a server
            # restart or report its retained lock after its own watchdog.
            source = self._source()
            observed = getattr(source, "observed_neutral", None)
            if callable(observed) and observed():
                self._owner = source
                self._motion_ticket = (self._motion_ticket[0] + 1, True)
                self._phase, self._reason = "fault", "neutral_unconfirmed"
                try:
                    source.inhibit_neutral_fault()
                    # An old WASD lease must not resume after recovery.
                    self._interrupt_keyboard()
                except Exception:  # noqa: BLE001
                    self._fault("command_failed")
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
        return self._motion_ticket

    def status(self):
        with self._lock:
            self._advance()
            # Retain the response shape for older clients; no new freewheel
            # session exists, hence active is always false in this server.
            return {"phase": self._phase, "locked": self._phase != "idle",
                    "active": False, "reason": self._reason}

    def interrupt(self, reason="stopped"):
        """STOP/E-STOP stay available and invalidate an unfinished recovery."""
        with self._lock:
            if self._phase != "idle":
                self._fault(reason)

    def restore(self):
        """Queue a fresh brake; release the interlock only after four ACKs."""
        with self._lock:
            # A restore request may arrive before the first state poll.
            self._advance()
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
