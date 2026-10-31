"""Guarded optical calibration and per-wheel, durable full-zero alignment.

The USB owner reserves all motion paths; the wizard never declares a save from
an enqueued command alone. Each fw23 board checks its own optical reference,
movement, limit and EEPROM read-back, then returns a role/CRC/token ACK.
A durable journal inhibits driving after an interrupted four-board commit.
"""
from __future__ import annotations

import json
import math
import os
import secrets
import tempfile
import threading
import time
from pathlib import Path

MODULES = ("FL", "FR", "RL", "RR")
MAX_OFFSET_DEG = 20.0


def pending_path() -> Path:
    configured = os.environ.get("RC_FULL_ZERO_PENDING", "").strip()
    return Path(configured) if configured else Path(__file__).resolve().parent.parent / "data" / "full-zero.pending"


def _sync_directory(path: Path) -> None:
    fd = os.open(str(path), os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _make_directory_durable(directory: Path) -> None:
    # Persist each newly-created directory entry, not just the leaf. Otherwise
    # a custom journal in several new directories can vanish after power loss.
    missing = []
    current = directory
    while not current.exists():
        missing.append(current)
        parent = current.parent
        if parent == current:
            raise OSError("journal directory has no existing ancestor")
        current = parent
    for item in reversed(missing):
        item.mkdir(exist_ok=True)
        _sync_directory(item.parent)


def write_pending(path: Path) -> None:
    """Durably record intent *before* sending the first EEPROM commit."""
    path = Path(path)
    _make_directory_durable(path.parent)
    name = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=".full-zero-", delete=False) as handle:
            name = handle.name
            json.dump({"version": 1, "reason": "full_zero_commit_pending"}, handle)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(name, path)
        name = None
        _sync_directory(path.parent)
    finally:
        if name is not None:
            try:
                os.unlink(name)
            except FileNotFoundError:
                pass


def clear_pending(path: Path) -> None:
    path = Path(path)
    try:
        path.unlink()
    except FileNotFoundError:
        # Not having a journal is only legitimate on a fully confirmed commit;
        # still sync the directory if one exists.
        pass
    if path.parent.exists():
        _sync_directory(path.parent)


class FullZeroControl:
    """One lease-backed operator session, fail-closed across all motion paths."""

    LEASE_S = 8.0
    TELEMETRY_AGE_S = 0.75
    BRAKE_TIMEOUT_S = 5.0
    CAL_START_TIMEOUT_S = 15.0
    CAL_TIMEOUT_S = 125.0
    MOVE_TIMEOUT_S = 10.0
    REPLY_TIMEOUT_S = 5.0

    def __init__(self, source, estop, neutral, navigation_active=lambda: False,
                 clock=time.monotonic):
        self._source = source
        self._estop = estop
        self._neutral = neutral
        self._navigation_active = navigation_active
        self._clock = clock
        self._lock = threading.RLock()
        self._owner = None
        self._token = None
        self._phase = "idle"
        self._reason = ""
        self._brake_id = 0
        self._stage_at = 0.0
        self._last_contact = 0.0
        self._seen = set()
        self._pending_move = None
        self._move_query_at = 0.0
        # A random initial sequence avoids reusing an ACK from a previous
        # process; subsequent IDs never repeat during an ordinary session.
        self._request_id = secrets.randbelow(0x7FFFFFFE) + 1
        self._expected = {}
        self._last_acks = {}
        self._stop_attempt = 0.0
        self._watcher = None
        self._motion_ticket = (0, False)

    def ticket(self):
        # Snapshot before waiting on the HTTP gate, like the neutral lease.
        source = self._source()
        return (self._motion_ticket[0], self._motion_ticket[1] or
                bool(getattr(source, "_full_zero_inhibit", False)))

    def _next_request_id(self):
        self._request_id = self._request_id % 0x7FFFFFFE + 1
        return self._request_id

    @staticmethod
    def _number(value):
        return type(value) in (int, float) and math.isfinite(value)

    def _snapshot(self, *, allow_motion=False, require_home=False):
        source = self._source()
        if source is not self._owner:
            return None, "source_changed"
        try:
            snap = source.read()
        except Exception:  # noqa: BLE001 — fail closed on communication error
            return None, "module_offline"
        if not isinstance(snap, dict) or not (snap.get("drive") or {}).get("available"):
            return None, "serial_unavailable"
        entries = snap.get("motors")
        if not isinstance(entries, list) or len(entries) != len(MODULES):
            return None, "module_offline"
        motors = {m.get("id"): m for m in entries if isinstance(m, dict)}
        if set(motors) != set(MODULES):
            return None, "module_offline"
        for m in MODULES:
            motor = motors[m]
            age = motor.get("telemetryAgeS")
            if motor.get("online") is not True or not self._number(age) or not 0 <= age < self.TELEMETRY_AGE_S:
                return None, "module_offline"
            if type(motor.get("firmware")) is not int or motor["firmware"] < 23:
                return None, "firmware_update_required"
            if type(motor.get("fullZeroFault")) is not bool or motor["fullZeroFault"]:
                return None, "eeprom_error"
            if (type(motor.get("neutralLocked")) is not bool or motor["neutralLocked"] or
                    type(motor.get("neutralId")) is not int or motor["neutralId"] != 0):
                return None, "neutral_locked"
            if type(motor.get("brakeId")) is not int:
                return None, "brake_unconfirmed"
            if type(motor.get("drivePercent")) is not int or motor["drivePercent"] != 0:
                return None, "module_busy"
            rpm = motor.get("scRpm")
            if not self._number(rpm) or rpm < 0 or type(motor.get("scRecent")) is not bool:
                return None, "sc_invalid"
            if rpm > 1.0 or motor["scRecent"]:
                return None, "wheel_moving"
            if (any(type(motor.get(flag)) is not bool for flag in ("moving", "calibrating", "cycling", "homed")) or
                    type(motor.get("calibError")) is not int or not 0 <= motor["calibError"] <= 15):
                return None, "invalid_telemetry"
            if motor["cycling"] or (not allow_motion and (motor["moving"] or motor["calibrating"])):
                return None, "module_busy"
            if require_home and (motor["homed"] is not True or motor.get("homeSource") != "opto" or
                                 motor["calibError"] != 0):
                return None, "optical_zero_required"
            if not self._number(motor.get("angle")) or not self._number(motor.get("targetAngle")):
                return None, "invalid_angle"
        drive = snap.get("drive") or {}
        if (drive.get("active") or drive.get("commandedPercent") not in (None, 0) or
                any(v not in (None, 0) for v in (drive.get("commandedModules") or {}).values()) or
                (drive.get("maneuver") or {}).get("active")):
            return None, "module_busy"
        return motors, None

    def _respond(self, *, ok=True, include_token=False):
        motors = {}
        if self._owner is not None:
            try:
                snap = self._owner.read()
                for motor in snap.get("motors", []):
                    if motor.get("id") in MODULES:
                        motors[motor["id"]] = {k: motor.get(k) for k in
                            ("online", "angle", "targetAngle", "moving", "calibrating", "homed",
                             "homeSource", "calibError", "firmware")}
            except Exception:  # noqa: BLE001
                pass
        response = {"ok": ok, "phase": self._phase, "reason": self._reason or None,
                    "motors": motors, "calibratingSeen": sorted(self._seen),
                    "acknowledged": sorted(m for m, ack in self._last_acks.items()
                                           if ack.get("status") == 2),
                    "pendingRecovery": bool(getattr(self._owner or self._source(), "_full_zero_pending", False)),
                    "maxOffsetDeg": MAX_OFFSET_DEG}
        if include_token:
            response["token"] = self._token
        return response

    def _watch(self):
        while True:
            time.sleep(0.2)
            with self._lock:
                if self._phase in {"idle", "saved", "recovery"}:
                    self._watcher = None
                    return
                try:
                    if self._phase != "stopping" and self._clock() - self._last_contact > self.LEASE_S:
                        self._abort("operator_timeout")
                    self._advance()
                except Exception:  # noqa: BLE001 — never lose the watchdog on a bad source
                    self._abort("internal_error")

    def _start_watch(self):
        if self._watcher is None or not self._watcher.is_alive():
            self._watcher = threading.Thread(target=self._watch, name="full-zero-watch", daemon=True)
            self._watcher.start()

    def begin(self):
        with self._lock:
            source = self._source()

            def reject(reason):
                return {"ok": False, "reason": reason,
                        "pendingRecovery": bool(getattr(source, "_full_zero_pending", False))}, 409

            if self._phase not in {"idle", "saved", "recovery"}:
                # Do not expose another session's actionable phase without its
                # token: the second window must not enable adjust/save buttons.
                return reject("full_zero_active")
            if self._estop():
                return reject("estop")
            if (self._neutral() or {}).get("locked"):
                return reject("neutral_locked")
            if self._navigation_active():
                return reject("navigation_active")
            required = ("begin_full_zero", "full_zero_calibrate", "full_zero_steer",
                        "full_zero_move_probe", "full_zero_request", "full_zero_acks",
                        "full_zero_mark_pending", "full_zero_clear_pending",
                        "stop_full_zero", "finish_full_zero")
            if not all(callable(getattr(source, name, None)) for name in required):
                return reject("control_not_supported")
            # Before reserving, require four live, correctly versioned boards.
            self._owner = source
            _, error = self._snapshot(allow_motion=True)
            if error and error not in {"module_busy", "wheel_moving"}:
                self._owner = None
                return reject(error)
            try:
                result = source.begin_full_zero()
            except Exception:  # noqa: BLE001 — reservation might already have happened
                result = {"ok": False, "reserved": True, "reason": "internal_error"}
            if not isinstance(result, dict):
                result = {"ok": False, "reserved": True, "reason": "internal_error"}
            if not result.get("ok") or not result.get("brakeId"):
                if result.get("reserved"):
                    now = self._clock()
                    self._token = secrets.token_urlsafe(24)
                    self._phase, self._last_contact = "braking", now
                    self._motion_ticket = (self._motion_ticket[0] + 1, True)
                    self._abort(result.get("reason", "brake_unconfirmed"))
                    self._start_watch()
                    return self._respond(ok=False, include_token=True), 409
                self._owner = None
                return reject(result.get("reason", "brake_unconfirmed"))
            now = self._clock()
            self._token = secrets.token_urlsafe(24)
            self._phase, self._reason = "braking", ""
            self._motion_ticket = (self._motion_ticket[0] + 1, True)
            self._brake_id, self._stage_at, self._last_contact = result["brakeId"], now, now
            self._seen, self._last_acks, self._expected = set(), {}, {}
            self._pending_move = None
            self._start_watch()
            return self._respond(include_token=True), 200

    def _abort(self, reason):
        if self._phase in {"idle", "saved", "recovery", "stopping"}:
            return
        self._reason = reason
        self._phase = "stopping"
        self._stage_at = self._clock()
        self._stop_attempt = self._stage_at
        try:
            result = self._owner.stop_full_zero()
        except Exception:  # noqa: BLE001
            result = {}
        self._brake_id = result.get("brakeId", 0) if result.get("ok") else 0

    def interrupt(self, reason="stop", *, repeat=False):
        with self._lock:
            if self._phase == "stopping" and repeat:
                try:
                    result = self._owner.stop_full_zero()
                    self._brake_id = result.get("brakeId", 0) if result.get("ok") else 0
                    self._stop_attempt = self._clock()
                except Exception:  # noqa: BLE001
                    self._brake_id = 0
            else:
                self._abort(reason)

    def _advance(self):
        phase = self._phase
        if phase in {"idle", "saved", "recovery"}:
            return
        now = self._clock()
        if phase == "stopping":
            motors, error = self._snapshot(allow_motion=False)
            if (not error and self._brake_id and
                    all(motors[m]["brakeId"] == self._brake_id for m in MODULES)):
                self._owner.finish_full_zero()
                self._phase = "recovery" if getattr(self._owner, "_full_zero_pending", False) else "idle"
                self._motion_ticket = (self._motion_ticket[0], self._phase == "recovery")
                self._token = None
            elif now - self._stop_attempt >= 2.0:
                self._stop_attempt = now
                try:
                    result = self._owner.stop_full_zero()
                    self._brake_id = result.get("brakeId", 0) if result.get("ok") else 0
                except Exception:  # noqa: BLE001
                    self._brake_id = 0
            return
        if self._estop():
            self._abort("estop")
            return
        if (self._neutral() or {}).get("locked"):
            self._abort("neutral_locked")
            return
        allow_motion = phase in {"calibrating", "adjusting"}
        require_home = phase in {"adjust", "adjusting", "checking", "saving"}
        motors, error = self._snapshot(allow_motion=allow_motion, require_home=require_home)
        if error:
            # During the initial STOP a busy wheel can legitimately be settling.
            if phase == "braking" and error in {"module_busy", "wheel_moving"} and now - self._stage_at < self.BRAKE_TIMEOUT_S:
                return
            self._abort(error)
            return
        if phase == "braking":
            if all(motors[m]["brakeId"] == self._brake_id for m in MODULES):
                self._phase = "ready"
            elif now - self._stage_at > self.BRAKE_TIMEOUT_S:
                self._abort("brake_unconfirmed")
            return
        # A different STOP, reset or neutral transition invalidates the session.
        if any(motors[m]["brakeId"] != self._brake_id for m in MODULES):
            self._abort("brake_changed")
            return
        if phase == "calibrating":
            for module in MODULES:
                if motors[module].get("calibrating"):
                    self._seen.add(module)
                elif module in self._seen and motors[module].get("calibError"):
                    self._abort("calibration_failed")
                    return
            if (len(self._seen) == 4 and all(
                motors[m].get("homed") is True and motors[m].get("homeSource") == "opto" and
                not motors[m].get("moving") and not motors[m].get("calibrating") and
                motors[m]["calibError"] == 0 for m in MODULES)):
                self._phase, self._reason = "adjust", ""
            elif (now - self._stage_at > self.CAL_TIMEOUT_S or
                  (len(self._seen) < 4 and now - self._stage_at > self.CAL_START_TIMEOUT_S)):
                self._abort("calibration_unconfirmed")
        elif phase == "adjusting":
            module, target = self._pending_move
            m = motors[module]
            ack = self._owner.full_zero_acks(self._request_id).get(module)
            # The zq follows w in the same wheel's FIFO. Status 1 proves that
            # this unique query ran while the board was idle; old target/angle
            # telemetry alone cannot acknowledge a repeated fine adjustment.
            if ack and ack.get("status") not in (1, 4):
                self._abort({3: "optical_zero_required", 5: "offset_limit",
                             6: "eeprom_error"}.get(ack.get("status"), "invalid_ack"))
            elif (ack and ack.get("status") == 1 and
                    abs(m["targetAngle"] - target) <= 0.03 and
                    abs(m["angle"] - target) <= 0.75 and not m["moving"]):
                # Pot deadband/ADC quantisation are not physical 0.1° accuracy.
                self._phase, self._pending_move = "adjust", None
            elif now - self._stage_at > self.MOVE_TIMEOUT_S:
                self._abort("steering_unconfirmed")
            elif ack and ack.get("status") == 4 and now - self._move_query_at >= 0.3:
                # Busy is cached by the firmware per ID; use a fresh query.
                request_id = self._next_request_id()
                result = self._owner.full_zero_move_probe(module, request_id)
                if not result.get("ok"):
                    self._abort(result.get("reason", "steering_unconfirmed"))
                else:
                    self._move_query_at = now
        elif phase in {"checking", "saving"}:
            acks = self._owner.full_zero_acks(self._request_id)
            self._last_acks = acks
            if any(ack.get("status") != (1 if phase == "checking" else 2) for ack in acks.values()):
                if phase == "checking":
                    self._phase = "adjust"
                    failed = next((a for a in acks.values() if a.get("status") != 1), {})
                    self._reason = {3: "optical_zero_required", 4: "module_busy", 5: "offset_limit",
                                    6: "eeprom_error"}.get(failed.get("status"), "save_rejected")
                else:
                    self._abort("partial_save")
                return
            if len(acks) < 4:
                if now - self._stage_at > self.REPLY_TIMEOUT_S:
                    if phase == "checking":
                        self._phase, self._reason = "adjust", "preflight_unconfirmed"
                    else:
                        self._abort("partial_save")
                return
            if set(acks) != set(MODULES):
                self._abort("invalid_ack")
                return
            if phase == "checking":
                self._expected = {m: acks[m]["proposedSteps"] for m in MODULES}
                if not self._owner.full_zero_mark_pending().get("ok"):
                    self._phase, self._reason = "adjust", "journal_unavailable"
                    return
                self._next_request_id()
                result = self._owner.full_zero_request("zc", self._request_id)
                if not result.get("ok"):
                    self._abort("partial_save")
                    return
                self._phase, self._stage_at, self._last_acks = "saving", now, {}
            else:
                if any(acks[m].get("storedSteps") != self._expected[m] or
                       acks[m].get("proposedSteps") != self._expected[m] for m in MODULES):
                    self._abort("partial_save")
                    return
                if not self._owner.full_zero_clear_pending().get("ok"):
                    self._abort("journal_unavailable")
                    return
                self._owner.finish_full_zero()
                self._motion_ticket = (self._motion_ticket[0], False)
                self._phase, self._reason = "saved", ""

    def handle(self, body):
        if not isinstance(body, dict) or not isinstance(body.get("action"), str):
            return {"ok": False, "reason": "payload"}, 400
        if body["action"] == "begin" and set(body) == {"action"}:
            return self.begin()
        with self._lock:
            token = body.get("token")
            if (not self._token or not isinstance(token, str) or not token.isascii() or
                    not secrets.compare_digest(token, self._token)):
                response = self._respond(ok=False)
                response["reason"] = self._reason or "session_expired"
                return response, 409
            self._last_contact = self._clock()
            self._advance()
            action = body["action"]
            if action == "status" and set(body) == {"action", "token"}:
                return self._respond(), 200
            if action == "cancel" and set(body) == {"action", "token"}:
                self._abort("cancelled")
                return self._respond(), 200
            if action == "calibrate" and set(body) == {"action", "token"} and self._phase == "ready":
                result = self._owner.full_zero_calibrate()
                if not result.get("ok") or result.get("offline"):
                    self._abort(result.get("reason", "module_offline"))
                    return self._respond(ok=False), 409
                self._phase, self._stage_at, self._seen = "calibrating", self._clock(), set()
                return self._respond(), 200
            if action == "adjust" and set(body) == {"action", "token", "module", "delta"} and self._phase == "adjust":
                module, delta = body.get("module"), body.get("delta")
                if module not in MODULES or not self._number(delta) or abs(delta) not in (0.1, 1.0):
                    return {"ok": False, "reason": "payload"}, 400
                motors, error = self._snapshot(require_home=True)
                if error:
                    self._abort(error)
                    return self._respond(ok=False), 409
                # Pot feedback may resynchronise the reported position away
                # from the old target. A +/- click is relative to where the
                # wheel actually reports standing, not that stale target.
                target = round(motors[module]["angle"] + delta, 1)
                if abs(target) > MAX_OFFSET_DEG:
                    return {"ok": False, "reason": "offset_limit"}, 400
                request_id = self._next_request_id()
                result = self._owner.full_zero_steer(module, target, request_id)
                if not result.get("ok"):
                    self._abort(result.get("reason", "command_failed"))
                    return self._respond(ok=False), 409
                self._pending_move = (module, target)
                self._phase, self._stage_at, self._reason = "adjusting", self._clock(), ""
                self._move_query_at = self._stage_at
                return self._respond(), 200
            if action == "save" and set(body) == {"action", "token"} and self._phase == "adjust":
                self._next_request_id()
                result = self._owner.full_zero_request("zq", self._request_id)
                if not result.get("ok"):
                    return {"ok": False, "reason": result.get("reason", "command_failed")}, 409
                self._phase, self._stage_at, self._reason, self._last_acks = "checking", self._clock(), "", {}
                return self._respond(), 200
            return {"ok": False, "reason": "invalid_phase", "phase": self._phase}, 409
