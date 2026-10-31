"""Full-zero safety, four-board acknowledgements and interrupted saves."""
from __future__ import annotations

import copy
import os
import queue
import threading
import time

import pytest

from robot_control import slam
from robot_control.full_zero import FullZeroControl, clear_pending, write_pending

MODULES = ("FL", "FR", "RL", "RR")


class Clock:
    def __init__(self):
        self.value = 10.0

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


class FourBoards:
    def __init__(self, journal):
        self.journal = journal
        self._full_zero_pending = journal.exists()
        self._full_zero_inhibit = self._full_zero_pending
        self.motors = {m: {
            "id": m, "online": True, "telemetryAgeS": 0.02, "firmware": 23, "fullZeroFault": False,
            "neutralLocked": False, "neutralId": 0, "brakeId": 0, "drivePercent": 0,
            "scRpm": 0.0, "scRecent": False, "moving": False, "calibrating": False,
            "cycling": False, "homed": False, "homeSource": "none", "calibError": 0,
            "angle": 0.0, "targetAngle": 0.0,
        } for m in MODULES}
        self.drive = {"available": True, "active": False, "commandedPercent": 0,
                      "commandedModules": {}, "maneuver": {"active": False}}
        self.brake = 10
        self.calls = []
        self.requests = []
        self.acks = {}
        self.offline = []

    def read(self):
        return {"motors": copy.deepcopy(list(self.motors.values())), "drive": copy.deepcopy(self.drive)}

    def _brake(self):
        self.brake += 1
        for motor in self.motors.values():
            motor.update(brakeId=self.brake, drivePercent=0, moving=False, calibrating=False)
        return {"ok": True, "brakeId": self.brake}

    def begin_full_zero(self):
        self.calls.append("begin")
        self._full_zero_inhibit = True
        return self._brake()

    def stop_full_zero(self):
        self.calls.append("stop_steering_and_drive")
        self._full_zero_inhibit = True
        return self._brake()

    def finish_full_zero(self):
        self.calls.append("finish")
        self._full_zero_inhibit = self._full_zero_pending

    def full_zero_calibrate(self):
        self.calls.append("calibrate")
        for motor in self.motors.values():
            motor.update(calibrating=True, homed=False, homeSource="none")
        return {"ok": True, "offline": self.offline}

    def full_zero_steer(self, module, angle, req):
        self.calls.append(("steer", module, angle))
        self.motors[module].update(targetAngle=angle, angle=angle, moving=False)
        self.requests.append(("move", req))
        self.acks = {module: {"status": 1}}
        return {"ok": True}

    def full_zero_move_probe(self, module, req):
        self.requests.append(("move_probe", req))
        self.acks = {}
        return {"ok": True}

    def full_zero_request(self, operation, req):
        self.calls.append(operation)
        self.requests.append((operation, req))
        self.acks = {}
        return {"ok": True, "id": req}

    def full_zero_acks(self, req):
        if self.requests and self.requests[-1][1] == req:
            return copy.deepcopy(self.acks)
        return {}

    def acknowledge(self, status, modules=MODULES, proposed=None):
        values = proposed or {m: i * 13 for i, m in enumerate(MODULES)}
        self.acks = {m: {"module": m, "status": status, "storedSteps": values[m] if status == 2 else 0,
                         "proposedSteps": values[m]} for m in modules}

    def full_zero_mark_pending(self):
        self.calls.append("journal")
        write_pending(self.journal)
        self._full_zero_pending = True
        return {"ok": True}

    def full_zero_clear_pending(self):
        self.calls.append("clear_journal")
        clear_pending(self.journal)
        self._full_zero_pending = False
        return {"ok": True}


@pytest.fixture
def setup(tmp_path):
    clock = Clock()
    boards = FourBoards(tmp_path / "pending")
    control = FullZeroControl(lambda: boards, lambda: None, lambda: {"locked": False}, clock=clock)
    yield control, boards, clock
    control.interrupt("test_cleanup")
    control._advance()


def call(control, token, action, **values):
    return control.handle({"action": action, "token": token, **values})


def begin(control):
    result, code = control.handle({"action": "begin"})
    assert code == 200 and result["ok"] and result["phase"] == "braking"
    token = result["token"]
    result, code = call(control, token, "status")
    assert code == 200 and result["phase"] == "ready"
    return token


def homed(control, boards, token):
    result, code = call(control, token, "calibrate")
    assert code == 200 and result["phase"] == "calibrating"
    result, _ = call(control, token, "status")
    assert set(result["calibratingSeen"]) == set(MODULES)
    for motor in boards.motors.values():
        motor.update(calibrating=False, homed=True, homeSource="opto")
    result, _ = call(control, token, "status")
    assert result["phase"] == "adjust"


def test_full_workflow_saves_only_after_four_crc_acks(setup):
    control, boards, _ = setup
    token = begin(control)
    assert boards._full_zero_inhibit and control.ticket()[1]
    homed(control, boards, token)
    result, code = call(control, token, "adjust", module="FR", delta=0.1)
    assert code == 200 and result["phase"] == "adjusting"
    assert boards.calls[-1] == ("steer", "FR", 0.1)
    # The telemetry must echo the target and be idle, not just enqueue w.
    result, _ = call(control, token, "status")
    assert result["phase"] == "adjust"
    result, code = call(control, token, "save")
    assert code == 200 and result["phase"] == "checking" and not boards.journal.exists()
    boards.acknowledge(1, modules=MODULES[:3])
    result, _ = call(control, token, "status")
    assert result["phase"] == "checking" and "zc" not in boards.calls
    boards.acknowledge(1)
    result, _ = call(control, token, "status")
    assert result["phase"] == "saving" and boards.journal.exists()
    assert boards.calls.index("journal") < boards.calls.index("zc")
    boards.acknowledge(2, modules=MODULES[:3])
    result, _ = call(control, token, "status")
    assert result["phase"] == "saving" and boards.journal.exists() and boards._full_zero_inhibit
    boards.acknowledge(2)
    result, _ = call(control, token, "status")
    assert result["phase"] == "saved" and result["acknowledged"] == list(MODULES)
    assert not boards.journal.exists() and not boards._full_zero_inhibit and not control.ticket()[1]


def test_interrupted_save_is_latched_across_server_restart_and_recoverable(setup):
    control, boards, clock = setup
    token = begin(control)
    homed(control, boards, token)
    call(control, token, "save")
    boards.acknowledge(1)
    call(control, token, "status")
    boards.acknowledge(2, modules=("FL", "FR", "RL"))
    clock.advance(6)
    result, _ = call(control, token, "status")
    assert result["phase"] == "stopping" and result["reason"] == "partial_save"
    # Next telemetry confirms the stop; the durable marker survives cancellation.
    result, code = call(control, token, "status")
    assert code == 200 and result["phase"] == "recovery" and result["pendingRecovery"]
    assert boards.journal.exists() and boards._full_zero_inhibit and control.ticket()[1]
    rebooted = FourBoards(boards.journal)
    assert rebooted._full_zero_pending and rebooted._full_zero_inhibit
    recovered = FullZeroControl(lambda: rebooted, lambda: None, lambda: {"locked": False}, clock=clock)
    new_token = begin(recovered)
    homed(recovered, rebooted, new_token)
    call(recovered, new_token, "save")
    rebooted.acknowledge(1)
    call(recovered, new_token, "status")
    rebooted.acknowledge(2)
    result, _ = call(recovered, new_token, "status")
    assert result["phase"] == "saved" and not rebooted.journal.exists()


def test_missing_start_manual_zero_and_partial_offline_never_allow_adjust(setup):
    control, boards, clock = setup
    token = begin(control)
    boards.offline = ["RR"]
    result, code = call(control, token, "calibrate")
    assert code == 409 and result["phase"] == "stopping"
    assert "stop_steering_and_drive" in boards.calls
    call(control, token, "status")
    boards.offline = []
    token = begin(control)
    call(control, token, "calibrate")
    # Three started; RR never reports calibration, even if it reports an old home.
    boards.motors["RR"].update(calibrating=False, homed=True, homeSource="manual")
    result, _ = call(control, token, "status")
    assert set(result["calibratingSeen"]) == {"FL", "FR", "RL"}
    for m in ("FL", "FR", "RL"):
        boards.motors[m].update(calibrating=False, homed=True, homeSource="opto")
    clock.advance(16)
    result, _ = call(control, token, "status")
    assert result["phase"] == "stopping" and result["reason"] == "calibration_unconfirmed"
    assert not boards.journal.exists()


def test_preflight_rejection_and_partial_ack_do_not_write(setup):
    control, boards, _ = setup
    token = begin(control)
    homed(control, boards, token)
    call(control, token, "save")
    boards.acknowledge(1)
    boards.acks["RR"]["status"] = 5
    result, _ = call(control, token, "status")
    assert result["phase"] == "adjust" and result["reason"] == "offset_limit"
    assert "journal" not in boards.calls and "zc" not in boards.calls


def test_stop_revokes_session_and_keeps_stop_available(setup):
    control, boards, _ = setup
    before = control.ticket()
    token = begin(control)
    during = control.ticket()
    assert during[0] != before[0] and during[1]
    homed(control, boards, token)
    control.interrupt("estop")
    assert boards.calls[-1] == "stop_steering_and_drive"
    result, code = call(control, token, "adjust", module="FL", delta=0.1)
    assert code == 409 and result["reason"] in {"session_expired", "estop", "invalid_phase"}
    assert not control.ticket()[1] and control.ticket()[0] != before[0]
    assert not any(isinstance(c, tuple) and c[0] == "steer" for c in boards.calls)


def test_old_firmware_bad_payload_and_foreign_session_fail_closed(setup):
    control, boards, _ = setup
    boards.motors["RL"]["firmware"] = 22
    result, code = control.handle({"action": "begin"})
    assert code == 409 and result["reason"] == "firmware_update_required"
    assert "begin" not in boards.calls
    boards.motors["RL"]["firmware"] = 23
    token = begin(control)
    result, code = call(control, "чужой", "calibrate")
    assert code == 409 and not result["ok"]
    result, code = call(control, token, "adjust", module="XX", delta=True)
    assert code == 409  # wrong phase; no motion
    assert "calibrate" not in boards.calls


def test_watchdog_aborts_if_browser_disappears(tmp_path):
    boards = FourBoards(tmp_path / "pending")
    control = FullZeroControl(lambda: boards, lambda: None, lambda: {"locked": False})
    control.LEASE_S = 0.15
    token = begin(control)
    homed(control, boards, token)
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline and "stop_steering_and_drive" not in boards.calls:
        time.sleep(.03)
    assert "stop_steering_and_drive" in boards.calls
    assert not boards.journal.exists()


def test_checked_reply_rejects_bad_crc_role_version_and_shape():
    backend = slam.load_backend()
    def frame(payload):
        return f"{payload} {backend.crc16(payload.encode()):04X}"
    line = frame("@Z 23 2 456 2 -130 260")
    parsed = backend.parse_full_zero_reply(line)
    assert parsed == {"module": "FR", "firmware": 23, "id": 456, "status": 2,
                      "storedSteps": -130, "proposedSteps": 260}
    assert backend.parse_full_zero_reply(line[:-1] + ("0" if line[-1] != "0" else "1")) is None
    for payload in ("@Z 22 2 456 2 0 0", "@Z 23 5 456 2 0 0", "@Z 23 2 0 2 0 0",
                    "@Z 23 2 456 9 0 0", "@Z 23 2 456 2 2668 0", "@Z 23 2 456 2 0"):
        assert backend.parse_full_zero_reply(frame(payload)) is None


def test_journal_atomic_and_restore_interlock(tmp_path, monkeypatch):
    path = tmp_path / "private" / "pending"
    monkeypatch.setenv("RC_FULL_ZERO_PENDING", str(path))
    write_pending(path)
    assert path.exists() and b'full_zero_commit_pending' in path.read_bytes()
    backend = slam.load_backend()
    source = backend.MasterSource(auto_discovery=True, port_provider=lambda: [])
    try:
        assert source._full_zero_pending and source._full_zero_inhibit
        assert source._neutral_blocks_motion()
        assert source.drive_servo(5)["ok"] is False
    finally:
        source._stop = True
        source._dispatch_wakeup.set()
    clear_pending(path)
    assert not path.exists()


def test_onboard_and_remote_routes_exist_and_require_markers(kiosk_client, client, operator):
    from conftest import login
    assert 'id="full-zero-open"' in kiosk_client.get('/kiosk').text
    assert kiosk_client.get('/full_zero.js').status_code == 200
    assert kiosk_client.post('/api/kiosk_full_zero', json={'action': 'begin'}).status_code == 400
    unavailable = kiosk_client.post('/api/kiosk_full_zero', json={'action': 'begin'},
                                    headers={'X-Robot-Control': 'steering'})
    assert unavailable.status_code == 409 and unavailable.json['reason'] == 'control_not_supported'
    login(client)
    page = client.get('/controls')
    assert 'id="full-zero-open"' in page.text and client.get('/full_zero.js').status_code == 200
    assert client.post('/api/control/full_zero', json={'action': 'begin'},
                       headers={'X-Robot-Control': 'steering'}).json['reason'] == 'csrf_required'


def test_failed_initial_brake_retains_interlock_until_fresh_stop(setup):
    control, boards, _ = setup
    def failed_reservation():
        boards._full_zero_inhibit = True
        return {"ok": False, "reserved": True, "reason": "brake_unconfirmed"}
    boards.begin_full_zero = failed_reservation
    result, code = control.handle({"action": "begin"})
    assert code == 409 and result["phase"] == "stopping" and control.ticket()[1]
    assert boards._full_zero_inhibit and "stop_steering_and_drive" in boards.calls
    token = result["token"]
    result, code = call(control, token, "status")
    assert code == 200 and result["phase"] == "idle"
    assert not boards._full_zero_inhibit and not control.ticket()[1]


def test_corrupt_eeprom_report_prevents_wizard(setup):
    control, boards, _ = setup
    boards.motors["FL"]["fullZeroFault"] = True
    result, status = control.handle({"action": "begin"})
    assert status == 409 and result["reason"] == "eeprom_error"
    assert "begin" not in boards.calls


def test_fault_on_one_fw23_board_inhibits_all_direct_servo(monkeypatch, tmp_path):
    backend = slam.load_backend()
    monkeypatch.setenv("RC_FULL_ZERO_PENDING", str(tmp_path / "no-pending"))
    source = backend.MasterSource(auto_discovery=True, port_provider=lambda: [])
    try:
        source.serial_ready.set()
        now = time.time()
        with source.lock:
            source.frames = {m: ({"firmware": 23, "fullZeroFault": m == "RR",
                                  "driveMaxPercent": 100, "drivePercent": 0,
                                  "cal": True, "moving": False, "calib": False,
                                  "cycle": False, "deg": 0}, now) for m in MODULES}
        result = source.servo_drive_status(now)
        assert not result["canDrive"] and result["reason"] == "full_zero_eeprom_fault"
        assert all(not result["perModule"][m]["canDrive"] for m in MODULES)
        assert source.drive_servo(5, "FL")["reason"] == "full_zero_eeprom_fault"
    finally:
        source._stop = True
        source._dispatch_wakeup.set()


def test_new_fault_frame_brakes_all_and_closes_every_motion_path(monkeypatch, tmp_path):
    backend = slam.load_backend()
    monkeypatch.setenv("RC_FULL_ZERO_PENDING", str(tmp_path / "no-pending"))
    source = backend.MasterSource(auto_discovery=True, port_provider=lambda: [])
    try:
        source._stop = True  # no scanner races; exercise the real gated frame handler
        source._dispatch_wakeup.set()
        links = {m: backend._WheelLink("/virtual/" + m) for m in MODULES}
        for module, link in links.items():
            link.module = module
        with source.lock:
            source.links = {link.path: link for link in links.values()}
            source.modules = {module: link.path for module, link in links.items()}
        source.serial_ready.set()
        payload = "@F 23 4 0 0 %d 0 100 0 0 0 90 0 0 0 0 0 0 0 1" % (0x2000 | 0x4000 | 0x40 | 0x20 | 0x02)
        line = "%s %04X" % (payload, backend.crc16(payload.encode("ascii")))
        source.commands.put_nowait(b"v 50\n")
        assert source._link_line(links["RR"], line)
        assert source.commands.empty()
        for link in links.values():
            assert link.commands.get_nowait().startswith(b"s q\nb ")
        # A previously checked request cannot enqueue after the fault arrived.
        source.serial_ready.set()
        assert source._queue_motion(b"w FL 1\n")["reason"] == "full_zero_eeprom_fault"
        assert source.drive_servo(5, "FL")["reason"] in {"full_zero_eeprom_fault", "module_offline"}
    finally:
        source._stop = True
        source._dispatch_wakeup.set()


def test_recovery_warning_survives_an_offline_begin(setup):
    control, boards, _ = setup
    boards._full_zero_pending = boards._full_zero_inhibit = True
    boards.motors["RR"]["online"] = False
    data, status = control.handle({"action": "begin"})
    assert status == 409 and data["reason"] == "module_offline" and data["pendingRecovery"]
    assert boards._full_zero_inhibit


def test_unexpected_reservation_exception_still_revokes_and_brakes(setup):
    control, boards, _ = setup
    def broken_begin():
        boards._full_zero_inhibit = True
        raise OSError("link broke after reservation")
    boards.begin_full_zero = broken_begin
    data, status = control.handle({"action": "begin"})
    assert status == 409 and data["phase"] == "stopping"
    assert data["reason"] == "internal_error" and "stop_steering_and_drive" in boards.calls
    assert boards._full_zero_inhibit


def test_watchdog_turns_unexpected_source_errors_into_stop(setup):
    control, boards, _ = setup
    token = begin(control)
    original = control._advance
    seen = threading.Event()
    def once():
        if not seen.is_set():
            seen.set()
            raise ValueError("bad telemetry")
        return original()
    control._advance = once
    assert seen.wait(1)
    deadline = time.monotonic() + 1
    while "stop_steering_and_drive" not in boards.calls and time.monotonic() < deadline:
        time.sleep(0.005)
    assert boards._full_zero_inhibit and "stop_steering_and_drive" in boards.calls
    status, _ = call(control, token, "status")
    assert status["reason"] == "internal_error"


def test_journal_fsync_failure_never_issues_commit_and_retains_existing_pending(setup, monkeypatch):
    control, boards, _ = setup
    token = begin(control)
    homed(control, boards, token)
    call(control, token, "save")
    boards.acknowledge(1)
    def cannot_sync():
        # Like an fsync failure after rename: marker may already exist.
        boards.journal.write_text("possibly durable")
        boards._full_zero_pending = True
        return {"ok": False, "reason": "journal_unavailable"}
    monkeypatch.setattr(boards, "full_zero_mark_pending", cannot_sync)
    result, status = call(control, token, "status")
    assert status == 200 and result["phase"] == "adjust" and result["reason"] == "journal_unavailable"
    assert "zc" not in boards.calls and boards._full_zero_inhibit and result["pendingRecovery"]
    control.interrupt("cancelled")
    result, _ = call(control, token, "status")
    assert result["phase"] == "recovery" and boards._full_zero_inhibit


def test_real_owner_fsync_failures_leave_in_memory_interlock(monkeypatch, tmp_path):
    import robot_control.full_zero as full_zero
    path = tmp_path / "pending"
    monkeypatch.setenv("RC_FULL_ZERO_PENDING", str(path))
    backend = slam.load_backend()
    source = backend.MasterSource(auto_discovery=True, port_provider=lambda: [])
    source._stop = True
    source._dispatch_wakeup.set()
    source._full_zero_inhibit = True  # reservation was taken before journal work
    def fail_sync(_directory):
        raise OSError("injected directory fsync failure")
    monkeypatch.setattr(full_zero, "_sync_directory", fail_sync)
    result = source.full_zero_mark_pending()
    assert not result["ok"] and path.exists()
    assert source._full_zero_pending and source._full_zero_inhibit
    result = source.full_zero_clear_pending()
    assert not result["ok"] and source._full_zero_pending and source._full_zero_inhibit
    # unlink already happened: without a successful directory fsync a restart
    # may observe either a present or an absent marker. The controller only
    # calls clear_pending after four matching commit ACKs; it must not claim
    # durable recovery from the in-memory latch alone after a process restart.
    assert not path.exists()
    restarted = backend.MasterSource(auto_discovery=True, port_provider=lambda: [])
    restarted._stop = True
    restarted._dispatch_wakeup.set()
    assert not restarted._full_zero_pending


def test_missing_safety_field_cannot_be_treated_as_idle(setup):
    control, boards, _ = setup
    boards.motors["RR"].pop("moving")
    result, status = control.handle({"action": "begin"})
    assert status == 409 and result["reason"] == "invalid_telemetry"
    assert "begin" not in boards.calls


def test_steering_target_echo_without_arrival_is_not_confirmation(setup):
    control, boards, clock = setup
    token = begin(control)
    homed(control, boards, token)
    response, status = call(control, token, "adjust", module="FR", delta=0.1)
    assert status == 200 and response["phase"] == "adjusting"
    boards.motors["FR"]["angle"] = 1.0  # target echoed, but position did not arrive
    response, _ = call(control, token, "status")
    assert response["phase"] == "adjusting"
    clock.advance(11)
    response, _ = call(control, token, "status")
    assert response["phase"] == "stopping" and response["reason"] == "steering_unconfirmed"


def test_fine_step_uses_reported_position_after_pot_resynchronisation(setup):
    control, boards, _ = setup
    token = begin(control)
    homed(control, boards, token)
    boards.motors["FR"].update(angle=0.4, targetAngle=0.0)
    response, status = call(control, token, "adjust", module="FR", delta=0.1)
    assert status == 200 and response["phase"] == "adjusting"
    assert boards.calls[-1] == ("steer", "FR", 0.5)


def test_pending_fsyncs_every_new_directory_entry(monkeypatch, tmp_path):
    import robot_control.full_zero as full_zero
    synced = []
    original = full_zero._sync_directory
    def tracking(directory):
        synced.append(directory)
        original(directory)
    monkeypatch.setattr(full_zero, "_sync_directory", tracking)
    journal = tmp_path / "new-a" / "new-b" / "pending"
    write_pending(journal)
    assert synced == [tmp_path, tmp_path / "new-a", tmp_path / "new-a" / "new-b"]
    assert journal.exists()


def test_another_window_cannot_appear_to_own_active_adjustment(setup):
    control, boards, _ = setup
    token = begin(control)
    homed(control, boards, token)
    other, status = control.handle({"action": "begin"})
    assert status == 409 and other["reason"] == "full_zero_active"
    assert "phase" not in other and "token" not in other
    result, status = call(control, token, "status")
    assert status == 200 and result["phase"] == "adjust"


def test_repeated_fine_step_waits_for_fresh_idle_ack_and_requeries_busy(setup):
    control, boards, clock = setup
    token = begin(control)
    homed(control, boards, token)
    # The preceding move left the same target echo, but the pot resynced the
    # reported position. A second 0.1° press must not reuse that old echo.
    boards.motors["FR"].update(angle=0.0, targetAngle=0.1)
    result, code = call(control, token, "adjust", module="FR", delta=0.1)
    assert code == 200 and result["phase"] == "adjusting"
    first_id = boards.requests[-1][1]
    boards.acks = {}  # queued move/query has not actually been acknowledged
    result, _ = call(control, token, "status")
    assert result["phase"] == "adjusting"
    # The initial zq was processed after w while the board was still moving.
    boards.acks = {"FR": {"status": 4}}
    clock.advance(.4)
    result, _ = call(control, token, "status")
    assert result["phase"] == "adjusting"
    assert boards.requests[-1][0] == "move_probe" and boards.requests[-1][1] != first_id
    result, _ = call(control, token, "status")
    assert result["phase"] == "adjusting"  # still no ACK for the new query
    boards.acks = {"FR": {"status": 1}}
    result, _ = call(control, token, "status")
    assert result["phase"] == "adjust"
