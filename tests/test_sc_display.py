"""SC D19: совместимость v16, счётчики v17 и неизменная ручная тяга."""
from __future__ import annotations

import queue
import threading

from robot_control import maneuvers, slam

MODULES = ("FL", "FR", "RL", "RR")


def line(module, *, sc="60.0", hz="90", ppr="90", extra=""):
    return (f"@TLM mod={module} deg=0 tgt=0 moving=0 cal=1 cycle=0 calib=0 "
            f"drive=0 fw=16 tach=0 dmax=100 sv=0 duty=0 "
            f"sc_rpm={sc} sc_hz={hz} sc_ppr={ppr} {extra}")


def test_sc_fields_are_separate_from_legacy_control_feedback():
    backend = slam.load_backend()
    frame = backend.parse_master_line(line("FL", extra="rpm=999 tach=1"))
    assert frame["scRpm"] == 60.0 and frame["scHz"] == 90 and frame["scPpr"] == 90
    assert frame["rpm"] is None and frame["tach"] is False

    # Исторический протокол остаётся совместимым; v15 не выдумывает обороты.
    legacy = backend.parse_master_line(
        "@TLM mod=FL deg=0 tgt=0 moving=0 cal=1 drive=0 fw=12 tach=1 rpm=55")
    assert legacy["rpm"] == 55 and legacy["tach"] is True and legacy["scRpm"] is None
    v15 = backend.parse_master_line(
        "@TLM mod=FL deg=0 tgt=0 moving=0 cal=1 drive=0 fw=15 tach=0")
    assert v15["rpm"] is None and v15["scRpm"] is None


def test_corrupt_sc_diagnostics_do_not_discard_drive_telemetry():
    backend = slam.load_backend()
    for value in ("bad", "nan", "inf", "-1", "1e999"):
        frame = backend.parse_master_line(line("FL", sc=value, hz="bad", ppr="0"))
        assert frame is not None and frame["drivePercent"] == 0
        assert frame["scRpm"] is None and frame["scHz"] is None and frame["scPpr"] is None
        assert frame["rpm"] is None and frame["tach"] is False


def test_readout_is_fresh_per_wheel_and_does_not_change_drive_readiness():
    backend = slam.load_backend()
    source = object.__new__(backend.MasterSource)
    source.lock = threading.Lock()
    source.frames = {}
    source.drive_percent = 0
    source.drive_last_command = 0.0
    source.commands = queue.Queue(maxsize=8)
    source.serial_ready = threading.Event()
    source.serial_ready.set()
    for module, rpm in zip(MODULES, (0, 60, 120, 15.5)):
        assert source.feed(line(module, sc=str(rpm)), now=100.0)
    snapshot = source.read(now=100.2)
    values = {motor["id"]: motor for motor in snapshot["motors"]}
    assert [values[m]["scRpm"] for m in MODULES] == [0.0, 60.0, 120.0, 15.5]
    assert all(values[m]["rpm"] is None and values[m]["tach"] is False for m in MODULES)
    assert snapshot["speedMps"] is None and snapshot["drive"]["canDrive"]
    # Отдельная скорость обода — не скорость корпуса и не пробег робота.
    assert snapshot["wheelSpeedMps"] == round((0 + 60 + 120 + 15.5) / 4 * 3.141592653589793 * 0.2159 / 60, 3)
    assert source.commands.empty(), "одна телеметрия не отправляет команды"

    # Поломка именно диагностического поля не превращает свежую плату в offline.
    assert source.feed(line("FL", sc="garbage"), now=100.3)
    assert source.servo_drive_status(now=100.4)["perModule"]["FL"]["canDrive"]
    assert source.read(now=100.4)["motors"][0]["scRpm"] is None
    assert source.read(now=102.0)["motors"][0]["scRpm"] is None  # устарело


def test_v16_cannot_start_a_maneuver_from_averaged_rpm():
    backend = slam.load_backend()
    frames = {module: backend.parse_master_line(line(module, sc="60", extra="tach=1 rpm=999 sc_count=90 t=1000"))
              for module in MODULES}
    assert all(frame["scCount"] is None for frame in frames.values())
    runner = maneuvers.ManeuverRunner()
    commands = []
    assert runner.start("forward", 0.0)["ok"]
    runner.tick(0.05, frames, lambda *args: commands.append(args), lambda *args: commands.append(args))
    assert runner.status()["reason"] == "sc_firmware_required"
    assert not commands


def test_v17_counter_and_uptime_are_validated_without_discarding_drive_data():
    backend = slam.load_backend()
    v17 = line("FL", sc="12", extra="sc_count=4294967295 t=4294967295").replace("fw=16", "fw=17")
    frame = backend.parse_master_line(v17)
    assert frame["scCount"] == frame["uptimeMs"] == 4294967295
    assert frame["tach"] is False and frame["rpm"] is None
    for bad in ("-1", "4294967296", "nan", "1.2", "oops"):
        damaged = backend.parse_master_line(v17.replace("sc_count=4294967295", "sc_count=" + bad))
        assert damaged["scCount"] is None and damaged["drivePercent"] == 0


def test_v18_neutral_fields_are_strict_without_corrupting_other_telemetry():
    backend = slam.load_backend()
    text = line("FL", extra="brake_id=20 neutral_id=0 neutral_lock=1 t=1000").replace("fw=16", "fw=18")
    frame = backend.parse_master_line(text)
    assert frame["neutralId"] == 0 and frame["neutralLocked"] is True and frame["brakeId"] == 20
    for malformed in ("-1", "2", "nan", "true"):
        frame = backend.parse_master_line(text.replace("neutral_lock=1", "neutral_lock=" + malformed))
        assert frame["neutralLocked"] is None and frame["drivePercent"] == 0
    v17 = backend.parse_master_line(text.replace("fw=18", "fw=17"))
    assert v17["neutralId"] is None and v17["neutralLocked"] is None


def test_source_fails_closed_on_unknown_v18_neutral_state():
    backend = slam.load_backend()
    source = object.__new__(backend.MasterSource)
    source.lock = threading.Lock()
    frame = backend.parse_master_line(line("FL", extra="brake_id=20 neutral_id=0 neutral_lock=0 t=1000")
                                      .replace("fw=16", "fw=18"))
    source.frames = {"FL": (frame, 0)}
    assert not source.observed_neutral()
    frame["neutralLocked"] = None
    assert source.observed_neutral()  # unknown is not a confirmed brake
    frame["firmware"] = 17
    assert not source.observed_neutral()  # v17 remains compatible with ordinary drive


def test_v19_recent_sc_field_is_strict_and_not_invented_for_old_boards():
    backend = slam.load_backend()
    text = line("FL", sc="0", extra="sc_count=1 sc_recent=1 brake_id=1 neutral_id=0 neutral_lock=0 t=1000").replace("fw=16", "fw=19")
    assert backend.parse_master_line(text)["scRecent"] is True
    assert backend.parse_master_line(text.replace("sc_recent=1", "sc_recent=0"))["scRecent"] is False
    for bad in ("true", "2", "-1", "nan", ""):
        frame = backend.parse_master_line(text.replace("sc_recent=1", "sc_recent=" + bad))
        assert frame["scRecent"] is None and frame["drivePercent"] == 0
    assert backend.parse_master_line(text.replace("fw=19", "fw=18"))["scRecent"] is None


def test_v19_manual_steering_rejects_recent_or_unknown_sc():
    backend = slam.load_backend()
    source = object.__new__(backend.MasterSource)
    source.lock = threading.Lock()
    source.frames = {}
    source.drive_percent = 0
    source.drive_last_command = 0
    source.commands = queue.Queue(maxsize=8)
    source.serial_ready = threading.Event()
    source.serial_ready.set()
    frame = line("FL", sc="0", extra="sc_count=1 sc_recent=1 brake_id=1 neutral_id=0 neutral_lock=0 t=1000").replace("fw=16", "fw=19")
    source.feed(frame, now=100.0)
    assert source.steer_module("FL", 25, now=100.1)["reason"] == "wheel_moving"
    source.feed(frame.replace("sc_recent=1", "sc_recent=broken"), now=100.0)
    assert source.steer_module("FL", 25, now=100.1)["reason"] == "sc_invalid"
    source.feed(frame.replace("sc_recent=1", "sc_recent=0"), now=100.0)
    assert source.steer_module("FL", 25, now=100.1)["ok"]
    assert source.commands.get_nowait() == b"w FL 25.0\n"


def test_v20_live_ack_is_strict_and_never_invented_for_older_firmware():
    backend = slam.load_backend()
    text = line("FL", sc="0", extra="sc_count=1 sc_recent=0 brake_id=9 neutral_id=0 neutral_lock=0 live_id=9 live_seq=12 t=1000").replace("fw=16", "fw=20")
    frame = backend.parse_master_line(text)
    assert frame["liveId"] == 9 and frame["liveSeq"] == 12
    assert backend.parse_master_line(text.replace("fw=20", "fw=19"))["liveId"] is None
    for field in ("live_id=9", "live_seq=12"):
        for bad in ("-1", "2147483647", "nan", "true", ""):
            parsed = backend.parse_master_line(text.replace(field, field.split('=')[0] + '=' + bad))
            key = "liveId" if field.startswith("live_id") else "liveSeq"
            assert parsed[key] is None
