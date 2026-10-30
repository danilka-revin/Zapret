"""SC D19 — только показания; обратная связь и движение не меняются."""
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
    assert source.commands.empty(), "одна телеметрия не отправляет команды"

    # Поломка именно диагностического поля не превращает свежую плату в offline.
    assert source.feed(line("FL", sc="garbage"), now=100.3)
    assert source.servo_drive_status(now=100.4)["perModule"]["FL"]["canDrive"]
    assert source.read(now=100.4)["motors"][0]["scRpm"] is None
    assert source.read(now=102.0)["motors"][0]["scRpm"] is None  # устарело


def test_sc_values_never_influence_maneuver_mode_commands_or_learning():
    backend = slam.load_backend()

    def run(sc):
        frames = {module: backend.parse_master_line(line(module, sc=str(sc), extra="tach=1 rpm=999"))
                  for module in MODULES}
        runner = maneuvers.ManeuverRunner()
        commands = []
        assert runner.start("forward", 0.0)["ok"]
        for i in range(1, 31):
            runner.tick(i * 0.05, frames,
                        lambda module, value: commands.append(("steer", module, value)),
                        lambda module, value: commands.append(("drive", module, value)))
        assert runner.status()["mode"] == "time"
        assert not any(runner._travel.values())
        assert runner.status()["learned"] is None
        return runner.status(), commands

    assert run(0) == run(10000)
