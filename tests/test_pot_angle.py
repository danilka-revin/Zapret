"""fw26: A0 is a checked display channel, distinct from control step counts."""
from __future__ import annotations

import math
import queue
import threading
from pathlib import Path

import pytest

from robot_control import slam

MODULES = ("FL", "FR", "RL", "RR")


def checked(backend, payload, *, compact=False):
    suffix = f"{backend.crc16(payload.encode('ascii')):04X}"
    return f"{payload} {suffix}" if compact else f"{payload} crc={suffix}"


def fast(backend, *, fw=26, raw=825, status=0x40 | 0x2000 | 0x1000 | 0x80 | 0x20 | 0x02,
         role=2, deg=0, tgt=0, live=0):
    payload = f"@F {fw} {role} {deg} {tgt} {status} 0 100 0 0 0 90 0 0 0 0 0 {live} 0 1000"
    if raw is not None:
        payload += f" {raw}"
    return checked(backend, payload, compact=True)


def full(backend, mid="FR", *, fw=26, raw=825, pot=1, pm=1, cal=1, calib=0, live=0, deg=0, tgt=0, zfault=0):
    payload = (f"@TLM mod={mid} deg={deg} tgt={tgt} moving=0 cal={cal} cycle=0 calib={calib} "
               f"opto=0 home={'opto' if cal else 'none'} err=0 drive=0 fw={fw} pot={pot} pm={pm} "
               f"ph=0 zfault={zfault} tach=0 dmax=100 sv=0 duty=0 sc_rpm=0.0 sc_hz=0 sc_ppr=90 "
               f"sc_count=0 sc_recent=0 brake_id=0 neutral_id=0 neutral_lock=0 "
               f"live_id={live} live_seq=0 t=1000")
    if raw is not None:
        payload += f" pa={raw}"
    return checked(backend, payload)


def source(backend):
    src = object.__new__(backend.MasterSource)
    src.lock = threading.Lock()
    src.frames = {}
    src.drive_percent = 0
    src.drive_last_command = 0.0
    src.commands = queue.Queue(maxsize=4)
    src.serial_ready = threading.Event()
    src.serial_ready.set()
    src.messages = []
    src.on_message = None
    return src


def test_both_crc_frames_preserve_step_and_sensor_angles():
    backend = slam.load_backend()
    for raw in (0, 825, -1234, -18000, 18000):
        short = backend.parse_master_line(fast(backend, raw=raw, deg=100, tgt=200))
        long = backend.parse_master_line(full(backend, raw=raw, deg=1, tgt=2))
        assert short == long
        assert short["deg"] == 1 and short["tgt"] == 2 and short["potAngle"] == raw / 100
    assert backend.parse_master_line(fast(backend), module="FL") is None
    corrupt = fast(backend).replace("825", "826")
    assert backend.parse_master_line(corrupt) is None


@pytest.mark.parametrize("raw", [32767, 18001, -18001, 10**20, None])
def test_invalid_sensor_never_invalidates_other_full_safety_fields(raw):
    backend = slam.load_backend()
    frame = backend.parse_master_line(full(backend, raw=raw))
    assert frame and frame["potAngle"] is None and frame["drivePercent"] == 0
    if raw is not None:
        frame = backend.parse_master_line(fast(backend, raw=raw))
        assert frame and frame["potAngle"] is None


def test_fw26_format_cannot_silently_fall_back_to_old_compact_protocol():
    backend = slam.load_backend()
    assert backend.parse_master_line(fast(backend, fw=26, raw=None)) is None
    assert backend.parse_master_line(fast(backend, fw=25, raw=825)) is None
    assert backend.parse_master_line(fast(backend, fw=25, raw=None))["potAngle"] is None
    assert backend.parse_master_line(full(backend, fw=25, raw=825))["potAngle"] is None


@pytest.mark.parametrize("fields", [dict(pot=0), dict(pm=0), dict(cal=0), dict(calib=1), dict(zfault=1)])
def test_full_snapshot_does_not_trust_sensor_without_ready_state(fields):
    backend = slam.load_backend()
    assert backend.parse_master_line(full(backend, **fields))["potAngle"] is None


def test_fast_snapshot_does_not_trust_sensor_without_ready_state():
    backend = slam.load_backend()
    ready = 0x40 | 0x2000 | 0x1000 | 0x80 | 0x20 | 0x02
    for status in (ready & ~0x80, ready & ~0x1000, ready & ~0x02, ready | 0x04, ready | 0x4000):
        assert backend.parse_master_line(fast(backend, status=status))["potAngle"] is None


def test_state_exposes_fresh_sensor_separately_and_stale_is_not_displayed():
    backend = slam.load_backend()
    src = source(backend)
    src.feed(full(backend, raw=-1234, deg=2), now=100)
    motor = next(m for m in src.read(now=100.2)["motors"] if m["id"] == "FR")
    assert motor["angle"] == 2 and motor["potAngle"] == -12.34
    assert motor["displayAngle"] == -12.34 and motor["angleSource"] == "pot"
    stale = next(m for m in src.read(now=102)["motors"] if m["id"] == "FR")
    assert not stale["online"] and stale["angle"] is stale["potAngle"] is stale["displayAngle"] is None
    src.feed(full(backend, fw=25, raw=None, deg=2), now=100)
    old = next(m for m in src.read(now=100.2)["motors"] if m["id"] == "FR")
    assert old["angle"] == old["displayAngle"] == 2 and old["angleSource"] == "steps"
    assert old["potAngle"] is None


def test_hold_preflight_names_the_blocking_wheel_and_off_remains_available():
    backend = slam.load_backend()
    src = source(backend)
    for mid in MODULES:
        src.feed(full(backend, mid, raw=0), now=100)
    src.feed(full(backend, "RL", raw=1550), now=100)
    result = src.set_pot_hold("on", now=100.2)
    assert result == {"ok": False, "reason": "pot_hold_far", "modules": ["RL"]}
    assert src.commands.empty()
    src.feed(full(backend, "RL", raw=32767), now=100)
    assert src.set_pot_hold("on", now=100.2) == {
        "ok": False, "reason": "pot_angle_unavailable", "modules": ["RL"]}
    src.feed(full(backend, "RL", raw=0, zfault=1), now=100)
    assert src.set_pot_hold("on", now=100.2) == {
        "ok": False, "reason": "full_zero_eeprom_fault", "modules": ["RL"]}
    src.feed(full(backend, "RL", raw=0), now=100)
    src.feed(full(backend, "FR", raw=0, live=42), now=100)
    assert src.set_pot_hold("on", now=100.2) == {
        "ok": False, "reason": "keyboard_active", "modules": ["FR"]}
    assert src.commands.empty()
    assert src.set_pot_hold("off", now=100.2)["ok"]
    assert src.commands.get_nowait() == b"ph 0\n"
