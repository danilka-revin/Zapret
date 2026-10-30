"""Event-driven wheel dispatch and its safety ordering, without real hardware."""
from __future__ import annotations

import queue
import threading
import time

import pytest

from robot_control import slam

MODULES = ("FL", "FR", "RL", "RR")
TLM = (b"@TLM deg=0 tgt=0 moving=0 cal=1 cycle=0 calib=0 opto=0 "
       b"home=manual err=0 drive=0 fw=17 pot=0 pm=0 tach=0 dmax=100 sv=0 duty=0 "
       b"sc_rpm=0.0 sc_hz=0 sc_ppr=90 sc_count=0 brake_id=0 t=1\n")


def wait_for(predicate, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.002)
    return bool(predicate())


class HeldReadPort:
    """The second read never returns until the test explicitly releases it."""

    def __init__(self):
        self.initial = bytearray(TLM)
        self.read_blocked = threading.Event()
        self.release_read = threading.Event()
        self.write_blocked = threading.Event()
        self.release_write = threading.Event()
        self.block_value = None
        self.fail_write = False
        self.closed = False
        self.writes = []
        self.lock = threading.Lock()

    @property
    def in_waiting(self):
        return len(self.initial)

    def read(self, size):
        if self.initial:
            data = bytes(self.initial[:size])
            del self.initial[:size]
            return data
        self.read_blocked.set()
        self.release_read.wait(5)
        return b""

    def write(self, value):
        if self.fail_write:
            raise OSError("simulated USB write failure")
        value = bytes(value)
        if value == self.block_value:
            self.write_blocked.set()
            if not self.release_write.wait(3):
                raise TimeoutError("test writer not released")
        with self.lock:
            self.writes.append(value)
        return len(value)

    def values(self):
        with self.lock:
            return list(self.writes)

    def flush(self):
        raise AssertionError("per-command flush would unnecessarily block the writer")

    def cancel_read(self):
        self.release_read.set()

    def cancel_write(self):
        self.release_write.set()

    def close(self):
        self.closed = True
        self.release_read.set()
        self.release_write.set()


@pytest.fixture
def held_wheels(monkeypatch, tmp_path):
    serial = pytest.importorskip("serial")
    backend = slam.load_backend()
    paths = {m: str(tmp_path / m) for m in MODULES}
    for path in paths.values():
        open(path, "wb").close()
    ports = {m: HeldReadPort() for m in MODULES}
    by_path = {paths[m]: ports[m] for m in MODULES}
    monkeypatch.setattr(serial, "Serial", lambda path, *args, **kwargs: by_path[path])
    source = backend.MasterSource(module_ports=paths)
    try:
        assert wait_for(lambda: len(source.modules) == 4)
        assert all(p.read_blocked.wait(2) for p in ports.values())
        yield source, ports
    finally:
        source._stop = True
        source._dispatch_wakeup.set()
        for port in ports.values():
            port.release_read.set()
            port.release_write.set()
        assert wait_for(lambda: not source.links)


def test_commands_and_stop_are_sent_without_waiting_for_telemetry(held_wheels):
    source, ports = held_wheels
    assert source._queue_motion(b"w 12\n")["ok"]
    assert wait_for(lambda: all(b"w 12\n" in p.values() for p in ports.values()))
    result = source.brake_for_control()
    command = ("b %d\n" % result["brakeId"]).encode()
    assert result["ok"]
    assert wait_for(lambda: all(command in p.values() for p in ports.values()))
    # None of the four blocked reads was released to make either send possible.
    assert all(not p.release_read.is_set() for p in ports.values())


def test_stop_clears_stale_queued_traction_while_writes_are_in_flight(held_wheels):
    source, ports = held_wheels
    for port in ports.values():
        port.block_value = b"v 20\n"
    assert source._queue_motion(b"v 20\n")["ok"]
    assert all(p.write_blocked.wait(2) for p in ports.values())
    assert source._queue_motion(b"v 99\n")["ok"]
    assert wait_for(lambda: all(b"v 99\n" in list(link.commands.queue) for link in source.links.values()))
    source._brake_all()
    for port in ports.values():
        port.release_write.set()
    assert wait_for(lambda: all(b"b\n" in p.values() for p in ports.values()))
    for port in ports.values():
        assert port.values() == [b"v 20\n", b"b\n"]


def test_write_failure_removes_frame_and_brakes_the_remaining_wheels(held_wheels):
    source, ports = held_wheels
    ports["FR"].fail_write = True
    with source.lock:
        source.drive_percent = 20
    assert source._queue_motion(b"v 20\n")["ok"]
    assert wait_for(lambda: "FR" not in source.modules and "FR" not in source.frames)
    assert wait_for(lambda: all(b"b\n" in ports[m].values() for m in ("FL", "RL", "RR")))
    assert source.drive_percent == 0
    assert ports["FR"].closed


def test_dispatch_queue_wakes_even_when_idle_and_does_not_accelerate_maneuver_ticks(monkeypatch):
    pytest.importorskip("serial")
    backend = slam.load_backend()
    dispatched = threading.Event()
    ticks = []
    original = backend.MasterSource._fanout_commands

    def fanout(source):
        if not source.commands.empty():
            dispatched.set()
        return original(source)

    def tick(source):
        ticks.append(time.monotonic())

    monkeypatch.setattr(backend.MasterSource, "_fanout_commands", fanout)
    monkeypatch.setattr(backend.MasterSource, "_maneuver_tick", tick)
    source = backend.MasterSource(module_ports={})
    try:
        assert wait_for(lambda: len(ticks) > 0)
        # No links: old dispatcher would sleep for RETRY_S = 2 seconds.
        source.commands.put_nowait(b"p\n")
        assert dispatched.wait(0.5)
        # Frequent wakeups must not cause the maneuver ramp to run faster.
        end = time.monotonic() + 0.3
        while time.monotonic() < end:
            try:
                source.commands.put_nowait(b"p\n")
            except queue.Full:
                pass
            time.sleep(0.004)
        assert len(ticks) >= 4
        assert all(b - a >= 0.045 for a, b in zip(ticks, ticks[1:])), ticks
    finally:
        source._stop = True
        source._dispatch_wakeup.set()


def test_brake_never_writes_to_unidentified_port():
    backend = slam.load_backend()
    source = object.__new__(backend.MasterSource)
    source.lock = threading.Lock()
    source._commands_lock = threading.RLock()
    source.commands = queue.Queue()
    source.drive_percent = 0
    source.drive_modules = {}
    source.servo_commands = {}
    source.drive_last_command = 0
    unknown = backend._WheelLink("/dev/unknown")
    wheel = backend._WheelLink("/dev/wheel")
    wheel.module = "FL"
    source.links = {unknown.path: unknown, wheel.path: wheel}
    source._brake_all(123)
    assert unknown.commands.empty()
    assert wheel.commands.get_nowait() == b"b 123\n"


def test_short_write_is_an_error_and_never_flushes():
    backend = slam.load_backend()

    class ShortPort:
        def write(self, data):
            return len(data) - 1

        def flush(self):
            raise AssertionError("flush must not be used")

    with pytest.raises(OSError, match="неполная"):
        backend.MasterSource._write_link_command(ShortPort(), b"b 123\n")


def test_chunk_reader_handles_fragments_and_discards_entire_oversized_lines():
    backend = slam.load_backend()
    source = object.__new__(backend.MasterSource)
    link = backend._WheelLink("/dev/fake")
    lines = []
    source._link_line = lambda _link, line: lines.append(line) or line.startswith("@TLM ")
    assert not source._link_bytes(link, b"@TL")
    assert not source._link_bytes(link, b"M drive=0")
    assert source._link_bytes(link, b"\r\nnoise\n@TLM drive=1\n")
    assert lines == ["@TLM drive=0\r", "noise", "@TLM drive=1"]
    lines.clear()
    assert not source._link_bytes(link, b"x" * (source.MAX_LINE_BYTES + 1) + b"@TLM fake=1\n")
    assert not lines  # no valid-looking suffix from corrupt data
    assert source._link_bytes(link, b"@TLM good=1\n")
    assert lines == ["@TLM good=1"]

    class Port:
        def __init__(self, waiting):
            self.in_waiting, self.requested = waiting, None

        def read(self, amount):
            self.requested = amount
            return b"x"

    empty, full = Port(0), Port(5000)
    backend.MasterSource._read_link_chunk(empty)
    backend.MasterSource._read_link_chunk(full)
    assert empty.requested == 1 and full.requested == 1024


def make_v20_frames(source):
    with source.lock:
        for module, (frame, stamp) in source.frames.items():
            frame.update(firmware=20, liveId=0, liveSeq=0, scRecent=False,
                         neutralId=0, neutralLocked=False)
            source.frames[module] = (frame, stamp)


def test_live_commands_are_addressed_expiring_and_stop_cancels_steering(held_wheels):
    source, ports = held_wheels
    make_v20_frames(source)
    angles = {"FL": 25, "FR": 25, "RL": -25, "RR": -25}
    assert source.keyboard_live(55, 7, 20, angles, time.monotonic() + 0.4)["ok"]
    assert wait_for(lambda: all(p.values() for p in ports.values()))
    for module, port in ports.items():
        assert port.values() == [("j 55 7 20 %.1f\n" % angles[module]).encode()]
    result = source.brake_for_keyboard()
    assert result["ok"]
    stop_and_brake = ("s q\nb %d\n" % result["brakeId"]).encode()
    assert wait_for(lambda: all(stop_and_brake in p.values() for p in ports.values()))
    assert all(p.values()[-1] == stop_and_brake for p in ports.values())


def test_live_queue_full_brakes_every_board_and_discards_stale_commands(held_wheels):
    source, ports = held_wheels
    make_v20_frames(source)
    link = next(link for link in source.links.values() if link.module == "FR")
    ports["FR"].block_value = b"hold\n"
    link.commands.put_nowait(b"hold\n")
    assert ports["FR"].write_blocked.wait(2)
    for _ in range(link.commands.maxsize):
        link.commands.put_nowait(b"stale\n")
    angles = {m: 0 for m in MODULES}
    result = source.keyboard_live(55, 7, 20, angles, time.monotonic() + 0.4)
    assert result["reason"] == "command_queue_full" and source.drive_percent == 0
    ports["FR"].release_write.set()
    assert wait_for(lambda: all(b"b\n" in p.values() for p in ports.values()))
    for port in ports.values():
        assert b"stale\n" not in port.values()
        assert not any(v.startswith(b"j ") for v in port.values()[port.values().index(b"b\n") + 1:])


def test_live_refuses_outdated_board_and_expired_work(held_wheels):
    source, ports = held_wheels
    make_v20_frames(source)
    with source.lock:
        source.frames["RL"][0]["firmware"] = 19
    angles = {m: 0 for m in MODULES}
    assert source.keyboard_live(9, 1, 20, angles, time.monotonic() + 0.4)["reason"] == "live_firmware_required"
    assert wait_for(lambda: all(b"b\n" in p.values() for p in ports.values()))
    assert all(not any(v.startswith(b"j ") for v in p.values()) for p in ports.values())
    assert source.keyboard_live(9, 2, 20, angles, time.monotonic() - 1)["reason"] == "command_expired"


def test_live_deadline_rechecked_after_waiting_for_command_lock(held_wheels):
    source, ports = held_wheels
    make_v20_frames(source)
    response = []
    with source._commands_lock:
        worker = threading.Thread(target=lambda: response.append(source.keyboard_live(
            9, 1, 20, {m: 0 for m in MODULES}, time.monotonic() + 0.05)))
        worker.start()
        time.sleep(0.10)
    worker.join(timeout=2)
    assert response == [{"ok": False, "reason": "command_expired"}]
    assert source.drive_percent == 0
    assert all(not p.values() for p in ports.values())
