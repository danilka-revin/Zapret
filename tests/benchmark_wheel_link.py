"""Measure wheel dispatch on four fake PTYs, optionally with the firmware simulator.

This never opens real serial ports. PTYs have no USB adapter or physical brake;
--wire-baud optionally adds an ideal 8N1 FIFO, not real USB/driver timing.

    PYTHONPATH=. .venv/bin/python tests/benchmark_wheel_link.py
    g++ -std=c++17 -O1 -w -I tests/firmware_sim/stubs tests/firmware_sim/sim.cpp -o /tmp/wheel-sim
    PYTHONPATH=. .venv/bin/python tests/benchmark_wheel_link.py --simulator /tmp/wheel-sim --wire-baud 115200 --keyboard
"""
from __future__ import annotations

import argparse
import os
import random
import select
import statistics
import subprocess
import tempfile
import threading
import time
import tty
from collections import defaultdict
from pathlib import Path

from robot_control import slam

MODULES = ("FL", "FR", "RL", "RR")
FRAME = (b"@TLM deg=0 tgt=0 moving=0 cal=1 cycle=0 calib=0 opto=0 home=manual "
         b"err=0 drive=0 fw=17 pot=0 pm=0 tach=0 dmax=100 sv=0 duty=0 "
         b"sc_rpm=0.0 sc_hz=0 sc_ppr=90 sc_count=0 brake_id=0 t=1\r\n")


def wait_for(predicate, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.001)
    return bool(predicate())


def report(label, samples):
    values = sorted(x * 1000 for x in samples)
    p90 = values[max(0, (len(values) * 90 + 99) // 100 - 1)]
    print(f"{label}: n={len(values)}, median={statistics.median(values):.1f} ms, "
          f"p90={p90:.1f} ms, max={values[-1]:.1f} ms")


def dispatch_benchmark(count):
    boards, paths = {}, {}
    for module in MODULES:
        master, slave = os.openpty()
        boards[module] = master, slave
        paths[module] = os.ttyname(slave)
    stop, lock = threading.Event(), threading.Lock()
    received = defaultdict(dict)
    threads = []

    def emulate(module):
        fd = boards[module][0]
        next_frame, buffer = time.monotonic(), b""
        while not stop.is_set():
            now = time.monotonic()
            if now >= next_frame:
                try:
                    os.write(fd, FRAME)
                except OSError:
                    return
                next_frame = now + 0.1
            try:
                ready, _, _ = select.select([fd], [], [], min(0.01, max(0, next_frame - now)))
                if not ready:
                    continue
                buffer += os.read(fd, 4096)
            except OSError:
                return
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                if line.strip().startswith(b"b "):
                    try:
                        ident = int(line.strip()[2:])
                    except ValueError:
                        continue
                    with lock:
                        received[ident][module] = time.monotonic()

    source = None
    try:
        for module in MODULES:
            thread = threading.Thread(target=emulate, args=(module,), daemon=True)
            thread.start()
            threads.append(thread)
        source = slam.load_backend().MasterSource(module_ports=paths)
        if not wait_for(lambda: len(source.modules) == 4):
            raise RuntimeError("fake wheel ports were not identified")
        rng = random.Random(12)
        for kind in ("queued dispatch", "priority brake"):
            samples = []
            for i in range(count):
                time.sleep(rng.uniform(0.01, 0.04))
                if kind == "queued dispatch":
                    ident = 1_000_000 + i
                    started = time.monotonic()
                    reply = source._queue_motion(f"b {ident}\n".encode())
                else:
                    started = time.monotonic()
                    reply = source.brake_for_control()
                    ident = reply.get("brakeId")
                if not reply.get("ok"):
                    raise RuntimeError(str(reply))

                def all_received():
                    with lock:
                        return len(received[ident]) == 4

                if not wait_for(all_received, timeout=1):
                    raise TimeoutError(f"{kind}: not all four PTYs received the command")
                with lock:
                    samples.append(max(received[ident].values()) - started)
            report(kind + " → last PTY", samples)
    finally:
        if source is not None:
            source._stop = True
            source._dispatch_wakeup.set()
        stop.set()
        for thread in threads:
            thread.join(timeout=1)
        for master, slave in boards.values():
            os.close(master)
            os.close(slave)


class WireRateRelay:
    """Idealised full-duplex 8N1 serialisation between two PTYs.

    This models only baud-limited wire time, not USB packets, driver buffers,
    electronics or motor behaviour. Used exclusively with the simulator.
    """

    def __init__(self, board_path, baud):
        self.board_fd = os.open(board_path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(self.board_fd)
        self.master_fd, self.slave_fd = os.openpty()
        tty.setraw(self.slave_fd)
        os.set_blocking(self.master_fd, False)
        self.host_path = os.ttyname(self.slave_fd)
        self.byte_seconds = 10.0 / baud
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        # Each direction has its own FIFO and independent 8N1 bit clock.
        directions = {self.board_fd: (self.master_fd, bytearray(), None),
                      self.master_fd: (self.board_fd, bytearray(), None)}
        while not self.stop_event.is_set():
            ready, _, _ = select.select(list(directions), [], [], 0.001)
            now = time.monotonic()
            for fd in ready:
                try:
                    chunk = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                target, pending, due = directions[fd]
                if chunk:
                    if not pending:
                        due = max(now, due or now) + self.byte_seconds
                    pending.extend(chunk)
                    directions[fd] = target, pending, due
            now = time.monotonic()
            for fd, (target, pending, due) in list(directions.items()):
                if not pending or due is None or now < due:
                    continue
                size = min(len(pending), 1 + int((now - due) / self.byte_seconds))
                try:
                    written = os.write(target, pending[:size])
                except BlockingIOError:
                    continue
                if written:
                    del pending[:written]
                    directions[fd] = target, pending, due + written * self.byte_seconds

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2)
        for fd in (self.board_fd, self.master_fd, self.slave_fd):
            os.close(fd)


def ack_benchmark(binary, count, wire_baud=None, keyboard=False):
    with tempfile.TemporaryDirectory() as tmp:
        base = str(Path(tmp) / "wheel")
        process = subprocess.Popen([binary, "pty", base], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        source = None
        relays = []
        try:
            paths = {m: f"{base}-{m}" for m in MODULES}
            if not wait_for(lambda: all(Path(p).exists() for p in paths.values())):
                raise RuntimeError("firmware simulator did not open its four PTYs")
            if wire_baud:
                relays = [WireRateRelay(paths[m], wire_baud) for m in MODULES]
                paths = {m: relay.host_path for m, relay in zip(MODULES, relays)}
            source = slam.load_backend().MasterSource(module_ports=paths)
            if not wait_for(lambda: len(source.modules) == 4, timeout=8):
                raise RuntimeError("simulated boards were not identified")
            rng, samples = random.Random(123), []
            for _ in range(count):
                time.sleep(rng.uniform(0.01, 0.04))
                started = time.monotonic()
                reply = source.brake_for_keyboard() if keyboard else source.brake_for_control()
                if not reply.get("ok"):
                    raise RuntimeError(str(reply))
                ident = reply["brakeId"]

                def all_acked():
                    with source.lock:
                        return all(source.frames.get(m) and source.frames[m][0].get("brakeId") == ident
                                   for m in MODULES)

                if not wait_for(all_acked, timeout=1):
                    raise TimeoutError("not all four simulated boards acknowledged the brake")
                samples.append(time.monotonic() - started)
            report(("keyboard STOP" if keyboard else "brake") + " → four firmware ACKs", samples)
        finally:
            if source is not None:
                source._stop = True
                source._dispatch_wakeup.set()
            for relay in relays:
                relay.close()
            process.terminate()
            process.wait(timeout=3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=30)
    parser.add_argument("--simulator", help="path to the compiled firmware simulator")
    parser.add_argument("--wire-baud", type=int, help="model ideal 8N1 wire rate on the ACK test, e.g. 115200")
    parser.add_argument("--keyboard", action="store_true", help="measure steering-stop plus tagged brake")
    args = parser.parse_args()
    if not 1 <= args.count <= 1000:
        parser.error("count must be 1..1000")
    if args.wire_baud is not None and (not args.simulator or not 1200 <= args.wire_baud <= 1000000):
        parser.error("--wire-baud requires --simulator and must be 1200..1000000")
    print("Simulation only: no real USB, UART hardware or motors are involved.")
    if args.wire_baud:
        print(f"ACK test includes an ideal {args.wire_baud}-baud 8N1 FIFO, not USB latency.")
    dispatch_benchmark(args.count)
    if args.simulator:
        ack_benchmark(args.simulator, args.count, args.wire_baud, args.keyboard)


if __name__ == "__main__":
    main()
