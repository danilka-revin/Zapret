"""End-to-end USB discovery with firmware simulators and real pseudo terminals.

These tests exercise routing, shared locks and failure handling, not the safety
of a physical robot, Arduino reset timings or actual BMS/lidar hardware.
"""
from __future__ import annotations

import json
import os
import select
import shutil
import subprocess
import threading
import time
import traceback
import tty
from pathlib import Path

import pytest

from robot_control import battery, lidar, slam

ROOT = Path(__file__).resolve().parents[1]
MODULES = ("FL", "FR", "RL", "RR")
pytestmark = pytest.mark.skipif(os.name != "posix", reason="requires POSIX PTYs and flock")


def wait_for(predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(.02)
    return bool(predicate())


@pytest.fixture(scope="module")
def wheel_simulator(tmp_path_factory):
    compiler = shutil.which(os.environ.get("CXX", "g++"))
    if not compiler:
        pytest.skip("C++ compiler unavailable")
    out = tmp_path_factory.mktemp("usb-wheel-sim") / "sim"
    subprocess.run([compiler, "-std=c++17", "-O1", "-w", "-I", str(ROOT / "tests/firmware_sim/stubs"),
                    str(ROOT / "tests/firmware_sim/sim.cpp"), "-o", str(out)],
                   check=True, capture_output=True, timeout=180)
    return out


@pytest.fixture
def boards(wheel_simulator, tmp_path):
    processes = []

    def launch(name="wheels"):
        prefix = str(tmp_path / name)
        process = subprocess.Popen([str(wheel_simulator), "pty", prefix],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        processes.append(process)
        paths = {m: prefix + "-" + m for m in MODULES}
        assert wait_for(lambda: all(os.path.exists(path) for path in paths.values()), 5)
        return paths

    yield launch
    for process in processes:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)


@pytest.fixture
def usb_context(tmp_path, monkeypatch):
    pytest.importorskip("serial")
    backend = slam.load_backend()
    data = tmp_path / "data"
    monkeypatch.setattr(lidar, "DATA_DIR", data)
    monkeypatch.setenv("RC_USB_DISCOVERY_MODE", "1")
    monkeypatch.setenv("RC_USB_DISCOVERY_DIR", str(data / "ports"))
    monkeypatch.setenv("RC_USB_LOCK_DIR", str(data))
    monkeypatch.setenv("RC_LIDAR_PEER_URLS", "")
    # Short windows keep PTY tests fast; the production passive window is 5 s.
    monkeypatch.setattr(backend.MasterSource, "PROBE_S", .45)
    monkeypatch.setattr(backend.MasterSource, "AUTO_RESCAN_S", .10)
    monkeypatch.setattr(backend.MasterSource, "RETRY_S", .20)
    sources = []

    def start(paths, blocked=None):
        source = backend.MasterSource(
            auto_discovery=True, port_provider=lambda: list(paths),
            blocked_ports=blocked or (lambda: set()),
            claim=lidar.try_lock_port, release=lidar.unlock_port,
            tag_module=lidar.tag_module_port, untag_module=lidar.untag_module_port,
            publish_discovery=lidar.publish_usb_scan,
        )
        sources.append(source)
        return source

    yield backend, start
    for source in sources:
        source._stop = True
        source._dispatch_wakeup.set()
        with source.lock:
            links = list(source.links.values())
        for link in links:
            if link.serial:
                try:
                    link.serial.cancel_read()
                    link.serial.close()
                except Exception:
                    pass
        wait_for(lambda: not source.links and not source._spawning, 3)
    battery._set_auto_port("")


class Device:
    """A device's PTY endpoint, with serial input recorded for assertions."""
    def __init__(self):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        self.path = os.ttyname(self.slave)
        self.stop_flag = threading.Event()
        self.received = bytearray()
        self.thread = None

    def start(self, target):
        self.thread = threading.Thread(target=target, daemon=True)
        self.thread.start()
        return self

    def read(self):
        try:
            ready, _, _ = select.select([self.master], [], [], .003)
            data = os.read(self.master, 4096) if ready else b""
        except (OSError, BlockingIOError):
            data = b""
        self.received.extend(data)
        return data

    def write(self, data):
        try:
            os.write(self.master, data)
        except (OSError, BlockingIOError):
            pass

    def close(self):
        self.stop_flag.set()
        if self.thread:
            self.thread.join(timeout=2)
        os.close(self.slave)
        os.close(self.master)


def jbd_response(register, payload, status=0):
    body = bytes([status, len(payload)]) + payload
    checksum = (-sum(body)) & 0xFFFF
    return bytes([0xDD, register]) + body + checksum.to_bytes(2, "big") + b"\x77"


def bms_device(*, partial=False):
    dev = Device()
    basic = bytearray(23)
    basic[0:2] = (5120).to_bytes(2, "big")
    basic[18:23] = bytes([0x21, 55, 3, 4, 0])
    cells = b"".join(v.to_bytes(2, "big") for v in (3200, 3205, 3210, 3215))

    def run():
        buffer = b""
        while not dev.stop_flag.is_set():
            buffer += dev.read()
            while len(buffer) >= 7:
                start = buffer.find(b"\xdd\xa5")
                if start < 0:
                    buffer = buffer[-1:]
                    break
                if len(buffer) - start < 7:
                    buffer = buffer[start:]
                    break
                frame, buffer = buffer[start:start + 7], buffer[start + 7:]
                reg = frame[2]
                if reg == 3:
                    dev.write(jbd_response(reg, basic))
                elif reg == 4 and not partial:
                    dev.write(jbd_response(reg, cells))
                elif reg != 4:
                    dev.write(jbd_response(reg, b"", status=0x80))
    return dev.start(run)


def lidar_device():
    dev = Device()

    def packet(index):
        pkt = bytearray(42)
        pkt[0], pkt[1] = 0xFA, 0xA0 + index % 60
        pkt[2:4] = (300).to_bytes(2, "little")
        for i in range(6):
            off = 4 + i * 6
            pkt[off:off + 2] = (200).to_bytes(2, "little")
            pkt[off + 2:off + 4] = (1500 + i).to_bytes(2, "little")
        pkt[40:42] = lidar.lds01_checksum(bytes(pkt[:40])).to_bytes(2, "little")
        return bytes(pkt)

    def run():
        index = 0
        while not dev.stop_flag.is_set():
            dev.read()
            dev.write(packet(index))
            index += 1
            time.sleep(.003)
    return dev.start(run)


def legacy_wheel(backend):
    dev = Device()
    values = [21, 0, 0, 0x3042, 0, 100, 0, 0, 0, 90, 0, 0, 0, 0, 0, 0, 0, 5]
    payload = "@F " + " ".join(str(v) for v in values)
    frame = (f"{payload} {backend.crc16(payload.encode()):04X}\r\n").encode()

    def run():
        while not dev.stop_flag.is_set():
            dev.read()
            dev.write(frame)
            time.sleep(.03)
    return dev.start(run)


def test_roles_survive_usb_reordering_and_commands_target_the_verified_board(boards, usb_context, monkeypatch, tmp_path):
    backend, start = usb_context
    wheel = boards()
    aliases = [tmp_path / ("usb-%d" % i) for i in range(4)]
    order = ["RR", "FL", "RL", "FR"]
    for alias, role in zip(aliases, order):
        alias.symlink_to(wheel[role])
    paths = [str(x) for x in aliases]
    writes = []
    original = backend.MasterSource._write_link_command

    def record(dev, command):
        writes.append((dev.port, bytes(command)))
        return original(dev, command)

    monkeypatch.setattr(backend.MasterSource, "_write_link_command", staticmethod(record))
    source = start(paths)
    assert wait_for(lambda: len(source.modules) == 4 and source.serial_ready.is_set())
    assert source.modules == {role: str(alias) for alias, role in zip(aliases, order)}
    assert source.zero_module("FR")["ok"]
    assert wait_for(lambda: any(path == str(aliases[3]) and cmd == b"w zero\n" for path, cmd in writes))
    assert not any(cmd == b"w zero\n" and path != str(aliases[3]) for path, cmd in writes)

    # Replug simulation: aliases now resolve to different actual boards. The
    # old open handles are invalidated before the new mapping is accepted.
    new_order = ["FL", "RR", "FR", "RL"]
    for alias, role in zip(aliases, new_order):
        replacement = alias.with_suffix(".new")
        replacement.symlink_to(wheel[role])
        os.replace(replacement, alias)
    assert wait_for(lambda: source.modules == {role: str(alias) for alias, role in zip(aliases, new_order)}
                    and source.serial_ready.is_set(), 10), (source.modules, source.identity_fault, source.last_error)
    assert not source.identity_fault


def test_duplicate_identity_latches_and_stops_every_verified_wheel(boards, usb_context, monkeypatch):
    backend, start = usb_context
    wheel, extra = boards("first"), boards("second")
    paths = list(wheel.values())
    writes = []
    original = backend.MasterSource._write_link_command

    def record(dev, command):
        writes.append((dev.port, bytes(command), time.monotonic()))
        return original(dev, command)

    monkeypatch.setattr(backend.MasterSource, "_write_link_command", staticmethod(record))
    source = start(paths)
    assert wait_for(lambda: len(source.modules) == 4 and source.serial_ready.is_set())
    paths.append(extra["FL"])
    assert wait_for(lambda: bool(source.identity_fault), 5)
    assert "FL" in source.identity_fault and "Две платы" in source.identity_fault
    assert not source.serial_ready.is_set() and not source.frames
    assert source.drive_servo(20)["reason"] == "serial_unavailable"
    assert wait_for(lambda: {path for path, cmd, _ in writes if cmd == b"s q\nb\n"} >= set(paths), 5)
    original_error = source.identity_fault
    time.sleep(.25)
    assert source.identity_fault == original_error  # no repeated suffix on new frames
    paths.remove(extra["FL"])
    time.sleep(.4)
    assert not source.serial_ready.is_set()  # explicitly latched until restart


def test_fw21_wheel_is_never_assigned_or_sent_commands(boards, usb_context):
    backend, start = usb_context
    old = legacy_wheel(backend)
    try:
        wheel = boards()
        source = start(list(wheel.values()) + [old.path])
        assert wait_for(lambda: any(link.legacy_wheel for link in list(source.links.values())), 5)
        assert not source.serial_ready.is_set()
        assert all(link.module is None for link in source.links.values() if link.path == old.path)
        assert source.drive_servo(20)["reason"] == "serial_unavailable"
        time.sleep(.2)
        assert not old.received  # no STOP or probe written to unidentified firmware
    finally:
        old.close()


def test_bms_and_python_lidar_share_discovery_without_touching_wheels(boards, usb_context, monkeypatch):
    backend, start = usb_context
    wheel = boards()
    bms, laser = bms_device(), lidar_device()
    manager = None
    battery_manager = battery.BatteryManager()
    owner_lock = threading.Lock()
    owners = {}
    original_claim, original_release = lidar.try_lock_port, lidar.unlock_port
    def tracking_claim(port):
        fd = original_claim(port)
        if fd not in (None, -1):
            with owner_lock:
                owners[fd] = {"port": port, "thread": threading.current_thread().name,
                              "stack": traceback.format_stack(limit=5)}
        return fd
    def tracking_release(fd):
        with owner_lock:
            owners.pop(fd, None)
        original_release(fd)
    monkeypatch.setattr(lidar, "try_lock_port", tracking_claim)
    monkeypatch.setattr(lidar, "unlock_port", tracking_release)
    try:
        paths = [laser.path, wheel["RR"], bms.path, wheel["FL"], wheel["RL"], wheel["FR"]]
        source = start(paths, blocked=lambda: {lidar.lidar_active_port()} - {None})
        battery_manager.enable_auto(source)
        battery_manager.start()
        assert not source.serial_ready.is_set()  # unknown USBs must be classified first
        assert wait_for(lambda: battery_manager.snapshot()["connected"], 8), battery_manager.snapshot()
        assert wait_for(lambda: os.path.realpath(laser.path) in
                        {os.path.realpath(p) for p in lidar.checked_lidar_ports()}, 8)
        assert wait_for(source.serial_ready.is_set, 5)
        assert battery_manager.snapshot()["port"] == bms.path
        assert {os.path.realpath(p) for p in lidar.checked_lidar_ports()} == {os.path.realpath(laser.path)}
        assert all(os.path.realpath(p) not in {os.path.realpath(x) for x in wheel.values()}
                   for p in lidar.checked_usb_ports())
        assert lidar.try_lock_port(bms.path) is None  # BMS owns its lock

        # Stale manual hints must not steer the new search onto a wheel/BMS.
        monkeypatch.setenv("RC_LIDAR_SOURCE", "serial")
        monkeypatch.setenv("RC_LIDAR_PORT", wheel["FL"])
        monkeypatch.setenv("RC_LIDAR_PROTOCOL", "lds01")
        monkeypatch.setenv("RC_LIDAR_MOTOR", "none")
        monkeypatch.setattr(lidar, "LIDAR_RESCAN_S", .1)
        manager = lidar.LidarManager()
        failures = []
        acquire = manager._acquire_port_lock
        def diagnostic_acquire(port):
            if acquire(port):
                return True
            failures.append({"port": port, "locked": lidar.port_locked_by_other(port),
                             "allowed": lidar.checked_lidar_ports(),
                             "bms": sorted(battery.reserved_ports()),
                             "wheels": sorted(lidar.chassis_module_ports())})
            return False
        monkeypatch.setattr(manager, "_acquire_port_lock", diagnostic_acquire)
        manager.start()
        if not wait_for(lambda: len(manager.get_scan() or []) >= 300, 8):
            with owner_lock:
                current_owners = dict(owners)
            pytest.fail(json.dumps({"status": manager.get_status(), "last_failures": failures[-3:],
                                    "lock_owners": current_owners}, indent=2, default=str))

        assert manager.get_status()["detected"]["protocol"] == "lds01"
        assert os.path.realpath(manager.get_status()["detected"]["port"]) == os.path.realpath(laser.path)
        time.sleep(.5)  # its active-port tag must not remove classification
        assert battery_manager.snapshot()["connected"] and source.serial_ready.is_set()
        assert lidar.try_lock_port(laser.path) is None
        assert len(source.modules) == 4 and not source.identity_fault
    finally:
        if manager is not None:
            manager.stop()
        battery_manager.stop()
        bms.close()
        laser.close()


def test_two_bms_are_ambiguous_and_cannot_enable_motion(boards, usb_context):
    _, start = usb_context
    wheel = boards()
    one, two = bms_device(), bms_device()
    manager = battery.BatteryManager()
    try:
        source = start(list(wheel.values()) + [one.path, two.path])
        manager.enable_auto(source)
        manager.start()
        assert wait_for(lambda: "несколько" in manager.snapshot()["reason"], 8), manager.snapshot()
        assert not manager.snapshot()["connected"]
        assert not source.serial_ready.is_set()
        assert source.drive_servo(10)["reason"] == "serial_unavailable"
    finally:
        manager.stop()
        one.close()
        two.close()


def test_partial_jbd_response_is_not_a_negative_classification(usb_context):
    _, start = usb_context
    partial = bms_device(partial=True)
    manager = battery.BatteryManager()
    try:
        source = start([partial.path])
        manager.enable_auto(source)
        manager.start()
        assert wait_for(lambda: b"\xdd\xa5\x04" in partial.received, 5)
        time.sleep(1)
        assert partial.path not in source.auto_bms_results()
        assert partial.path not in lidar.checked_lidar_ports()
        assert not source.serial_ready.is_set()
    finally:
        manager.stop()
        partial.close()


def test_ros_and_python_allowlists_use_same_fingerprint_and_flock(usb_context, monkeypatch, tmp_path):
    _, _ = usb_context
    monkeypatch.syspath_prepend(str(ROOT / "ros2_ws/src/zapret_lidar"))
    from zapret_lidar import drivers, probe, usb_guard

    wheel, bms, laser = [tmp_path / name for name in ("wheel", "bms", "laser")]
    for path in (wheel, bms, laser):
        path.touch()
    def fp(path):
        st = path.stat()
        return (st.st_dev, st.st_ino, st.st_rdev, st.st_ctime_ns)
    lidar.publish_usb_scan([(str(bms), fp(bms), True), (str(laser), fp(laser), False)])
    assert drivers.candidate_ports([str(wheel), str(bms), str(laser)]) == [str(laser)]
    assert usb_guard.permitted(str(laser)) and not usb_guard.permitted(str(bms))
    assert all(item.port != str(wheel) for item in drivers.build_plan(ports=[str(wheel), str(laser)]))
    opened = []
    monkeypatch.setattr(probe, "_open", lambda port, baud, timeout: opened.append(port) or None)
    assert probe.read_bytes(str(wheel), 115200, window=.02) == b"" and not opened
    assert probe.read_bytes(str(laser), 115200, window=.02) == b"" and opened == [str(laser)]
    fd = lidar.try_lock_port(str(laser))
    assert fd not in (None, -1) and usb_guard.claim(str(laser)) is None
    lidar.unlock_port(fd)
    fd = usb_guard.claim(str(laser))
    assert fd not in (None, -1) and lidar.try_lock_port(str(laser)) is None
    usb_guard.release(fd)
    # Replace while the original inode still exists. unlink()+touch() may
    # immediately reuse both inode and ctime on a coarse-timestamp filesystem;
    # those metadata alone cannot prove an unplug/replug in all environments.
    replacement = tmp_path / "replacement-laser"
    replacement.touch()
    replacement.replace(laser)
    assert not usb_guard.permitted(str(laser))
    assert not lidar.checked_lidar_ports()


def test_state_publications_are_atomic_under_concurrency(usb_context, tmp_path):
    path = tmp_path / "state.json"
    errors = []
    done = threading.Event()

    def reader():
        while not done.is_set():
            try:
                value = json.loads(path.read_text())
                assert value["payload"] == "x" * 1000
            except FileNotFoundError:
                pass
            except Exception as exc:
                errors.append(exc)
                return

    thread = threading.Thread(target=reader)
    thread.start()
    writers = [threading.Thread(target=lambda n=n: [lidar._write_state_file(
        path, {"writer": n, "at": i, "payload": "x" * 1000}) for i in range(40)]) for n in range(6)]
    for worker in writers:
        worker.start()
    for worker in writers:
        worker.join(timeout=10)
    done.set()
    thread.join(timeout=2)
    assert not errors and json.loads(path.read_text())["payload"] == "x" * 1000
    assert not list(tmp_path.glob("state.json.*.tmp"))


def test_full_zero_end_to_end_with_four_fw23_pty_boards(boards, usb_context, kiosk_client, monkeypatch, tmp_path):
    """Real firmware encoders, USB queues, HTTP lease, EEPROM ACKs; no physical mechanics."""
    backend, start = usb_context
    monkeypatch.setenv("RC_FULL_ZERO_PENDING", str(tmp_path / "full-zero.pending"))
    wheel = boards("full-zero")
    writes = []
    original_write = backend.MasterSource._write_link_command
    def record_write(dev, command):
        writes.append((dev.port, bytes(command)))
        return original_write(dev, command)
    monkeypatch.setattr(backend.MasterSource, "_write_link_command", staticmethod(record_write))
    source = start([wheel[m] for m in ("RR", "FR", "FL", "RL")])
    assert wait_for(lambda: len(source.modules) == 4 and source.serial_ready.is_set(), 8)
    kiosk_client.application.extensions["rc_slam"].source = source
    endpoint = "/api/kiosk_full_zero"
    marker = {"X-Robot-Control": "steering"}

    def send(action, token=None, **kwargs):
        body = {"action": action, **kwargs}
        if token is not None:
            body["token"] = token
        reply = kiosk_client.post(endpoint, json=body, headers=marker)
        assert reply.is_json, reply.data
        return reply.status_code, reply.json

    code, result = send("begin")
    assert code == 200, result
    token = result["token"]

    def wait_phase(target, timeout=12):
        last = {}
        def check():
            nonlocal last
            code, last = send("status", token)
            assert code == 200 and last["phase"] not in {"stopping", "recovery", "idle"}, last
            return last["phase"] == target
        assert wait_for(check, timeout), last
        return last

    wait_phase("ready")
    # While reserved, every ordinary steering/traction entry point fails closed.
    assert kiosk_client.post("/api/kiosk_steer", json={"module": "FL", "angle": 1}, headers=marker).json["reason"] == "full_zero_active"
    assert kiosk_client.post("/api/kiosk_drive", json={"value": 5}, headers={"X-Robot-Control": "drive"}).json["reason"] == "full_zero_active"
    code, result = send("calibrate", token)
    assert code == 200, result
    result = wait_phase("adjust", timeout=40)
    assert set(result["calibratingSeen"]) == set(MODULES)
    for module, delta in zip(MODULES, (0.1, -0.1, 1.0, -1.0)):
        code, result = send("adjust", token, module=module, delta=delta)
        assert code == 200 and result["phase"] == "adjusting", result
        wait_phase("adjust")
        # The real serial writer sent the wheel move before the unique zq
        # barrier in a single FIFO item, never on a different physical role.
        assert any(port == wheel[module] and command.startswith(b"w ") and
                   b"\nzq " in command for port, command in writes)
    code, result = send("save", token)
    assert code == 200 and result["phase"] == "checking", result
    result = wait_phase("saved")
    assert set(result["acknowledged"]) == set(MODULES)
    assert not (tmp_path / "full-zero.pending").exists()
    assert not source._full_zero_inhibit
    # New zero was applied to the real simulated boards, not just the screen.
    assert wait_for(lambda: all(abs(m["angle"]) < .02 for m in source.read()["motors"]), 3)


@pytest.mark.skipif(not os.environ.get("ZAPRET_BROWSER_NODE_MODULES") or not shutil.which("node"),
                    reason="optional Chromium/playwright-core not installed")
def test_full_zero_real_browser_and_fw23(boards, usb_context, kiosk_client, monkeypatch, tmp_path):
    """Browser → kiosk → four PTYs → actual firmware and EEPROM acknowledgements."""
    from werkzeug.serving import WSGIRequestHandler, make_server
    _, start = usb_context
    monkeypatch.setenv("RC_FULL_ZERO_PENDING", str(tmp_path / "browser-full-zero.pending"))
    wheel = boards("browser-zero")
    source = start([wheel[m] for m in MODULES])
    assert wait_for(lambda: len(source.modules) == 4 and source.serial_ready.is_set(), 8)
    app = kiosk_client.application
    app.extensions["rc_slam"].source = source

    class Quiet(WSGIRequestHandler):
        def log_request(self, *args, **kwargs):
            pass

    server = make_server("0.0.0.0", 0, app, threaded=True, request_handler=Quiet)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        result = subprocess.run(["node", str(ROOT / "tests/js/full_zero_browser.mjs"),
                                 f"http://127.0.0.1:{server.server_port}"],
                                capture_output=True, text=True, timeout=150)
        assert result.returncode == 0, result.stdout + result.stderr
        assert wait_for(lambda: not source._full_zero_inhibit, 5)
        assert not (tmp_path / "browser-full-zero.pending").exists()
        assert wait_for(lambda: all(not m["calibrating"] and not m["moving"] for m in source.read()["motors"]), 5)
    finally:
        server.shutdown()
        thread.join(timeout=3)


@pytest.mark.skipif(not os.environ.get("ZAPRET_BROWSER_NODE_MODULES") or not shutil.which("node"),
                    reason="optional Chromium/playwright-core not installed")
def test_full_zero_remote_browser_and_fw23(boards, usb_context, kiosk_client,
                                           app, operator, monkeypatch, tmp_path):
    """Authenticated web page → same-origin proxy → onboard owner → four boards."""
    from werkzeug.serving import WSGIRequestHandler, make_server
    _, start = usb_context
    monkeypatch.setenv("RC_FULL_ZERO_PENDING", str(tmp_path / "remote-full-zero.pending"))
    wheel = boards("remote-zero")
    source = start([wheel[m] for m in MODULES])
    assert wait_for(lambda: len(source.modules) == 4 and source.serial_ready.is_set(), 8)
    kiosk_client.application.extensions["rc_slam"].source = source

    class Quiet(WSGIRequestHandler):
        def log_request(self, *args, **kwargs):
            pass

    onboard = make_server("0.0.0.0", 0, kiosk_client.application, threaded=True, request_handler=Quiet)
    monkeypatch.setenv("RC_SLAM_PEER_URL", f"http://127.0.0.1:{onboard.server_port}/api/kiosk_state")
    remote = make_server("0.0.0.0", 0, app, threaded=True, request_handler=Quiet)
    threads = [threading.Thread(target=server.serve_forever, daemon=True)
               for server in (onboard, remote)]
    for thread in threads:
        thread.start()
    try:
        result = subprocess.run(["node", str(ROOT / "tests/js/full_zero_browser.mjs"),
                                 f"http://127.0.0.1:{remote.server_port}", "remote"],
                                capture_output=True, text=True, timeout=150)
        assert result.returncode == 0, result.stdout + result.stderr
        assert wait_for(lambda: not source._full_zero_inhibit, 5)
        assert not (tmp_path / "remote-full-zero.pending").exists()
        assert wait_for(lambda: all(not m["calibrating"] and not m["moving"] for m in source.read()["motors"]), 5)
    finally:
        remote.shutdown()
        onboard.shutdown()
        for thread in threads:
            thread.join(timeout=3)
