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

from robot_control import battery, bms_setup, lidar, master_port, slam

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

    def launch(name="wheels", *, bad_zero="", fail_eeprom="", pot_reverse="", pot_control=None):
        prefix = str(tmp_path / name)
        env = os.environ.copy()
        if bad_zero:
            env["RC_SIM_BAD_ZERO"] = bad_zero
        if fail_eeprom:
            env["RC_SIM_EEPROM_FAIL"] = fail_eeprom
        if pot_reverse:
            env["RC_SIM_POT_REVERSE"] = pot_reverse
        if pot_control is not None:
            env["RC_SIM_POT_CONTROL_DIR"] = str(pot_control)
        process = subprocess.Popen([str(wheel_simulator), "pty", prefix], env=env,
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
            passive_lidar_probe=lidar.passive_lidar_probe,
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


def test_master_discovery_ignores_stale_serial_mode_ports(boards, usb_context, monkeypatch):
    """An old serial configuration must not hide CRC-identifying wheels in master mode."""
    _, start = usb_context
    wheel = boards("old-serial-config")
    monkeypatch.setenv("RC_SLAM_PORTS", ",".join(wheel.values()))
    monkeypatch.delenv("RC_LIDAR_EXCLUDE", raising=False)
    monkeypatch.setattr(lidar, "lidar_active_port", lambda: None)
    source = start(list(wheel.values()), blocked=master_port.autodiscovery_reserved_ports)
    assert wait_for(lambda: len(source.modules) == 4 and source.serial_ready.is_set(), 4), (
        source.modules, source.last_error, master_port.autodiscovery_reserved_ports())
    assert source.modules == wheel


def test_explicit_exclusion_is_visible_and_connects_after_operator_removes_it(boards, usb_context, monkeypatch):
    _, start = usb_context
    wheel = boards("excluded")
    monkeypatch.setattr(lidar, "lidar_active_port", lambda: None)
    monkeypatch.setenv("RC_LIDAR_EXCLUDE", wheel["FR"])
    source = start(list(wheel.values()), blocked=master_port.autodiscovery_reserved_ports)
    assert wait_for(lambda: len(source.modules) == 3 and wheel["FR"] in source.read()["masterPort"]["excluded"])
    assert "FR" not in source.read()["masterPort"]["modules"]
    assert not next(m for m in source.read()["motors"] if m["id"] == "FR")["port"]
    monkeypatch.delenv("RC_LIDAR_EXCLUDE")
    assert wait_for(lambda: len(source.modules) == 4 and source.serial_ready.is_set())
    assert wheel["FR"] not in source.read()["masterPort"]["excluded"]


def test_all_excluded_ports_are_not_reported_as_unclassified_candidates(boards, usb_context, monkeypatch):
    _, start = usb_context
    wheel = boards("all-excluded")
    monkeypatch.setattr(lidar, "lidar_active_port", lambda: None)
    monkeypatch.setenv("RC_LIDAR_EXCLUDE", ",".join(wheel.values()))
    source = start(list(wheel.values()), blocked=master_port.autodiscovery_reserved_ports)
    assert wait_for(lambda: len(source.read()["masterPort"]["excluded"]) == 4)
    info = source.read()["masterPort"]
    assert info["candidates"] == []
    assert info["unclassified"] == []
    assert not source.serial_ready.is_set()


def test_new_exclusion_also_removes_a_previously_silent_unknown_device(boards, usb_context, monkeypatch):
    _, start = usb_context
    wheel = boards("late-exclusion")
    silent = Device().start(lambda: None)
    monkeypatch.setattr(lidar, "lidar_active_port", lambda: None)
    monkeypatch.delenv("RC_LIDAR_EXCLUDE", raising=False)
    try:
        source = start(list(wheel.values()) + [silent.path], blocked=master_port.autodiscovery_reserved_ports)
        assert wait_for(lambda: silent.path in source.passively_checked_unknown_ports(), 6)
        assert not source.serial_ready.is_set()
        monkeypatch.setenv("RC_LIDAR_EXCLUDE", silent.path)
        assert wait_for(lambda: silent.path in source.read()["masterPort"]["excluded"], 6)
        assert silent.path not in source.read()["masterPort"]["candidates"]
        # A forced rescan cannot open the excluded device or write to it.
        source._open_new_links()
        assert silent.path not in source.links and silent.path not in source._spawning
        assert not silent.read() and not silent.received
    finally:
        silent.close()


def test_busy_usb_is_reported_then_recovers_without_manual_assignment(boards, usb_context):
    _, start = usb_context
    wheel = boards("busy")
    token = lidar.try_lock_port(wheel["RL"])
    assert token not in (None, -1)
    try:
        source = start(list(wheel.values()))
        assert wait_for(lambda: "занят" in (source.read()["masterPort"]["error"] or ""), 6)
        assert wheel["RL"] in source.read()["masterPort"]["unclassified"]
        assert not source.serial_ready.is_set()
    finally:
        lidar.unlock_port(token)
    assert wait_for(lambda: len(source.modules) == 4 and source.serial_ready.is_set(), 8)
    assert source.read()["masterPort"]["error"] is None


def test_confirmed_ports_survive_owner_api_and_peer_and_disappear_when_peer_stale(
        boards, usb_context, kiosk_client):
    backend, start = usb_context
    wheel = boards("api-and-peer")
    source = start(list(wheel.values()))
    assert wait_for(lambda: len(source.modules) == 4 and source.serial_ready.is_set())
    kiosk_client.application.extensions["rc_slam"].source = source
    response = kiosk_client.get("/api/kiosk_state")
    assert response.status_code == 200
    data = response.get_json()["data"]
    assert {m["id"]: m["port"] for m in data["motors"]} == {
        role: os.path.realpath(path) for role, path in wheel.items()}
    assert data["masterPort"]["modules"] == wheel

    peer = backend.MasterPeerSource("owner", fetch=lambda: {"data": source.read()})
    assert wait_for(lambda: peer.read().get("masterPort") is not None)
    peer_data = peer.read()
    assert {m["id"]: m["port"] for m in peer_data["motors"]} == {
        role: os.path.realpath(path) for role, path in wheel.items()}
    # Once the owner stops answering, a peer must not keep old USB paths alive.
    peer._fetch = lambda: (_ for _ in ()).throw(OSError("owner gone"))
    with peer.lock:
        peer.snapshot_time = time.time() - peer.STALE_S - 1
    stale = peer.read()
    assert stale.get("masterPort") is None
    assert all(m.get("port") is None and not m["online"] for m in stale["motors"])


def test_master_ignores_old_serial_list_but_preserves_explicit_usb_exclusions(usb_context, monkeypatch, tmp_path):
    """Old lists can't mask classified peripherals; explicit excludes and locks remain."""
    _, _ = usb_context
    monkeypatch.syspath_prepend(str(ROOT / "ros2_ws/src/zapret_lidar"))
    from zapret_lidar import drivers

    wheel, laser, excluded = [tmp_path / name for name in ("wheel", "laser", "excluded")]
    for path in (wheel, laser, excluded):
        path.touch()
    monkeypatch.setenv("RC_SLAM_PORTS", f"{wheel},{laser}")
    monkeypatch.setenv("RC_LIDAR_EXCLUDE", str(excluded))
    monkeypatch.setattr(lidar, "lidar_active_port", lambda: None)
    assert str(wheel) not in master_port.autodiscovery_reserved_ports()
    assert str(excluded) in master_port.autodiscovery_reserved_ports()
    assert str(laser) not in lidar.excluded_ports()
    assert str(excluded) in lidar.excluded_ports()
    assert str(laser) not in bms_setup.master_ports()
    assert str(laser) not in drivers.reserved_ports()
    assert str(excluded) in drivers.reserved_ports()

    # Positive lidar classification is still essential; removing the stale
    # serial hint is not permission to probe a wheel or an unknown device.
    st = laser.stat()
    lidar.publish_usb_scan([(str(laser), (st.st_dev, st.st_ino, st.st_rdev, st.st_ctime_ns), False)])
    assert drivers.candidate_ports([str(wheel), str(laser), str(excluded)]) == [str(laser)]
    assert str(wheel) not in lidar.checked_lidar_ports()
    lock = lidar.try_lock_port(str(wheel))
    try:
        assert lock not in (None, -1)
        assert lidar.try_lock_port(str(wheel)) is None
    finally:
        lidar.unlock_port(lock)

    # In the separate serial mode, the same configured names still protect
    # serial chassis ports from the lidar and BMS.
    monkeypatch.delenv("RC_USB_DISCOVERY_MODE")
    assert str(wheel) in lidar.excluded_ports()
    assert str(wheel) in bms_setup.master_ports()
    assert str(wheel) in drivers.reserved_ports()


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
    snap = source.read()
    motors = {m["id"]: m for m in snap["motors"]}
    for alias, role in zip(aliases, order):
        assert motors[role]["port"] == os.path.realpath(wheel[role])
        assert motors[role]["portPath"] == str(alias)
        assert snap["masterPort"]["modules"][role] == str(alias)
    # Never present an old path as currently attached after telemetry expires.
    stale = source.read(now=time.time() + backend.MasterSource.STALE_S + 1)
    assert all(m["port"] is None and m["portPath"] is None for m in stale["motors"])
    assert stale["masterPort"]["modules"] == {}
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
        # No snapshot may attach the new occupant to the old role, even in the
        # interval before the reader notices the changed symlink.
        for motor in source.read()["motors"]:
            if motor["port"] is not None:
                assert motor["port"] == os.path.realpath(wheel[motor["id"]])
                assert os.path.realpath(motor["portPath"]) == motor["port"]
    assert wait_for(lambda: source.modules == {role: str(alias) for alias, role in zip(aliases, new_order)}
                    and source.serial_ready.is_set(), 10), (source.modules, source.identity_fault, source.last_error)
    assert not source.identity_fault
    snap = source.read()
    for alias, role in zip(aliases, new_order):
        motor = next(m for m in snap["motors"] if m["id"] == role)
        assert (motor["port"], motor["portPath"]) == (os.path.realpath(wheel[role]), str(alias))
        assert snap["masterPort"]["modules"][role] == str(alias)


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
    snap = source.read()
    assert snap["masterPort"]["modules"] == {}
    assert all(m["port"] is None for m in snap["motors"])
    assert "Две платы" in snap["masterPort"]["error"]
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
        assert old.path in source.read()["masterPort"]["legacy"]
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
    battery_manager = battery.BatteryManager(bms.path)
    monkeypatch.setenv("RC_BMS_PORT", bms.path)
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
        battery_manager.use_manual_discovery(source)
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
        assert b"\xdd\xa5" not in laser.received  # no JBD auto-search of the lidar
        assert lidar.try_lock_port(laser.path) is None
        assert len(source.modules) == 4 and not source.identity_fault
    finally:
        if manager is not None:
            manager.stop()
        battery_manager.stop()
        bms.close()
        laser.close()


def test_only_selected_bms_is_polled_and_unknown_device_blocks_motion(boards, usb_context, monkeypatch):
    _, start = usb_context
    wheel = boards()
    one, two = bms_device(), bms_device()
    monkeypatch.setenv("RC_BMS_PORT", one.path)
    manager = battery.BatteryManager(one.path)
    paths = list(wheel.values()) + [one.path, two.path]
    try:
        source = start(paths)
        manager.use_manual_discovery(source)
        manager.start()
        assert wait_for(lambda: manager.snapshot()["connected"], 8), manager.snapshot()
        assert b"\xdd\xa5\x03" in one.received
        assert not two.received  # manual selection never probes another BMS
        assert not source.serial_ready.is_set()  # the unknown USB is not safe to assume
        assert source.drive_servo(10)["reason"] == "serial_unavailable"
        paths.remove(two.path)
        assert wait_for(source.serial_ready.is_set, 5)
        assert not two.received
    finally:
        manager.stop()
        one.close()
        two.close()


def test_partial_jbd_response_is_not_a_lidar_or_success(usb_context, monkeypatch):
    _, start = usb_context
    partial = bms_device(partial=True)
    monkeypatch.setenv("RC_BMS_PORT", partial.path)
    manager = battery.BatteryManager(partial.path)
    try:
        source = start([partial.path])
        manager.use_manual_discovery(source)
        manager.start()
        assert wait_for(lambda: b"\xdd\xa5\x04" in partial.received, 8)
        time.sleep(1)
        assert partial.path not in source.peripheral_identities()
        assert partial.path not in lidar.checked_lidar_ports()
        assert not manager.snapshot()["connected"] and not source.serial_ready.is_set()
    finally:
        manager.stop()
        partial.close()


def test_silent_unselected_usb_is_not_probed_or_treated_as_lidar(boards, usb_context):
    _, start = usb_context
    wheel = boards()
    silent = Device().start(lambda: None)
    try:
        source = start(list(wheel.values()) + [silent.path])
        assert wait_for(lambda: silent.path in source.passively_checked_unknown_ports(), 6)
        assert not source.serial_ready.is_set()
        assert silent.path in source.read()["masterPort"]["unclassified"]
        assert silent.path not in lidar.checked_lidar_ports()
        assert not silent.read() and not silent.received
    finally:
        silent.close()


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
def test_wheel_port_cards_in_browser_with_live_owner_and_peer(
        boards, usb_context, kiosk_client, app, operator, monkeypatch):
    """Four PTYs → onboard owner → API/browser, peer, unplug and recovery."""
    from flask import jsonify
    from werkzeug.serving import WSGIRequestHandler, make_server
    from robot_control.camera import CameraManager
    backend, start = usb_context
    wheel = boards("browser-ports")
    real_ports = {role: os.path.realpath(path) for role, path in wheel.items()}
    rl_target = os.readlink(wheel["RL"])
    source = start([wheel[m] for m in MODULES])
    assert wait_for(lambda: len(source.modules) == 4 and source.serial_ready.is_set(), 8)
    kiosk = kiosk_client.application
    kiosk.extensions["rc_slam"].source = source

    # Only registered in this test app. No real hardware or application route
    # is involved in the simulated plug/unplug.
    @kiosk.post("/__test/unplug")
    def unplug():
        os.unlink(wheel["RL"])
        return jsonify(ok=True)

    @kiosk.post("/__test/replug")
    def replug():
        os.symlink(rl_target, wheel["RL"])
        return jsonify(ok=True)

    class Quiet(WSGIRequestHandler):
        def log_request(self, *args, **kwargs):
            pass

    onboard = make_server("0.0.0.0", 0, kiosk, threaded=True, request_handler=Quiet)
    peer_url = f"http://127.0.0.1:{onboard.server_port}/api/kiosk_state"
    monkeypatch.setenv("RC_SLAM_PEER_URL", peer_url)
    app.extensions["rc_slam"].source = backend.MasterPeerSource(peer_url)
    remote = make_server("0.0.0.0", 0, app, threaded=True, request_handler=Quiet)
    threads = [threading.Thread(target=server.serve_forever, daemon=True) for server in (onboard, remote)]
    for thread in threads:
        thread.start()
    previous_camera_manager = CameraManager._instance
    CameraManager._instance = CameraManager()
    try:
        result = subprocess.run(["node", str(ROOT / "tests/js/wheel_ports_browser.mjs"),
                                 f"http://127.0.0.1:{onboard.server_port}",
                                 f"http://127.0.0.1:{remote.server_port}", json.dumps(real_ports)],
                                capture_output=True, text=True, timeout=120)
        assert result.returncode == 0, result.stdout + result.stderr
        assert "unplug, replug and stale server passed" in result.stdout
    finally:
        remote.shutdown()
        onboard.shutdown()
        for thread in threads:
            thread.join(timeout=3)
        CameraManager.reset_instance()
        CameraManager._instance = previous_camera_manager


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


def test_eeprom_recovery_and_optical_calibration_on_four_fw23_pty_boards(
        boards, usb_context, kiosk_client, monkeypatch, tmp_path):
    """Corrupt boot -> explicit reset -> tagged reply -> optical calibration -> save."""
    backend, start = usb_context
    journal = tmp_path / "recover-full-zero.pending"
    monkeypatch.setenv("RC_FULL_ZERO_PENDING", str(journal))
    writes = []
    original_write = backend.MasterSource._write_link_command
    def record_write(dev, command):
        writes.append((dev.port, bytes(command)))
        return original_write(dev, command)
    monkeypatch.setattr(backend.MasterSource, "_write_link_command", staticmethod(record_write))
    wheel = boards("bad-zero", bad_zero="FR,RL")
    source = start([wheel[m] for m in MODULES])
    assert wait_for(lambda: len(source.modules) == 4 and source.serial_ready.is_set(), 8)
    assert wait_for(lambda: {m["id"] for m in source.read()["motors"] if m["fullZeroFault"]} ==
                    {"FR", "RL"}, 5)
    kiosk_client.application.extensions["rc_slam"].source = source
    marker = {"X-Robot-Control": "steering"}

    def send(action, token=None, **kwargs):
        body = {"action": action, **kwargs}
        if token is not None:
            body["token"] = token
        response = kiosk_client.post("/api/kiosk_full_zero", json=body, headers=marker)
        assert response.is_json, response.data
        return response.status_code, response.json

    code, response = send("begin")
    assert code == 200, response
    token = response["token"]

    def wait_phase(target, timeout=12):
        last = {}
        def check():
            nonlocal last
            code, last = send("status", token)
            assert code == 200 and last["phase"] not in {"stopping", "idle", "recovery"}, last
            return last["phase"] == target
        assert wait_for(check, timeout), last
        return last

    result = wait_phase("fault")
    assert result["faultyModules"] == ["FR", "RL"]
    assert send("calibrate", token)[0] == 409
    assert send("reset_fault", token, confirm=False)[0] == 409
    assert wait_for(journal.exists, 5)  # fault is durable even before reset
    assert not any(b"zr CONFIRM" in command for _, command in writes)
    code, result = send("reset_fault", token, confirm=True)
    assert code == 200 and result["phase"] == "resetting" and journal.exists(), result
    result = wait_phase("ready")
    assert result["pendingRecovery"] and not result["faultyModules"]
    assert source._full_zero_inhibit
    code, result = send("calibrate", token)
    assert code == 200 and result["phase"] == "calibrating", result
    wait_phase("adjust", timeout=40)
    code, result = send("save", token)
    assert code == 200, result
    result = wait_phase("saved")
    assert result["acknowledged"] == list(MODULES) and not journal.exists()
    assert not source._full_zero_inhibit


def test_partial_eeprom_reset_failure_keeps_journal_and_all_motion_blocked(
        boards, usb_context, kiosk_client, monkeypatch, tmp_path):
    _, start = usb_context
    journal = tmp_path / "failed-reset.pending"
    monkeypatch.setenv("RC_FULL_ZERO_PENDING", str(journal))
    wheel = boards("unwritable", bad_zero="FR,RL", fail_eeprom="RL")
    source = start([wheel[m] for m in MODULES])
    assert wait_for(lambda: len(source.modules) == 4 and source.serial_ready.is_set(), 8)
    assert wait_for(lambda: {m["id"] for m in source.read()["motors"] if m["fullZeroFault"]} ==
                    {"FR", "RL"}, 5)
    kiosk_client.application.extensions["rc_slam"].source = source
    header = {"X-Robot-Control": "steering"}

    def post(body):
        response = kiosk_client.post("/api/kiosk_full_zero", json=body, headers=header)
        assert response.is_json
        return response.status_code, response.json

    code, result = post({"action": "begin"})
    assert code == 200, result
    token = result["token"]
    phase = {}
    def status_is(target):
        nonlocal phase
        code, phase = post({"action": "status", "token": token})
        return code == 200 and phase["phase"] == target

    assert wait_for(lambda: status_is("fault"), 8), phase
    code, result = post({"action": "reset_fault", "token": token, "confirm": True})
    assert code == 200 and journal.exists(), result
    # RL cannot write: FR may have been reset, but no traction or calibration
    # is permitted until the whole recovery is completed deliberately.
    assert wait_for(lambda: phase.get("reason") == "eeprom_reset_unconfirmed" or
                    (status_is("stopping") and phase.get("reason") == "eeprom_reset_unconfirmed") or
                    (status_is("recovery") and phase.get("reason") == "eeprom_reset_unconfirmed"), 10), phase
    assert journal.exists() and source._full_zero_inhibit
    assert wait_for(lambda: {m["id"] for m in source.read()["motors"] if m["fullZeroFault"]} == {"RL"}, 5)
    assert source.drive_servo(20)["ok"] is False
    assert not post({"action": "calibrate", "token": token})[1].get("ok")
    # A fresh owner sees the durable marker even before any serial link opens.
    backend, _ = usb_context
    restarted = backend.MasterSource(auto_discovery=True, port_provider=lambda: [],
                                     claim=lidar.try_lock_port, release=lidar.unlock_port)
    try:
        assert restarted._full_zero_pending and restarted._full_zero_inhibit
    finally:
        restarted._stop = True
        restarted._dispatch_wakeup.set()


def test_usb_loss_during_eeprom_reset_retains_recovery_interlock(
        boards, usb_context, kiosk_client, monkeypatch, tmp_path):
    backend, start = usb_context
    journal = tmp_path / "lost-reset.pending"
    monkeypatch.setenv("RC_FULL_ZERO_PENDING", str(journal))
    wheel = boards("lost-board", bad_zero="FR")
    original_write = backend.MasterSource._write_link_command
    lost = threading.Event()

    def lose_after_reset(dev, command):
        original_write(dev, command)
        if b"zr CONFIRM" in bytes(command) and dev.port == wheel["FR"]:
            # Drop the selected board after reset could have reached it but
            # before any ACK can establish the resulting EEPROM state.
            os.unlink(wheel["FR"])
            try:
                dev.cancel_read()
                dev.close()
            finally:
                lost.set()

    monkeypatch.setattr(backend.MasterSource, "_write_link_command", staticmethod(lose_after_reset))
    source = start([wheel[m] for m in MODULES])
    assert wait_for(lambda: len(source.modules) == 4 and source.serial_ready.is_set(), 8)
    assert wait_for(lambda: any(m["id"] == "FR" and m["fullZeroFault"]
                                for m in source.read()["motors"]), 5)
    kiosk_client.application.extensions["rc_slam"].source = source
    header = {"X-Robot-Control": "steering"}

    def post(action, token=None, **extra):
        data = {"action": action, **extra}
        if token:
            data["token"] = token
        response = kiosk_client.post("/api/kiosk_full_zero", json=data, headers=header)
        return response.status_code, response.json

    code, result = post("begin")
    assert code == 200, result
    token = result["token"]
    last = {}
    def fault_ready():
        nonlocal last
        _, last = post("status", token)
        return last["phase"] == "fault"
    assert wait_for(fault_ready, 8), last
    assert post("reset_fault", token, confirm=True)[0] == 200
    assert lost.wait(5) and journal.exists()
    def stopped():
        nonlocal last
        _, last = post("status", token)
        return last["phase"] in {"stopping", "recovery"}
    assert wait_for(stopped, 8), last
    assert last["reason"] in {"module_offline", "serial_unavailable", "eeprom_reset_unconfirmed"}
    assert source._full_zero_inhibit and journal.exists()
    assert source.drive_servo(20)["ok"] is False


@pytest.mark.skipif(not os.environ.get("ZAPRET_BROWSER_NODE_MODULES") or not shutil.which("node"),
                    reason="optional Chromium/playwright-core not installed")
def test_manual_bms_browser_with_wheels_and_lidar(boards, usb_context, kiosk_client, monkeypatch, tmp_path):
    from werkzeug.serving import WSGIRequestHandler, make_server
    from robot_control import bms_setup
    _, start = usb_context
    monkeypatch.delenv("RC_BMS_PORT", raising=False)
    monkeypatch.setattr(bms_setup, "env_file_path", lambda: tmp_path / ".env")
    wheel = boards("manual-battery-browser")
    bms, laser = bms_device(), lidar_device()
    manager = kiosk_client.application.extensions["rc_battery"]
    source = start(list(wheel.values()) + [bms.path, laser.path])
    kiosk_client.application.extensions["rc_slam"].source = source
    manager.use_manual_discovery(source)
    assert wait_for(lambda: source.manual_bms_port_allowed(bms.path), 8)
    assert wait_for(lambda: os.path.realpath(laser.path) in
                    {os.path.realpath(p) for p in lidar.checked_lidar_ports()}, 8)
    assert not bms.received and not source.serial_ready.is_set()

    class Quiet(WSGIRequestHandler):
        def log_request(self, *args, **kwargs):
            pass

    server = make_server("0.0.0.0", 0, kiosk_client.application, threaded=True, request_handler=Quiet)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        result = subprocess.run(["node", str(ROOT / "tests/js/battery_manual_browser.mjs"),
                                 f"http://127.0.0.1:{server.server_port}", bms.path],
                                capture_output=True, text=True, timeout=90)
        assert result.returncode == 0, result.stdout + result.stderr
        assert f"RC_BMS_PORT={bms.path}" in (tmp_path / ".env").read_text()
        assert manager.snapshot()["connected"] and wait_for(source.serial_ready.is_set, 5)
        assert b"\xdd\xa5" not in laser.received
    finally:
        manager.stop()
        server.shutdown()
        thread.join(timeout=3)
        bms.close()
        laser.close()


def test_manual_bms_api_rejects_wheel_lidar_and_busy_port(boards, usb_context, kiosk_client, monkeypatch, tmp_path):
    from robot_control import bms_setup
    _, start = usb_context
    monkeypatch.delenv("RC_BMS_PORT", raising=False)
    monkeypatch.setattr(bms_setup, "env_file_path", lambda: tmp_path / ".env")
    wheel = boards("manual-conflicts")
    bms, laser = bms_device(), lidar_device()
    source = start(list(wheel.values()) + [bms.path, laser.path])
    manager = kiosk_client.application.extensions["rc_battery"]
    manager.use_manual_discovery(source)
    kiosk_client.application.extensions["rc_slam"].source = source
    try:
        assert wait_for(lambda: len(source.modules) == 4 and source.manual_bms_port_allowed(bms.path), 8)
        assert wait_for(lambda: os.path.realpath(laser.path) in
                        {os.path.realpath(p) for p in lidar.checked_lidar_ports()}, 8)
        for port in (wheel["FL"], laser.path):
            assert kiosk_client.post("/api/battery/probe", json={"port": port}).status_code == 409
            assert kiosk_client.post("/api/battery/port", json={"port": port}).status_code == 400
        fd = lidar.try_lock_port(bms.path)
        assert fd not in (None, -1)
        try:
            response = kiosk_client.post("/api/battery/port", json={"port": bms.path})
            assert response.status_code == 400 and "занят" in response.json["error"]
        finally:
            lidar.unlock_port(fd)
        assert not bms.received and b"\xdd\xa5" not in laser.received
        assert not (tmp_path / ".env").exists()
    finally:
        manager.stop()
        bms.close()
        laser.close()


def test_manual_bms_reconnects_only_selected_alias_after_revalidation(boards, usb_context, monkeypatch, tmp_path):
    _, start = usb_context
    wheel = boards("manual-reconnect")
    one, two = bms_device(), bms_device()
    alias = tmp_path / "serial-bms"
    alias.symlink_to(one.path)
    monkeypatch.setenv("RC_BMS_PORT", str(alias))
    manager = battery.BatteryManager(str(alias))
    source = start(list(wheel.values()) + [str(alias)])
    manager.use_manual_discovery(source)
    manager.start()
    try:
        assert wait_for(lambda: manager.snapshot()["connected"] and source.serial_ready.is_set(), 8)
        assert one.received and not two.received
        replacement = tmp_path / "replacement"
        replacement.symlink_to(two.path)
        os.replace(replacement, alias)
        assert wait_for(lambda: b"\xdd\xa5\x03" in two.received, 12)
        assert wait_for(lambda: manager.snapshot()["connected"] and source.serial_ready.is_set(), 8)
        assert source.manual_bms_port_allowed(str(alias))
        assert manager.snapshot()["port"] == str(alias)
    finally:
        manager.stop()
        one.close()
        two.close()


def test_old_negative_jbd_scan_is_not_positive_lidar_permission(usb_context, tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / "ros2_ws/src/zapret_lidar"))
    from zapret_lidar import usb_guard
    path = tmp_path / "silent-device"
    path.touch()
    st = path.stat()
    directory = lidar.ports_state_dir()
    directory.mkdir(parents=True, exist_ok=True)
    legacy = directory / f"usb-scan-{os.getpid()}.json"
    legacy.write_text(json.dumps({"pid": os.getpid(), "at": time.time(), "ports": [{
        "port": str(path), "device": str(path),
        "fingerprint": [st.st_dev, st.st_ino, st.st_rdev, st.st_ctime_ns], "bms": False,
    }]}))
    assert str(path) in lidar.checked_usb_ports()
    assert str(path) not in lidar.checked_lidar_ports()
    assert not usb_guard.permitted(str(path))
    lidar.publish_usb_scan([(str(path), (st.st_dev, st.st_ino, st.st_rdev, st.st_ctime_ns), False)])
    assert usb_guard.permitted(str(path))
    monkeypatch.setenv("RC_BMS_PORT", str(path))
    assert not usb_guard.permitted(str(path))
    monkeypatch.delenv("RC_BMS_PORT")
    lidar.publish_bms_port(str(path))
    assert not usb_guard.permitted(str(path))  # live reservation even if ROS environment is stale
    lidar.publish_bms_port(None)


def test_cancelling_fault_wizard_does_not_unblock_motion(boards, usb_context, kiosk_client, monkeypatch, tmp_path):
    _, start = usb_context
    monkeypatch.setenv("RC_FULL_ZERO_PENDING", str(tmp_path / "cancel-before-reset.pending"))
    wheel = boards("cancel-fault", bad_zero="RR")
    source = start([wheel[m] for m in MODULES])
    assert wait_for(lambda: len(source.modules) == 4 and source.serial_ready.is_set(), 8)
    assert wait_for(lambda: source._full_zero_inhibit and any(
        m["id"] == "RR" and m["fullZeroFault"] for m in source.read()["motors"]), 5)
    kiosk_client.application.extensions["rc_slam"].source = source
    header = {"X-Robot-Control": "steering"}
    def send(action, token=None):
        payload = {"action": action}
        if token:
            payload["token"] = token
        response = kiosk_client.post("/api/kiosk_full_zero", json=payload, headers=header)
        return response.status_code, response.json
    code, state = send("begin")
    assert code == 200, state
    token = state["token"]
    assert wait_for(lambda: send("status", token)[1]["phase"] == "fault", 8)
    assert send("cancel", token)[0] == 200
    assert wait_for(lambda: not source._full_zero_session, 8)
    assert source._full_zero_inhibit and source.drive_servo(20, "FL")["ok"] is False
    # A missing faulty board cannot make the other wheels runnable in this process.
    os.unlink(wheel["RR"])
    assert wait_for(lambda: "RR" not in source.modules, 5)
    assert source._full_zero_inhibit and source.drive_servo(20, "FL")["ok"] is False
    assert source._full_zero_fault_latched
    # Finalising another cancelled session with the bad frame gone cannot
    # clear the observed fault before a successful four-board save.
    source.finish_full_zero()
    assert source._full_zero_inhibit
    journal = tmp_path / "cancel-before-reset.pending"
    assert wait_for(journal.exists, 5)
    # A new owner remains blocked even if the faulty board was disconnected.
    backend, _ = usb_context
    restarted = backend.MasterSource(auto_discovery=True, port_provider=lambda: [])
    try:
        assert restarted._full_zero_pending and restarted._full_zero_inhibit
        assert not restarted.drive_servo(20, "FL")["ok"]
    finally:
        restarted._stop = True
        restarted._dispatch_wakeup.set()


def test_failed_fault_journal_keeps_motion_blocked_and_prevents_reset(
        boards, usb_context, kiosk_client, monkeypatch, tmp_path):
    from robot_control import full_zero
    backend, start = usb_context
    path = tmp_path / "unavailable.pending"
    monkeypatch.setenv("RC_FULL_ZERO_PENDING", str(path))
    attempts = []

    def fail_write(journal):
        attempts.append(journal)
        raise OSError("injected disk failure")

    monkeypatch.setattr(full_zero, "write_pending", fail_write)
    writes = []
    original = backend.MasterSource._write_link_command
    def capture(dev, command):
        writes.append(bytes(command))
        return original(dev, command)
    monkeypatch.setattr(backend.MasterSource, "_write_link_command", staticmethod(capture))
    wheel = boards("fault-disk-failure", bad_zero="FR")
    source = start([wheel[m] for m in MODULES])
    assert wait_for(lambda: len(source.modules) == 4 and source.serial_ready.is_set(), 8)
    assert wait_for(lambda: bool(attempts) and source._full_zero_fault_latched, 5)
    assert not path.exists() and source._full_zero_inhibit
    assert not source.drive_servo(20, "FL")["ok"]
    kiosk_client.application.extensions["rc_slam"].source = source
    header = {"X-Robot-Control": "steering"}
    def post(body):
        response = kiosk_client.post("/api/kiosk_full_zero", json=body, headers=header)
        return response.status_code, response.json
    code, result = post({"action": "begin"})
    assert code == 200, result
    token = result["token"]
    assert wait_for(lambda: post({"action": "status", "token": token})[1]["phase"] == "fault", 8)
    code, result = post({"action": "reset_fault", "token": token, "confirm": True})
    assert code == 409 and result["reason"] == "journal_unavailable"
    assert source._full_zero_inhibit and not any(b"zr CONFIRM" in command for command in writes)


def test_slow_fault_journal_never_delays_stop_on_the_faulty_wheel(
        boards, usb_context, monkeypatch, tmp_path):
    from robot_control import full_zero
    backend, start = usb_context
    path = tmp_path / "slow.pending"
    monkeypatch.setenv("RC_FULL_ZERO_PENDING", str(path))
    entered, release = threading.Event(), threading.Event()
    original_journal = full_zero.write_pending
    def slow_journal(journal):
        entered.set()
        if not release.wait(10):
            raise OSError("injected stuck fsync")
        return original_journal(journal)
    monkeypatch.setattr(full_zero, "write_pending", slow_journal)
    writes = []
    original_write = backend.MasterSource._write_link_command
    def capture(dev, command):
        writes.append((dev.port, bytes(command)))
        return original_write(dev, command)
    monkeypatch.setattr(backend.MasterSource, "_write_link_command", staticmethod(capture))
    wheel = boards("slow-fault-journal", bad_zero="RR")
    paths = [wheel["RR"]]
    source = start(paths)
    try:
        assert entered.wait(8)
        assert source._full_zero_inhibit and not path.exists()
        # The board reports its bad EEPROM in the first identity frame. Its
        # writer must still start and deliver STOP while fsync is blocked.
        assert wait_for(lambda: any(port == wheel["RR"] and b"s q\nb" in command
                                    for port, command in writes), 5), writes
        # Boards discovered only after the original fault also get a stop.
        paths.extend(wheel[m] for m in ("FL", "FR", "RL"))
        assert wait_for(lambda: all(any(port == wheel[m] and b"s q\nb" in command
                                        for port, command in writes)
                                    for m in MODULES), 8), writes
    finally:
        release.set()
    assert wait_for(path.exists, 5)


def test_wrong_cached_hint_only_reorders_all_live_usb_candidates(boards, usb_context, tmp_path, monkeypatch):
    """A hint to RR as FL cannot assign it, exclude another port or allow motion."""
    import re
    from robot_control import device_registry
    _, start = usb_context
    wheel = boards('registry-wheels')
    unknown = Device()
    paths = [wheel[m] for m in MODULES] + [unknown.path]
    # Production accepts /dev paths only; PTY aliases in this test live in tmp.
    monkeypatch.setattr(device_registry, 'SERIAL_RE', re.compile(r'^' + re.escape(str(tmp_path)) + r'/[A-Za-z0-9_-]+$'))
    reg = device_registry.DeviceRegistry(tmp_path / 'devices.json')
    reg.set_hint('wheel_FL', wheel['RR'])
    ordered = reg.order_serial(paths)
    assert ordered[0] == wheel['RR'] and set(ordered) == set(paths) and len(ordered) == len(paths)
    source = start(paths)
    source.port_provider = lambda: reg.order_serial(paths)
    try:
        assert wait_for(lambda: len(source.modules) == 4, 8)
        snapshot = source.read()
        motors = {motor['id']: motor for motor in snapshot['motors']}
        assert motors['FL']['port'] == os.path.realpath(wheel['FL'])
        assert motors['RR']['port'] == os.path.realpath(wheel['RR'])
        assert wait_for(lambda: unknown.path in source.read()['masterPort']['unclassified'], 6)
        assert not source.serial_ready.is_set()
        assert not source.drive_servo(20, 'FL')['ok']
        assert not unknown.read(), 'no motor/lidar/BMS command may be sent to unknown USB'
    finally:
        unknown.close()


def silent_lds_device(*, invalid_crc=False):
    """LDS-like UART emits nothing at all until it receives exactly `b`."""
    dev = Device()

    def packet(index):
        pkt = bytearray(42)
        pkt[0], pkt[1] = 0xFA, 0xA0 + index % 60
        pkt[2:4] = (300).to_bytes(2, "little")
        for i in range(6):
            off = 4 + i * 6
            pkt[off:off + 2] = (200).to_bytes(2, "little")
            pkt[off + 2:off + 4] = (1500 + i).to_bytes(2, "little")
        checksum = lidar.lds01_checksum(bytes(pkt[:40]))
        pkt[40:42] = (checksum ^ (1 if invalid_crc else 0)).to_bytes(2, "little")
        return bytes(pkt)

    def run():
        index, running = 0, False
        while not dev.stop_flag.is_set():
            incoming = dev.read()
            if b"b" in incoming:
                running = True
            if running:
                dev.write(packet(index))
                index += 1
            time.sleep(.003)
    return dev.start(run)


def lds_test_service(kiosk_client, source, paths, monkeypatch):
    """Inject PTY-only topology/whitelist in tests, never in production."""
    import re
    from robot_control import lidar_start

    class Topology:
        def usb(self, *, refresh=False):
            return {"available": True, "devices": [
                {"topology": "1-%d" % index, "vidPid": "1234:5678", "serial": "",
                 "nodes": [{"path": path, "kind": "serial", "aliases": [], "interface": "1.0"}]}
                for index, path in enumerate(paths, 1)]}

    monkeypatch.setattr(lidar_start, "SERIAL_RE", re.compile(r"/.+"))
    service = kiosk_client.application.extensions["rc_lidar_start"]
    service.registry = Topology()
    kiosk_client.application.extensions["rc_slam"].source = source
    monkeypatch.setattr(lidar.LidarManager.get_instance(), "restart_search", lambda: {"ros_rescan": "requested"})
    return service


def test_silent_lds_requires_physical_confirmation_then_crc_unlocks_ros(boards, usb_context, kiosk_client, monkeypatch):
    """The real HTTP route, 4 PTYs and ROS guard; no physical device is tested."""
    _, start = usb_context
    monkeypatch.syspath_prepend(str(ROOT / "ros2_ws/src/zapret_lidar"))
    from zapret_lidar import usb_guard
    laser = silent_lds_device()
    try:
        wheel = boards("silent-lds")
        paths = list(wheel.values()) + [laser.path]
        source = start(paths)
        service = lds_test_service(kiosk_client, source, paths, monkeypatch)
        assert wait_for(lambda: len(source.modules) == 4 and source.manual_printer_port_allowed(laser.path), 8)
        assert not source.serial_ready.is_set() and not usb_guard.permitted(laser.path)
        assert not laser.received  # auto-discovery and a hint never transmit
        headers = {"X-Robot-Control": "devices"}
        assert kiosk_client.post("/api/devices/lidar/prepare", json={"port": laser.path}).status_code == 400
        wheel_attempt = kiosk_client.post("/api/devices/lidar/prepare", json={"port": wheel["FL"]}, headers=headers)
        assert wheel_attempt.status_code == 409
        prepared = kiosk_client.post("/api/devices/lidar/prepare", json={"port": laser.path}, headers=headers)
        assert prepared.status_code == 200, prepared.json
        assert not laser.received and not usb_guard.permitted(laser.path)
        token = prepared.json["token"]
        rejected = kiosk_client.post("/api/devices/lidar/start", json={"token": token, "confirmed": False}, headers=headers)
        assert rejected.status_code == 400 and not laser.received
        started = kiosk_client.post("/api/devices/lidar/start", json={"token": token, "confirmed": True}, headers=headers)
        assert started.status_code == 200, started.json
        assert started.json["protocol"] == "lds01" and started.json["frames"] >= 6
        assert started.json["rosRescan"] == "requested"
        assert wait_for(lambda: bytes(laser.received) == b"b")
        assert wait_for(lambda: usb_guard.permitted(laser.path) and source.serial_ready.is_set())
        replay = kiosk_client.post("/api/devices/lidar/start", json={"token": token, "confirmed": True}, headers=headers)
        assert replay.status_code == 409 and bytes(laser.received) == b"b"
        assert not source._manual_lidar_probes
    finally:
        laser.close()


def test_manual_lds_bad_crc_stays_unknown_and_cannot_enable_ros(boards, usb_context, kiosk_client, monkeypatch):
    _, start = usb_context
    monkeypatch.syspath_prepend(str(ROOT / "ros2_ws/src/zapret_lidar"))
    from zapret_lidar import usb_guard
    laser = silent_lds_device(invalid_crc=True)
    try:
        wheel = boards("bad-lds")
        paths = list(wheel.values()) + [laser.path]
        source = start(paths)
        service = lds_test_service(kiosk_client, source, paths, monkeypatch)
        service.READ_SECONDS = .6
        assert wait_for(lambda: len(source.modules) == 4 and source.manual_printer_port_allowed(laser.path), 8)
        prepared = service.prepare({"port": laser.path})
        from robot_control.lidar_start import LidarStartError
        with pytest.raises(LidarStartError, match="кадров"):
            service.start({"token": prepared["token"], "confirmed": True})
        assert bytes(laser.received) == b"b"
        assert not usb_guard.permitted(laser.path) and not source.serial_ready.is_set()
        assert not source._manual_lidar_probes
        lock = lidar.try_lock_port(laser.path)
        assert lock not in (None, -1)
        lidar.unlock_port(lock)
        # A new failed/replayed token must never send another byte.
        with pytest.raises(LidarStartError):
            service.start({"token": prepared["token"], "confirmed": True})
        assert bytes(laser.received) == b"b"
    finally:
        laser.close()


def test_manual_lds_fails_closed_for_replacement_reservations_and_lock(boards, usb_context, kiosk_client, monkeypatch, tmp_path):
    _, start = usb_context
    one, two = silent_lds_device(), silent_lds_device()
    try:
        alias = tmp_path / "serial-alias"
        alias.symlink_to(one.path)
        wheel = boards("lds-replacement")
        paths = list(wheel.values()) + [str(alias)]
        source = start(paths)
        service = lds_test_service(kiosk_client, source, paths, monkeypatch)
        assert wait_for(lambda: len(source.modules) == 4 and source.manual_printer_port_allowed(str(alias)), 8)
        from robot_control.lidar_start import LidarStartError
        # BMS, printer and explicit exclusions are never start candidates.
        for env_name in ("RC_BMS_PORT", "RC_LIDAR_EXCLUDE"):
            monkeypatch.setenv(env_name, str(alias))
            with pytest.raises(LidarStartError):
                service.prepare({"port": str(alias)})
            monkeypatch.delenv(env_name)
        from robot_control import printer
        monkeypatch.setattr(printer, "printer_reserved_ports", lambda: {str(alias)})
        with pytest.raises(LidarStartError):
            service.prepare({"port": str(alias)})
        monkeypatch.setattr(printer, "printer_reserved_ports", lambda: set())
        assert not one.received and not two.received

        prepared = service.prepare({"port": str(alias)})
        lock = lidar.try_lock_port(str(alias))
        try:
            with pytest.raises(LidarStartError, match="замок|занят"):
                service.start({"token": prepared["token"], "confirmed": True})
        finally:
            lidar.unlock_port(lock)
        assert not one.received and not two.received

        prepared = service.prepare({"port": str(alias)})
        replacement = tmp_path / "replacement-lds"
        replacement.symlink_to(two.path)
        os.replace(replacement, alias)
        with pytest.raises(LidarStartError, match="изменился"):
            service.start({"token": prepared["token"], "confirmed": True})
        assert not one.received and not two.received
    finally:
        one.close()
        two.close()


@pytest.mark.skipif(not shutil.which("node") or not os.environ.get("RC_TEST_CHROMIUM") or
                    not os.environ.get("RC_TEST_PLAYWRIGHT_CORE"),
                    reason="Chromium and Playwright required")
def test_lidar_recovery_button_through_browser_and_simulated_uart(boards, usb_context, kiosk_client, monkeypatch):
    from werkzeug.serving import make_server
    _, start = usb_context
    monkeypatch.syspath_prepend(str(ROOT / "ros2_ws/src/zapret_lidar"))
    from zapret_lidar import usb_guard
    laser = silent_lds_device()
    server = thread = None
    try:
        wheel = boards("lds-browser")
        paths = list(wheel.values()) + [laser.path]
        source = start(paths)
        service = lds_test_service(kiosk_client, source, paths, monkeypatch)
        registry = kiosk_client.application.extensions["rc_devices"]
        monkeypatch.setattr(registry, "usb", service.registry.usb)
        assert wait_for(lambda: len(source.modules) == 4 and source.manual_printer_port_allowed(laser.path), 8)
        server = make_server("127.0.0.1", 0, kiosk_client.application, threaded=True)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        result = subprocess.run([shutil.which("node"), str(ROOT / "tests/js/lidar_recovery_browser.mjs"),
                                 f"http://127.0.0.1:{server.server_port}", laser.path],
                                capture_output=True, text=True, timeout=60)
        assert result.returncode == 0, result.stdout + "\n" + result.stderr
        assert "LDS recovery UI + simulated UART: OK" in result.stdout
        assert bytes(laser.received) == b"b" and usb_guard.permitted(laser.path)
    finally:
        if server:
            server.shutdown()
            server.server_close()
        if thread:
            thread.join(timeout=3)
        laser.close()


def test_lidar_replacement_during_crc_wait_never_confirms_new_device(boards, usb_context, kiosk_client, monkeypatch, tmp_path):
    _, start = usb_context
    monkeypatch.syspath_prepend(str(ROOT / "ros2_ws/src/zapret_lidar"))
    from zapret_lidar import usb_guard
    old, replacement = silent_lds_device(invalid_crc=True), silent_lds_device()
    try:
        alias = tmp_path / "lds-live-alias"
        alias.symlink_to(old.path)
        wheel = boards("lds-swap-live")
        paths = list(wheel.values()) + [str(alias)]
        source = start(paths)
        service = lds_test_service(kiosk_client, source, paths, monkeypatch)
        assert wait_for(lambda: len(source.modules) == 4 and source.manual_printer_port_allowed(str(alias)), 8)
        prepared = service.prepare({"port": str(alias)})
        result = []
        def run():
            try:
                result.append(service.start({"token": prepared["token"], "confirmed": True}))
            except Exception as exc:  # outcome is asserted below
                result.append(exc)
        worker = threading.Thread(target=run)
        worker.start()
        assert wait_for(lambda: bytes(old.received) == b"b", 3)
        new_link = tmp_path / "new-lds-link"
        new_link.symlink_to(replacement.path)
        os.replace(new_link, alias)
        worker.join(timeout=5)
        assert not worker.is_alive() and len(result) == 1
        from robot_control.lidar_start import LidarStartError
        assert isinstance(result[0], LidarStartError) and "изменился" in str(result[0])
        assert not replacement.received and not usb_guard.permitted(str(alias))
        assert not source._manual_lidar_probes
        assert not source.serial_ready.is_set()
    finally:
        old.close()
        replacement.close()


def test_lds_expired_confirmation_and_parallel_classifiers_fail_closed(boards, usb_context, kiosk_client, monkeypatch):
    _, start = usb_context
    monkeypatch.syspath_prepend(str(ROOT / "ros2_ws/src/zapret_lidar"))
    from zapret_lidar import usb_guard
    from robot_control.lidar_start import LidarStartError
    laser = silent_lds_device(invalid_crc=True)
    worker = None
    try:
        wheel = boards("lds-concurrency")
        paths = list(wheel.values()) + [laser.path]
        source = start(paths)
        service = lds_test_service(kiosk_client, source, paths, monkeypatch)
        assert wait_for(lambda: len(source.modules) == 4 and source.manual_printer_port_allowed(laser.path), 8)
        now = [time.monotonic()]
        real_clock = service.clock
        service.clock = lambda: now[0]
        expired = service.prepare({"port": laser.path})
        now[0] += service.CONFIRM_SECONDS + 1
        with pytest.raises(LidarStartError, match="устарело"):
            service.start({"token": expired["token"], "confirmed": True})
        assert not laser.received
        service.clock = real_clock

        # Two confirmations can exist, but only one active check may write.
        # While it is waiting for CRC, BMS/printer/passive wheel discovery
        # must all be excluded from the same UART.
        first = service.prepare({"port": laser.path})
        second = service.prepare({"port": laser.path})
        service.READ_SECONDS = 1.0
        result = []
        def run():
            try:
                service.start({"token": first["token"], "confirmed": True})
            except LidarStartError as exc:
                result.append(exc)
        worker = threading.Thread(target=run)
        worker.start()
        assert wait_for(lambda: bytes(laser.received) == b"b", 3)
        real = os.path.realpath(laser.path)
        assert real in source._manual_lidar_probes
        assert not source.manual_bms_port_allowed(laser.path)
        assert not source.manual_printer_port_allowed(laser.path)
        with source.lock:
            source._unknown_checked_at[real] = 0  # force the normal passive re-check deadline
        source._open_new_links()
        with source.lock:
            assert not any(os.path.realpath(p) == real for p in source._spawning)
            assert not any(link.device == real for link in source.links.values())
            assert real not in source._bms_results
            # We forced a passive recheck due while the manual probe was
            # reserved. Restore its timestamp before the probe releases:
            # otherwise the legitimate background rescan may take the lock
            # before the release assertion below, racing the test itself.
            source._unknown_checked_at[real] = time.monotonic()
        with pytest.raises(LidarStartError, match="уже выполняется"):
            service.start({"token": second["token"], "confirmed": True})
        worker.join(timeout=4)
        assert not worker.is_alive() and len(result) == 1 and "кадров" in str(result[0])
        assert bytes(laser.received) == b"b" and not source._manual_lidar_probes
        assert not usb_guard.permitted(laser.path) and not source.serial_ready.is_set()
        lock = lidar.try_lock_port(laser.path)
        assert lock not in (None, -1)
        lidar.unlock_port(lock)
    finally:
        if worker and worker.is_alive():
            worker.join(timeout=4)
        laser.close()


def test_pot_hold_after_calibrating_four_autodiscovered_firmwares(boards, usb_context, kiosk_client):
    """UI API -> real dispatcher -> four simulated firmwares, not physical motors."""
    _, start = usb_context
    wheel = boards("pot-hold-on")
    source = start(list(wheel.values()))
    kiosk_client.application.extensions["rc_slam"].source = source
    assert wait_for(lambda: len(source.modules) == 4 and source.serial_ready.is_set(), 8), source.read()
    for module in MODULES:
        assert source.zero_module(module)["ok"]
    assert wait_for(lambda: all(m["homed"] for m in source.read()["motors"]), 5), source.read()
    headers = {"X-Robot-Control": "steering"}
    response = kiosk_client.post("/api/kiosk_pot_mode", json={"action": "calibrate"}, headers=headers)
    assert response.status_code == 200, response.json
    assert wait_for(lambda: all(m["calibrating"] for m in source.read()["motors"]), 5), source.read()
    assert wait_for(lambda: all(m["pot"] and not m["calibrating"] and not m["moving"]
                                for m in source.read()["motors"]), 20), source.read()
    response = kiosk_client.post("/api/kiosk_pot_hold", json={"action": "on"}, headers=headers)
    assert response.status_code == 200, response.json
    assert wait_for(lambda: all(m["potHold"] is True for m in source.read()["motors"]), 8), source.read()
    response = kiosk_client.post("/api/kiosk_pot_hold", json={"action": "off"}, headers=headers)
    assert response.status_code == 200, response.json
    assert wait_for(lambda: all(m["potHold"] is False for m in source.read()["motors"]), 8), source.read()


def test_pot_angle_and_hold_with_four_calibrated_firmwares(boards, usb_context, kiosk_client, tmp_path):
    """Actual firmware encoders -> four PTYs -> server; movement is simulated, not hardware."""
    control = tmp_path / "pot-input"
    control.mkdir()
    for mid in MODULES:
        (control / mid).write_text("0 1")
    _, start = usb_context
    wheel = boards("pot-feedback", pot_reverse="FR,RL", pot_control=control)
    source = start(list(wheel.values()))
    kiosk_client.application.extensions["rc_slam"].source = source

    def motors():
        return {m["id"]: m for m in source.read()["motors"]}

    def set_external(mid, degrees, connected=True):
        # Atomic replacement avoids a partially-written test control record.
        staging = control / (mid + ".tmp")
        staging.write_text(f"{degrees} {int(connected)}")
        staging.replace(control / mid)

    assert wait_for(lambda: len(source.modules) == 4 and source.serial_ready.is_set(), 8), source.read()
    for mid in MODULES:
        assert source.zero_module(mid)["ok"]
    assert wait_for(lambda: all(m["homed"] for m in motors().values()), 5)
    assert source.set_pot_mode("calibrate")["ok"]
    assert wait_for(lambda: all(m["calibrating"] for m in motors().values()), 5)
    assert wait_for(lambda: all(m["pot"] and not m["calibrating"] and not m["moving"]
                                for m in motors().values()), 20), motors()
    assert all(m["angleSource"] == "pot" and abs(m["potAngle"]) < 1 for m in motors().values()), motors()

    set_external("FL", -6)
    set_external("FR", 8)  # reversed wiring still reports the physical sign
    set_external("RL", 20)
    assert wait_for(lambda: all(abs(motors()[mid]["potAngle"] - value) < 1
                                for mid, value in (("FL", -6), ("FR", 8), ("RL", 20))), 5), motors()
    data = motors()
    assert all(abs(data[mid]["angle"]) < .1 for mid in MODULES), data  # no steps generated
    assert data["FR"]["displayAngle"] == data["FR"]["potAngle"]
    blocked = source.set_pot_hold("on")
    assert blocked["reason"] == "pot_hold_far" and blocked["modules"] == ["RL"]
    assert not any(m["potHold"] for m in motors().values())

    set_external("RL", 20, connected=False)
    assert wait_for(lambda: motors()["RL"]["pot"] is False, 5), motors()
    disconnected = motors()["RL"]
    assert disconnected["potAngle"] is None and disconnected["angleSource"] == "steps"
    assert disconnected["displayAngle"] == disconnected["angle"]
    blocked = source.set_pot_hold("on")
    assert blocked["reason"] == "pot_not_calibrated" and "RL" in blocked["modules"]
    assert not any(m["potHold"] for m in motors().values())

    set_external("FL", 0)
    set_external("FR", 0)
    set_external("RL", 0, connected=True)
    assert wait_for(lambda: all(m["potAngle"] is not None and abs(m["potAngle"]) < 1
                                for m in motors().values()), 5), motors()
    headers = {"X-Robot-Control": "steering"}
    response = kiosk_client.post("/api/kiosk_pot_hold", json={"action": "on"}, headers=headers)
    assert response.status_code == 200, response.json
    assert wait_for(lambda: all(m["potHold"] is True for m in motors().values()), 5), motors()
    set_external("FR", 6)
    assert wait_for(lambda: motors()["FR"]["moving"], 5), motors()
    assert wait_for(lambda: not motors()["FR"]["moving"] and abs(motors()["FR"]["potAngle"]) < 2.5, 7), motors()
    assert all(m["potHold"] is True for m in motors().values()), motors()

    set_external("FR", 12)
    assert wait_for(lambda: motors()["FR"]["moving"], 5), motors()
    set_external("FR", 12, connected=False)
    assert wait_for(lambda: (not motors()["FR"]["moving"] and not motors()["FR"]["potHold"]
                             and not motors()["FR"]["pot"]), 5), motors()
    assert all(motors()[mid]["potHold"] for mid in ("FL", "RL", "RR")), motors()
    stopped = motors()["FR"]["angle"]
    set_external("FR", 12, connected=True)
    assert wait_for(lambda: motors()["FR"]["potAngle"] is not None, 5), motors()
    time.sleep(.4)
    assert not motors()["FR"]["potHold"] and abs(motors()["FR"]["angle"] - stopped) < .1, motors()
    response = kiosk_client.post("/api/kiosk_pot_hold", json={"action": "off"}, headers=headers)
    assert response.status_code == 200, response.json
    assert wait_for(lambda: all(m["potHold"] is False for m in motors().values()), 5), motors()


@pytest.mark.skipif(not os.environ.get("RC_TEST_CHROMIUM") or not os.environ.get("RC_TEST_PLAYWRIGHT_CORE")
                    or not shutil.which("node"), reason="optional Chromium/playwright-core not installed")
@pytest.mark.parametrize("remote", [False, True], ids=["onboard", "remote-proxy"])
def test_pot_feedback_in_real_browser_through_four_firmwares(
        boards, usb_context, kiosk_client, app, operator, monkeypatch, tmp_path, remote):
    """Actual pages and proxy -> backend -> fw26 PTYs. No physical robot."""
    from werkzeug.serving import WSGIRequestHandler, make_server

    control = tmp_path / "browser-pot-controls"
    control.mkdir()
    for mid in MODULES:
        (control / mid).write_text("0 1")
    wheel = boards("browser-pot", pot_reverse="FR", pot_control=control)
    _, start = usb_context
    source = start(list(wheel.values()))
    kiosk_client.application.extensions["rc_slam"].source = source
    assert wait_for(lambda: len(source.modules) == 4 and source.serial_ready.is_set(), 8)
    for mid in MODULES:
        assert source.zero_module(mid)["ok"]
    assert wait_for(lambda: all(m["homed"] for m in source.read()["motors"]), 5)
    assert source.set_pot_mode("calibrate")["ok"]
    assert wait_for(lambda: all(m["calibrating"] for m in source.read()["motors"]), 5)
    assert wait_for(lambda: all(m["pot"] and m["potAngle"] is not None and not m["moving"] and
                                not m["calibrating"] for m in source.read()["motors"]), 20), source.read()

    class Quiet(WSGIRequestHandler):
        def log_request(self, *args, **kwargs):
            pass

    onboard = make_server("127.0.0.1", 0, kiosk_client.application, threaded=True, request_handler=Quiet)
    board_origin = f"http://127.0.0.1:{onboard.server_port}"
    servers = [onboard]
    if remote:
        monkeypatch.setenv("RC_SLAM_PEER_URL", board_origin + "/api/kiosk_state")
        web = make_server("127.0.0.1", 0, app, threaded=True, request_handler=Quiet)
        servers.append(web)
        origin = f"http://127.0.0.1:{web.server_port}"
    else:
        origin = board_origin
    threads = [threading.Thread(target=server.serve_forever, daemon=True) for server in servers]
    for thread in threads:
        thread.start()
    try:
        result = subprocess.run([shutil.which("node"), str(ROOT / "tests/js/pot_feedback_browser.mjs"),
                                 origin, board_origin, str(control), "remote" if remote else "onboard"],
                                capture_output=True, text=True, timeout=90)
        assert result.returncode == 0, result.stdout + result.stderr
        assert "four simulated potentiometers and confirmed hold: OK" in result.stdout
    finally:
        for server in reversed(servers):
            server.shutdown()
            server.server_close()
        for thread in threads:
            thread.join(timeout=3)
