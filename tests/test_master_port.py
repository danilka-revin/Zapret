"""Порт мастера: не только /dev/ttyACM0, а тот, где реально отвечает @TLM."""

from __future__ import annotations

import os
import select
import threading
import time

import pytest

from robot_control import master_port, slam


def test_normalize_port_accepts_bare_tty_name(monkeypatch):
    monkeypatch.setattr(master_port.os.path, "exists", lambda path: path == "/dev/ttyUSB0")
    assert master_port.normalize_port(" ttyUSB0 ") == "/dev/ttyUSB0"


def test_master_ports_keeps_missing_configured_port_first(monkeypatch):
    monkeypatch.setattr(master_port, "serial_ports",
                        lambda patterns=master_port.PORT_PATTERNS: ["/dev/ttyUSB0"])
    monkeypatch.setattr(master_port, "reserved_ports", lambda: set())
    monkeypatch.setattr(master_port.os.path, "exists", lambda path: path == "/dev/ttyUSB0")
    assert master_port.master_ports("/dev/ttyACM0") == ["/dev/ttyACM0", "/dev/ttyUSB0"]


def test_master_ports_skips_lidar_and_bms(monkeypatch):
    monkeypatch.setattr(master_port, "serial_ports",
                        lambda patterns=master_port.PORT_PATTERNS:
                        ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"])
    monkeypatch.setattr(master_port, "reserved_ports",
                        lambda: {"/dev/ttyUSB1", "/dev/ttyUSB2"})
    monkeypatch.setattr(master_port.os.path, "exists", lambda path: True)
    assert master_port.master_ports("/dev/ttyUSB0") == ["/dev/ttyUSB0"]


def test_master_ports_skips_configured_port_that_belongs_to_lidar(monkeypatch):
    monkeypatch.setattr(master_port, "serial_ports",
                        lambda patterns=master_port.PORT_PATTERNS:
                        ["/dev/ttyUSB0", "/dev/ttyUSB1"])
    monkeypatch.setattr(master_port, "reserved_ports", lambda: {"/dev/ttyUSB1"})
    monkeypatch.setattr(master_port.os.path, "exists", lambda path: True)
    assert master_port.master_ports("/dev/ttyUSB1") == ["/dev/ttyUSB0"]
    assert "лидаром или BMS" in master_port.conflict_note("/dev/ttyUSB1")


def test_master_source_explains_permission_and_busy_errors():
    backend = slam.load_backend()
    hint = backend.MasterSource._port_error_hint
    assert "dialout" in hint(Exception("Permission denied: '/dev/ttyUSB0'"))
    assert "занят" in hint(Exception("Could not exclusively lock port /dev/ttyUSB0"))
    assert hint(Exception("Input/output error")) == ""


@pytest.mark.skipif(os.name != "posix", reason="нужен POSIX pty")
def test_master_source_finds_master_on_second_port(monkeypatch):
    """Первый порт молчит, на втором идёт @TLM — источник переключается сам."""
    backend = slam.load_backend()
    monkeypatch.setattr(backend.MasterSource, "PROBE_S", 0.6)
    monkeypatch.setattr(backend.MasterSource, "RETRY_S", 0.1)

    silent_master, silent_slave = os.openpty()
    master_fd, slave_fd = os.openpty()
    silent_name = os.ttyname(silent_slave)
    name = os.ttyname(slave_fd)
    stop = threading.Event()

    def emulate_master():
        while not stop.is_set():
            ready, _, _ = select.select([master_fd], [], [], 0.05)
            if ready:
                try:
                    os.read(master_fd, 4096)      # команды не нужны
                except OSError:
                    return
            try:
                os.write(master_fd, b"@TLM mod=FL deg=0.00 tgt=0.00 moving=0 cal=1 "
                                    b"cycle=0 calib=0 opto=1 rpm=0.0 drive=0 fw=2 t=1\r\n")
            except OSError:
                return
            time.sleep(0.05)

    thread = threading.Thread(target=emulate_master, daemon=True)
    thread.start()
    statuses = []
    source = backend.MasterSource([silent_name, name], probe=True,
                                  on_status=lambda level, text: statuses.append((level, text)))
    try:
        deadline = time.time() + 5.0
        while not source.serial_ready.is_set() and time.time() < deadline:
            time.sleep(0.05)
        assert source.serial_ready.is_set(), "должен найтись второй порт"
        assert source.port == name
        out = source.read()
        assert out["linkOk"] is True
        assert out["masterPort"]["active"] == name
        assert any("мастер на связи" in text for _, text in statuses)
    finally:
        source._stop = True
        stop.set()
        for fd in (silent_master, silent_slave, master_fd, slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass


def test_make_source_master_uses_other_ports_when_configured_is_missing(monkeypatch):
    """RC_SLAM_PORT=/dev/ttyACM0, а Arduino на ttyUSB0: очередь портов, не отказ."""
    backend = slam.load_backend()
    monkeypatch.setenv("RC_SLAM_PORT", "/dev/definitely-missing-master")
    monkeypatch.delenv("RC_SLAM_MASTER_PORT", raising=False)
    monkeypatch.setattr(master_port, "serial_ports",
                        lambda patterns=master_port.PORT_PATTERNS: ["/dev/ttyUSB0"])
    monkeypatch.setattr(master_port, "reserved_ports", lambda: set())
    # configured-порт не существует, а найденный кандидат — да
    real_exists = os.path.exists
    monkeypatch.setattr(master_port.os.path, "exists",
                        lambda path: path == "/dev/ttyUSB0" or real_exists(path))
    # В тесте не открываем настоящие порты машины: очередь проверяем по списку.
    monkeypatch.setattr(backend.MasterSource, "_candidate_ports", lambda self: [])
    source = slam.make_source("master")
    try:
        assert isinstance(source, backend.MasterSource)
        assert source.configured_port == "/dev/definitely-missing-master"
        assert "/dev/ttyUSB0" in source.ports
        assert source.probe is True
    finally:
        source._stop = True
