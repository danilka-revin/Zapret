"""Порты колёс и лидара не отбирают друг у друга USB.

Источник колёс (``MasterSource``) сканирует все ``/dev/ttyUSB*``/``ttyACM*`` и
раньше держал открытыми **все** найденные порты, включая порт лидара, и писал
в них команды. Лидар глох: счётчик на экране мигал «200 → 15 → 50», скан
пропадал. Тесты фиксируют этикет портов, который уже был у ``SerialSource``:

* пока лидар ищет свой порт — колёса ждут (``yield_to_lidar``);
* порт под замком лидара не открывается (``claim``);
* порт, не назвавшийся колесом за окно опознания, отпускается и не трогается
  (на том же USB может быть лидар);
* в неопознанный порт не уходят даже рассылаемые команды;
* опознанный порт колеса помечается (``tag_module``), чтобы лидар его не
  перебирал: открытие порта сбрасывает плату и рвёт телеметрию.
"""

from __future__ import annotations

import os
import queue
import sys
import threading
import time
import tty
import types

import pytest

from robot_control import slam

pytestmark = pytest.mark.skipif(sys.platform.startswith("win"), reason="нужен POSIX pty")

TLM_FL = (b"@TLM mod=FL deg=0.00 tgt=0.00 moving=0 cal=1 cycle=0 calib=0 "
          b"opto=1 rpm=0.0 drive=0 fw=4 t=1\r\n")


def _backend():
    return slam.load_backend()


def _wait(condition, timeout=3.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if condition():
            return True
        time.sleep(0.02)
    return condition()


def test_port_that_is_not_a_wheel_is_released_and_left_alone():
    """Бинарный поток без @TLM (так выглядит лидар) отпускается и не трогается.

    Раньше такой порт оставался открытым навсегда: его держал источник колёс,
    а настоящий драйвер лидара не мог даже открыть устройство.
    """
    serial = pytest.importorskip("serial")
    backend = _backend()
    master_fd, slave_fd = os.openpty()
    name = os.ttyname(slave_fd)
    tty.setraw(slave_fd)                 # поток лидара — без строк
    src = backend.MasterSource(name, probe=True)
    src.PROBE_S = 0.4
    src.RESCAN_S = 0.2
    src.RETRY_S = 0.2
    try:
        deadline = time.time() + 4.0
        while time.time() < deadline and not (name in src._rejected and not src.links):
            try:
                os.write(master_fd, bytes(range(256)) * 4)
            except OSError:              # лидар «выдернули» — не мешает проверке
                pass
            time.sleep(0.04)
        assert name in src._rejected, "порт без @TLM так и остался открытым"
        assert not src.links
        # Порт свободен: драйвер лидара может забрать его монопольно.
        with serial.Serial(name, 115200, timeout=0.2, exclusive=True) as dev:
            dev.write(b"\x00")
        # И повторно колёса его не открывают.
        time.sleep(0.5)
        assert not src.links
    finally:
        src._stop = True
        os.close(master_fd)
        os.close(slave_fd)


def test_wheel_port_is_identified_tagged_and_gets_commands():
    """Порт, назвавшийся @TLM, помечается «здесь колесо» и получает команды."""
    pytest.importorskip("serial")
    backend = _backend()
    master_fd, slave_fd = os.openpty()
    name = os.ttyname(slave_fd)
    tagged, untagged, statuses = [], [], []
    src = backend.MasterSource(
        name, probe=True,
        tag_module=tagged.append,
        untag_module=untagged.append,
        on_status=lambda level, text: statuses.append((level, text)),
    )
    src.PROBE_S = 2.0
    try:
        deadline = time.time() + 4.0
        while time.time() < deadline and "FL" not in src.modules:
            try:
                os.write(master_fd, TLM_FL)
            except OSError:
                pass
            time.sleep(0.04)
        assert src.modules.get("FL") == name, "колесо не опознано: %s" % statuses
        assert _wait(lambda: name in tagged), "порт колеса не помечен для лидара"
        assert name not in src._rejected

        # Рассылаемая команда уходит только в опознанный порт.
        src.commands.put(b"b\r\n")
        os.set_blocking(master_fd, False)
        got = b""
        deadline = time.time() + 2.0
        while time.time() < deadline and b"b" not in got:
            try:
                got += os.read(master_fd, 256)
            except (BlockingIOError, OSError):
                pass
            time.sleep(0.02)
        assert b"b" in got, "команда не дошла до колеса"
    finally:
        src._stop = True
        os.close(master_fd)
        os.close(slave_fd)


def test_broadcast_command_never_goes_to_an_unidentified_port():
    """В порт, который ещё не назвался колесом, не пишется даже «b» (тормоз).

    Так команды попадали прямо в UART лидара: он замолкал, скан пропадал.
    """
    backend = _backend()
    src = object.__new__(backend.MasterSource)
    src.lock = threading.Lock()
    src.commands = queue.Queue()
    src.messages = []
    blind = types.SimpleNamespace(module=None, path="/dev/not-a-wheel",
                                  commands=queue.Queue(maxsize=8))
    wheel = types.SimpleNamespace(module="FL", path="/dev/wheel",
                                  commands=queue.Queue(maxsize=8))
    src.links = {blind.path: blind, wheel.path: wheel}

    src.commands.put(b"b\r\n")
    src._fanout_commands()

    assert blind.commands.empty(), "команда ушла в неопознанный порт (лидар!)"
    assert wheel.commands.get_nowait() == b"b\r\n"


def test_ports_wait_while_the_lidar_searches():
    """Пока лидар ищет свой порт, колёса порт не открывают (и не сбрасывают)."""
    pytest.importorskip("serial")
    backend = _backend()
    master_fd, slave_fd = os.openpty()
    name = os.ttyname(slave_fd)
    allow = {"lidar": True}
    src = backend.MasterSource(name, probe=True,
                               yield_to_lidar=lambda port: allow["lidar"])
    src.PROBE_S = 5.0
    src.RESCAN_S = 0.2
    try:
        time.sleep(0.5)
        assert not src.links, "колёса открыли порт, пока лидар его искал"
        assert name not in src._rejected
        allow["lidar"] = False
        assert _wait(lambda: name in src.links), "после поиска лидара порт не открылся"
    finally:
        src._stop = True
        os.close(master_fd)
        os.close(slave_fd)


def test_port_claimed_by_the_lidar_is_not_opened():
    """Замок порта у лидара — колёса ждут молча, порт не трогают."""
    pytest.importorskip("serial")
    backend = _backend()
    master_fd, slave_fd = os.openpty()
    name = os.ttyname(slave_fd)
    src = backend.MasterSource(name, probe=True, claim=lambda port: None)  # «занят»
    src.RESCAN_S = 0.2
    try:
        time.sleep(0.6)
        assert not src.links, "колёса открыли порт, который держит лидар"
        assert name not in src._rejected
    finally:
        src._stop = True
        os.close(master_fd)
        os.close(slave_fd)


def test_master_source_gets_the_same_port_etiquette_as_serial(monkeypatch, tmp_path):
    """make_source("master") подключает те же замки/уступки, что и SerialSource."""
    from robot_control import lidar

    monkeypatch.setenv("RC_SLAM_PORT", str(tmp_path / "no-such-port"))
    src = slam.make_source("master")
    try:
        assert src.claim is lidar.try_lock_port
        assert src.release is lidar.unlock_port
        assert src.yield_to_lidar is lidar.lidar_holds_port
        assert src.tag_module is lidar.tag_module_port
        assert src.untag_module is lidar.untag_module_port
    finally:
        src._stop = True
