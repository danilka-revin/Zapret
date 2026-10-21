"""CLI-проверка USB-мастера: @TLM, понятные ошибки и запись .env."""

from __future__ import annotations

import importlib.util
import os
import select
import threading
import time
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "master_check.py"


def load_cli():
    spec = importlib.util.spec_from_file_location("master_check_cli", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def cli():
    return load_cli()


def test_save_env_rewrites_only_master_line(tmp_path, cli):
    env = tmp_path / ".env"
    env.write_text("RC_SECRET_KEY=abc\n# комментарий\nRC_WHEEL_FL_PORT=/dev/ttyACM0\n"
                   "RC_LIDAR_PORT=/dev/ttyUSB1\n", encoding="utf-8")
    env.chmod(0o640)
    change = cli.save_env("/dev/ttyUSB0", env, wheel="FL")
    assert change == {"key": "RC_WHEEL_FL_PORT", "old": "/dev/ttyACM0", "new": "/dev/ttyUSB0"}
    assert env.read_text(encoding="utf-8").startswith(
        "RC_SECRET_KEY=abc\n# комментарий\nRC_WHEEL_FL_PORT=/dev/ttyUSB0\n"
        "RC_LIDAR_PORT=/dev/ttyUSB1\n")
    assert env.stat().st_mode & 0o777 == 0o640


def test_save_env_rejects_bad_port(tmp_path, cli):
    env = tmp_path / ".env"
    for bad in ("ttyUSB0", "/dev/tty; rm -rf /", "/dev/a b", "/dev/x\nRC_SLAM_PORT=/dev/y"):
        with pytest.raises(ValueError):
            cli.save_env(bad, env, wheel="FL")
    assert not env.exists()


def test_classify_error_explains_busy_and_permission(cli):
    assert cli.classify_error(Exception("could not exclusively lock port /dev/ttyUSB0"))[0] == "busy"
    assert cli.classify_error(Exception("Permission denied: '/dev/ttyACM0'"))[0] == "permission"
    assert cli.classify_error(Exception("could not open port: [Errno 2] No such file"))[0] == "missing"


@pytest.mark.skipif(os.name != "posix", reason="нужен POSIX pty")
def test_probe_finds_master_telemetry(cli):
    master_fd, slave_fd = os.openpty()
    name = os.ttyname(slave_fd)
    stop = threading.Event()

    def emulate_master():
        while not stop.is_set():
            ready, _, _ = select.select([master_fd], [], [], 0.05)
            if ready:
                try:
                    os.read(master_fd, 4096)
                except OSError:
                    return
            try:
                os.write(master_fd, b"# FL: \xd0\xb3\xd0\xbe\xd1\x82\xd0\xbe\xd0\xb2\n"
                                    b"@TLM mod=FL deg=1.00 tgt=1.00 moving=0 cal=1 "
                                    b"cycle=0 calib=0 opto=1 rpm=0.0 drive=0 fw=2 t=1\r\n")
            except OSError:
                return
            time.sleep(0.05)

    thread = threading.Thread(target=emulate_master, daemon=True)
    thread.start()
    try:
        report = cli.probe_port(name, seconds=1.5)
    finally:
        stop.set()
        for fd in (master_fd, slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass
    assert report["ok"] is True
    assert report["modules"] == []
    assert report["firmware"] == 2


@pytest.mark.skipif(os.name != "posix", reason="нужен POSIX pty")
def test_probe_reports_silent_port(cli):
    master_fd, slave_fd = os.openpty()
    name = os.ttyname(slave_fd)
    try:
        report = cli.probe_port(name, seconds=0.3)
    finally:
        for fd in (master_fd, slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass
    assert report["ok"] is False
    assert "@TLM" in report["detail"]
    assert report["modules"] == []


def test_cli_without_explicit_port_cannot_scan(cli, monkeypatch, tmp_path):
    opened = []
    monkeypatch.setattr(cli, 'probe_port', lambda port, **kw: opened.append(port))
    with pytest.raises(SystemExit) as exc:
        cli.main(['--env-file', str(tmp_path / '.env')])
    assert exc.value.code == 2
    assert opened == []


def test_cli_checks_exactly_one_selected_port(cli, monkeypatch, tmp_path):
    opened = []
    def probe(port, **kwargs):
        opened.append(port)
        return {'ok': True, 'port': port, 'detail': ''}
    monkeypatch.setattr(cli, 'probe_port', probe)
    assert cli.main(['--port', '/dev/ttyUSB7', '--json', '--env-file', str(tmp_path / '.env')]) == 0
    assert opened == ['/dev/ttyUSB7']
