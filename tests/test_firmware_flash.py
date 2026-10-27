"""Автопрошивка плат колёс: заявка после обновления и прошивка в порты из настроек.

Кнопка «Обновить с GitHub» не прошивает платы сама: USB-порты колёс держит
процесс бортового экрана. Поэтому обновление оставляет заявку
(``var/firmware-pending.json``), а экран при старте прошивает платы через
``scripts/flash_firmware.sh`` — до того, как откроет порты. Порты берутся из
настроек (``RC_WHEEL_*_PORT``), файлы прошивок так и остаются на рабочем столе.
"""

from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

import pytest

from robot_control import firmware_flash

ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"

FAKE_ARDUINO_CLI = """#!/usr/bin/env bash
printf '%s\\n' "$*" >> "$FAKE_LOG"
if [[ "${FAKE_ARDUINO_CLI_FAIL:-}" == "1" ]]; then
  echo "avrdude: stk500_recv(): programmer is not responding" >&2
  exit 1
fi
exit 0
"""


@pytest.fixture
def flashing_host(tmp_path, monkeypatch):
    """Фальшивый робот: свои .env, var/ и arduino-cli в PATH."""
    shims = tmp_path / "bin"
    shims.mkdir()
    tool = shims / "arduino-cli"
    tool.write_text(FAKE_ARDUINO_CLI, encoding="utf-8")
    tool.chmod(0o755)
    log = tmp_path / "arduino-cli.log"
    log.write_text("", encoding="utf-8")
    env_file = tmp_path / "env"
    env_file.write_text("", encoding="utf-8")
    monkeypatch.setenv("PATH", "%s%s%s" % (shims, os.pathsep, os.environ.get("PATH", "")))
    monkeypatch.setenv("FAKE_LOG", str(log))
    monkeypatch.setenv("RC_ENV_FILE", str(env_file))
    monkeypatch.setenv("RC_VAR_DIR", str(tmp_path / "var"))
    monkeypatch.delenv("RC_FIRMWARE_AUTOFLASH", raising=False)

    class Host:
        root = tmp_path
        calls = log
        env = env_file
        var = tmp_path / "var"

        def assign(self, *pairs):
            lines = list(pairs)
            self.env.write_text("\n".join(lines) + "\n", encoding="utf-8")

        def port(self, name):
            path = tmp_path / name
            path.write_text("", encoding="utf-8")
            return path

    return Host()


def run_flash(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(["bash", str(SCRIPTS / "flash_firmware.sh"), *args],
                          capture_output=True, text=True, check=False)


# --------------------------------------------------------------- заявка --- #


def test_pending_marker_roundtrip(tmp_path):
    assert firmware_flash.pending(tmp_path) is None
    path = firmware_flash.mark_pending("обновление с GitHub", "abc1234", tmp_path)
    assert path == tmp_path / "var" / "firmware-pending.json"
    request = firmware_flash.pending(tmp_path)
    assert request["reason"] == "обновление с GitHub" and request["sha"] == "abc1234"
    firmware_flash.clear_pending(tmp_path)
    assert firmware_flash.pending(tmp_path) is None
    # Повторная очистка не падает: заявки уже нет.
    firmware_flash.clear_pending(tmp_path)


def test_without_a_marker_nothing_is_flashed(tmp_path, monkeypatch):
    monkeypatch.delenv("PYTEST_CURRENT_TEST", raising=False)
    called = []
    monkeypatch.setattr(firmware_flash.subprocess, "run",
                        lambda *a, **k: called.append(a) or pytest.fail("прошивка не нужна"))
    assert firmware_flash.flash_if_pending(tmp_path) is None
    assert called == []


def test_request_runs_the_script_and_clears_itself(tmp_path, monkeypatch):
    monkeypatch.delenv("PYTEST_CURRENT_TEST", raising=False)
    (tmp_path / "scripts").mkdir()
    (tmp_path / "scripts" / "flash_firmware.sh").write_text("#!/bin/sh\n", encoding="utf-8")
    firmware_flash.mark_pending("обновление с GitHub", "abc1234", tmp_path)
    seen = {}

    class Result:
        returncode = 0
        stdout = "[firmware] колесо FL: прошито (/dev/ttyUSB0)\n"
        stderr = ""

    def fake_run(command, **kwargs):
        seen["command"] = command
        seen["cwd"] = kwargs.get("cwd")
        return Result()

    monkeypatch.setattr(firmware_flash.subprocess, "run", fake_run)
    report = firmware_flash.flash_if_pending(tmp_path)
    assert report["ok"] is True and report["code"] == 0
    assert seen["command"][:2] == ["bash", str(tmp_path / "scripts" / "flash_firmware.sh")]
    assert "--reason" in seen["command"]
    assert seen["cwd"] == str(tmp_path.resolve())
    # Заявка снята: следующий старт не прошивает платы повторно.
    assert firmware_flash.pending(tmp_path) is None


def test_missing_arduino_cli_is_a_warning_not_a_crash(tmp_path, monkeypatch):
    monkeypatch.delenv("PYTEST_CURRENT_TEST", raising=False)
    (tmp_path / "scripts").mkdir()
    (tmp_path / "scripts" / "flash_firmware.sh").write_text("#!/bin/sh\n", encoding="utf-8")
    firmware_flash.mark_pending("обновление", "", tmp_path)

    class Result:
        returncode = firmware_flash.EXIT_TOOL_MISSING
        stdout = "[firmware] arduino-cli не установлен — платы не прошиты автоматически.\n"
        stderr = ""

    monkeypatch.setattr(firmware_flash.subprocess, "run", lambda *a, **k: Result())
    report = firmware_flash.flash_if_pending(tmp_path)
    assert report["ok"] is False and report["code"] == firmware_flash.EXIT_TOOL_MISSING
    assert "arduino-cli" in report["output"]
    assert firmware_flash.pending(tmp_path) is None


# ------------------------------------------------------------- скрипт --- #


@pytest.mark.parametrize("name", ["flash_firmware.sh"])
def test_script_is_in_place_and_runs(name):
    path = SCRIPTS / name
    assert path.is_file(), name
    assert os.access(path, os.X_OK), "%s должен быть исполняемым" % name
    syntax = subprocess.run(["bash", "-n", str(path)], capture_output=True, text=True, check=False)
    assert syntax.returncode == 0, syntax.stderr


def test_no_ports_configured_is_explained(flashing_host):
    result = run_flash()
    assert result.returncode == 3
    assert "порты колёс не выбраны" in result.stdout
    assert "Настройка портов колёс" in result.stdout
    assert flashing_host.calls.read_text(encoding="utf-8") == ""


def test_assigned_ports_are_flashed_once(flashing_host):
    fl, fr = flashing_host.port("port-fl"), flashing_host.port("port-fr")
    flashing_host.assign("RC_WHEEL_FL_PORT=%s" % fl, "RC_WHEEL_FR_PORT=%s" % fr)

    result = run_flash("--reason", "обновление с GitHub")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "прошито 2" in result.stdout
    assert "причина: обновление с GitHub" in result.stdout
    calls = flashing_host.calls.read_text(encoding="utf-8").splitlines()
    assert len(calls) == 2
    assert all("compile --upload" in line and "--fqbn arduino:avr:mega" in line for line in calls)
    assert str(fl) in calls[0] and str(fr) in calls[1]
    # Имя колеса вписано в скетч, иначе все четыре платы назвались бы FL.
    sketch = flashing_host.var / "firmware-build" / "motor_wheel_FL" / "motor_wheel_FL.ino"
    assert '#define MODULE_ID "FL"' in sketch.read_text(encoding="utf-8")

    # второй запуск ничего не прошивает: уже эта прошивка
    again = run_flash()
    assert again.returncode == 0
    assert "прошивать нечего" in again.stdout
    assert len(flashing_host.calls.read_text(encoding="utf-8").splitlines()) == 2


def test_force_flashes_even_the_current_firmware(flashing_host):
    flashing_host.assign("RC_WHEEL_FL_PORT=%s" % flashing_host.port("port-fl"))
    assert run_flash().returncode == 0
    assert run_flash("--force").returncode == 0
    assert len(flashing_host.calls.read_text(encoding="utf-8").splitlines()) == 2


def test_check_only_shows_the_plan(flashing_host):
    flashing_host.assign("RC_WHEEL_FL_PORT=%s" % flashing_host.port("port-fl"))
    result = run_flash("--check")
    assert result.returncode == 0
    assert "прошить" in result.stdout
    assert flashing_host.calls.read_text(encoding="utf-8") == ""


def test_missing_port_device_is_skipped(flashing_host):
    flashing_host.assign("RC_WHEEL_FL_PORT=/dev/ttyUSB9")   # устройства нет
    result = run_flash()
    assert result.returncode == 0
    assert "устройства нет сейчас" in result.stdout
    assert flashing_host.calls.read_text(encoding="utf-8") == ""


def test_failed_upload_is_reported(flashing_host, monkeypatch):
    flashing_host.assign("RC_WHEEL_FL_PORT=%s" % flashing_host.port("port-fl"))
    monkeypatch.setenv("FAKE_ARDUINO_CLI_FAIL", "1")
    result = run_flash()
    assert result.returncode == 1
    assert "прошивка не удалась" in result.stderr


def test_without_arduino_cli_the_boards_are_left_alone(flashing_host, monkeypatch):
    flashing_host.assign("RC_WHEEL_FL_PORT=%s" % flashing_host.port("port-fl"))
    # PATH без arduino-cli: остаются только системные утилиты.
    monkeypatch.setenv("PATH", "/usr/bin:/bin")
    result = run_flash()
    assert result.returncode == 2
    assert "arduino-cli не установлен" in result.stderr
    assert "motor_wheel_*.ino" in result.stderr


def test_switch_off_disables_autoflashing(flashing_host, monkeypatch):
    flashing_host.assign("RC_WHEEL_FL_PORT=%s" % flashing_host.port("port-fl"))
    monkeypatch.setenv("RC_FIRMWARE_AUTOFLASH", "0")
    result = run_flash()
    assert result.returncode == 0
    assert "автопрошивка выключена" in result.stdout
    assert flashing_host.calls.read_text(encoding="utf-8") == ""


def test_desktop_copies_are_still_made():
    """Автопрошивка не заменяет копии на рабочем столе: они остаются."""
    sync = (SCRIPTS / "sync_firmware_desktop.sh").read_text(encoding="utf-8")
    assert "motor_wheel_${module}.ino" in sync
    flash = (SCRIPTS / "flash_firmware.sh").read_text(encoding="utf-8")
    assert "firmware/motor_wheel/motor_wheel.ino" in flash


def test_update_leaves_a_flash_request(tmp_path):
    """После обновления с GitHub остаётся заявка на автопрошивку."""
    from robot_control.updater import UpdateManager

    manager = UpdateManager(tmp_path)
    firmware_flash.mark_pending("обновление с GitHub", "deadbee", tmp_path)
    request = firmware_flash.pending(manager.root)
    assert request is not None
    assert json.loads((tmp_path / "var" / "firmware-pending.json").read_text(encoding="utf-8"))
