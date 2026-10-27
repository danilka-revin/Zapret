"""Прошивка плат без arduino-cli: avr-gcc + avrdude и ядро с GitHub.

В России ``downloads.arduino.cc`` недоступен, поэтому arduino-cli там не
поставить. Запасной путь — то, что и так есть в системе: ``gcc-avr`` /
``avr-libc`` / ``avrdude`` из apt, ядро ArduinoCore-avr и библиотека Servo с
GitHub (codeload), сборка ``avr-gcc`` и заливка ``avrdude`` теми же ключами,
что у Arduino IDE для Mega 2560.

Проверяем ровно то, от чего зависит «получится или нет»: поиск инструментов,
адреса загрузки, скачивание/распаковку ядра и библиотеки, команду avrdude и
путь установки через apt.
"""

from __future__ import annotations

import importlib.util
import io
import json
import os
import subprocess
import tarfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
SCRIPT = ROOT / "scripts" / "avr_upload.py"

TOOLS = ("avr-gcc", "avr-g++", "avr-objcopy", "avr-size", "avrdude")


def _load():
    spec = importlib.util.spec_from_file_location("avr_upload_test", str(SCRIPT))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def clean_env(tmp_path, monkeypatch):
    """Ничего не находим случайно: пустой PATH, свой HOME, пустые подсказки."""
    empty = tmp_path / "empty"
    empty.mkdir()
    monkeypatch.setenv("PATH", "/usr/bin:/bin")
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    monkeypatch.setenv("RC_AVR_ROOT", str(empty))
    monkeypatch.delenv("RC_ARDUINO_CORE", raising=False)
    monkeypatch.delenv("RC_ARDUINO_LIBS", raising=False)
    monkeypatch.delenv("RC_FIRMWARE_MIRROR", raising=False)
    return tmp_path


def _fake_tools(directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    for name in TOOLS:
        path = directory / name
        path.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
        path.chmod(0o755)


def _fake_core(directory: Path) -> Path:
    core = directory / "ArduinoCore-avr"
    (core / "cores" / "arduino").mkdir(parents=True)
    (core / "cores" / "arduino" / "Arduino.h").write_text("// core\n", encoding="utf-8")
    return core


def _run(*args, env=None):
    merged = dict(os.environ)
    merged.update(env or {})
    return subprocess.run(["python3", str(SCRIPT), *args], capture_output=True,
                          text=True, env=merged, check=False)


# ------------------------------------------------------------- инструменты --- #


def test_check_without_tools_says_it_cannot_flash(clean_env):
    result = _run("--check")
    assert result.returncode == 2
    assert "шить можно: нет" in result.stdout


def test_check_json_is_machine_readable(clean_env):
    result = _run("--check", "--json")
    payload = json.loads(result.stdout)
    assert payload["ready"] is False
    assert payload["gcc"] is None and payload["avrdude"] is None


def test_check_finds_avr_tools_and_the_core(clean_env):
    shims = clean_env / "bin"
    _fake_tools(shims)
    core = _fake_core(clean_env)
    env = {"PATH": "%s%s/usr/bin:/bin" % (shims, os.pathsep),
           "RC_ARDUINO_CORE": str(core)}
    result = _run("--check", "--json", env=env)
    payload = json.loads(result.stdout)
    assert result.returncode == 0, result.stdout + result.stderr
    assert payload["ready"] is True
    assert payload["gcc"].endswith("avr-gcc")
    assert payload["avrdude"].endswith("avrdude")
    assert payload["core"] == str(core)


# ----------------------------------------------------------------- загрузка --- #


def test_core_and_servo_are_downloaded_from_github(clean_env):
    """downloads.arduino.cc в России блокирован — берём архивы с GitHub."""
    module = _load()
    url = module.mirror_url("arduino/ArduinoCore-avr", "1.8.6")
    assert url == ("https://codeload.github.com/arduino/ArduinoCore-avr"
                   "/tar.gz/refs/tags/1.8.6")
    servo = module.mirror_url("arduino-libraries/Servo", "1.2.1")
    assert servo.startswith("https://codeload.github.com/arduino-libraries/Servo/")


def test_mirror_can_be_overridden_for_an_offline_lan(clean_env, monkeypatch):
    module = _load()
    monkeypatch.setenv("RC_FIRMWARE_MIRROR", "http://robot.lan/firmware")
    assert module.mirror_url("arduino/ArduinoCore-avr", "1.8.6") == \
        "http://robot.lan/firmware/arduino/ArduinoCore-avr/1.8.6.tar.gz"
    monkeypatch.setenv("RC_FIRMWARE_MIRROR",
                       "http://robot.lan/{repo}/{tag}.tar.gz")
    assert module.mirror_url("arduino/ArduinoCore-avr", "1.8.6") == \
        "http://robot.lan/arduino/ArduinoCore-avr/1.8.6.tar.gz"


def _archive(path: Path, top: str, member: str, text: str) -> None:
    data = text.encode("utf-8")
    with tarfile.open(path, "w:gz") as tar:
        info = tarfile.TarInfo("%s/%s" % (top, member))
        info.size = len(data)
        tar.addfile(info, io.BytesIO(data))


def test_core_is_unpacked_from_the_archive(clean_env, monkeypatch):
    module = _load()
    project = clean_env / "project"

    def fake_download(url, target, timeout=120.0):
        assert "codeload.github.com/arduino/ArduinoCore-avr" in url
        _archive(target, "ArduinoCore-avr-1.8.6", "cores/arduino/Arduino.h", "// core\n")
        return True

    monkeypatch.setattr(module, "_download", fake_download)
    core = module.ensure_core(project)
    assert core is not None
    assert (Path(core) / "cores" / "arduino" / "Arduino.h").is_file()
    # Архив после распаковки не валяется: место на борту не бесконечное.
    assert not (project / "var" / "toolchain" / "ArduinoCore-avr.tar.gz").exists()


def test_servo_library_is_unpacked_from_the_archive(clean_env, monkeypatch):
    module = _load()
    project = clean_env / "project"

    def fake_download(url, target, timeout=120.0):
        assert "codeload.github.com/arduino-libraries/Servo" in url
        _archive(target, "Servo-1.2.1", "src/Servo.h", "// servo\n")
        return True

    monkeypatch.setattr(module, "_download", fake_download)
    library = module.ensure_library("Servo", project)
    assert library is not None and (Path(library) / "src" / "Servo.h").is_file()


def test_network_failure_is_reported_not_raised(clean_env, monkeypatch):
    module = _load()

    def broken_download(url, target, timeout=120.0):
        return False

    monkeypatch.setattr(module, "_download", broken_download)
    assert module.ensure_core(clean_env / "project") is None
    assert module.ensure_library("Servo", clean_env / "project") is None


# --------------------------------------------------------------- avrdude --- #


def test_upload_uses_the_same_settings_as_arduino_ide(clean_env):
    module = _load()
    tools = {"avrdude": "/usr/bin/avrdude", "avrdude_conf": "/etc/avrdude.conf"}
    command = module.upload_command(Path("/tmp/motor.hex"), "/dev/ttyUSB0", tools)
    assert command[:3] == ["/usr/bin/avrdude", "-C", "/etc/avrdude.conf"]
    for part in (["-p", "atmega2560"], ["-c", "wiring"], ["-P", "/dev/ttyUSB0"],
                 ["-b", "115200"], ["-D", "-U"]):
        offset = command.index(part[0])
        assert command[offset:offset + len(part)] == part, part
    assert command[-1] == "flash:w:/tmp/motor.hex:i"
    assert "arduino-cli" not in " ".join(command)


def test_upload_without_avrdude_is_an_error(clean_env, monkeypatch):
    module = _load()
    monkeypatch.setattr(module, "find_toolchain", lambda: {"avrdude": None})
    with pytest.raises(RuntimeError) as error:
        module.upload(Path("/tmp/motor.hex"), "/dev/ttyUSB0")
    assert "avrdude" in str(error.value)


# ------------------------------------------------------------ установка apt --- #


def test_install_uses_apt_packages_available_in_russia(clean_env, monkeypatch):
    module = _load()
    calls = []

    def ready_after_install():
        # До установки инструментов нет, после — «появились».
        return {"ready": bool(calls), "gcc": None, "avrdude": None}

    monkeypatch.setattr(module, "find_toolchain", ready_after_install)
    monkeypatch.setattr(module, "say", lambda text: None)
    monkeypatch.setattr(module, "warn", lambda text: None)

    def fake_run(command, **kwargs):
        calls.append(list(command))
        return subprocess.CompletedProcess(command, 0, stdout="", stderr="")

    monkeypatch.setattr(module.subprocess, "run", fake_run)
    assert module.install_tools() is True
    assert calls[0] == ["sudo", "-n", "apt-get", "install", "-y",
                        "gcc-avr", "avr-libc", "avrdude"]


def test_install_explains_when_apt_fails(clean_env, monkeypatch):
    module = _load()
    warnings = []
    monkeypatch.setattr(module, "find_toolchain",
                        lambda: {"ready": False, "gcc": None, "avrdude": None})
    monkeypatch.setattr(module, "say", lambda text: None)
    monkeypatch.setattr(module, "warn", warnings.append)
    monkeypatch.setattr(module.subprocess, "run",
                        lambda command, **kwargs: subprocess.CompletedProcess(
                            command, 100, stdout="", stderr="E: Could not open lock"))
    assert module.install_tools() is False
    assert any("sudo apt install gcc-avr avr-libc avrdude" in line for line in warnings)
