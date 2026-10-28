"""Прошивка плат колёс — только вручную, утилитой из терминала.

В программе прошивки нет вовсе: ни автопрошивки при старте, ни кнопки на экране.
Скрипт ``scripts/flash_firmware.sh`` берёт
порты из настроек (``RC_WHEEL_*_PORT``), собирает прошивку из
``firmware/motor_wheel/motor_wheel.ino`` и сверяет ``fw=`` с
``FIRMWARE_PROTOCOL`` из скетча. Файлы прошивок остаются на рабочем столе.
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent


def expected_protocol(root: Path | None = None) -> int | None:
    """FIRMWARE_PROTOCOL из скетча колеса — эталон «актуальной версии».

    В программе прошивки плат больше нет (экран не прошивает ничего), но скрипт
    `scripts/flash_firmware.sh` и опрос плат по @TLM сверяются именно с этим
    числом, поэтому тест читает его прямо из скетча.
    """
    path = Path(root or ROOT) / "firmware" / "motor_wheel" / "motor_wheel.ino"
    if not path.is_file():
        return None
    match = re.search(r"FIRMWARE_PROTOCOL\s*=\s*(\d+)", path.read_text(encoding="utf-8"))
    return int(match.group(1)) if match else None
SCRIPTS = ROOT / "scripts"

FAKE_ARDUINO_CLI = """#!/usr/bin/env bash
printf '%s\\n' "$*" >> "$FAKE_LOG"
if [[ "${1:-}" == "core" && "${2:-}" == "list" ]]; then
  echo "arduino:avr 1.8.6 Arduino AVR Boards"; exit 0
fi
# Настоящая загрузка меняет прошивку на плате: фальшивая — тоже, если попросят.
if [[ "$*" == *"compile --upload"* && -n "${FAKE_FLASH_NEW:-}" && -n "${FAKE_PROBE_FILE:-}" ]]; then
  printf '%s\\n' "$FAKE_FLASH_NEW" > "$FAKE_PROBE_FILE"
fi
if [[ "${FAKE_ARDUINO_CLI_FAIL:-}" == "1" ]]; then
  echo "avrdude: stk500_recv(): programmer is not responding" >&2
  exit 1
fi
exit 0
"""

# Платы «отвечают» тем, что скажет тест: как будто прочитали fw= из @TLM.
# Если задан FAKE_PROBE_FILE — ответ берётся из него: так фальшивая прошивка
# «меняет» версию на плате, и видно, что проверка после загрузки работает.
FAKE_PROBE = """#!/usr/bin/env bash
if [[ -n "${FAKE_PROBE_FILE:-}" && -f "${FAKE_PROBE_FILE}" ]]; then
  cat "$FAKE_PROBE_FILE"
  exit 0
fi
[[ -n "${FAKE_PROBE_OUT:-}" ]] || exit 1
printf '%s\\n' "$FAKE_PROBE_OUT"
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
    probe = shims / "probe"
    probe.write_text(FAKE_PROBE, encoding="utf-8")
    probe.chmod(0o755)
    monkeypatch.setenv("PATH", "%s%s%s" % (shims, os.pathsep, os.environ.get("PATH", "")))
    monkeypatch.setenv("FAKE_LOG", str(log))
    monkeypatch.setenv("RC_ENV_FILE", str(env_file))
    monkeypatch.setenv("RC_VAR_DIR", str(tmp_path / "var"))
    monkeypatch.setenv("RC_FIRMWARE_PROBE_CMD", str(probe))
    monkeypatch.setenv("FAKE_PROBE_FILE", str(tmp_path / "probe.out"))
    monkeypatch.delenv("FAKE_FLASH_NEW", raising=False)
    monkeypatch.delenv("FAKE_PROBE_OUT", raising=False)
    monkeypatch.delenv("RC_FIRMWARE_VERIFY", raising=False)
    monkeypatch.delenv("RC_FIRMWARE_AUTOCLI", raising=False)
    # Ни apt, ни скачивание arduino-cli в тестах не запускаются: установка
    # инструментов проверяется отдельными тестами (tests/test_avr_upload.py).
    monkeypatch.setenv("RC_FIRMWARE_INSTALL", "0")

    class Host:
        root = tmp_path
        calls = log
        env = env_file
        var = tmp_path / "var"

        def board_says(self, text):
            """Что платы отвечают про fw= (строка вида «FL=5 FR=- RL=- RR=-»)."""
            (tmp_path / "probe.out").write_text(text + "\n", encoding="utf-8")

        def assign(self, *pairs):
            lines = list(pairs)
            self.env.write_text("\n".join(lines) + "\n", encoding="utf-8")

        def port(self, name):
            path = tmp_path / name
            path.write_text("", encoding="utf-8")
            return path

    Host.probe_path = probe
    return Host()


def run_flash(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(["bash", str(SCRIPTS / "flash_firmware.sh"), *args],
                          capture_output=True, text=True, check=False)


def flashes(host) -> list[str]:
    """Только строки загрузки: `core list` в журнал не считается прошивкой."""
    return [line for line in host.calls.read_text(encoding="utf-8").splitlines()
            if "compile --upload" in line]


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
    calls = flashes(flashing_host)
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
    assert len(flashes(flashing_host)) == 2


def test_force_flashes_even_the_current_firmware(flashing_host):
    flashing_host.assign("RC_WHEEL_FL_PORT=%s" % flashing_host.port("port-fl"))
    assert run_flash().returncode == 0
    assert run_flash("--force").returncode == 0
    assert len(flashes(flashing_host)) == 2


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


def test_without_any_tool_the_boards_are_left_alone(flashing_host, monkeypatch):
    flashing_host.assign("RC_WHEEL_FL_PORT=%s" % flashing_host.port("port-fl"))
    # PATH без arduino-cli и без avr-gcc: остаются только системные утилиты.
    monkeypatch.setenv("PATH", "/usr/bin:/bin")
    monkeypatch.setenv("RC_FIRMWARE_AUTOCLI", "0")
    result = run_flash()
    assert result.returncode == 2
    assert "нечем прошивать" in result.stderr
    # Главная подсказка — путь, который работает в России и без arduino-cli.
    assert "sudo apt install gcc-avr avr-libc avrdude" in result.stderr
    assert "motor_wheel_*.ino" in result.stderr


def test_flashing_goes_through_avr_gcc_when_arduino_cli_is_missing(flashing_host,
                                                                  monkeypatch, tmp_path):
    """Без arduino-cli скрипт выбирает avr-gcc + avrdude, а не сдаётся."""
    flashing_host.assign("RC_WHEEL_FL_PORT=%s" % flashing_host.port("port-fl"))
    shims = tmp_path / "avr"
    shims.mkdir()
    for name in ("avr-gcc", "avr-g++", "avr-objcopy", "avr-size", "avrdude"):
        tool = shims / name
        tool.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
        tool.chmod(0o755)
    core = tmp_path / "ArduinoCore-avr"
    (core / "cores" / "arduino").mkdir(parents=True)
    (core / "cores" / "arduino" / "Arduino.h").write_text("// core\n", encoding="utf-8")
    monkeypatch.setenv("PATH", "%s%s/usr/bin:/bin" % (shims, os.pathsep))
    monkeypatch.setenv("RC_ARDUINO_CORE", str(core))
    monkeypatch.setenv("RC_FIRMWARE_BACKEND", "avr")
    result = run_flash()
    # Сборка на пустышках не пройдёт — важно, что скрипт пошёл родным путём,
    # а не отправил человека на закрытый сайт Arduino.
    assert "прошиваю без arduino-cli: avr-gcc + avrdude" in result.stdout
    assert result.returncode == 1


def test_expected_protocol_comes_from_the_sketch():
    """«Актуальная версия» — это FIRMWARE_PROTOCOL из скетча, а не догадка."""
    sketch = (ROOT / "firmware" / "motor_wheel" / "motor_wheel.ino").read_text(encoding="utf-8")
    expected = expected_protocol()
    assert isinstance(expected, int) and expected > 0
    assert "FIRMWARE_PROTOCOL = %d" % expected in sketch
    # Нет скетча — нет и версии: скрипт тогда сверяет только хеш.
    assert expected_protocol(ROOT / "nope") is None


# ------------------------------------------------- проверка по самой плате --- #


def test_board_with_old_firmware_is_reflashed_even_if_marked_current(flashing_host, monkeypatch):
    """«Уже прошито» по локальной отметке — ещё не «на плате эта версия».

    Плату могли заменить, прошить старой копией с рабочего стола или прошивка
    не дошла. Плата сообщает свой fw=, и при расхождении с кодом её прошивают
    заново — даже если локальная отметка говорит «актуально».
    """
    flashing_host.assign("RC_WHEEL_FL_PORT=%s" % flashing_host.port("port-fl"))
    expected = expected_protocol()
    assert run_flash().returncode == 0                     # первая прошивка
    assert len(flashes(flashing_host)) == 1

    # Плата отвечает, что у неё старая прошивка (например, её заменили), а
    # после загрузки отвечает уже новой — как настоящая.
    flashing_host.board_says("FL=%d FR=- RL=- RR=-" % (expected - 1))
    monkeypatch.setenv("FAKE_FLASH_NEW", "FL=%d FR=- RL=- RR=-" % expected)
    result = run_flash()
    assert result.returncode == 0, result.stdout + result.stderr
    assert "на плате fw=%d, в коде fw=%d" % (expected - 1, expected) in result.stdout
    assert len(flashes(flashing_host)) == 2, "плату с устаревшей прошивкой надо прошить"
    assert "подтверждено по @TLM — fw=%d" % expected in result.stdout

    # Дальше плата на актуальной прошивке — её больше не трогают.
    confirm = run_flash()
    assert confirm.returncode == 0
    assert "прошивка актуальна" in confirm.stdout
    assert len(flashes(flashing_host)) == 2


def test_after_flashing_the_board_is_asked_which_version_it_runs(flashing_host, monkeypatch):
    """Без подтверждения от платы «загружено» остаётся только обещанием."""
    flashing_host.assign("RC_WHEEL_FL_PORT=%s" % flashing_host.port("port-fl"))
    expected = expected_protocol()
    flashing_host.board_says("FL=%d FR=- RL=- RR=-" % expected)
    result = run_flash()
    assert result.returncode == 0, result.stdout + result.stderr
    assert "проверяю, что платы действительно на fw=%d" % expected in result.stdout
    assert "подтверждено по @TLM — fw=%d" % expected in result.stdout


def test_board_that_answers_with_the_wrong_version_after_flash_is_reported(flashing_host,
                                                                         monkeypatch):
    flashing_host.assign("RC_WHEEL_FL_PORT=%s" % flashing_host.port("port-fl"))
    expected = expected_protocol()
    # Загрузка «прошла», но плата продолжает отвечать старой версией.
    flashing_host.board_says("FL=%d FR=- RL=- RR=-" % (expected - 1))
    result = run_flash()
    assert result.returncode == 1
    assert "сообщает fw=%d вместо %d" % (expected - 1, expected) in result.stderr
    # Отметка «не удалось»: сама такая плата не перепрошивается в цикле.
    state = (flashing_host.var / "firmware-flashed" / "FL.flash").read_text(encoding="utf-8")
    assert state.endswith("fail\n")
    again = run_flash()
    assert "--force" in again.stdout
    assert len(flashes(flashing_host)) == 1


def test_without_an_answer_the_local_mark_is_used(flashing_host, monkeypatch):
    flashing_host.assign("RC_WHEEL_FL_PORT=%s" % flashing_host.port("port-fl"))
    assert run_flash().returncode == 0
    (flashing_host.root / "probe.out").unlink(missing_ok=True)  # платы молчат
    monkeypatch.setenv("FAKE_PROBE_OUT", "")               # платы молчат
    result = run_flash()
    assert result.returncode == 0
    assert "не ответили на опрос" in result.stdout
    assert len(flashes(flashing_host)) == 1


def test_verification_can_be_switched_off(flashing_host, monkeypatch):
    flashing_host.assign("RC_WHEEL_FL_PORT=%s" % flashing_host.port("port-fl"))
    expected = expected_protocol()
    assert run_flash().returncode == 0
    flashing_host.board_says("FL=%d FR=- RL=- RR=-" % (expected - 1))
    result = run_flash("--no-verify")
    assert result.returncode == 0
    assert len(flashes(flashing_host)) == 1, "с --no-verify плату не трогают"


# ------------------------------------------------------- arduino-cli сам --- #


def test_installer_check_reports_missing_cli(tmp_path, monkeypatch):
    monkeypatch.setenv("PATH", "/usr/bin:/bin")
    monkeypatch.setenv("RC_ARDUINO_CLI_DIR", str(tmp_path / "cli"))
    result = subprocess.run(["bash", str(SCRIPTS / "install_arduino_cli.sh"), "--check"],
                            capture_output=True, text=True, check=False)
    assert result.returncode == 2
    assert "не установлен" in result.stdout


def test_installer_accepts_an_existing_cli(flashing_host):
    result = subprocess.run(["bash", str(SCRIPTS / "install_arduino_cli.sh"), "--check"],
                            capture_output=True, text=True, check=False)
    assert result.returncode == 0
    assert "уже есть" in result.stdout


def test_installer_downloads_the_official_archive(tmp_path, monkeypatch):
    """Нет arduino-cli — скрипт скачивает официальный архив и ставит ядро AVR."""
    payload = tmp_path / "payload"
    payload.mkdir()
    (payload / "arduino-cli").write_text("#!/usr/bin/env bash\n"
                                         "echo \"$*\" >> \"$FAKE_LOG\"\n"
                                         "exit 0\n", encoding="utf-8")
    (payload / "arduino-cli").chmod(0o755)
    archive = tmp_path / "arduino-cli.tar.gz"
    subprocess.run(["tar", "-czf", str(archive), "-C", str(payload), "arduino-cli"], check=True)
    log = tmp_path / "calls.log"
    log.write_text("", encoding="utf-8")
    shims = tmp_path / "bin"
    shims.mkdir()
    (shims / "curl").write_text(
        "#!/usr/bin/env bash\nprintf 'curl %s\\n' \"$*\" >> \"$FAKE_LOG\"\n"
        "out=''\nwhile [[ $# -gt 0 ]]; do case \"$1\" in -o) out=\"$2\"; shift 2;; "
        "*) shift;; esac; done\ncp \"$FAKE_ARCHIVE\" \"$out\"\n", encoding="utf-8")
    (shims / "curl").chmod(0o755)
    target = tmp_path / "cli"
    monkeypatch.setenv("PATH", "%s:/usr/bin:/bin" % shims)
    monkeypatch.setenv("FAKE_LOG", str(log))
    monkeypatch.setenv("FAKE_ARCHIVE", str(archive))
    monkeypatch.setenv("RC_ARDUINO_CLI_DIR", str(target))
    monkeypatch.chdir(tmp_path)

    result = subprocess.run(["bash", str(SCRIPTS / "install_arduino_cli.sh")],
                            capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
    assert (target / "arduino-cli").is_file()
    calls = log.read_text(encoding="utf-8")
    # Сначала GitHub: downloads.arduino.cc в России блокирован.
    assert "github.com/arduino/arduino-cli/releases" in calls
    assert calls.index("github.com") < calls.index("downloads.arduino.cc") \
        if "downloads.arduino.cc" in calls else True
    assert "core install arduino:avr" in calls


def test_flash_installs_the_cli_when_it_is_missing(flashing_host, monkeypatch, tmp_path):
    """Нет arduino-cli — автопрошивка ставит его сама и прошивает платы."""
    flashing_host.assign("RC_WHEEL_FL_PORT=%s" % flashing_host.port("port-fl"))
    cli_dir = tmp_path / "cli"
    cli_dir.mkdir()
    (cli_dir / "arduino-cli").write_text(FAKE_ARDUINO_CLI, encoding="utf-8")
    (cli_dir / "arduino-cli").chmod(0o755)
    # PATH без arduino-cli: он появится только после «установки».
    shims = tmp_path / "bin"
    installer = shims / "install_arduino_cli.sh"
    installer.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
    installer.chmod(0o755)
    monkeypatch.setenv("PATH", "/usr/bin:/bin")
    monkeypatch.setenv("RC_ARDUINO_CLI_DIR", str(cli_dir))
    result = run_flash()
    assert result.returncode == 0, result.stdout + result.stderr
    assert len(flashes(flashing_host)) == 1


# ------------------------------------------------ опрос плат по @TLM --- #


def _load_probe():
    import importlib.util

    path = SCRIPTS / "firmware_probe.py"
    spec = importlib.util.spec_from_file_location("firmware_probe_test", str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_probe_parses_the_firmware_field():
    probe = _load_probe()
    line = ("@TLM mod=FL deg=0.00 tgt=0.00 moving=0 cal=1 rpm=0.0 drive=0 fw=9 "
            "pot=1 pm=1 tach=1 dmax=100 us=1500 ppr=90 kmh=50 t=1234")
    assert probe.parse_firmware(line) == 9
    # Строка журнала прошивки или обрывок — не телеметрия.
    assert probe.parse_firmware("# FL: готов") is None
    assert probe.parse_firmware("@TLM deg=0 fw=") is None
    assert probe.parse_firmware("") is None


def test_probe_reads_wheel_ports_from_env_file(tmp_path, monkeypatch):
    probe = _load_probe()
    for module in probe.MODULES:
        monkeypatch.delenv(probe.ENV_KEYS[module], raising=False)
    env = tmp_path / "env"
    env.write_text("# комментарий\nRC_WHEEL_FL_PORT=/dev/ttyUSB0\n"
                   "RC_WHEEL_RR_PORT=\nRC_WHEEL_FR_PORT=\"/dev/ttyACM1\"\n",
                   encoding="utf-8")
    ports = probe.read_env_ports(env)
    assert ports["FL"] == "/dev/ttyUSB0" and ports["FR"] == "/dev/ttyACM1"
    assert ports["RR"] == "" and ports["RL"] == ""
    # Окружение главнее файла — как и у самой программы.
    monkeypatch.setenv("RC_WHEEL_RL_PORT", "/dev/ttyUSB7")
    assert probe.read_env_ports(env)["RL"] == "/dev/ttyUSB7"
    assert probe.read_env_ports(tmp_path / "нет-такого")["FL"] == ""


def test_probe_asks_every_board_and_tolerates_silence(tmp_path, monkeypatch):
    import sys
    import types

    probe = _load_probe()
    answers = {"/dev/fake-fl": b"@TLM deg=0 fw=6 t=1\n",
               "/dev/fake-fr": b""}
    closed = []

    class FakeSerial:
        def __init__(self, port, baud, timeout=0.2, exclusive=True):
            self.port = port
            self.lines = 0

        def readline(self):
            self.lines += 1
            if self.lines > 2:
                return b""
            return answers.get(self.port, b"")

        def close(self):
            closed.append(self.port)

    fake = types.ModuleType("serial")
    fake.Serial = FakeSerial
    monkeypatch.setitem(sys.modules, "serial", fake)
    ports = {"FL": "/dev/fake-fl", "FR": "/dev/fake-fr", "RL": "", "RR": "/dev/boom"}
    versions = probe.probe(ports, timeout=0.2)
    assert versions["FL"] == 6
    assert versions["FR"] is None and versions["RL"] is None and versions["RR"] is None
    assert "/dev/fake-fl" in closed      # порт обязательно закрывается


def test_probe_writes_a_single_machine_readable_line(tmp_path, monkeypatch, capsys):
    import sys
    import types

    probe = _load_probe()
    for module in probe.MODULES:
        monkeypatch.delenv(probe.ENV_KEYS[module], raising=False)

    class FakeSerial:
        def __init__(self, *args, **kwargs):
            pass

        def readline(self):
            return b"@TLM fw=6 t=1\n"

        def close(self):
            pass

    fake = types.ModuleType("serial")
    fake.Serial = FakeSerial
    monkeypatch.setitem(sys.modules, "serial", fake)
    env = tmp_path / "env"
    env.write_text("RC_WHEEL_FL_PORT=/dev/fake-fl\nRC_WHEEL_FR_PORT=/dev/fake-fr\n",
                   encoding="utf-8")
    assert probe.main(["--env-file", str(env), "--timeout", "0.2"]) == 0
    out = capsys.readouterr().out.strip()
    assert out == "FL=6 FR=6 RL=- RR=-"
    # JSON — для тех, кто вызывает из кода.
    assert probe.main(["--env-file", str(env), "--timeout", "0.2", "--json"]) == 0
    assert json.loads(capsys.readouterr().out)["FL"] == 6


def test_desktop_copies_are_still_made():
    """Автопрошивка не заменяет копии на рабочем столе: они остаются."""
    sync = (SCRIPTS / "sync_firmware_desktop.sh").read_text(encoding="utf-8")
    assert "motor_wheel_${module}.ino" in sync
    flash = (SCRIPTS / "flash_firmware.sh").read_text(encoding="utf-8")
    assert "firmware/motor_wheel/motor_wheel.ino" in flash
