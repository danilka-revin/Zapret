"""Автозапуск приложения: по умолчанию его нет, а включённый — выключается скриптом.

Проверяем поведение, из-за которого робот «сам поднимал» пульт и экран:
`install_ubuntu.sh` больше не включает автозапуск без флага, `make_shortcut.sh`
по умолчанию не ставит ярлык в автозапуск сеанса, а `scripts/autostart.sh` умеет
показать, выключить и вернуть автозапуск (службы systemd, ярлык сеанса,
автоподъём точки доступа Wi-Fi). `autostart.sh off` = «выключить всё»: он ещё
и останавливает запущенное (через `scripts/stop.sh`), а `--keep-running`
выключает только автозапуск. `scripts/stop.sh` останавливает и серверы по
pid-файлам, и службы, и окно киоск-браузера.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parent.parent / "scripts"

FAKE_SYSTEMCTL = """#!/usr/bin/env bash
printf '%s\\n' "$*" >> "$FAKE_LOG"
case "$1" in
  list-unit-files) exit 0 ;;
  cat)             exit 0 ;;
  is-enabled)      echo "${FAKE_UNIT_ENABLED:-disabled}"; exit 0 ;;
  is-active)       echo "${FAKE_UNIT_ACTIVE:-inactive}"; exit 0 ;;
  disable|enable)  exit 0 ;;
esac
exit 0
"""

FAKE_NMCLI = """#!/usr/bin/env bash
printf '%s\\n' "$*" >> "$FAKE_LOG"
if [[ "$*" == *"-g"* ]]; then
  echo "${FAKE_AP_AUTOCONNECT:-no}"; exit 0
fi
if [[ "$*" == *"connection show"* ]]; then
  echo "robot-ap"; exit 0
fi
exit 0
"""

FAKE_SUDO = """#!/usr/bin/env bash
exec "$@"
"""


@pytest.fixture
def fake_host(tmp_path, monkeypatch):
    """Фальшивый «робот»: HOME во временной папке и подменённые systemctl/nmcli."""
    home = tmp_path / "home"
    home.mkdir()
    shims = tmp_path / "bin"
    shims.mkdir()
    for name, body in (("systemctl", FAKE_SYSTEMCTL), ("nmcli", FAKE_NMCLI), ("sudo", FAKE_SUDO)):
        path = shims / name
        path.write_text(body, encoding="utf-8")
        path.chmod(0o755)
    log = tmp_path / "calls.log"
    log.write_text("", encoding="utf-8")

    monkeypatch.setenv("HOME", str(home))
    monkeypatch.setenv("PATH", "%s%s%s" % (shims, os.pathsep, os.environ.get("PATH", "")))
    monkeypatch.setenv("FAKE_LOG", str(log))
    monkeypatch.setenv("RC_VAR_DIR", str(tmp_path / "var"))
    monkeypatch.delenv("DISPLAY", raising=False)

    class Host:
        root = home
        calls = log

    return Host


def run(name: str, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["bash", str(SCRIPTS / name), *args],
        capture_output=True,
        text=True,
        check=False,
    )


def autostart_file(home: Path) -> Path:
    return home / ".config" / "autostart" / "robot-control.desktop"


# --------------------------------------------------------------- файлы скриптов --- #


@pytest.mark.parametrize("name", ["autostart.sh", "make_shortcut.sh", "install_ubuntu.sh", "start_robot.sh",
                                  "sync_firmware_desktop.sh", "start_desktop.sh"])
def test_script_is_in_place_and_runs(name):
    path = SCRIPTS / name
    assert path.is_file(), name
    assert os.access(path, os.X_OK), "%s должен быть исполняемым" % name
    syntax = subprocess.run(["bash", "-n", str(path)], capture_output=True, text=True, check=False)
    assert syntax.returncode == 0, syntax.stderr


def test_start_robot_is_single_supervisor_and_checks_dependencies_once():
    """Единый запуск проверяет зависимости один раз и владеет двумя дочерними процессами."""
    supervisor = (SCRIPTS / "start_robot.sh").read_text(encoding="utf-8")
    assert "./scripts/ensure_deps.sh" in supervisor
    assert 'export RC_SKIP_DEPS=1' in supervisor
    assert "./scripts/start.sh &" in supervisor
    assert "./scripts/kiosk.sh &" in supervisor
    assert "flock -n 9" in supervisor
    assert "wait -n" in supervisor
    for helper in ("start.sh", "kiosk.sh"):
        text = (SCRIPTS / helper).read_text(encoding="utf-8")
        assert '"${RC_SKIP_DEPS:-0}" != "1"' in text


# ----------------------------------------------------------------- автозапуск off --- #


def test_autostart_off_disables_services_session_entry_and_access_point(fake_host):
    """`autostart.sh off` убирает все места, из которых приложение стартовало само."""
    entry = autostart_file(fake_host.root)
    entry.parent.mkdir(parents=True)
    entry.write_text("[Desktop Entry]\nName=Экран робота (киоск)\n", encoding="utf-8")

    result = run("autostart.sh", "off")
    assert result.returncode == 0, result.stderr

    calls = fake_host.calls.read_text(encoding="utf-8")
    assert "disable --now robot-control.service" in calls
    assert "disable --now robot-kiosk.service" in calls
    # Драйвер лидара ROS 2 держит USB-порт: он тоже должен гаснуть вместе с
    # приложениями, иначе следующий запуск остаётся без скана.
    assert "disable --now robot-lidar.service" in calls
    assert "connection.autoconnect no" in calls
    assert not entry.exists(), "ярлык автозапуска сеанса должен быть удалён"
    assert "вручную" in result.stdout


def test_autostart_status_shows_what_starts_itself(fake_host, monkeypatch):
    """`status` отвечает на вопрос «что поднимается само» без запуска служб."""
    monkeypatch.setenv("FAKE_UNIT_ENABLED", "enabled")
    monkeypatch.setenv("FAKE_UNIT_ACTIVE", "active")
    monkeypatch.setenv("FAKE_AP_AUTOCONNECT", "yes")
    entry = autostart_file(fake_host.root)
    entry.parent.mkdir(parents=True)
    entry.write_text("[Desktop Entry]\nName=Экран робота (киоск)\n", encoding="utf-8")

    result = run("autostart.sh", "status")
    assert result.returncode == 0, result.stderr
    out = result.stdout
    # Пульт, бортовой экран и служба лидара ROS 2: последняя попадает в список
    # только если она установлена на этой машине (фальшивый systemctl отвечает
    # «установлена» на любой вызов `cat`).
    assert out.count("автозапуск ВКЛЮЧЁН") == 3
    assert "лидар ROS 2" in out
    assert "ВКЛЮЧЁН" in out and str(entry) in out
    assert "автоподъём ВКЛЮЧЁН" in out
    assert "autostart.sh off" in out


def test_autostart_status_is_quiet_when_nothing_is_enabled(fake_host):
    result = run("autostart.sh", "status")
    assert result.returncode == 0, result.stderr
    assert "само ничего не запускается" in result.stdout
    assert "./scripts/start_robot.sh" in result.stdout
    assert "./scripts/stop.sh" in result.stdout


def test_autostart_rejects_unknown_action(fake_host):
    result = run("autostart.sh", "ол-инклюд")
    assert result.returncode != 0
    assert "неизвестное действие" in result.stderr


def test_autostart_on_enables_services_and_session_entry(fake_host):
    result = run("autostart.sh", "on")
    assert result.returncode == 0, result.stderr
    calls = fake_host.calls.read_text(encoding="utf-8")
    assert "enable --now robot-control.service" in calls
    assert "enable --now robot-kiosk.service" in calls
    assert autostart_file(fake_host.root).is_file(), "ярлык автозапуска сеанса должен появиться"


def spawn_fake_server(var_dir: Path, *, name: str = "run.pid") -> subprocess.Popen:
    """Запускает «сервер» (долгоживущий процесс) и кладёт его pid в var/, как start.sh."""
    process = subprocess.Popen(["sleep", "60"], start_new_session=True)
    var_dir.mkdir(parents=True, exist_ok=True)
    (var_dir / name).write_text("%d\n" % process.pid, encoding="utf-8")
    return process


def alive(process: subprocess.Popen) -> bool:
    return process.poll() is None


def wait_dead(process: subprocess.Popen, timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            return True
        time.sleep(0.05)
    return False


def test_autostart_off_stops_what_is_running(fake_host, tmp_path):
    """`off` без флагов = выключить всё: и автозапуск, и то, что запущено сейчас."""
    var_dir = tmp_path / "var"
    server = spawn_fake_server(var_dir)
    try:
        result = run("autostart.sh", "off")
        assert result.returncode == 0, result.stderr
        assert wait_dead(server), "запущенный сервер должен быть остановлен"
        assert not (var_dir / "run.pid").exists(), "pid-файл должен быть убран"
        assert "останавливаю то, что запущено сейчас" in result.stdout
    finally:
        if alive(server):
            server.kill()
            server.wait(timeout=5)


def test_autostart_off_keep_running_only_disables(fake_host, tmp_path):
    """`off --keep-running` выключает автозапуск, но не глушит запущенное."""
    var_dir = tmp_path / "var"
    server = spawn_fake_server(var_dir)
    try:
        result = run("autostart.sh", "off", "--keep-running")
        assert result.returncode == 0, result.stderr
        assert alive(server), "с --keep-running работающий сервер остаётся"
        assert (var_dir / "run.pid").exists()
        assert "запущенное не трогаю" in result.stdout
        calls = fake_host.calls.read_text(encoding="utf-8")
        assert "disable robot-control.service" in calls
        assert "disable --now robot-control.service" not in calls
    finally:
        server.kill()
        server.wait(timeout=5)


def test_autostart_off_rejects_unknown_flag(fake_host):
    result = run("autostart.sh", "off", "--не-флаг")
    assert result.returncode != 0
    assert "неизвестный аргумент" in result.stderr


def test_stop_sh_stops_servers_from_pid_files(fake_host, tmp_path):
    """stop.sh гасит и пульт, и экран по pid-файлам, и убирает сами файлы."""
    var_dir = tmp_path / "var"
    control = spawn_fake_server(var_dir, name="run.pid")
    kiosk = spawn_fake_server(var_dir, name="kiosk.pid")
    try:
        result = run("stop.sh", "--servers")
        assert result.returncode == 0, result.stderr
        assert wait_dead(control) and wait_dead(kiosk)
        assert not any(var_dir.iterdir()), "pid-файлы должны быть удалены"
        assert "Всё остановлено" in result.stdout
        # --servers не трогает службы: ими занимается autostart.sh off
        assert "systemctl" not in fake_host.calls.read_text(encoding="utf-8")
    finally:
        for process in (control, kiosk):
            if alive(process):
                process.kill()
                process.wait(timeout=5)


def test_stop_sh_stops_screen_watchdog(fake_host, tmp_path):
    """Сторож экрана гасится вместе с серверами: иначе он открывает окно снова.

    Когда сервер 8090 поднимает не ``kiosk.sh`` (служба robot-kiosk или прошлый
    запуск), процесс с окном на мониторе живёт отдельно и pid-файла сервера не
    имеет — stop.sh его не видел, и экран возвращался на остановленном роботе.
    """
    var_dir = tmp_path / "var"
    kiosk = spawn_fake_server(var_dir, name="kiosk.pid")
    watcher = spawn_fake_server(var_dir, name="kiosk-display-8090.pid")
    try:
        result = run("stop.sh", "--servers")
        assert result.returncode == 0, result.stderr
        assert wait_dead(kiosk), "сервер экрана не остановлен"
        assert wait_dead(watcher), "сторож экрана пережил остановку"
        assert not any(var_dir.iterdir()), "pid-файлы должны быть удалены"
        assert "сторож экрана робота" in result.stdout
        assert "Всё остановлено" in result.stdout
    finally:
        for process in (kiosk, watcher):
            if alive(process):
                process.kill()
                process.wait(timeout=5)


def test_stop_sh_reports_nothing_running(fake_host):
    result = run("stop.sh")
    assert result.returncode == 0, result.stderr
    assert "Запущенного приложения не найдено" in result.stdout
    assert "autostart.sh" in result.stdout


def test_stop_sh_help_explains_autostart_separation(fake_host):
    result = run("stop.sh", "--help")
    assert result.returncode == 0, result.stderr
    assert "autostart.sh off" in result.stdout


# ------------------------------------------------------------- ярлыки на рабочем столе --- #


def test_make_shortcut_creates_one_desktop_launcher_without_autostart(fake_host):
    """Создаётся один общий ярлык, а не отдельные значки пульта и киоска."""
    result = run("make_shortcut.sh")
    assert result.returncode == 0, result.stderr
    desktop = fake_host.root / "Desktop"
    entries = sorted(desktop.glob("*.desktop"))
    assert len(entries) == 1
    assert entries[0].name == "robot-control.desktop"
    content = entries[0].read_text(encoding="utf-8")
    assert 'Name=Запустить робот' in content
    assert 'start_desktop.sh' in content
    assert not (fake_host.root / ".local" / "share" / "applications" / "robot-console.desktop").exists()
    assert not autostart_file(fake_host.root).exists()
    assert "Автозапуск выключен" in result.stdout


def test_make_shortcut_removes_old_autostart_entry(fake_host):
    """Обновление кода не должно оставлять на роботе старый автозапуск."""
    entry = autostart_file(fake_host.root)
    entry.parent.mkdir(parents=True)
    entry.write_text("[Desktop Entry]\nName=Экран робота (киоск)\n", encoding="utf-8")

    result = run("make_shortcut.sh")
    assert result.returncode == 0, result.stderr
    assert not entry.exists()
    assert "Автозапуск выключен" in result.stdout


def test_make_shortcut_autostart_only_on_request(fake_host):
    result = run("make_shortcut.sh", "--autostart")
    assert result.returncode == 0, result.stderr
    content = autostart_file(fake_host.root).read_text(encoding="utf-8")
    assert "X-GNOME-Autostart-enabled=true" in content
    assert "Terminal=false" in content

    result = run("make_shortcut.sh", "--no-autostart")
    assert result.returncode == 0, result.stderr
    assert not autostart_file(fake_host.root).exists()


# ------------------------------------------------- прошивки на рабочем столе --- #

FIRMWARE = Path(__file__).resolve().parent.parent / "firmware"


def desktop_firmware(home: Path) -> tuple[Path, Path]:
    desktop = home / "Desktop"
    return desktop / "motor_master.ino", desktop / "motor_node.ino"


def test_sync_firmware_desktop_copies_master_and_node(fake_host):
    """На рабочем столе появляются обе прошивки — байт в байт как в репозитории."""
    result = run("sync_firmware_desktop.sh")
    assert result.returncode == 0, result.stderr
    master, node = desktop_firmware(fake_host.root)
    assert master.read_bytes() == (FIRMWARE / "motor_master" / "motor_master.ino").read_bytes()
    assert node.read_bytes() == (FIRMWARE / "motor_node" / "motor_node.ino").read_bytes()
    assert "мастер" in result.stdout and "нода" in result.stdout


def test_sync_firmware_desktop_refreshes_after_update_and_keeps_fresh(fake_host):
    """Новая версия прошивки заменяет копию; актуальная копия не переписывается."""
    assert run("sync_firmware_desktop.sh").returncode == 0
    master, node = desktop_firmware(fake_host.root)
    master_mtime = master.stat().st_mtime_ns

    # «Обновилась версия»: копия ноды на рабочем столе теперь старая.
    node.write_text("// старая версия прошивки\n", encoding="utf-8")
    result = run("sync_firmware_desktop.sh")
    assert result.returncode == 0, result.stderr
    assert node.read_bytes() == (FIRMWARE / "motor_node" / "motor_node.ino").read_bytes()
    # Мастер не менялся — его файл не трогаем (даже время изменения то же).
    assert master.stat().st_mtime_ns == master_mtime


def test_desktop_launch_syncs_firmware(fake_host):
    """Запуск через ярлык и создание ярлыка обновляют прошивки на рабочем столе."""
    launcher = (SCRIPTS / "start_desktop.sh").read_text(encoding="utf-8")
    assert "sync_firmware_desktop.sh" in launcher
    assert launcher.index("sync_firmware_desktop.sh") < launcher.index("start_robot.sh")
    shortcut = (SCRIPTS / "make_shortcut.sh").read_text(encoding="utf-8")
    assert "sync_firmware_desktop.sh" in shortcut

    result = run("make_shortcut.sh")
    assert result.returncode == 0, result.stderr
    master, node = desktop_firmware(fake_host.root)
    assert master.is_file() and node.is_file(), "прошивки должны лежать рядом с ярлыком"


# ---------------------------------------------------------------- установка --- #


def test_install_does_not_enable_autostart_without_flag():
    """Автозапуск ставится только с флагом --autostart, а не «за компанию»."""
    text = (SCRIPTS / "install_ubuntu.sh").read_text(encoding="utf-8")
    assert "--autostart" in text
    assert "if [[ $AUTOSTART -eq 1 ]]; then" in text
    # без флага приложение поднимается в текущем окне терминала
    assert "if [[ $RUN -eq 1 ]]; then" in text
    assert "exec ./scripts/start_robot.sh" in text
    # точка доступа поднимается без автоподъёма при загрузке
    assert "./scripts/setup_ap.sh --autostart" in text


def test_install_help_explains_manual_start():
    result = run("install_ubuntu.sh", "--help")
    assert result.returncode == 0, result.stderr
    assert "Автозапуск при загрузке по умолчанию выключен" in result.stdout
    assert "--autostart" in result.stdout


def test_install_rejects_unknown_flag():
    result = run("install_ubuntu.sh", "--автозапуск")
    assert result.returncode != 0
    assert "неизвестный аргумент" in result.stderr


if __name__ == "__main__":  # pragma: no cover — ручной запуск
    sys.exit(pytest.main([__file__, "-q"]))
