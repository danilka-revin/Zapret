"""Сторож драйвера лидара ROS 2: драйвер поднимается сам, без терминала.

Проверяем решение, из-за которого оператору приходилось вводить команды руками:
приложение, запущенное не ярлыком «Запустить робот» (автозапуск, systemd,
второй экран, киоск), не запускало драйвер вовсе — `/scan` никто не публиковал.
Теперь приложение само решает, что делать: ждать, запускать супервизор или
поставить ROS 2 (окно пароля вместо терминала — `scripts/askpass_gui.sh`).
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from robot_control import lidar_boot


class FakeRos:
    def __init__(self, fresh: bool = False) -> None:
        self._fresh = fresh

    def fresh(self, timeout: float = 2.0) -> bool:
        return self._fresh


class FakeManager:
    def __init__(self, *, source_mode: str = "ros", protocol: str = "lds01",
                 fresh: bool = False) -> None:
        self.source_mode = source_mode
        self.protocol = protocol
        self.ros = FakeRos(fresh)


@pytest.fixture(autouse=True)
def _clean_state(monkeypatch):
    lidar_boot.reset_state()
    for key in ("RC_LIDAR_AUTOSTART", "RC_ROS_AUTO"):
        monkeypatch.delenv(key, raising=False)
    monkeypatch.delenv("PYTEST_CURRENT_TEST", raising=False)
    monkeypatch.setattr(lidar_boot, "service_active", lambda *a, **kw: False)
    yield
    lidar_boot.reset_state()


def test_autostart_switches(monkeypatch):
    assert lidar_boot.enabled() is True
    assert lidar_boot.install_allowed() is True
    monkeypatch.setenv("RC_LIDAR_AUTOSTART", "0")
    monkeypatch.setenv("RC_ROS_AUTO", "off")
    assert lidar_boot.enabled() is False
    assert lidar_boot.install_allowed() is False
    assert lidar_boot.step(FakeManager()) == "off"


def test_serial_source_never_touches_ros():
    """RC_LIDAR_SOURCE=serial — отладочный путь, драйвер не поднимаем."""
    assert lidar_boot.step(FakeManager(source_mode="serial")) == "serial"


def test_live_scan_means_nothing_to_do():
    assert lidar_boot.step(FakeManager(fresh=True)) == "fresh"
    assert lidar_boot.status()["state"] == "fresh"


def test_existing_supervisor_is_not_duplicated(monkeypatch):
    monkeypatch.setattr(lidar_boot, "supervisor_running", lambda root=None: True)
    monkeypatch.setattr(lidar_boot, "start_supervisor",
                        lambda root=None: pytest.fail("супервизор уже работает"))
    assert lidar_boot.step(FakeManager()) == "supervisor"


def test_ready_ros_starts_the_driver(monkeypatch):
    spawned = []
    monkeypatch.setattr(lidar_boot, "supervisor_running", lambda root=None: False)
    monkeypatch.setattr(lidar_boot, "readiness", lambda protocol="auto": (True, ""))
    monkeypatch.setattr(lidar_boot, "start_supervisor",
                        lambda root=None: spawned.append(root) or 4242)
    assert lidar_boot.step(FakeManager()) == "spawn"
    assert spawned == [None]
    assert lidar_boot.status()["spawned_pid"] == 4242
    assert lidar_boot.status()["note"]


def test_spawn_is_throttled(monkeypatch):
    """Не запускать супервизор каждые 20 секунд, если он сразу падает."""
    calls = []
    monkeypatch.setattr(lidar_boot, "supervisor_running", lambda root=None: False)
    monkeypatch.setattr(lidar_boot, "readiness", lambda protocol="auto": (True, ""))
    monkeypatch.setattr(lidar_boot, "start_supervisor", lambda root=None: calls.append(1) or 1)
    assert lidar_boot.step(FakeManager()) == "spawn"
    assert lidar_boot.step(FakeManager()) == "wait spawn"
    assert len(calls) == 1


def test_missing_ros_runs_installation(monkeypatch):
    installed = []
    monkeypatch.setattr(lidar_boot, "supervisor_running", lambda root=None: False)
    monkeypatch.setattr(lidar_boot, "readiness",
                        lambda protocol="auto": (False, "ROS 2 не установлен"))
    monkeypatch.setattr(lidar_boot, "start_setup", lambda root=None: installed.append(1) or 7)
    assert lidar_boot.step(FakeManager()) == "install"
    assert installed == [1]
    assert "ROS 2" in lidar_boot.status()["note"]
    # Повторно — не раньше INSTALL_EVERY_S.
    assert lidar_boot.step(FakeManager()) == "wait install"
    assert installed == [1]


def test_installation_is_not_started_when_autosetup_is_off(monkeypatch):
    monkeypatch.setenv("RC_ROS_AUTO", "0")
    monkeypatch.setattr(lidar_boot, "supervisor_running", lambda root=None: False)
    monkeypatch.setattr(lidar_boot, "readiness", lambda protocol="auto": (False, "нет драйверов"))
    monkeypatch.setattr(lidar_boot, "start_setup",
                        lambda root=None: pytest.fail("RC_ROS_AUTO=0 — ставить нельзя"))
    assert lidar_boot.step(FakeManager()) == "wait install off"
    assert "нет драйверов" in lidar_boot.status()["note"]


def test_read_logs_returns_bounded_tails_from_fixed_paths(tmp_path):
    log_path = tmp_path / lidar_boot.LIDAR_LOG
    log_path.parent.mkdir(parents=True)
    log_path.write_text("".join("line %d\n" % index for index in range(15)),
                         encoding="utf-8")

    result = lidar_boot.read_logs(5, root=tmp_path)
    lidar = next(item for item in result["files"] if item["path"] == "var/lidar.log")
    assert result["lines"] == 10  # API never returns an unexpectedly tiny or huge tail.
    assert lidar["exists"] is True
    assert lidar["lines"] == ["line %d" % index for index in range(5, 15)]
    assert all(not item["exists"] for item in result["files"] if item is not lidar)


def test_readiness_reports_why_ros_is_not_ready(monkeypatch):
    monkeypatch.setattr(lidar_boot.ros_env, "prefix", lambda: None)
    assert lidar_boot.readiness("lds01") == (False, "ROS 2 не установлен")

    monkeypatch.setattr(lidar_boot.ros_env, "prefix", lambda: Path("/opt/ros/humble"))
    monkeypatch.setattr(lidar_boot.ros_env, "installed_packages", lambda root=None: set())
    ready, why = lidar_boot.readiness("lds01")
    assert ready is False and "пакеты" in why

    monkeypatch.setattr(lidar_boot.ros_env, "installed_packages",
                        lambda root=None: {"rplidar_ros"})
    ready, why = lidar_boot.readiness("lds01")
    assert ready is False and "hls_lfcd_lds_driver" in why

    monkeypatch.setattr(lidar_boot.ros_env, "installed_packages",
                        lambda root=None: {"hls_lfcd_lds_driver", "zapret_lidar"})
    assert lidar_boot.readiness("lds01") == (True, "")


def test_supervisor_pid_ignores_foreign_processes(tmp_path):
    """Pid в файле мог достаться чужому процессу — верить только своей команде."""
    pid_file = tmp_path / lidar_boot.PID_FILE
    pid_file.parent.mkdir(parents=True, exist_ok=True)

    pid_file.write_text("", encoding="utf-8")
    assert lidar_boot.supervisor_pid(tmp_path) is None

    pid_file.write_text(str(os.getpid()), encoding="utf-8")
    # Наш собственный процесс — python, а не ros2_lidar.sh.
    assert lidar_boot.supervisor_pid(tmp_path) is None

    pid_file.write_text("999999", encoding="utf-8")
    assert lidar_boot.supervisor_pid(tmp_path) is None


def test_start_is_off_in_tests():
    assert lidar_boot.start(FakeManager(), testing=True) is None
    assert lidar_boot.status()["state"] == "idle"


def test_api_exposes_the_watchdog_state():
    """`GET /api/lidar` → `ros.boot`: видно, что приложение само поднимает драйвер."""
    from robot_control.lidar import LidarManager

    manager = LidarManager()
    try:
        boot = manager.get_status()["ros"]["boot"]
        assert "state" in boot and "enabled" in boot and "install_allowed" in boot
    finally:
        manager.stop()


def test_start_lidar_hooks_the_watchdog(monkeypatch):
    """Сторож поднимается при старте приложения (оба экрана), а не только ярлыком."""
    from robot_control import slam

    started = []
    monkeypatch.setattr(slam.lidar_boot, "start",
                        lambda manager=None, **kwargs: started.append(manager))
    monkeypatch.setenv("RC_LIDAR_AUTOSTART", "1")
    # pytest выставляет PYTEST_CURRENT_TEST на время теста, а start_lidar по нему
    # намеренно молчит: здесь проверяем именно рабочую ветку.
    monkeypatch.delenv("PYTEST_CURRENT_TEST", raising=False)
    manager = slam.start_lidar(testing=False, port=1)
    try:
        assert started and started[0] is manager
    finally:
        manager.stop()


def test_watchdog_never_starts_inside_a_test_run(monkeypatch):
    """Тесты не должны случайно поставить ROS 2: сторож видит сам pytest."""
    monkeypatch.delenv("PYTEST_CURRENT_TEST", raising=False)
    assert lidar_boot.in_tests() is True
    assert lidar_boot.start(FakeManager()) is None
