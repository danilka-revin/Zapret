"""Лидар: в режиме ``auto`` молчащий ROS 2 не должен отменять USB-путь.

Это ровно та поломка, из-за которой лидар «опять не показывался» на обоих
экранах: ROS 2 ставится сам (ярлык «Запустить робот»), режим ``auto`` начинал
доверять ему лидар целиком, а если драйвер не поднялся — ``/scan`` пуст, USB
не перебирается, и на пульте с бортом навсегда «скана лидара нет».

Здесь проверяется:

* окно доверия ``RC_LIDAR_ROS_GRACE_S``: пока оно идёт, порт не трогаем
  (драйвер может подниматься несколько секунд);
* после окна USB-перебор включается и находит лидар, даже если ROS 2
  установлен;
* живой скан из ``/scan`` снова делает ROS 2 единственным хозяином: с USB-порта
  снимается попытка, как только пошли сообщения;
* ``RC_LIDAR_SOURCE=ros`` (явный выбор) и ``serial`` ведут себя как раньше;
* причина и подсказка на экране объясняют переход на USB.
"""

from __future__ import annotations

import threading

import pytest

from robot_control import lidar
from robot_control.lidar import LidarManager

from test_lidar_serial import FakeLds01, FakeSerial, fake_port, fake_serial, wait_for  # noqa: F401


class FakeRos:
    """Заглушка подписки ROS 2: доступна, но скан может не идти."""

    def __init__(self, *, available: bool = True, fresh: bool = False,
                 quiet_s: float = 0.0, publishers: int = 0) -> None:
        self.available = available
        self.topic = "/scan"
        self.publishers = publishers
        self._fresh = fresh
        self.scan_at = 0.0 if quiet_s == float("inf") else __import__("time").time() - quiet_s
        self.rescan_requested_at = 0.0

    def fresh(self) -> bool:
        return self._fresh

    def summary(self) -> str:
        return "ROS 2 есть, но /scan никто не публикует — драйвер лидара не запущен"

    def hint(self) -> str:
        return "запусти драйвер: ./scripts/ros2_lidar.sh"

    def status(self) -> dict:
        return {"available": self.available, "topic": self.topic, "connected": self._fresh,
                "reason": self.summary(), "hint": self.hint(), "publishers": 0}


@pytest.fixture(autouse=True)
def fast_lidar(monkeypatch):
    """Короткие таймауты: тесты не должны ждать реальные секунды перебора."""
    monkeypatch.setenv("RC_LIDAR_PEER_URLS", "")
    monkeypatch.setattr(lidar, "LIDAR_PROBE_TIMEOUT_S", 0.05)
    monkeypatch.setattr(lidar, "LIDAR_DATA_TIMEOUT_S", 0.05)
    monkeypatch.setattr(lidar, "LIDAR_RESCAN_S", 0.05)
    monkeypatch.setattr(lidar, "LIDAR_ATTEMPT_MAX_S", 0.3)
    yield


@pytest.fixture(autouse=True)
def data_dir(monkeypatch, tmp_path):
    """Каталог состояния лидара (замки портов, ``lidar-state-<pid>.json``) — свой."""
    path = tmp_path / "data"
    path.mkdir(exist_ok=True)
    monkeypatch.setattr(lidar, "DATA_DIR", path)
    return path


@pytest.fixture
def manager():
    """Менеджер без фонового потока: состояние ROS 2 задаётся в каждом тесте."""
    previous = LidarManager._instance
    instance = LidarManager()
    LidarManager._instance = instance
    try:
        yield instance
    finally:
        instance.stop()
        LidarManager._instance = previous


@pytest.fixture
def make_manager():
    """Создаёт менеджер **после** того, как тест задал ``RC_LIDAR_*``.

    Менеджер читает порт и настройки в конструкторе: если поднять его раньше
    подмены окружения, он останется без порта и будет искать лидар по всем USB.
    """
    created: list[LidarManager] = []
    previous = LidarManager._instance

    def make() -> LidarManager:
        instance = LidarManager()
        LidarManager._instance = instance
        created.append(instance)
        return instance

    try:
        yield make
    finally:
        for instance in created:
            instance.stop()
        LidarManager._instance = previous


# ---------------------------------------------------------------------------
# Окно доверия ROS 2
# ---------------------------------------------------------------------------


def test_ros_still_owns_the_lidar_inside_the_grace_window(manager):
    """Пока идёт окно, ROS 2 — хозяин: драйвер поднимается не мгновенно."""
    manager.source_mode = "auto"
    manager.ros = FakeRos(available=True, fresh=False, quiet_s=1.0)
    assert manager.ros_holds_lidar() is True
    assert manager.ros_fallback_note() == ""


def test_usb_takes_over_when_the_driver_never_comes_up(manager, monkeypatch):
    """ROS 2 установлен, но молчит дольше окна — читаем USB сами."""
    monkeypatch.setenv("RC_LIDAR_ROS_GRACE_S", "20")
    manager.source_mode = "auto"
    manager.ros = FakeRos(available=True, fresh=False, quiet_s=120.0)
    assert manager.ros_holds_lidar() is False
    note = manager.ros_fallback_note()
    assert "/scan" in note and "USB" in note
    # Причина на экране получает это объяснение, а не только «порт не найден».
    manager._set_status("порт лидара не найден", hint="подключи лидар")
    assert "USB" in manager.hint and "/scan" in manager.hint


def test_grace_window_is_configurable(manager, monkeypatch):
    """``RC_LIDAR_ROS_GRACE_S=0`` — не ждать драйвер вовсе."""
    manager.source_mode = "auto"
    manager.ros = FakeRos(available=True, fresh=False, quiet_s=0.1)
    assert manager.ros_holds_lidar() is True
    monkeypatch.setenv("RC_LIDAR_ROS_GRACE_S", "0")
    assert manager.ros_holds_lidar() is False


def test_fresh_ros_scan_wins_again(manager, monkeypatch):
    """Драйвер ожил — USB больше не трогаем, хозяин снова ROS 2."""
    monkeypatch.setenv("RC_LIDAR_ROS_GRACE_S", "0")
    manager.source_mode = "auto"
    manager.ros = FakeRos(available=True, fresh=False, quiet_s=60.0)
    assert manager.ros_holds_lidar() is False
    manager.ros._fresh = True
    assert manager.ros_holds_lidar() is True
    assert manager.ros_fallback_note() == ""


def test_rescan_request_restarts_the_wait(manager, monkeypatch):
    """Кнопка «Пересканировать» просит супервизор поднять драйвер — снова ждём."""
    import time as _time

    monkeypatch.setenv("RC_LIDAR_ROS_GRACE_S", "20")
    manager.source_mode = "auto"
    manager.ros = FakeRos(available=True, fresh=False, quiet_s=300.0)
    manager.ros.rescan_requested_at = _time.time()
    assert manager.ros_holds_lidar() is True


def test_live_driver_keeps_the_port_even_without_points(manager, monkeypatch):
    """Топик публикуется, а точек нет: порт у драйвера не отбираем.

    Два читателя одного USB-адаптера ломают скан обоим, поэтому «/scan
    публикуется, но пустой» — это не повод перейти на USB: на экране честно
    написано, что драйвер топик публикует, а точек нет (см. ``ros.summary``).
    """
    monkeypatch.setenv("RC_LIDAR_ROS_GRACE_S", "0")
    manager.source_mode = "auto"
    manager.ros = FakeRos(available=True, fresh=False, quiet_s=600.0, publishers=1)
    assert manager.ros_holds_lidar() is True
    assert manager.ros_fallback_note() == ""      # объяснять переход нечего: не переходим


def test_explicit_sources_keep_their_meaning(manager, monkeypatch):
    """``ros`` — только ROS 2, ``serial`` — только USB: выбор оператора не меняем."""
    monkeypatch.setenv("RC_LIDAR_ROS_GRACE_S", "0")
    manager.source_mode = "ros"
    manager.ros = FakeRos(available=True, fresh=False, quiet_s=999.0)
    assert manager.ros_holds_lidar() is True
    assert manager.ros_fallback_note() == ""

    manager.source_mode = "serial"
    manager.ros = FakeRos(available=True, fresh=True)
    assert manager.ros_holds_lidar() is False
    assert manager.ros_fallback_note() == ""


def test_status_block_shows_why_usb_is_used(manager, monkeypatch):
    """В ``GET /api/lidar`` видно и «ROS выбран», и «ROS владеет», и переход."""
    monkeypatch.setenv("RC_LIDAR_ROS_GRACE_S", "0")
    manager.source_mode = "auto"
    manager.ros = FakeRos(available=True, fresh=False, quiet_s=90.0)
    ros = manager.get_status()["ros"]
    assert ros["owns_lidar"] is True
    assert ros["holds_lidar"] is False
    assert "USB" in ros["usb_fallback"]
    assert ros["grace_s"] == 0.0


# ---------------------------------------------------------------------------
# Что происходит на самом деле: скан приходит по USB, пока ROS 2 молчит
# ---------------------------------------------------------------------------


def test_lidar_is_read_over_usb_while_ros_is_silent(make_manager, fake_serial, fake_port, monkeypatch):
    """Главная проверка: ``auto`` + установленный ROS 2 без скана → лидар виден."""
    monkeypatch.setenv("RC_LIDAR_ROS_GRACE_S", "0")
    monkeypatch.setenv("RC_LIDAR_PORT", fake_port)
    monkeypatch.setenv("RC_LIDAR_AUTOPROBE", "0")
    device = FakeLds01()
    FakeSerial.devices[fake_port] = device
    manager = make_manager()
    manager.ros = FakeRos(available=True, fresh=False, quiet_s=600.0)
    manager.source_mode = "auto"

    worker = threading.Thread(target=manager._serial_worker, daemon=True)
    worker.start()
    try:
        assert wait_for(lambda: bool(manager.get_scan())), "лидар не найден по USB"
        status = manager.get_status()
        assert status["connected"] is True
        assert status["detected"]["protocol"] == "lds01"
        assert status["ros"]["usb_fallback"]
        assert device.started is True, "лидару не отдали команду запуска"
    finally:
        manager.stop()
        worker.join(timeout=2.0)


def test_usb_port_is_not_touched_while_the_driver_is_starting(make_manager, fake_serial, fake_port, monkeypatch):
    """Окно доверия: драйвер ещё поднимается — порт не трогаем, но говорим об этом.

    Это вторая половина того же правила: без окна приложение бросалось бы на
    USB сразу (и мешало драйверу), а без ограничения по времени — ждало бы
    драйвер вечно, как это и было.
    """
    monkeypatch.setenv("RC_LIDAR_ROS_GRACE_S", "20")
    monkeypatch.setenv("RC_LIDAR_PORT", fake_port)
    monkeypatch.setenv("RC_LIDAR_AUTOPROBE", "0")
    manager = make_manager()
    manager.source_mode = "auto"
    manager.ros = FakeRos(available=True, fresh=False, quiet_s=1.0)   # драйвер поднимается
    monkeypatch.setattr(lidar, "open_serial",
                        lambda *_a, **_k: pytest.fail("полез на USB-порт, пока драйвер поднимается"))
    worker = threading.Thread(target=manager._serial_worker, daemon=True)
    worker.start()
    try:
        assert wait_for(lambda: "ROS 2" in manager.reason, timeout=3.0), manager.reason
        assert "жду скан из ROS 2" in manager.hint, manager.hint
    finally:
        manager.stop()
        worker.join(timeout=2.0)


def test_usb_port_is_not_touched_while_ros_gives_a_scan(make_manager, fake_serial, fake_port, monkeypatch):
    """ROS 2 отдаёт скан: порт не открываем — его держит драйвер."""
    monkeypatch.setenv("RC_LIDAR_ROS_GRACE_S", "0")
    monkeypatch.setenv("RC_LIDAR_PORT", fake_port)
    manager = make_manager()
    manager.ros = FakeRos(available=True, fresh=True, quiet_s=0.0)
    manager.source_mode = "auto"
    monkeypatch.setattr(lidar, "open_serial",
                        lambda *_a, **_k: pytest.fail("полез на USB-порт при живом ROS 2"))
    worker = threading.Thread(target=manager._serial_worker, daemon=True)
    worker.start()
    try:
        assert wait_for(lambda: "ROS 2" in manager.reason)
        assert manager.get_status()["ros"]["holds_lidar"] is True
    finally:
        manager.stop()
        worker.join(timeout=2.0)


def test_running_attempt_stops_when_ros_scan_arrives(make_manager, fake_serial, fake_port, monkeypatch):
    """Драйвер ожил во время чтения порта — USB отпускаем, хозяин снова ROS 2.

    Иначе два читателя одного USB: драйвер не получит порт, а приложение будет
    показывать свой скан вместо ``/scan``.
    """
    monkeypatch.setenv("RC_LIDAR_ROS_GRACE_S", "0")
    monkeypatch.setenv("RC_LIDAR_PORT", fake_port)
    monkeypatch.setenv("RC_LIDAR_AUTOPROBE", "0")
    FakeSerial.devices[fake_port] = FakeLds01()      # поток не кончается
    manager = make_manager()
    manager.source_mode = "auto"
    manager.ros = FakeRos(available=True, fresh=False, quiet_s=600.0)

    worker = threading.Thread(target=manager._serial_worker, daemon=True)
    worker.start()
    try:
        assert wait_for(lambda: bool(manager.get_scan())), "лидар не читается по USB"
        manager.ros._fresh = True                    # драйвер начал публиковать /scan
        assert wait_for(lambda: "ROS 2" in manager.reason, timeout=3.0), manager.reason
        assert manager.ros_holds_lidar() is True
        # Порт отпущен, свой скан больше не обновляется и честно устаревает.
        assert wait_for(lambda: manager.get_scan() is None, timeout=3.0)
        status = manager.get_status()
        assert status["connected"] is False
        assert "не отвечает" not in manager.reason
    finally:
        manager.stop()
        worker.join(timeout=2.0)
