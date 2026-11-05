"""Общие фикстуры: приложение и хранилище операторов во временной папке."""

from __future__ import annotations

import re
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_control import eventlog, slam  # noqa: E402
from robot_control.app import create_app  # noqa: E402
from robot_control.auth import OperatorStore  # noqa: E402
from robot_control.config import Config  # noqa: E402

OPERATOR = "operator"
PASSWORD = "correct-horse-battery"
FAST_ITERATIONS = 1_000  # тесты не должны тормозить на PBKDF2


@pytest.fixture(scope="session", autouse=True)
def idle_lidar_singleton_for_tests():
    """App/API tests must not leave a real USB scanner running for the suite.

    Tests of the actual worker create/start their own managers explicitly. A
    background singleton from an unrelated HTTP test can otherwise steal a
    later PTY's lock when /dev/pts/N is reused.
    """
    from robot_control.lidar import LidarManager
    previous = LidarManager._instance
    instance = LidarManager()
    LidarManager._instance = instance
    try:
        yield
    finally:
        current = LidarManager._instance
        if current is not None and current is not instance and current is not previous:
            current.stop()
        instance.stop()
        LidarManager._instance = previous


@pytest.fixture(autouse=True)
def isolated_estop_latch(tmp_path, monkeypatch):
    """Защёлка E-STOP — файл; тесты не должны трогать реальный data/estop.json."""
    monkeypatch.setenv("RC_ESTOP_FILE", str(tmp_path / "estop.json"))
    yield


@pytest.fixture(autouse=True)
def isolated_device_registry(tmp_path, monkeypatch):
    """Discovery hints in tests must never modify the robot's local registry."""
    monkeypatch.setenv("RC_DEVICE_REGISTRY_FILE", str(tmp_path / "device_registry.json"))
    yield


@pytest.fixture(autouse=True)
def isolated_device_ports_file(tmp_path, monkeypatch):
    """Ручные назначения портов в тестах — во временный файл, не в рабочий .env."""
    monkeypatch.setenv("RC_DEVICE_PORTS_FILE", str(tmp_path / "device-ports.env"))
    monkeypatch.setenv("RC_CAMERA_ENV_FILE", str(tmp_path / "camera-ports.env"))
    yield


@pytest.fixture(autouse=True)
def isolated_printer_config(tmp_path, monkeypatch):
    """Printer tests and inventory must never inspect or overwrite robot settings."""
    monkeypatch.setenv("RC_PRINTER_CONFIG", str(tmp_path / "printer.json"))
    yield


@pytest.fixture(autouse=True)
def isolated_maneuver_config(tmp_path, monkeypatch):
    """Пресеты манёвров («метр вперёд», «90°») — файл во временной папке.

    Иначе тест, где манёвр доехал до цели, записал бы измеренную скорость в
    рабочий data/maneuvers.json и поменял поведение следующего запуска.
    """
    monkeypatch.setenv("RC_MANEUVER_CONFIG", str(tmp_path / "maneuvers.json"))
    yield


@pytest.fixture(autouse=True)
def isolated_map_files(tmp_path, monkeypatch):
    """Файлы карты — во временную папку: тесты не трогают рабочий стол."""
    monkeypatch.setenv("RC_MAP_DIR", str(tmp_path / "map"))
    monkeypatch.setenv("RC_MAP_SETTINGS", str(tmp_path / "map_settings.json"))
    yield


@pytest.fixture(autouse=True)
def clean_event_journal():
    """Журнал событий бортового экрана — состояние процесса, а не теста.

    Без очистки записи предыдущих тестов («PIN не принят», «скана нет»)
    переезжали бы в проверки следующего; поэтому каждый тест начинается с
    пустого журнала и заканчивается тоже без него.
    """
    eventlog.reset()
    slam.reset_journal_notes()
    yield
    eventlog.reset()
    slam.reset_journal_notes()


@pytest.fixture
def config(tmp_path: Path) -> Config:
    return Config(
        secret_key="test-secret-key",
        operators_file=tmp_path / "operators.json",
        bind_host="127.0.0.1",
        port=8080,
        max_failed_attempts=3,
        lockout_seconds=300,
        pbkdf2_iterations=FAST_ITERATIONS,
        testing=True,
        # интерфейс робота без железа: значения пустые (демо-режима нет), файл замка — во временной папке
        slam_source="none",
        slam_lock_file=tmp_path / "lock.json",
        stats_file=tmp_path / "stats.json",
    )


@pytest.fixture
def store(config: Config) -> OperatorStore:
    return OperatorStore(config.operators_file)


@pytest.fixture
def operator(store: OperatorStore, config: Config):
    return store.add_or_update(OPERATOR, PASSWORD, iterations=config.pbkdf2_iterations)


@pytest.fixture
def kiosk_client(config: Config):
    """Клиент бортового экрана (8090) — то же, что поднимает run_kiosk.py."""
    from robot_control.kiosk_app import create_kiosk_app

    application = create_kiosk_app(config)
    application.config.update(TESTING=True)
    return application.test_client()


@pytest.fixture
def app(config: Config):
    application = create_app(config)
    application.config.update(TESTING=True)
    return application


@pytest.fixture
def client(app):
    return app.test_client()


def csrf_token(response) -> str:
    """Достаём CSRF-токен из отрендеренной формы."""
    match = re.search(r'name="csrf_token" value="([^"]+)"', response.get_data(as_text=True))
    assert match, "в ответе нет CSRF-токена"
    return match.group(1)


def login(client, username=OPERATOR, password=PASSWORD):
    page = client.get("/login")
    return client.post(
        "/login",
        data={
            "username": username,
            "password": password,
            "csrf_token": csrf_token(page),
        },
        follow_redirects=False,
    )
