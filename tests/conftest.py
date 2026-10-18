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


@pytest.fixture(autouse=True)
def isolated_estop_latch(tmp_path, monkeypatch):
    """Защёлка E-STOP — файл; тесты не должны трогать реальный data/estop.json."""
    monkeypatch.setenv("RC_ESTOP_FILE", str(tmp_path / "estop.json"))
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
