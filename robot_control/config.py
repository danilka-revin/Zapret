"""Настройки приложения: переменные окружения + файл .env."""

from __future__ import annotations

import os
from collections.abc import Iterator
from dataclasses import dataclass, field
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def dotenv_items(path: Path) -> Iterator[tuple[str, str]]:
    """Строки KEY=VALUE без исполнения файла или изменения окружения."""
    if not path.is_file():
        return
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key:
            yield key, value


def _load_dotenv(path: Path) -> None:
    """Заданные переменные окружения имеют приоритет над файлом."""
    for key, value in dotenv_items(path):
        os.environ.setdefault(key, value)


def _env_bool(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


@dataclass(frozen=True)
class Config:
    """Все параметры, которые нужны приложению."""

    secret_key: str = ""
    operators_file: Path = PROJECT_ROOT / "data" / "operators.json"
    bind_host: str = "0.0.0.0"
    port: int = 8080
    cookie_secure: bool = False
    cookie_samesite: str = "Lax"
    max_failed_attempts: int = 5
    lockout_seconds: int = 300
    session_lifetime_minutes: int = 60
    pbkdf2_iterations: int = 200_000
    testing: bool = False
    # --- интерфейс BULKA_v1 ---
    slam_source: str = "master"        # master | auto | sim | serial | ros
    slam_ports: tuple[str, ...] = ()   # UART-порты для serial
    #: Ручные порты колёсных модулей (RC_WHEEL_PORTS). Роли платы сообщают сами.
    wheel_ports: tuple[str, ...] = ()
    slam_lock_file: Path = PROJECT_ROOT / "data" / "lock.json"
    slam_pin: str = ""                 # пусто → заводской PIN из backend.py
    slam_max_attempts: int = 5
    slam_lock_seconds: int = 30
    # --- выбор режима бортового экрана по PIN на стартовом экране ---
    #: PIN, открывающий обычный бортовой киоск (все функции, настройка, руление).
    kiosk_pin_main: str = "7232"
    #: PIN, открывающий гостевой «бортовой монитор»: общая информация и кейс.
    kiosk_pin_monitor: str = "5656"
    #: Сколько неверных вводов стартового PIN допускается и пауза после них, с.
    kiosk_mode_max_attempts: int = 5
    kiosk_mode_lock_seconds: int = 30
    # --- камеры: раздельные V4L2-порты для передней и задней камер ---
    camera_front: str = ""             # RC_CAMERA_FRONT или RC_CAMERA_FRONT_PORT
    camera_rear: str = ""              # RC_CAMERA_REAR  или RC_CAMERA_REAR_PORT (пусто → не назначена)
    # --- сводка работы робота (рейсы, энергия, замок) ---
    stats_file: Path = PROJECT_ROOT / "data" / "stats.json"
    _loaded_from: str = field(default="", repr=False)

    @classmethod
    def from_env(cls, project_root: Path = PROJECT_ROOT) -> "Config":
        from .camera_settings import load_camera_ports

        _load_dotenv(project_root / ".env")
        load_camera_ports(project_root)
        operators_file = os.environ.get("RC_OPERATORS_FILE") or "data/operators.json"
        path = Path(operators_file)
        if not path.is_absolute():
            path = project_root / path
        return cls(
            secret_key=os.environ.get("RC_SECRET_KEY", ""),
            operators_file=path,
            bind_host=os.environ.get("RC_BIND_HOST", "0.0.0.0"),
            port=int(os.environ.get("RC_PORT", "8080")),
            cookie_secure=_env_bool("RC_COOKIE_SECURE", False),
            cookie_samesite=os.environ.get("RC_COOKIE_SAMESITE", "Lax"),
            max_failed_attempts=int(os.environ.get("RC_MAX_FAILED_ATTEMPTS", "5")),
            lockout_seconds=int(os.environ.get("RC_LOCKOUT_SECONDS", "300")),
            session_lifetime_minutes=int(os.environ.get("RC_SESSION_MINUTES", "60")),
            pbkdf2_iterations=int(os.environ.get("RC_PBKDF2_ITERATIONS", "200000")),
            testing=_env_bool("RC_TESTING", False),
            slam_source=(os.environ.get("RC_SLAM_SOURCE") or "master").strip().lower(),
            slam_ports=tuple(
                p.strip()
                for p in (os.environ.get("RC_SLAM_PORTS") or "").split(",")
                if p.strip()
            ),
            wheel_ports=tuple(
                p.strip()
                for p in (os.environ.get("RC_WHEEL_PORTS") or "").split(",")
                if p.strip()
            ),
            slam_lock_file=(
                Path(os.environ["RC_SLAM_LOCK_FILE"])
                if os.environ.get("RC_SLAM_LOCK_FILE")
                else project_root / "data" / "lock.json"
            ),
            slam_pin=(os.environ.get("RC_SLAM_PIN") or "").strip(),
            slam_max_attempts=int(os.environ.get("RC_SLAM_MAX_ATTEMPTS", "5")),
            slam_lock_seconds=int(os.environ.get("RC_SLAM_LOCK_SEC", "30")),
            kiosk_pin_main=(os.environ.get("RC_KIOSK_PIN_MAIN") or "7232").strip(),
            kiosk_pin_monitor=(os.environ.get("RC_KIOSK_PIN_MONITOR") or "5656").strip(),
            kiosk_mode_max_attempts=max(1, int(os.environ.get("RC_KIOSK_MODE_MAX_ATTEMPTS", "5"))),
            kiosk_mode_lock_seconds=max(0, int(os.environ.get("RC_KIOSK_MODE_LOCK_SEC", "30"))),
            camera_front=(
                (os.environ.get("RC_CAMERA_FRONT") or os.environ.get("RC_CAMERA_FRONT_PORT") or "").strip()
            ),
            camera_rear=(
                (os.environ.get("RC_CAMERA_REAR") or os.environ.get("RC_CAMERA_REAR_PORT") or "").strip()
            ),
            stats_file=(
                Path(os.environ["RC_STATS_FILE"])
                if os.environ.get("RC_STATS_FILE")
                else project_root / "data" / "stats.json"
            ),
            _loaded_from=str(project_root / ".env"),
        )
