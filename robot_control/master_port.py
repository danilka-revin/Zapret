"""USB port names and reservations. Wheel assignments are manual only.

Listing serial devices never opens them. Legacy RC_SLAM_PORT diagnostics are
retained for compatibility; the wheel source uses only RC_WHEEL_*_PORT.
"""

from __future__ import annotations

import glob
import os
import re
from typing import Any, Iterable

#: Переменная с портом мастера (и её исторический синоним).
ENV_KEY = "RC_SLAM_PORT"
ENV_ALIASES = ("RC_SLAM_MASTER_PORT",)
#: Порт по умолчанию, если не задано ничего другого.
DEFAULT_PORT = "/dev/ttyACM0"
#: Сначала постоянные имена by-id, потом обычные имена ядра.
PORT_PATTERNS = ("/dev/serial/by-path/*", "/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*")
#: Признак телеметрии мастера в строке монитора.
TELEMETRY_PREFIX = "@TLM "


def _real(path: str) -> str:
    try:
        return os.path.realpath(path)
    except OSError:
        return path


def _natural_key(path: str) -> list[Any]:
    """``/dev/ttyUSB2`` раньше ``/dev/ttyUSB10``, как в ``lidar.py``."""
    return [int(part) if part.isdigit() else part.lower()
            for part in re.split(r"(\d+)", path)]


def normalize_port(value: str) -> str:
    """Привести имя порта к виду, который понимает pyserial.

    ``ttyUSB0`` и ``usb0`` люди пишут без ``/dev/`` — добавляем его, если такой
    файл существует. Остальное возвращаем как есть: путь откроет pyserial и
    скажет понятную ошибку. Никакие запасные порты не проверяются.
    """
    text = str(value or "").strip().strip('"').strip("'")
    text = re.sub(r"[\x00-\x1f\x7f]", "", text).strip()
    if not text:
        return ""
    if text.startswith("/"):
        return text
    for candidate in ("/dev/" + text, "/dev/serial/by-id/" + text):
        if os.path.exists(candidate):
            return candidate
    return text


def configured_port() -> str:
    """Порт из ``.env``/окружения (с синонимом), иначе порт по умолчанию."""
    for name in (ENV_KEY,) + ENV_ALIASES:
        value = normalize_port(os.environ.get(name) or "")
        if value:
            return value
    return DEFAULT_PORT


def serial_ports(patterns: Iterable[str] = PORT_PATTERNS) -> list[str]:
    """Все USB-последовательные порты, по одному имени на устройство.

    Постоянное имя из ``/dev/serial/by-id`` важнее ``/dev/ttyUSBx``: оно не
    меняется после перезагрузки, поэтому в ``.env`` лучше писать его.
    """
    found: list[str] = []
    seen: set[str] = set()
    for pattern in patterns:
        for path in sorted(glob.glob(pattern), key=_natural_key):
            real = _real(path)
            if real in seen:
                continue
            seen.add(real)
            found.append(path)
    return found


def reserved_ports() -> set[str]:
    """Порты лидара и BMS: открывать их мастеру нельзя.

    Ошибки импорта не должны ломать поиск: без списка исключений мастер просто
    проверит больше портов, а не откажется работать.
    """
    values: set[str] = set()
    try:
        from .battery import reserved_ports as bms_reserved
        values |= set(bms_reserved())
    except Exception:  # noqa: BLE001 — модуль BMS может быть недоступен
        pass
    try:
        from .lidar import lidar_active_port, lidar_reserved_ports
        values |= set(lidar_reserved_ports())
        active = lidar_active_port()
        if active:
            values.add(active)
    except Exception:  # noqa: BLE001 — лидар не должен мешать мастеру
        pass
    # Явные исключения оператора и порты модулей шасси (в режиме master они не
    # используются): открывать их мастеру тоже нельзя.
    for name in ("RC_SLAM_PORTS", "RC_LIDAR_EXCLUDE"):
        for chunk in (os.environ.get(name) or "").replace(";", ",").split(","):
            chunk = chunk.strip()
            if chunk:
                values.add(chunk)
    out: set[str] = set()
    for value in values:
        value = normalize_port(value)
        if not value:
            continue
        out.add(value)
        out.add(_real(value))
    return out


def master_ports(configured: str | None = None) -> list[str]:
    """Only explicitly assigned wheel ports; no fallback or USB scan."""
    from .wheel_settings import current_settings
    blocked = reserved_ports()
    out, seen = [], set()
    for path in current_settings().values():
        if not path:
            continue
        real = _real(path)
        if real in seen or path in blocked or real in blocked:
            continue
        seen.add(real)
        out.append(path)
    return out


def stable_port_path(port: str) -> str:
    """Постоянное имя ``/dev/serial/by-id/…`` для порта (если оно есть).

    Номер ``ttyUSBx`` зависит от порядка подключения USB-устройств; имя by-id
    привязано к самому переходнику и переживает перезагрузку. Им и надо
    заполнять ``RC_SLAM_PORT``.
    """
    if not port:
        return port
    try:
        from .lidar import stable_port_path as lidar_stable_path
        return lidar_stable_path(port)
    except Exception:                     # noqa: BLE001 — без лидара живём и так
        return port


def conflict_note(configured: str | None = None) -> str:
    """Почему настроенный порт пропущен (лидар/BMS), или пустая строка."""
    first = normalize_port(configured) if configured is not None else configured_port()
    if not first:
        return ""
    blocked = reserved_ports()
    if first in blocked or _real(first) in blocked:
        return ("RC_SLAM_PORT=%s занят лидаром или BMS — выберите "
                "порт вручную; поправьте .env" % first)
    return ""


def port_hint(configured: str | None = None) -> str:
    """No device probing: direct the operator to manual assignments."""
    ports = master_ports()
    if not ports:
        return "Порты колёс не назначены: бортовой экран → Тяга и руление → USB-порты колёс"
    missing = [p for p in ports if not os.path.exists(p)]
    return "Назначенные порты не найдены: " + ", ".join(missing) if missing else ""


def diagnostics(configured: str | None = None) -> dict[str, Any]:
    """Снимок для журнала запуска и скрипта проверки."""
    first = normalize_port(configured) if configured is not None else configured_port()
    first = first or DEFAULT_PORT
    ports = serial_ports()
    blocked = sorted(p for p in reserved_ports() if p)
    return {
        "configured": first,
        "configuredExists": os.path.exists(first),
        "accessible": os.access(first, os.R_OK | os.W_OK) if os.path.exists(first) else False,
        "conflict": conflict_note(first) or None,
        "ports": ports,
        "reserved": blocked,
        "candidates": master_ports(first),
        "hint": port_hint(first),
    }
