"""Ручное назначение портов всех устройств робота.

Автопоиск портов отключён: ни один порт не назначается программой. Оператор
на бортовом экране (страница «Устройства») сам указывает:

* четыре порта колёсных модулей — платы сами сообщают роли FL/FR/RL/RR в
  защищённой телеметрии fw22+, поэтому порядок портов не важен;
* отдельный видеопорт каждой из четырёх камер (FL, FR, RL, RR);
* порт лидара, Kinect, чекового принтера и BMS.

Значения хранятся в ``.env`` (те же ключи, что читают драйверы) и сразу
применяются к работающим подсистемам там, где это возможно. Никаких
«подсказок» вместо настроек: сохранённый путь — это назначение оператора,
а не доказательство типа устройства. Безопасность движения от этого не
меняется: колесо считается найденным только после свежей телеметрии с CRC.
"""
from __future__ import annotations

import glob
import os
import re
import secrets
import tempfile
from pathlib import Path
from typing import Any, Iterable

from .config import PROJECT_ROOT

WHEELS = ("FL", "FR", "RL", "RR")
WHEEL_TITLES = {"FL": "Переднее левое", "FR": "Переднее правое",
                "RL": "Заднее левое", "RR": "Заднее правое"}

#: Один ключ на четыре порта: роли колёс определяет прошивка, а не порядок.
WHEEL_ENV_KEY = "RC_WHEEL_PORTS"
WHEEL_SLOTS = 4

#: Роль страницы → ключ .env (первый ключ — тот, что пишется).
ROLE_ENV: dict[str, tuple[str, ...]] = {
    "camera_FL": ("RC_CAMERA_FRONT",),
    "camera_FR": ("RC_CAMERA_FRONT_RIGHT",),
    "camera_RL": ("RC_CAMERA_REAR",),
    "camera_RR": ("RC_CAMERA_REAR_RIGHT",),
    "lidar": ("RC_LIDAR_PORT",),
    "kinect": ("RC_KINECT_PORT",),
    "bms": ("RC_BMS_PORT",),
    "printer": ("RC_PRINTER_PORT",),
}
#: Роль страницы → ключ настроек камер (camera_settings).
CAMERA_SETTING: dict[str, str] = {
    "camera_FL": "front", "camera_FR": "front_right",
    "camera_RL": "rear", "camera_RR": "rear_right",
}
ROLE_TITLES: dict[str, str] = {
    "camera_FL": "Камера · передняя левая",
    "camera_FR": "Камера · передняя правая",
    "camera_RL": "Камера · задняя левая",
    "camera_RR": "Камера · задняя правая",
    "lidar": "Лидар",
    "kinect": "Kinect · глубина",
    "bms": "BMS · аккумулятор",
    "printer": "Чековый принтер",
}
ROLE_KIND: dict[str, str] = {
    "camera_FL": "video", "camera_FR": "video",
    "camera_RL": "video", "camera_RR": "video",
    "lidar": "serial", "bms": "serial", "printer": "serial", "kinect": "usb",
}
ROLE_ENV_HINTS: dict[str, str] = {
    "lidar": "Порт UART лидара. Сохраняется в .env и применяется сразу.",
    "kinect": "USB-топология Kinect (usb:1-2). Публикатор глубины читает её при запуске.",
    "bms": "Порт USB-UART BMS (JBD). Сохраняется в .env и применяется сразу.",
    "printer": "Порт USB-UART принтера. Подтверждается физически и сразу включается.",
}

SERIAL_RE = re.compile(r"^/dev/(?:tty(?:USB|ACM|AMA)\d+|serial/by-(?:id|path)/[A-Za-z0-9_.:+-]+)$")
VIDEO_RE = re.compile(r"^/dev/(?:video\d+|v4l/by-(?:id|path)/[A-Za-z0-9_.:+-]+)$")
USB_RE = re.compile(r"^(?:usb:\d+-\d+(?:\.\d+)*|/dev/bus/usb/\d{3}/\d{3})$")
KIND_PATTERNS = {"serial": SERIAL_RE, "video": VIDEO_RE, "usb": USB_RE}


class PortError(ValueError):
    """Назначение порта отклонено; текст показывается оператору."""


def env_file_path() -> Path:
    """Файл настроек: ``.env`` проекта или явный путь (тесты/несколько бортов)."""
    override = (os.environ.get("RC_DEVICE_PORTS_FILE") or "").strip()
    if override:
        return Path(override)
    return PROJECT_ROOT / ".env"


def _real(path: str) -> str:
    try:
        return os.path.realpath(path)
    except OSError:
        return path


def _clean(value: Any) -> str:
    return re.sub(r"[\x00-\x1f\x7f]", "", str(value or "")).strip()


def env_value(*keys: str) -> str:
    """Первое непустое значение из окружения (``.env`` грузится при старте)."""
    for key in keys:
        value = _clean(os.environ.get(key))
        if value:
            return value
    return ""


def wheel_ports(raw: Iterable[str] | None = None) -> list[str]:
    """Четыре ручных порта колёс; позиция в списке не задаёт роль колеса."""
    if raw is None:
        source: Iterable[str] = (os.environ.get(WHEEL_ENV_KEY) or "").split(",")
    else:
        source = raw
    slots: list[str] = []
    seen: set[str] = set()
    for chunk in source:
        value = _clean(chunk)
        if value:
            real = _real(value)
            if real in seen:
                continue
            seen.add(real)
        slots.append(value)
    return (slots + [""] * WHEEL_SLOTS)[:WHEEL_SLOTS]


def role_port(role: str) -> str:
    keys = ROLE_ENV.get(role)
    if not keys:
        return ""
    return env_value(*keys)


def assignments() -> dict[str, str]:
    return {role: role_port(role) for role in ROLE_ENV}


def assigned_ports(except_role: str | None = None) -> dict[str, str]:
    """Все назначенные порты (включая четыре колеса) для проверки уникальности."""
    out: dict[str, str] = {}
    for index, port in enumerate(wheel_ports()):
        if port and except_role != "wheels":
            out["wheel_" + str(index + 1)] = port
    for role in ROLE_ENV:
        if role == except_role:
            continue
        port = role_port(role)
        if port:
            out[role] = port
    return out


def assignment_owner(port: str, except_role: str | None = None) -> str:
    """Кому уже назначен этот путь (по realpath), или пустая строка."""
    if not port:
        return ""
    real = _real(port)
    for role, value in assigned_ports(except_role=except_role).items():
        if value == port or _real(value) == real:
            return role
    return ""


def validate(role: str, port: str) -> str:
    """Проверить путь и уникальность назначения. Пустая строка = «не назначен»."""
    port = _clean(port)
    if role == "wheels":
        raise PortError("Четыре порта колёс проверяются вместе (validate_wheels)")
    if role not in ROLE_ENV:
        raise PortError("Неизвестное устройство: %s" % role)
    if not port:
        return ""
    if len(port) > 256 or ".." in port:
        raise PortError("Недопустимый путь порта")
    kind = "serial" if role == "wheels" else ROLE_KIND[role]
    if not KIND_PATTERNS[kind].fullmatch(port):
        if kind == "video":
            raise PortError("Для камеры нужен видеопорт /dev/video… или /dev/v4l/…")
        if kind == "usb":
            raise PortError("Для Kinect нужна USB-топология вида usb:1-2")
        raise PortError("Выберите USB-порт /dev/ttyUSB…, /dev/ttyACM… или /dev/serial/…")
    owner = assignment_owner(port, except_role="wheels" if role == "wheels" else role)
    if owner:
        raise PortError("Этот порт уже назначен другому устройству: %s" % owner)
    return port


def validate_wheels(ports: Iterable[Any]) -> list[str]:
    values = [_clean(value) for value in (ports or [])]
    values = (values + [""] * WHEEL_SLOTS)[:WHEEL_SLOTS]
    if any(len(value) > 256 or ".." in value for value in values):
        raise PortError("Недопустимый путь порта")
    seen: set[str] = set()
    for index, value in enumerate(values):
        if not value:
            continue
        if not SERIAL_RE.fullmatch(value):
            raise PortError("Порт %d: выберите USB-порт /dev/ttyUSB…, /dev/ttyACM… или /dev/serial/…"
                            % (index + 1))
        real = _real(value)
        if real in seen:
            raise PortError("Один USB-порт нельзя указать дважды")
        seen.add(real)
        owner = assignment_owner(value, except_role="wheels")
        if owner:
            raise PortError("Порт %d уже назначен другому устройству: %s" % (index + 1, owner))
    return values


# --------------------------------------------------------------- .env запись #
def write_env(values: dict[str, str], env_path: Path | None = None) -> Path:
    """Атомарно записать ключи в ``.env``, не трогая остальные строки."""
    target = Path(env_path) if env_path is not None else env_file_path()
    lines = target.read_text(encoding="utf-8").splitlines() if target.is_file() else []
    out: list[str] = []
    written: set[str] = set()
    for raw in lines:
        stripped = raw.strip()
        key = stripped.split("=", 1)[0].strip() if "=" in stripped else ""
        if key in values:
            if key in written:
                continue
            written.add(key)
            out.append("%s=%s" % (key, values[key]))
        else:
            out.append(raw)
    for key, value in values.items():
        if key not in written:
            out.append("%s=%s" % (key, value))
    if not target.is_file() and not any(line.strip().startswith("RC_SECRET_KEY=") for line in out):
        out.insert(0, "RC_SECRET_KEY=%s" % secrets.token_urlsafe(48))
    target.parent.mkdir(parents=True, exist_ok=True)
    mode = 0o600
    try:
        mode = target.stat().st_mode & 0o777 or 0o600
    except OSError:
        pass
    fd, tmp_name = tempfile.mkstemp(prefix=".env-ports-", suffix=".tmp", dir=target.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write("\n".join(out).rstrip() + "\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(tmp_name, mode)
        os.replace(tmp_name, target)
    finally:
        try:
            os.unlink(tmp_name)
        except FileNotFoundError:
            pass
    for key, value in values.items():
        if value:
            os.environ[key] = value
        else:
            os.environ.pop(key, None)
    return target


def save_wheels(ports: Iterable[Any], env_path: Path | None = None) -> list[str]:
    values = validate_wheels(ports)
    write_env({WHEEL_ENV_KEY: ",".join(value for value in values if value)}, env_path=env_path)
    return values


def save_role(role: str, port: str, env_path: Path | None = None) -> str:
    value = validate(role, port)
    keys = ROLE_ENV[role]
    write_env({keys[0]: value}, env_path=env_path)
    return value


# ------------------------------------------------------------- список портов #
def _serial_options() -> list[dict[str, Any]]:
    """Все последовательные порты машины. Список только для выбора, не для назначения."""
    seen: dict[str, dict[str, Any]] = {}
    descriptions: dict[str, Any] = {}
    try:
        from serial.tools import list_ports
        for item in list_ports.comports():
            descriptions[_real(item.device)] = item
    except Exception:  # noqa: BLE001 — без pyserial список остаётся пустым/glob
        descriptions = {}
    paths: list[str] = []
    for pattern in ("/dev/serial/by-id/*", "/dev/serial/by-path/*", "/dev/ttyUSB*", "/dev/ttyACM*"):
        paths.extend(sorted(glob.glob(pattern)))
    for role_port_value in list(wheel_ports()) + list(assignments().values()):
        if role_port_value.startswith("/dev/") and role_port_value not in paths:
            paths.append(role_port_value)
    for path in paths:
        real = _real(path)
        if real in seen:
            continue
        item = descriptions.get(real)
        name = " · ".join(part for part in (getattr(item, "manufacturer", None),
                                            getattr(item, "product", None)) if part and part != "n/a")
        seen[real] = {"port": path, "name": name or (item.description if item else "") or "",
                      "exists": os.path.exists(path)}
    return list(seen.values())


def _video_options(registry=None) -> list[dict[str, Any]]:
    try:
        from .camera_settings import candidate_video_paths, stable_camera_path
        paths = list(candidate_video_paths())
    except Exception:  # noqa: BLE001 — настройки не должны падать
        stable_camera_path = lambda value: value  # noqa: E731
        paths = sorted(glob.glob("/dev/video*"))
    # Дополнительно показываем видеопорты из USB-топологии: на некоторых
    # сборках by-id нет, и glob /dev/video* ничего не находит.
    try:
        topology = registry.usb() if registry is not None else None
        for device in (topology or {}).get("devices", []):
            for node in device.get("nodes", []):
                if node.get("kind") == "video" and node.get("path") not in paths:
                    paths.append(node["path"])
    except Exception:  # noqa: BLE001 — список остаётся тем, что нашли
        pass
    unique: dict[str, dict[str, Any]] = {}
    for path in paths:
        real = _real(path)
        if real in unique:
            continue
        stable = stable_camera_path(path) or path
        unique[real] = {"port": stable, "name": "V4L2", "exists": os.path.exists(stable)}
    return list(unique.values())


def _usb_options(registry=None) -> list[dict[str, Any]]:
    try:
        topology = registry.usb() if registry is not None else None
        if not topology:
            from .device_registry import KINECT_CAMERA_IDS, UsbTopology
            topology = UsbTopology().collect()
    except Exception:  # noqa: BLE001 — без sysfs список пуст
        return []
    from .device_registry import KINECT_CAMERA_IDS
    out = []
    for device in topology.get("devices", []):
        if device.get("isHub"):
            continue
        out.append({"port": "usb:" + str(device.get("topology")),
                    "name": device.get("product") or "USB-устройство",
                    "note": device.get("connection") or "",
                    "vidPid": device.get("vidPid"),
                    "kinect": device.get("vidPid") in KINECT_CAMERA_IDS,
                    "exists": True})
    return out


def payload(registry=None, wheel_status: dict | None = None,
            env_path: Path | None = None) -> dict[str, Any]:
    """Текущие ручные назначения + списки доступных портов для выбора."""
    status = wheel_status if isinstance(wheel_status, dict) else {}
    modules = status.get("modules") if isinstance(status.get("modules"), dict) else {}
    roles = {}
    for role, title in ROLE_TITLES.items():
        port = role_port(role)
        roles[role] = {
            "title": title,
            "port": port,
            "envKey": ROLE_ENV[role][0],
            "kind": ROLE_KIND[role],
            "assigned": bool(port),
            "note": ROLE_ENV_HINTS.get(role, "Отдельный порт этой камеры."),
        }
    options = {"serial": _serial_options(), "video": _video_options(registry),
               "usb": _usb_options(registry)}
    return {
        "ok": True,
        "manual": True,
        "envFile": str(Path(env_path) if env_path is not None else env_file_path()),
        "wheels": {
            "envKey": WHEEL_ENV_KEY,
            "slots": wheel_ports(),
            "titles": [WHEEL_TITLES[w] for w in WHEELS],
            "resolved": {wheel: modules.get(wheel) or "" for wheel in WHEELS},
            "ready": bool(status.get("ready")),
            "candidates": status.get("candidates") or [],
            "error": status.get("error"),
        },
        "roles": roles,
        "options": options,
    }
