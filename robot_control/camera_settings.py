"""Настройки четырёх камер C930e бортового экрана (отдельный порт на каждую).

Раньше камеры были жёстко зашиты как /dev/video0 и /dev/video1. Теперь на
борту (8090) оператор видит все V4L2-устройства, назначает порт каждой из
четырёх камер (передняя левая, передняя правая, задняя левая, задняя правая),
сохраняет выбор в `.env` и сразу переподключает камеры без перезапуска.
Углы установки и положение задаются отдельно в `.env` (см.
`sensor_layout.py` и `docs/AUTONOMY.md`) — здесь только порты.

Старые ключи `RC_CAMERA_FRONT` / `RC_CAMERA_REAR` и поля `front` / `rear`
сохранены: они означают переднюю левую и заднюю левую камеры. Пульт (8080)
читает настройки по HTTP, запись доступна только на борту.
"""

from __future__ import annotations

import glob
import os
import re
import secrets
import tempfile
from pathlib import Path
from typing import Any

from .config import PROJECT_ROOT
from .sensor_layout import CAMERA_ENV_KEYS, camera_env_keys, layout_payload

# Четыре роли в порядке id. Основной ключ пишется в .env, остальные — алиасы.
ENV_KEYS_BY_ROLE: dict[str, tuple[str, ...]] = {
    "front": ("RC_CAMERA_FRONT", "RC_CAMERA_FL_PORT", "RC_CAMERA_FRONT_PORT", "RC_CAM_FRONT"),
    "rear": ("RC_CAMERA_REAR", "RC_CAMERA_RL_PORT", "RC_CAMERA_REAR_PORT", "RC_CAM_REAR"),
    "front_right": ("RC_CAMERA_FRONT_RIGHT", "RC_CAMERA_FR_PORT", "RC_CAMERA_FRONT_RIGHT_PORT"),
    "rear_right": ("RC_CAMERA_REAR_RIGHT", "RC_CAMERA_RR_PORT", "RC_CAMERA_REAR_RIGHT_PORT"),
}
ENV_PRIMARY_KEYS: dict[str, str] = {role: keys[0] for role, keys in ENV_KEYS_BY_ROLE.items()}
SETTING_KEYS: tuple[str, ...] = tuple(ENV_KEYS_BY_ROLE)
#: Автопоиск портов отключён: пустая строка означает «камера не назначена».
#: Раньше здесь были /dev/video0…3 — это была догадка, а не настройка.
DEFAULT_PORTS: dict[str, str] = {
    "front": "", "rear": "", "front_right": "", "rear_right": "",
}
ROLE_LABELS: dict[str, str] = {
    "front": "передняя левая", "front_right": "передняя правая",
    "rear": "задняя левая", "rear_right": "задняя правая",
}
# Имена ролей в настройках портов → роли в sensor_layout (геометрия).
ROLE_TO_LAYOUT: dict[str, str] = {
    "front": "front_left", "front_right": "front_right",
    "rear": "rear_left", "rear_right": "rear_right",
}

# Исторические имена для внешнего кода/тестов.
ENV_FRONT = ENV_PRIMARY_KEYS["front"]
ENV_REAR = ENV_PRIMARY_KEYS["rear"]
ENV_FRONT_KEYS = ENV_KEYS_BY_ROLE["front"]
ENV_REAR_KEYS = ENV_KEYS_BY_ROLE["rear"]

_PORT_RE = re.compile(r"^[A-Za-z0-9_./-]+$")


def _real(path: str) -> str:
    try:
        return os.path.realpath(path)
    except OSError:
        return path


def _env_camera_value(keys: tuple[str, ...]) -> str:
    """Читает первый непустой ключ из списка алиасов."""
    for key in keys:
        value = (os.environ.get(key) or "").strip()
        if value:
            return value
    return ""


def camera_front_from_env() -> str:
    return _env_camera_value(ENV_KEYS_BY_ROLE["front"])


def camera_rear_from_env() -> str:
    return _env_camera_value(ENV_KEYS_BY_ROLE["rear"])


def current_camera_settings() -> dict[str, str]:
    """Текущие порты из окружения; пустая строка = камера не назначена."""
    return {role: _env_camera_value(keys) for role, keys in ENV_KEYS_BY_ROLE.items()}


def candidate_video_paths() -> list[str]:
    """Все доступные V4L2-порты на машине (by-id сначала)."""
    seen: dict[str, str] = {}
    for pattern in ("/dev/v4l/by-id/*", "/dev/v4l/by-path/*", "/dev/video*"):
        for path in sorted(glob.glob(pattern)):
            if not os.path.exists(path):
                continue
            real = _real(path)
            if real not in seen:
                seen[real] = path
    for configured in current_camera_settings().values():
        if configured and configured not in seen.values() and configured not in seen:
            seen[_real(configured)] = configured
    return sorted(seen.values())


def stable_camera_path(path: str) -> str:
    """Предпочтительный стабильный путь: by-id, иначе by-path, иначе как есть."""
    if not path:
        return ""
    if path.startswith("/dev/v4l/"):
        return path
    real = _real(path)
    for pattern in ("/dev/v4l/by-id/*", "/dev/v4l/by-path/*"):
        for by_path in sorted(glob.glob(pattern)):
            if _real(by_path) == real:
                return by_path
    return path


def describe_camera_ports() -> list[dict[str, Any]]:
    """Список видеопортов с метаданными для UI (роль, занятость, наличие)."""
    reserved: set[str] = set()
    try:
        from .lidar import lidar_reserved_ports, lidar_active_port
        reserved |= set(lidar_reserved_ports())
        active = lidar_active_port()
        if active:
            reserved.add(active)
            reserved.add(_real(active))
    except Exception:
        pass
    try:
        from .battery import reserved_ports as bms_reserved
        reserved |= set(bms_reserved())
    except Exception:
        pass
    try:
        from .bms_setup import master_ports
        reserved |= set(master_ports())
    except Exception:
        pass

    settings = current_camera_settings()
    assigned: dict[str, str] = {}
    for role, value in settings.items():
        if value:
            assigned[value] = role
            assigned[_real(value)] = role

    ports = sorted(set(candidate_video_paths()) | set(glob.glob("/dev/video*")))
    result: list[dict[str, Any]] = []
    for path in ports:
        real = _real(path)
        exists = os.path.exists(path) or os.path.exists(real)
        role = assigned.get(path) or assigned.get(real) or ""
        if role:
            note = f"{ROLE_LABELS[role]} камера (.env)"
        elif path in reserved or real in reserved:
            role, note = "busy", "занят другой подсистемой — не использовать для камеры"
        elif exists:
            role, note = "free", "свободен"
        else:
            role, note = "missing", "устройство не подключено"
        result.append({"port": path, "real": real, "exists": exists,
                       "role": role, "note": note})
    return result


_ALIASES: dict[str, tuple[str, ...]] = {
    "front": ("front", "frontPort", "front_port", "front_left", "cam0", "camera_front", "0"),
    "rear": ("rear", "rearPort", "rear_port", "rear_left", "cam1", "camera_rear", "1"),
    "front_right": ("front_right", "frontRight", "front_right_port", "cam2", "camera_front_right", "2"),
    "rear_right": ("rear_right", "rearRight", "rear_right_port", "cam3", "camera_rear_right", "3"),
}


def normalize_camera_settings(payload: Any) -> dict[str, str]:
    """Проверяет и нормализует порты четырёх камер.

    Принимает dict с ключами front/rear/front_right/rear_right (и алиасами).
    Пустая строка = камера не назначена (автопоиск отключён). Две камеры не
    могут делить один порт.
    """
    if not isinstance(payload, dict):
        raise ValueError("настройки камер: ожидается объект с портами front/rear/front_right/rear_right")

    def pick(keys: tuple[str, ...]) -> str:
        for key in keys:
            if key in payload:
                value = payload[key]
                return "" if value is None else str(value).strip()
        return ""

    values = {role: pick(keys) for role, keys in _ALIASES.items()}
    for role, value in values.items():
        if not value:
            continue
        label = ROLE_LABELS[role]
        if len(value) > 256:
            raise ValueError(f"порт {label} камеры слишком длинный")
        if "\n" in value or "\r" in value or any(ch in value for ch in "\"'`$;|&"):
            raise ValueError(f"недопустимые символы в пути {label} камеры")
        if not _PORT_RE.match(value):
            raise ValueError(f"недопустимый путь {label} камеры: {value}")
        if not value.startswith("/dev/"):
            raise ValueError(f"порт {label} камеры должен начинаться с /dev/: {value}")
        # Разрешаем любой /dev/*: by-id/by-path и нестандартные VideoCapture
        # индексы проверяет сам драйвер, а UI покажет порт как «настроенный».
    non_empty = [v for v in values.values() if v]
    if len(non_empty) != len(set(non_empty)):
        raise ValueError("две камеры не могут использовать один и тот же порт")
    return values


def env_file_path() -> Path:
    """``.env`` проекта; ``RC_CAMERA_ENV_FILE`` — изоляция в тестах/на другом борту."""
    override = (os.environ.get("RC_CAMERA_ENV_FILE") or "").strip()
    if override:
        return Path(override)
    return PROJECT_ROOT / ".env"


def save_camera_settings(settings: dict[str, str], env_path: Path | None = None) -> dict[str, str]:
    """Пишет порты четырёх камер в .env атомарно, не трогая остальные строки."""
    normalized = normalize_camera_settings(settings)
    target = Path(env_path) if env_path is not None else env_file_path()
    lines = target.read_text(encoding="utf-8").splitlines() if target.is_file() else []
    values = {ENV_PRIMARY_KEYS[role]: normalized[role] for role in SETTING_KEYS}
    all_camera_keys: set[str] = set()
    for keys in ENV_KEYS_BY_ROLE.values():
        all_camera_keys.update(keys)
    for keys in CAMERA_ENV_KEYS.values():
        all_camera_keys.update(keys)
    seen: set[str] = set()
    out: list[str] = []
    for raw in lines:
        stripped = raw.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            out.append(raw)
            continue
        key = stripped.split("=", 1)[0].strip()
        if key not in all_camera_keys:
            out.append(raw)
            continue
        if key in values:
            if key in seen:
                continue
            seen.add(key)
            out.append(f"{key}={values[key]}")
        # Алиасы удаляем, чтобы не было двух значений на одну камеру.
    for key, value in values.items():
        if key not in seen:
            out.append(f"{key}={value}")
    if not target.is_file() and not any(line.strip().startswith("RC_SECRET_KEY=") for line in out):
        out.insert(0, f"RC_SECRET_KEY={secrets.token_urlsafe(48)}")
    target.parent.mkdir(parents=True, exist_ok=True)
    mode = 0o600
    try:
        mode = target.stat().st_mode & 0o777 or 0o600
    except OSError:
        pass
    fd, tmp_name = tempfile.mkstemp(prefix=".env-", suffix=".tmp", dir=target.parent)
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
    return normalized


def settings_payload(manager=None) -> dict[str, Any]:
    """Пейлоад для UI: порты, варианты, статус камер и геометрия сенсоров."""
    try:
        from .camera import CameraManager
        mgr = manager or CameraManager.get_instance()
        cameras = mgr.list_cameras()
    except Exception:
        cameras = []
    raw = current_camera_settings()
    display = {role: raw[role] for role in SETTING_KEYS}
    return {
        "ok": True,
        "settings": raw,
        "displaySettings": display,
        "defaultPorts": dict(DEFAULT_PORTS),
        "roleLabels": dict(ROLE_LABELS),
        "options": {"ports": describe_camera_ports()},
        "cameras": cameras,
        "envKeys": {role: ENV_PRIMARY_KEYS[role] for role in SETTING_KEYS},
        "layout": {
            "calibrated": layout_payload()["calibrated"],
            "cameraEnvKeys": {role: camera_env_keys(camera_role)
                              for role, camera_role in ROLE_TO_LAYOUT.items()},
        },
    }


def apply_camera_settings(payload: Any, manager=None, env_path: Path | None = None) -> dict[str, Any]:
    """Сохраняет четыре порта в .env и сразу применяет к менеджеру камер."""
    inner = payload.get("settings") if isinstance(payload, dict) and "settings" in payload else payload
    normalized = save_camera_settings(inner, env_path=env_path)
    for role, keys in ENV_KEYS_BY_ROLE.items():
        for key in keys:
            os.environ.pop(key, None)
        if normalized[role]:
            os.environ[ENV_PRIMARY_KEYS[role]] = normalized[role]
    try:
        from .camera import CameraManager
        mgr = manager or CameraManager.get_instance()
        mgr.reconfigure(normalized)
        cameras = mgr.list_cameras()
    except Exception:
        cameras = []
    return {"ok": True, "settings": normalized, "cameras": cameras, "status": "применено"}
