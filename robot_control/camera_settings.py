"""Настройки камер бортового экрана: передняя и задняя — отдельные порты.

Раньше обе камеры были жёстко зашиты как /dev/video0 и /dev/video1 без
возможности перенастроить из интерфейса. Теперь оператор на бортовом экране
(8090) может:

* увидеть все доступные V4L2-устройства (/dev/video*, /dev/v4l/by-id/*);
* выбрать порт отдельно для передней (CAM 0) и задней (CAM 1);
* сохранить выбор в .env (RC_CAMERA_FRONT / RC_CAMERA_REAR) и сразу
  переподключить камеры без перезапуска сервера;
* выполнить автопоиск — система сама найдёт свободные видеодевайсы.

Порт лидара и моста шасси не трогаются, но камеры — независимые V4L2-
устройства, поэтому конфликтов с ними нет. На пульте (8080) настройки
только читаются; запись — только на борту (8090), где камеры физически
подключены.
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

# Ключи .env для камер. Поддерживаем алиасы *_PORT для единообразия с
# другими настройками (RC_BMS_PORT, RC_LIDAR_PORT).
ENV_FRONT_KEYS = ("RC_CAMERA_FRONT", "RC_CAMERA_FRONT_PORT", "RC_CAM_FRONT")
ENV_REAR_KEYS = ("RC_CAMERA_REAR", "RC_CAMERA_REAR_PORT", "RC_CAM_REAR")
# Основные ключи, которые пишутся в .env (остальные считаются алиасами при чтении)
ENV_FRONT = "RC_CAMERA_FRONT"
ENV_REAR = "RC_CAMERA_REAR"

# Допустимые префиксы устройств камер
_VALID_PREFIXES = ("/dev/video", "/dev/v4l/")

# Безопасный паттерн для имени порта (не должен содержать опасных символов)
_PORT_RE = re.compile(r"^[A-Za-z0-9_./-]+$")


def _real(path: str) -> str:
    try:
        return os.path.realpath(path)
    except OSError:
        return path


def _env_camera_value(keys: tuple[str, ...]) -> str:
    """Читает первый непустой ключ из списка алиасов."""
    for k in keys:
        v = (os.environ.get(k) or "").strip()
        if v:
            return v
    return ""


def camera_front_from_env() -> str:
    return _env_camera_value(ENV_FRONT_KEYS)


def camera_rear_from_env() -> str:
    return _env_camera_value(ENV_REAR_KEYS)


def current_camera_settings() -> dict[str, str]:
    """Текущие настройки камер из окружения (пусто = авто)."""
    return {"front": camera_front_from_env(), "rear": camera_rear_from_env()}


def candidate_video_paths() -> list[str]:
    """Все доступные V4L2-порты на машине."""
    seen: dict[str, str] = {}
    # Сначала стабильные by-id/by-path, затем /dev/video*
    for pattern in ("/dev/v4l/by-id/*", "/dev/v4l/by-path/*", "/dev/video*"):
        for path in sorted(glob.glob(pattern)):
            if not os.path.exists(path):
                continue
            real = _real(path)
            if real not in seen:
                # Предпочитаем by-id путь как отображаемый, если он есть
                seen[real] = path
    # Добавить настроенные порты, даже если устройство сейчас не подключено
    for configured in (camera_front_from_env(), camera_rear_from_env()):
        if configured and configured not in seen.values() and configured not in seen:
            seen[_real(configured)] = configured
    return sorted(seen.values())


def stable_camera_path(path: str) -> str:
    """Предпочтительный стабильный путь: by-id если есть, иначе реальный."""
    if not path:
        return ""
    # Если это уже by-id/by-path — оставить как есть
    if path.startswith("/dev/v4l/"):
        return path
    # Попробовать найти by-id symlink на это устройство
    real = _real(path)
    for by_id in sorted(glob.glob("/dev/v4l/by-id/*")):
        if _real(by_id) == real:
            return by_id
    for by_path in sorted(glob.glob("/dev/v4l/by-path/*")):
        if _real(by_path) == real:
            return by_path
    return path


def describe_camera_ports() -> list[dict[str, Any]]:
    """Список видеопортов с метаданными для UI."""
    # Резервируемые порты (лидар/шасси) — камеры их не должны занимать,
    # но по факту это разные подсистемы (/dev/video vs /dev/tty*), так что
    # обычно пересечений нет. Всё равно помечаем, если совпало.
    reserved: set[str] = set()
    try:
        from .lidar import lidar_reserved_ports, lidar_active_port

        reserved |= set(lidar_reserved_ports())
        ap = lidar_active_port()
        if ap:
            reserved.add(ap)
            reserved.add(_real(ap))
    except Exception:
        pass
    try:
        from .battery import reserved_ports as bms_reserved

        reserved |= set(bms_reserved())
    except Exception:
        pass
    try:
        from .bms_setup import master_ports

        reserved |= master_ports()
    except Exception:
        pass

    result: list[dict[str, Any]] = []
    # Соберём candidate + настроенные (даже отсутствующие)
    ports = candidate_video_paths()
    # Также добавить все /dev/video* даже если glob их не нашёл из-за прав
    for extra in sorted(glob.glob("/dev/video*")):
        if extra not in ports:
            ports.append(extra)
    ports = sorted(set(ports))
    front = camera_front_from_env()
    rear = camera_rear_from_env()
    front_real = _real(front) if front else ""
    rear_real = _real(rear) if rear else ""
    for path in ports:
        real = _real(path)
        exists = os.path.exists(path) or os.path.exists(real)
        if path == front or real == front_real:
            role, note = "front", "передняя камера (RC_CAMERA_FRONT)"
        elif path == rear or real == rear_real:
            role, note = "rear", "задняя камера (RC_CAMERA_REAR)"
        elif path in reserved or real in reserved:
            role, note = "busy", "занят другой подсистемой — не использовать для камеры"
        elif exists:
            role, note = "free", "свободен"
        else:
            role, note = "missing", "устройство не подключено"
        result.append(
            {
                "port": path,
                "real": real,
                "exists": exists,
                "role": role,
                "note": note,
            }
        )
    # Если список пуст — вернуть хотя бы подсказку с дефолтами
    if not result:
        result = [
            {"port": "/dev/video0", "real": "/dev/video0", "exists": False, "role": "front", "note": "по умолчанию передняя (авто)"},
            {"port": "/dev/video1", "real": "/dev/video1", "exists": False, "role": "rear", "note": "по умолчанию задняя (авто)"},
        ]
    return result


def normalize_camera_settings(payload: Any) -> dict[str, str]:
    """Проверяет и нормализует настройки камер.

    Принимает dict с ключами front/rear (или frontPort/rearPort, 0/1).
    Пустая строка = авто (/dev/video0 / /dev/video1).
    """
    if not isinstance(payload, dict):
        raise ValueError("настройки камер: ожидается объект с полями front/rear")

    def pick(*keys: str) -> str:
        for k in keys:
            if k in payload:
                v = payload[k]
                if v is None:
                    return ""
                return str(v).strip()
        return ""

    front = pick("front", "frontPort", "front_port", "cam0", "camera_front", "0")
    rear = pick("rear", "rearPort", "rear_port", "cam1", "camera_rear", "1")

    # Алиасы: если передали только один порт как "port" — игнорируем, нужны оба
    # Но поддержим общий случай: payload = {"settings": {"front": ...}} уже развернут выше

    for label, value in (("передняя", front), ("задняя", rear)):
        if not value:
            continue
        if len(value) > 256:
            raise ValueError(f"порт {label} камеры слишком длинный")
        if "\n" in value or "\r" in value or any(ch in value for ch in "\"'`$;|&"):
            raise ValueError(f"недопустимые символы в пути {label} камеры")
        if not _PORT_RE.match(value):
            raise ValueError(f"недопустимый путь {label} камеры: {value}")
        # Разрешаем только /dev/... или пусто (авто)
        if not value.startswith("/dev/"):
            raise ValueError(f"порт {label} камеры должен начинаться с /dev/: {value}")
        # Разрешаем видеодевайсы и v4l пути; но не запрещаем tty для тестов
        # Проверка префикса мягкая — предупреждаем, но не блокируем кастомные пути
        # Для строгости: требуем хотя бы /dev/video или /dev/v4l
        if not value.startswith(_VALID_PREFIXES) and not value.startswith("/dev/video"):
            # Для гибкости позволим любой /dev/*, т.к. некоторые камеры могут быть
            # на /dev/v4l/by-id/..., но предупредим через note в UI, не здесь
            pass
        if front and rear and front == rear:
            raise ValueError("передняя и задняя камеры не могут использовать один порт")

    return {"front": front, "rear": rear}


def env_file_path() -> Path:
    return PROJECT_ROOT / ".env"


def save_camera_settings(settings: dict[str, str], env_path: Path | None = None) -> dict[str, str]:
    """Пишет настройки камер в .env атомарно, не трогая остальные строки."""
    normalized = normalize_camera_settings(settings)
    target = Path(env_path) if env_path is not None else env_file_path()
    lines: list[str] = []
    if target.is_file():
        lines = target.read_text(encoding="utf-8").splitlines()
    values = {ENV_FRONT: normalized["front"], ENV_REAR: normalized["rear"]}
    # Удаляем все алиасы при записи, оставляем только основные
    all_camera_keys = set(ENV_FRONT_KEYS) | set(ENV_REAR_KEYS)
    seen: set[str] = set()
    out: list[str] = []
    for raw in lines:
        stripped = raw.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            out.append(raw)
            continue
        key = stripped.split("=", 1)[0].strip()
        if key in all_camera_keys:
            if key in (ENV_FRONT, ENV_REAR):
                if key in seen:
                    continue
                seen.add(key)
                out.append(f"{key}={values[key]}")
            else:
                # Алиас — удаляем, чтобы не дублировать
                if key in seen:
                    continue
                # Не добавляем алиас обратно
                continue
        else:
            out.append(raw)
    for key in (ENV_FRONT, ENV_REAR):
        if key not in seen:
            out.append(f"{key}={values[key]}")
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
    """Пейлоад для UI: текущие настройки, варианты портов и статус камер."""
    try:
        from .camera import CameraManager

        mgr = manager or CameraManager.get_instance()
        cameras = mgr.list_cameras()
    except Exception:
        mgr = None
        cameras = []
    settings = current_camera_settings()
    # Если настройки пустые — показать дефолты в UI как подсказку
    if not settings["front"]:
        settings["front"] = "/dev/video0"
    if not settings["rear"]:
        settings["rear"] = "/dev/video1"
    # Но для сохранения пусто означает авто, поэтому в payload отдаём реальные env + подсказку
    raw_settings = current_camera_settings()
    discovered = describe_camera_ports()
    return {
        "ok": True,
        "settings": {"front": raw_settings["front"], "rear": raw_settings["rear"]},
        "displaySettings": settings,
        "options": {"ports": discovered},
        "cameras": cameras,
        "envKeys": {"front": ENV_FRONT, "rear": ENV_REAR},
    }


def apply_camera_settings(payload: Any, manager=None, env_path: Path | None = None) -> dict[str, Any]:
    """Сохраняет настройки в .env и сразу применяет к менеджеру камер."""
    # payload может быть {settings: {front, rear}} или {front, rear}
    settings = payload.get("settings") if isinstance(payload, dict) and "settings" in payload else payload
    normalized = save_camera_settings(settings, env_path=env_path)
    # Обновляем окружение текущего процесса
    if normalized["front"]:
        os.environ[ENV_FRONT] = normalized["front"]
        # Чистим алиасы
        for k in ENV_FRONT_KEYS:
            if k != ENV_FRONT:
                os.environ.pop(k, None)
    else:
        for k in ENV_FRONT_KEYS:
            os.environ.pop(k, None)
    if normalized["rear"]:
        os.environ[ENV_REAR] = normalized["rear"]
        for k in ENV_REAR_KEYS:
            if k != ENV_REAR:
                os.environ.pop(k, None)
    else:
        for k in ENV_REAR_KEYS:
            os.environ.pop(k, None)
    # Применяем к менеджеру
    try:
        from .camera import CameraManager

        mgr = manager or CameraManager.get_instance()
        mgr.reconfigure(normalized)
        cameras = mgr.list_cameras()
    except Exception:
        cameras = []
    return {"ok": True, "settings": normalized, "cameras": cameras, "status": "применено"}
