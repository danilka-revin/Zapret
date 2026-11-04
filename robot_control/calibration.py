"""Мастер калибровки геометрии сенсоров.

Углы и положение 4 × Logitech C930e и Kinect 360 нельзя узнать из кода: их
измеряют на собранном роботе. Этот модуль даёт UI список полей с текущими
значениями и безопасно сохраняет введённые числа в ``.env`` (атомарно, не
трогая остальные строки), после чего подсистемы сразу видят новую геометрию
через ``os.environ``. Порты камер настраиваются отдельно — ``camera_settings``.

Никакого железа модуль не открывает; проверка диапазонов не даёт опечатке
(например, yaw 2500°) испортить карту.
"""
from __future__ import annotations

import math
import os
import secrets
import tempfile
from pathlib import Path
from typing import Any

from . import sensor_layout as layout
from .config import PROJECT_ROOT

# Поля, которые оператор заполняет после замеров. Порядок = порядок в форме.
CAMERA_FIELDS: tuple[tuple[str, str, str], ...] = (
    ("x_m", "X вперёд, м", "м"),
    ("y_m", "Y влево, м", "м"),
    ("z_m", "Высота, м", "м"),
    ("yaw_deg", "Угол вправо/влево, °", "°"),
    ("pitch_deg", "Наклон, °", "°"),
)
KINECT_FIELDS: tuple[tuple[str, str, str], ...] = (
    ("x_m", "X вперёд, м", "м"),
    ("y_m", "Y влево, м", "м"),
    ("z_m", "Высота, м", "м"),
    ("yaw_deg", "Поворот, °", "°"),
    ("pitch_deg", "Наклон, °", "°"),
    ("hfov_deg", "Горизонтальный угол, °", "°"),
    ("vfov_deg", "Вертикальный угол, °", "°"),
    ("min_range_m", "Мин. дальность, м", "м"),
    ("max_range_m", "Макс. дальность, м", "м"),
    ("min_height_m", "Мин. высота точки, м", "м"),
    ("max_height_m", "Макс. высота точки, м", "м"),
    ("scan_columns", "Лучей в карту", "шт"),
    ("max_age_s", "Свежесть кадра, с", "с"),
)


def _allowed_keys() -> dict[str, tuple[str, float, float]]:
    """env-ключ → (имя поля, минимум, максимум)."""
    out: dict[str, tuple[str, float, float]] = {}
    for field, env_key in layout.KINECT_ENV_KEYS.items():
        lo, hi = layout.FIELD_LIMITS[field]
        out[env_key] = (field, lo, hi)
    for role in layout.CAMERA_ROLES:
        keys = layout.camera_env_keys(role)
        for field in ("x_m", "y_m", "z_m", "yaw_deg", "pitch_deg"):
            lo, hi = layout.FIELD_LIMITS[field]
            out[keys[field]] = (field, lo, hi)
    return out


ALLOWED_KEYS = _allowed_keys()


def env_file_path() -> Path:
    return PROJECT_ROOT / ".env"


def _field_steps(field: str) -> float:
    if field == "scan_columns":
        return 1.0
    if field.endswith("_deg"):
        return 0.5
    return 0.01


def geometry_payload() -> dict[str, Any]:
    """Описание формы для ``GET /api/calibration/geometry``."""
    groups: list[dict[str, Any]] = []
    kinect_fields: list[dict[str, Any]] = []
    for field, label, unit in KINECT_FIELDS:
        env_key = layout.KINECT_ENV_KEYS[field]
        lo, hi = layout.FIELD_LIMITS[field]
        kinect_fields.append({
            "key": env_key, "field": field, "label": label, "unit": unit,
            "value": layout.kinect_spec()[field], "min": lo, "max": hi,
            "step": _field_steps(field),
        })
    groups.append({
        "id": "kinect", "title": "Kinect 360 · спереди по центру",
        "fields": kinect_fields,
    })
    specs = {c["role"]: c for c in layout.camera_specs()}
    for role in layout.CAMERA_ROLES:
        camera = specs[role]
        keys = layout.camera_env_keys(role)
        fields: list[dict[str, Any]] = []
        for field, label, unit in CAMERA_FIELDS:
            env_key = keys[field]
            lo, hi = layout.FIELD_LIMITS[field]
            fields.append({
                "key": env_key, "field": field, "label": label, "unit": unit,
                "value": camera[field], "min": lo, "max": hi,
                "step": _field_steps(field),
            })
        groups.append({"id": role, "title": layout.CAMERA_LABELS[role],
                       "port": camera["port"], "fields": fields})
    return {
        "ok": True,
        "calibrated": layout.geometry_calibrated(),
        "groups": groups,
        "fieldCount": sum(len(g["fields"]) for g in groups),
        "envPath": str(env_file_path()),
        "hint": ("Замерьте углы и смещения на собранном роботе, заполните поля "
                 "и нажмите «Сохранить». Значения попадут в .env; до этого "
                 "геометрия считается неоткалиброванной."),
    }


class CalibrationError(ValueError):
    """Некорректные значения формы."""


def validate(values: Any) -> dict[str, str]:
    """Проверить и привести к строкам значения ``{ENV_KEY: число}``."""
    if not isinstance(values, dict) or not values:
        raise CalibrationError("нужны значения хотя бы одного поля")
    clean: dict[str, str] = {}
    for key, raw in values.items():
        if key not in ALLOWED_KEYS:
            raise CalibrationError("неизвестный параметр: %s" % key)
        field, lo, hi = ALLOWED_KEYS[key]
        if isinstance(raw, bool):
            raise CalibrationError("%s: ожидается число" % key)
        try:
            number = float(raw)
        except (TypeError, ValueError):
            raise CalibrationError("%s: ожидается число" % key) from None
        if not math.isfinite(number) or not lo <= number <= hi:
            raise CalibrationError("%s: допустимо %.2f…%.2f" % (key, lo, hi))
        if field == "scan_columns":
            clean[key] = str(int(round(number)))
        else:
            clean[key] = ("%g" % number)
    return clean


def _write_env(values: dict[str, str], target: Path) -> None:
    """Заменить только наши ключи, сохранив остальные строки .env."""
    lines = target.read_text(encoding="utf-8").splitlines() if target.is_file() else []
    seen: set[str] = set()
    out: list[str] = []
    for raw in lines:
        stripped = raw.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            out.append(raw)
            continue
        key = stripped.split("=", 1)[0].strip()
        if key not in ALLOWED_KEYS:
            out.append(raw)
            continue
        if key in seen:
            continue
        seen.add(key)
        out.append("%s=%s" % (key, values.get(key, stripped.split("=", 1)[1].strip())))
    for key, value in values.items():
        if key not in seen:
            out.append("%s=%s" % (key, value))
    if not target.is_file() and not any(line.strip().startswith("RC_SECRET_KEY=") for line in out):
        out.insert(0, "RC_SECRET_KEY=%s" % secrets.token_urlsafe(48))
    target.parent.mkdir(parents=True, exist_ok=True)
    try:
        mode = target.stat().st_mode & 0o777 or 0o600
    except OSError:
        mode = 0o600
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


def save_geometry(values: Any, env_path: Path | None = None) -> dict[str, Any]:
    """Сохранить замеры в .env и применить их к текущему процессу."""
    clean = validate(values)
    target = Path(env_path) if env_path is not None else env_file_path()
    _write_env(clean, target)
    for key, value in clean.items():
        os.environ[key] = value
    return {
        "ok": True,
        "savedKeys": sorted(clean),
        "calibrated": layout.geometry_calibrated(),
        "envPath": str(target),
    }
