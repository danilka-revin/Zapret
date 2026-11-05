"""Геометрия сенсоров робота: 4 × Logitech C930e и Kinect 360.

Схема установки (вид сверху, ось X — вперёд, ось Y — влево, как на карте
лидара): две камеры спереди по бокам и две сзади по бокам, Kinect 360 по
центру спереди. Углы и положение **не зашиты в код**: всё читается из `.env`,
шаблон — в `.env.example`, подробности — в `docs/AUTONOMY.md`.

Значения по умолчанию грубо повторяют эскиз (передняя пара 50° между лучами,
задняя 90°), но это только отправная точка: после монтажа замерьте углы и
впишите свои `RC_CAMERA_*_YAW_DEG`, `RC_KINECT_*`. Пока переменные не заданы,
`layout_payload()` честно сообщает ``calibrated=False`` — карта при этом
строится только лидаром, а камеры работают как круговой обзор.

Никакого железа модуль не открывает: только числа для остальных подсистем.
"""
from __future__ import annotations

import math
import os
from typing import Any

# Роль → числовой id камеры. 0 и 1 сохранены за старыми «передняя/задняя»,
# чтобы /api/camera/front, /api/camera/rear и прежние экраны не сломались.
CAMERA_IDS: dict[str, int] = {
    "front_left": 0,
    "rear_left": 1,
    "front_right": 2,
    "rear_right": 3,
}
CAMERA_ROLES: tuple[str, ...] = tuple(CAMERA_IDS)

CAMERA_LABELS: dict[str, str] = {
    "front_left": "Передняя левая (FL)",
    "front_right": "Передняя правая (FR)",
    "rear_left": "Задняя левая (RL)",
    "rear_right": "Задняя правая (RR)",
}

# Основные ключи .env и алиасы. FL/RL наследуют старые RC_CAMERA_FRONT/REAR.
CAMERA_ENV_KEYS: dict[str, tuple[str, ...]] = {
    "front_left": ("RC_CAMERA_FL_PORT", "RC_CAMERA_FRONT", "RC_CAMERA_FRONT_PORT", "RC_CAM_FRONT"),
    "rear_left": ("RC_CAMERA_RL_PORT", "RC_CAMERA_REAR", "RC_CAMERA_REAR_PORT", "RC_CAM_REAR"),
    "front_right": ("RC_CAMERA_FR_PORT", "RC_CAMERA_FRONT_RIGHT", "RC_CAMERA_FRONT_RIGHT_PORT"),
    "rear_right": ("RC_CAMERA_RR_PORT", "RC_CAMERA_REAR_RIGHT", "RC_CAMERA_REAR_RIGHT_PORT"),
}

# Logitech C930e: 90° по диагонали, ~82° по горизонтали при 16:9.
C930E_HFOV_DEG = 82.0
C930E_VFOV_DEG = 52.0

# Грубые значения по эскизу. Y>0 — влево, yaw>0 — поворот налево.
_CAMERA_DEFAULTS: dict[str, dict[str, Any]] = {
    "front_left": {"port": "/dev/video0", "x_m": 0.30, "y_m": 0.25,
                   "z_m": 0.28, "yaw_deg": -25.0, "pitch_deg": 0.0},
    "front_right": {"port": "/dev/video2", "x_m": 0.30, "y_m": -0.25,
                    "z_m": 0.28, "yaw_deg": 25.0, "pitch_deg": 0.0},
    "rear_left": {"port": "/dev/video1", "x_m": -0.30, "y_m": 0.25,
                  "z_m": 0.28, "yaw_deg": -135.0, "pitch_deg": 0.0},
    "rear_right": {"port": "/dev/video3", "x_m": -0.30, "y_m": -0.25,
                   "z_m": 0.28, "yaw_deg": 135.0, "pitch_deg": 0.0},
}

_KINECT_DEFAULTS: dict[str, Any] = {
    # Положение и ориентация на корпусе.
    "x_m": 0.35, "y_m": 0.0, "z_m": 0.30,
    "yaw_deg": 0.0, "pitch_deg": 0.0,
    # Kinect 360: 57° × 43°, рабочий диапазон глубины 0,45…4,0 м.
    "hfov_deg": 57.0, "vfov_deg": 43.0,
    "min_range_m": 0.45, "max_range_m": 4.0,
    # Пол и потолок в карту не пишем; препятствия ниже 12 см тоже не видим.
    "min_height_m": 0.12, "max_height_m": 1.60,
    # Сколько угловых секторов отдаём в карту (около 0,7° на луч при 80).
    "scan_columns": 80,
    "max_age_s": 1.5,
}

KINECT_ENV_KEYS: dict[str, str] = {
    "x_m": "RC_KINECT_X_M",
    "y_m": "RC_KINECT_Y_M",
    "z_m": "RC_KINECT_Z_M",
    "yaw_deg": "RC_KINECT_YAW_DEG",
    "pitch_deg": "RC_KINECT_PITCH_DEG",
    "hfov_deg": "RC_KINECT_HFOV_DEG",
    "vfov_deg": "RC_KINECT_VFOV_DEG",
    "min_range_m": "RC_KINECT_MIN_RANGE_M",
    "max_range_m": "RC_KINECT_MAX_RANGE_M",
    "min_height_m": "RC_KINECT_MIN_HEIGHT_M",
    "max_height_m": "RC_KINECT_MAX_HEIGHT_M",
    "scan_columns": "RC_KINECT_SCAN_COLUMNS",
    "max_age_s": "RC_KINECT_MAX_AGE_S",
}

# Допустимые диапазоны: не даём опечатке в .env превратить робота в телегу.
_LIMITS: dict[str, tuple[float, float]] = {
    "x_m": (-2.0, 2.0), "y_m": (-2.0, 2.0), "z_m": (0.0, 2.0),
    "yaw_deg": (-360.0, 360.0), "pitch_deg": (-90.0, 90.0),
    "hfov_deg": (10.0, 170.0), "vfov_deg": (10.0, 140.0),
    "min_range_m": (0.05, 2.0), "max_range_m": (0.5, 10.0),
    "min_height_m": (0.0, 1.0), "max_height_m": (0.2, 3.0),
    "scan_columns": (16, 320), "max_age_s": (0.2, 10.0),
}


# Публичная копия для мастера калибровки (robot_control/calibration.py).
FIELD_LIMITS: dict[str, tuple[float, float]] = dict(_LIMITS)


def _env_float(name: str, default: float, lo: float, hi: float) -> float:
    raw = (os.environ.get(name) or "").strip()
    if not raw:
        return float(default)
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return float(default)
    if not math.isfinite(value):
        return float(default)
    return max(lo, min(hi, value))


def _env_port(role: str, default: str) -> str:
    for key in CAMERA_ENV_KEYS[role]:
        value = (os.environ.get(key) or "").strip()
        if value:
            return value
    return default


def _env_is_set(*keys: str) -> bool:
    """True только когда заданы все перечисленные переменные.

    Раньше здесь было ``any``: одного ``RC_KINECT_X_M`` хватало, чтобы экран
    объявил геометрию откалиброванной, хотя углы камер ещё не измерены.
    """
    return all(bool((os.environ.get(key) or "").strip()) for key in keys)


def camera_specs() -> list[dict[str, Any]]:
    """Четыре камеры в порядке числовых id (0…3)."""
    specs: list[dict[str, Any]] = []
    for role, cam_id in sorted(CAMERA_IDS.items(), key=lambda item: item[1]):
        defaults = _CAMERA_DEFAULTS[role]
        yaw_key = f"RC_CAMERA_{_role_suffix(role)}_YAW_DEG"
        pitch_key = f"RC_CAMERA_{_role_suffix(role)}_PITCH_DEG"
        x_key = f"RC_CAMERA_{_role_suffix(role)}_X_M"
        y_key = f"RC_CAMERA_{_role_suffix(role)}_Y_M"
        z_key = f"RC_CAMERA_{_role_suffix(role)}_Z_M"
        specs.append({
            "id": cam_id,
            "role": role,
            "label": CAMERA_LABELS[role],
            "port": _env_port(role, ""),  # только ручное назначение: без порта камера не открывается
            "x_m": _env_float(x_key, float(defaults["x_m"]), -2.0, 2.0),
            "y_m": _env_float(y_key, float(defaults["y_m"]), -2.0, 2.0),
            "z_m": _env_float(z_key, float(defaults["z_m"]), 0.0, 2.0),
            "yaw_deg": _env_float(yaw_key, float(defaults["yaw_deg"]), -360.0, 360.0),
            "pitch_deg": _env_float(pitch_key, float(defaults["pitch_deg"]), -90.0, 90.0),
            "hfov_deg": C930E_HFOV_DEG,
            "vfov_deg": C930E_VFOV_DEG,
            "model": "Logitech C930e",
        })
    return specs


def _role_suffix(role: str) -> str:
    return {
        "front_left": "FL", "front_right": "FR",
        "rear_left": "RL", "rear_right": "RR",
    }[role]


def camera_env_keys(role: str) -> dict[str, str]:
    """Имена переменных геометрии одной камеры — для UI и документации."""
    suffix = _role_suffix(role)
    return {
        "port": CAMERA_ENV_KEYS[role][0],
        "x_m": f"RC_CAMERA_{suffix}_X_M",
        "y_m": f"RC_CAMERA_{suffix}_Y_M",
        "z_m": f"RC_CAMERA_{suffix}_Z_M",
        "yaw_deg": f"RC_CAMERA_{suffix}_YAW_DEG",
        "pitch_deg": f"RC_CAMERA_{suffix}_PITCH_DEG",
    }


# Второй Kinect 360 сзади по центру — опция. В исходном описании сзади были
# только две C930e, поэтому по умолчанию выключен; если он всё же стоит,
# достаточно RC_KINECT_REAR_ENABLE=1 и своих замеров ниже.
REAR_KINECT_ENABLE_KEY = "RC_KINECT_REAR_ENABLE"
KINECT_REAR_ENV_KEYS: dict[str, str] = {
    field: key.replace("RC_KINECT_", "RC_KINECT_REAR_", 1)
    for field, key in KINECT_ENV_KEYS.items()
}
_KINECT_REAR_DEFAULTS = dict(_KINECT_DEFAULTS, x_m=-0.35, yaw_deg=180.0, pitch_deg=0.0)


def rear_kinect_enabled() -> bool:
    return (os.environ.get(REAR_KINECT_ENABLE_KEY) or "").strip().lower() in {
        "1", "true", "yes", "on"}


def kinect_spec(unit: str = "front") -> dict[str, Any]:
    """Настройки Kinect 360: монтаж, оптика, фильтры и число лучей.

    ``unit='rear'`` — опциональный датчик сзади; у него свои переменные .env
    и свой курс (по умолчанию 180°), но та же математика псевдоскана.
    """
    if unit == "rear":
        env_keys, defaults = KINECT_REAR_ENV_KEYS, _KINECT_REAR_DEFAULTS
    elif unit == "front":
        env_keys, defaults = KINECT_ENV_KEYS, _KINECT_DEFAULTS
    else:
        raise ValueError("неизвестный Kinect: %s" % unit)
    spec = {
        key: _env_float(env_keys[key], float(defaults[key]), *_LIMITS[key])
        for key in env_keys
    }
    spec["scan_columns"] = int(spec["scan_columns"])
    spec["model"] = "Kinect 360"
    # Защита от несовместимых значений в .env.
    if spec["max_range_m"] <= spec["min_range_m"]:
        spec["max_range_m"] = spec["min_range_m"] + 0.5
    if spec["max_height_m"] <= spec["min_height_m"]:
        spec["max_height_m"] = spec["min_height_m"] + 0.5
    return spec


def kinect_specs() -> dict[str, dict[str, Any]]:
    """Активные датчики глубины: передний всегда, задний — по флагу .env."""
    units = {"front": kinect_spec("front")}
    if rear_kinect_enabled():
        units["rear"] = kinect_spec("rear")
    return units


def geometry_calibrated() -> bool:
    """Вписал ли оператор углы/положение сенсоров в .env.

    Пока ``False``, карту строит только проверенный лидар; глубина Kinect
    всё равно может приходить, но в документации это помечено как «до калибровки».
    """
    keys: list[str] = []
    for role in CAMERA_ROLES:
        keys.append(camera_env_keys(role)["yaw_deg"])
    keys.extend((KINECT_ENV_KEYS["x_m"], KINECT_ENV_KEYS["yaw_deg"],
                 KINECT_ENV_KEYS["hfov_deg"]))
    return _env_is_set(*keys)


def layout_payload() -> dict[str, Any]:
    """Снимок геометрии для ``GET /api/sensors/layout`` и страницы карты."""
    cameras = camera_specs()
    ports = [str(c["port"]) for c in cameras]
    return {
        "ok": True,
        "calibrated": geometry_calibrated(),
        "cameraCount": len(cameras),
        "cameras": cameras,
        "kinect": kinect_spec("front"),
        "kinects": kinect_specs(),
        "rearKinectEnabled": rear_kinect_enabled(),
        "rearKinectEnvKeys": dict(KINECT_REAR_ENV_KEYS),
        "rearKinectEnableKey": REAR_KINECT_ENABLE_KEY,
        "cameraEnvKeys": {role: camera_env_keys(role) for role in CAMERA_ROLES},
        "kinectEnvKeys": dict(KINECT_ENV_KEYS),
        # Пустой порт = «не назначен», это не дубль.
        "duplicatePorts": sorted({p for p in ports if p and ports.count(p) > 1}),
    }
