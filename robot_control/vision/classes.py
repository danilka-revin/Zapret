"""Словарь классов полигона: одна точка правды для обучения, API и экрана.

Знаки взяты из техзадания («Стоп», «Пешеходный переход», «Искусственная
неровность») и фотографий в ``p25543770-lang/yolo-stop``. Четвёртый класс —
светофор, пятый — ограничительные конусы/столбики. Порядок ``CLASS_NAMES`` —
публичный контракт API и старых весов. ``TRAINING_CLASS_NAMES`` расширяет его
тремя сигналами; на борту они сводятся к ``traffic_light`` + ``signal``.
"""
from __future__ import annotations

from typing import Any

CLASS_NAMES: tuple[str, ...] = (
    "stop_sign",
    "pedestrian_crossing",
    "speed_bump",
    "traffic_light",
    "cone",
)

#: Подписи, цвет рамки, действие робота и ФИЗИЧЕСКИЕ размеры знака в метрах.
#: height_m используется для оценки дальности по высоте рамки; width_m /
#: thickness_m / shape / center_height_m — для 3D-обзора Kinect в масштабе 1:1.
#: Это типовые размеры учебных макетов: перед соревнованием замерьте свои знаки
#: и поправьте константы (или RC_VISION_SIGN_SCALE), иначе 1:1 будет условным.
CLASS_INFO: dict[str, dict[str, Any]] = {
    "stop_sign": {
        "label": "Знак «Стоп»", "short": "STOP", "color": "#ff3d71",
        "action": "stop", "height_m": 0.30, "width_m": 0.30,
        "thickness_m": 0.02, "center_height_m": 0.95, "shape": "octagon",
    },
    "pedestrian_crossing": {
        "label": "Пешеходный переход", "short": "ПЕШ", "color": "#2f9bff",
        "action": "slow", "height_m": 0.35, "width_m": 0.35,
        "thickness_m": 0.02, "center_height_m": 0.95, "shape": "square",
    },
    "speed_bump": {
        "label": "Искусственная неровность", "short": "НЕРОВН", "color": "#ffb020",
        "action": "slow", "height_m": 0.35, "width_m": 0.35,
        "thickness_m": 0.02, "center_height_m": 0.95, "shape": "square",
    },
    "traffic_light": {
        "label": "Светофор", "short": "СВЕТОФОР", "color": "#a78bfa",
        "action": "signal", "height_m": 0.60, "width_m": 0.25,
        "thickness_m": 0.05, "center_height_m": 1.05, "shape": "panel",
    },
    "cone": {
        "label": "Конус / столбик", "short": "КОНУС", "color": "#ff7a29",
        "action": "avoid", "height_m": 0.70, "width_m": 0.25,
        "thickness_m": 0.25, "center_height_m": 0.35, "shape": "cone",
    },
}

TRAFFIC_LIGHT_STATES: tuple[str, ...] = ("red", "yellow", "green")
# Не переставлять первые пять id: старые датасеты/веса остаются совместимыми.
# generic traffic_light нужен для выключенного/неоднозначного табло.
TRAINING_CLASS_NAMES = CLASS_NAMES + tuple(
    f"traffic_light_{state}" for state in TRAFFIC_LIGHT_STATES
)
TRAFFIC_LIGHT_LABELS: dict[str, str] = {
    "red": "Красный — стоим",
    "yellow": "Жёлтый — готовимся",
    "green": "Зелёный — можно ехать",
    "unknown": "Сигнал не определён",
}

#: Алиасы, которые могут встретиться в ``model.names`` чужой модели.
_ALIASES: dict[str, str] = {
    "stop": "stop_sign", "stop sign": "stop_sign", "стоп": "stop_sign",
    "pedestrian": "pedestrian_crossing", "crosswalk": "pedestrian_crossing",
    "ped crossing": "pedestrian_crossing", "пешеходный переход": "pedestrian_crossing",
    "bump": "speed_bump", "speed bump": "speed_bump", "hump": "speed_bump",
    "неровность": "speed_bump", "искусственная неровность": "speed_bump",
    "light": "traffic_light", "traffic light": "traffic_light", "светофор": "traffic_light",
    "cone": "cone", "pillar": "cone", "bollard": "cone", "конус": "cone",
}


def class_id(name: str) -> int:
    """Индекс класса по имени; ``ValueError`` для неизвестного имени."""
    if name in CLASS_NAMES:
        return CLASS_NAMES.index(name)
    raise ValueError(f"неизвестный класс знака: {name!r}")


def class_from_id(index: int) -> str:
    """Имя класса по индексу; ``ValueError`` для чужого индекса."""
    if 0 <= index < len(CLASS_NAMES):
        return CLASS_NAMES[index]
    raise ValueError(f"неизвестный индекс класса: {index!r}")


def normalize_class(name: Any) -> str | None:
    """Привести имя класса из ``model.names`` к нашему каноническому имени."""
    if isinstance(name, int):
        try:
            return class_from_id(name)
        except ValueError:
            return None
    key = str(name or "").strip().lower()
    if key in CLASS_NAMES:
        return key
    if signal_from_class(key):
        return "traffic_light"
    return _ALIASES.get(key)


def signal_from_class(name: Any) -> str | None:
    """Сигнал из имени нейросетевого класса, без изменения публичных id."""
    key = str(name or "").strip().lower().replace(" ", "_").replace("-", "_")
    for state in TRAFFIC_LIGHT_STATES:
        if key in (f"traffic_light_{state}", f"{state}_light", f"{state}_traffic_light"):
            return state
    return None
