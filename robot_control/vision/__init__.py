"""Распознавание дорожных знаков и светофора по четырём бортовым камерам.

Пакет не открывает железо сам: кадры берутся у :class:`~robot_control.camera.CameraManager`.
Есть два взаимозаменяемых источника детекций:

* ``YoloDetector`` — обученная модель ``models/vision/yolo-stop.pt`` (Ultralytics);
* ``HeuristicDetector`` — встроенный детектор по цвету/форме, работает без PyTorch.

Оба возвращают один и тот же список :class:`Detection`, поэтому экран, поле карты
и API не зависят от того, установлен ли YOLO на конкретном роботе.
"""

from .classes import (  # noqa: F401
    CLASS_INFO,
    CLASS_NAMES,
    TRAFFIC_LIGHT_LABELS,
    TRAFFIC_LIGHT_STATES,
    class_from_id,
    class_id,
)
from .detector import Detection, HeuristicDetector  # noqa: F401
from .service import VisionService, get_vision_service  # noqa: F401
from .traffic import TrafficLightTracker, classify_traffic_light  # noqa: F401

__all__ = [
    "CLASS_INFO", "CLASS_NAMES", "TRAFFIC_LIGHT_LABELS", "TRAFFIC_LIGHT_STATES",
    "class_from_id", "class_id", "Detection", "HeuristicDetector",
    "VisionService", "get_vision_service", "TrafficLightTracker",
    "classify_traffic_light",
]
