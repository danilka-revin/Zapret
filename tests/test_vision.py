"""Распознавание знаков, цвета светофора, отметки поля и API /api/vision/*."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from robot_control.vision.classes import CLASS_NAMES, class_from_id, class_id
from robot_control.vision.detector import Detection, HeuristicDetector, non_max_suppression
from robot_control.vision.service import VisionConfig, VisionService
from robot_control.vision.traffic import (
    TrafficLightTracker, classify_traffic_light, color_counts,
)

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

needs_cv2 = pytest.mark.skipif(cv2 is None, reason="нужен opencv-python-headless")


# ------------------------------------------------------------------ классы
def test_class_order_is_stable():
    assert CLASS_NAMES == ("stop_sign", "pedestrian_crossing", "speed_bump",
                           "traffic_light", "cone")
    for index, name in enumerate(CLASS_NAMES):
        assert class_id(name) == index
        assert class_from_id(index) == name
    with pytest.raises(ValueError):
        class_id("unknown_sign")


def test_sign_scale_is_configurable(monkeypatch):
    monkeypatch.setenv("RC_VISION_SIGN_SCALE", "1.2")
    assert VisionConfig.from_env().sign_scale == 1.2
    monkeypatch.setenv("RC_VISION_SIGN_SCALE", "999")
    assert VisionConfig.from_env().sign_scale == 4.0  # защита от опечатки


def test_detection_box_clamped_and_yolo_line():
    det = Detection("stop_sign", (0.9, 0.8, 0.5, 0.5), 0.9)
    assert det.box == (0.9, 0.8, 0.1, 0.2)
    assert det.yolo_line() == "0 0.950000 0.900000 0.100000 0.200000"
    payload = det.to_dict()
    assert payload["label"] == "Знак «Стоп»" and payload["action"] == "stop"


def test_nms_keeps_the_strongest_overlapping_box():
    weak = Detection("stop_sign", (0.10, 0.10, 0.20, 0.20), 0.55)
    strong = Detection("stop_sign", (0.12, 0.12, 0.20, 0.20), 0.90)
    other = Detection("cone", (0.12, 0.12, 0.20, 0.20), 0.60)
    kept = non_max_suppression([weak, strong, other])
    assert strong in kept and other in kept and weak not in kept


# ---------------------------------------------------------------- светофор
@pytest.mark.parametrize("bgr,expected", [
    ((30, 30, 240), "red"),
    ((40, 230, 250), "yellow"),
    ((40, 210, 60), "green"),
])
def test_traffic_light_color_from_pixels(bgr, expected):
    frame = np.full((24, 24, 3), bgr, dtype=np.uint8)
    counts = color_counts(frame)
    assert counts[expected] > 0
    assert sum(counts.values()) == 24 * 24
    result = classify_traffic_light(frame)
    assert result["state"] == expected
    assert result["confidence"] > 0.9


def test_traffic_light_no_signal_on_dark_frame():
    frame = np.full((20, 20, 3), (15, 15, 15), dtype=np.uint8)
    result = classify_traffic_light(frame)
    assert result["state"] == "unknown"
    assert result["confidence"] == 0.0


def test_tracker_hysteresis_and_stale_state():
    tracker = TrafficLightTracker(window=5, switch_votes=2, stale_s=1.0)
    assert tracker.update("red", 0.9, 0, stamp=0.0)["state"] == "red"
    # Один кадр зелёного не должен переключить решение.
    assert tracker.update("green", 0.9, 0, stamp=0.1)["state"] == "red"
    assert tracker.update("green", 0.9, 0, stamp=0.2)["state"] == "green"
    assert tracker.snapshot(now=0.3)["action"] == "go"
    assert tracker.snapshot(now=2.5)["state"] == "unknown"


# --------------------------------------------------------------- детектор
@needs_cv2
@pytest.mark.parametrize("bgr,expected", [
    ((40, 40, 240), "red"),
    ((40, 220, 250), "yellow"),
    ((40, 210, 70), "green"),
])
def test_heuristic_finds_led_panel_and_its_color(bgr, expected):
    """Синтетическое тёмное табло со светодиодами: рамка + цвет сигнала."""
    frame = np.full((400, 400, 3), (18, 18, 18), dtype=np.uint8)
    for row in range(8):
        for col in range(12):
            x, y = 140 + col * 10, 120 + row * 16
            cv2.rectangle(frame, (x, y), (x + 4, y + 4), bgr, -1)
    detections = HeuristicDetector().detect(frame)
    lights = [d for d in detections if d.cls == "traffic_light"]
    assert lights, "табло со светодиодами должно находиться"
    assert lights[0].color == expected, lights[0].color_counts
    assert 0.2 < lights[0].box[2] < 0.9 and 0.2 < lights[0].box[3] < 0.9


@needs_cv2
def test_heuristic_on_real_dataset_frame():
    """Если кадры скачаны, STOP и светофор находятся и на настоящем фото."""
    photo = Path(__file__).resolve().parents[1] / "datasets" / "yolo-stop" / "raw" / "1794903271329.jpg"
    if not photo.is_file():
        pytest.skip("датасет не скачан: scripts/vision_dataset.py")
    frame = cv2.imread(str(photo))
    detections = HeuristicDetector().detect(frame)
    classes = {d.cls for d in detections}
    assert "stop_sign" in classes and "traffic_light" in classes
    assert any(d.color in ("red", "yellow", "green") for d in detections)


# ----------------------------------------------------------------- сервис
class _FakeDetector:
    def __init__(self):
        self.signal = "green"

    def detect(self, _frame):
        return [
            Detection("stop_sign", (0.40, 0.42, 0.12, 0.18), 0.91),
            Detection("traffic_light", (0.52, 0.25, 0.14, 0.30), 0.88,
                      self.signal, {"red": 0, "yellow": 0, "green": 420}),
        ]


@needs_cv2
def test_service_state_field_and_annotated_frame():
    config = VisionConfig(enabled=True, model_path=Path("/definitely-missing.pt"),
                          traffic_stale_s=5.0)
    service = VisionService(config=config,
                            camera_provider=lambda: None,
                            layout_provider=lambda: [{
                                "id": 0, "role": "front_left", "x_m": 0.3, "y_m": 0.25,
                                "yaw_deg": -25.0, "hfov_deg": 82.0}])
    service.initialize()
    assert service.backend == "heuristic"
    service._detector = _FakeDetector()
    frame = np.full((360, 640, 3), (30, 30, 30), dtype=np.uint8)
    ok, jpeg = cv2.imencode(".jpg", frame)
    assert ok
    service.process_frame(0, jpeg.tobytes())
    state = service.state()
    assert state["ok"] and state["enabled"]
    assert state["counts"]["stop_sign"] == 1
    assert state["trafficLight"]["state"] == "green"
    assert state["trafficLight"]["counts"]["green"] == 420
    assert len(state["field"]) == 2  # конусы на поле не отмечаются
    marker = next(m for m in state["field"] if m["class"] == "stop_sign")
    assert marker["cameraId"] == 0 and 0.3 <= marker["distanceM"] <= 6.0
    assert marker["action"] == "stop"
    # 3D-обзор Kinect: физический размер 1:1, высота и ориентация таблички.
    assert marker["width_m"] == 0.3 and marker["height_m"] == 0.3
    assert marker["thickness_m"] > 0 and marker["shape"] == "octagon"
    assert 0.15 <= marker["z_m"] <= 2.2 and marker["centerZ_m"] == marker["z_m"]
    assert -180.0 <= marker["headingDeg"] <= 180.0
    assert marker["scale1to1"] is True and marker["sizeScale"] == 1.0
    assert marker["positionSource"] == "camera-estimate"
    annotated = service.annotated_frame(0)
    assert annotated and annotated[:2] == b"\xff\xd8"

    # A slow inference finishing now cannot turn an old exposure into a live frame.
    service.process_frame(0, jpeg.tobytes(), stamp=service._cameras[0].stamp - 3)
    assert service.annotated_frame(0) is None

    off = service.configure(enabled=False)
    assert off["enabled"] is False and off["field"] == []
    assert service.annotated_frame(3) is None


def test_js_panel_and_camera_url_swap():
    """Node-тест панели YOLO (если в системе есть node)."""
    import shutil
    import subprocess

    node = shutil.which("node")
    if not node:
        pytest.skip("нужен node для проверки фронтенда")
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [node, str(root / "tests" / "js" / "vision_panel.js")],
        cwd=root, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "vision panel: OK" in result.stdout


def test_camera_pollers_use_vision_when_available():
    """Оба экрана камер умеют брать размеченный кадр у RCVision."""
    for name in ("slam_gui/camera_frames.js", "slam_gui/camera_view.js"):
        source = (Path(__file__).resolve().parents[1] / name).read_text(encoding="utf-8")
        assert "RCVision.loadFrame" in source
        assert "api/camera/" in source


@needs_cv2
def test_api_routes(tmp_path):
    from flask import Flask
    from robot_control.vision.api import register_vision_routes

    config = VisionConfig(enabled=True, model_path=Path("/definitely-missing.pt"),
                          traffic_stale_s=5.0)
    service = VisionService(config=config, camera_provider=lambda: None,
                            layout_provider=lambda: [])
    service.initialize()
    service._detector = _FakeDetector()
    app = Flask(__name__)
    app.config.update(TESTING=True)
    register_vision_routes(app, service=service, autostart=False)
    client = app.test_client()

    assert client.get("/api/vision/classes").get_json()["classes"][0]["name"] == "stop_sign"
    assert client.get("/api/vision/state").get_json()["backend"] == "heuristic"
    assert client.get("/api/vision/field").get_json()["field"] == []
    assert client.get("/api/vision/camera/0/frame").status_code == 404

    frame = np.full((120, 160, 3), (25, 25, 25), dtype=np.uint8)
    ok, jpeg = cv2.imencode(".jpg", frame)
    assert ok
    response = client.post("/api/vision/camera/2/frame", data=jpeg.tobytes(),
                           content_type="image/jpeg")
    assert response.status_code == 200 and response.get_json()["ok"]
    assert client.get("/api/vision/camera/2/frame").mimetype == "image/jpeg"
    assert len(client.get("/api/vision/field").get_json()["field"]) == 2

    changed = client.post("/api/vision/config", json={"confidence": 0.6, "enabled": True})
    assert changed.status_code == 200
    assert changed.get_json()["intervalS"] == 0.25
    assert client.post("/api/vision/config", json={"enabled": "да"}).status_code == 400
