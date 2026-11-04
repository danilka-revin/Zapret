"""Глубина Kinect 360 → псевдоскан: математика без железа."""
from __future__ import annotations

import array
import base64
import math
import time

import pytest

from robot_control.kinect import (
    DepthError, DepthFrame, KinectDepthService, decode_depth_payload,
    depth_to_scan, points_from_payload,
)
from robot_control.sensor_layout import kinect_spec


def demo_payload(width=80, height=60, wall_mm=2000):
    samples = array.array("H", [0]) * (width * height)
    for y in range(height):
        for x in range(width):
            if height // 3 < y < height * 2 // 3 and width // 4 < x < width * 3 // 4:
                samples[y * width + x] = wall_mm
            elif y > height * 2 // 3:
                samples[y * width + x] = 1200   # «пол» — должен отфильтроваться
    return {
        "width": width, "height": height,
        "depth": base64.b64encode(samples.tobytes()).decode("ascii"),
        "units": "mm", "stamp": 1000.0,
    }


def test_decode_and_reject_bad_frames():
    frame = decode_depth_payload(demo_payload())
    assert frame.width == 80 and frame.height == 60 and len(frame.values) == 4800
    assert frame.values[30 * 80 + 40] == pytest.approx(2.0)
    with pytest.raises(DepthError):
        decode_depth_payload({"width": 4, "height": 4, "depth": "!!!!"})
    with pytest.raises(DepthError):
        decode_depth_payload({"width": 4, "height": 4, "depth": base64.b64encode(b"\x00" * 8).decode()})
    with pytest.raises(DepthError):
        decode_depth_payload({"width": 4, "height": 4, "depth": [1] * 16, "units": "parsecs"})


def test_depth_to_scan_wall_in_front_and_floor_filtered():
    frame = decode_depth_payload(demo_payload())
    points = depth_to_scan(frame)
    assert points, "стена впереди должна дать лучи"
    assert all(-math.radians(30) <= p["a"] <= math.radians(30) for p in points)
    assert all(1.8 <= p["r"] <= 2.2 for p in points), points
    assert all(abs(p["r"] - 1.2) > 0.3 for p in points), "пол не должен попасть в карту"
    assert points == sorted(points, key=lambda p: p["a"])


def test_yaw_and_pitch_change_angles_and_height_filter():
    frame = decode_depth_payload(demo_payload())
    spec = dict(kinect_spec())
    spec["yaw_deg"] = 90.0
    turned = depth_to_scan(frame, spec)
    assert turned
    assert all(math.radians(60) < p["a"] < math.radians(120) for p in turned)
    # Сильный наклон вниз уводит стену ниже порога высоты → лучей нет.
    spec["yaw_deg"] = 0.0
    spec["pitch_deg"] = 35.0
    assert depth_to_scan(frame, spec) == []


def test_range_and_point_payloads():
    ranges = [2.0] * 41
    points, stamp, source = points_from_payload({
        "ranges": ranges, "angle_min": -0.5, "angle_max": 0.5, "stamp": 12.0})
    assert source == "ranges" and stamp == 12.0
    assert len(points) == 41
    assert points[0]["a"] == pytest.approx(-0.5, abs=0.01)
    points, _stamp, source = points_from_payload({
        "points": [{"a": 0.1, "r": 1.5}, {"a": 0.2, "r": 0.01}, "oops"], "stamp": 13.0})
    assert source == "points" and len(points) == 1
    with pytest.raises(DepthError):
        points_from_payload({"ranges": [1, 2], "angle_min": 1, "angle_max": 0})


def test_service_consumes_each_frame_once_and_reports_freshness():
    service = KinectDepthService()
    assert service.status()["connected"] is False
    assert service.fresh() is False
    result = service.feed(demo_payload())
    assert result["ok"] and result["points"] > 0
    assert service.status()["connected"] is True
    first = service.consume()
    assert first and first[0]
    assert service.consume() is None, "один кадр не должен вписываться дважды"
    service.reset()
    assert service.fresh() is False


def test_stale_depth_is_not_fresh():
    clock = {"now": time.time()}
    service = KinectDepthService(clock=lambda: clock["now"])
    payload = demo_payload()
    payload["stamp"] = clock["now"]
    service.feed(payload)
    assert service.fresh() is True
    clock["now"] += 10.0
    assert service.fresh() is False
    assert service.status()["connected"] is False
