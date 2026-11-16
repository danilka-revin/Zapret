"""Ямы/кочки по Kinect и приближающиеся объекты по лидару: синтетика, без USB."""
from __future__ import annotations

import math

import pytest

from robot_control.ground_hazards import (GroundConfig, GroundHazardTracker,
                                          analyze_ground_points, ground_points_from_depth)
from robot_control.kinect import DepthFrame, depth_to_scan
from robot_control.object_tracks import ObjectTracker, cluster_scan
from robot_control.road_perception import RoadPerception
from robot_control.sensor_layout import kinect_spec


def ground_points(kind="clear"):
    points = []
    for ix in range(60):
        x = 0.5 + ix * 0.05
        for iy in range(-20, 21):
            y = iy * 0.05
            z = 0.0
            if kind == "pothole" and 1.5 <= x <= 1.8 and -0.2 <= y <= 0.2:
                z = -0.08
            if kind == "bump" and 1.5 <= x <= 1.8 and -0.2 <= y <= 0.2:
                z = 0.06
            points.append({"x": x, "y": y, "z": z})
    return points


def test_ground_classifier_separates_pothole_bump_and_unknown():
    clear = analyze_ground_points(ground_points("clear"))
    assert clear["status"] == "ok" and clear["hazards"] == []
    pothole = analyze_ground_points(ground_points("pothole"))
    assert pothole["status"] == "ok"
    assert any(h["kind"] == "pothole" for h in pothole["hazards"])
    bump = analyze_ground_points(ground_points("bump"))
    assert any(h["kind"] == "bump" for h in bump["hazards"])
    unknown = analyze_ground_points([])
    assert unknown["status"] == "unknown"
    assert unknown["hazards"] == []


def test_ground_hazard_requires_multiple_frames():
    tracker = GroundHazardTracker(GroundConfig(confirm_frames=3))
    analysis = analyze_ground_points(ground_points("pothole"))
    first = tracker.update(analysis, 1.0)
    second = tracker.update(analysis, 2.0)
    third = tracker.update(analysis, 3.0)
    assert first["confirmedCount"] == 0
    assert second["confirmedCount"] == 0
    assert third["confirmedCount"] >= 1
    assert third["hazards"][0]["kind"] == "pothole"


def test_depth_converter_keeps_floor_unlike_obstacle_scan():
    spec = kinect_spec()
    spec["pitch_deg"] = 30.0
    frame = DepthFrame(width=8, height=8, values=[1.0] * 64, stamp=1.0)
    points = ground_points_from_depth(frame, spec)
    assert points
    assert any(p["z"] < spec["min_height_m"] for p in points)
    scan = depth_to_scan(frame, spec)
    assert all(p["r"] >= spec["min_range_m"] for p in scan)


def test_lidar_clusters_and_stationary_wall_is_not_moving():
    points = [{"a": 0.0, "r": 5.0}, {"a": 0.02, "r": 5.0}, {"a": -0.02, "r": 5.0}]
    clusters = cluster_scan(points)
    assert len(clusters) == 1
    tracker = ObjectTracker()
    tracker.update(points, 1.0, ego_speed_mps=0.0)
    tracker.update([{"a": 0.0, "r": 4.5}, {"a": 0.02, "r": 4.5}, {"a": -0.02, "r": 4.5}], 2.0, ego_speed_mps=0.5)
    result = tracker.update([{"a": 0.0, "r": 4.0}, {"a": 0.02, "r": 4.0}, {"a": -0.02, "r": 4.0}], 3.0, ego_speed_mps=0.5)
    assert result["tracks"][0]["moving"] is False
    assert result["stopRecommended"] is False


def test_approaching_object_triggers_stop_recommendation():
    tracker = ObjectTracker(gate_m=1.5)
    for stamp, radius in ((1.0, 5.0), (2.0, 4.0), (3.0, 3.0), (4.0, 2.0)):
        points = [{"a": 0.0, "r": radius}, {"a": 0.02, "r": radius}, {"a": -0.02, "r": radius}]
        result = tracker.update(points, stamp)
    assert result["tracks"][0]["moving"] is True
    assert result["stopRecommended"] is True
    assert result["riskyTracks"][0]["ttcS"] <= tracker.stop_ttc_s


def test_perception_unknown_and_stale_are_not_free():
    perception = RoadPerception()
    state = perception.state(now=10.0)
    assert state["groundFresh"] is False
    assert any(r["reason"] == "ground_unknown" for r in state["recommendations"])
    perception.feed_ground_points(ground_points("clear"), stamp=10.0)
    fresh = perception.state(now=10.1)
    assert fresh["groundFresh"] is True
    stale = perception.state(now=12.0)
    assert stale["groundFresh"] is False
    assert stale["ground"]["reason"] == "ground_stale"
