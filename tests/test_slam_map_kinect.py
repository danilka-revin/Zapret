"""Kinect 360 вписывается в общую карту, не подменяя позу лидара."""
from __future__ import annotations

import pytest

from robot_control.slam_map import SlamMapService, UNKNOWN


def test_ingest_kinect_marks_cells_without_lidar_scans():
    service = SlamMapService()
    before = service.navigation_snapshot()
    assert before["scans"] == 0 and before["trusted"] is True
    touched = service.ingest_kinect([
        {"a": 0.0, "r": 1.0}, {"a": 0.2, "r": 1.5}, {"a": -0.2, "r": 1.5},
    ], stamp=1234.0)
    after = service.navigation_snapshot()
    assert touched > 0
    assert after["scans"] == 0, "Kinect не должен считаться лидарным сканом"
    assert after["trusted"] is True
    assert after["revision"] > before["revision"]
    assert after["data"] != before["data"]
    snapshot = service.snapshot(with_png=False)
    assert snapshot["kinectScans"] == 1
    assert snapshot["kinectPoints"] == 3
    assert snapshot["kinectUpdatedAt"] == 1234.0


def test_empty_kinect_frame_changes_nothing():
    service = SlamMapService()
    before = service.navigation_snapshot()
    assert service.ingest_kinect([], stamp=1.0) == 0
    assert service.navigation_snapshot()["revision"] == before["revision"]


def test_kinect_pose_follows_mount_offset(monkeypatch):
    """Две разные точки монтажа дают разные занятые клетки."""
    monkeypatch.setenv("RC_KINECT_X_M", "0.35")
    left = SlamMapService()
    left.ingest_kinect([{"a": 0.0, "r": 0.8}], stamp=1.0)
    monkeypatch.setenv("RC_KINECT_X_M", "0.90")
    right = SlamMapService()
    right.ingest_kinect([{"a": 0.0, "r": 0.8}], stamp=1.0)
    a = left.navigation_snapshot()["data"]
    b = right.navigation_snapshot()["data"]
    assert a != b
    assert sum(1 for v in a if v != UNKNOWN) > 0
    assert sum(1 for v in b if v != UNKNOWN) > 0


def test_reset_clears_kinect_counters():
    service = SlamMapService()
    service.ingest_kinect([{"a": 0.0, "r": 1.0}], stamp=1.0)
    assert service.snapshot(with_png=False)["kinectScans"] == 1
    service.reset(save=False)
    payload = service.snapshot(with_png=False)
    assert payload["kinectScans"] == 0
    assert payload["kinectPoints"] == 0
    assert payload["kinectUpdatedAt"] == 0.0
