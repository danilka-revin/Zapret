"""Lidar mounting transform shared by the live scan, mapping and navigation."""
import math
import shutil
import subprocess
import time
from pathlib import Path

import pytest

from robot_control.lidar import LidarManager, lidar_mount_yaw_deg
from robot_control.slam_map import MapSettings, SlamMapService


def test_serial_and_ros_transform_only_once_and_peer_is_already_body_frame(monkeypatch):
    monkeypatch.setenv('RC_LIDAR_YAW_DEG', '180')
    raw = [{'a': math.pi, 'r': 2.0}, {'a': 0.0, 'r': 3.0},
           {'a': -math.pi / 2, 'r': 1.0}]
    serial = LidarManager()
    serial._store_scan(raw, 'serial:ttyUSB0')
    body = serial.get_scan()
    assert body[0]['a'] == pytest.approx(0, abs=0.0001)
    assert abs(body[1]['a']) == pytest.approx(math.pi, abs=0.0001)
    assert body[2]['a'] == pytest.approx(math.pi / 2, abs=0.0001)
    assert raw[0]['a'] == math.pi
    ros = LidarManager()
    ros.feed_scan_msg([2.0], math.pi, math.pi, 1, source_label='ros', stamp=time.time())
    assert ros.get_scan()[0]['a'] == pytest.approx(0, abs=0.0001)
    peer = LidarManager()
    peer.feed_points(body, source_label='peer', stamp=time.time(),
                     peer_source='serial:ttyUSB0', peer_time_trusted=True)
    assert [math.cos(p['a']) for p in peer.get_scan()] == pytest.approx(
        [math.cos(p['a']) for p in body], abs=1e-4)
    assert peer.get_scan()[0]['a'] == pytest.approx(0, abs=1e-4)
    monkeypatch.setenv('RC_LIDAR_YAW_DEG', '0')
    unrotated = LidarManager()
    unrotated._store_scan(raw, 'serial:ttyUSB0')
    assert unrotated.get_scan() == raw
    monkeypatch.setenv('RC_LIDAR_YAW_DEG', 'nan')
    assert lidar_mount_yaw_deg() == 180


def test_rotated_front_obstacle_in_grid_from_28cm_sensor(monkeypatch):
    monkeypatch.setenv('RC_LIDAR_YAW_DEG', '180')
    monkeypatch.delenv('RC_LIDAR_OFFSET_M', raising=False)
    manager = LidarManager()
    manager._store_scan([{'a': math.pi, 'r': 2}], 'serial:ttyUSB0')
    mapping = SlamMapService(MapSettings(size_m=10, resolution=0.05))
    assert mapping.ingest(manager.get_scan())
    assert mapping.navigation_snapshot()['lidarOffsetM'] == pytest.approx(.28)
    assert mapping._grid.get(*mapping._grid.cell_of(2.28, 0)) >= 65
    assert mapping._grid.get(*mapping._grid.cell_of(-1.72, 0)) == 255
    assert mapping.navigation_snapshot()['pose'] == {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    mapping._pose.pose = (-.28, .28, math.pi / 2)
    pose = mapping.navigation_snapshot()['pose']
    assert pose['x'] == pytest.approx(0, abs=1e-9)
    assert pose['y'] == pytest.approx(0, abs=1e-9)


@pytest.mark.skipif(not shutil.which('node'), reason='node required')
def test_live_view_projects_points_from_lidar_mount():
    root = Path(__file__).resolve().parent.parent
    subprocess.run(['node', 'tests/js/lidar_mount_view.js'], cwd=root, check=True, timeout=10)
