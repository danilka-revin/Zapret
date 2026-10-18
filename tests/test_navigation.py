"""Constrained SLAM-grid navigation: planner safety and endpoint ownership."""
from __future__ import annotations

import io
import json
import math
import threading
from urllib.error import HTTPError

import pytest

from conftest import login
from robot_control import map_peer
from robot_control.navigation import (
    NavigationError,
    NavigationService,
    plan_grid_path,
    steering_deviation,
)
from robot_control.slam_map import MapSettings, SlamMapService


def grid_snapshot(*, free_box=(-1.5, 2.5, -1.5, 1.5), obstacle=None):
    resolution = 0.1
    width = height = 100
    origin = (-5.0, -5.0)
    data = bytearray([255]) * (width * height)
    xmin, xmax, ymin, ymax = free_box
    for iy in range(height):
        y = origin[1] + (iy + 0.5) * resolution
        for ix in range(width):
            x = origin[0] + (ix + 0.5) * resolution
            if xmin <= x <= xmax and ymin <= y <= ymax:
                data[iy * width + ix] = 0
    if obstacle:
        ox0, ox1, oy0, oy1 = obstacle
        for iy in range(height):
            y = origin[1] + (iy + 0.5) * resolution
            for ix in range(width):
                x = origin[0] + (ix + 0.5) * resolution
                if ox0 <= x <= ox1 and oy0 <= y <= oy1:
                    data[iy * width + ix] = 100
    return {
        "data": bytes(data), "width": width, "height": height,
        "resolution": resolution, "origin": {"x": origin[0], "y": origin[1]},
        "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "quality": 0.9, "scans": 4, "updatedAt": 1_800_000_000.0,
        "revision": 7, "settings": {"min_range": 0.3},
    }


def test_planner_returns_short_forward_route_with_steering_limit():
    plan = plan_grid_path(grid_snapshot(), 1.5, 0.0)
    assert plan["ok"] is True
    assert plan["waypoints"] >= 1
    assert 1.2 <= plan["distanceM"] <= 2.0
    assert plan["mapRevision"] == 7
    points = plan["path"]
    assert points[0] == {"x": 0.0, "y": 0.0}
    assert math.hypot(plan["goal"]["x"] - 1.5, plan["goal"]["y"]) < 0.15
    for a, b in zip(points, points[1:]):
        angle = math.atan2(b["y"] - a["y"], b["x"] - a["x"])
        assert steering_deviation(angle, 0.0) <= math.radians(28.0) + 1e-6


def test_planner_rejects_unknown_goal_and_occupied_goal():
    snapshot = grid_snapshot()
    with pytest.raises(NavigationError) as unknown:
        plan_grid_path(snapshot, 3.0, 0.0)
    assert unknown.value.code == "goal_not_free"

    snapshot = grid_snapshot(obstacle=(1.0, 1.2, -0.2, 0.2))
    with pytest.raises(NavigationError) as occupied:
        plan_grid_path(snapshot, 1.1, 0.0)
    assert occupied.value.code in {"goal_not_free", "goal_too_close"}


def test_planner_does_not_cross_inflated_wall():
    # Полная стена перекрывает всю изученную область: неизвестные клетки за ней
    # не используются как короткий путь в обход.
    snapshot = grid_snapshot(
        free_box=(-1.5, 2.5, -1.2, 1.2),
        obstacle=(0.75, 0.85, -1.2, 1.2),
    )
    with pytest.raises(NavigationError) as blocked:
        plan_grid_path(snapshot, 1.8, 0.0)
    assert blocked.value.code == "no_path"


def test_navigation_snapshot_is_a_copy_with_revision():
    service = SlamMapService(MapSettings(resolution=0.1, size_m=8.0))
    before = service.navigation_snapshot()
    assert before["revision"] == 0
    assert isinstance(before["data"], bytes)
    service.ingest([{"a": 0.0, "r": 2.0}, {"a": 1.0, "r": 2.0}])
    after = service.navigation_snapshot()
    assert after["revision"] > before["revision"]
    assert before["data"] != after["data"]


def test_navigation_status_is_disabled_without_explicit_enable():
    service = SlamMapService(MapSettings(resolution=0.1, size_m=8.0))
    navigation = NavigationService(service, None, None, enabled=False)
    status = navigation.status()
    assert status["ok"] is True
    assert status["enabled"] is False
    assert status["canStart"] is False
    assert "выключена" in status["reason"].lower()


def test_preflight_refuses_existing_manual_throttle():
    class Map:
        def navigation_snapshot(self):
            snapshot = grid_snapshot()
            snapshot['updatedAt'] = __import__('time').time()
            return snapshot

    class Lidar:
        def get_scan(self, max_age=1.0):
            return [{"a": 0.0, "r": 2.0}]

    class MovingMaster:
        name = 'master'
        supports_drive = True
        supports_individual_steer = True

        def drive_status(self):
            return {"canDrive": True, "available": True, "reason": None,
                    "active": True, "commandedPercent": 2}

    navigation = NavigationService(
        Map(), MovingMaster(), Lidar(), enabled=True, mps_per_percent=0.02,
        speed_mps=0.05, max_drive_percent=4,
    )
    status = navigation.status()
    assert status['canStart'] is False
    assert status['diagnostics']['checks']['driveStopped'] is False
    assert 'активная тяга' in status['reason'].lower()


def test_start_requires_review_when_fresh_map_changes_the_preview():
    class DynamicMap:
        def __init__(self, snapshot):
            self.current = snapshot

        def navigation_snapshot(self):
            return self.current

    class Lidar:
        def get_scan(self, max_age=1.0):
            return [{"a": 0.0, "r": 2.0}]

    class Master:
        name = 'master'
        supports_drive = True
        supports_individual_steer = True

        def drive_status(self):
            return {"canDrive": True, "available": True, "reason": None}

    initial = grid_snapshot()
    initial['updatedAt'] = __import__('time').time()
    service = DynamicMap(initial)
    navigation = NavigationService(
        service, Master(), Lidar(), enabled=True, mps_per_percent=0.02,
        speed_mps=0.05, max_drive_percent=4,
    )
    preview = navigation.plan({'goal': {'x': 1.5, 'y': 0.0}})['plan']
    service.current = grid_snapshot(obstacle=(0.65, 0.75, -0.5, 0.5))
    service.current['updatedAt'] = __import__('time').time()

    with pytest.raises(NavigationError) as changed:
        navigation.start({'goal': {'x': 1.5, 'y': 0.0}})
    assert changed.value.code == 'plan_changed'
    assert changed.value.details['plan']['path'] != preview['path']
    assert navigation.active is False


def test_start_accepts_unchanged_preview_when_goal_snaps_to_cell_center(monkeypatch):
    class DynamicMap:
        def navigation_snapshot(self):
            snapshot = grid_snapshot()
            snapshot['updatedAt'] = __import__('time').time()
            return snapshot

    class Lidar:
        def get_scan(self, max_age=1.0):
            return [{"a": 0.0, "r": 2.0}]

    class Master:
        name = 'master'
        supports_drive = True
        supports_individual_steer = True

        def drive_status(self):
            return {"canDrive": True, "available": True, "reason": None,
                    "active": False, "commandedPercent": 0}

    navigation = NavigationService(
        DynamicMap(), Master(), Lidar(), enabled=True, mps_per_percent=0.02,
        speed_mps=0.05, max_drive_percent=4,
    )
    goal = {'x': 1.5, 'y': 0.0}
    preview = navigation.plan({'goal': goal})['plan']
    assert preview['goal'] != goal  # planner snaps to a nearby safe cell point
    captured = {}
    captured_event = threading.Event()

    def fake_run(target, cancel, plan):
        captured.update(target=target, plan=plan)
        captured_event.set()

    monkeypatch.setattr(navigation, '_run', fake_run)

    _payload, code = navigation.start({'goal': goal})
    assert code == 202
    assert captured_event.wait(1.0)
    assert captured['target']['requested'] == {'x': 1.5, 'y': 0.0}
    assert captured['plan']['goalRequested'] == {'x': 1.5, 'y': 0.0}


def test_kiosk_navigation_requires_control_header_and_reports_safety_gate(kiosk_client):
    response = kiosk_client.get('/api/navigation/status')
    assert response.status_code == 200
    assert response.json['ok'] is True
    assert response.json['enabled'] is False
    assert response.json['canStart'] is False

    missing_header = kiosk_client.post('/api/navigation/start', json={'goal': {'x': 1, 'y': 0}})
    assert missing_header.status_code == 400
    assert missing_header.json['reason'] == 'request_header_required'

    blocked = kiosk_client.post(
        '/api/navigation/start', json={'goal': {'x': 1, 'y': 0}},
        headers={'X-Robot-Control': 'navigation'},
    )
    assert blocked.status_code == 409
    assert blocked.json['code'] == 'not_ready'

    assert kiosk_client.post('/api/navigation/estop').status_code == 400
    estop = kiosk_client.post('/api/navigation/estop', headers={'X-Robot-Control': 'navigation'})
    assert estop.status_code == 200
    assert estop.json['estop'] is True
    assert estop.json['status']['diagnostics']['checks']['estopClear'] is False


def test_web_navigation_proxy_requires_csrf_and_forwards_only_fixed_control(client, operator,
                                                                            kiosk_client, monkeypatch):
    login(client)
    calls = []

    class Reply(io.BytesIO):
        def __init__(self, payload, code=200):
            raw = payload if isinstance(payload, bytes) else json.dumps(payload).encode()
            super().__init__(raw)
            self.code = code

    def forward(req, timeout):
        calls.append((req.full_url, req.method, dict(req.header_items())))
        path = req.full_url.split('/api/navigation', 1)[1]
        try:
            response = kiosk_client.open(
                '/api/navigation' + path, method=req.method, data=req.data,
                headers={key: value for key, value in req.header_items()
                         if key.lower() in {'x-robot-control', 'content-type'}},
                content_type='application/json',
            )
        except HTTPError as exc:  # pragma: no cover - Flask test client returns responses
            return Reply(exc.read(), exc.code)
        return Reply(response.data, response.status_code)

    monkeypatch.setenv('RC_KIOSK_PORT', '18090')
    monkeypatch.setattr(map_peer, 'urlopen', forward)

    # Signed operator session alone is not enough for a movement command.
    rejected = client.post('/api/navigation/start', json={'goal': {'x': 1, 'y': 0}})
    assert rejected.status_code == 400
    assert calls == []

    status = client.get('/api/navigation/status')
    assert status.status_code == 200
    token = status.json['csrfToken']
    assert token

    blocked = client.post(
        '/api/navigation/start', json={'goal': {'x': 1, 'y': 0}},
        headers={'X-Robot-Control': 'navigation', 'X-CSRF-Token': token},
    )
    assert blocked.status_code == 409
    assert blocked.json['code'] == 'not_ready'
    assert calls[-1][0].startswith('http://127.0.0.1:18090/api/navigation/start')
    forwarded_headers = {key.lower(): value for key, value in calls[-1][2].items()}
    assert forwarded_headers['x-robot-control'] == 'navigation'
    assert 'x-csrf-token' not in forwarded_headers

    estop = client.post(
        '/api/navigation/estop', json={},
        headers={'X-Robot-Control': 'navigation', 'X-CSRF-Token': token},
    )
    assert estop.status_code == 200
    assert estop.json['estop'] is True
    assert calls[-1][0].endswith('/api/navigation/estop')


def test_master_peer_web_estop_assert_and_pin_release_are_proxied_to_onboard(
        client, operator, monkeypatch):
    from types import SimpleNamespace

    login(client)
    token = client.get('/api/navigation/status').json['csrfToken']
    assert token
    monkeypatch.setenv('RC_KIOSK_PORT', '18090')
    client.application.extensions['rc_slam'].source = SimpleNamespace(name='master-peer')
    calls = []

    class Reply(io.BytesIO):
        code = 200

    def forward(req, timeout):
        calls.append((req.full_url, req.method, dict(req.header_items()),
                      json.loads(req.data.decode('utf-8'))))
        return Reply(json.dumps({'ok': True, 'estop': True}).encode())

    monkeypatch.setattr(map_peer, 'urlopen', forward)
    headers = {'X-Robot-Control': 'drive', 'X-CSRF-Token': token}

    asserted = client.post('/api/estop', json={}, headers=headers)
    assert asserted.status_code == 200
    assert asserted.json['estop'] is True
    released = client.post('/api/estop/release', json={'pin': '2468'}, headers=headers)
    assert released.status_code == 200
    assert released.json['estop'] is True

    assert [call[0] for call in calls] == [
        'http://127.0.0.1:18090/api/estop',
        'http://127.0.0.1:18090/api/estop/release',
    ]
    assert calls[0][1] == calls[1][1] == 'POST'
    assert calls[0][3] == {}
    assert calls[1][3] == {'pin': '2468'}
    for _url, _method, headers_sent, _body in calls:
        lowered = {key.lower(): value for key, value in headers_sent.items()}
        assert lowered['x-robot-control'] == 'drive'
        assert 'x-csrf-token' not in lowered


def test_navigation_page_contains_goal_and_supervised_controls(client, operator):
    login(client)
    page = client.get('/map')
    assert page.status_code == 200
    for control in ('nav-goal-x', 'nav-goal-y', 'nav-select-goal', 'nav-plan',
                    'nav-start', 'nav-cancel', 'nav-estop', 'nav-route-summary'):
        assert 'id="%s"' % control in page.text
    assert 'RCMapView.init()' in page.text
