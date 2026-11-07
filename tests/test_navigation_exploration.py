"""Conservative frontier planning and the navigation worker without hardware.

These simulations exercise the real planner/worker, but are not physical tests
of lidar occlusion, braking, steering, wheel slip or localization reliability.
"""
from __future__ import annotations

import copy
import math
import os
import shutil
import subprocess
import threading
import time
from pathlib import Path

import pytest

from robot_control import estop
from robot_control.navigation import (
    NavigationError, NavigationService, _ClearanceGrid, _cell, _obstacle_ahead,
    _scan_coverage, plan_route,
)
from robot_control.slam_map import MapSettings, SlamMapService
from test_slam_map import room_scan


def partial_grid(free_until=2.0):
    width = 100
    res = 0.1
    origin = -5.0
    data = bytearray(width * width)
    for y in range(width):
        for x in range(width):
            if origin + x * res >= free_until:
                data[y * width + x] = 255
    return {"width": width, "height": width, "resolution": res,
            "origin": {"x": origin, "y": origin}, "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "data": bytes(data), "revision": 1, "scans": 3, "quality": 1.0,
            "updatedAt": time.time(), "trusted": True, "navigationIntegrity": True,
            "settings": {"matching": True, "decay": 0.0, "min_range": 0.3}}


def check_path_clear(snapshot, plan, footprint=0.55):
    clearance = _ClearanceGrid(snapshot, footprint)
    for a, b in zip(plan["path"], plan["path"][1:]):
        assert clearance.clear_line(snapshot, (a["x"], a["y"]), (b["x"], b["y"]))


def test_unknown_goal_returns_only_safe_prefix_and_original_request():
    snapshot = partial_grid()
    original = copy.deepcopy(snapshot)
    requested = [{"x": 3.0, "y": 0.0}]
    plan = plan_route(snapshot, requested, footprint_m=0.55)
    assert plan["partial"] is True and plan["executable"] is True
    assert plan["goalsRequested"] == requested
    assert 0.3 < plan["goal"]["x"] < 2.0 - 0.55
    assert plan["goal"] != requested[0]
    check_path_clear(snapshot, plan)
    assert snapshot == original


def test_partial_route_advances_only_after_unknown_becomes_known():
    before = partial_grid(free_until=2.0)
    after = partial_grid(free_until=3.8)
    goals = [{"x": 3.0, "y": 0.0}]
    first = plan_route(before, goals, footprint_m=0.55)
    later = plan_route(after, goals, footprint_m=0.55)
    assert first["partial"] and not later["partial"]
    assert first["goal"]["x"] < later["goal"]["x"]
    check_path_clear(before, first)
    check_path_clear(after, later)


def test_empty_map_never_offers_a_fabricated_drivable_path():
    snapshot = partial_grid(free_until=-5)
    snapshot["scans"] = 0
    plan = plan_route(snapshot, [{"x": 2.0, "y": 0}], footprint_m=0.55)
    assert plan["partial"] and plan["frontierBlocked"]
    assert not plan["executable"] and len(plan["path"]) == 1


def test_frontier_is_not_a_success_and_intermediate_unknown_is_not_skipped():
    snapshot = partial_grid(free_until=2)
    goals = [{"x": 3, "y": 0}, {"x": 0, "y": 0}]
    plan = plan_route(snapshot, goals, footprint_m=0.55)
    assert plan["partialGoalIndex"] == 0
    assert plan["goalsRequested"] == goals
    assert plan["goal"]["x"] > 0
    check_path_clear(snapshot, plan)


def test_occupied_goal_or_known_wall_is_not_made_free():
    snapshot = partial_grid(free_until=3)
    data = bytearray(snapshot["data"])
    ix, iy = _cell(snapshot, 1.0, 0.0)
    data[iy * snapshot["width"] + ix] = 100
    snapshot["data"] = bytes(data)
    with pytest.raises(NavigationError):
        plan_route(snapshot, [{"x": 1.0, "y": 0.0}], footprint_m=0.55)


def test_incomplete_scan_and_front_rear_offset_fail_conservatively():
    scan = [{"a": -math.pi + i * math.tau / 360, "r": 3} for i in range(360)]
    assert _scan_coverage(scan)[0]
    assert not _scan_coverage(scan[:20])[0]
    assert not _scan_coverage([p for p in scan if abs(p["a"]) < 2])[0]
    # The scanner is 35 cm in front of the centre: a point 1 m behind the
    # sensor is only 0.65 m behind the centre and must trigger the rear guard.
    assert _obstacle_ahead([{"a": math.pi, "r": 1}], math.pi, 0.85,
                           half_width_m=0.6, lidar_offset_m=0.35) == pytest.approx(0.65)
    assert _obstacle_ahead([{"a": 0, "r": 0.1}], 0, 0.85,
                           half_width_m=0.6, lidar_offset_m=0.35) == pytest.approx(0.45)


class FakeScan:
    def __init__(self):
        self.points = [{"a": -math.pi + i * math.tau / 360, "r": 6.0} for i in range(360)]
        self.trusted = True
        self.alive = True

    def get_scan_info(self, max_age=1.0):
        del max_age
        return {"points": self.points if self.alive else None, "trusted": self.trusted}


class FakeMap:
    def __init__(self, reveal_after=0.45):
        self.lock = threading.RLock()
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.free_until = 1.8
        self.reveal_after = reveal_after
        self.revision = 1
        self.wall_x = None
        self.quality = 1.0
        self.settings = {"matching": True, "decay": 0.0, "min_range": 0.3}
        self.trusted = True

    def advance(self, dx, dy):
        with self.lock:
            self.pose["x"] += dx
            self.pose["y"] += dy
            if self.reveal_after is not None and self.pose["x"] >= self.reveal_after:
                self.free_until = 4.0
            self.revision += 1

    def navigation_snapshot(self):
        with self.lock:
            result = partial_grid(self.free_until)
            result.update(pose=dict(self.pose), revision=self.revision, quality=self.quality,
                          settings=dict(self.settings), trusted=self.trusted, updatedAt=time.time())
            if self.wall_x is not None:
                data = bytearray(result["data"])
                x, _ = _cell(result, self.wall_x, 0)
                for y in range(result["height"]):
                    data[y * result["width"] + x] = 100
                result["data"] = bytes(data)
            return result


class FakeMaster:
    name = "master"
    supports_drive = True
    supports_individual_steer = True
    STALE_S = 1.0

    def __init__(self, mapping):
        self.lock = threading.RLock()
        self.mapping = mapping
        self.percent = 0
        self.brake_id = 0
        self.ack = True
        self.moves = []
        self.brakes = 0
        self.counts = {m: 0 for m in ("FL", "FR", "RL", "RR")}
        self.on_drive = None
        self.angles = {m: 0.0 for m in ("FL", "FR", "RL", "RR")}
        self.frames = {}
        self._refresh()

    def _refresh(self):
        with self.lock:
            now = time.time()
            self.frames = {m: ({"cal": True, "moving": False, "calib": False,
                                "cycle": False, "deg": self.angles[m],
                                "drivePercent": self.percent, "scRpm": 0.0,
                                "scRecent": False, "scCount": self.counts[m], "firmware": 23,
                                "brakeId": self.brake_id if self.ack else 0}, now)
                           for m in self.angles}

    def drive_status(self):
        self._refresh()
        return {"available": True, "canDrive": True, "reason": "", "active": self.percent != 0,
                "commandedPercent": self.percent}

    def steer_module(self, name, angle, now=None):
        del now
        self.angles[name] = angle
        self._refresh()
        return {"ok": True}

    def drive(self, percent, now=None):
        del now
        self.percent = percent
        if percent:
            # Firmware positive is clockwise/right; map +Y is left.
            theta = -math.radians(sum(self.angles.values()) / 4)
            direction = 1 if percent > 0 else -1
            dx, dy = direction * 0.10 * math.cos(theta), direction * 0.10 * math.sin(theta)
            self.mapping.advance(dx, dy)
            for module in self.counts:
                self.counts[module] += 1
            self.moves.append((dx, dy))
            if self.on_drive:
                self.on_drive(self)
        self._refresh()
        return {"ok": True}

    def brake_for_keyboard(self):
        self.brake_id += 1
        self.brakes += 1
        self.percent = 0
        self._refresh()
        return {"ok": True, "brakeId": self.brake_id}


def service(mapping=None):
    mapping = mapping or FakeMap()
    source = FakeMaster(mapping)
    lidar = FakeScan()
    nav = NavigationService(mapping, source, lidar, enabled=True,
                            mps_per_percent=0.02, speed_mps=0.04, max_drive_percent=4)
    return nav, mapping, source, lidar


def await_finished(nav, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = nav.status()
        if state["phase"] in ("succeeded", "blocked", "error", "cancelled", "planned") and not nav.active:
            return state
        time.sleep(0.05)
    nav.cancel("test timeout")
    pytest.fail("navigation worker did not finish: %r" % nav.status())


def test_worker_drives_partial_then_replans_and_reaches_real_goal():
    nav, mapping, source, _lidar = service()
    body = {"goal": {"x": 2.5, "y": 0.0}}
    preview = nav.plan(body)
    assert preview["plan"]["partial"]
    assert preview["status"]["goal"] == body["goal"]
    assert preview["status"]["canStart"]
    _reply, code = nav.start(body)
    assert code == 202
    state = await_finished(nav)
    assert state["phase"] == "succeeded", state
    assert abs(mapping.pose["x"] - body["goal"]["x"]) <= 0.14
    assert source.moves and source.brakes >= 2 and source.percent == 0


def test_worker_stops_at_unrevealed_frontier_instead_of_reporting_success():
    nav, mapping, source, _lidar = service(FakeMap(reveal_after=None))
    body = {"goal": {"x": 2.5, "y": 0.0}}
    assert nav.plan(body)["plan"]["partial"]
    nav.start(body)
    state = await_finished(nav)
    assert state["phase"] == "blocked" and "границ" in state["message"].lower()
    assert mapping.pose["x"] < mapping.free_until - 0.55
    assert source.percent == 0 and source.moves


def test_worker_stops_when_scan_is_lost():
    nav, _mapping, source, lidar = service()
    source.on_drive = lambda _source: setattr(lidar, "alive", False)
    body = {"goal": {"x": 2.5, "y": 0}}
    nav.plan(body)
    nav.start(body)
    state = await_finished(nav)
    assert state["phase"] == "blocked"
    assert source.percent == 0 and len(source.moves) == 1


def test_worker_stops_when_estop_latches(monkeypatch):
    nav, _mapping, source, _lidar = service()
    source.on_drive = lambda _source: estop.latch("integration test")
    body = {"goal": {"x": 2.5, "y": 0}}
    nav.plan(body)
    nav.start(body)
    state = await_finished(nav)
    assert state["phase"] == "blocked" and "E-STOP" in state["message"]
    assert source.percent == 0 and len(source.moves) == 1


def test_worker_brakes_before_actuation_when_stop_ack_missing():
    nav, _mapping, source, _lidar = service()
    source.ack = False
    body = {"goal": {"x": 2.5, "y": 0}}
    nav.plan(body)
    nav.start(body)
    state = await_finished(nav, timeout=8)
    assert state["phase"] == "blocked" and "STOP" in state["message"]
    assert not source.moves and source.percent == 0


def test_preflight_requires_full_coverage_matching_and_geometric_footprint(monkeypatch):
    nav, mapping, _source, lidar = service()
    assert nav._preflight()[0]
    lidar.points = lidar.points[:10]
    assert not nav._preflight()[0]
    lidar.points = FakeScan().points
    mapping.settings["matching"] = False
    assert not nav._preflight()[0]
    mapping.settings["matching"] = True
    mapping.settings["decay"] = 1
    assert not nav._preflight()[0]
    mapping.settings["decay"] = 0
    nav.footprint_m = 0.42
    assert not nav._preflight()[0]
    nav.footprint_m = 0.55
    mapping.trusted = False
    assert not nav._preflight()[0]


def test_worker_brakes_before_crossing_a_new_obstacle_on_map():
    nav, mapping, source, _lidar = service()
    # Appears in the previously planned section on the first traction command.
    source.on_drive = lambda _source: setattr(mapping, "wall_x", 0.8)
    body = {"goal": {"x": 2.5, "y": 0}}
    nav.plan(body)
    nav.start(body)
    state = await_finished(nav)
    assert state["phase"] == "blocked", state
    assert len(source.moves) == 1 and source.percent == 0
    assert source.brakes >= 2


def test_kiosk_api_exposes_partial_route_and_executes_only_after_start(kiosk_client):
    nav = kiosk_client.application.extensions["rc_navigation"]
    fake, mapping, source, lidar = service()
    nav.map_service, nav.source, nav.lidar_manager = mapping, source, lidar
    nav.enabled, nav.mps_per_percent, nav.speed_mps, nav.max_drive_percent = True, 0.02, 0.04, 4
    goal = {"goal": {"x": 2.5, "y": 0}}
    response = kiosk_client.post("/api/navigation/plan", json=goal)
    assert response.status_code == 200
    assert response.json["plan"]["partial"] and response.json["status"]["canStart"]
    assert response.json["status"]["goal"] == goal["goal"]
    assert not source.moves
    assert kiosk_client.post("/api/navigation/start", json=goal).status_code == 400
    response = kiosk_client.post("/api/navigation/start", json=goal,
                                 headers={"X-Robot-Control": "navigation"})
    assert response.status_code == 202
    result = await_finished(nav)
    assert result["phase"] == "succeeded", result
    assert source.percent == 0 and source.moves


def test_real_map_scan_matcher_planner_and_worker_in_synthetic_room():
    """Actual SLAM -> grid -> partial goal -> worker; simulated motor and scans."""
    class SlamWorld:
        def __init__(self):
            self.lock = threading.RLock()
            self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
            self.map = SlamMapService(MapSettings(size_m=10, max_range=1.8, autosave=False))
            self.lidar = FakeScan()
            self.tick()
            self.tick()

        def tick(self):
            with self.lock:
                scan = room_scan(9, 9, px=self.pose["x"] + 0.28, py=self.pose["y"])
                self.lidar.points = scan
                self.map.ingest(scan, stamp=time.time(), trusted=True)

        def advance(self, dx, dy):
            with self.lock:
                self.pose["x"] += dx
                self.pose["y"] += dy
                self.tick()

        def navigation_snapshot(self):
            return self.map.navigation_snapshot()

    world = SlamWorld()
    source = FakeMaster(world)
    nav = NavigationService(world, source, world.lidar, enabled=True,
                            mps_per_percent=0.02, speed_mps=0.04, max_drive_percent=4)
    stop = threading.Event()

    def scan_loop():
        while not stop.wait(0.3):
            world.tick()

    thread = threading.Thread(target=scan_loop, daemon=True)
    thread.start()
    try:
        body = {"goal": {"x": 2.0, "y": 0}}
        preview = nav.plan(body)
        assert preview["plan"]["partial"] and preview["plan"]["manualBlindZone"] and preview["status"]["canStart"]
        with pytest.raises(NavigationError) as missing:
            nav.start(body)
        assert missing.value.code == "blind_zone_confirmation_required" and not source.moves
        nav.start({**body, "blindZoneChecked": True})
        state = await_finished(nav, timeout=25)
        assert state["phase"] == "succeeded", state
        assert abs(world.pose["x"] - 2.0) < 0.25
        assert source.percent == 0
    finally:
        stop.set()
        thread.join(timeout=2)


def test_blind_exception_is_world_fixed_and_never_hides_known_obstacles():
    snapshot = partial_grid(free_until=4)
    values = bytearray(snapshot['data'])
    for x in (0.35, 0.45, 2.35):
        ix, iy = _cell(snapshot, x, 0)
        values[iy * snapshot['width'] + ix] = 255
    snapshot['data'] = bytes(values)
    initial_zone = {'x': 0.35, 'y': 0.0, 'radiusM': 0.3}
    clearance = _ClearanceGrid(snapshot, 0.55, initial_zone)
    assert clearance.ignored_blind_cells == 2
    assert clearance.clear_cell(*_cell(snapshot, 0, 0))
    # A new blind circle at x=2.35 is NOT automatically assumed free later.
    assert not clearance.clear_cell(*_cell(snapshot, 2.35, 0))
    moved_zone = _ClearanceGrid(snapshot, 0.55, {'x': 2.35, 'y': 0.0, 'radiusM': 0.3})
    assert moved_zone.clear_cell(*_cell(snapshot, 2.35, 0))
    # Even within the manually checked circle a real hit remains blocked.
    ix, iy = _cell(snapshot, 0.35, 0)
    values[iy * snapshot['width'] + ix] = 100
    snapshot['data'] = bytes(values)
    assert not _ClearanceGrid(snapshot, 0.55, initial_zone).clear_cell(*_cell(snapshot, 0, 0))


def test_grid_line_cannot_cut_an_occupied_corner_or_follow_its_boundary():
    from robot_control.navigation import _line_is_clear
    snapshot = {'width': 4, 'height': 4, 'resolution': 1.0,
                'origin': {'x': 0.0, 'y': 0.0}, 'data': bytes(16)}
    blocked = bytearray(16)
    blocked[0 * 4 + 1] = 1
    # Touching (1,1) touches all four cells, including the blocked corner.
    assert not _line_is_clear(snapshot, blocked, (0.1, 0.1), (1.9, 1.9))
    # A line exactly on the grid border must check both sides.
    assert not _line_is_clear(snapshot, blocked, (1.0, 0.2), (1.0, 0.9))
    assert _line_is_clear(snapshot, blocked, (0.1, 2.2), (1.9, 2.2))


class BlindSpotMap(FakeMap):
    def navigation_snapshot(self):
        snapshot = super().navigation_snapshot()
        data = bytearray(snapshot['data'])
        # Fixed world cell under the original lidar at the first preview.
        ix, iy = _cell(snapshot, 0.28, 0.0)
        data[iy * snapshot['width'] + ix] = 255
        snapshot['data'] = bytes(data)
        return snapshot


def test_blind_zone_moved_after_preview_requires_new_inspection_and_plan():
    nav, mapping, source, _ = service(BlindSpotMap())
    body = {'goal': {'x': 2.5, 'y': 0}}
    first = nav.plan(body)['plan']
    assert first['manualBlindZone'] and first['blindInspectionRadiusM'] == 0.48
    mapping.pose['x'] += 0.05
    with pytest.raises(NavigationError) as exc:
        nav.start({**body, 'blindZoneChecked': True})
    assert exc.value.code == 'plan_changed'
    assert exc.value.details['plan']['blindZone']['x'] == pytest.approx(0.33)
    assert not source.moves and not nav.active


def test_blind_zone_moved_during_stop_barrier_never_starts_driving():
    nav, mapping, source, _ = service(BlindSpotMap())
    body = {'goal': {'x': 2.5, 'y': 0}}
    assert nav.plan(body)['plan']['manualBlindZone']
    original_brake = source.brake_for_keyboard
    moved = False

    def brake_and_move():
        nonlocal moved
        result = original_brake()
        if not moved:
            mapping.pose['x'] += 0.05
            moved = True
        return result

    source.brake_for_keyboard = brake_and_move
    nav.start({**body, 'blindZoneChecked': True})
    state = await_finished(nav)
    assert state['phase'] == 'planned' and 'провер' in state['message'].lower()
    assert state['plan']['blindZone']['x'] == pytest.approx(0.33)
    assert not source.moves and source.percent == 0


def test_running_feedback_rejects_silent_sc_and_unconfirmed_traction():
    from types import SimpleNamespace
    nav, _, source, _ = service()
    source.percent = 2
    source._refresh()
    bridge = SimpleNamespace(percent=2)
    now = time.monotonic()
    samples = {'FL': {'count': 0, 'sampleAt': now - 0.1, 'pulseAt': now - 3.1}}
    with pytest.raises(NavigationError) as exc:
        nav._check_running_feedback(bridge, samples, now - 4)
    assert exc.value.code == 'sc_stalled'
    source.percent = 0
    source._refresh()
    with pytest.raises(NavigationError) as exc:
        nav._check_running_feedback(bridge, {}, now - 4)
    assert exc.value.code == 'drive_unconfirmed'
    del source.frames['FL']
    with pytest.raises(NavigationError) as exc:
        nav._check_running_feedback(bridge, {}, now - 4)
    assert exc.value.code == 'module_offline'


def test_worker_stops_when_a_wheel_never_confirms_traction():
    nav, _mapping, source, _lidar = service()
    original_refresh = source._refresh

    def no_drive_ack():
        original_refresh()
        if source.percent:
            source.frames['FL'][0]['drivePercent'] = 0

    source._refresh = no_drive_ack
    body = {'goal': {'x': 2.5, 'y': 0}}
    nav.plan(body)
    nav.start(body)
    state = await_finished(nav)
    assert state['phase'] == 'blocked' and 'не подтвердило тягу' in state['message']
    assert source.moves and source.percent == 0


def test_worker_can_explore_around_an_obstacle_without_corner_cutting():
    class ObstacleMap(FakeMap):
        def advance(self, dx, dy):
            super().advance(dx, dy)
            if self.free_until == 4.0:
                self.free_until = 4.8

        def navigation_snapshot(self):
            snapshot = super().navigation_snapshot()
            data = bytearray(snapshot['data'])
            ix, _ = _cell(snapshot, 1.5, 0)
            for iy in range(snapshot['height']):
                y = snapshot['origin']['y'] + (iy + 0.5) * snapshot['resolution']
                if abs(y) <= 0.25:
                    data[iy * snapshot['width'] + ix] = 100
            snapshot['data'] = bytes(data)
            return snapshot

    mapping = ObstacleMap(reveal_after=0.5)
    mapping.free_until = 3.0
    nav, _, source, _ = service(mapping)
    body = {'goal': {'x': 3.5, 'y': 0.0}}
    preview = nav.plan(body)
    assert preview['plan']['partial'] and len(preview['plan']['path']) > 2
    check_path_clear(mapping.navigation_snapshot(), preview['plan'])
    nav.start(body)
    state = await_finished(nav, timeout=45)
    assert state['phase'] == 'succeeded', state
    assert math.hypot(mapping.pose['x'] - 3.5, mapping.pose['y']) <= 0.14
    assert source.percent == 0 and source.brakes > 3


def test_worker_stops_when_sc_pulses_disappear():
    class SlowMap(FakeMap):
        def advance(self, dx, dy):
            super().advance(dx * 0.1, dy * 0.1)

    nav, _, source, _ = service(SlowMap())
    source.on_drive = lambda master: master.counts.update({name: 0 for name in master.counts})
    body = {'goal': {'x': 2.5, 'y': 0}}
    nav.plan(body)
    nav.start(body)
    state = await_finished(nav, timeout=12)
    assert state['phase'] == 'blocked' and 'импульсов SC' in state['message']
    assert source.moves and source.percent == 0


def test_stop_barrier_times_out_if_sc_reports_wheel_still_moving(monkeypatch):
    from types import SimpleNamespace
    import robot_control.navigation as module

    nav, _, source, _ = service()
    original_refresh = source._refresh

    def sc_still_moving():
        original_refresh()
        source.frames['FL'][0]['scRecent'] = True
        source.frames['FL'][0]['scRpm'] = 3.0

    source._refresh = sc_still_moving
    times = iter([0.0, 1.0, 6.0])
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: next(times, 6.0)))
    cancel = threading.Event()
    monkeypatch.setattr(cancel, 'wait', lambda _timeout: False)
    with pytest.raises(NavigationError) as exc:
        nav._await_stopped(SimpleNamespace(brake=lambda _reason: None), cancel, 'test')
    assert exc.value.code == 'brake_unconfirmed'
    assert not source.moves and source.percent == 0


BROWSER_UNAVAILABLE = (not os.environ.get('RC_TEST_CHROMIUM')
                       or not os.environ.get('RC_TEST_PLAYWRIGHT_CORE')
                       or not shutil.which('node'))


@pytest.mark.skipif(BROWSER_UNAVAILABLE, reason='реальному браузеру нужны Chromium и playwright-core')
def test_navigation_browser_propagates_navigation_failure():
    script = Path(__file__).parent / 'js' / 'navigation_browser.mjs'
    result = subprocess.run([shutil.which('node'), str(script), 'http://127.0.0.1:1'],
                            env=os.environ.copy(), capture_output=True, text=True, timeout=30)
    assert result.returncode != 0
    assert 'page.goto' in result.stderr
    assert 'simulated goal: OK' not in result.stdout


@pytest.mark.skipif(BROWSER_UNAVAILABLE, reason='реальному браузеру нужны Chromium и playwright-core')
def test_partial_navigation_in_real_browser_with_simulated_map_and_wheels(kiosk_client):
    from werkzeug.serving import make_server

    app = kiosk_client.application
    nav = app.extensions['rc_navigation']
    _, mapping, source, lidar = service(BlindSpotMap())
    nav.map_service, nav.source, nav.lidar_manager = mapping, source, lidar
    nav.enabled, nav.mps_per_percent, nav.speed_mps, nav.max_drive_percent = True, 0.02, 0.04, 4
    server = make_server('127.0.0.1', 0, app, threaded=True)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    script = Path(__file__).parent / 'js' / 'navigation_browser.mjs'
    try:
        result = subprocess.run([shutil.which('node'), str(script),
                                 f'http://127.0.0.1:{server.server_port}'],
                                env=os.environ.copy(), capture_output=True, text=True, timeout=60)
        assert result.returncode == 0, result.stdout + '\n' + result.stderr
        assert 'simulated goal: OK' in result.stdout
        assert source.moves and source.percent == 0
    finally:
        if nav.active:
            worker = nav._worker
            nav.cancel('browser test cleanup')
            if worker is not None:
                worker.join(timeout=3)
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)
