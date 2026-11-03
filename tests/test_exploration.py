"""Автономное обследование: границы карты, цикл, блокировки, ориентиры."""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import pytest

from robot_control.exploration import (
    ExplorationError, ExplorationService, LandmarkRecorder, coverage_pct,
    find_frontiers, known_cells,
)
from robot_control.navigation import NavigationError
from robot_control.slam_map import UNKNOWN
from test_navigation_exploration import await_finished, service as real_nav_service


WIDTH = HEIGHT = 60
RES = 0.1
ORIGIN = -3.0


def grid_snapshot(free_until=1.0, pose=(0.0, 0.0, 0.0)):
    data = bytearray([UNKNOWN]) * (WIDTH * HEIGHT)
    for iy in range(HEIGHT):
        for ix in range(WIDTH):
            if ORIGIN + ix * RES < free_until:
                data[iy * WIDTH + ix] = 0
    return {
        "data": bytes(data), "width": WIDTH, "height": HEIGHT,
        "resolution": RES, "origin": {"x": ORIGIN, "y": ORIGIN},
        "pose": {"x": pose[0], "y": pose[1], "yaw": pose[2]},
        "scans": 3, "quality": 1.0, "updatedAt": time.time(),
        "revision": 1, "trusted": True, "navigationIntegrity": True,
        "settings": {"matching": True, "decay": 0.0, "min_range": 0.3},
    }


class FakeMap:
    def __init__(self, free_until=1.0, reveal=0.6):
        self.free_until = free_until
        self.reveal = reveal
        self.saved = 0

    def navigation_snapshot(self):
        return grid_snapshot(self.free_until)

    def advance(self):
        self.free_until = min(ORIGIN + WIDTH * RES - 0.2, self.free_until + self.reveal)

    def save(self):
        self.saved += 1
        return {"ok": True}


class FakeKinect:
    def __init__(self, fresh=True):
        self.value = fresh

    def fresh(self):
        return self.value


class FakeNavigation:
    def __init__(self, mapping, *, enabled=True):
        self.enabled = enabled
        self.mapping = mapping
        self.active = False
        self.starts = []
        self.cancels = []
        self.hold_active = False

    def start(self, body, *, autonomous=False):
        assert autonomous is True, "обследование не имеет права обходить проверенный план оператора"
        assert body.get("blindZoneChecked") is True
        self.starts.append(body["goal"])
        self.mapping.advance()
        self.active = self.hold_active

    def cancel(self, reason):
        self.cancels.append(reason)
        self.active = False


class FakeCamera:
    def __init__(self, frame=b"jpeg"):
        self.frame = frame
        self.roles = {i: f"role{i}" for i in range(4)}

    def get_camera(self, cam_id):
        return type("Dev", (), {"role": self.roles.get(cam_id, ""),
                                "spec": {"yaw_deg": cam_id * 10.0}})()

    def get_frame(self, cam_id, max_age_s=2.0):
        return self.frame if 0 <= cam_id < 4 else None


def wait_done(service, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not service.active:
            return
        time.sleep(0.01)
    service.stop("test timeout")
    pytest.fail("цикл обследования не завершился")


def test_frontiers_found_only_where_free_meets_unknown():
    snapshot = grid_snapshot(free_until=1.0)
    assert known_cells(snapshot) > 0 and 0 < coverage_pct(snapshot) < 100
    candidates = find_frontiers(snapshot)
    assert candidates, "должна быть граница между свободным и неизвестным"
    best = candidates[0]
    assert best["cells"] >= 4
    assert best["goal"]["x"] > best["x"] > 0.9
    assert abs(best["headingDeg"]) < 45, "вперёд приоритетнее, чем за спиной"
    assert math.isfinite(best["distanceM"])


def test_no_frontiers_on_empty_or_fully_known_map():
    empty = grid_snapshot(free_until=-10.0)
    assert find_frontiers(empty) == []
    full = grid_snapshot(free_until=10.0)
    assert find_frontiers(full) == []
    assert coverage_pct(full) == 100.0


def test_frontier_near_wall_is_not_offered():
    snapshot = grid_snapshot(free_until=1.0)
    data = bytearray(snapshot["data"])
    # Занятая клетка напротив границы: соседние UNKNOWN не должны стать целью.
    ix = int((1.0 - ORIGIN) / RES)
    for iy in range(25, 35):
        data[iy * WIDTH + ix] = 100
    snapshot["data"] = bytes(data)
    candidates = find_frontiers(snapshot)
    assert all(c["x"] > 1.0 for c in candidates)


def test_start_requires_all_confirmations(monkeypatch):
    monkeypatch.setenv("RC_EXPLORE_POLL_S", "0.01")
    mapping = FakeMap(free_until=10.0)  # границ нет — поток сразу завершится
    kinect = FakeKinect(fresh=False)
    nav = FakeNavigation(mapping)
    service = ExplorationService(mapping, nav, kinect=kinect, enabled=True,
                                 estop_check=lambda: None)
    with pytest.raises(ExplorationError) as exc:
        service.start({"blindZoneChecked": True})
    assert exc.value.code == "depth_unavailable"
    kinect.value = True
    with pytest.raises(ExplorationError) as exc:
        service.start({})
    assert exc.value.code == "blind_zone_confirmation_required"
    reply, code = service.start({"blindZoneChecked": True})
    assert code == 202 and reply["ok"] is True
    wait_done(service)
    assert nav.starts == []          # карта полная — ехать некуда
    assert service.status()["phase"] == "done"


def test_disabled_and_estop_block_start():
    mapping = FakeMap(free_until=10.0)
    nav = FakeNavigation(mapping)
    disabled = ExplorationService(mapping, nav, kinect=FakeKinect(), enabled=False,
                                  estop_check=lambda: None)
    with pytest.raises(ExplorationError) as exc:
        disabled.start({"blindZoneChecked": True})
    assert exc.value.code == "disabled"
    nav.enabled = False
    no_nav = ExplorationService(mapping, nav, kinect=FakeKinect(), enabled=True,
                                estop_check=lambda: None)
    with pytest.raises(ExplorationError) as exc:
        no_nav.start({"blindZoneChecked": True})
    assert exc.value.code == "navigation_disabled"
    nav.enabled = True
    estopped = ExplorationService(mapping, nav, kinect=FakeKinect(), enabled=True,
                                  estop_check=lambda: {"latched": True})
    with pytest.raises(ExplorationError) as exc:
        estopped.start({"blindZoneChecked": True})
    assert exc.value.code == "estop"


def test_loop_drives_until_map_is_complete(monkeypatch):
    monkeypatch.setenv("RC_EXPLORE_POLL_S", "0.01")
    monkeypatch.setenv("RC_EXPLORE_MIN_GAIN_CELLS", "1")
    monkeypatch.setenv("RC_EXPLORE_MAX_NO_GAIN", "5")
    monkeypatch.setenv("RC_EXPLORE_MAX_STEPS", "30")
    mapping = FakeMap(free_until=1.0, reveal=0.7)
    nav = FakeNavigation(mapping)
    service = ExplorationService(mapping, nav, kinect=FakeKinect(), enabled=True,
                                 estop_check=lambda: None)
    service.start({"blindZoneChecked": True})
    wait_done(service, timeout=8.0)
    status = service.status()
    assert status["phase"] == "done", status
    assert status["steps"] >= 3 and len(nav.starts) == status["steps"]
    assert mapping.saved >= status["steps"]
    assert status["coveragePct"] >= 90.0


def test_estop_during_drive_stops_the_loop_and_brakes(monkeypatch):
    monkeypatch.setenv("RC_EXPLORE_POLL_S", "0.01")
    mapping = FakeMap(free_until=1.0)
    nav = FakeNavigation(mapping)
    nav.hold_active = True
    flags = {"latched": False}
    service = ExplorationService(mapping, nav, kinect=FakeKinect(), enabled=True,
                                 estop_check=lambda: {"latched": True} if flags["latched"] else None)
    service.start({"blindZoneChecked": True})
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and not nav.starts:
        time.sleep(0.01)
    assert nav.starts, "поездка должна была начаться"
    flags["latched"] = True
    wait_done(service, timeout=4.0)
    assert service.status()["phase"] in {"error", "cancelled"}
    assert any("E-STOP" in reason for reason in nav.cancels)


def test_stop_cancels_worker(monkeypatch):
    monkeypatch.setenv("RC_EXPLORE_POLL_S", "0.01")
    mapping = FakeMap(free_until=1.0)
    nav = FakeNavigation(mapping)
    nav.hold_active = True
    service = ExplorationService(mapping, nav, kinect=FakeKinect(), enabled=True,
                                 estop_check=lambda: None)
    service.start({"blindZoneChecked": True})
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and not nav.starts:
        time.sleep(0.01)
    reply = service.stop()
    assert reply["ok"] is True
    wait_done(service, timeout=4.0)
    assert nav.active is False


def test_landmarks_capture_four_frames_and_clear(tmp_path):
    cameras = FakeCamera()
    recorder = LandmarkRecorder(lambda: cameras, directory=tmp_path, clock=lambda: 42.0)
    entry = recorder.capture(1, {"x": 1.0, "y": 0.5, "yaw": 0.1}, {"x": 2.0, "y": 0.0}, "departure")
    assert entry is not None and len(entry["frames"]) == 4
    for frame in entry["frames"]:
        assert (tmp_path / frame["file"]).is_file()
    payload = json.loads((tmp_path / "index.json").read_text(encoding="utf-8"))
    assert payload[0]["pose"]["x"] == 1.0
    assert recorder.list() == [entry]
    assert recorder.clear() == 1
    assert recorder.list() == []
    assert not any(tmp_path.glob("step*.jpg"))


def test_real_navigation_autonomous_start_keeps_safety_checks():
    """autonomous=True не обходит preflight, blind zone и исполнитель."""
    nav, mapping, source, lidar = real_nav_service()
    body = {"goal": {"x": 2.5, "y": 0.0}}
    with pytest.raises(NavigationError) as exc:
        nav.start(body)                       # операторский путь требует plan()
    assert exc.value.code == "plan_required"
    reply, code = nav.start({**body, "blindZoneChecked": True}, autonomous=True)
    assert code == 202 and reply["ok"] is True
    state = await_finished(nav)
    assert state["phase"] in {"succeeded", "blocked"}, state
    assert source.brakes >= 2 and source.percent == 0
