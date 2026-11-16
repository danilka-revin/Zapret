"""Векторная карта, маршрут и локализация: без железа и без тяги."""
from __future__ import annotations

import copy
import math
from pathlib import Path

import pytest

from robot_control.road_map import RoadMap, RoadMapError
from robot_control.road_localization import RoadLocalization
from robot_control.road_route import RoadRoutePlanner

ROOT = Path(__file__).resolve().parents[1]
DRAFT = ROOT / "docs" / "delivery_map_draft.json"


def small_map(**overrides):
    payload = {
        "schemaVersion": 1, "mapId": "test", "title": "Тест", "status": "draft",
        "nodes": [
            {"id": "a", "xy": [0, 0], "kind": "dock"},
            {"id": "b", "xy": [10, 0], "kind": "junction"},
            {"id": "c", "xy": [10, 10], "kind": "delivery", "approachYawDeg": 90},
            {"id": "d", "xy": [0, 10], "kind": "junction"},
        ],
        "edges": [
            {"id": "ab", "from": "a", "to": "b", "geometry": [[0, 0], [10, 0]], "direction": "both"},
            {"id": "bc", "from": "b", "to": "c", "geometry": [[10, 0], [10, 10]], "direction": "both"},
            {"id": "ad", "from": "a", "to": "d", "geometry": [[0, 0], [0, 10]], "direction": "both"},
            {"id": "dc", "from": "d", "to": "c", "geometry": [[0, 10], [10, 10]], "direction": "both"},
        ],
    }
    payload.update(overrides)
    return payload


def test_draft_map_loads_and_stays_unscaled():
    road = RoadMap.from_file(DRAFT)
    assert road.status == "draft"
    assert road.units == "map units"
    assert road.distance_m(100) is None
    assert len(road.delivery_nodes()) >= 3
    plan = road.plan({"x": 22, "y": 140, "yawDeg": 80}, "ntc_delivery", snap_tolerance_units=20)
    assert plan["ok"] and plan["hardwareEnabled"] is False
    assert plan["totalDistanceM"] is None
    assert plan["warnings"]


def test_invalid_maps_are_rejected():
    with pytest.raises(RoadMapError):
        RoadMap(small_map(schemaVersion=2))
    duplicate = small_map()
    duplicate["nodes"].append(copy.deepcopy(duplicate["nodes"][0]))
    with pytest.raises(RoadMapError) as exc:
        RoadMap(duplicate)
    assert exc.value.code == "duplicate_node"
    mismatch = small_map()
    mismatch["edges"][0]["geometry"] = [[0, 1], [10, 0]]
    with pytest.raises(RoadMapError) as exc:
        RoadMap(mismatch)
    assert exc.value.code == "geometry_mismatch"
    closed = small_map()
    closed["edges"][0]["closed"] = "false"
    with pytest.raises(RoadMapError):
        RoadMap(closed)
    calibrated = small_map(status="calibrated")
    with pytest.raises(RoadMapError) as exc:
        RoadMap(calibrated)
    assert exc.value.code == "scale_required"


def test_start_requires_road_and_heading():
    road = RoadMap(small_map())
    with pytest.raises(RoadMapError) as exc:
        road.snap_start(50, 50, 0, tolerance_units=1)
    assert exc.value.code == "start_not_on_road"
    forward = road.snap_start(5, 0, 0, tolerance_units=1)
    reverse = road.snap_start(5, 0, 180, tolerance_units=1)
    assert forward.reverse is False and reverse.reverse is True
    with pytest.raises(RoadMapError) as exc:
        road.plan({"x": 5, "y": 0}, "c")
    assert exc.value.code == "heading_required"


def test_route_uses_directed_graph_and_blockers():
    payload = small_map()
    payload["edges"][0]["direction"] = "forward"
    road = RoadMap(payload)
    path = road.shortest_path("a", "c")
    assert [item.edge.id for item in path] == ["ab", "bc"]
    with pytest.raises(RoadMapError) as exc:
        road.shortest_path("b", "a", blocked_edges={"bc", "ad", "dc"})
    assert exc.value.code == "no_route"
    plan = road.plan({"x": 0, "y": 0, "yawDeg": 90}, "c", blocked_edges=["ab"], snap_tolerance_units=1)
    assert [leg["edgeId"] for leg in plan["legs"]] == ["ad", "dc"]
    with pytest.raises(RoadMapError):
        road.plan({"x": 0, "y": 0, "yawDeg": 90}, "c", blocked_edges=["ab", "ad"], snap_tolerance_units=1)


def test_plan_computes_turn_angles_and_calibrated_distance():
    road = RoadMap(small_map())
    plan = road.plan({"x": 0, "y": 0, "yawDeg": 0}, "c")
    turns = [m for m in plan["maneuvers"] if m["instruction"] != "alignment"]
    assert turns[0]["nodeId"] == "b"
    assert turns[0]["turnDeg"] == pytest.approx(90.0)
    assert turns[0]["instruction"] == "left"
    assert plan["totalDistanceUnits"] == pytest.approx(20.0)
    calibrated = small_map(status="calibrated", reference={"scaleMetersPerUnit": 0.5})
    metric = RoadMap(calibrated).plan({"x": 0, "y": 0, "yawDeg": 0}, "c")
    assert metric["units"] == "m"
    assert metric["totalDistanceM"] == pytest.approx(10.0)


def test_planner_never_permits_drive():
    road = RoadMap(small_map())
    planner = RoadRoutePlanner(road)
    plan = planner.plan({"x": 0, "y": 0, "yawDeg": 0}, "c")
    assert plan["drivePermitted"] is False
    assert "dry_run_only" in plan["driveBlockers"]
    assert "map_not_calibrated" in plan["driveBlockers"]


def test_localization_requires_confirmed_junction():
    road = RoadMap(small_map())
    loc = RoadLocalization(road, junction_window_units=2.0)
    ready = loc.set_start(0, 0, 0, tolerance_units=1, heading_tolerance_deg=10)
    assert ready["trusted"] is True
    assert loc.advance(5)["state"] == "tracking"
    waiting = loc.advance(5)
    assert waiting["state"] == "awaiting_junction"
    assert waiting["trusted"] is False
    assert loc.confirm_junction("dc")["reason"] == "junction_mismatch"
    assert loc.trusted is False


def test_localization_confirms_correct_junction_and_lost_stops():
    road = RoadMap(small_map())
    loc = RoadLocalization(road, junction_window_units=2.0)
    loc.set_start(0, 0, 0, tolerance_units=1, heading_tolerance_deg=10)
    loc.advance(10)
    confirmed = loc.confirm_junction("bc")
    assert confirmed["trusted"] is True
    assert confirmed["edgeId"] == "bc"
    assert confirmed["distanceAlongUnits"] == 0
    lost = loc.lose("localization_lost")
    assert lost["trusted"] is False
    assert loc.advance(1)["state"] == "lost"


def test_traffic_light_and_stop_sign_validation():
    payload = small_map()
    payload["trafficLights"] = [
        {"id": "tl_b", "nodeId": "b", "controlledEdges": ["ab"], "stopOffsetUnits": 2,
         "cycle": {"red": 5, "green": 5, "yellow": 1}, "initialState": "red"}
    ]
    payload["stopSigns"] = [
        {"id": "stop_c", "nodeId": "c", "approachEdgeId": "bc", "stopOffsetUnits": 1, "stopDurationS": 2}
    ]
    payload["crosswalks"] = [
        {"id": "cross_ab", "edgeId": "ab", "atDistanceUnits": 5, "widthUnits": 2, "signalized": False}
    ]
    road = RoadMap(payload)
    assert len(road.traffic_lights) == 1
    assert len(road.stop_signs) == 1
    assert len(road.crosswalks) == 1
    # проверка привязки к маршруту
    plan = road.plan({"x": 0, "y": 0, "yawDeg": 0}, "c")
    assert any(ctrl["type"] == "traffic_light" for ctrl in plan["trafficControls"])
    assert any(ctrl["type"] == "stop_sign" for ctrl in plan["trafficControls"])
    assert any(ctrl["type"] == "crosswalk" for ctrl in plan["trafficControls"])
    # неверная привязка
    bad = small_map()
    bad["trafficLights"] = [{"id": "tl_bad", "nodeId": "c", "controlledEdges": ["ab"]}]
    with pytest.raises(RoadMapError):
        RoadMap(bad)
    bad2 = small_map()
    bad2["crosswalks"] = [{"id": "cross_bad", "edgeId": "ab", "atDistanceUnits": 0.1}]
    # atDistance внутри ребра ok, но проверим дубликат id между типами
    bad2["trafficLights"] = [{"id": "cross_bad", "nodeId": "b", "controlledEdges": ["ab"]}]
    with pytest.raises(RoadMapError) as exc:
        RoadMap(bad2)
    assert exc.value.code == "duplicate_traffic_id"


def test_draft_map_has_traffic_infrastructure():
    road = RoadMap.from_file(DRAFT)
    assert road.summary()["trafficLightCount"] >= 1
    assert road.summary()["stopSignCount"] >= 1
    assert road.summary()["crosswalkCount"] >= 1
