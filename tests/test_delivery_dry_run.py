"""Автомат миссии, сухой прогон и сервис доставки: тяга не подключается."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from robot_control.delivery_mission import DeliveryMission
from robot_control.delivery_service import DeliveryError, DeliveryService
from robot_control.delivery_sim import (SCENARIOS, SimulationConfig, decide_ground_hazard,
                                        pothole_bypass, simulate_route, slowdown_distance,
                                        stopping_distance)
from robot_control.road_map import RoadMap

ROOT = Path(__file__).resolve().parents[1]
DRAFT = ROOT / "docs" / "delivery_map_draft.json"


def route():
    road = RoadMap.from_file(DRAFT)
    return road, road.plan({"x": 22, "y": 140, "yawDeg": 80}, "ntc_delivery", snap_tolerance_units=20)


def test_mission_order_and_no_traction():
    _, plan = route()
    mission = DeliveryMission(plan)
    assert mission.start({"route_valid": True})["state"] == "FOLLOW_ROAD"
    assert mission.confirm_junction("west_mid")["state"] == "STOP_SAFE"
    mission = DeliveryMission(plan)
    mission.start({"route_valid": True})
    mission.reach_junction("west_mid")
    assert mission.confirm_junction("wrong")["reason"] == "junction_mismatch"
    assert mission.to_dict()["tractionConnected"] is False


def test_mission_preflight_failure_stops():
    _, plan = route()
    mission = DeliveryMission(plan)
    result = mission.start({"route_valid": False})
    assert result["state"] == "STOP_SAFE"
    assert result["reason"] == "preflight_failed"


def test_stopping_and_slowdown_formulas():
    cfg = SimulationConfig()
    assert stopping_distance(10, cfg) == pytest.approx(10 * cfg.latency_s + 100 / (2 * cfg.decel_units_s2) + cfg.stop_margin_units)
    assert slowdown_distance(10, 5, cfg) < stopping_distance(10, cfg)


def test_pothole_bypass_requires_full_footprint_inside_road():
    cfg = SimulationConfig()
    small = {"bbox": [10, -1, 12, 1]}
    assert pothole_bypass(small, cfg) is not None
    wide = {"bbox": [10, -cfg.road_half_width_units, 12, cfg.road_half_width_units]}
    assert pothole_bypass(wide, cfg) is None
    decision = decide_ground_hazard(wide, 1.0, cfg.speed_units_per_s, cfg)
    assert decision["action"] == "stop"


@pytest.mark.parametrize("scenario,ok,final", [
    ("clear", True, "done"),
    ("bump", True, "done"),
    ("pothole_bypass", True, "done"),
    ("pothole_blocked", False, "blocked_by_road_hazard"),
    ("moving_object", True, "done"),
    ("localization_lost", False, "localization_lost"),
    ("traffic_light_red", True, "done"),
    ("traffic_light_green", True, "done"),
    ("stop_sign", True, "done"),
    ("pedestrian_crossing", True, "done"),
    ("speed_limit", True, "done"),
])
def test_simulation_scenarios(scenario, ok, final):
    road, plan = route()
    result = simulate_route(plan, scenario, road_map=road)
    assert result["ok"] is ok
    assert result["finalState"] == final
    assert result["tractionConnected"] is False
    assert result["hardwareEnabled"] is False
    assert result["events"]
    if scenario == "pothole_bypass":
        assert result["maxLateralOffsetUnits"] > 0
    if scenario == "bump":
        assert any(event["state"] == "SLOW_FOR_BUMP" for event in result["events"])
    if scenario == "moving_object":
        assert any(event["state"] == "YIELD" for event in result["events"])
    if scenario == "traffic_light_red":
        assert any(event["state"] == "WAIT_TRAFFIC_LIGHT" for event in result["events"])
    if scenario == "pedestrian_crossing":
        assert any(event["state"] == "WAIT_PEDESTRIAN" for event in result["events"])
    if scenario == "stop_sign":
        assert any(event["state"] == "WAIT_STOP" for event in result["events"])
    if scenario == "speed_limit":
        assert any(event["state"] == "SPEED_LIMIT" for event in result["events"])


def test_service_setup_plan_simulate_and_reset(tmp_path):
    service = DeliveryService(map_path=DRAFT, directory=tmp_path / "delivery")
    body = {"start": {"x": 22, "y": 140, "yawDeg": 80}, "goalNodeId": "ntc_delivery",
            "scenario": "clear", "draftAcknowledged": True}
    saved = service.save_setup(body)
    assert saved["ok"] and saved["plan"]["hardwareEnabled"] is False
    assert (tmp_path / "delivery" / "setup.json").is_file()
    planned = service.plan()
    assert planned["plan"]["totalDistanceM"] is None
    simulated = service.simulate({"scenario": "bump"})
    assert simulated["simulation"]["ok"] is True
    assert simulated["status"]["drivePermitted"] is False
    reset = service.reset()
    assert reset["status"]["lastSimulation"] is None
    assert reset["status"]["setup"]["goalNodeId"] == "ntc_delivery"


def test_service_requires_draft_acknowledgement_for_simulation(tmp_path):
    service = DeliveryService(map_path=DRAFT, directory=tmp_path / "delivery")
    service.save_setup({"start": {"x": 22, "y": 140, "yawDeg": 80}, "goalNodeId": "ntc_delivery"})
    with pytest.raises(DeliveryError) as exc:
        service.simulate()
    assert exc.value.code == "draft_not_acknowledged"


def test_service_rejects_roof_goal_and_missing_map(tmp_path):
    service = DeliveryService(map_path=DRAFT, directory=tmp_path / "delivery")
    with pytest.raises(DeliveryError) as exc:
        service.save_setup({"start": {"x": 22, "y": 140, "yawDeg": 80}, "goalNodeId": "north_mid"})
    assert exc.value.code == "invalid_goal_kind"
    missing = DeliveryService(map_path=tmp_path / "missing.json", directory=tmp_path / "delivery2")
    assert missing.map_payload()["ok"] is False
    with pytest.raises(DeliveryError) as exc:
        missing.plan()
    assert exc.value.code == "map_unavailable"
