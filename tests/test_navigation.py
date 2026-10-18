"""Multi-stop routes must preserve mandatory stops and the existing safety planner."""
import copy

import pytest

from robot_control.navigation import NavigationError, _route_goals, plan_grid_path, plan_route


def grid():
    return {"width": 80, "height": 80, "resolution": 0.1,
            "origin": {"x": -4, "y": -4}, "pose": {"x": 0, "y": 0, "yaw": 0},
            "data": bytes(80 * 80), "revision": 7}


def test_route_preserves_order_and_snapshot():
    snapshot = grid()
    original = copy.deepcopy(snapshot)
    goals = [{"x": 1, "y": 0}, {"x": 2, "y": 0.2}, {"x": 0, "y": 0}]
    plan = plan_route(snapshot, goals, footprint_m=0.42)
    cursor = 0
    current = dict(snapshot)
    for goal in goals:
        leg = plan_grid_path(current, goal["x"], goal["y"], footprint_m=0.42)
        cursor = plan["path"].index(leg["goal"], cursor) + 1
        current["pose"] = {**snapshot["pose"], **leg["goal"]}
    assert plan["goal"] == leg["goal"]
    assert plan["goalsRequested"] == goals
    assert plan["waypoints"] == len(plan["path"]) - 1
    assert plan["distanceM"] > 4
    assert snapshot == original


def test_single_stop_uses_existing_planner():
    snapshot = grid()
    direct = plan_grid_path(snapshot, 1, 0, footprint_m=0.42)
    route = plan_route(snapshot, [{"x": 1, "y": 0}], footprint_m=0.42)
    assert route["path"] == direct["path"]
    assert route["distanceM"] == direct["distanceM"]


def test_blocked_intermediate_stop_cannot_be_skipped():
    snapshot = grid()
    data = bytearray(snapshot["data"])
    data[40 * 80 + 50] = 100
    snapshot["data"] = bytes(data)
    with pytest.raises(NavigationError, match="Точка 1"):
        plan_route(snapshot, [{"x": 1, "y": 0}, {"x": 2, "y": 0}], footprint_m=0.42)


@pytest.mark.parametrize("points", [[], [None], [{"x": float("nan"), "y": 0}],
                                    [{"x": 0, "y": 0}] * 21, [{"x": 2, "y": 0}]])
def test_invalid_route_points(points):
    with pytest.raises(NavigationError):
        _route_goals({"goals": points}, 1, 0)


def test_legacy_goal_payload():
    assert _route_goals({}, 1, 0) == [{"x": 1, "y": 0}]
