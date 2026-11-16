#!/usr/bin/env python3
"""Сухой прогон доставки по векторной карте без железа.

    ./scripts/delivery_sim.py
    ./scripts/delivery_sim.py --scenario pothole_blocked
    ./scripts/delivery_sim.py --json
    ./scripts/delivery_sim.py --goal south_east --start-x 22 --start-y 140 --start-yaw 80

Проверяются граф дорог, стартовая поза, углы поворотов, автомат миссии и реакции
на синтетические препятствия. Тяга, колёса, E-STOP железа и USB не открываются.
Это не замена проверки дорожного восприятия и тормозного пути на реальном роботе.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_control.delivery_sim import SCENARIOS, simulate_route
from robot_control.road_map import RoadMap, RoadMapError

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MAP = ROOT / "docs" / "delivery_map_draft.json"
EXPECTED = {
    "clear": "done",
    "bump": "done",
    "pothole_bypass": "done",
    "pothole_blocked": "blocked_by_road_hazard",
    "moving_object": "done",
    "localization_lost": "localization_lost",
    "traffic_light_red": "done",
    "traffic_light_green": "done",
    "stop_sign": "done",
    "pedestrian_crossing": "done",
    "speed_limit": "done",
}


def run_simulation(map_path: Path | str = DEFAULT_MAP, *, start_x: float = 22.0,
                   start_y: float = 140.0, start_yaw: float = 80.0,
                   goal: str = "ntc_delivery", scenarios=None) -> dict:
    selected = list(scenarios or sorted(SCENARIOS))
    report = {"ok": False, "map": str(map_path), "goal": goal, "results": [],
              "tractionConnected": False, "hardwareEnabled": False}
    try:
        road = RoadMap.from_file(map_path)
        route = road.plan({"x": start_x, "y": start_y, "yawDeg": start_yaw}, goal,
                          snap_tolerance_units=25.0, heading_tolerance_deg=40.0)
    except RoadMapError as exc:
        report["error"] = exc.payload()
        return report
    report["route"] = {"totalDistanceUnits": route["totalDistanceUnits"],
                       "totalDistanceM": route["totalDistanceM"],
                       "legs": len(route["legs"]), "maneuvers": len(route["maneuvers"])}
    all_ok = True
    for scenario in selected:
        result = simulate_route(route, scenario, road_map=road)
        expected = EXPECTED.get(scenario)
        passed = result["finalState"] == expected and result["tractionConnected"] is False
        all_ok = all_ok and passed
        report["results"].append({"scenario": scenario, "passed": passed,
                                  "ok": result["ok"], "finalState": result["finalState"],
                                  "missionState": result["missionState"],
                                  "distanceUnits": result["distanceUnits"],
                                  "elapsedS": result["elapsedS"], "events": len(result["events"]),
                                  "maxLateralOffsetUnits": result["maxLateralOffsetUnits"]})
    report["ok"] = all_ok
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", default=str(DEFAULT_MAP))
    parser.add_argument("--start-x", type=float, default=22.0)
    parser.add_argument("--start-y", type=float, default=140.0)
    parser.add_argument("--start-yaw", type=float, default=80.0)
    parser.add_argument("--goal", default="ntc_delivery")
    parser.add_argument("--scenario", choices=sorted(SCENARIOS) + ["all"], default="all")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    scenarios = None if args.scenario == "all" else [args.scenario]
    report = run_simulation(args.map, start_x=args.start_x, start_y=args.start_y,
                            start_yaw=args.start_yaw, goal=args.goal, scenarios=scenarios)
    if args.json:
        print(json.dumps(report, ensure_ascii=False))
        return 0 if report["ok"] else 1
    print("Сухой прогон доставки по векторной карте (без железа)")
    if report.get("error"):
        print("  ошибка:", report["error"].get("error"))
        return 1
    route = report["route"]
    print("  маршрут: %s ед. карты, участков %s, манёвров %s" %
          (route["totalDistanceUnits"], route["legs"], route["maneuvers"]))
    for item in report["results"]:
        print("  %-18s %s → %s (%s событий)" %
              (item["scenario"], "OK" if item["passed"] else "ПРОБЛЕМА",
               item["finalState"], item["events"]))
    print("  итог:", "логика проверена, тяга не подключалась" if report["ok"] else "есть проблемы")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
