"""Подготовка маршрута по векторной карте и контроль его безопасности до поездки.

Модуль строит план и показывает, чего ещё не хватает для реального движения. Он
не включает тягу и не выдаёт команды колёсам. Черновая карта без метрического
масштаба пригодна для выбора цели и проверки топологии, но не для привода.
"""
from __future__ import annotations

import time
from typing import Any, Optional

from .road_map import RoadMap, RoadMapError


class RoadRoutePlanner:
    def __init__(self, road_map: RoadMap) -> None:
        self.road_map = road_map
        self._last_plan: Optional[dict[str, Any]] = None

    def plan(self, start: dict[str, Any], goal_node_id: str, *,
             blocked_edges: Optional[list[str]] = None,
             snap_tolerance_units: float = 25.0,
             heading_tolerance_deg: float = 40.0) -> dict[str, Any]:
        plan = self.road_map.plan(start, goal_node_id, blocked_edges=blocked_edges,
                                  snap_tolerance_units=snap_tolerance_units,
                                  heading_tolerance_deg=heading_tolerance_deg)
        plan["createdAt"] = time.time()
        plan["drivePermitted"] = False
        plan["driveBlockers"] = self.drive_blockers(plan)
        plan["hardwareEnabled"] = False
        self._last_plan = plan
        return plan

    def drive_blockers(self, plan: dict[str, Any]) -> list[str]:
        blockers = ["dry_run_only", "no_motion_arbiter_connected"]
        if self.road_map.status != "calibrated":
            blockers.append("map_not_calibrated")
        if self.road_map.status == "calibrated" and not self.road_map.scale_meters_per_unit:
            blockers.append("scale_missing")
        if not plan.get("ok"):
            blockers.append("route_invalid")
        return blockers

    def state(self) -> dict[str, Any]:
        return {"map": self.road_map.summary(), "lastPlan": self._last_plan,
                "hardwareEnabled": False, "tractionConnected": False}
