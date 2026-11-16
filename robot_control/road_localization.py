"""Привязка к векторной дороге; состояние и проверки, без управления колёсами.

Стартовая поза задаётся человеком до поездки. Дальше продольный путь приходит от
одометрии, курс — от гироскопа, боковая ошибка — от камеры, а узел подтверждается
отдельным событием. Потеря доверия означает STOP_SAFE, а не «выбрать ближайшую
дорогу». Единицы остаются единицами карты до подтверждения масштаба.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional

from .road_map import RoadEdge, RoadMap, RoadMapError, angle_deg, normalize_angle


def _finite(value: Any, name: str, lo: float, hi: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name}: ожидается число") from None
    if not math.isfinite(result) or not lo <= result <= hi:
        raise ValueError(f"{name}: число вне диапазона")
    return result


def _tangent_at(edge: RoadEdge, distance: float, reverse: bool) -> float:
    geometry = edge.geometry if not reverse else tuple(reversed(edge.geometry))
    travelled = 0.0
    for a, b in zip(geometry, geometry[1:]):
        length = math.dist(a, b)
        if travelled + length >= distance - 1e-9:
            return math.atan2(b[1] - a[1], b[0] - a[0])
        travelled += length
    a, b = geometry[-2], geometry[-1]
    return math.atan2(b[1] - a[1], b[0] - a[0])


@dataclass
class RoadLocalization:
    road_map: RoadMap
    junction_window_units: float = 10.0
    max_lateral_units: float = 12.0
    max_yaw_error_deg: float = 35.0
    state: str = "uninitialized"
    edge_id: Optional[str] = None
    reverse: bool = False
    distance_along_units: float = 0.0
    lateral_units: float = 0.0
    yaw_deg: float = 0.0
    yaw_error_deg: float = 0.0
    confidence: float = 0.0
    reason: str = ""

    def __post_init__(self) -> None:
        self.junction_window_units = _finite(self.junction_window_units, "junction_window_units", 0.0, 1e6)
        self.max_lateral_units = _finite(self.max_lateral_units, "max_lateral_units", 1e-6, 1e6)
        self.max_yaw_error_deg = _finite(self.max_yaw_error_deg, "max_yaw_error_deg", 0.0, 180.0)

    @property
    def trusted(self) -> bool:
        return self.state in {"ready", "tracking"} and self.confidence >= 0.55

    @property
    def edge(self) -> Optional[RoadEdge]:
        return self.road_map.edges.get(self.edge_id) if self.edge_id is not None else None

    def set_start(self, x: float, y: float, yaw_deg: float, *, tolerance_units: float = 25.0,
                  heading_tolerance_deg: float = 40.0) -> dict[str, Any]:
        try:
            snap = self.road_map.snap_start(x, y, yaw_deg, tolerance_units=tolerance_units,
                                           heading_tolerance_deg=heading_tolerance_deg)
        except RoadMapError as exc:
            self.state = "lost"
            self.confidence = 0.0
            self.reason = exc.code
            raise
        self.edge_id = snap.edge.id
        self.reverse = snap.reverse
        self.distance_along_units = snap.distance_along
        self.lateral_units = snap.lateral_distance
        self.yaw_deg = _finite(yaw_deg, "yaw_deg", -360.0, 360.0)
        self.yaw_error_deg = angle_deg(normalize_angle(math.radians(self.yaw_deg) - math.radians(snap.tangent_deg)))
        position_quality = 1.0 - abs(snap.lateral_distance) / max(1e-6, tolerance_units)
        heading_quality = 1.0 - abs(snap.heading_error_deg or 0.0) / max(1e-6, heading_tolerance_deg)
        self.confidence = max(0.0, min(1.0, 0.5 * position_quality + 0.5 * heading_quality))
        self.state = "ready"
        if not self.trusted:
            self.state = "lost"
            self.reason = "start_confidence_low"
        else:
            self.reason = ""
        return self.to_dict()

    def advance(self, distance_units: float, *, yaw_delta_deg: float = 0.0) -> dict[str, Any]:
        distance_units = _finite(distance_units, "distance_units", 0.0, 1e6)
        yaw_delta_deg = _finite(yaw_delta_deg, "yaw_delta_deg", -360.0, 360.0)
        if not self.trusted or self.edge is None:
            return self.lose("localization_not_ready")
        self.distance_along_units += distance_units
        self.yaw_deg = angle_deg(normalize_angle(math.radians(self.yaw_deg) + math.radians(yaw_delta_deg)))
        length = self.edge.length_units
        expected = _tangent_at(self.edge, min(self.distance_along_units, length), self.reverse)
        self.yaw_error_deg = angle_deg(normalize_angle(math.radians(self.yaw_deg) - expected))
        if abs(self.lateral_units) > self.max_lateral_units or abs(self.yaw_error_deg) > self.max_yaw_error_deg:
            return self.lose("road_departure")
        if self.distance_along_units > length + self.junction_window_units:
            return self.lose("junction_confirmation_timeout")
        if self.distance_along_units >= length:
            self.state = "awaiting_junction"
            self.reason = "junction_confirmation_required"
            self.confidence = min(self.confidence, 0.5)
        else:
            self.state = "tracking"
            self.reason = ""
            self.confidence = max(0.55, self.confidence - 0.001 * distance_units)
        return self.to_dict()

    def correct(self, *, lateral_units: Optional[float] = None,
                yaw_error_deg: Optional[float] = None,
                confidence: Optional[float] = None) -> dict[str, Any]:
        if self.state == "lost":
            return self.to_dict()
        if lateral_units is not None:
            self.lateral_units = _finite(lateral_units, "lateral_units", -1e6, 1e6)
        if yaw_error_deg is not None:
            self.yaw_error_deg = _finite(yaw_error_deg, "yaw_error_deg", -180.0, 180.0)
            if self.edge is not None:
                expected = _tangent_at(self.edge, min(self.distance_along_units, self.edge.length_units), self.reverse)
                self.yaw_deg = angle_deg(normalize_angle(expected + math.radians(self.yaw_error_deg)))
        if confidence is not None:
            self.confidence = _finite(confidence, "confidence", 0.0, 1.0)
        if abs(self.lateral_units) > self.max_lateral_units or abs(self.yaw_error_deg) > self.max_yaw_error_deg:
            return self.lose("road_departure")
        if self.confidence < 0.55:
            return self.lose("localization_confidence_low")
        return self.to_dict()

    def confirm_junction(self, edge_id: str, *, reverse: bool = False) -> dict[str, Any]:
        if edge_id not in self.road_map.edges:
            return self.lose("unknown_edge")
        next_edge = self.road_map.edges[edge_id]
        if next_edge.closed or next_edge.direction == ("forward" if reverse else "reverse"):
            return self.lose("edge_not_allowed")
        if self.edge is not None and self.state != "uninitialized":
            expected_node = self.edge.to_node if not self.reverse else self.edge.from_node
            actual_node = next_edge.from_node if not reverse else next_edge.to_node
            if expected_node != actual_node:
                return self.lose("junction_mismatch")
            if self.distance_along_units < self.edge.length_units - self.junction_window_units:
                return self.lose("junction_too_early")
        self.edge_id = edge_id
        self.reverse = reverse
        self.distance_along_units = 0.0
        self.lateral_units = 0.0
        self.yaw_error_deg = 0.0
        self.yaw_deg = angle_deg(_tangent_at(next_edge, 0.0, reverse))
        self.confidence = 0.75
        self.state = "tracking"
        self.reason = ""
        return self.to_dict()

    def lose(self, reason: str) -> dict[str, Any]:
        self.state = "lost"
        self.confidence = 0.0
        self.reason = reason
        return self.to_dict()

    def to_dict(self) -> dict[str, Any]:
        edge = self.edge
        remaining = None if edge is None else round(edge.length_units - self.distance_along_units, 4)
        return {"state": self.state, "trusted": self.trusted, "edgeId": self.edge_id,
                "reverse": self.reverse, "distanceAlongUnits": round(self.distance_along_units, 4),
                "remainingUnits": remaining, "lateralUnits": round(self.lateral_units, 4),
                "yawDeg": round(self.yaw_deg, 3), "yawErrorDeg": round(self.yaw_error_deg, 3),
                "confidence": round(self.confidence, 3), "reason": self.reason,
                "expectedNodeId": None if edge is None else (edge.from_node if self.reverse else edge.to_node),
                "units": self.road_map.units, "tractionConnected": False}
