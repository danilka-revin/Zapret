"""Векторная карта проездов и маршрут по графу.

Модуль намеренно не знает о лидаре, Kinect, колёсах и HTTP. Он проверяет схему
дорог, привязывает стартовую позу к ребру и считает маршрут до узла доставки.
Результат пригоден для предпросмотра и сухого прогона; разрешение тяги здесь
не выдаётся и выдано быть не может.

Координаты карты: X вправо, Y вверх, угол 0 вдоль +X, положительный поворот
против часовой стрелки. Если карта нарисована по снимку экрана, единицы могут
быть пикселями/условными единицами до подтверждения масштаба.
"""
from __future__ import annotations

import heapq
import json
import math
import os
import re
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Optional

SCHEMA_VERSION = 1
MAX_NODES = 500
MAX_EDGES = 1000
MAX_GEOMETRY_POINTS = 20000
ID_RE = re.compile(r"[A-Za-z0-9_-]{1,64}\Z")
DIRECTIONS = {"both", "forward", "reverse"}
NODE_KINDS = {"junction", "delivery", "dock", "start", "gate"}


class RoadMapError(ValueError):
    """Ошибка схемы или маршрутизации с машинным кодом."""

    def __init__(self, code: str, message: str, status: int = 400):
        super().__init__(message)
        self.code = code
        self.message = message
        self.status = status

    def payload(self) -> dict[str, Any]:
        return {"ok": False, "code": self.code, "error": self.message}


def _number(value: Any, name: str, lo: float = -1e7, hi: float = 1e7) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise RoadMapError("invalid_number", f"{name}: ожидается число") from None
    if not math.isfinite(result) or not lo <= result <= hi:
        raise RoadMapError("invalid_number", f"{name}: число вне допустимого диапазона")
    return result


def _text(value: Any, name: str, limit: int = 120) -> str:
    if not isinstance(value, str):
        raise RoadMapError("invalid_text", f"{name}: ожидается строка")
    result = value.strip()
    if len(result) > limit:
        raise RoadMapError("invalid_text", f"{name}: строка длиннее {limit} символов")
    return result


def _identifier(value: Any, name: str) -> str:
    result = _text(value, name, 64)
    if not ID_RE.fullmatch(result):
        raise RoadMapError("invalid_id", f"{name}: допустимы буквы, цифры, _ и -, до 64 символов")
    return result


def normalize_angle(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


def angle_deg(rad: float) -> float:
    return math.degrees(normalize_angle(rad))


def _point(raw: Any, name: str) -> tuple[float, float]:
    if not isinstance(raw, (list, tuple)) or len(raw) != 2:
        raise RoadMapError("invalid_point", f"{name}: ожидается пара [x, y]")
    return _number(raw[0], f"{name}.x"), _number(raw[1], f"{name}.y")


def _polyline(raw: Any, name: str) -> tuple[tuple[float, float], ...]:
    if not isinstance(raw, (list, tuple)) or len(raw) < 2:
        raise RoadMapError("invalid_geometry", f"{name}: нужно не меньше двух точек")
    points = tuple(_point(item, f"{name}[{i}]") for i, item in enumerate(raw))
    if any(math.dist(a, b) <= 1e-9 for a, b in zip(points, points[1:])):
        raise RoadMapError("invalid_geometry", f"{name}: соседние точки совпадают")
    return points


def polyline_length(points: Iterable[tuple[float, float]]) -> float:
    items = list(points)
    return sum(math.dist(a, b) for a, b in zip(items, items[1:]))


def _truncate(points: tuple[tuple[float, float], ...], start: float,
              end: float) -> tuple[tuple[float, float], ...]:
    """Кусок ломаной от расстояния start до end вдоль неё."""
    if end <= start:
        return (points[0],)
    result: list[tuple[float, float]] = []
    travelled = 0.0
    started = False
    for a, b in zip(points, points[1:]):
        seg = math.dist(a, b)
        seg_end = travelled + seg
        if not started and start <= seg_end:
            t = max(0.0, min(1.0, (start - travelled) / seg))
            result.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
            started = True
        if started and end <= seg_end:
            t = max(0.0, min(1.0, (end - travelled) / seg))
            result.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
            break
        if started:
            result.append(b)
        travelled = seg_end
    if len(result) < 2:
        return (result[0], result[0]) if result else (points[0], points[0])
    # Убираем повтор последней точки при точном попадании в узел.
    cleaned = [result[0]]
    for item in result[1:]:
        if math.dist(cleaned[-1], item) > 1e-9:
            cleaned.append(item)
    return tuple(cleaned) if len(cleaned) >= 2 else (cleaned[0], cleaned[0])


@dataclass(frozen=True)
class RoadNode:
    id: str
    label: str
    x: float
    y: float
    kind: str = "junction"
    approach_yaw_deg: Optional[float] = None

    def to_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {"id": self.id, "label": self.label,
                                  "xy": [self.x, self.y], "kind": self.kind}
        if self.approach_yaw_deg is not None:
            result["approachYawDeg"] = self.approach_yaw_deg
        return result


@dataclass(frozen=True)
class RoadEdge:
    id: str
    label: str
    from_node: str
    to_node: str
    geometry: tuple[tuple[float, float], ...]
    direction: str = "both"
    width_units: Optional[float] = None
    speed_limit_mps: Optional[float] = None
    closed: bool = False

    @property
    def length_units(self) -> float:
        return polyline_length(self.geometry)

    def to_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "id": self.id, "label": self.label, "from": self.from_node,
            "to": self.to_node, "geometry": [list(p) for p in self.geometry],
            "direction": self.direction, "closed": self.closed,
        }
        if self.width_units is not None:
            result["widthUnits"] = self.width_units
        if self.speed_limit_mps is not None:
            result["speedLimitMps"] = self.speed_limit_mps
        return result


@dataclass(frozen=True)
class DirectedEdge:
    edge: RoadEdge
    reverse: bool
    from_node: str
    to_node: str
    geometry: tuple[tuple[float, float], ...]

    @property
    def length_units(self) -> float:
        return polyline_length(self.geometry)


@dataclass(frozen=True)
class StartSnap:
    edge: RoadEdge
    reverse: bool
    point: tuple[float, float]
    distance_along: float
    lateral_distance: float
    heading_error_deg: Optional[float]
    tangent_deg: float


class RoadMap:
    """Проверенная векторная схема проездов."""

    def __init__(self, payload: dict[str, Any]):
        if not isinstance(payload, dict):
            raise RoadMapError("invalid_map", "Карта должна быть JSON-объектом")
        if payload.get("schemaVersion") != SCHEMA_VERSION:
            raise RoadMapError("unsupported_schema", f"Поддерживается schemaVersion={SCHEMA_VERSION}")
        self.map_id = _identifier(payload.get("mapId"), "mapId")
        self.title = _text(payload.get("title") or self.map_id, "title")
        self.status = _text(payload.get("status") or "draft", "status", 32)
        if self.status not in {"draft", "calibrated"}:
            raise RoadMapError("invalid_status", "status: допустимо draft или calibrated")
        reference = payload.get("reference") or {}
        if not isinstance(reference, dict):
            raise RoadMapError("invalid_reference", "reference должен быть объектом")
        self.reference = dict(reference)
        scale = reference.get("scaleMetersPerUnit")
        self.scale_meters_per_unit: Optional[float]
        if scale is None:
            self.scale_meters_per_unit = None
        else:
            self.scale_meters_per_unit = _number(scale, "reference.scaleMetersPerUnit", 1e-6, 1000.0)
        if self.status == "calibrated" and self.scale_meters_per_unit is None:
            raise RoadMapError("scale_required", "Для calibrated нужен reference.scaleMetersPerUnit")
        self.notes = _text(payload.get("notes") or "", "notes", 2000)
        raw_nodes = payload.get("nodes")
        raw_edges = payload.get("edges")
        if not isinstance(raw_nodes, list) or not 1 <= len(raw_nodes) <= MAX_NODES:
            raise RoadMapError("invalid_nodes", f"nodes: от 1 до {MAX_NODES} узлов")
        if not isinstance(raw_edges, list) or len(raw_edges) > MAX_EDGES:
            raise RoadMapError("invalid_edges", f"edges: не больше {MAX_EDGES} рёбер")
        nodes: dict[str, RoadNode] = {}
        for index, raw in enumerate(raw_nodes):
            if not isinstance(raw, dict):
                raise RoadMapError("invalid_node", f"nodes[{index}]: ожидается объект")
            node_id = _identifier(raw.get("id"), f"nodes[{index}].id")
            if node_id in nodes:
                raise RoadMapError("duplicate_node", f"Узел {node_id} повторяется")
            x, y = _point(raw.get("xy"), f"nodes[{index}].xy")
            kind = _text(raw.get("kind") or "junction", f"nodes[{index}].kind", 32)
            if kind not in NODE_KINDS:
                raise RoadMapError("invalid_node_kind", f"Неизвестный kind узла: {kind}")
            approach = raw.get("approachYawDeg")
            approach_yaw = None if approach is None else _number(approach, "approachYawDeg", -360.0, 360.0)
            nodes[node_id] = RoadNode(node_id, _text(raw.get("label") or node_id, "label"),
                                      x, y, kind, approach_yaw)
        edges: dict[str, RoadEdge] = {}
        geometry_points = 0
        for index, raw in enumerate(raw_edges):
            if not isinstance(raw, dict):
                raise RoadMapError("invalid_edge", f"edges[{index}]: ожидается объект")
            edge_id = _identifier(raw.get("id"), f"edges[{index}].id")
            if edge_id in edges:
                raise RoadMapError("duplicate_edge", f"Ребро {edge_id} повторяется")
            from_node = _identifier(raw.get("from"), f"edges[{index}].from")
            to_node = _identifier(raw.get("to"), f"edges[{index}].to")
            if from_node not in nodes or to_node not in nodes:
                raise RoadMapError("unknown_node", f"Ребро {edge_id} ссылается на неизвестный узел")
            if from_node == to_node:
                raise RoadMapError("invalid_edge", f"Ребро {edge_id}: петля нулевой длины запрещена")
            geometry = _polyline(raw.get("geometry"), f"edges[{index}].geometry")
            geometry_points += len(geometry)
            if geometry_points > MAX_GEOMETRY_POINTS:
                raise RoadMapError("map_too_large", "Слишком много точек геометрии")
            if math.dist(geometry[0], (nodes[from_node].x, nodes[from_node].y)) > 1e-6 or \
                    math.dist(geometry[-1], (nodes[to_node].x, nodes[to_node].y)) > 1e-6:
                raise RoadMapError("geometry_mismatch", f"Геометрия {edge_id} не начинается/заканчивается в своих узлах")
            direction = _text(raw.get("direction") or "both", "direction", 16)
            if direction not in DIRECTIONS:
                raise RoadMapError("invalid_direction", f"direction ребра {edge_id}: {direction}")
            width = raw.get("widthUnits")
            width_units = None if width is None else _number(width, "widthUnits", 1e-6, 1e6)
            speed = raw.get("speedLimitMps")
            speed_limit = None if speed is None else _number(speed, "speedLimitMps", 0.01, 30.0)
            closed_raw = raw.get("closed", False)
            if not isinstance(closed_raw, bool):
                raise RoadMapError("invalid_closed", f"closed ребра {edge_id}: ожидается true/false")
            closed = closed_raw
            edges[edge_id] = RoadEdge(edge_id, _text(raw.get("label") or edge_id, "label"),
                                      from_node, to_node, geometry, direction,
                                      width_units, speed_limit, closed)
        self.nodes = nodes
        self.edges = edges
        # ПДД-инфраструктура (опционально, для сухого прогона без тяги)
        try:
            from .road_traffic import _validate_crosswalks, _validate_stop_signs, _validate_traffic_lights
        except ImportError:
            _validate_traffic_lights = _validate_stop_signs = _validate_crosswalks = None  # type: ignore
        if _validate_traffic_lights is not None:
            self.traffic_lights = _validate_traffic_lights(payload.get("trafficLights"), nodes, edges)
            self.stop_signs = _validate_stop_signs(payload.get("stopSigns"), nodes, edges)
            self.crosswalks = _validate_crosswalks(payload.get("crosswalks"), nodes, edges, self.traffic_lights)
            # Кросс-проверка: идентификаторы ПДД не должны пересекаться между типами
            all_ids = set(self.traffic_lights) | set(self.stop_signs) | set(self.crosswalks)
            if len(all_ids) != len(self.traffic_lights) + len(self.stop_signs) + len(self.crosswalks):
                raise RoadMapError("duplicate_traffic_id", "Идентификатор ПДД-объекта повторяется между типами")
        else:
            self.traffic_lights = {}
            self.stop_signs = {}
            self.crosswalks = {}
        self.payload = self.to_dict()

    @classmethod
    def from_json(cls, text: str) -> "RoadMap":
        try:
            payload = json.loads(text)
        except ValueError as exc:
            raise RoadMapError("invalid_json", "Карта не является корректным JSON") from exc
        return cls(payload)

    @classmethod
    def from_file(cls, path: Path | str) -> "RoadMap":
        try:
            return cls.from_json(Path(path).read_text(encoding="utf-8"))
        except (OSError, UnicodeError) as exc:
            raise RoadMapError("map_unavailable", f"Не удалось прочитать карту: {exc}", 503) from exc

    def to_dict(self) -> dict[str, Any]:
        reference = dict(self.reference)
        result: dict[str, Any] = {
            "schemaVersion": SCHEMA_VERSION,
            "mapId": self.map_id,
            "title": self.title,
            "status": self.status,
            "reference": reference,
            "notes": self.notes,
            "nodes": [node.to_dict() for node in self.nodes.values()],
            "edges": [edge.to_dict() for edge in self.edges.values()],
        }
        if getattr(self, "traffic_lights", None):
            result["trafficLights"] = [light.to_dict() for light in self.traffic_lights.values()]
        if getattr(self, "stop_signs", None):
            result["stopSigns"] = [sign.to_dict() for sign in self.stop_signs.values()]
        if getattr(self, "crosswalks", None):
            result["crosswalks"] = [cw.to_dict() for cw in self.crosswalks.values()]
        return result

    @property
    def units(self) -> str:
        return "m" if self.status == "calibrated" and self.scale_meters_per_unit is not None else "map units"

    def distance_m(self, distance_units: float) -> Optional[float]:
        if self.status != "calibrated" or self.scale_meters_per_unit is None:
            return None
        return distance_units * self.scale_meters_per_unit

    def summary(self) -> dict[str, Any]:
        return {
            "mapId": self.map_id, "title": self.title, "status": self.status,
            "units": self.units, "scaleMetersPerUnit": self.scale_meters_per_unit,
            "nodeCount": len(self.nodes), "edgeCount": len(self.edges),
            "deliveryNodes": [node.to_dict() for node in self.delivery_nodes()],
            "reference": dict(self.reference), "notes": self.notes,
            "hardwareEnabled": False,
            "trafficLightCount": len(getattr(self, "traffic_lights", {})),
            "stopSignCount": len(getattr(self, "stop_signs", {})),
            "crosswalkCount": len(getattr(self, "crosswalks", {})),
        }

    def directed_edges(self, *, include_closed: bool = False) -> list[DirectedEdge]:
        result: list[DirectedEdge] = []
        for edge in self.edges.values():
            if edge.closed and not include_closed:
                continue
            if edge.direction in {"both", "forward"}:
                result.append(DirectedEdge(edge, False, edge.from_node, edge.to_node, edge.geometry))
            if edge.direction in {"both", "reverse"}:
                result.append(DirectedEdge(edge, True, edge.to_node, edge.from_node,
                                           tuple(reversed(edge.geometry))))
        return result

    def delivery_nodes(self) -> list[RoadNode]:
        return [node for node in self.nodes.values() if node.kind in {"delivery", "dock"}]

    def _snap_candidates(self, x: float, y: float, yaw_deg: Optional[float], *,
                         tolerance_units: float, heading_tolerance_deg: float,
                         blocked_edges: Optional[set[str]] = None) -> list[StartSnap]:
        x = _number(x, "start.x")
        y = _number(y, "start.y")
        tolerance_units = _number(tolerance_units, "snapToleranceUnits", 1e-6, 1e6)
        heading_tolerance_deg = _number(heading_tolerance_deg, "headingToleranceDeg", 0.0, 180.0)
        yaw = None if yaw_deg is None else math.radians(_number(yaw_deg, "start.yawDeg", -360.0, 360.0))
        blocked_edges = blocked_edges or set()
        candidates: list[StartSnap] = []
        for directed in self.directed_edges():
            if directed.edge.id in blocked_edges:
                continue
            travelled = 0.0
            best_on_edge: Optional[StartSnap] = None
            for a, b in zip(directed.geometry, directed.geometry[1:]):
                dx, dy = b[0] - a[0], b[1] - a[1]
                length = math.hypot(dx, dy)
                t = 0.0 if length <= 1e-12 else max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / (length * length)))
                px, py = a[0] + dx * t, a[1] + dy * t
                lateral = math.hypot(x - px, y - py)
                tangent = math.atan2(dy, dx)
                heading_error = None if yaw is None else abs(angle_deg(normalize_angle(yaw - tangent)))
                if lateral <= tolerance_units and (heading_error is None or heading_error <= heading_tolerance_deg):
                    candidate = StartSnap(directed.edge, directed.reverse, (px, py),
                                          travelled + length * t, lateral, heading_error,
                                          angle_deg(tangent))
                    key = (candidate.lateral_distance, candidate.heading_error_deg or 0.0)
                    if best_on_edge is None or key < (best_on_edge.lateral_distance,
                                                      best_on_edge.heading_error_deg or 0.0):
                        best_on_edge = candidate
                travelled += length
            if best_on_edge is not None:
                candidates.append(best_on_edge)
        candidates.sort(key=lambda item: (item.lateral_distance, item.heading_error_deg or 0.0,
                                          item.edge.id, item.reverse))
        return candidates

    def snap_start(self, x: float, y: float, yaw_deg: Optional[float] = None, *,
                   tolerance_units: float = 25.0,
                   heading_tolerance_deg: float = 40.0) -> StartSnap:
        candidates = self._snap_candidates(x, y, yaw_deg, tolerance_units=tolerance_units,
                                           heading_tolerance_deg=heading_tolerance_deg)
        if not candidates:
            raise RoadMapError("start_not_on_road",
                               "Старт не лежит на разрешённом проезде с заданным курсом. Поставьте точку на дорогу и укажите направление.",
                               409)
        return candidates[0]

    def shortest_path(self, from_node: str, to_node: str,
                      blocked_edges: Optional[set[str]] = None) -> list[DirectedEdge]:
        if from_node not in self.nodes or to_node not in self.nodes:
            raise RoadMapError("unknown_node", "Начальный или конечный узел не существует")
        blocked_edges = blocked_edges or set()
        adjacency: dict[str, list[DirectedEdge]] = {node_id: [] for node_id in self.nodes}
        for directed in self.directed_edges():
            if directed.edge.id not in blocked_edges:
                adjacency[directed.from_node].append(directed)
        queue: list[tuple[float, str]] = [(0.0, from_node)]
        best: dict[str, float] = {from_node: 0.0}
        previous: dict[str, DirectedEdge] = {}
        while queue:
            cost, node_id = heapq.heappop(queue)
            if cost > best.get(node_id, math.inf):
                continue
            if node_id == to_node:
                break
            for directed in adjacency[node_id]:
                candidate = cost + directed.length_units
                if candidate < best.get(directed.to_node, math.inf):
                    best[directed.to_node] = candidate
                    previous[directed.to_node] = directed
                    heapq.heappush(queue, (candidate, directed.to_node))
        if to_node not in best:
            raise RoadMapError("no_route", "По разрешённым проездам маршрут не найден", 409)
        path: list[DirectedEdge] = []
        cursor = to_node
        while cursor != from_node:
            directed = previous[cursor]
            path.append(directed)
            cursor = directed.from_node
        path.reverse()
        return path

    def plan(self, start: dict[str, Any], goal_node_id: str, *,
             snap_tolerance_units: float = 25.0,
             heading_tolerance_deg: float = 40.0,
             blocked_edges: Optional[Iterable[str]] = None) -> dict[str, Any]:
        if not isinstance(start, dict):
            raise RoadMapError("invalid_start", "start должен быть объектом {x, y, yawDeg}")
        if start.get("yawDeg") is None:
            raise RoadMapError("heading_required", "Перед построением маршрута задайте курс робота: одной точки недостаточно.")
        goal = self.nodes.get(_identifier(goal_node_id, "goalNodeId"))
        if goal is None:
            raise RoadMapError("unknown_goal", "Пункт назначения не найден на карте")
        if isinstance(blocked_edges, (str, bytes)) or not isinstance(blocked_edges, (list, tuple, set, frozenset, type(None))):
            raise RoadMapError("invalid_blocked_edges", "blockedEdges должен быть списком идентификаторов рёбер")
        blocked: set[str] = set()
        for item in blocked_edges or ():
            edge_id = _identifier(item, "blockedEdges[]")
            if edge_id not in self.edges:
                raise RoadMapError("unknown_blocked_edge", f"Закрытое ребро {edge_id} отсутствует на карте")
            blocked.add(edge_id)
        x = _number(start.get("x"), "start.x")
        y = _number(start.get("y"), "start.y")
        candidates = self._snap_candidates(x, y, start.get("yawDeg"),
                                           tolerance_units=snap_tolerance_units,
                                           heading_tolerance_deg=heading_tolerance_deg,
                                           blocked_edges=blocked)
        if not candidates:
            raise RoadMapError("start_not_on_road",
                               "Старт не лежит на открытом разрешённом проезде с заданным курсом. Поставьте точку на дорогу и укажите направление.",
                               409)
        if math.hypot(x - goal.x, y - goal.y) <= 1e-6:
            raise RoadMapError("goal_too_close", "Старт уже находится в пункте назначения", 409)
        best_plan: Optional[dict[str, Any]] = None
        best_cost = math.inf
        route_found = False
        for snap in candidates:
            first_end = snap.edge.to_node if not snap.reverse else snap.edge.from_node
            directed_geometry = snap.edge.geometry if not snap.reverse else tuple(reversed(snap.edge.geometry))
            first_geometry = _truncate(directed_geometry, snap.distance_along, snap.edge.length_units)
            first_length = polyline_length(first_geometry)
            legs: list[dict[str, Any]] = []
            if first_length > 1e-9:
                legs.append(self._leg(snap.edge, snap.reverse, first_geometry, first_end))
            try:
                remainder = self.shortest_path(first_end, goal.id, blocked) if first_end != goal.id else []
            except RoadMapError as exc:
                if exc.code == "no_route":
                    continue
                raise
            route_found = True
            for directed in remainder:
                legs.append(self._leg(directed.edge, directed.reverse, directed.geometry, directed.to_node))
            if not legs:
                continue
            total = sum(polyline_length(tuple(tuple(point) for point in leg["geometry"])) for leg in legs)
            cost = total + snap.lateral_distance
            if cost < best_cost - 1e-9:
                best_cost = cost
                best_plan = {"snap": snap, "legs": legs, "total": total}
        if best_plan is None:
            if route_found:
                raise RoadMapError("goal_too_close", "Старт уже находится в пункте назначения", 409)
            raise RoadMapError("no_route", "По открытым разрешённым проездам маршрут не найден", 409)
        snap: StartSnap = best_plan["snap"]
        legs = best_plan["legs"]
        total = float(best_plan["total"])
        maneuvers: list[dict[str, Any]] = []
        warnings: list[str] = []
        if self.status != "calibrated" or self.scale_meters_per_unit is None:
            warnings.append("Масштаб карты не подтверждён: расстояния показаны в единицах карты, не в метрах.")
        if goal.kind not in {"delivery", "dock"}:
            warnings.append("Цель не помечена как пункт доставки или док.")
        if blocked:
            warnings.append("Часть рёбер исключена из маршрута как временно закрытая; постоянная карта не изменена.")
        cumulative = 0.0
        for index, current in enumerate(legs):
            cumulative += polyline_length(tuple(tuple(point) for point in current["geometry"]))
            if index + 1 >= len(legs):
                continue
            following = legs[index + 1]
            in_geom = current["geometry"]
            out_geom = following["geometry"]
            in_angle = math.atan2(in_geom[-1][1] - in_geom[-2][1], in_geom[-1][0] - in_geom[-2][0])
            out_angle = math.atan2(out_geom[1][1] - out_geom[0][1], out_geom[1][0] - out_geom[0][0])
            turn = angle_deg(normalize_angle(out_angle - in_angle))
            node = self.nodes[current["toNodeId"]]
            maneuvers.append({
                "nodeId": node.id, "label": node.label, "xy": [node.x, node.y],
                "turnDeg": round(turn, 2), "instruction": turn_instruction(turn),
                "atDistanceUnits": round(cumulative, 3),
            })
        if goal.approach_yaw_deg is not None:
            last_geom = legs[-1]["geometry"]
            final_angle = math.atan2(last_geom[-1][1] - last_geom[-2][1], last_geom[-1][0] - last_geom[-2][0])
            alignment = angle_deg(normalize_angle(math.radians(goal.approach_yaw_deg) - final_angle))
            maneuvers.append({"nodeId": goal.id, "label": goal.label, "xy": [goal.x, goal.y],
                              "turnDeg": round(alignment, 2), "instruction": "alignment",
                              "atDistanceUnits": round(total, 3)})
        geometry: list[list[float]] = []
        for leg in legs:
            for point in leg["geometry"]:
                if not geometry or math.dist(geometry[-1], point) > 1e-9:
                    geometry.append([round(point[0], 3), round(point[1], 3)])
        distance_m = self.distance_m(total)
        # ПДД-объекты вдоль маршрута (для сухого прогона и отрисовки)
        traffic_controls: list[dict[str, Any]] = []
        try:
            from .road_traffic import traffic_controls_along_route
            traffic_controls = traffic_controls_along_route(self, legs)
        except Exception:
            traffic_controls = []
        if traffic_controls:
            warnings.append(f"На маршруте {len(traffic_controls)} объект(ов) ПДД: светофоры/знаки/переходы — требуется соблюдение правил.")
        return {
            "ok": True,
            "mapId": self.map_id,
            "mapStatus": self.status,
            "hardwareEnabled": False,
            "units": self.units,
            "start": {"x": round(snap.point[0], 3), "y": round(snap.point[1], 3),
                      "yawDeg": round(angle_deg(math.radians(_number(start.get("yawDeg"), "start.yawDeg"))), 3),
                      "edgeId": snap.edge.id, "reverse": snap.reverse,
                      "tangentDeg": round(snap.tangent_deg, 3),
                      "distanceAlongUnits": round(snap.distance_along, 3),
                      "lateralDistanceUnits": round(snap.lateral_distance, 3),
                      "headingErrorDeg": None if snap.heading_error_deg is None else round(snap.heading_error_deg, 2)},
            "goal": goal.to_dict(),
            "legs": legs,
            "maneuvers": maneuvers,
            "trafficControls": traffic_controls,
            "geometry": geometry,
            "totalDistanceUnits": round(total, 3),
            "totalDistanceM": None if distance_m is None else round(distance_m, 3),
            "warnings": warnings,
        }

    def _leg(self, edge: RoadEdge, reverse: bool, geometry: tuple[tuple[float, float], ...],
             to_node_id: str) -> dict[str, Any]:
        length = polyline_length(geometry)
        distance_m = self.distance_m(length)
        return {
            "edgeId": edge.id, "label": edge.label, "reverse": reverse,
            "toNodeId": to_node_id, "geometry": [[round(x, 3), round(y, 3)] for x, y in geometry],
            "lengthUnits": round(length, 3),
            "lengthM": None if distance_m is None else round(distance_m, 3),
            "widthUnits": edge.width_units,
            "speedLimitMps": edge.speed_limit_mps,
        }


def turn_instruction(turn_deg: float) -> str:
    if abs(turn_deg) < 15.0:
        return "straight"
    if abs(turn_deg) > 160.0:
        return "u_turn"
    return "left" if turn_deg > 0 else "right"


def atomic_write_json(path: Path | str, payload: dict[str, Any]) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=target.name + ".", suffix=".tmp", dir=str(target.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, target)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
