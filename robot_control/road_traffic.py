"""Инфраструктура ПДД на векторной карте: светофоры, знаки стоп, переходы.

Модуль проверяет схему ПДД и даёт детерминированную модель состояний для сухого
прогона. Он не включает тягу и не читает камеру — только решает, должен ли
робот стоять перед стоп-линией в модели.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional

from .road_map import RoadMapError, _identifier, _number, _text, polyline_length

MAX_TRAFFIC_LIGHTS = 200
MAX_STOP_SIGNS = 200
MAX_CROSSWALKS = 500


def _edge_length(edge) -> float:
    return polyline_length(edge.geometry)


@dataclass(frozen=True)
class TrafficLight:
    id: str
    label: str
    node_id: str
    controlled_edges: tuple[str, ...]
    stop_offset_units: float
    cycle_red_s: float
    cycle_yellow_s: float
    cycle_green_s: float
    initial_state: str

    @property
    def cycle_total(self) -> float:
        return self.cycle_red_s + self.cycle_yellow_s + self.cycle_green_s

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id, "label": self.label, "nodeId": self.node_id,
            "controlledEdges": list(self.controlled_edges),
            "stopOffsetUnits": self.stop_offset_units,
            "cycle": {"red": self.cycle_red_s, "yellow": self.cycle_yellow_s, "green": self.cycle_green_s},
            "initialState": self.initial_state,
        }

    def state_at(self, elapsed_s: float) -> str:
        """Детерминированное состояние по времени от старта симуляции."""
        total = self.cycle_total
        if total <= 1e-9:
            return self.initial_state
        # Порядок цикла: red -> yellow -> green -> red (зациклено)
        order = ["red", "yellow", "green"]
        # Начальный индекс
        idx = order.index(self.initial_state) if self.initial_state in order else 0
        # Длительности в порядке order, но сдвинутые относительно initial
        durations = {"red": self.cycle_red_s, "yellow": self.cycle_yellow_s, "green": self.cycle_green_s}
        # Собираем последовательность длительностей начиная с initial
        seq = []
        for offset in range(3):
            state = order[(idx + offset) % 3]
            seq.append((state, durations[state]))
        t = elapsed_s % total if total > 0 else 0
        cursor = 0
        for state, dur in seq:
            if dur <= 1e-9:
                continue
            if t < cursor + dur - 1e-9:
                return state
            cursor += dur
        return seq[-1][0]

    def is_stop_signal(self, elapsed_s: float) -> bool:
        return self.state_at(elapsed_s) in ("red", "yellow")


@dataclass(frozen=True)
class StopSign:
    id: str
    label: str
    node_id: str
    approach_edge_id: str
    stop_offset_units: float
    stop_duration_s: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id, "label": self.label, "nodeId": self.node_id,
            "approachEdgeId": self.approach_edge_id,
            "stopOffsetUnits": self.stop_offset_units,
            "stopDurationS": self.stop_duration_s,
        }


@dataclass(frozen=True)
class Crosswalk:
    id: str
    label: str
    edge_id: str
    at_distance_units: float
    width_units: float
    signalized: bool
    traffic_light_id: Optional[str] = None

    def to_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "id": self.id, "label": self.label, "edgeId": self.edge_id,
            "atDistanceUnits": self.at_distance_units,
            "widthUnits": self.width_units,
            "signalized": self.signalized,
        }
        if self.traffic_light_id:
            result["trafficLightId"] = self.traffic_light_id
        return result


def _validate_traffic_lights(raw_list: Any, nodes: dict, edges: dict) -> dict[str, TrafficLight]:
    if raw_list is None:
        return {}
    if not isinstance(raw_list, list):
        raise RoadMapError("invalid_traffic_lights", "trafficLights должен быть списком")
    if len(raw_list) > MAX_TRAFFIC_LIGHTS:
        raise RoadMapError("too_many_traffic_lights", f"Светофоров не больше {MAX_TRAFFIC_LIGHTS}")
    result: dict[str, TrafficLight] = {}
    for idx, raw in enumerate(raw_list):
        name = f"trafficLights[{idx}]"
        if not isinstance(raw, dict):
            raise RoadMapError("invalid_traffic_light", f"{name}: ожидается объект")
        light_id = _identifier(raw.get("id"), f"{name}.id")
        if light_id in result:
            raise RoadMapError("duplicate_traffic_light", f"Светофор {light_id} повторяется")
        node_id = _identifier(raw.get("nodeId"), f"{name}.nodeId")
        if node_id not in nodes:
            raise RoadMapError("unknown_node", f"{name}.nodeId {node_id} отсутствует")
        label = _text(raw.get("label") or light_id, f"{name}.label", 120)
        controlled = raw.get("controlledEdges")
        if not isinstance(controlled, list) or not controlled:
            raise RoadMapError("invalid_controlled_edges", f"{name}.controlledEdges: нужен непустой список")
        controlled_edges: list[str] = []
        for c_idx, item in enumerate(controlled):
            edge_id = _identifier(item, f"{name}.controlledEdges[{c_idx}]")
            if edge_id not in edges:
                raise RoadMapError("unknown_edge", f"{name}.controlledEdges {edge_id} отсутствует")
            # Проверяем, что ребро действительно ведёт в этот узел (с учётом обоих направлений)
            edge = edges[edge_id]
            if edge.from_node != node_id and edge.to_node != node_id:
                raise RoadMapError("traffic_light_mismatch", f"{name}: ребро {edge_id} не соединено с узлом {node_id}")
            controlled_edges.append(edge_id)
        stop_offset = _number(raw.get("stopOffsetUnits", 6.0), f"{name}.stopOffsetUnits", 0.0, 1000.0)
        cycle_raw = raw.get("cycle") or {}
        if not isinstance(cycle_raw, dict):
            raise RoadMapError("invalid_cycle", f"{name}.cycle должен быть объектом")
        red = _number(cycle_raw.get("red", 5.0), f"{name}.cycle.red", 0.0, 3600.0)
        yellow = _number(cycle_raw.get("yellow", 1.0), f"{name}.cycle.yellow", 0.0, 3600.0)
        green = _number(cycle_raw.get("green", 5.0), f"{name}.cycle.green", 0.0, 3600.0)
        if red + yellow + green <= 1e-9:
            raise RoadMapError("invalid_cycle", f"{name}.cycle: суммарная длительность должна быть >0")
        initial = _text(raw.get("initialState") or "red", f"{name}.initialState", 10).lower()
        if initial not in {"red", "yellow", "green"}:
            raise RoadMapError("invalid_initial_state", f"{name}.initialState: red/yellow/green")
        result[light_id] = TrafficLight(light_id, label, node_id, tuple(controlled_edges),
                                        stop_offset, red, yellow, green, initial)
    return result


def _validate_stop_signs(raw_list: Any, nodes: dict, edges: dict) -> dict[str, StopSign]:
    if raw_list is None:
        return {}
    if not isinstance(raw_list, list):
        raise RoadMapError("invalid_stop_signs", "stopSigns должен быть списком")
    if len(raw_list) > MAX_STOP_SIGNS:
        raise RoadMapError("too_many_stop_signs", f"Знаков стоп не больше {MAX_STOP_SIGNS}")
    result: dict[str, StopSign] = {}
    for idx, raw in enumerate(raw_list):
        name = f"stopSigns[{idx}]"
        if not isinstance(raw, dict):
            raise RoadMapError("invalid_stop_sign", f"{name}: ожидается объект")
        sign_id = _identifier(raw.get("id"), f"{name}.id")
        if sign_id in result:
            raise RoadMapError("duplicate_stop_sign", f"Знак стоп {sign_id} повторяется")
        node_id = _identifier(raw.get("nodeId"), f"{name}.nodeId")
        if node_id not in nodes:
            raise RoadMapError("unknown_node", f"{name}.nodeId {node_id} отсутствует")
        approach = _identifier(raw.get("approachEdgeId"), f"{name}.approachEdgeId")
        if approach not in edges:
            raise RoadMapError("unknown_edge", f"{name}.approachEdgeId {approach} отсутствует")
        edge = edges[approach]
        if edge.from_node != node_id and edge.to_node != node_id:
            raise RoadMapError("stop_sign_mismatch", f"{name}: ребро {approach} не соединено с узлом {node_id}")
        label = _text(raw.get("label") or sign_id, f"{name}.label", 120)
        stop_offset = _number(raw.get("stopOffsetUnits", 5.0), f"{name}.stopOffsetUnits", 0.0, 1000.0)
        duration = _number(raw.get("stopDurationS", 2.5), f"{name}.stopDurationS", 0.5, 60.0)
        result[sign_id] = StopSign(sign_id, label, node_id, approach, stop_offset, duration)
    return result


def _validate_crosswalks(raw_list: Any, nodes: dict, edges: dict, traffic_lights: dict[str, TrafficLight]) -> dict[str, Crosswalk]:
    if raw_list is None:
        return {}
    if not isinstance(raw_list, list):
        raise RoadMapError("invalid_crosswalks", "crosswalks должен быть списком")
    if len(raw_list) > MAX_CROSSWALKS:
        raise RoadMapError("too_many_crosswalks", f"Переходов не больше {MAX_CROSSWALKS}")
    result: dict[str, Crosswalk] = {}
    for idx, raw in enumerate(raw_list):
        name = f"crosswalks[{idx}]"
        if not isinstance(raw, dict):
            raise RoadMapError("invalid_crosswalk", f"{name}: ожидается объект")
        cw_id = _identifier(raw.get("id"), f"{name}.id")
        if cw_id in result:
            raise RoadMapError("duplicate_crosswalk", f"Переход {cw_id} повторяется")
        edge_id = _identifier(raw.get("edgeId"), f"{name}.edgeId")
        if edge_id not in edges:
            raise RoadMapError("unknown_edge", f"{name}.edgeId {edge_id} отсутствует")
        edge = edges[edge_id]
        length = _edge_length(edge)
        at_dist = _number(raw.get("atDistanceUnits"), f"{name}.atDistanceUnits", 0.0, 1e7)
        if at_dist < 1e-6 or at_dist >= length - 1e-6:
            raise RoadMapError("invalid_crosswalk_position",
                               f"{name}.atDistanceUnits должен быть внутри ребра (0, {round(length,3)})")
        width = _number(raw.get("widthUnits", 3.0), f"{name}.widthUnits", 0.5, 100.0)
        signalized_raw = raw.get("signalized", False)
        if not isinstance(signalized_raw, bool):
            raise RoadMapError("invalid_signalized", f"{name}.signalized должен быть true/false")
        signalized = signalized_raw
        tl_id = raw.get("trafficLightId")
        tl_id_norm = None
        if tl_id is not None:
            tl_id_norm = _identifier(tl_id, f"{name}.trafficLightId")
            if tl_id_norm not in traffic_lights:
                raise RoadMapError("unknown_traffic_light", f"{name}.trafficLightId {tl_id_norm} отсутствует")
            if not signalized:
                raise RoadMapError("crosswalk_signal_mismatch", f"{name}: signalized должен быть true при наличии trafficLightId")
        label = _text(raw.get("label") or cw_id, f"{name}.label", 120)
        result[cw_id] = Crosswalk(cw_id, label, edge_id, at_dist, width, signalized, tl_id_norm)
    return result


def point_along_polyline(geometry: tuple[tuple[float, float], ...], distance: float) -> tuple[float, float]:
    """Точка на ломаной на расстоянии distance от её начала."""
    if distance <= 0:
        return geometry[0]
    travelled = 0.0
    for a, b in zip(geometry, geometry[1:]):
        seg = math.dist(a, b)
        if travelled + seg >= distance - 1e-9:
            t = max(0.0, min(1.0, (distance - travelled) / seg)) if seg > 1e-12 else 0.0
            return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        travelled += seg
    return geometry[-1]


def traffic_controls_along_route(road_map, legs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Все ПДД-объекты на маршруте, отсортированные по расстоянию вдоль маршрута.

    Возвращает список словарей с полями type, id, atDistanceUnits, xy и исходными данными.
    """
    if not legs or road_map is None:
        return []
    # Кумулятивные расстояния до начала каждого участка
    cumulative: list[float] = []
    total = 0.0
    for leg in legs:
        cumulative.append(total)
        total += float(leg.get("lengthUnits") or 0.0)
    total_route = total
    edge_to_leg: dict[str, list[int]] = {}
    for idx, leg in enumerate(legs):
        edge_to_leg.setdefault(str(leg.get("edgeId")), []).append(idx)

    result: list[dict[str, Any]] = []

    # Светофоры и стоп-знаки привязаны к узлу: ищем участок, который заканчивается в этом узле
    node_to_cumulative: dict[str, float] = {}
    for idx, leg in enumerate(legs):
        node_to_cumulative[str(leg.get("toNodeId"))] = cumulative[idx] + float(leg.get("lengthUnits") or 0.0)
    # Для первого участка начальный узел не входит в node_to_cumulative, но можем добавить 0 для стартового
    # Но светофор на стартовом узле не имеет смысла (робот уже за ним)

    for light in getattr(road_map, "traffic_lights", {}).values():
        if light.node_id not in node_to_cumulative:
            continue
        node_dist = node_to_cumulative[light.node_id]
        at_dist = node_dist - light.stop_offset_units
        if at_dist < 0:
            at_dist = 0
        if at_dist > total_route + 1e-6:
            continue
        # Проверяем, что маршрут входит в узел по одному из контролируемых рёбер
        # Находим индекс участка, который заканчивается в node_id
        leg_idx = None
        for idx, leg in enumerate(legs):
            if str(leg.get("toNodeId")) == light.node_id and str(leg.get("edgeId")) in light.controlled_edges:
                leg_idx = idx
                break
        if leg_idx is None:
            # Если маршрут проходит через узел, но по другому ребру — светофор не для нас
            continue
        # Координата стоп-линии: точка на геометрии участка, отстоящая от конца на stopOffset
        leg = legs[leg_idx]
        geom = leg.get("geometry") or []
        leg_len = float(leg.get("lengthUnits") or 0.0)
        dist_along_leg = max(0.0, leg_len - light.stop_offset_units)
        if len(geom) >= 2:
            xy = point_along_polyline(tuple(tuple(p) for p in geom), dist_along_leg)
        else:
            # fallback: узел
            node = road_map.nodes.get(light.node_id)
            xy = (node.x, node.y) if node else (0, 0)
        result.append({
            "type": "traffic_light",
            "id": light.id,
            "label": light.label,
            "nodeId": light.node_id,
            "controlledEdges": list(light.controlled_edges),
            "atDistanceUnits": round(at_dist, 3),
            "stopOffsetUnits": light.stop_offset_units,
            "xy": [round(xy[0], 3), round(xy[1], 3)],
            "cycle": {"red": light.cycle_red_s, "yellow": light.cycle_yellow_s, "green": light.cycle_green_s},
            "initialState": light.initial_state,
        })

    for sign in getattr(road_map, "stop_signs", {}).values():
        if sign.node_id not in node_to_cumulative:
            continue
        node_dist = node_to_cumulative[sign.node_id]
        at_dist = node_dist - sign.stop_offset_units
        if at_dist < 0:
            at_dist = 0
        if at_dist > total_route + 1e-6:
            continue
        leg_idx = None
        for idx, leg in enumerate(legs):
            if str(leg.get("toNodeId")) == sign.node_id and str(leg.get("edgeId")) == sign.approach_edge_id:
                leg_idx = idx
                break
        if leg_idx is None:
            continue
        leg = legs[leg_idx]
        geom = leg.get("geometry") or []
        leg_len = float(leg.get("lengthUnits") or 0.0)
        dist_along_leg = max(0.0, leg_len - sign.stop_offset_units)
        if len(geom) >= 2:
            xy = point_along_polyline(tuple(tuple(p) for p in geom), dist_along_leg)
        else:
            node = road_map.nodes.get(sign.node_id)
            xy = (node.x, node.y) if node else (0, 0)
        result.append({
            "type": "stop_sign",
            "id": sign.id,
            "label": sign.label,
            "nodeId": sign.node_id,
            "approachEdgeId": sign.approach_edge_id,
            "atDistanceUnits": round(at_dist, 3),
            "stopOffsetUnits": sign.stop_offset_units,
            "stopDurationS": sign.stop_duration_s,
            "xy": [round(xy[0], 3), round(xy[1], 3)],
        })

    for cw in getattr(road_map, "crosswalks", {}).values():
        if cw.edge_id not in edge_to_leg:
            continue
        edge = road_map.edges.get(cw.edge_id)
        if edge is None:
            continue
        edge_len = _edge_length(edge)
        # Для каждого вхождения ребра в маршрут (обычно 1)
        for leg_idx in edge_to_leg[cw.edge_id]:
            leg = legs[leg_idx]
            leg_len = float(leg.get("lengthUnits") or 0.0)
            reverse = bool(leg.get("reverse"))
            geom = leg.get("geometry") or []
            # Позиция пешеходного перехода вдоль ребра от его начала (from_node)
            dist_along_edge = cw.at_distance_units
            if reverse:
                dist_along_edge = edge_len - dist_along_edge
            # Для первого участка геометрия может быть усечена началом
            # Нам нужно учесть, что leg.geometry это усечённый кусок ребра
            # Если первый участок усечён на snap-расстояние, то dist_along_edge должен быть скорректирован
            # Упрощение: если leg_idx==0 и snap есть, то legs уже усечены, но edge_len vs leg_len позволяют оценить смещение
            # Для первого участка усечённое начало: начало leg соответствует точке snap, конец — конец ребра
            # Тогда расстояние от начала leg до перехода = dist_along_edge - snap_offset
            # snap_offset = edge_len - leg_len для первого участка (если усечён)
            if leg_idx == 0 and abs(edge_len - leg_len) > 1e-6:
                snap_offset = edge_len - leg_len
                # dist_along_edge измеряется от начала ребра (from_node)
                # Начало leg = snap_offset от начала ребра
                dist_in_leg = dist_along_edge - snap_offset
                if dist_in_leg < -1e-6 or dist_in_leg > leg_len + 1e-6:
                    continue
            else:
                # Для неусечённых участков leg соответствует всему ребру
                dist_in_leg = dist_along_edge if not reverse else (edge_len - cw.at_distance_units)
                # Но для неусечённых leg_len == edge_len, так что просто cw_at
                # Пересчитаем корректно: для reversed leg geometry уже развёрнута, поэтому dist_in_leg = dist_along_leg_in_direction
                if reverse:
                    dist_in_leg = edge_len - cw.at_distance_units
                else:
                    dist_in_leg = cw.at_distance_units
                # Для неусечённых это внутри (0, leg_len)
                if dist_in_leg < -1e-6 or dist_in_leg > leg_len + 1e-6:
                    continue
            at_dist = cumulative[leg_idx] + max(0.0, min(leg_len, dist_in_leg))
            if len(geom) >= 2:
                xy = point_along_polyline(tuple(tuple(p) for p in geom), max(0.0, min(leg_len, dist_in_leg)))
            else:
                xy = (0, 0)
            result.append({
                "type": "crosswalk",
                "id": cw.id,
                "label": cw.label,
                "edgeId": cw.edge_id,
                "atDistanceUnits": round(at_dist, 3),
                "widthUnits": cw.width_units,
                "signalized": cw.signalized,
                "trafficLightId": cw.traffic_light_id,
                "xy": [round(xy[0], 3), round(xy[1], 3)],
            })

    result.sort(key=lambda x: x["atDistanceUnits"])
    return result

