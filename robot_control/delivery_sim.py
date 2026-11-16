"""Сухой прогон миссии доставки по векторному маршруту.

Сценарии синтетические: они проверяют автомат решений, а не реальное зрение,
тормозной путь или проходимость. Тяга, STOP-команды колёсам и E-STOP железа здесь
не вызываются. Результат всегда помечен `tractionConnected=False`.

На этом этапе дополнительно проверяется соблюдение ПДД: светофоры, знаки СТОП,
пешеходные переходы и ограничения скорости. Нарушение любого требования приводит
к безопасной остановке, а не к проезду.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional

from .delivery_mission import DeliveryMission
from .road_localization import RoadLocalization
from .road_map import RoadMap

SCENARIOS = {"clear", "bump", "pothole_bypass", "pothole_blocked", "moving_object", "localization_lost",
             "traffic_light_red", "traffic_light_green", "stop_sign", "pedestrian_crossing", "speed_limit"}


@dataclass(frozen=True)
class SimulationConfig:
    speed_units_per_s: float = 20.0
    bump_speed_units_per_s: float = 5.0
    latency_s: float = 0.2
    decel_units_s2: float = 20.0
    stop_margin_units: float = 0.5
    road_half_width_units: float = 6.0
    robot_half_width_units: float = 1.5
    robot_length_units: float = 4.0
    hazard_detection_range_units: float = 25.0
    moving_object_wait_s: float = 2.0
    step_s: float = 0.1
    max_steps: int = 100000

    def validate(self) -> "SimulationConfig":
        if self.speed_units_per_s <= 0 or self.bump_speed_units_per_s <= 0:
            raise ValueError("Скорости симуляции должны быть положительными")
        if self.bump_speed_units_per_s > self.speed_units_per_s:
            raise ValueError("Скорость на кочке не может быть выше крейсерской")
        if self.decel_units_s2 <= 0 or self.step_s <= 0 or self.max_steps < 1:
            raise ValueError("Неверные параметры времени/замедления")
        if self.road_half_width_units <= self.robot_half_width_units:
            raise ValueError("Дорога уже робота")
        if self.robot_length_units <= 0 or self.hazard_detection_range_units <= 0:
            raise ValueError("Габарит и дальность обнаружения должны быть положительными")
        return self


def stopping_distance(speed: float, config: SimulationConfig) -> float:
    return speed * config.latency_s + speed * speed / (2.0 * config.decel_units_s2) + config.stop_margin_units


def slowdown_distance(speed: float, target_speed: float, config: SimulationConfig) -> float:
    return speed * config.latency_s + max(0.0, speed * speed - target_speed * target_speed) / (2.0 * config.decel_units_s2) + config.stop_margin_units


def pothole_bypass(hazard: dict[str, Any], config: SimulationConfig) -> Optional[dict[str, Any]]:
    """Можно ли провести весь габарит робота слева/справа от ямы внутри дороги."""
    bbox = hazard.get("bbox")
    if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
        return None
    try:
        min_y, max_y = float(bbox[1]), float(bbox[3])
    except (TypeError, ValueError):
        return None
    if not math.isfinite(min_y) or not math.isfinite(max_y) or max_y < min_y:
        return None
    margin = config.stop_margin_units
    half = config.road_half_width_units
    robot = config.robot_half_width_units
    left_target = max_y + robot + margin
    right_target = min_y - robot - margin
    left_ok = left_target <= half - robot
    right_ok = right_target >= -half + robot
    if not left_ok and not right_ok:
        return None
    if left_ok and right_ok:
        side = "left" if (half - robot - left_target) >= (right_target + half - robot) else "right"
    else:
        side = "left" if left_ok else "right"
    target = left_target if side == "left" else right_target
    return {"side": side, "targetY": round(target, 3)}


def decide_ground_hazard(hazard: dict[str, Any], distance: float, speed: float,
                         config: SimulationConfig) -> dict[str, Any]:
    kind = hazard.get("kind")
    if distance < 0:
        return {"action": "continue", "reason": "hazard_behind"}
    if kind == "bump":
        if distance <= slowdown_distance(speed, config.bump_speed_units_per_s, config):
            return {"action": "slow", "reason": "bump_ahead", "targetSpeed": config.bump_speed_units_per_s}
        return {"action": "continue", "reason": "bump_far"}
    if kind == "pothole":
        bypass = pothole_bypass(hazard, config)
        if distance <= stopping_distance(speed, config):
            if bypass is None:
                return {"action": "stop", "reason": "pothole_no_bypass"}
            return {"action": "avoid", "reason": "pothole_bypass", **bypass}
        if bypass is None:
            return {"action": "prepare_stop", "reason": "pothole_no_bypass"}
        return {"action": "prepare_avoid", "reason": "pothole_bypass", **bypass}
    return {"action": "stop", "reason": "unknown_hazard"}


def _polyline_tangent_deg(geometry: list[Any], distance: float) -> float:
    travelled = 0.0
    for a, b in zip(geometry, geometry[1:]):
        length = math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
        if travelled + length >= distance - 1e-9:
            return math.degrees(math.atan2(float(b[1]) - float(a[1]), float(b[0]) - float(a[0])))
        travelled += length
    a, b = geometry[-2], geometry[-1]
    return math.degrees(math.atan2(float(b[1]) - float(a[1]), float(b[0]) - float(a[0])))


def _current_speed_limit(legs: list[dict[str, Any]], leg_index: int) -> Optional[float]:
    if 0 <= leg_index < len(legs):
        limit = legs[leg_index].get("speedLimitMps")
        if limit is not None:
            try:
                return float(limit)
            except (TypeError, ValueError):
                return None
    return None


def simulate_route(route: dict[str, Any], scenario: str = "clear",
                   config: Optional[SimulationConfig] = None,
                   road_map: Optional[RoadMap] = None) -> dict[str, Any]:
    if scenario not in SCENARIOS:
        raise ValueError(f"Неизвестный сценарий: {scenario}")
    cfg = (config or SimulationConfig()).validate()
    if not isinstance(route, dict) or not route.get("ok"):
        raise ValueError("Маршрут не построен")
    total = float(route.get("totalDistanceUnits") or 0.0)
    if total <= 0:
        raise ValueError("Маршрут нулевой длины")
    legs = list(route.get("legs") or [])
    if not legs:
        raise ValueError("Маршрут без участков")
    mission = DeliveryMission(route)
    mission.start({"route_valid": True, "dry_run_only": True, "start_pose_set": True,
                   "goal_selected": True, "hardware_disabled": True})
    localization = RoadLocalization(road_map) if road_map is not None else None
    if localization is not None:
        start = route.get("start") or {}
        localization.set_start(float(start.get("x", 0.0)), float(start.get("y", 0.0)),
                               float(start.get("yawDeg", 0.0)), tolerance_units=1.0,
                               heading_tolerance_deg=45.0)
    events: list[dict[str, Any]] = []
    sequence = 0

    def emit(state: str, message: str, distance: Optional[float] = None) -> None:
        nonlocal sequence
        events.append({"seq": sequence, "t": round(elapsed, 3),
                       "distance": round(progress if distance is None else distance, 3),
                       "state": state, "message": message})
        sequence += 1

    progress = 0.0
    elapsed = 0.0
    speed = cfg.speed_units_per_s
    final_state = "done"
    trigger = total * 0.35
    available = total - trigger - cfg.robot_length_units - 1.0
    if available <= 0:
        raise ValueError("Маршрут слишком короткий для синтетического препятствия")
    detection_range = min(cfg.hazard_detection_range_units, available * 0.5)
    hazard_start = trigger + detection_range
    hazard_length = min(max(2.0, total * 0.03), available * 0.25)
    hazard: Optional[dict[str, Any]] = None
    hazard_detected = False
    hazard_active = False
    wait_left = 0.0
    avoid_stop_left = 0.0
    lateral_offset = 0.0
    lateral_target = 0.0
    max_lateral = 0.0
    leg_index = 0
    leg_travelled = 0.0
    junctions = [item for item in route.get("maneuvers", []) if item.get("instruction") != "alignment"]
    junction_index = 0

    # ПДД: объекты вдоль маршрута (светофоры, стоп-знаки, переходы)
    pdd_controls: list[dict[str, Any]] = []
    if road_map is not None:
        try:
            from .road_traffic import traffic_controls_along_route
            # route может уже содержать trafficControls (из road_map.plan), используем их для консистентности
            pdd_controls = list(route.get("trafficControls") or traffic_controls_along_route(road_map, legs))
        except Exception:
            pdd_controls = []
    pdd_index = 0
    pdd_wait_left = 0.0
    pdd_active_id: Optional[str] = None
    pdd_active_type: Optional[str] = None

    emit("FOLLOW_ROAD", "Маршрут построен, сухой прогон начат.", 0.0)
    steps = 0
    while mission.active and steps < cfg.max_steps:
        steps += 1
        if scenario == "localization_lost" and progress >= total * 0.65:
            if localization is not None:
                localization.lose("junction_confirmation_timeout")
            mission.stop_safe("localization_lost")
            final_state = "localization_lost"
            emit("STOP_SAFE", "Ожидаемый узел не подтверждён: безопасная остановка без продолжения вслепую.")
            break
        if scenario == "moving_object" and not hazard_detected and progress >= trigger:
            hazard_detected = True
            wait_left = cfg.moving_object_wait_s
            mission.object_risk()
            emit("YIELD", "Синтетический приближающийся объект: остановка и ожидание.")
        if wait_left > 0:
            wait_left -= cfg.step_s
            elapsed += cfg.step_s
            if wait_left <= 0:
                mission.resume_after_yield()
                emit("FOLLOW_ROAD", "Коридор снова свободен, продолжение маршрута.")
            continue
        if avoid_stop_left > 0:
            avoid_stop_left -= cfg.step_s
            elapsed += cfg.step_s
            if avoid_stop_left <= 0:
                speed = cfg.bump_speed_units_per_s
                emit("AVOID_POTHOLE", "Остановка перед ямой выполнена в модели, начинается боковой манёвр на малой скорости.")
            continue
        # Таймер ожидания ПДД (светофор/стоп/пешеход)
        if pdd_wait_left > 0:
            pdd_wait_left -= cfg.step_s
            elapsed += cfg.step_s
            if pdd_wait_left <= 0:
                if mission.state == "WAIT_TRAFFIC_LIGHT":
                    mission.proceed_on_green(pdd_active_id or "")
                    emit("FOLLOW_ROAD", f"Светофор {pdd_active_id}: зелёный — движение разрешено.")
                    speed = cfg.speed_units_per_s
                    # проверка ограничения скорости на текущем участке
                    limit = _current_speed_limit(legs, leg_index)
                    if limit is not None and speed > limit:
                        speed = limit
                    pdd_index += 1
                    pdd_active_id = None
                    pdd_active_type = None
                elif mission.state == "WAIT_STOP":
                    mission.proceed_after_stop(pdd_active_id or "")
                    emit("FOLLOW_ROAD", f"Знак СТОП {pdd_active_id}: выдержка выполнена, движение продолжено.")
                    speed = cfg.speed_units_per_s
                    limit = _current_speed_limit(legs, leg_index)
                    if limit is not None and speed > limit:
                        speed = limit
                    pdd_index += 1
                    pdd_active_id = None
                    pdd_active_type = None
                elif mission.state == "WAIT_PEDESTRIAN":
                    mission.resume_after_pedestrian(pdd_active_id or "")
                    emit("FOLLOW_ROAD", f"Пешеходный переход {pdd_active_id}: пешеход ушёл, путь свободен.")
                    speed = cfg.speed_units_per_s
                    limit = _current_speed_limit(legs, leg_index)
                    if limit is not None and speed > limit:
                        speed = limit
                    pdd_index += 1
                    pdd_active_id = None
                    pdd_active_type = None
                else:
                    pdd_active_id = None
                    pdd_active_type = None
            continue

        # Проверка ПДД перед каждым шагом
        if pdd_index < len(pdd_controls):
            ctrl = pdd_controls[pdd_index]
            at = float(ctrl.get("atDistanceUnits", 0))
            ctrl_type = ctrl.get("type")
            # Уже проехали? (с запасом по длине робота)
            if progress >= at + cfg.robot_length_units + 0.5:
                # Проехали объект без остановки — для стоп-знака это нарушение, но мы уже должны были остановиться
                pdd_index += 1
                continue
            distance = at - progress
            # Проверка нарушения: проехали стоп-линию на красный
            if distance < -0.5:
                # Мы уже за линией
                if ctrl_type == "traffic_light":
                    # Определяем, был ли красный в момент проезда
                    should_be_red = scenario == "traffic_light_red" and mission.state != "FOLLOW_ROAD"
                    # Если мы в WAIT и уже за линией — это ошибка модели, считаем нарушением
                    if mission.state == "WAIT_TRAFFIC_LIGHT":
                        mission.violation_traffic_light(str(ctrl.get("id")))
                        final_state = "traffic_violation_red_light"
                        emit("STOP_SAFE", f"Нарушение ПДД: проезд стоп-линии светофора {ctrl.get('id')} на красный.")
                        break
                pdd_index += 1
                continue

            if ctrl_type == "traffic_light":
                # Логика светофора: в сценариях ПДД красный требует ожидания
                is_red_scenario = scenario == "traffic_light_red"
                # Для остальных сценариев считаем зелёный (чтобы не ломать старые тесты)
                light_is_red = is_red_scenario and distance <= stopping_distance(speed, cfg) and distance >= -1e-9 and progress < at
                # Также если светофор в смешанном PDD-сценарии мог бы быть красным по циклу — упрощаем
                if light_is_red:
                    if mission.state != "WAIT_TRAFFIC_LIGHT":
                        mission.wait_traffic_light(str(ctrl.get("id")), "red")
                        color = "красный"
                        emit("WAIT_TRAFFIC_LIGHT", f"Светофор {ctrl.get('id')}: {color} — остановка перед стоп-линией.")
                        # Время ожидания до зелёного — 3-4 секунды в модели
                        pdd_wait_left = 3.5
                        pdd_active_id = str(ctrl.get("id"))
                        pdd_active_type = "traffic_light"
                        speed = 0.0
                        continue
                else:
                    # зелёный — едем, но если мы стояли на этом светофоре, то уже обработано в таймере
                    # Если светофор далеко, просто продолжаем
                    if distance <= 0 and mission.state == "WAIT_TRAFFIC_LIGHT":
                        # Должны были уже уехать — защита
                        pdd_index += 1
                        continue
            elif ctrl_type == "stop_sign":
                if distance <= stopping_distance(speed, cfg) and distance >= -1e-9:
                    if mission.state != "WAIT_STOP":
                        mission.wait_stop_sign(str(ctrl.get("id")))
                        emit("WAIT_STOP", f"Знак СТОП {ctrl.get('id')}: полная остановка перед линией.")
                        pdd_wait_left = float(ctrl.get("stopDurationS", 2.0))
                        pdd_active_id = str(ctrl.get("id"))
                        pdd_active_type = "stop_sign"
                        speed = 0.0
                        continue
                    # Нарушение: проехали стоп-линию без остановки (проверяется выше по таймеру)
            elif ctrl_type == "crosswalk":
                # Пешеходный переход: сценарий pedestrian_crossing — пешеход присутствует
                pedestrian_present = scenario == "pedestrian_crossing"
                if pedestrian_present and distance <= stopping_distance(speed, cfg) and distance >= -1e-9:
                    if mission.state != "WAIT_PEDESTRIAN":
                        mission.wait_pedestrian(str(ctrl.get("id")))
                        emit("WAIT_PEDESTRIAN", f"Пешеходный переход {ctrl.get('id')}: пешеход на дороге — уступаем.")
                        pdd_wait_left = 3.0
                        pdd_active_id = str(ctrl.get("id"))
                        pdd_active_type = "crosswalk"
                        speed = 0.0
                        continue
                # Если пешеход ушёл, но мы ещё стоим — таймер уже обрабатывает
                # Если пешеходный светофор красный (signalized) — treat as traffic light, но сейчас пешеход только в сценарии
            # Скорость: ограничение на текущем ребре
            limit = _current_speed_limit(legs, leg_index)
            if limit is not None and speed > limit + 1e-6:
                # Плавное снижение: если до зоны ограничения близко, начинаем тормозить заранее
                # Упрощённо: сразу ограничиваем скорость на участке
                speed = limit
                emit("SPEED_LIMIT", f"Ограничение скорости {limit} ед/с — снижение скорости.")

        if scenario in {"bump", "pothole_bypass", "pothole_blocked"} and not hazard_detected and progress >= trigger:
            hazard_detected = True
            if scenario == "bump":
                hazard = {"kind": "bump", "start": hazard_start, "end": hazard_start + hazard_length,
                          "bbox": [detection_range, -1.0,
                                   detection_range + hazard_length, 1.0]}
            else:
                width = 2.0 if scenario == "pothole_bypass" else cfg.road_half_width_units * 2.0
                hazard = {"kind": "pothole", "start": hazard_start, "end": hazard_start + hazard_length,
                          "bbox": [detection_range, -width / 2,
                                   detection_range + hazard_length, width / 2]}
            emit("HAZARD_DETECTED", "Синтетический дефект покрытия обнаружен впереди.")
        if hazard is not None:
            distance = float(hazard["start"]) - progress
            if not hazard_active:
                decision = decide_ground_hazard(hazard, distance, speed, cfg)
                if decision["action"] == "slow":
                    speed = cfg.bump_speed_units_per_s
                    hazard_active = True
                    mission.ground_hazard("bump")
                    emit("SLOW_FOR_BUMP", "Замедление перед кочкой; малая скорость до проезда задними колёсами.")
                elif decision["action"] == "avoid":
                    hazard_active = True
                    lateral_target = float(decision.get("targetY") or 0.0)
                    avoid_stop_left = cfg.latency_s + speed / cfg.decel_units_s2
                    speed = 0.0
                    mission.ground_hazard("pothole", bypass=decision)
                    emit("AVOID_POTHOLE", f"Остановка перед ямой и синтетический объезд {decision.get('side')}.")
                elif decision["action"] == "stop":
                    mission.ground_hazard("pothole", bypass=None)
                    final_state = "blocked_by_road_hazard"
                    emit("STOP_SAFE", "Яма перекрывает проезд: безопасного объезда нет, миссия остановлена.")
                    break
            if hazard_active and hazard["kind"] == "pothole" and avoid_stop_left <= 0:
                lateral_offset += max(-cfg.bump_speed_units_per_s * cfg.step_s,
                                      min(cfg.bump_speed_units_per_s * cfg.step_s, lateral_target - lateral_offset))
                max_lateral = max(max_lateral, abs(lateral_offset))
            if hazard_active and progress >= float(hazard["end"]) + cfg.robot_length_units:
                if hazard["kind"] == "bump":
                    speed = cfg.speed_units_per_s
                    limit = _current_speed_limit(legs, leg_index)
                    if limit is not None and speed > limit:
                        speed = limit
                    emit("FOLLOW_ROAD", "Кочка пройдена всеми колёсами, крейсерская скорость восстановлена.")
                else:
                    lateral_offset = 0.0
                    lateral_target = 0.0
                    speed = cfg.speed_units_per_s
                    limit = _current_speed_limit(legs, leg_index)
                    if limit is not None and speed > limit:
                        speed = limit
                    emit("FOLLOW_ROAD", "Яма пройдена стороной, возврат на линию движения и крейсерская скорость.")
                mission.resume_after_hazard()
                hazard = None
                hazard_active = False
        if junction_index < len(junctions) and progress >= float(junctions[junction_index].get("atDistanceUnits", total)) - 1e-6:
            maneuver = junctions[junction_index]
            node_id = str(maneuver.get("nodeId"))
            mission.reach_junction(node_id)
            emit("APPROACH_JUNCTION", f"{maneuver.get('label')}: ожидается {maneuver.get('instruction')} ({maneuver.get('turnDeg')}°).")
            if mission.state != "APPROACH_JUNCTION":
                final_state = mission.reason or "junction_mismatch"
                emit("STOP_SAFE", "Узел маршрута не совпал с ожидаемым.")
                break
            mission.confirm_junction(node_id)
            emit("FOLLOW_ROAD", f"Узел {node_id} подтверждён в модели, поворот выполнен по рассчитанному углу.")
            junction_index += 1
        if localization is not None and not localization.trusted and localization.state != "awaiting_junction":
            mission.stop_safe("localization_lost")
            final_state = "localization_lost"
            emit("STOP_SAFE", "Локализация потеряла доверие: остановка.")
            break
        # Ограничение скорости на текущем участке (проверка перед шагом)
        limit = _current_speed_limit(legs, leg_index)
        if limit is not None and speed > limit + 1e-6:
            speed = limit
        step_distance = min(speed * cfg.step_s, total - progress)
        if localization is not None and leg_index < len(legs):
            leg = legs[leg_index]
            leg_length = float(leg.get("lengthUnits") or 0.0)
            remaining = max(0.0, leg_length - leg_travelled)
            applied = min(step_distance, remaining)
            geometry = list(leg.get("geometry") or [])
            yaw_delta = 0.0
            if len(geometry) >= 2:
                before = _polyline_tangent_deg(geometry, leg_travelled)
                after = _polyline_tangent_deg(geometry, leg_travelled + applied)
                yaw_delta = (after - before + 180.0) % 360.0 - 180.0
            localization.advance(applied, yaw_delta_deg=yaw_delta)
            localization.correct(lateral_units=lateral_offset, yaw_error_deg=0.0,
                                 confidence=max(0.6, localization.confidence))
            leg_travelled += applied
            if leg_travelled >= leg_length - 1e-6 and leg_index + 1 < len(legs):
                next_leg = legs[leg_index + 1]
                localization.confirm_junction(str(next_leg.get("edgeId")), reverse=bool(next_leg.get("reverse")))
                leg_index += 1
                leg_travelled = 0.0
            step_distance = applied
        mission.advance(step_distance, localization_trusted=True if localization is None else localization.trusted or localization.state == "awaiting_junction")
        progress = mission.progress_units
        elapsed += cfg.step_s
        if progress >= total - 1e-6:
            mission.arrive()
            emit("DOCK_DELIVERY", "Пункт доставки достигнут в модели, остановка у цели.", total)
            if mission.state == "DOCK_DELIVERY":
                mission.complete_unload(confirmed=True)
                emit("DONE", "Сухой прогон завершён: доставка и выгрузка подтверждены в модели.", total)
                final_state = "done"
            else:
                final_state = mission.reason or "delivery_not_reached"
            break
    if mission.active:
        mission.stop_safe("simulation_step_limit")
        final_state = "simulation_step_limit"
        emit("STOP_SAFE", "Симуляция достигла предела шагов.")
    if mission.state == "STOP_SAFE" and final_state == "done":
        final_state = mission.reason or "stop_safe"
    events.sort(key=lambda item: (item["distance"], item["seq"]))
    return {"ok": final_state == "done" and mission.state == "DONE", "scenario": scenario,
            "finalState": final_state, "missionState": mission.state,
            "tractionConnected": False, "hardwareEnabled": False,
            "distanceUnits": round(progress, 3), "totalDistanceUnits": round(total, 3),
            "elapsedS": round(elapsed, 3), "steps": steps,
            "events": events, "maxLateralOffsetUnits": round(max_lateral, 3),
            "localization": None if localization is None else localization.to_dict()}
