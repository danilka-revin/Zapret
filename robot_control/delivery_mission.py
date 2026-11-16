"""Автомат миссии доставки. Только состояния и решения, без моторных команд.

Класс проверяет порядок этапов и не даёт перескочить через предполётную проверку,
подтверждение узла или остановку при потере локализации. Реальный исполнитель
движения сюда не подключён: все ответы помечены `tractionConnected=False`.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Optional

ACTIVE_STATES = {"PREFLIGHT", "FOLLOW_ROAD", "APPROACH_JUNCTION", "YIELD",
                 "AVOID_POTHOLE", "SLOW_FOR_BUMP", "DOCK_DELIVERY", "UNLOAD",
                 "WAIT_TRAFFIC_LIGHT", "WAIT_STOP", "WAIT_PEDESTRIAN", "SPEED_LIMIT"}
TERMINAL_STATES = {"DONE", "STOP_SAFE"}


@dataclass
class DeliveryMission:
    route: dict[str, Any]
    state: str = "IDLE"
    reason: str = ""
    progress_units: float = 0.0
    maneuver_index: int = 0
    started_at: Optional[float] = None
    finished_at: Optional[float] = None
    events: list[dict[str, Any]] = field(default_factory=list)

    @property
    def active(self) -> bool:
        return self.state in ACTIVE_STATES

    @property
    def total_units(self) -> float:
        return float(self.route.get("totalDistanceUnits") or 0.0)

    def _event(self, state: str, message: str, distance: Optional[float] = None) -> None:
        self.events.append({"t": None if self.started_at is None else round(time.time() - self.started_at, 3),
                            "distance": round(self.progress_units if distance is None else distance, 3),
                            "state": state, "message": message})

    def _set(self, state: str, reason: str = "") -> dict[str, Any]:
        self.state = state
        self.reason = reason
        if state in TERMINAL_STATES:
            self.finished_at = time.time()
        return self.to_dict()

    def start(self, preflight: dict[str, bool]) -> dict[str, Any]:
        if self.state != "IDLE":
            return self.to_dict()
        self.started_at = time.time()
        self._set("PREFLIGHT")
        self._event("PREFLIGHT", "Проверка карты, старта, цели и готовность к сухому прогону.")
        failed = [key for key, value in preflight.items() if not value]
        if not self.route.get("ok") or self.total_units <= 0:
            failed.append("route_valid")
        if failed:
            self._event("STOP_SAFE", "Предполётная проверка не пройдена: " + ", ".join(failed))
            return self._set("STOP_SAFE", "preflight_failed")
        self._event("FOLLOW_ROAD", "Маршрут построен, движение разрешено только в модели.")
        return self._set("FOLLOW_ROAD")

    def advance(self, distance_units: float, *, localization_trusted: bool = True) -> dict[str, Any]:
        if not self.active:
            return self.to_dict()
        if not localization_trusted:
            return self.stop_safe("localization_lost")
        if distance_units < 0:
            return self.stop_safe("negative_progress")
        self.progress_units = min(self.total_units, self.progress_units + distance_units)
        return self.to_dict()

    def reach_junction(self, node_id: str) -> dict[str, Any]:
        if not self.active:
            return self.to_dict()
        maneuvers = [item for item in self.route.get("maneuvers", []) if item.get("instruction") != "alignment"]
        if self.maneuver_index >= len(maneuvers):
            return self.stop_safe("unexpected_junction")
        expected = maneuvers[self.maneuver_index]
        if expected.get("nodeId") != node_id:
            return self.stop_safe("junction_mismatch")
        self._event("APPROACH_JUNCTION", f"Ожидаемый узел {node_id}: поворот уже рассчитан по карте.")
        return self._set("APPROACH_JUNCTION")

    def confirm_junction(self, node_id: str) -> dict[str, Any]:
        if self.state != "APPROACH_JUNCTION":
            return self.stop_safe("junction_out_of_order")
        maneuvers = [item for item in self.route.get("maneuvers", []) if item.get("instruction") != "alignment"]
        expected = maneuvers[self.maneuver_index]
        if expected.get("nodeId") != node_id:
            return self.stop_safe("junction_mismatch")
        self.maneuver_index += 1
        self._event("FOLLOW_ROAD", f"Узел {node_id} подтверждён в модели, продолжение по маршруту.")
        return self._set("FOLLOW_ROAD")

    def object_risk(self) -> dict[str, Any]:
        if not self.active:
            return self.to_dict()
        self._event("YIELD", "Приближающийся объект в коридоре: остановка и ожидание.")
        return self._set("YIELD", "moving_object")

    def resume_after_yield(self) -> dict[str, Any]:
        if self.state != "YIELD":
            return self.to_dict()
        self._event("FOLLOW_ROAD", "Коридор снова свободен, продолжение маршрута.")
        return self._set("FOLLOW_ROAD")

    def ground_hazard(self, kind: str, *, bypass: Optional[dict[str, Any]] = None) -> dict[str, Any]:
        if not self.active:
            return self.to_dict()
        if kind == "bump":
            self._event("SLOW_FOR_BUMP", "Кочка впереди: малая скорость до проезда задними колёсами.")
            return self._set("SLOW_FOR_BUMP", "bump_ahead")
        if kind == "pothole":
            if bypass is None:
                self._event("STOP_SAFE", "Яма перекрывает проезд: безопасного объезда нет.")
                return self._set("STOP_SAFE", "blocked_by_road_hazard")
            self._event("AVOID_POTHOLE", f"Остановка перед ямой и объезд {bypass.get('side')} внутри дороги.")
            return self._set("AVOID_POTHOLE", "pothole_bypass")
        return self.stop_safe("unknown_hazard")

    def resume_after_hazard(self) -> dict[str, Any]:
        if self.state not in {"SLOW_FOR_BUMP", "AVOID_POTHOLE"}:
            return self.to_dict()
        self._event("FOLLOW_ROAD", "Препятствие покрытия пройдено, возврат к линии движения.")
        return self._set("FOLLOW_ROAD")

    # --- ПДД ---
    def wait_traffic_light(self, light_id: str, state: str = "red") -> dict[str, Any]:
        if not self.active:
            return self.to_dict()
        color = {"red": "красный", "yellow": "жёлтый", "green": "зелёный"}.get(state, state)
        self._event("WAIT_TRAFFIC_LIGHT", f"Светофор {light_id}: {color} — остановка перед стоп-линией.")
        return self._set("WAIT_TRAFFIC_LIGHT", f"traffic_light_{state}")

    def proceed_on_green(self, light_id: str) -> dict[str, Any]:
        if self.state != "WAIT_TRAFFIC_LIGHT":
            return self.to_dict()
        self._event("FOLLOW_ROAD", f"Светофор {light_id}: зелёный — движение разрешено.")
        return self._set("FOLLOW_ROAD")

    def violation_traffic_light(self, light_id: str) -> dict[str, Any]:
        self._event("STOP_SAFE", f"Нарушение ПДД: проезд на красный светофор {light_id}.")
        return self._set("STOP_SAFE", "traffic_violation_red_light")

    def wait_stop_sign(self, sign_id: str) -> dict[str, Any]:
        if not self.active:
            return self.to_dict()
        self._event("WAIT_STOP", f"Знак СТОП {sign_id}: полная остановка.")
        return self._set("WAIT_STOP", "stop_sign")

    def proceed_after_stop(self, sign_id: str) -> dict[str, Any]:
        if self.state != "WAIT_STOP":
            return self.to_dict()
        self._event("FOLLOW_ROAD", f"Знак СТОП {sign_id}: выдержка выполнена, движение продолжено.")
        return self._set("FOLLOW_ROAD")

    def violation_stop_sign(self, sign_id: str) -> dict[str, Any]:
        self._event("STOP_SAFE", f"Нарушение ПДД: проезд без остановки у знака СТОП {sign_id}.")
        return self._set("STOP_SAFE", "traffic_violation_stop_sign")

    def wait_pedestrian(self, cross_id: str) -> dict[str, Any]:
        if not self.active:
            return self.to_dict()
        self._event("WAIT_PEDESTRIAN", f"Пешеходный переход {cross_id}: пешеход на дороге — уступаем.")
        return self._set("WAIT_PEDESTRIAN", "pedestrian_crossing")

    def resume_after_pedestrian(self, cross_id: str) -> dict[str, Any]:
        if self.state != "WAIT_PEDESTRIAN":
            return self.to_dict()
        self._event("FOLLOW_ROAD", f"Пешеходный переход {cross_id}: путь свободен.")
        return self._set("FOLLOW_ROAD")

    def violation_pedestrian(self, cross_id: str) -> dict[str, Any]:
        self._event("STOP_SAFE", f"Нарушение ПДД: не уступил пешеходу на переходе {cross_id}.")
        return self._set("STOP_SAFE", "traffic_violation_pedestrian")

    def speed_limit_violation(self, limit: float) -> dict[str, Any]:
        self._event("STOP_SAFE", f"Нарушение ПДД: превышение скорости, лимит {limit}.")
        return self._set("STOP_SAFE", "traffic_violation_speed")

    def arrive(self) -> dict[str, Any]:
        if not self.active:
            return self.to_dict()
        maneuvers = [item for item in self.route.get("maneuvers", []) if item.get("instruction") != "alignment"]
        if self.progress_units < self.total_units - 1e-6 or self.maneuver_index < len(maneuvers):
            return self.stop_safe("delivery_not_reached")
        self._event("DOCK_DELIVERY", "Пункт доставки достигнут в модели, остановка у цели.")
        return self._set("DOCK_DELIVERY")

    def complete_unload(self, *, confirmed: bool = True) -> dict[str, Any]:
        if self.state != "DOCK_DELIVERY":
            return self.to_dict()
        if not confirmed:
            return self.stop_safe("unload_not_confirmed")
        self._event("UNLOAD", "Выгрузка подтверждена в синтетическом сценарии.")
        self._set("UNLOAD")
        self._event("DONE", "Сухой прогон завершён без подключения тяги.")
        return self._set("DONE")

    def stop_safe(self, reason: str) -> dict[str, Any]:
        if self.state in TERMINAL_STATES:
            return self.to_dict()
        self._event("STOP_SAFE", f"Безопасная остановка: {reason}.")
        return self._set("STOP_SAFE", reason)

    def cancel(self, reason: str = "cancelled") -> dict[str, Any]:
        return self.stop_safe(reason)

    def to_dict(self) -> dict[str, Any]:
        return {"state": self.state, "reason": self.reason, "active": self.active,
                "progressUnits": round(self.progress_units, 3),
                "totalDistanceUnits": round(self.total_units, 3),
                "maneuverIndex": self.maneuver_index, "events": list(self.events),
                "hardwareEnabled": False, "tractionConnected": False}
