"""Сервис экрана доставки: карта, стартовая поза, цель и сухой прогон.

Не открывает последовательный порт, не создаёт исполнителя тяги и не вызывает аппаратный E-STOP.
Единственное «движение» здесь — детерминированная модель `delivery_sim`.
Постоянная карта читается из файла и не изменяется сервисом; настройка старта и
цели сохраняется отдельно в `RC_DELIVERY_DIR/setup.json`.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Any, Optional

from .config import PROJECT_ROOT
from .delivery_sim import SCENARIOS, SimulationConfig, simulate_route
from .road_map import RoadMap, RoadMapError, atomic_write_json
from .road_route import RoadRoutePlanner

DEFAULT_MAP_PATH = PROJECT_ROOT / "docs" / "delivery_map_draft.json"
DEFAULT_DIRECTORY = PROJECT_ROOT / "data" / "delivery"
SNAP_TOLERANCE_UNITS = 25.0
HEADING_TOLERANCE_DEG = 40.0

SCENARIO_LABELS = {
    "clear": "Чистая дорога",
    "bump": "Кочка: замедлиться",
    "pothole_bypass": "Яма: объехать внутри дороги",
    "pothole_blocked": "Яма: объезда нет, остановиться",
    "moving_object": "Приближающийся объект: уступить",
    "localization_lost": "Потеря локализации: остановиться",
    "traffic_light_red": "Светофор: красный — дождаться зелёного",
    "traffic_light_green": "Светофор: зелёный — проехать",
    "stop_sign": "Знак СТОП: полная остановка",
    "pedestrian_crossing": "Пешеходный переход: уступить пешеходу",
    "speed_limit": "Ограничение скорости: снизить",
}

CHECK_LABELS = {
    "mapLoaded": "Векторная карта загружена и прошла проверку схемы",
    "startPoseSet": "Старт задан точкой и курсом на разрешённом проезде",
    "goalSelected": "Выбран пункт доставки или док",
    "routeFound": "Маршрут построен по графу дорог",
    "draftAcknowledged": "Подтверждено, что карта черновая и масштаб не подтверждён",
    "dryRunOnly": "Режим только сухого прогона",
    "hardwareDisabled": "Тяга и аппаратные команды отключены",
}

DRIVE_BLOCKER_LABELS = {
    "dry_run_only": "Этот этап проверяет логику без движения робота",
    "no_motion_arbiter_connected": "Арбитр реального движения не подключён",
    "map_not_calibrated": "Масштаб и топология карты не подтверждены",
    "scale_missing": "Не задан масштаб метров на единицу карты",
    "route_invalid": "Маршрут не построен",
}


class DeliveryError(ValueError):
    def __init__(self, code: str, message: str, status: int = 400):
        super().__init__(message)
        self.code = code
        self.message = message
        self.status = status

    def payload(self) -> dict[str, Any]:
        return {"ok": False, "code": self.code, "error": self.message}


def _number(value: Any, name: str, lo: float, hi: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise DeliveryError("invalid_number", f"{name}: ожидается число") from None
    if not math.isfinite(result) or not lo <= result <= hi:
        raise DeliveryError("invalid_number", f"{name}: число вне диапазона")
    return result


class DeliveryService:
    def __init__(self, map_path: Optional[Path | str] = None,
                 directory: Optional[Path | str] = None) -> None:
        self.map_path = Path(map_path or os.environ.get("RC_DELIVERY_MAP") or DEFAULT_MAP_PATH)
        self.directory = Path(directory or os.environ.get("RC_DELIVERY_DIR") or DEFAULT_DIRECTORY)
        self.setup_path = self.directory / "setup.json"
        self._lock = threading.RLock()
        self._road_map: Optional[RoadMap] = None
        self._planner: Optional[RoadRoutePlanner] = None
        self._map_error: Optional[dict[str, Any]] = None
        self._setup: Optional[dict[str, Any]] = None
        self._last_plan: Optional[dict[str, Any]] = None
        self._last_simulation: Optional[dict[str, Any]] = None
        self._load_map()
        self._load_setup()

    def _load_map(self) -> None:
        try:
            self._road_map = RoadMap.from_file(self.map_path)
            self._planner = RoadRoutePlanner(self._road_map)
            self._map_error = None
        except RoadMapError as exc:
            self._road_map = None
            self._planner = None
            self._map_error = exc.payload()

    def _load_setup(self) -> None:
        try:
            payload = json.loads(self.setup_path.read_text(encoding="utf-8"))
        except (OSError, ValueError, UnicodeError):
            return
        if not isinstance(payload, dict):
            return
        try:
            self._setup = self._validate_setup(payload, require_ack=False)
        except DeliveryError:
            self._setup = None

    def _validate_setup(self, body: dict[str, Any], *, require_ack: bool = True) -> dict[str, Any]:
        if not isinstance(body, dict):
            raise DeliveryError("invalid_setup", "Ожидается JSON-объект настройки")
        start = body.get("start")
        if not isinstance(start, dict):
            raise DeliveryError("invalid_start", "Старт должен содержать x, y и yawDeg")
        goal = body.get("goalNodeId")
        if not isinstance(goal, str) or not goal.strip():
            raise DeliveryError("invalid_goal", "Выберите пункт назначения")
        scenario = body.get("scenario") or "clear"
        if not isinstance(scenario, str) or scenario not in SCENARIOS:
            raise DeliveryError("invalid_scenario", "Неизвестный сценарий сухого прогона")
        ack_raw = body.get("draftAcknowledged", False)
        if not isinstance(ack_raw, bool):
            raise DeliveryError("invalid_acknowledgement", "draftAcknowledged должен быть true/false")
        ack = ack_raw
        if require_ack and self._road_map is not None and self._road_map.status == "draft" and not ack:
            raise DeliveryError("draft_not_acknowledged", "Подтвердите, что карта черновая и не готова к реальному движению.", 409)
        result = {"start": {"x": _number(start.get("x"), "start.x", -1e7, 1e7),
                            "y": _number(start.get("y"), "start.y", -1e7, 1e7),
                            "yawDeg": _number(start.get("yawDeg"), "start.yawDeg", -360.0, 360.0)},
                "goalNodeId": goal.strip(), "scenario": scenario, "draftAcknowledged": ack}
        if self._road_map is not None:
            node = self._road_map.nodes.get(result["goalNodeId"])
            if node is None:
                raise DeliveryError("unknown_goal", "Пункт назначения не найден на карте")
            if node.kind not in {"delivery", "dock"}:
                raise DeliveryError("invalid_goal_kind", "Целью должен быть пункт доставки или док, а не произвольный узел дороги.")
        return result

    def map_payload(self) -> dict[str, Any]:
        with self._lock:
            if self._road_map is None:
                return {"ok": False, "error": "Векторная карта недоступна", "mapError": self._map_error,
                        "hardwareEnabled": False, "tractionConnected": False}
            return {"ok": True, "map": self._road_map.to_dict(), "summary": self._road_map.summary(),
                    "referenceImageUrl": "/delivery_reference.png",
                    "scenarios": [{"id": key, "label": SCENARIO_LABELS[key]} for key in sorted(SCENARIOS)],
                    "checkLabels": dict(CHECK_LABELS), "driveBlockerLabels": dict(DRIVE_BLOCKER_LABELS),
                    "snapToleranceUnits": SNAP_TOLERANCE_UNITS,
                    "headingToleranceDeg": HEADING_TOLERANCE_DEG,
                    "hardwareEnabled": False, "tractionConnected": False}

    def save_setup(self, body: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            if self._road_map is None or self._planner is None:
                raise DeliveryError("map_unavailable", "Векторная карта недоступна", 503)
            setup = self._validate_setup(body, require_ack=False)
            try:
                plan = self._planner.plan(setup["start"], setup["goalNodeId"],
                                          snap_tolerance_units=SNAP_TOLERANCE_UNITS,
                                          heading_tolerance_deg=HEADING_TOLERANCE_DEG)
            except RoadMapError as exc:
                raise DeliveryError(exc.code, str(exc), exc.status) from exc
            atomic_write_json(self.setup_path, setup)
            self._setup = setup
            self._last_plan = plan
            self._last_simulation = None
            return {"ok": True, "setup": setup, "plan": plan, "status": self.status()}

    def plan(self, body: Optional[dict[str, Any]] = None) -> dict[str, Any]:
        with self._lock:
            if self._road_map is None or self._planner is None:
                raise DeliveryError("map_unavailable", "Векторная карта недоступна", 503)
            merged = dict(self._setup or {})
            if isinstance(body, dict):
                if isinstance(body.get("start"), dict):
                    merged["start"] = dict(body["start"])
                for key in ("goalNodeId", "scenario", "draftAcknowledged"):
                    if key in body:
                        merged[key] = body[key]
            setup = self._validate_setup(merged, require_ack=False)
            try:
                plan = self._planner.plan(setup["start"], setup["goalNodeId"],
                                          snap_tolerance_units=SNAP_TOLERANCE_UNITS,
                                          heading_tolerance_deg=HEADING_TOLERANCE_DEG)
            except RoadMapError as exc:
                raise DeliveryError(exc.code, str(exc), exc.status) from exc
            self._setup = setup
            self._last_plan = plan
            self._last_simulation = None
            return {"ok": True, "plan": plan, "status": self.status()}

    def simulate(self, body: Optional[dict[str, Any]] = None) -> dict[str, Any]:
        with self._lock:
            if self._road_map is None or self._planner is None:
                raise DeliveryError("map_unavailable", "Векторная карта недоступна", 503)
            merged = dict(self._setup or {})
            if isinstance(body, dict):
                if isinstance(body.get("start"), dict):
                    merged["start"] = dict(body["start"])
                for key in ("goalNodeId", "scenario", "draftAcknowledged"):
                    if key in body:
                        merged[key] = body[key]
            setup = self._validate_setup(merged, require_ack=True)
            try:
                plan = self._planner.plan(setup["start"], setup["goalNodeId"],
                                          snap_tolerance_units=SNAP_TOLERANCE_UNITS,
                                          heading_tolerance_deg=HEADING_TOLERANCE_DEG)
                result = simulate_route(plan, setup["scenario"], SimulationConfig(), road_map=self._road_map)
            except RoadMapError as exc:
                raise DeliveryError(exc.code, str(exc), exc.status) from exc
            except ValueError as exc:
                raise DeliveryError("simulation_invalid", str(exc), 409) from exc
            self._setup = setup
            self._last_plan = plan
            self._last_simulation = dict(result, createdAt=time.time())
            return {"ok": True, "simulation": self._last_simulation, "plan": plan, "status": self.status()}

    def reset(self) -> dict[str, Any]:
        with self._lock:
            self._last_plan = None
            self._last_simulation = None
            return {"ok": True, "status": self.status()}

    def status(self) -> dict[str, Any]:
        with self._lock:
            setup = self._setup
            plan = self._last_plan
            checks = {
                "mapLoaded": self._road_map is not None,
                "startPoseSet": bool(setup and isinstance(setup.get("start"), dict)),
                "goalSelected": bool(setup and setup.get("goalNodeId")),
                "routeFound": bool(plan and plan.get("ok")),
                "draftAcknowledged": bool(self._road_map is not None and
                                          (self._road_map.status != "draft" or (setup and setup.get("draftAcknowledged")))),
                "dryRunOnly": True,
                "hardwareDisabled": True,
            }
            blockers = [] if self._road_map is None else self._planner.drive_blockers(plan or {})
            return {"ok": True, "mode": "dry_run", "map": None if self._road_map is None else self._road_map.summary(),
                    "mapError": self._map_error, "setup": setup, "lastPlan": plan,
                    "lastSimulation": self._last_simulation, "checks": checks,
                    "checkLabels": dict(CHECK_LABELS), "canSimulate": all(checks.values()),
                    "drivePermitted": False, "driveBlockers": blockers,
                    "driveBlockerLabels": dict(DRIVE_BLOCKER_LABELS),
                    "hardwareEnabled": False, "tractionConnected": False}
