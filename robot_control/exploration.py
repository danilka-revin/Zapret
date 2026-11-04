"""Автономное обследование: робот сам достраивает карту по границам неизвестного.

Использует уже существующую осторожную навигацию (:mod:`navigation`) и карту
(:mod:`slam_map`): на каждом шаге выбирается ближайшая доступная граница
UNKNOWN-области, строится маршрут, исполняется с теми же проверками
безопасности, карта сохраняется, шаг повторяется. Это **не** Nav2: скорость
ограничена, движение без разворота на месте (±28°), любая потеря скана,
E-STOP, кнопка «Стоп» или ручной ввод останавливают цикл.

Ближняя слепая зона лидара — единственное место, где оператор обязан
подтвердить осмотр (`blindZoneChecked`) перед запуском обследования; свежий
кадр Kinect 360 дополнительно требуется для старта (`RC_EXPLORE_REQUIRE_DEPTH`).
На каждом шаге с четырёх C930e снимаются «визуальные ориентиры»: кадры с
привязкой к позе, чтобы потом сверить карту с тем, что робот реально видел.

Все настройки — `.env`, шаблон и инструкция в `docs/AUTONOMY.md`.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Any, Callable, Optional

from . import estop as estop_latch
from . import eventlog
from . import sensor_layout
from .config import PROJECT_ROOT
from .lidar import normalize_angle
from .navigation import FREE_MAX, OCCUPIED_MIN, NavigationError
from .slam_map import UNKNOWN

DEFAULT_MAX_STEPS = 40
DEFAULT_MAX_DURATION_S = 900.0
DEFAULT_POLL_S = 0.5
DEFAULT_STEP_TIMEOUT_S = 180.0
DEFAULT_MIN_GAIN_CELLS = 12
DEFAULT_MAX_NO_GAIN = 3
DEFAULT_MAX_FAILURES = 3
DEFAULT_GOAL_AHEAD_M = 0.8
DEFAULT_MIN_GOAL_M = 0.5
DEFAULT_MAX_GOAL_M = 6.0
DEFAULT_MIN_CLUSTER_CELLS = 4
DEFAULT_MAX_CANDIDATES = 6
DEFAULT_LANDMARK_LIMIT = 60


class ExplorationError(ValueError):
    def __init__(self, code: str, message: str, status: int = 409):
        super().__init__(message)
        self.code = code
        self.message = message
        self.status = status

    def payload(self) -> dict[str, Any]:
        return {"ok": False, "code": self.code, "error": self.message}


def _env_bool(name: str, default: bool) -> bool:
    raw = (os.environ.get(name) or "").strip().lower()
    if not raw:
        return default
    return raw in {"1", "true", "yes", "on", "да"}


def _env_float(name: str, default: float, lo: float, hi: float) -> float:
    try:
        value = float((os.environ.get(name) or "").strip() or default)
    except (TypeError, ValueError):
        return default
    return max(lo, min(hi, value)) if math.isfinite(value) else default


def _env_int(name: str, default: int, lo: int, hi: int) -> int:
    try:
        value = int((os.environ.get(name) or "").strip() or default)
    except (TypeError, ValueError):
        return default
    return max(lo, min(hi, value))


def known_cells(snapshot: dict) -> int:
    """Сколько клеток карты уже исследовано (не UNKNOWN)."""
    data = snapshot.get("data") or b""
    return sum(1 for value in data if value != UNKNOWN)


def coverage_pct(snapshot: dict) -> float:
    data = snapshot.get("data") or b""
    if not data:
        return 0.0
    return round(100.0 * known_cells(snapshot) / len(data), 2)


def _cell_center(snapshot: dict, ix: int, iy: int) -> tuple[float, float]:
    origin = snapshot.get("origin") or {}
    res = float(snapshot["resolution"])
    return (float(origin.get("x", 0.0)) + (ix + 0.5) * res,
            float(origin.get("y", 0.0)) + (iy + 0.5) * res)


def _clamp_goal(snapshot: dict, x: float, y: float) -> tuple[float, float]:
    origin = snapshot.get("origin") or {}
    res = float(snapshot["resolution"])
    lo_x = float(origin.get("x", 0.0)) + res
    lo_y = float(origin.get("y", 0.0)) + res
    hi_x = lo_x + (int(snapshot["width"]) - 2) * res
    hi_y = lo_y + (int(snapshot["height"]) - 2) * res
    return (max(lo_x, min(hi_x, x)), max(lo_y, min(hi_y, y)))


def find_frontiers(snapshot: dict, *, min_cluster_cells: int = DEFAULT_MIN_CLUSTER_CELLS,
                   min_goal_m: float = DEFAULT_MIN_GOAL_M,
                   max_goal_m: float = DEFAULT_MAX_GOAL_M,
                   goal_ahead_m: float = DEFAULT_GOAL_AHEAD_M,
                   max_candidates: int = DEFAULT_MAX_CANDIDATES) -> list[dict[str, Any]]:
    """Найти кластеры границы «свободно → неизвестно» и предложить цели.

    Цель ставится чуть дальше центра кластера: `plan_route` сам построит
    безопасный префикс до подтверждённой границы (см. `navigation.plan_route`).
    Кластеры рядом со стенами не предлагаются, а направление вперёд имеет
    приоритет — у шасси нет разворота на месте.
    """
    try:
        width, height = int(snapshot["width"]), int(snapshot["height"])
        data = snapshot["data"]
        pose = snapshot.get("pose") or {}
        px, py = float(pose.get("x", 0.0)), float(pose.get("y", 0.0))
        yaw = float(pose.get("yaw", 0.0))
    except (KeyError, TypeError, ValueError):
        return []
    if width <= 2 or height <= 2 or len(data) != width * height:
        return []

    frontier = bytearray(width * height)
    for iy in range(1, height - 1):
        row = iy * width
        for ix in range(1, width - 1):
            idx = row + ix
            if data[idx] != UNKNOWN:
                continue
            neighbours = (data[idx - 1], data[idx + 1], data[idx - width], data[idx + width])
            if not any(value <= FREE_MAX for value in neighbours):
                continue
            # Соседняя стена: граница у препятствия — плохая цель.
            if any(OCCUPIED_MIN <= value < UNKNOWN for value in neighbours):
                continue
            frontier[idx] = 1

    visited = bytearray(width * height)
    clusters: list[dict[str, Any]] = []
    for idx in range(width * height):
        if not frontier[idx] or visited[idx]:
            continue
        visited[idx] = 1
        stack = [idx]
        members: list[int] = []
        while stack:
            cell = stack.pop()
            members.append(cell)
            cx, cy = cell % width, cell // width
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    nx, ny = cx + dx, cy + dy
                    if not (0 <= nx < width and 0 <= ny < height):
                        continue
                    neighbour = ny * width + nx
                    if frontier[neighbour] and not visited[neighbour]:
                        visited[neighbour] = 1
                        stack.append(neighbour)
        if len(members) < min_cluster_cells:
            continue
        sx = sum(_cell_center(snapshot, m % width, m // width)[0] for m in members)
        sy = sum(_cell_center(snapshot, m % width, m // width)[1] for m in members)
        centre_x, centre_y = sx / len(members), sy / len(members)
        dx, dy = centre_x - px, centre_y - py
        distance = math.hypot(dx, dy)
        if not min_goal_m <= distance <= max_goal_m:
            continue
        direction = normalize_angle(math.atan2(dy, dx) - yaw)
        # Позади робота ехать почти невозможно без разворота на месте.
        direction_penalty = 1.0 + 2.0 * abs(direction) / math.pi
        goal_x, goal_y = _clamp_goal(
            snapshot,
            centre_x + (dx / distance) * goal_ahead_m,
            centre_y + (dy / distance) * goal_ahead_m,
        )
        clusters.append({
            "x": round(centre_x, 3), "y": round(centre_y, 3),
            "goal": {"x": round(goal_x, 3), "y": round(goal_y, 3)},
            "cells": len(members), "distanceM": round(distance, 3),
            "headingDeg": round(math.degrees(direction), 1),
            "score": round(len(members) / (1.0 + distance) / direction_penalty, 4),
        })
    clusters.sort(key=lambda item: (-item["score"], item["distanceM"]))
    return clusters[:max_candidates]


class LandmarkRecorder:
    """Кадры четырёх камер с привязкой к позе (только локальные файлы)."""

    def __init__(self, camera_provider: Callable[[], Any], *,
                 directory: Optional[Path] = None, clock=time.time):
        self._cameras = camera_provider
        self._clock = clock
        self._dir = Path(directory) if directory is not None else PROJECT_ROOT / "data" / "landmarks"
        self._lock = threading.RLock()
        self._index_path = self._dir / "index.json"

    @property
    def directory(self) -> Path:
        return self._dir

    def _read(self) -> list[dict[str, Any]]:
        try:
            payload = json.loads(self._index_path.read_text(encoding="utf-8"))
            return payload if isinstance(payload, list) else []
        except (OSError, ValueError):
            return []

    def _write(self, entries: list[dict[str, Any]]) -> None:
        self._dir.mkdir(parents=True, exist_ok=True)
        tmp = self._dir / ".index.tmp"
        tmp.write_text(json.dumps(entries, ensure_ascii=False, indent=2), encoding="utf-8")
        os.replace(tmp, self._index_path)

    def capture(self, step: int, pose: dict[str, Any], goal: Optional[dict[str, Any]] = None,
                note: str = "") -> Optional[dict[str, Any]]:
        """Сохранить по одному свежему кадру с каждой камеры."""
        with self._lock:
            try:
                manager = self._cameras()
            except Exception:
                return None
            if manager is None:
                return None
            frames: list[dict[str, Any]] = []
            for cam_id in range(4):
                try:
                    device = manager.get_camera(cam_id)
                    role_check = getattr(device, "role_verified", None) if device else None
                    # Never store an orientation-labelled landmark from a
                    # guessed/default or socket-only camera association.
                    if callable(role_check) and role_check() is False:
                        continue
                    jpeg = manager.get_frame(cam_id, max_age_s=2.0)
                except Exception:
                    continue
                if not jpeg:
                    continue
                self._dir.mkdir(parents=True, exist_ok=True)
                name = f"step{step:03d}-cam{cam_id}.jpg"
                try:
                    (self._dir / name).write_bytes(jpeg)
                except OSError:
                    continue
                frames.append({
                    "camera": cam_id,
                    "role": getattr(device, "role", "") if device else "",
                    "yawDeg": (device.spec.get("yaw_deg") if device else None),
                    "file": name,
                })
            entry = {
                "step": int(step), "at": self._clock(),
                "pose": {key: pose.get(key) for key in ("x", "y", "yaw")},
                "goal": dict(goal) if goal else None,
                "note": str(note)[:120], "frames": frames,
            }
            entries = self._read()
            entries.append(entry)
            self._write(entries[-DEFAULT_LANDMARK_LIMIT:])
            return entry

    def list(self, limit: int = 20) -> list[dict[str, Any]]:
        with self._lock:
            return self._read()[-max(1, min(200, int(limit))):]

    def clear(self) -> int:
        with self._lock:
            entries = self._read()
            for item in entries:
                for frame in item.get("frames", []):
                    try:
                        (self._dir / str(frame["file"])).unlink()
                    except OSError:
                        pass
            self._write([])
            return len(entries)


class ExplorationService:
    """Фоновый цикл «найти границу → доехать → сохранить карту → повторить»."""

    def __init__(self, map_service, navigation, *, camera_provider=None,
                 kinect: Any = None, enabled: Optional[bool] = None,
                 clock=time.time, estop_check=None, landmarks: Optional[LandmarkRecorder] = None):
        self.map_service = map_service
        self.navigation = navigation
        self.kinect = kinect
        self.clock = clock
        self._estop = estop_check or estop_latch.latched
        self.enabled = _env_bool("RC_EXPLORE_ENABLE", False) if enabled is None else bool(enabled)
        self.max_steps = _env_int("RC_EXPLORE_MAX_STEPS", DEFAULT_MAX_STEPS, 1, 500)
        self.max_duration_s = _env_float("RC_EXPLORE_MAX_DURATION_S", DEFAULT_MAX_DURATION_S, 30.0, 7200.0)
        self.poll_s = _env_float("RC_EXPLORE_POLL_S", DEFAULT_POLL_S, 0.1, 5.0)
        self.step_timeout_s = _env_float("RC_EXPLORE_STEP_TIMEOUT_S", DEFAULT_STEP_TIMEOUT_S, 10.0, 900.0)
        self.min_gain = _env_int("RC_EXPLORE_MIN_GAIN_CELLS", DEFAULT_MIN_GAIN_CELLS, 0, 10000)
        self.max_no_gain = _env_int("RC_EXPLORE_MAX_NO_GAIN", DEFAULT_MAX_NO_GAIN, 1, 20)
        self.max_failures = _env_int("RC_EXPLORE_MAX_FAILURES", DEFAULT_MAX_FAILURES, 1, 20)
        self.goal_ahead_m = _env_float("RC_EXPLORE_GOAL_AHEAD_M", DEFAULT_GOAL_AHEAD_M, 0.1, 3.0)
        self.min_goal_m = _env_float("RC_EXPLORE_MIN_GOAL_M", DEFAULT_MIN_GOAL_M, 0.1, 5.0)
        self.max_goal_m = _env_float("RC_EXPLORE_MAX_GOAL_M", DEFAULT_MAX_GOAL_M, 0.5, 12.0)
        self.min_cluster_cells = _env_int("RC_EXPLORE_MIN_CLUSTER_CELLS", DEFAULT_MIN_CLUSTER_CELLS, 1, 200)
        self.max_candidates = _env_int("RC_EXPLORE_MAX_CANDIDATES", DEFAULT_MAX_CANDIDATES, 1, 20)
        self.require_depth = _env_bool("RC_EXPLORE_REQUIRE_DEPTH", True)
        self.require_blind_check = _env_bool("RC_EXPLORE_REQUIRE_BLIND_CHECK", True)
        self._landmarks = landmarks if landmarks is not None else LandmarkRecorder(
            camera_provider or (lambda: None), clock=clock)
        self._lock = threading.RLock()
        self._cancel = threading.Event()
        self._worker: Optional[threading.Thread] = None
        self._state: dict[str, Any] = {
            "phase": "idle", "message": "Обследование не запущено",
            "steps": 0, "coveragePct": 0.0, "frontiers": 0,
            "lastGoal": None, "lastGainCells": 0, "updatedAt": self.clock(),
        }

    @property
    def active(self) -> bool:
        with self._lock:
            return bool(self._worker and self._worker.is_alive())

    def _set(self, **fields: Any) -> None:
        with self._lock:
            self._state.update(fields)
            self._state["updatedAt"] = self.clock()

    def _snapshot(self) -> dict:
        return self.map_service.navigation_snapshot()

    def _kinect_fresh(self) -> bool:
        if self.kinect is None:
            return False
        try:
            return bool(self.kinect.fresh())
        except Exception:
            return False

    def start(self, body: Any = None) -> tuple[dict[str, Any], int]:
        """Запустить обследование. Требует явного подтверждения оператора."""
        payload = body if isinstance(body, dict) else {}
        if not self.enabled:
            raise ExplorationError("disabled", "Автообследование выключено (RC_EXPLORE_ENABLE=1)", 409)
        if self.active:
            raise ExplorationError("already_running", "Обследование уже идёт", 409)
        if not getattr(self.navigation, "enabled", False):
            raise ExplorationError("navigation_disabled", "Сначала включите навигацию (RC_NAV_ENABLE=1)", 409)
        if self._estop():
            raise ExplorationError("estop", "E-STOP взведён: снимите аварийную остановку", 409)
        if self.require_depth and not self._kinect_fresh():
            raise ExplorationError("depth_unavailable",
                                   "Нет свежего кадра Kinect 360: ближняя зона не просматривается", 409)
        if self.require_blind_check and payload.get("blindZoneChecked") is not True:
            raise ExplorationError("blind_zone_confirmation_required",
                                   "Осмотрите ближнюю зону вокруг робота и подтвердите это перед стартом", 409)
        with self._lock:
            if self.active:
                raise ExplorationError("already_running", "Обследование уже идёт", 409)
            self._cancel = threading.Event()
            self._state = {
                "phase": "starting", "message": "Проверка карты и датчиков…",
                "steps": 0, "coveragePct": coverage_pct(self._snapshot()),
                "frontiers": 0, "lastGoal": None, "lastGainCells": 0,
                "updatedAt": self.clock(),
            }
            worker = threading.Thread(target=self._run, args=(self._cancel,),
                                      name="robot-exploration", daemon=True)
            self._worker = worker
            worker.start()
        eventlog.record("ok", "автообследование запущено оператором", source="навигация")
        return {"ok": True, "status": self.status()}, 202

    def stop(self, reason: str = "Остановлено оператором") -> dict[str, Any]:
        with self._lock:
            self._cancel.set()
        try:
            self.navigation.cancel(reason)
        except Exception:
            pass
        self._set(phase="cancelled", message=reason)
        eventlog.record("warn", "автообследование остановлено: %s" % reason, source="навигация")
        return {"ok": True, "status": self.status()}

    def preflight(self) -> dict[str, Any]:
        """Что мешает пуску, а что стоит просто проверить — для панели UI.

        ``blockers`` повторяет ровно те проверки, которые :meth:`start` всё
        равно выполнит; ``warnings`` не блокируют кнопку, а подсказывают, что
        карта или геометрия ещё не готовы к точной работе.
        """
        blockers: list[str] = []
        warnings: list[str] = []
        if not self.enabled:
            blockers.append("Автообследование выключено: задайте RC_EXPLORE_ENABLE=1")
        if not getattr(self.navigation, "enabled", False):
            blockers.append("Навигация выключена: задайте RC_NAV_ENABLE=1")
        if self._estop():
            blockers.append("Взведён E-STOP: снимите аварийную остановку")
        if self.require_depth and not self._kinect_fresh():
            blockers.append("Нет свежего кадра Kinect 360: запустите публикатор глубины")
        if self.require_blind_check:
            warnings.append("Перед пуском осмотрите ближнюю зону вокруг робота")
        try:
            if known_cells(self._snapshot()) == 0:
                warnings.append("Карта пуста: дождитесь свежего скана лидара")
        except Exception:  # noqa: BLE001 — статус не должен ронять экран
            warnings.append("Не удалось прочитать карту")
        if not sensor_layout.geometry_calibrated():
            warnings.append("Геометрия не откалибрована: углы взяты по умолчанию — "
                            "откройте «Калибровка геометрии» и внесите замеры")
        duplicate = sensor_layout.layout_payload().get("duplicatePorts") or []
        if duplicate:
            warnings.append("Один порт у нескольких камер: " + ", ".join(duplicate))
        return {"blockers": blockers, "warnings": warnings,
                "canStart": not blockers and not self.active}

    def status(self) -> dict[str, Any]:
        with self._lock:
            state = dict(self._state)
        try:
            snapshot = self._snapshot()
            state["coveragePct"] = coverage_pct(snapshot)
            state["knownCells"] = known_cells(snapshot)
            state["frontiers"] = len(find_frontiers(
                snapshot, min_cluster_cells=self.min_cluster_cells,
                min_goal_m=self.min_goal_m, max_goal_m=self.max_goal_m,
                goal_ahead_m=self.goal_ahead_m, max_candidates=self.max_candidates))
        except Exception as exc:  # noqa: BLE001 — статус не должен ронять экран
            state["mapError"] = str(exc)[:160]
        state.update(self.preflight())
        state.update({
            "ok": True,
            "enabled": self.enabled,
            "active": self.active,
            "navigationEnabled": bool(getattr(self.navigation, "enabled", False)),
            "navigationActive": bool(getattr(self.navigation, "active", False)),
            "maxSteps": self.max_steps,
            "kinectFresh": self._kinect_fresh(),
            "requireDepth": self.require_depth,
            "requireBlindCheck": self.require_blind_check,
            "landmarks": len(self._landmarks.list(limit=200)),
        })
        return state

    @property
    def landmark_directory(self) -> Path:
        return self._landmarks.directory

    def landmarks(self, limit: int = 20) -> list[dict[str, Any]]:
        return self._landmarks.list(limit=limit)

    def clear_landmarks(self) -> int:
        return self._landmarks.clear()

    # -- рабочий цикл ------------------------------------------------------- #
    def _wait_navigation(self, cancel: threading.Event) -> bool:
        """Ждать окончания поездки; False — отмена/E-STOP/таймаут."""
        deadline = self.clock() + self.step_timeout_s
        while not cancel.is_set():
            if self._estop():
                self.navigation.cancel("E-STOP: обследование остановлено.")
                self._finish("error", "E-STOP взведён во время движения.")
                return False
            if not getattr(self.navigation, "active", False):
                return True
            if self.clock() > deadline:
                self.navigation.cancel("Поездка превысила таймаут обследования.")
                self._finish("error", "Поездка не завершилась вовремя.")
                return False
            cancel.wait(self.poll_s)
        return False

    def _finish(self, phase: str, message: str, level: str = "ok") -> None:
        self._set(phase=phase, message=message)
        eventlog.record(level, "автообследование: %s" % message, source="навигация")

    def _save_map(self) -> None:
        try:
            self.map_service.save()
        except Exception as exc:  # noqa: BLE001 — запись файла не ломает цикл
            eventlog.record("warn", "карта обследования не сохранена: %s" % str(exc)[:120],
                            source="карта")

    def _capture(self, step: int, snapshot: dict, goal: Optional[dict], note: str) -> None:
        try:
            self._landmarks.capture(step, snapshot.get("pose") or {}, goal, note)
        except Exception:
            pass

    def _run(self, cancel: threading.Event) -> None:
        started = self.clock()
        steps = 0
        no_gain = 0
        failures = 0
        try:
            while not cancel.is_set():
                if self._estop():
                    self.navigation.cancel("E-STOP: обследование остановлено.")
                    self._finish("error", "E-STOP взведён.", "warn")
                    break
                if self.clock() - started > self.max_duration_s:
                    self._finish("done", "Обследование завершено по времени.")
                    break
                if steps >= self.max_steps:
                    self._finish("done", "Достигнут предел шагов (%d)." % self.max_steps)
                    break
                if getattr(self.navigation, "active", False):
                    if not self._wait_navigation(cancel):
                        break
                    continue
                snapshot = self._snapshot()
                candidates = find_frontiers(
                    snapshot, min_cluster_cells=self.min_cluster_cells,
                    min_goal_m=self.min_goal_m, max_goal_m=self.max_goal_m,
                    goal_ahead_m=self.goal_ahead_m, max_candidates=self.max_candidates)
                if not candidates:
                    self._finish("done", "Новых границ нет — карта обследована.")
                    break
                before = known_cells(snapshot)
                self._set(phase="planning", message="Ищу границу неизведанного…",
                          frontiers=len(candidates), steps=steps,
                          coveragePct=coverage_pct(snapshot))
                launched = False
                last_error = ""
                for candidate in candidates:
                    if cancel.is_set() or self._estop():
                        break
                    try:
                        self.navigation.start(
                            {"goal": candidate["goal"], "blindZoneChecked": True,
                             "explore": True},
                            autonomous=True,
                        )
                        launched = True
                        self._set(phase="driving", lastGoal=candidate["goal"],
                                  message="Еду к границе карты (%.2f, %.2f) м." % (
                                      candidate["goal"]["x"], candidate["goal"]["y"]))
                        break
                    except NavigationError as exc:
                        last_error = "%s: %s" % (exc.code, exc.message)
                        if exc.code in {"not_ready", "drive_unavailable", "navigation_active"}:
                            break
                        continue
                if not launched:
                    failures += 1
                    if failures >= self.max_failures:
                        self._finish("error", last_error or "Не удалось построить маршрут.", "err")
                        break
                    self._set(phase="waiting", message=last_error or "Маршрут недоступен, жду новые сканы.")
                    cancel.wait(self.poll_s * 4)
                    continue
                failures = 0
                self._capture(steps, snapshot, candidates[0]["goal"], "departure")
                if not self._wait_navigation(cancel):
                    break
                steps += 1
                after_snapshot = self._snapshot()
                gain = known_cells(after_snapshot) - before
                self._save_map()
                self._capture(steps, after_snapshot, None, "arrival")
                self._set(steps=steps, lastGainCells=gain,
                          phase="mapping", message="Шаг %d: +%d клеток карты." % (steps, gain),
                          coveragePct=coverage_pct(after_snapshot))
                if self.require_depth and not self._kinect_fresh():
                    self._finish("error", "Пропал кадр Kinect 360 — движение остановлено.", "warn")
                    break
                if gain < self.min_gain:
                    no_gain += 1
                    if no_gain >= self.max_no_gain:
                        self._finish("done", "Карта перестала расти — обследование завершено.")
                        break
                else:
                    no_gain = 0
        except Exception as exc:  # noqa: BLE001 — поток обязан завершиться с ошибкой в статусе
            self._finish("error", "Сбой обследования: %s" % str(exc)[:160], "err")
        finally:
            if cancel.is_set():
                self._set(phase="cancelled", message="Обследование остановлено.")
            with self._lock:
                self._worker = None
