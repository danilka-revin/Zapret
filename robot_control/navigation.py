"""Короткопробежная навигация по карте лидара.

Планировщик не считает неизученные клетки свободными, кроме начальной
ближней слепой зоны датчика, физически проверенной и подтверждённой оператором.
Она фиксирована в координатах карты и не движется вместе с роботом.
Исполнитель намеренно ограничен возможностями
прошивки Zapret: общая тяга всех колёс, одинаковый угол, без разворота на месте.
Поэтому путь состоит только из прямых/крабовых отрезков в секторе ±28° от
текущего курса (запас до аппаратного предела ``STEER_LIMIT_DEG``, сейчас ±100°),
с остановкой перед сменой угла. Автоматическая тяга выключена по умолчанию и требует явного включения и
измеренной калибровки скорости.

Это не Nav2 и не замена сертифицированному защитному контуру. Перед запуском
оператор обязан проверить карту и зону движения; E-STOP и аппаратный watchdog
остаются независимыми от веб-клиента.
"""
from __future__ import annotations

import heapq
import logging
import math
import os
import threading
import time
from math import gcd
from typing import Any, Optional

from robot_control import estop as estop_latch
from robot_control.rus_bridge import STEER_LIMIT_DEG, MasterBridge
from robot_control.rus_core.kinematics import SwerveKinematics
from robot_control.slam_map import DEFAULT_LIDAR_OFFSET_M, lidar_mount_offset

log = logging.getLogger("robot_control.navigation")

FREE_MAX = 19
OCCUPIED_MIN = 65
#: Сектор планирования: намеренно у́же аппаратного предела, чтобы A* строил
#: короткие прямые/крабовые отрезки, а не «боком» через всю карту.
PLAN_STEER_DEG = 28.0
#: Аппаратный предел руля — одна константа на прошивку, сервер и мост.
HARDWARE_STEER_DEG = STEER_LIMIT_DEG
DEFAULT_FOOTPRINT_M = 0.55
DEFAULT_ROBOT_LENGTH_M = 0.75
DEFAULT_ROBOT_WIDTH_M = 0.60
DEFAULT_ROBOT_HEIGHT_M = 0.30
DEFAULT_QUALITY_MIN = 0.55
DEFAULT_SPEED_MPS = 0.05
DEFAULT_MAX_PERCENT = 4
DEFAULT_SCAN_AGE_S = 1.0
DEFAULT_WAYPOINT_TOLERANCE_M = 0.14
MAX_MAP_CELLS = 1_000_000
MAX_PLAN_DISTANCE_M = 12.0
MAX_PLAN_EXPANSIONS = 250_000


class NavigationError(ValueError):
    """Понятная ошибка навигации с HTTP-кодом для API."""

    def __init__(self, code: str, message: str, status: int = 409, *, details: Optional[dict] = None):
        super().__init__(message)
        self.code = code
        self.message = message
        self.status = status
        self.details = details or {}

    def payload(self) -> dict:
        return {"ok": False, "code": self.code, "error": self.message, **self.details}


def _env_bool(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _env_float(name: str, default: float, lo: float, hi: float) -> float:
    try:
        value = float(os.environ.get(name, ""))
    except (TypeError, ValueError):
        return default
    if not math.isfinite(value):
        return default
    return max(lo, min(hi, value))


def _env_optional_positive(name: str) -> Optional[float]:
    try:
        value = float(os.environ.get(name, ""))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) and value > 0 else None


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise NavigationError("invalid_goal", "%s должна быть числом" % name, 400)
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise NavigationError("invalid_goal", "%s должна быть числом" % name, 400)
    if not math.isfinite(result):
        raise NavigationError("invalid_goal", "%s должна быть конечным числом" % name, 400)
    return result


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def steering_deviation(angle: float, yaw: float) -> float:
    """Меньший угол до курса вперёд или назад (крабовый привод)."""
    delta = abs(_wrap_pi(angle - yaw))
    return min(delta, abs(math.pi - delta))


def robot_dimensions_payload(
    footprint_m: float = DEFAULT_FOOTPRINT_M,
    *,
    length_m: float = DEFAULT_ROBOT_LENGTH_M,
    width_m: float = DEFAULT_ROBOT_WIDTH_M,
    height_m: float = DEFAULT_ROBOT_HEIGHT_M,
    lidar_offset_m: float = DEFAULT_LIDAR_OFFSET_M,
) -> dict:
    """Паспортные габариты робота (75 × 60 × 30 см) и положение переднего лидара."""
    return {
        "lengthM": round(length_m, 2),
        "widthM": round(width_m, 2),
        "heightM": round(height_m, 2),
        "lengthCm": int(round(length_m * 100)),
        "widthCm": int(round(width_m * 100)),
        "heightCm": int(round(height_m * 100)),
        "footprintM": round(footprint_m, 2),
        "lidarMount": "front",
        "lidarOffsetM": round(lidar_offset_m, 2),
        "lidarOffsetCm": int(round(lidar_offset_m * 100)),
    }


def _cell(snapshot: dict, x: float, y: float) -> Optional[tuple[int, int]]:
    resolution = float(snapshot["resolution"])
    origin = snapshot["origin"]
    ix = int(math.floor((x - float(origin["x"])) / resolution))
    iy = int(math.floor((y - float(origin["y"])) / resolution))
    if 0 <= ix < int(snapshot["width"]) and 0 <= iy < int(snapshot["height"]):
        return ix, iy
    return None


def _cell_center(snapshot: dict, ix: int, iy: int) -> tuple[float, float]:
    resolution = float(snapshot["resolution"])
    origin = snapshot["origin"]
    return (float(origin["x"]) + (ix + 0.5) * resolution,
            float(origin["y"]) + (iy + 0.5) * resolution)


def _map_values(snapshot: dict) -> tuple[int, int, float, bytes]:
    try:
        width = int(snapshot["width"])
        height = int(snapshot["height"])
        resolution = float(snapshot["resolution"])
        data = bytes(snapshot["data"])
    except (KeyError, TypeError, ValueError, OverflowError):
        raise NavigationError("map_invalid", "Сетка карты повреждена или недоступна", 503)
    if (width < 2 or height < 2 or width * height > MAX_MAP_CELLS
            or not math.isfinite(resolution) or not 0.025 <= resolution <= 1.0
            or len(data) != width * height):
        raise NavigationError("map_invalid", "Размер или разрешение карты не поддерживается", 503)
    return width, height, resolution, data


class _ClearanceGrid:
    """Консервативный квадрат: known free или проверенная слепая зона старта.

    Суммированная таблица позволяет быстро проверить весь габарит, включая
    неизвестные клетки и границы карты. Квадрат больше описанной окружности:
    это намеренный дополнительный запас, а не точная модель корпуса.
    """

    def __init__(self, snapshot: dict, footprint_m: float,
                 blind_zone: Optional[dict] = None):
        width, height, resolution, values = _map_values(snapshot)
        if not math.isfinite(footprint_m) or footprint_m <= 0:
            raise NavigationError("footprint_invalid", "Некорректный габарит робота", 503)
        self.width, self.height = width, height
        self.radius = int(math.ceil(footprint_m / resolution))
        stride = width + 1
        self.stride = stride
        # Python int-таблица ограничена MAX_MAP_CELLS. Все сомнительные и
        # UNKNOWN-клетки считаются препятствиями, а не свободным пространством.
        prefix = [0] * (stride * (height + 1))
        self.ignored_blind_cells = 0
        origin = snapshot["origin"]
        blind_x = float(blind_zone["x"]) if blind_zone else 0.0
        blind_y = float(blind_zone["y"]) if blind_zone else 0.0
        blind_radius = float(blind_zone["radiusM"]) if blind_zone else -1.0
        # A grid cell touching the sensor's blind circle is not fully observed.
        blind_radius += resolution * math.sqrt(0.5) if blind_zone else 0.0
        for y in range(height):
            row_sum = 0
            previous = y * stride
            current = (y + 1) * stride
            data_row = y * width
            for x in range(width):
                value = values[data_row + x]
                blocked = value > FREE_MAX
                if value == 255 and blind_zone:
                    cell_x = float(origin["x"]) + (x + 0.5) * resolution
                    cell_y = float(origin["y"]) + (y + 0.5) * resolution
                    if math.hypot(cell_x - blind_x, cell_y - blind_y) <= blind_radius:
                        # Fixed at the operator-confirmed *starting* sensor
                        # position. Never follows a moving robot into UNKNOWN.
                        blocked = False
                        self.ignored_blind_cells += 1
                row_sum += blocked
                prefix[current + x + 1] = prefix[previous + x + 1] + row_sum
        self.prefix = prefix

    def clear_cell(self, ix: int, iy: int) -> bool:
        r = self.radius
        x0, y0, x1, y1 = ix - r, iy - r, ix + r + 1, iy + r + 1
        if x0 < 0 or y0 < 0 or x1 > self.width or y1 > self.height:
            return False
        p, stride = self.prefix, self.stride
        return p[y1 * stride + x1] - p[y0 * stride + x1] - p[y1 * stride + x0] + p[y0 * stride + x0] == 0

    def blocked_mask(self) -> bytearray:
        return bytearray(0 if self.clear_cell(x, y) else 1
                         for y in range(self.height) for x in range(self.width))

    def clear_line(self, snapshot: dict, a: tuple[float, float],
                   b: tuple[float, float]) -> bool:
        return all(self.clear_cell(ix, iy) for ix, iy in _segment_cells(snapshot, a, b))


def _inflated_blocked(snapshot: dict, footprint_m: float,
                      start: tuple[float, float]) -> tuple[bytearray, bytes]:
    """Маска для центра: весь габарит находится в изученных свободных клетках."""
    del start  # никакое UNKNOWN около робота автоматически не объявляется свободным
    values = _map_values(snapshot)[3]
    return _ClearanceGrid(snapshot, footprint_m).blocked_mask(), values

def _segment_cells(snapshot: dict, a: tuple[float, float], b: tuple[float, float]):
    """Supercover: every grid cell touched by the closed line segment.

    Uniform sampling can miss a thin corner crossing; checking intersections
    and the intervals between them also handles lines exactly on grid borders.
    """
    res = float(snapshot["resolution"])
    origin = snapshot["origin"]
    x0, y0 = (a[0] - float(origin["x"])) / res, (a[1] - float(origin["y"])) / res
    x1, y1 = (b[0] - float(origin["x"])) / res, (b[1] - float(origin["y"])) / res
    dx, dy = x1 - x0, y1 - y0
    times = [0.0, 1.0]
    for start, end, delta in ((x0, x1, dx), (y0, y1, dy)):
        if not delta:
            continue
        for gridline in range(math.floor(min(start, end)) + 1, math.floor(max(start, end)) + 1):
            t = (gridline - start) / delta
            if 0 < t < 1:
                times.append(t)
    times.sort()

    def indices(value: float):
        nearest = round(value)
        if abs(value - nearest) <= 1e-9:
            return (nearest - 1, nearest)
        return (math.floor(value),)

    for i, t in enumerate(times):
        for sample in (t, (t + times[i + 1]) / 2.0) if i + 1 < len(times) else (t,):
            for ix in indices(x0 + dx * sample):
                for iy in indices(y0 + dy * sample):
                    yield ix, iy


def _line_is_clear(snapshot: dict, blocked: bytearray, a: tuple[float, float],
                    b: tuple[float, float]) -> bool:
    width, height = int(snapshot["width"]), int(snapshot["height"])
    return all(0 <= ix < width and 0 <= iy < height and not blocked[iy * width + ix]
               for ix, iy in _segment_cells(snapshot, a, b))


def _move_offsets(resolution: float, yaw: float, max_steer_deg: float) -> list[tuple[int, int, float]]:
    """Клеточные шаги, которые не требуют разворота корпуса.

    Набор лучей строится по самой occupancy-сетке (целые dx/dy), а не по
    диагонали 45°: 45° для прошивки Zapret слишком велико, поэтому его нельзя
    отправлять в рулевую команду.
    """
    max_cells = min(10, max(3, int(math.ceil(0.55 / resolution))))
    limit = math.radians(max_steer_deg)
    moves = []
    for dy in range(-max_cells, max_cells + 1):
        for dx in range(-max_cells, max_cells + 1):
            if not (dx or dy) or gcd(abs(dx), abs(dy)) != 1:
                continue
            distance = math.hypot(dx, dy) * resolution
            if distance > 0.60:
                continue
            angle = math.atan2(dy, dx)
            if steering_deviation(angle, yaw) <= limit:
                moves.append((dx, dy, distance))
    return moves


def plan_grid_path(snapshot: dict, goal_x: Any, goal_y: Any, *,
                   footprint_m: float = DEFAULT_FOOTPRINT_M,
                   max_steer_deg: float = PLAN_STEER_DEG,
                   max_segment_m: float = 1.0,
                   max_distance_m: float = MAX_PLAN_DISTANCE_M,
                   _blocked: Optional[bytearray] = None) -> dict:
    """A* по occupancy grid с ограниченными движениями шасси.

    UNKNOWN и промежуточная занятость не считаются свободными. Препятствия
    расширяются на footprint робота; пути с курсом за пределами ±28° от
    переднего/заднего направления не выдаются.
    """
    goal_x = _number(goal_x, "X")
    goal_y = _number(goal_y, "Y")
    width, height, resolution, values = _map_values(snapshot)
    pose = snapshot.get("pose") or {}
    start_x = _number(pose.get("x"), "поза X")
    start_y = _number(pose.get("y"), "поза Y")
    yaw = _number(pose.get("yaw"), "курс")
    origin = snapshot.get("origin") or {}
    if not isinstance(origin, dict) or "x" not in origin or "y" not in origin:
        raise NavigationError("map_invalid", "У карты нет начала координат", 503)
    if math.hypot(goal_x - start_x, goal_y - start_y) > max_distance_m:
        raise NavigationError("goal_too_far", "Цель дальше разрешённых %.0f м от робота" % max_distance_m, 400)

    start_cell = _cell(snapshot, start_x, start_y)
    goal_cell = _cell(snapshot, goal_x, goal_y)
    if start_cell is None or goal_cell is None:
        raise NavigationError("goal_outside_map", "Точка находится за границами карты", 400)
    sx, sy = start_cell
    gx, gy = goal_cell
    goal_index = gy * width + gx

    blocked = (_blocked if _blocked is not None
               else _inflated_blocked(snapshot, footprint_m, (start_x, start_y))[0])
    if (values[goal_index] == 255 or values[goal_index] > FREE_MAX) and blocked[goal_index]:
        raise NavigationError("goal_not_free", "Цель должна быть на изученной свободной клетке", 409)
    start_index = sy * width + sx
    if blocked[start_index]:
        raise NavigationError("start_blocked", "Робот слишком близко к препятствию для безопасного старта", 409)
    if blocked[goal_index]:
        raise NavigationError("goal_too_close", "Цель слишком близко к препятствию: оставьте запас вокруг корпуса", 409)

    start = (start_x, start_y)
    route_limit_m = max_distance_m * 1.5
    offsets = _move_offsets(resolution, yaw, max_steer_deg)
    if not offsets and start_index != goal_index:
        raise NavigationError("no_path", "Нет допустимого направления движения для этого курса", 409)

    # A*: линейная память по клеткам и ограниченный размер карты.
    count = width * height
    g_score = [math.inf] * count
    parent = [-1] * count
    closed = bytearray(count)
    g_score[start_index] = 0.0
    queue: list[tuple[float, float, int]] = []
    heuristic = lambda ix, iy: math.hypot(gx - ix, gy - iy) * resolution
    heapq.heappush(queue, (heuristic(sx, sy), 0.0, start_index))
    expansions = 0

    while queue:
        _f, cost, current = heapq.heappop(queue)
        if closed[current] or cost > g_score[current] + 1e-9:
            continue
        if current == goal_index:
            break
        closed[current] = 1
        expansions += 1
        if expansions > MAX_PLAN_EXPANSIONS:
            raise NavigationError("planner_limit", "Планировщик достиг лимита поиска; выберите цель ближе", 409)
        cx, cy = current % width, current // width
        current_world = start if current == start_index else _cell_center(snapshot, cx, cy)
        for dx, dy, edge_cost in offsets:
            nx, ny = cx + dx, cy + dy
            if nx < 0 or ny < 0 or nx >= width or ny >= height:
                continue
            neighbor = ny * width + nx
            if closed[neighbor] or blocked[neighbor]:
                continue
            neighbor_world = _cell_center(snapshot, nx, ny)
            # Не режем углы и не допускаем непроходимых диагоналей.
            if current == start_index:
                dx_world = neighbor_world[0] - start_x
                dy_world = neighbor_world[1] - start_y
                # Cell centres very close to the actual pose can create a
                # sub-tolerance first waypoint or an impossible steering angle.
                # The worker would repeatedly replan without moving (or brake).
                # Connect the real pose only to a meaningful, drivable segment.
                if math.hypot(dx_world, dy_world) <= DEFAULT_WAYPOINT_TOLERANCE_M + 0.02:
                    continue
                angle = math.atan2(dy_world, dx_world)
                if steering_deviation(angle, yaw) > math.radians(max_steer_deg):
                    continue
            if not _line_is_clear(snapshot, blocked, current_world, neighbor_world):
                continue
            tentative = cost + edge_cost
            if tentative > route_limit_m:
                continue
            if tentative + 1e-9 >= g_score[neighbor]:
                continue
            g_score[neighbor] = tentative
            parent[neighbor] = current
            estimate = heuristic(nx, ny)
            heapq.heappush(queue, (tentative + estimate, tentative, neighbor))

    if start_index != goal_index and parent[goal_index] < 0:
        raise NavigationError(
            "no_path",
            "Безопасного пути нет: проверьте свободную область, курс робота и ограничение руля ±28°",
            409,
        )

    cells = [goal_index]
    while cells[-1] != start_index:
        previous = parent[cells[-1]]
        if previous < 0:
            raise NavigationError("no_path", "Не удалось восстановить маршрут", 409)
        cells.append(previous)
    cells.reverse()

    raw_points = [start]
    for index in cells[1:]:
        raw_points.append(_cell_center(snapshot, index % width, index // width))
    # Поставленная цель остаётся точной, только если конечный отрезок не
    # нарушает steering-ограничение и не срезает inflation препятствий.
    if raw_points:
        candidate = (goal_x, goal_y)
        last = raw_points[-1]
        if len(raw_points) > 1:
            previous = raw_points[-2]
            segment_angle = math.atan2(candidate[1] - previous[1], candidate[0] - previous[0])
            if (math.hypot(candidate[0] - last[0], candidate[1] - last[1]) < resolution * 1.5
                    and steering_deviation(segment_angle, yaw) <= math.radians(max_steer_deg)
                    and _line_is_clear(snapshot, blocked, previous, candidate)):
                raw_points[-1] = candidate
        elif cells[-1] == start_index:
            # Одна клетка: конечная точка обычно в пределах нескольких см.
            if (steering_deviation(math.atan2(goal_y - start_y, goal_x - start_x), yaw)
                    <= math.radians(max_steer_deg)
                    and _line_is_clear(snapshot, blocked, start, candidate)):
                raw_points.append(candidate)

    # Упрощаем ломаную по line-of-sight, сохраняя предел направления и
    # ограничивая длину одного отрезка, чтобы скан/поза регулярно проверялись.
    simplified = [raw_points[0]]
    index = 0
    while index < len(raw_points) - 1:
        chosen = None
        for candidate_index in range(len(raw_points) - 1, index, -1):
            a = raw_points[index]
            b = raw_points[candidate_index]
            distance = math.hypot(b[0] - a[0], b[1] - a[1])
            if distance > max_segment_m:
                continue
            if distance > 1e-6:
                angle = math.atan2(b[1] - a[1], b[0] - a[0])
                if steering_deviation(angle, yaw) > math.radians(max_steer_deg):
                    continue
            if _line_is_clear(snapshot, blocked, a, b):
                chosen = candidate_index
                break
        if chosen is None:
            raise NavigationError("no_path", "Не найден безопасный участок с допустимым углом руления", 409)
        simplified.append(raw_points[chosen])
        index = chosen

    # Не оставляем в конце короткий «хвост» (< 0.12 м) после жадного разбиения:
    # такой микросегмент иначе блокирует старт проверкой минимальной длины участка.
    if len(simplified) >= 3:
        tail_len = math.hypot(simplified[-1][0] - simplified[-2][0],
                              simplified[-1][1] - simplified[-2][1])
        if tail_len < 0.12:
            prev_a = simplified[-3]
            last_b = simplified[-1]
            merged_len = math.hypot(last_b[0] - prev_a[0], last_b[1] - prev_a[1])
            merged_angle = math.atan2(last_b[1] - prev_a[1], last_b[0] - prev_a[0])
            if (merged_len <= max_segment_m * 1.25
                    and steering_deviation(merged_angle, yaw) <= math.radians(max_steer_deg)
                    and _line_is_clear(snapshot, blocked, prev_a, last_b)):
                simplified[-2:] = [last_b]

    length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                 for a, b in zip(simplified, simplified[1:]))
    if length > route_limit_m:
        raise NavigationError("route_too_long", "Найденный обход слишком длинный; выберите цель ближе", 409)
    path = [{"x": round(x, 6), "y": round(y, 6)} for x, y in simplified]
    # Rounding a waypoint onto the other side of a grid line can turn a
    # previously clear segment into a corner cut. Fall back to exact values.
    if any(not _line_is_clear(snapshot, blocked, (a["x"], a["y"]), (b["x"], b["y"]))
           or steering_deviation(math.atan2(b["y"] - a["y"], b["x"] - a["x"]), yaw)
           > math.radians(max_steer_deg) + 1e-12
           for a, b in zip(path, path[1:])):
        path = [{"x": x, "y": y} for x, y in simplified]
    return {
        "ok": True,
        "path": path,
        "goal": {"x": path[-1]["x"], "y": path[-1]["y"]},
        "distanceM": round(length, 2),
        "waypoints": max(0, len(path) - 1),
        "mapRevision": int(snapshot.get("revision", 0)),
        "steerLimitDeg": max_steer_deg,
        "footprintM": round(footprint_m, 2),
        "robotDimensions": robot_dimensions_payload(footprint_m),
    }


def _route_goals(body: dict, x: float, y: float) -> list[dict]:
    points = body.get("goals", [{"x": x, "y": y}])
    if not isinstance(points, list) or not 1 <= len(points) <= 20:
        raise NavigationError("invalid_goals", "Укажите от 1 до 20 точек маршрута", 400)
    result = []
    for point in points:
        if not isinstance(point, dict):
            raise NavigationError("invalid_goals", "Точка должна содержать X и Y", 400)
        result.append({"x": _number(point.get("x"), "X"), "y": _number(point.get("y"), "Y")})
    if abs(result[-1]["x"] - x) > 0.01 or abs(result[-1]["y"] - y) > 0.01:
        raise NavigationError("invalid_goals", "Последняя точка должна совпадать с целью", 400)
    return result


# Держим подтверждённый участок дальше границы неизвестного, чем допуск
# остановки у точки. Дальше этого участка робот не получает команду движения.
FRONTIER_BACKOFF_M = 0.25
MIN_FRONTIER_PROGRESS_M = 0.30


def _frontier_on_guide(snapshot: dict, guide: list[dict], clearance: _ClearanceGrid) -> Optional[tuple[float, float]]:
    """Найти конец *подтверждённого* префикса предположительного пути.

    Оптимистическая карта нужна только для выбора направления. Ни один её
    участок за границей фактически свободного габарита не исполняется.
    """
    resolution = float(snapshot["resolution"])
    samples: list[tuple[float, tuple[float, float]]] = []
    total = 0.0
    blocked_seen = False
    for a, b in zip(guide, guide[1:]):
        ax, ay = float(a["x"]), float(a["y"])
        bx, by = float(b["x"]), float(b["y"])
        segment = math.hypot(bx - ax, by - ay)
        count = max(1, int(math.ceil(segment / max(0.025, resolution * 0.45))))
        for n in range(count + 1):
            t = n / count
            point = (ax + (bx - ax) * t, ay + (by - ay) * t)
            cell = _cell(snapshot, *point)
            if cell is None or not clearance.clear_cell(*cell):
                blocked_seen = True
                break
            samples.append((total + segment * t, point))
        total += segment
        if blocked_seen:
            break
    if not samples:
        return None
    # При отсутствии границы прямой безопасный маршрут должен был быть найден
    # раньше. Не выдаём предположительный путь как проверенный при ошибке A*.
    if not blocked_seen:
        return None
    usable_distance = samples[-1][0] - FRONTIER_BACKOFF_M
    if usable_distance < MIN_FRONTIER_PROGRESS_M:
        return None
    return next((point for distance, point in reversed(samples)
                 if MIN_FRONTIER_PROGRESS_M <= distance <= usable_distance), None)


def _initial_blind_zone(snapshot: dict) -> dict:
    pose = snapshot.get("pose") or {}
    x = _number(pose.get("x"), "поза X")
    y = _number(pose.get("y"), "поза Y")
    yaw = _number(pose.get("yaw"), "курс")
    offset = _number(snapshot.get("lidarOffsetM", DEFAULT_LIDAR_OFFSET_M), "смещение лидара")
    radius = _number((snapshot.get("settings") or {}).get("min_range", 0.3), "ближняя граница лидара")
    if not 0 <= offset <= 1.5 or not 0 <= radius <= 2.0:
        raise NavigationError("map_invalid", "Некорректная геометрия лидара", 503)
    return {"x": x + offset * math.cos(yaw), "y": y + offset * math.sin(yaw), "radiusM": radius}


def _blind_zone_moved(snapshot: dict, confirmed: Optional[dict]) -> bool:
    """A physical inspection is tied to the original sensor location."""
    if not isinstance(confirmed, dict):
        return True
    current = _initial_blind_zone(snapshot)
    try:
        return (math.hypot(current["x"] - float(confirmed["x"]),
                           current["y"] - float(confirmed["y"])) > 0.03
                or abs(current["radiusM"] - float(confirmed["radiusM"])) > 1e-9)
    except (KeyError, TypeError, ValueError):
        return True


def plan_route(snapshot: dict, goals: list[dict], *, footprint_m: float,
               blind_zone: Optional[dict] = None) -> dict:
    """Маршрут по известному + осторожный префикс к неисследованной цели.

    Для направления поиска допускается гипотеза, что UNKNOWN свободна, но для
    исполнения строится отдельный путь по известному пространству. Единственное
    исключение — вручную проверенная ближняя слепая зона на месте старта.
    Никаких пропусков обязательных промежуточных точек.
    """
    if not goals:
        raise NavigationError("invalid_goals", "Нужна хотя бы одна точка", 400)
    width, height, _resolution, values = _map_values(snapshot)
    del width, height
    # The only UNKNOWN exception is the sensor's initial near blind spot,
    # explicitly checked by the operator, anchored in world coordinates.
    # It is never shifted forward during replanning.
    blind_zone = dict(blind_zone or _initial_blind_zone(snapshot))
    clearance = _ClearanceGrid(snapshot, footprint_m, blind_zone)
    safe_mask = clearance.blocked_mask()
    has_unknown = 255 in values
    optimistic = None
    optimistic_mask = None
    current = dict(snapshot)
    pose = snapshot.get("pose") or {}
    path: list[dict] = []
    distance = 0.0
    partial = False
    partial_index = None
    frontier_blocked = False
    leg: Optional[dict] = None

    for index, goal in enumerate(goals):
        try:
            try:
                leg = plan_grid_path(current, goal["x"], goal["y"], footprint_m=footprint_m,
                                     _blocked=safe_mask)
            except NavigationError as direct_error:
                goal_x, goal_y = _number(goal["x"], "X"), _number(goal["y"], "Y")
                goal_cell = _cell(snapshot, goal_x, goal_y)
                goal_value = (values[goal_cell[1] * int(snapshot["width"]) + goal_cell[0]]
                              if goal_cell else None)
                can_explore = (has_unknown and goal_value is not None
                               and (goal_value == 255 or goal_value <= FREE_MAX)
                               and direct_error.code in {"goal_not_free", "goal_too_close", "no_path", "start_blocked"})
                if not can_explore:
                    raise
                if optimistic is None:
                    optimistic = dict(snapshot)
                    optimistic["data"] = bytes(0 if v == 255 else v for v in values)
                    optimistic_mask = _ClearanceGrid(optimistic, footprint_m).blocked_mask()
                optimistic_current = dict(optimistic)
                optimistic_current["pose"] = current["pose"]
                guide = plan_grid_path(optimistic_current, goal_x, goal_y,
                                       footprint_m=footprint_m, _blocked=optimistic_mask)
                frontier = _frontier_on_guide(snapshot, guide["path"], clearance)
                partial = True
                partial_index = index
                if frontier is None:
                    frontier_blocked = True
                    start = current["pose"]
                    leg = {"ok": True, "path": [{"x": round(float(start["x"]), 3),
                                                  "y": round(float(start["y"]), 3)}],
                           "distanceM": 0.0, "waypoints": 0,
                           "mapRevision": int(snapshot.get("revision", 0)),
                           "steerLimitDeg": PLAN_STEER_DEG,
                           "footprintM": round(footprint_m, 2),
                           "robotDimensions": robot_dimensions_payload(footprint_m)}
                    leg["goal"] = dict(leg["path"][-1])
                else:
                    # Вторая, независимая проверка тем же безопасным A*:
                    # гипотетический маршрут никогда не отправляется моторам.
                    leg = plan_grid_path(current, frontier[0], frontier[1],
                                         footprint_m=footprint_m, _blocked=safe_mask)
        except NavigationError as exc:
            raise NavigationError(exc.code, "Точка %d: %s" % (index + 1, exc.message), exc.status) from exc
        path.extend(leg["path"] if not path else leg["path"][1:])
        distance += leg["distanceM"]
        if distance > MAX_PLAN_DISTANCE_M * 1.5:
            raise NavigationError("route_too_long", "Суммарный путь длиннее разрешённых %.0f м" % (MAX_PLAN_DISTANCE_M * 1.5), 409)
        current["pose"] = {**pose, **leg["goal"]}
        if partial:
            break
    assert leg is not None
    # Include whole grid cells touching the blind circle and the small pose
    # tolerance between preview and start in the physical inspection radius.
    inspection_radius = math.ceil((blind_zone["radiusM"] + _resolution * math.sqrt(2) + 0.03) * 100) / 100
    leg.update(path=path, distanceM=round(distance, 2), waypoints=max(0, len(path) - 1),
               goalsRequested=[dict(g) for g in goals], partial=partial,
               blindZone=blind_zone, blindInspectionRadiusM=inspection_radius,
               manualBlindZone=clearance.ignored_blind_cells > 0,
               partialGoalIndex=partial_index, frontierBlocked=frontier_blocked,
               executable=len(path) >= 2 and distance >= 0.10)
    return leg

def _plans_equivalent(preview: Optional[dict], current: dict,
                     waypoint_tolerance_m: float = 0.12) -> bool:
    """Require a fresh operator preview when replanning changes the route."""
    if not isinstance(preview, dict):
        return False
    if (bool(preview.get("partial")) != bool(current.get("partial"))
            or bool(preview.get("manualBlindZone")) != bool(current.get("manualBlindZone"))
            or (bool(preview.get("manualBlindZone"))
                and preview.get("blindInspectionRadiusM") != current.get("blindInspectionRadiusM"))
            or preview.get("blindZone") != current.get("blindZone")
            or preview.get("partialGoalIndex") != current.get("partialGoalIndex")
            or preview.get("goalsRequested") != current.get("goalsRequested")):
        return False
    requested = preview.get("goalRequested") or {}
    current_requested = current.get("goalRequested") or {}
    try:
        if (abs(float(requested["x"]) - float(current_requested["x"])) > 0.01
                or abs(float(requested["y"]) - float(current_requested["y"])) > 0.01):
            return False
        old_path = preview["path"]
        new_path = current["path"]
    except (KeyError, TypeError, ValueError):
        return False
    if not isinstance(old_path, list) or not isinstance(new_path, list) or len(old_path) != len(new_path):
        return False
    # A small live-pose change is expected between preview and click; a larger
    # localization jump requires a new human review even if the goal is unchanged.
    try:
        if math.hypot(float(old_path[0]["x"]) - float(new_path[0]["x"]),
                      float(old_path[0]["y"]) - float(new_path[0]["y"])) > max(0.20, waypoint_tolerance_m * 1.5):
            return False
    except (KeyError, TypeError, ValueError, IndexError):
        return False
    # Compare every waypoint the operator reviewed, including the destination.
    for old, new in zip(old_path[1:], new_path[1:]):
        try:
            if math.hypot(float(old["x"]) - float(new["x"]),
                         float(old["y"]) - float(new["y"])) > waypoint_tolerance_m:
                return False
        except (KeyError, TypeError, ValueError):
            return False
    return True


def _obstacle_ahead(points: list[dict], body_angle: float,
                    lookahead_m: float, half_width_m: float = 0.32,
                    lidar_offset_m: float = 0.0) -> Optional[float]:
    """Динамическая остановка: скан впереди по текущему вектору движения."""
    ux, uy = math.cos(body_angle), math.sin(body_angle)
    nearest = None
    for point in points:
        try:
            angle = float(point["a"])
            distance = float(point["r"])
        except (KeyError, TypeError, ValueError):
            continue
        if not math.isfinite(angle) or not math.isfinite(distance) or distance <= 0:
            continue
        # Точки приходят относительно переднего лидара, не центра корпуса.
        # Особенно важно для реверса: хвост дальше от датчика на 35 см.
        px, py = lidar_offset_m + distance * math.cos(angle), distance * math.sin(angle)
        along = px * ux + py * uy
        cross = abs(px * uy - py * ux)
        if 0.0 <= along <= lookahead_m and cross <= half_width_m:
            nearest = along if nearest is None else min(nearest, along)
    return nearest


MIN_NAV_SCAN_POINTS = 60
MAX_NAV_SCAN_GAP_RAD = math.radians(30.0)


def _scan_coverage(points: Optional[list[dict]]) -> tuple[bool, int, Optional[float]]:
    """Нельзя ехать с обрывком скана или отсутствующим сектором обзора.

    Плотный скан не подтверждает отсутствие низких, прозрачных или невидимых
    лидару препятствий; физический контроль остаётся обязательным.
    """
    angles = []
    for point in points or []:
        try:
            angle, distance = float(point["a"]), float(point["r"])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(angle) and math.isfinite(distance) and distance > 0:
            angles.append(angle % math.tau)
    if not angles:
        return False, 0, None
    angles.sort()
    gaps = [b - a for a, b in zip(angles, angles[1:])]
    gaps.append(angles[0] + math.tau - angles[-1])
    gap = max(gaps)
    return len(angles) >= MIN_NAV_SCAN_POINTS and gap <= MAX_NAV_SCAN_GAP_RAD, len(angles), math.degrees(gap)


class NavigationService:
    """Серверный планировщик и опциональный ограниченный исполнитель пути."""

    def __init__(self, map_service, source, lidar_manager=None, *,
                 enabled: Optional[bool] = None, mps_per_percent: Optional[float] = None,
                 speed_mps: Optional[float] = None, max_drive_percent: Optional[int] = None,
                 clock=time.time):
        self.map_service = map_service
        self.source = source
        self.lidar_manager = lidar_manager
        self.enabled = _env_bool("RC_NAV_ENABLE", False) if enabled is None else bool(enabled)
        self.mps_per_percent = (
            _env_optional_positive("RC_NAV_MPS_PER_PERCENT")
            if mps_per_percent is None else mps_per_percent
        )
        self.speed_mps = (speed_mps if speed_mps is not None else
                          _env_float("RC_NAV_SPEED_MPS", DEFAULT_SPEED_MPS, 0.02, 0.10))
        if max_drive_percent is None:
            try:
                max_drive_percent = int(os.environ.get("RC_NAV_MAX_PERCENT", DEFAULT_MAX_PERCENT))
            except (TypeError, ValueError):
                max_drive_percent = DEFAULT_MAX_PERCENT
        self.max_drive_percent = max(1, min(DEFAULT_MAX_PERCENT, int(max_drive_percent)))
        self.footprint_m = _env_float("RC_NAV_FOOTPRINT_M", DEFAULT_FOOTPRINT_M, 0.30, 2.0)
        self.robot_length_m = _env_float("RC_ROBOT_LENGTH_M", DEFAULT_ROBOT_LENGTH_M, 0.30, 2.5)
        self.robot_width_m = _env_float("RC_ROBOT_WIDTH_M", DEFAULT_ROBOT_WIDTH_M, 0.25, 2.0)
        self.robot_height_m = _env_float("RC_ROBOT_HEIGHT_M", DEFAULT_ROBOT_HEIGHT_M, 0.10, 2.0)
        self.lidar_offset_m = lidar_mount_offset()
        self.quality_min = _env_float("RC_NAV_QUALITY_MIN", DEFAULT_QUALITY_MIN, 0.30, 0.95)
        self.scan_age_s = _env_float("RC_NAV_SCAN_AGE_S", DEFAULT_SCAN_AGE_S, 0.3, 2.0)
        self.clock = clock
        self._lock = threading.RLock()
        # Сериализуем heartbeat автономии и команду STOP: после возврата из
        # cancel() поток навигации уже не сможет отправить ещё один импульс.
        self._motion_lock = threading.RLock()
        # Старт автономии и ручная команда не могут одновременно захватить
        # тягу после одной и той же проверки «моторы стоят».
        self._operator_motion_lock = threading.RLock()
        self._cancel = threading.Event()
        self._worker: Optional[threading.Thread] = None
        self._bridge: Optional[MasterBridge] = None
        self._plan: Optional[dict] = None
        self._goal: Optional[dict] = None
        self._state = {
            "phase": "idle", "message": "Цель не задана", "progress": 0.0,
            "waypoint": 0, "totalWaypoints": 0, "updatedAt": self.clock(),
        }

    @property
    def active(self) -> bool:
        with self._lock:
            return bool(self._worker and self._worker.is_alive())

    def _snapshot(self) -> dict:
        return self.map_service.navigation_snapshot()

    def _plan_route(self, snapshot: dict, goals: list[dict], *, blind_zone: Optional[dict] = None) -> dict:
        plan = plan_route(snapshot, goals, footprint_m=self.footprint_m, blind_zone=blind_zone)
        plan["robotDimensions"] = robot_dimensions_payload(
            self.footprint_m, length_m=self.robot_length_m, width_m=self.robot_width_m,
            height_m=self.robot_height_m, lidar_offset_m=self.lidar_offset_m)
        return plan

    def _scan(self) -> Optional[list[dict]]:
        if self.lidar_manager is None:
            return None
        getter = getattr(self.lidar_manager, "get_scan_info", None)
        if not callable(getter):
            return None
        try:
            info = getter(max_age=self.scan_age_s)
            return info.get("points") if info.get("trusted") is True else None
        except Exception:
            return None

    def _preflight(self, snapshot: Optional[dict] = None, *,
                   allow_alignment: bool = False) -> tuple[bool, str, dict]:
        snapshot = snapshot or self._snapshot()
        now = self.clock()
        age = now - float(snapshot.get("updatedAt") or 0.0)
        map_ready = snapshot.get("scans", 0) >= 2
        scan = self._scan()
        scan_ready, scan_points, scan_gap = _scan_coverage(scan)
        quality = float(snapshot.get("quality") or 0.0)
        map_settings = snapshot.get("settings") or {}
        matching_enabled = map_settings.get("matching") is True
        decay = map_settings.get("decay")
        decay_disabled = (isinstance(decay, (int, float)) and not isinstance(decay, bool)
                          and math.isfinite(decay) and decay == 0)
        pose_ready = math.isfinite(quality) and quality >= self.quality_min and matching_enabled
        minimum_footprint = math.hypot(self.robot_length_m, self.robot_width_m) / 2.0
        footprint_usable = self.footprint_m >= minimum_footprint
        drive_status = {}
        drive_status_fn = getattr(self.source, "drive_status", None)
        if callable(drive_status_fn):
            try:
                drive_status = drive_status_fn()
            except Exception as exc:  # noqa: BLE001 — API показывает готовность
                drive_status = {"canDrive": False, "reason": str(exc)[:120]}
        drive_reason = str(drive_status.get("reason") or "")
        drive_usable = bool(drive_status.get("canDrive"))
        if allow_alignment and drive_reason in {"module_busy", "steering_misaligned"}:
            # Во время законной остановки/перекладки руля привод временно
            # сообщает «занят» (или, на старом сервере, «углы не совпадают») —
            # мост ждёт, не едет. Угол колёс сам по себе движению не мешает:
            # ехать можно на любом, включая краб и боковой ход.
            drive_usable = bool(drive_status.get("available"))
        commanded = drive_status.get("commandedPercent")
        reported_active = bool(drive_status.get("active"))
        if commanded is not None:
            try:
                reported_active = reported_active or abs(float(commanded)) > 0.01
            except (TypeError, ValueError):
                reported_active = True
        drive_stopped = not reported_active
        if allow_alignment:
            with self._lock:
                bridge = self._bridge
            if bridge is not None:
                try:
                    bridge_percent = float(bridge.percent)
                    command_matches = (commanded is None
                                       or abs(float(commanded) - bridge_percent) < 0.01)
                    if command_matches and (not reported_active or commanded is not None):
                        # Includes the brief telemetry lag after our own STOP.
                        drive_stopped = True
                except (TypeError, ValueError):
                    pass
        hardware_ready = (
            getattr(self.source, "name", "") == "master"
            and bool(getattr(self.source, "supports_drive", False))
            and bool(getattr(self.source, "supports_individual_steer", False))
            and callable(getattr(self.source, "brake_for_keyboard", None))
            and drive_usable
            and drive_stopped
        )
        speed_percent = (
            int(round(self.speed_mps / self.mps_per_percent))
            if self.mps_per_percent and self.mps_per_percent > 0 else 0
        )
        speed_usable = 1 <= speed_percent <= self.max_drive_percent
        checks = {
            "enabled": self.enabled,
            "speedCalibrated": bool(self.mps_per_percent and self.mps_per_percent > 0),
            "speedWithinLimit": speed_usable,
            "masterReady": hardware_ready,
            "driveStopped": drive_stopped,
            "mapFresh": age >= 0 and age <= self.scan_age_s * 1.5,
            "mapReady": map_ready,
            "mapTrusted": snapshot.get("trusted") is True,
            "mapIntegrity": snapshot.get("navigationIntegrity") is True,
            "scanReady": scan_ready,
            "poseReady": pose_ready,
            "mapDecayDisabled": decay_disabled,
            "footprintSufficient": footprint_usable,
            "estopClear": estop_latch.latched() is None,
        }
        reason = ""
        if not checks["estopClear"]:
            reason = "Взведён E-STOP — сначала проверьте площадку и снимите защёлку."
        elif not checks["enabled"]:
            reason = "Автономная тяга выключена. Для включения задайте RC_NAV_ENABLE=1 после проверки системы."
        elif not checks["speedCalibrated"]:
            reason = "Нет калибровки скорости. Укажите измеренное RC_NAV_MPS_PER_PERCENT; значение не угадывается."
        elif not checks["speedWithinLimit"]:
            reason = "Скорость не укладывается в лимит тяги %d%%; скорректируйте RC_NAV_SPEED_MPS или калибровку." % self.max_drive_percent
        elif not hardware_ready:
            drive_reason = drive_status.get("reason")
            if not drive_stopped:
                reason = "Обнаружена активная тяга вне этой навигации; сначала остановите привод."
            elif getattr(self.source, "name", "") != "master":
                reason = "Для движения нужен прямой источник RC_SLAM_SOURCE=master на борту (8090)."
            elif drive_reason:
                reason = "Шасси не готово: %s" % drive_reason.replace("_", " ")
            else:
                reason = "Текущий источник не поддерживает безопасную тягу и адресное руление."
        elif not checks["footprintSufficient"]:
            reason = "Защитный радиус %.2f м меньше половины диагонали корпуса %.2f м; увеличьте RC_NAV_FOOTPRINT_M с запасом." % (self.footprint_m, minimum_footprint)
        elif not checks["mapDecayDisabled"]:
            reason = "В карте включено забывание препятствий (decay); для движения установите decay=0 и постройте карту заново."
        elif not checks["scanReady"]:
            reason = "Нет свежего полного скана лидара: нужны не менее %d точек и обзор без больших пропусков." % MIN_NAV_SCAN_POINTS
        elif not checks["mapReady"]:
            reason = "Нужно не менее двух сканов для первичной локализации."
        elif not checks["mapTrusted"]:
            reason = "В карту попал неподтверждённый API-скан: сбросьте карту и постройте её по физическому лидару."
        elif not checks["mapIntegrity"]:
            reason = "Карта строилась без сопоставления сканов или с забыванием препятствий; сбросьте её и соберите заново."
        elif not checks["mapFresh"]:
            reason = "Карта устарела — дождитесь свежего скана лидара."
        elif not checks["poseReady"]:
            reason = "Низкое качество сопоставления сканов (%.0f%%); проверьте локализацию." % (quality * 100)
        return not reason, reason, {
            "checks": checks,
            "drive": drive_status,
            "mapAgeS": round(max(0.0, age), 2) if math.isfinite(age) else None,
            "poseQuality": round(quality, 3),
            "scanPoints": scan_points,
            "scanMaxGapDeg": round(scan_gap, 1) if scan_gap is not None else None,
            "minimumFootprintM": round(minimum_footprint, 3),
        }

    def status(self) -> dict:
        with self._lock:
            state = dict(self._state)
            plan = self._plan
            goal = self._goal
        ready = False
        try:
            snapshot = self._snapshot()
            ready, reason, diagnostics = self._preflight(snapshot)
        except Exception as exc:  # noqa: BLE001 — состояние не должно ронять экран
            reason = "Карта навигации недоступна: %s" % str(exc)[:160]
            diagnostics = {"checks": {}, "drive": {}, "mapAgeS": None, "poseQuality": 0.0}
        if ready and plan and not plan.get("executable", True):
            reason = "Нет подтверждённого свободного участка к цели; дождитесь новых сканов или измените цель."
        state.update({
            "ok": True,
            "enabled": self.enabled,
            "available": bool(diagnostics.get("checks", {}).get("masterReady")),
            "canStart": bool(ready and plan and plan.get("executable", True) and not self.active),
            "reason": reason,
            "diagnostics": diagnostics,
            "goal": dict(goal) if goal else None,
            "plan": plan,
            "robotDimensions": robot_dimensions_payload(
                self.footprint_m,
                length_m=self.robot_length_m,
                width_m=self.robot_width_m,
                height_m=self.robot_height_m,
                lidar_offset_m=self.lidar_offset_m,
            ),
        })
        return state

    def plan(self, body: Any) -> dict:
        if not isinstance(body, dict):
            raise NavigationError("invalid_goal", "Ожидается JSON-объект с целью", 400)
        goal = body.get("goal", body)
        if not isinstance(goal, dict):
            raise NavigationError("invalid_goal", "Цель должна содержать координаты X и Y", 400)
        x = _number(goal.get("x"), "X")
        y = _number(goal.get("y"), "Y")
        goals = _route_goals(body, x, y)
        if self.active:
            raise NavigationError("navigation_active", "Сначала остановите текущую поездку", 409)
        try:
            snapshot = self._snapshot()
            plan = self._plan_route(snapshot, goals)
        except NavigationError as exc:
            with self._lock:
                if not self.active:
                    self._plan = None
                    self._goal = {"x": round(x, 3), "y": round(y, 3)}
                    self._state = {
                        "phase": "blocked", "message": exc.message, "progress": 0.0,
                        "waypoint": 0, "totalWaypoints": 0, "updatedAt": self.clock(),
                    }
            raise
        plan["goalRequested"] = {"x": round(x, 3), "y": round(y, 3)}
        with self._lock:
            if self.active:
                raise NavigationError("navigation_active", "Сначала остановите текущую поездку", 409)
            self._plan = plan
            self._goal = dict(plan["goalRequested"])
            self._state = {
                "phase": "planned",
                "message": ("Показан только известный участок; дальше робот будет сканировать и перепланировать."
                            if plan.get("partial") else
                            "Маршрут рассчитан; перед стартом карта и готовность будут проверены заново."),
                "progress": 0.0,
                "waypoint": 0,
                "totalWaypoints": plan["waypoints"],
                "updatedAt": self.clock(),
            }
        return {"ok": True, "plan": plan, "status": self.status()}

    def start(self, body: Any) -> tuple[dict, int]:
        if not isinstance(body, dict):
            raise NavigationError("invalid_goal", "Ожидается JSON-объект с целью", 400)
        goal = body.get("goal", body)
        if not isinstance(goal, dict):
            raise NavigationError("invalid_goal", "Цель должна содержать координаты X и Y", 400)
        x = _number(goal.get("x"), "X")
        y = _number(goal.get("y"), "Y")
        if self.active:
            raise NavigationError("navigation_active", "Навигация уже выполняется", 409)
        snapshot = self._snapshot()
        ready, reason, _details = self._preflight(snapshot)
        if not ready:
            raise NavigationError("not_ready", reason, 409)
        with self._lock:
            preview = self._plan
        if not preview:
            raise NavigationError("plan_required", "Сначала постройте маршрут и проверьте его на карте", 409)
        goals = _route_goals(body, x, y)
        if goals != preview.get("goalsRequested", goals):
            raise NavigationError("plan_mismatch", "Точки изменились; постройте маршрут заново", 409)
        if preview.get("manualBlindZone") and body.get("blindZoneChecked") is not True:
            raise NavigationError("blind_zone_confirmation_required", "Осмотрите ближнюю слепую зону лидара и подтвердите это перед стартом", 409)
        preview_goal = preview.get("goalRequested") or {}
        if (abs(float(preview_goal.get("x", math.inf)) - x) > 0.01
                or abs(float(preview_goal.get("y", math.inf)) - y) > 0.01):
            raise NavigationError("plan_mismatch", "Цель отличается от проверенного маршрута; постройте новый план", 409)
        blind_moved = (bool(preview.get("manualBlindZone"))
                       and _blind_zone_moved(snapshot, preview.get("blindZone")))
        blind_zone = _initial_blind_zone(snapshot) if blind_moved else preview.get("blindZone")
        try:
            plan = self._plan_route(snapshot, goals, blind_zone=blind_zone)
        except NavigationError as exc:
            with self._lock:
                if self._plan is preview:
                    self._plan = None
                    self._goal = None
                    self._state = {
                        "phase": "blocked", "message": exc.message, "progress": 0.0,
                        "waypoint": 0, "totalWaypoints": 0, "updatedAt": self.clock(),
                    }
            raise
        plan["goalRequested"] = {"x": round(x, 3), "y": round(y, 3)}
        if blind_moved or not _plans_equivalent(preview, plan):
            with self._lock:
                self._plan = plan
                self._goal = dict(plan["goalRequested"])
                self._state = {
                    "phase": "planned",
                    "message": "Карта или поза изменились: обновлённую траекторию нужно проверить ещё раз.",
                    "progress": 0.0,
                    "waypoint": 0,
                    "totalWaypoints": plan["waypoints"],
                    "updatedAt": self.clock(),
                }
            raise NavigationError(
                "plan_changed",
                "Карта или поза изменились; новый маршрут показан на карте. Проверьте его и нажмите «Поехать» ещё раз.",
                409,
                details={"plan": plan, "status": self.status()},
            )
        path = plan.get("path") or []
        segment_lengths = [math.hypot(b["x"] - a["x"], b["y"] - a["y"])
                           for a, b in zip(path, path[1:])]
        if (len(path) < 2 or plan.get("distanceM", 0.0) < 0.10
                or any(length < 0.10 for length in segment_lengths)):
            raise NavigationError("goal_too_close",
                                  "Цель слишком близко или маршрут содержит участок короче 0,10 м",
                                  409)
        if not hasattr(self.source, "drive_status"):
            raise NavigationError("drive_unavailable", "Нет подтверждения готовности тяги", 409)
        with self._operator_motion_lock:
            # За время построения маршрута мог начаться ручной ввод.
            ready, reason, _details = self._preflight(self._snapshot())
            if not ready:
                raise NavigationError("not_ready", reason, 409)
            with self._lock:
                if self.active:
                    raise NavigationError("navigation_active", "Навигация уже выполняется", 409)
                if self._plan is not preview:
                    raise NavigationError("plan_changed", "План был изменён параллельно; проверьте текущую траекторию ещё раз", 409)
                self._cancel = threading.Event()
                self._plan = plan
                self._goal = dict(plan["goalRequested"])
                self._state = {
                    "phase": "starting",
                    "message": "Запуск: повторная проверка скана, локализации и шасси.",
                    "progress": 0.0,
                    "waypoint": 0,
                    "totalWaypoints": plan["waypoints"],
                    "updatedAt": self.clock(),
                }
                worker_goal = dict(plan["goalRequested"])
                worker_goal["requested"] = dict(plan["goalRequested"])
                worker = threading.Thread(
                    target=self._run, args=(worker_goal, self._cancel, plan),
                    name="robot-navigation", daemon=True,
                )
                self._worker = worker
                worker.start()
        return {"ok": True, "status": self.status()}, 202

    def manual_override(self, send, *, wait_for_exit: bool = True):
        """Отдать тягу оператору после завершения автономного STOP.

        В обычном ручном режиме не опрашивает карту/лидар. Если старый worker
        ещё не завершился, ненулевая команда не идёт вслед за его будущим
        тормозом; оператор может повторить её после остановки навигации.
        """
        with self._operator_motion_lock:
            with self._lock:
                worker = self._worker
            if worker and worker.is_alive():
                self.cancel("Ручная тяга — автономный маршрут остановлен.")
                if wait_for_exit and worker is not threading.current_thread():
                    worker.join(timeout=0.35)
                    if worker.is_alive():
                        return None
            return send()

    def cancel(self, reason: str = "Отменено оператором") -> dict:
        with self._lock:
            worker = self._worker
            active = bool(worker and worker.is_alive()
                          and self._state.get("phase") in {"starting", "executing", "stopping"})
            if active:
                self._cancel.set()
                self._state.update({"phase": "stopping", "message": reason,
                                    "updatedAt": self.clock()})
        # Аппаратный STOP идёт сразу, не ожидая следующего тика 5 Hz. Общий
        # motion-lock не даёт worker-у отправить heartbeat после этой команды.
        if active:
            with self._motion_lock:
                bridge = self._bridge
                try:
                    self._brake(bridge, "navigation_cancel")
                except Exception as exc:  # noqa: BLE001 — watchdog всё равно остаётся
                    log.error("navigation stop: %s", exc)
        return {"ok": True, "active": active, "status": self.status()}

    def clear_plan(self) -> None:
        with self._lock:
            if self.active:
                self._cancel.set()
            self._plan = None
            self._goal = None
            if not self.active:
                self._state = {
                    "phase": "idle", "message": "Цель не задана", "progress": 0.0,
                    "waypoint": 0, "totalWaypoints": 0, "updatedAt": self.clock(),
                }

    def _set_state(self, **fields: Any) -> None:
        with self._lock:
            self._state.update(fields)
            self._state["updatedAt"] = self.clock()

    def _brake(self, bridge: Optional[MasterBridge], reason: str) -> dict:
        with self._motion_lock:
            if bridge is not None:
                bridge.brake(reason)
            # STOP also interrupts a partially completed steering move. The
            # tagged response is required before the next autonomous segment.
            barrier = getattr(self.source, "brake_for_keyboard", None)
            if callable(barrier):
                return barrier()
            if getattr(self.source, "supports_drive", False):
                return self.source.drive(0)
            return {"ok": False, "reason": "brake_unavailable"}

    def _await_stopped(self, bridge: MasterBridge, cancel: threading.Event,
                       reason: str) -> None:
        """Brake and require a *new* matching ACK plus stopped SC on all boards."""
        with self._motion_lock:
            if cancel.is_set():
                raise NavigationError("cancelled", "Маршрут отменён оператором", 409)
            result = self._brake(bridge, reason)
        brake_id = result.get("brakeId") if isinstance(result, dict) else None
        if not result or not result.get("ok") or type(brake_id) is not int or brake_id <= 0:
            raise NavigationError("brake_unconfirmed", "Не удалось отправить подтверждаемый STOP четырём колёсам", 409)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            self._fresh_snapshot(cancel)
            lock = getattr(self.source, "lock", None)
            if lock is None:
                frames = dict(getattr(self.source, "frames", {}))
            else:
                with lock:
                    frames = dict(getattr(self.source, "frames", {}))
            now = self.clock()
            stopped = True
            for name in ("FL", "FR", "RL", "RR"):
                entry = frames.get(name)
                if not entry or not 0 <= now - entry[1] < 0.75:
                    stopped = False
                    break
                frame = entry[0]
                rpm = frame.get("scRpm")
                if (frame.get("brakeId") != brake_id or frame.get("drivePercent") != 0
                        or frame.get("cal") is not True or frame.get("moving")
                        or frame.get("calib") or frame.get("cycle")
                        or frame.get("scRecent") is not False
                        or type(frame.get("firmware")) is not int or frame["firmware"] < 19
                        or type(frame.get("scCount")) is not int or not 0 <= frame["scCount"] <= 0xFFFFFFFF
                        or isinstance(rpm, bool) or not isinstance(rpm, (int, float))
                        or not math.isfinite(rpm) or not 0 <= rpm <= 1.0):
                    stopped = False
                    break
            if stopped:
                return
            self._set_state(phase="executing", message="Ждём STOP и полной остановки всех четырёх колёс.")
            cancel.wait(0.05)
        raise NavigationError("brake_unconfirmed", "Не получены свежие подтверждения STOP и остановки SC всех колёс", 409)

    def _check_running_feedback(self, bridge: MasterBridge, samples: dict,
                                started_at: float) -> None:
        """Reject missing wheel commands, stale telemetry or silent SC sensors.

        SC is unsigned and is not independent localization; it is only a
        limited fail-safe against a stopped/disconnected wheel while moving.
        """
        lock = getattr(self.source, "lock", None)
        if lock is None:
            frames = dict(getattr(self.source, "frames", {}))
        else:
            with lock:
                frames = dict(getattr(self.source, "frames", {}))
        now, monotonic = self.clock(), time.monotonic()
        for name in ("FL", "FR", "RL", "RR"):
            entry = frames.get(name)
            if not entry or not 0 <= now - entry[1] < 0.75:
                raise NavigationError("module_offline", "Нет свежей телеметрии колеса %s" % name, 409)
            frame = entry[0]
            count = frame.get("scCount")
            if (type(frame.get("firmware")) is not int or frame["firmware"] < 19
                    or type(count) is not int or not 0 <= count <= 0xFFFFFFFF
                    or type(frame.get("scRecent")) is not bool):
                raise NavigationError("sc_invalid", "Нет корректной телеметрии SC колеса %s" % name, 409)
            if (monotonic - started_at > 0.75 and frame.get("drivePercent") != bridge.percent):
                raise NavigationError("drive_unconfirmed", "Колесо %s не подтвердило тягу" % name, 409)
            sample = samples.get(name)
            if sample is None:
                samples[name] = {"count": count, "sampleAt": monotonic, "pulseAt": monotonic}
                continue
            delta = (count - sample["count"]) & 0xFFFFFFFF
            if delta > 6700 * (max(0.0, monotonic - sample["sampleAt"]) + 0.5) + 2:
                raise NavigationError("sc_changed", "Счётчик SC колеса %s сброшен или повреждён" % name, 409)
            if delta:
                sample["pulseAt"] = monotonic
            sample.update(count=count, sampleAt=monotonic)
            if monotonic - sample["pulseAt"] >= 3.0:
                raise NavigationError("sc_stalled", "Нет импульсов SC колеса %s при движении" % name, 409)

    def _fail_safe(self, bridge: Optional[MasterBridge], message: str,
                   phase: str = "blocked") -> None:
        try:
            self._brake(bridge, "navigation_" + phase)
        except Exception as exc:  # noqa: BLE001 — аппаратный watchdog
            log.error("navigation brake failed: %s", exc)
        self._set_state(phase=phase, message=message, progress=0.0)
        log.warning("navigation stopped (%s): %s", phase, message)

    def _fresh_snapshot(self, cancel: threading.Event) -> dict:
        if cancel.is_set():
            raise NavigationError("cancelled", "Маршрут отменён оператором", 409)
        if estop_latch.latched() is not None:
            raise NavigationError("estop", "E-STOP взведён — движение остановлено", 409)
        snapshot = self._snapshot()
        ready, reason, _diagnostics = self._preflight(snapshot, allow_alignment=True)
        if not ready:
            raise NavigationError("not_ready", reason, 409)
        return snapshot

    def _run(self, goal: dict, cancel: threading.Event,
             confirmed_plan: Optional[dict] = None) -> None:
        bridge: Optional[MasterBridge] = None
        try:
            bridge = MasterBridge(
                self.source,
                SwerveKinematics(),
                mps_per_percent=self.mps_per_percent,
                max_steer_deg=HARDWARE_STEER_DEG,
                max_drive_percent=self.max_drive_percent,
                align_timeout_s=4.0,
            )
            with self._lock:
                self._bridge = bridge
            self._await_stopped(bridge, cancel, "navigation_start")
            initial = self._fresh_snapshot(cancel)
            remaining_goals = [dict(g) for g in (confirmed_plan or {}).get("goalsRequested", [goal])]
            blind_zone = (confirmed_plan or {}).get("blindZone") or _initial_blind_zone(initial)
            blind_moved = (bool((confirmed_plan or {}).get("manualBlindZone"))
                           and _blind_zone_moved(initial, blind_zone))
            if blind_moved:
                blind_zone = _initial_blind_zone(initial)
            plan = self._plan_route(initial, remaining_goals, blind_zone=blind_zone)
            requested_goal = goal.get("requested") or {"x": goal["x"], "y": goal["y"]}
            plan["goalRequested"] = {"x": round(float(requested_goal["x"]), 3),
                                     "y": round(float(requested_goal["y"]), 3)}
            if confirmed_plan and (blind_moved or not _plans_equivalent(confirmed_plan, plan)):
                with self._lock:
                    self._plan = plan
                    self._goal = dict(plan["goalRequested"])
                raise NavigationError(
                    "plan_changed",
                    "Карта изменилась перед началом движения; новый маршрут показан для повторной проверки.",
                    409,
                )
            # Partial preview is not the mission length. Budget for the
            # requested points as well, otherwise exploration times out early.
            point = initial["pose"]
            requested_length = 0.0
            for requested in remaining_goals:
                requested_length += math.hypot(requested["x"] - point["x"], requested["y"] - point["y"])
                point = requested
            total_length = max(0.01, float(plan["distanceM"]), requested_length)
            mission_timeout = max(60.0, min(600.0, total_length / self.speed_mps * 4.0 + 30.0))
            with self._lock:
                self._plan = plan
            traveled_estimate = 0.0
            started = self.clock()
            last_position = (float(initial["pose"]["x"]), float(initial["pose"]["y"]))
            last_progress_at = self.clock()
            segment_index = 0
            frontier_wait_since = None
            segment_block_count = 0
            self._set_state(phase="executing", message="Едем по рассчитанной траектории.",
                            progress=0.0, waypoint=0,
                            totalWaypoints=plan["waypoints"], plan=plan)

            while not cancel.is_set():
                snapshot = self._fresh_snapshot(cancel)
                pose = snapshot["pose"]
                current = (float(pose["x"]), float(pose["y"]))
                # Proximity alone is not success: a nearby requested point can
                # still be UNKNOWN or blocked under the robot's footprint.
                near_first = math.hypot(remaining_goals[0]["x"] - current[0],
                                        remaining_goals[0]["y"] - current[1]) <= DEFAULT_WAYPOINT_TOLERANCE_M
                if near_first:
                    clearance = _ClearanceGrid(snapshot, self.footprint_m, blind_zone)
                    goal_cell = _cell(snapshot, remaining_goals[0]["x"], remaining_goals[0]["y"])
                    if goal_cell is not None and clearance.clear_cell(*goal_cell):
                        remaining_goals.pop(0)
                        if not remaining_goals:
                            self._brake(bridge, "navigation_goal")
                            self._set_state(phase="succeeded", message="Цель достигнута.", progress=1.0,
                                            waypoint=segment_index, totalWaypoints=plan["waypoints"])
                            return
                if self.clock() - started > mission_timeout:
                    raise NavigationError("mission_timeout", "Истёк тайм-аут маршрута", 409)

                # Перестраиваем путь от текущей позы: карта обновляется живым
                # лидаром, а неизученное/новое препятствие не обходится вслепую.
                plan = self._plan_route(snapshot, remaining_goals, blind_zone=blind_zone)
                requested_goal = goal.get("requested") or {"x": goal["x"], "y": goal["y"]}
                plan["goalRequested"] = {"x": round(float(requested_goal["x"]), 3),
                                         "y": round(float(requested_goal["y"]), 3)}
                with self._lock:
                    self._plan = plan
                path = plan["path"]
                if len(path) < 2:
                    if plan.get("partial"):
                        if frontier_wait_since is None:
                            frontier_wait_since = time.monotonic()
                        if time.monotonic() - frontier_wait_since < 1.5:
                            self._set_state(phase="executing", message="На границе известного: ждём новые сканы, тяга выключена.")
                            cancel.wait(0.2)
                            continue
                        raise NavigationError("frontier_blocked", "Нет подтверждённого свободного пути дальше: робот остановлен у границы карты", 409)
                    raise NavigationError("no_path", "Остаток пути исчез после перепланирования", 409)
                frontier_wait_since = None
                target = path[1]
                dx = float(target["x"]) - current[0]
                dy = float(target["y"]) - current[1]
                segment_length = math.hypot(dx, dy)
                if segment_length < 0.10:
                    raise NavigationError("waypoint_too_close",
                                          "Планировщик вернул слишком короткий участок; остановка вместо повторного цикла",
                                          409)
                world_angle = math.atan2(dy, dx)
                if steering_deviation(world_angle, float(pose["yaw"])) > math.radians(HARDWARE_STEER_DEG - 0.5):
                    raise NavigationError("steer_limit", "Маршрут требует поворота корпуса, который это шасси не умеет", 409)
                ux, uy = dx / segment_length, dy / segment_length
                segment_start = current
                segment_started = self.clock()
                last_progress_at = segment_started
                segment_start_distance = segment_length
                segment_timeout = max(8.0, segment_length / max(0.01, self.speed_mps) * 5.0 + 5.0)
                sc_samples: dict = {}
                drive_started_at = None

                while not cancel.is_set():
                    snapshot = self._fresh_snapshot(cancel)
                    pose = snapshot["pose"]
                    current = (float(pose["x"]), float(pose["y"]))
                    remaining = math.hypot(target["x"] - current[0], target["y"] - current[1])
                    if remaining <= DEFAULT_WAYPOINT_TOLERANCE_M:
                        self._await_stopped(bridge, cancel, "navigation_waypoint")
                        traveled_estimate += max(0.0, segment_start_distance - remaining)
                        segment_index += 1
                        segment_block_count = 0
                        break
                    # The old segment can become unknown/occupied after a new
                    # scan or map reset. Brake immediately, then replan while
                    # stopped; never continue on a stale safe corridor.
                    clearance = _ClearanceGrid(snapshot, self.footprint_m, blind_zone)
                    if not clearance.clear_line(snapshot, current, (float(target["x"]), float(target["y"]))):
                        self._await_stopped(bridge, cancel, "navigation_map_changed")
                        segment_block_count += 1
                        if segment_block_count > 3:
                            raise NavigationError("map_blocked", "Карта часто меняется; дальнейший участок не подтверждён", 409)
                        break
                    offset_x = current[0] - segment_start[0]
                    offset_y = current[1] - segment_start[1]
                    along_track = offset_x * ux + offset_y * uy
                    cross_track = abs(offset_x * uy - offset_y * ux)
                    if cross_track > max(0.25, self.footprint_m * 0.5):
                        raise NavigationError("off_path", "Робот ушёл в сторону от участка траектории", 409)
                    if along_track > segment_start_distance + DEFAULT_WAYPOINT_TOLERANCE_M:
                        raise NavigationError("waypoint_overshot", "Робот прошёл следующую точку — остановка и перепланирование", 409)
                    if self.clock() - segment_started > segment_timeout:
                        raise NavigationError("segment_timeout", "Робот не дошёл до следующей точки", 409)
                    if self.clock() - started > mission_timeout:
                        raise NavigationError("mission_timeout", "Истёк тайм-аут маршрута", 409)

                    moved = math.hypot(current[0] - last_position[0], current[1] - last_position[1])
                    if moved >= 0.025:
                        last_position = current
                        last_progress_at = self.clock()
                    elif bridge.percent == 0:
                        last_progress_at = self.clock()  # руление/ожидание — не движение
                    elif self.clock() - last_progress_at > 4.0:
                        raise NavigationError("no_progress", "Поза робота не меняется — привод остановлен", 409)

                    yaw = float(pose["yaw"])
                    body_x = math.cos(yaw) * ux + math.sin(yaw) * uy
                    body_y = -math.sin(yaw) * ux + math.cos(yaw) * uy
                    body_angle = math.atan2(body_y, body_x)
                    if abs(body_angle) > math.radians(HARDWARE_STEER_DEG + 0.5) and abs(abs(body_angle) - math.pi) > math.radians(HARDWARE_STEER_DEG + 0.5):
                        raise NavigationError(
                            "steer_limit",
                            "Курс изменился: новый участок вне допуска руля ±%.0f°" % HARDWARE_STEER_DEG,
                            409,
                        )

                    scan = self._scan()
                    if not _scan_coverage(scan)[0]:
                        raise NavigationError("lidar_stale", "Скан лидара устарел или потерял обзор во время движения", 409)
                    obstruction = _obstacle_ahead(
                        scan, body_angle,
                        self.footprint_m + max(0.30, self.speed_mps * 3.0 + 0.15),
                        half_width_m=self.footprint_m + 0.05,
                        lidar_offset_m=self.lidar_offset_m,
                    )
                    if obstruction is not None:
                        raise NavigationError("obstacle", "Лидар увидел препятствие в %.2f м — остановка" % obstruction, 409)

                    if bridge.percent != 0:
                        if drive_started_at is None:
                            drive_started_at = time.monotonic()
                        self._check_running_feedback(bridge, sc_samples, drive_started_at)
                    else:
                        drive_started_at = None
                        sc_samples.clear()

                    # Медленная тяга, рассчитанная только по измеренной калибровке.
                    vx, vy = body_x * self.speed_mps, body_y * self.speed_mps
                    with self._motion_lock:
                        if cancel.is_set():
                            break
                        result = bridge.step(vx, vy, 0.0)
                    if cancel.is_set():
                        break
                    if not result.get("ok"):
                        reason = str(result.get("reason") or "drive_refused")
                        if (reason == "module_busy" and bridge.percent == 0
                                and self.clock() - segment_started <= 4.5):
                            # После STOP прошивка ещё может подтверждать движение
                            # или заканчивать рулевой цикл; тяги нет, подождём.
                            self._set_state(phase="executing",
                                            message="Ждём полной остановки рулевых модулей.",
                                            waypoint=segment_index,
                                            totalWaypoints=plan["waypoints"])
                            cancel.wait(0.2)
                            continue
                        raise NavigationError(reason,
                                              "Шасси остановило маршрут: %s" % reason,
                                              409)
                    phase = str(result.get("phase") or "executing")
                    progress = min(0.99, max(0.0, (traveled_estimate + max(0.0, segment_start_distance - remaining)) / total_length))
                    self._set_state(phase="executing", message={
                        "steering": "Останавливаемся и выставляем колёса.",
                        "aligning": "Ждём подтверждения положения колёс.",
                        "driving": "Едем по траектории.",
                        "braking": "Остановка перед сменой направления.",
                        "stopped": "Движение приостановлено мостом шасси.",
                    }.get(phase, "Едем по траектории."), progress=progress,
                        waypoint=segment_index, totalWaypoints=plan["waypoints"],
                        currentPose={"x": round(current[0], 3), "y": round(current[1], 3),
                                     "yaw": round(yaw, 3)}, drivePercent=result.get("percent", 0))
                    cancel.wait(0.2)
                if cancel.is_set():
                    break

            self._brake(bridge, "navigation_cancel")
            self._set_state(phase="cancelled", message="Маршрут остановлен.", progress=0.0)
        except NavigationError as exc:
            phase = "cancelled" if exc.code == "cancelled" else (
                "planned" if exc.code == "plan_changed" else "blocked")
            self._fail_safe(bridge, exc.message, phase=phase)
        except Exception as exc:  # noqa: BLE001 — любой сбой заканчивается тормозом
            log.exception("unexpected navigation failure")
            self._fail_safe(bridge, "Ошибка навигации: %s" % str(exc)[:160], phase="error")
        finally:
            with self._lock:
                self._bridge = None
                self._worker = None
                if self._cancel.is_set() and self._state.get("phase") == "stopping":
                    self._state.update({"phase": "cancelled", "message": "Маршрут остановлен.",
                                        "updatedAt": self.clock()})
