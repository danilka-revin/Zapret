"""Сопровождение ближних объектов по 2D-лидару с компенсацией движения робота.

Кластеры хранятся в мировой системе координат. Сканы переводятся в неё через
интегрированную позу робота, поэтому неподвижная стена не становится «едущей
навстречу», когда робот сам движется вперёд. Модуль не классифицирует
«человек/машина» и не управляет тягой.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable, Optional


def _number(value: Any, name: str, lo: float, hi: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name}: ожидается число") from None
    if not math.isfinite(result) or not lo <= result <= hi:
        raise ValueError(f"{name}: число вне диапазона")
    return result


def _rotate(x: float, y: float, angle: float) -> tuple[float, float]:
    c, s = math.cos(angle), math.sin(angle)
    return x * c - y * s, x * s + y * c


@dataclass
class Track:
    id: int
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    last_stamp: float = 0.0
    hits: int = 1
    misses: int = 0

    def to_dict(self, ego_x: float, ego_y: float, ego_yaw: float,
                ego_speed: float, ego_yaw_rate: float, moving_speed: float) -> dict[str, Any]:
        dx, dy = self.x - ego_x, self.y - ego_y
        x, y = _rotate(dx, dy, -ego_yaw)
        # Скорость точки корпуса под роботом: поступательная + omega × r.
        ego_vx = ego_speed - ego_yaw_rate * y
        ego_vy = ego_yaw_rate * x
        rvx, rvy = _rotate(self.vx - ego_vx, self.vy - ego_vy, -ego_yaw)
        speed = math.hypot(self.vx, self.vy)
        closing = max(0.0, -rvx)
        ttc = x / closing if closing > 1e-6 and x > 0 else None
        return {"id": self.id, "x": round(x, 3), "y": round(y, 3),
                "distanceM": round(math.hypot(x, y), 3),
                "vx": round(rvx, 3), "vy": round(rvy, 3),
                "worldVx": round(self.vx, 3), "worldVy": round(self.vy, 3),
                "moving": self.hits >= 3 and speed >= moving_speed,
                "closingSpeedMps": round(closing, 3),
                "ttcS": None if ttc is None else round(ttc, 3), "hits": self.hits}


def cluster_scan(points: Iterable[Any], *, gap_m: float = 0.25, min_points: int = 3,
                 max_range_m: float = 12.0) -> list[tuple[float, float, int]]:
    """Кластеры полярного скана `{a, r}` в системе робота: X вперёд, Y влево."""
    gap_m = _number(gap_m, "gap_m", 1e-6, 10.0)
    max_range_m = _number(max_range_m, "max_range_m", 0.05, 100.0)
    if not isinstance(min_points, int) or min_points < 1:
        raise ValueError("min_points должен быть положительным целым")
    xy: list[tuple[float, float]] = []
    for raw in points:
        if not isinstance(raw, dict):
            continue
        try:
            angle = float(raw.get("a"))
            radius = float(raw.get("r"))
        except (TypeError, ValueError):
            continue
        if not math.isfinite(angle) or not math.isfinite(radius) or not (0.05 <= radius <= max_range_m):
            continue
        xy.append((radius * math.cos(angle), radius * math.sin(angle)))
    if len(xy) < min_points:
        return []
    buckets: dict[tuple[int, int], list[int]] = {}
    for index, (x, y) in enumerate(xy):
        buckets.setdefault((int(math.floor(x / gap_m)), int(math.floor(y / gap_m))), []).append(index)
    parent = list(range(len(xy)))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(i: int, j: int) -> None:
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[rj] = ri

    gap2 = gap_m * gap_m
    for (cx, cy), indices in buckets.items():
        neighbours: list[int] = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                neighbours.extend(buckets.get((cx + dx, cy + dy), ()))
        for i in indices:
            xi, yi = xy[i]
            for j in neighbours:
                if j <= i:
                    continue
                xj, yj = xy[j]
                if (xi - xj) ** 2 + (yi - yj) ** 2 <= gap2:
                    union(i, j)
    groups: dict[int, list[int]] = {}
    for index in range(len(xy)):
        groups.setdefault(find(index), []).append(index)
    result: list[tuple[float, float, int]] = []
    for members in groups.values():
        if len(members) < min_points:
            continue
        x = sum(xy[i][0] for i in members) / len(members)
        y = sum(xy[i][1] for i in members) / len(members)
        result.append((x, y, len(members)))
    result.sort(key=lambda item: (math.hypot(item[0], item[1]), item[0], item[1]))
    return result


@dataclass
class ObjectTracker:
    gate_m: float = 0.6
    max_misses: int = 4
    velocity_blend: float = 0.35
    corridor_half_width_m: float = 0.8
    stop_ttc_s: float = 2.5
    min_moving_speed_mps: float = 0.15
    max_dt_s: float = 1.0
    _tracks: list[Track] = field(default_factory=list)
    _next_id: int = 1
    _ego_x: float = 0.0
    _ego_y: float = 0.0
    _ego_yaw: float = 0.0
    _last_stamp: Optional[float] = None
    _ego_speed: float = 0.0
    _ego_yaw_rate: float = 0.0

    def __post_init__(self) -> None:
        self.gate_m = _number(self.gate_m, "gate_m", 1e-6, 10.0)
        self.corridor_half_width_m = _number(self.corridor_half_width_m, "corridor_half_width_m", 0.05, 20.0)
        self.stop_ttc_s = _number(self.stop_ttc_s, "stop_ttc_s", 0.05, 60.0)
        self.min_moving_speed_mps = _number(self.min_moving_speed_mps, "min_moving_speed_mps", 0.01, 10.0)
        self.velocity_blend = _number(self.velocity_blend, "velocity_blend", 0.01, 1.0)
        self.max_dt_s = _number(self.max_dt_s, "max_dt_s", 0.01, 10.0)
        if not isinstance(self.max_misses, int) or self.max_misses < 0:
            raise ValueError("max_misses должен быть неотрицательным целым")

    def reset(self) -> None:
        self._tracks.clear()
        self._next_id = 1
        self._ego_x = self._ego_y = self._ego_yaw = 0.0
        self._last_stamp = None
        self._ego_speed = self._ego_yaw_rate = 0.0

    def update(self, points: Iterable[Any], stamp: float, *, ego_speed_mps: float = 0.0,
               ego_yaw_rate_rps: float = 0.0) -> dict[str, Any]:
        stamp = _number(stamp, "stamp", 0.0, 1e12)
        ego_speed_mps = _number(ego_speed_mps, "ego_speed_mps", -10.0, 10.0)
        ego_yaw_rate_rps = _number(ego_yaw_rate_rps, "ego_yaw_rate_rps", -10.0, 10.0)
        if self._last_stamp is not None and stamp <= self._last_stamp:
            raise ValueError("stamp должен возрастать")
        dt = 0.0 if self._last_stamp is None else stamp - self._last_stamp
        if dt > self.max_dt_s:
            self._tracks.clear()
            dt = 0.0
        if dt > 0:
            mid_yaw = self._ego_yaw + ego_yaw_rate_rps * dt / 2.0
            self._ego_x += ego_speed_mps * dt * math.cos(mid_yaw)
            self._ego_y += ego_speed_mps * dt * math.sin(mid_yaw)
            self._ego_yaw += ego_yaw_rate_rps * dt
        self._last_stamp = stamp
        self._ego_speed = ego_speed_mps
        self._ego_yaw_rate = ego_yaw_rate_rps
        clusters = cluster_scan(points)
        world_clusters: list[tuple[float, float, int]] = []
        for x, y, count in clusters:
            rx, ry = _rotate(x, y, self._ego_yaw)
            world_clusters.append((self._ego_x + rx, self._ego_y + ry, count))
        used: set[int] = set()
        previous = {track.id: (track.x, track.y, track.last_stamp) for track in self._tracks}
        for track in self._tracks:
            if dt > 0:
                track.x += track.vx * dt
                track.y += track.vy * dt
        for x, y, count in world_clusters:
            best: Optional[Track] = None
            best_distance = self.gate_m
            for track in self._tracks:
                if track.id in used:
                    continue
                distance = math.hypot(track.x - x, track.y - y)
                if distance <= best_distance:
                    best, best_distance = track, distance
            if best is None:
                best = Track(self._next_id, x, y, last_stamp=stamp)
                self._next_id += 1
                self._tracks.append(best)
            else:
                prev_x, prev_y, prev_stamp = previous[best.id]
                measured_dt = max(1e-6, stamp - prev_stamp)
                measured_vx = (x - prev_x) / measured_dt
                measured_vy = (y - prev_y) / measured_dt
                blend = self.velocity_blend if best.hits > 1 else 1.0
                best.vx = (1.0 - blend) * best.vx + blend * measured_vx
                best.vy = (1.0 - blend) * best.vy + blend * measured_vy
                best.x, best.y = x, y
                best.last_stamp = stamp
                best.hits += 1
                best.misses = 0
            used.add(best.id)
            del count
        surviving: list[Track] = []
        for track in self._tracks:
            if track.id not in used:
                track.misses += 1
            if track.misses <= self.max_misses:
                surviving.append(track)
        self._tracks = surviving
        tracks = [track.to_dict(self._ego_x, self._ego_y, self._ego_yaw,
                                self._ego_speed, self._ego_yaw_rate, self.min_moving_speed_mps)
                  for track in self._tracks]
        risky = [track for track in tracks
                 if track["moving"] and track["x"] > 0
                 and abs(track["y"]) <= self.corridor_half_width_m
                 and track["ttcS"] is not None and track["ttcS"] <= self.stop_ttc_s]
        return {"ok": True, "stamp": stamp, "tracks": tracks,
                "riskyTracks": risky, "stopRecommended": bool(risky), "tractionConnected": False}
