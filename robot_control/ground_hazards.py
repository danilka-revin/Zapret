"""Рельеф покрытия по Kinect: ямы, кочки и неизвестная поверхность.

Это вспомогательный слой, а не разрешение движения. В отличие от
`kinect.depth_to_scan`, пол здесь не отбрасывается: точки сохраняются в XYZ,
усредняются по ячейкам, сравниваются с опорной плоскостью дороги и только
подтверждённые области получают класс `pothole` или `bump`.

Пустой/засвеченный кадр не становится ровной дорогой: возвращается `unknown`.
Модуль не открывает USB и не управляет приводом.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Any, Iterable, Optional, Sequence


@dataclass(frozen=True)
class GroundConfig:
    cell_size_m: float = 0.05
    min_x_m: float = 0.30
    max_x_m: float = 4.0
    half_width_m: float = 1.20
    pothole_depth_m: float = 0.04
    bump_height_m: float = 0.03
    min_cells: int = 4
    plane_inlier_m: float = 0.02
    min_plane_points: int = 24
    ransac_iterations: int = 80
    min_valid_cells: int = 30
    min_plane_inlier_ratio: float = 0.55
    max_plane_slope: float = 0.5
    confirm_frames: int = 3
    match_distance_m: float = 0.25
    max_missed_frames: int = 2

    def validate(self) -> "GroundConfig":
        if not (0.01 <= self.cell_size_m <= 0.5):
            raise ValueError("cell_size_m вне диапазона")
        if not (0.0 <= self.min_x_m < self.max_x_m <= 20.0):
            raise ValueError("Неверный продольный диапазон")
        if not (0.1 <= self.half_width_m <= 10.0):
            raise ValueError("half_width_m вне диапазона")
        if not (0.005 <= self.pothole_depth_m <= 0.5 and 0.005 <= self.bump_height_m <= 0.5):
            raise ValueError("Пороги рельефа вне диапазона")
        if self.min_cells < 1 or self.min_plane_points < 3 or self.min_valid_cells < 3:
            raise ValueError("Слишком мало ячеек для подтверждения")
        if not (0.1 <= self.min_plane_inlier_ratio <= 1.0 and 0.0 <= self.max_plane_slope <= 2.0):
            raise ValueError("Неверные ограничения опорной плоскости")
        if self.confirm_frames < 1 or self.max_missed_frames < 0 or self.match_distance_m <= 0:
            raise ValueError("Неверные параметры подтверждения между кадрами")
        return self


def _finite_point(raw: Any) -> Optional[tuple[float, float, float]]:
    if isinstance(raw, dict):
        values = (raw.get("x"), raw.get("y"), raw.get("z"))
    elif isinstance(raw, (list, tuple)) and len(raw) == 3:
        values = raw
    else:
        return None
    try:
        x, y, z = (float(v) for v in values)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in (x, y, z)):
        return None
    return x, y, z


def _solve3(a: list[list[float]], b: list[float]) -> Optional[tuple[float, float, float]]:
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-12:
            return None
        m[col], m[pivot] = m[pivot], m[col]
        for r in range(col + 1, 3):
            k = m[r][col] / m[col][col]
            for c in range(col, 4):
                m[r][c] -= k * m[col][c]
    x = [0.0] * 3
    for r in range(2, -1, -1):
        x[r] = (m[r][3] - sum(m[r][c] * x[c] for c in range(r + 1, 3))) / m[r][r]
    return x[0], x[1], x[2]


def _plane_from_points(a: tuple[float, float, float], b: tuple[float, float, float],
                       c: tuple[float, float, float]) -> Optional[tuple[float, float, float]]:
    # z = p*x + q*y + r
    return _solve3([[a[0], a[1], 1.0], [b[0], b[1], 1.0], [c[0], c[1], 1.0]],
                   [a[2], b[2], c[2]])


def _fit_plane(cells: list[tuple[float, float, float]], config: GroundConfig,
               seed: int = 7) -> Optional[tuple[tuple[float, float, float], float]]:
    if len(cells) < config.min_plane_points:
        return None
    rng = random.Random(seed)
    best: Optional[tuple[tuple[float, float, float], list[tuple[float, float, float]]]] = None
    for _ in range(max(1, config.ransac_iterations)):
        a, b, c = rng.sample(cells, 3)
        plane = _plane_from_points(a, b, c)
        if plane is None:
            continue
        p, q, r = plane
        inliers = [cell for cell in cells if abs(cell[2] - (p * cell[0] + q * cell[1] + r)) <= config.plane_inlier_m]
        if best is None or len(inliers) > len(best[1]):
            best = (plane, inliers)
    if best is None or len(best[1]) < config.min_plane_points:
        return None
    inliers = best[1]
    ata = [[0.0] * 3 for _ in range(3)]
    atb = [0.0] * 3
    for x, y, z in inliers:
        row = [x, y, 1.0]
        for i in range(3):
            atb[i] += row[i] * z
            for j in range(3):
                ata[i][j] += row[i] * row[j]
    refit = _solve3(ata, atb) or best[0]
    return refit, len(inliers) / len(cells)


def _median(values: list[float]) -> float:
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def _components(flags: dict[tuple[int, int], float]) -> list[list[tuple[int, int]]]:
    visited: set[tuple[int, int]] = set()
    result: list[list[tuple[int, int]]] = []
    for start in flags:
        if start in visited:
            continue
        stack = [start]
        visited.add(start)
        component: list[tuple[int, int]] = []
        while stack:
            item = stack.pop()
            component.append(item)
            x, y = item
            for neighbour in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if neighbour in flags and neighbour not in visited:
                    visited.add(neighbour)
                    stack.append(neighbour)
        result.append(component)
    return result


def analyze_ground_points(points: Iterable[Any], config: Optional[GroundConfig] = None,
                          *, seed: int = 7) -> dict[str, Any]:
    """Ямы/кочки по точкам XYZ в системе робота: X вперёд, Y влево, Z вверх."""
    cfg = (config or GroundConfig()).validate()
    buckets: dict[tuple[int, int], list[float]] = {}
    valid = 0
    for raw in points:
        point = _finite_point(raw)
        if point is None:
            continue
        x, y, z = point
        if not (cfg.min_x_m <= x <= cfg.max_x_m and abs(y) <= cfg.half_width_m):
            continue
        valid += 1
        key = (int(math.floor(x / cfg.cell_size_m)), int(math.floor(y / cfg.cell_size_m)))
        buckets.setdefault(key, []).append(z)
    cells: list[tuple[float, float, float]] = []
    cell_heights: dict[tuple[int, int], float] = {}
    for (ix, iy), values in buckets.items():
        x = (ix + 0.5) * cfg.cell_size_m
        y = (iy + 0.5) * cfg.cell_size_m
        z = _median(values)
        cells.append((x, y, z))
        cell_heights[(ix, iy)] = z
    base = {"ok": True, "validPoints": valid, "validCells": len(cells),
            "cellSizeM": cfg.cell_size_m, "hazards": [], "tractionConnected": False}
    if len(cells) < cfg.min_valid_cells:
        return dict(base, status="unknown", reason="insufficient_ground_data")
    fitted = _fit_plane(cells, cfg, seed=seed)
    if fitted is None:
        return dict(base, status="unknown", reason="plane_not_confirmed")
    plane, inlier_ratio = fitted
    p, q, r = plane
    if inlier_ratio < cfg.min_plane_inlier_ratio or math.hypot(p, q) > cfg.max_plane_slope:
        return dict(base, status="unknown", reason="plane_not_reliable")
    residuals: dict[tuple[int, int], float] = {}
    for (ix, iy), z in cell_heights.items():
        x = (ix + 0.5) * cfg.cell_size_m
        y = (iy + 0.5) * cfg.cell_size_m
        residuals[(ix, iy)] = z - (p * x + q * y + r)
    hazards: list[dict[str, Any]] = []
    for kind, threshold, sign in (("pothole", cfg.pothole_depth_m, -1),
                                  ("bump", cfg.bump_height_m, 1)):
        flags = {key: value for key, value in residuals.items() if sign * value >= threshold}
        for component in _components(flags):
            if len(component) < cfg.min_cells:
                continue
            xs = [(key[0] + 0.5) * cfg.cell_size_m for key in component]
            ys = [(key[1] + 0.5) * cfg.cell_size_m for key in component]
            depths = [residuals[key] for key in component]
            severity = max(abs(value) for value in depths)
            center_x = sum(xs) / len(xs)
            center_y = sum(ys) / len(ys)
            hazards.append({
                "kind": kind,
                "center": [round(center_x, 3), round(center_y, 3)],
                "bbox": [round(min(xs) - cfg.cell_size_m / 2, 3),
                         round(min(ys) - cfg.cell_size_m / 2, 3),
                         round(max(xs) + cfg.cell_size_m / 2, 3),
                         round(max(ys) + cfg.cell_size_m / 2, 3)],
                "distanceM": round(center_x, 3),
                "severityM": round(severity, 3),
                "areaM2": round(len(component) * cfg.cell_size_m * cfg.cell_size_m, 4),
                "cells": len(component),
                "confidence": round(min(1.0, len(component) / max(1, cfg.min_cells)) * inlier_ratio, 3),
            })
    hazards.sort(key=lambda item: (item["distanceM"], item["kind"]))
    return dict(base, status="ok", reason="", plane=[round(p, 5), round(q, 5), round(r, 5)],
                planeInlierRatio=round(inlier_ratio, 3), hazards=hazards)


def ground_points_from_depth(frame: Any, spec: dict[str, Any], *,
                             body_pitch_deg: float = 0.0,
                             body_roll_deg: float = 0.0) -> list[dict[str, float]]:
    """Кадр глубины → XYZ без фильтра пола.

    `frame` совместим с `robot_control.kinect.DepthFrame`. Используются монтаж
    и оптика из `sensor_layout.kinect_spec()`. Знак высоты сохраняется.

    Наклон корпуса компенсируется отдельно: положительный pitch — нос вниз,
    положительный roll — левый борт вверх. Если IMU не даёт свежий наклон,
    вызывающий код должен передать 0 и пометить результат как менее надёжный,
    а не угадывать углы.
    """
    width = int(frame.width)
    height = int(frame.height)
    if width < 2 or height < 2 or len(frame.values) != width * height:
        raise ValueError("неверный размер кадра глубины")
    body_pitch = math.radians(_finite_scalar(body_pitch_deg, "body_pitch_deg", -90.0, 90.0))
    body_roll = math.radians(_finite_scalar(body_roll_deg, "body_roll_deg", -90.0, 90.0))
    cbp, sbp = math.cos(body_pitch), math.sin(body_pitch)
    cbr, sbr = math.cos(body_roll), math.sin(body_roll)
    fx = float(getattr(frame, "fx", 0.0)) or width / (2.0 * math.tan(math.radians(float(spec["hfov_deg"])) / 2.0))
    fy = float(getattr(frame, "fy", 0.0)) or height / (2.0 * math.tan(math.radians(float(spec["vfov_deg"])) / 2.0))
    cx, cy = (width - 1) / 2.0, (height - 1) / 2.0
    yaw = math.radians(float(spec.get("yaw_deg", 0.0)))
    pitch = math.radians(float(spec.get("pitch_deg", 0.0)))
    cos_p, sin_p = math.cos(pitch), math.sin(pitch)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    min_r = float(spec.get("min_range_m", 0.45))
    max_r = float(spec.get("max_range_m", 4.0))
    mount_x = float(spec.get("x_m", 0.0))
    mount_y = float(spec.get("y_m", 0.0))
    mount_z = float(spec.get("z_m", 0.45))
    result: list[dict[str, float]] = []
    values = frame.values
    for v in range(height):
        row = v * width
        for u in range(width):
            z = float(values[row + u])
            if not math.isfinite(z) or z < min_r or z > max_r:
                continue
            x_cam = (u - cx) / fx * z
            y_cam = -(v - cy) / fy * z
            y_up = y_cam * cos_p - z * sin_p
            z_fwd = y_cam * sin_p + z * cos_p
            x_r, y_r = z_fwd, -x_cam
            x = x_r * cos_y - y_r * sin_y + mount_x
            y = x_r * sin_y + y_r * cos_y + mount_y
            z_body = y_up + mount_z
            # Из системы наклонённого корпуса в горизонтальную систему робота.
            x_level = x * cbp + z_body * sbp
            z_pitch = -x * sbp + z_body * cbp
            y_level = y * cbr - z_pitch * sbr
            z_level = y * sbr + z_pitch * cbr
            result.append({"x": x_level, "y": y_level, "z": z_level})
    return result


def _finite_scalar(value: Any, name: str, lo: float, hi: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name}: ожидается число") from None
    if not math.isfinite(result) or not lo <= result <= hi:
        raise ValueError(f"{name}: число вне диапазона")
    return result


@dataclass
class _GroundTrack:
    kind: str
    center: tuple[float, float]
    bbox: tuple[float, float, float, float]
    hits: int = 1
    misses: int = 0
    last_stamp: float = 0.0
    payload: Optional[dict[str, Any]] = None


class GroundHazardTracker:
    """Подтверждает дефект покрытия в нескольких последовательных кадрах."""

    def __init__(self, config: Optional[GroundConfig] = None) -> None:
        self.config = (config or GroundConfig()).validate()
        self._tracks: list[_GroundTrack] = []
        self._last_stamp: Optional[float] = None

    def reset(self) -> None:
        self._tracks.clear()
        self._last_stamp = None

    def update(self, analysis: dict[str, Any], stamp: float) -> dict[str, Any]:
        stamp = _finite_scalar(stamp, "stamp", 0.0, 1e12)
        if self._last_stamp is not None and stamp <= self._last_stamp:
            raise ValueError("stamp должен возрастать")
        self._last_stamp = stamp
        candidates = list(analysis.get("hazards") or []) if analysis.get("status") == "ok" else []
        used: set[int] = set()
        for hazard in candidates:
            center = tuple(float(v) for v in hazard.get("center", (0.0, 0.0)))
            bbox = tuple(float(v) for v in hazard.get("bbox", (0.0, 0.0, 0.0, 0.0)))
            best_index = None
            best_distance = self.config.match_distance_m
            for index, track in enumerate(self._tracks):
                if index in used or track.kind != hazard.get("kind"):
                    continue
                distance = math.hypot(track.center[0] - center[0], track.center[1] - center[1])
                if distance <= best_distance:
                    best_index, best_distance = index, distance
            if best_index is None:
                track = _GroundTrack(str(hazard.get("kind")), center, bbox, last_stamp=stamp, payload=dict(hazard))
                self._tracks.append(track)
                used.add(len(self._tracks) - 1)
            else:
                track = self._tracks[best_index]
                track.hits += 1
                track.misses = 0
                track.center = center
                track.bbox = bbox
                track.last_stamp = stamp
                track.payload = dict(hazard)
                used.add(best_index)
        confirmed: list[dict[str, Any]] = []
        surviving: list[_GroundTrack] = []
        for index, track in enumerate(self._tracks):
            if index not in used:
                track.misses += 1
                track.hits = 0
            if track.misses <= self.config.max_missed_frames:
                surviving.append(track)
            if track.hits >= self.config.confirm_frames and track.misses == 0 and track.payload is not None:
                item = dict(track.payload)
                item["confirmedFrames"] = track.hits
                item["confidence"] = round(min(1.0, float(item.get("confidence", 0.0))), 3)
                confirmed.append(item)
        self._tracks = surviving
        confirmed.sort(key=lambda item: (item.get("distanceM", 0.0), item.get("kind", "")))
        status = analysis.get("status", "unknown")
        return {"ok": True, "status": status, "reason": analysis.get("reason", ""),
                "hazards": confirmed, "candidates": candidates,
                "candidateCount": len(candidates), "confirmedCount": len(confirmed),
                "validPoints": analysis.get("validPoints", 0), "validCells": analysis.get("validCells", 0),
                "plane": analysis.get("plane"), "planeInlierRatio": analysis.get("planeInlierRatio"),
                "tractionConnected": False}
