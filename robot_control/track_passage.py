"""Geometric guard for the closed course, in chassis coordinates (X forward, Y left).

Orange posts are occupied space. A pair can mark a gate, but seeing a gap in
an image is not sufficient: the fixed-yaw crab motion must fit through it.
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .track_autonomy import TrackConfig

PASSAGE_REASONS = {
    "cones_missing": "Не видно двух границ из столбиков",
    "cones_far": "Столбики слишком далеко",
    "corridor_unpaired": "Столбики не образуют подтверждённого проезда",
    "corridor_narrow": "Проезд уже корпуса с запасом",
    "corridor_wide": "Границы слишком далеко друг от друга",
    "too_far_from_cones": "Робот слишком далеко от центра проезда",
    "unreachable_gap": "Не получается войти в проезд без пересечения границы",
    "post_in_path": "Оранжевый столбик на пути корпуса",
}


def _xy(point: dict[str, Any]) -> tuple[float, float] | None:
    try:
        x, y = float(point["x"]), float(point["y"])
    except (KeyError, TypeError, ValueError):
        return None
    return (x, y) if math.isfinite(x) and math.isfinite(y) else None


def swept_hit(x: float, y: float, angle_deg: float, travel_m: float,
              half_length: float, half_width: float) -> bool:
    """Does the translated (not rotated) chassis rectangle cover this point?"""
    angle = math.radians(angle_deg)
    dx, dy = travel_m * math.cos(angle), travel_m * math.sin(angle)
    lo, hi = 0.0, 1.0
    for coord, shift, half_size in ((x, dx, half_length), (y, dy, half_width)):
        if abs(shift) < 1e-9:
            if abs(coord) > half_size:
                return False
            continue
        a, b = sorted(((coord - half_size) / shift, (coord + half_size) / shift))
        lo, hi = max(lo, a), min(hi, b)
        if lo > hi:
            return False
    return True


def post_in_path(posts: list[dict[str, Any]], angle_deg: float,
                 travel_m: float, cfg: TrackConfig) -> bool:
    padding = cfg.clearance_m + cfg.post_radius_m
    for post in posts:
        xy = _xy(post)
        if xy and swept_hit(*xy, angle_deg, travel_m,
                            cfg.robot_length_m / 2 + padding, cfg.robot_width_m / 2 + padding):
            return True
    return False


def passage_fits(passage: dict[str, Any], angle_deg: float, cfg: TrackConfig) -> bool:
    """Check actual commanded ray against both edges, including deadband/gyro."""
    slope = math.tan(math.radians(angle_deg))
    margin = cfg.robot_width_m / 2 + cfg.clearance_m + cfg.post_radius_m
    return all(slope * gate["rightX"] - gate["rightY"] >= margin and
               gate["leftY"] - slope * gate["leftX"] >= margin
               for gate in passage.get("gates") or [])


def estimate_corridor(cones: list[dict[str, Any]], cfg: TrackConfig) -> dict[str, Any]:
    """Find two observed edges and steer through a reachable, unobstructed gap.

    Pair opposing posts at nearby longitudinal stations. One boundary never
    implies a second boundary, and a distant wide pair can't erase a closer
    pinch. Even unpaired/outlying detections are still checked as obstacles.
    """
    required = max(cfg.min_corridor_m, cfg.robot_width_m + 2 * cfg.clearance_m + 2 * cfg.post_radius_m)
    base: dict[str, Any] = {"ok": False, "count": 0, "steerLeftDeg": 0.0,
                             "centerM": None, "widthM": None, "requiredWidthM": round(required, 3),
                             "nearestM": None, "reason": "cones_missing", "gates": []}
    forward = [xy for p in cones if (xy := _xy(p)) is not None
               and 0.10 <= xy[0] and abs(xy[1]) <= cfg.max_lateral_m]
    if not forward:
        return base
    nearest = min(x for x, _ in forward)
    base["nearestM"] = round(nearest, 2)
    if nearest > cfg.max_nearest_m:
        return dict(base, reason="cones_far")
    # Collapse duplicate detections before assigning boundaries, but don't
    # discard them from the collision check below.
    seen: list[tuple[float, float]] = []
    for x, y in sorted(forward):
        if x <= cfg.max_nearest_m and not any(math.hypot(x - px, y - py) < 0.12 for px, py in seen):
            seen.append((x, y))
    ordered = sorted(seen, key=lambda p: p[1])
    base["count"] = len(ordered)
    if len(ordered) < max(2, cfg.min_cones):
        return base
    cut = max(range(1, len(ordered)), key=lambda i: ordered[i][1] - ordered[i - 1][1])
    if ordered[cut][1] - ordered[cut - 1][1] < 0.12:
        return base
    right, left = ordered[:cut], ordered[cut:]
    base.update(leftCount=len(left), rightCount=len(right))
    # At most one partner per post, preferring the closest X. This permits
    # staggered rows without pairing a near post with a distant turn.
    matches = sorted((abs(r[0] - l[0]), ri, li) for ri, r in enumerate(right)
                     for li, l in enumerate(left) if abs(r[0] - l[0]) <= 0.75)
    used_right, used_left = set(), set()
    gates = []
    for _, ri, li in matches:
        if ri in used_right or li in used_left:
            continue
        used_right.add(ri)
        used_left.add(li)
        rx, ry = right[ri]
        lx, ly = left[li]
        gates.append({"x": (rx + lx) / 2, "y": (ry + ly) / 2, "width": ly - ry,
                      "rightX": rx, "rightY": ry, "leftX": lx, "leftY": ly})
    if not gates:
        return dict(base, reason="corridor_unpaired")
    gates.sort(key=lambda g: g["x"])
    width = min(g["width"] for g in gates)
    base.update(widthM=round(width, 3), centerM=round(gates[0]["y"], 3))
    if width < required:
        return dict(base, reason="corridor_narrow")
    if max(g["width"] for g in gates) > cfg.max_corridor_m:
        return dict(base, reason="corridor_wide")
    if abs(gates[0]["y"]) > cfg.max_offset_m:
        return dict(base, reason="too_far_from_cones")

    preferred = min(range(len(gates)), key=lambda i: abs(gates[i]["x"] - cfg.lookahead_m))
    reason = "unreachable_gap"
    # A short safe segment may be possible before a bend. Never try another
    # gap to the outside or drive toward the largest empty image region.
    for index in range(preferred, -1, -1):
        gate = gates[index]
        heading = math.degrees(math.atan2(gate["y"], gate["x"]))
        steer = max(-cfg.max_steer_deg, min(cfg.max_steer_deg, heading * cfg.steer_gain))
        travel = max(cfg.stop_clearance_m, max(gate["leftX"], gate["rightX"]) /
                     math.cos(math.radians(steer)))
        checked = [g for g in gates if min(g["leftX"], g["rightX"]) <= travel + cfg.robot_length_m / 2]
        if not passage_fits({"gates": checked}, steer, cfg):
            continue
        if post_in_path(cones, steer, travel, cfg):
            reason = "post_in_path"
            continue
        return dict(base, ok=True, gates=checked, reason="", headingDeg=round(heading, 2),
                    targetXm=round(gate["x"], 3), centerAheadM=round(gate["y"], 3),
                    travelM=round(travel, 3), steerLeftDeg=steer)
    return dict(base, reason=reason)
