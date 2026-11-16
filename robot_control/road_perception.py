"""Сведение Kinect и лидара в рекомендации для дороги. Только логика, без привода."""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Optional

from .ground_hazards import (GroundConfig, GroundHazardTracker, analyze_ground_points,
                             ground_points_from_depth)
from .object_tracks import ObjectTracker


@dataclass(frozen=True)
class PerceptionConfig:
    ground_max_age_s: float = 0.8
    lidar_max_age_s: float = 0.6
    speed_mps: float = 0.25
    stop_ttc_s: float = 2.5


class RoadPerception:
    """Вход: глубина Kinect / скан лидара. Выход: рекомендации, не моторные команды.

    Kinect-дефект попадает в `hazards` только после подтверждения в нескольких
    кадрах. Пустой, старый или засвеченный кадр остаётся `unknown`, а не «чисто».
    """

    def __init__(self, config: Optional[PerceptionConfig] = None,
                 ground_config: Optional[GroundConfig] = None) -> None:
        self.config = config or PerceptionConfig()
        self.ground_config = (ground_config or GroundConfig()).validate()
        self.tracker = ObjectTracker(stop_ttc_s=self.config.stop_ttc_s)
        self.ground_tracker = GroundHazardTracker(self.ground_config)
        self._ground: dict[str, Any] = {"status": "unknown", "reason": "no_frame", "hazards": [], "candidates": []}
        self._objects: dict[str, Any] = {"ok": False, "reason": "no_scan", "tracks": [], "riskyTracks": []}
        self._ground_at: Optional[float] = None
        self._objects_at: Optional[float] = None
        self._ground_sequence = 0

    def feed_ground_points(self, points: Any, *, stamp: Optional[float] = None) -> dict[str, Any]:
        now = time.monotonic() if stamp is None else float(stamp)
        analysis = analyze_ground_points(points, self.ground_config)
        # Монотонный ключ не зависит от часов датчика; повторный stamp не подтверждает дефект.
        if self._ground_at is not None and now <= self._ground_at:
            return dict(self._ground, ok=False, reason="non_increasing_ground_stamp")
        self._ground_sequence += 1
        result = self.ground_tracker.update(analysis, float(self._ground_sequence))
        self._ground = result
        self._ground_at = now
        return result

    def feed_depth_frame(self, frame: Any, spec: dict[str, Any], *, stamp: Optional[float] = None,
                         body_pitch_deg: float = 0.0, body_roll_deg: float = 0.0) -> dict[str, Any]:
        now = time.monotonic() if stamp is None else float(stamp)
        try:
            points = ground_points_from_depth(frame, spec, body_pitch_deg=body_pitch_deg,
                                              body_roll_deg=body_roll_deg)
            return self.feed_ground_points(points, stamp=now)
        except (TypeError, ValueError, AttributeError, IndexError) as exc:
            self._ground = {"ok": False, "status": "unknown", "reason": "invalid_depth_frame",
                            "detail": str(exc), "hazards": [], "candidates": []}
            self._ground_at = now
            self.ground_tracker.reset()
            return self._ground

    def feed_lidar(self, points: Any, *, stamp: Optional[float] = None,
                   ego_speed_mps: float = 0.0, ego_yaw_rate_rps: float = 0.0,
                   trusted: bool = True) -> dict[str, Any]:
        now = time.monotonic() if stamp is None else float(stamp)
        if not trusted:
            self.tracker.reset()
            self._objects = {"ok": False, "reason": "lidar_not_trusted", "tracks": [], "riskyTracks": []}
            self._objects_at = now
            return self._objects
        try:
            result = self.tracker.update(points, now, ego_speed_mps=ego_speed_mps,
                                         ego_yaw_rate_rps=ego_yaw_rate_rps)
        except ValueError as exc:
            self.tracker.reset()
            self._objects = {"ok": False, "reason": "invalid_lidar_scan", "detail": str(exc),
                             "tracks": [], "riskyTracks": []}
        else:
            self._objects = result
        self._objects_at = now
        return self._objects

    def state(self, *, now: Optional[float] = None) -> dict[str, Any]:
        now = time.monotonic() if now is None else float(now)
        ground_age = None if self._ground_at is None else max(0.0, now - self._ground_at)
        objects_age = None if self._objects_at is None else max(0.0, now - self._objects_at)
        ground_fresh = ground_age is not None and ground_age <= self.config.ground_max_age_s
        objects_fresh = objects_age is not None and objects_age <= self.config.lidar_max_age_s
        ground = self._ground if ground_fresh else {"status": "unknown", "reason": "ground_stale", "hazards": [], "candidates": []}
        objects = self._objects if objects_fresh else {"ok": False, "reason": "objects_stale", "tracks": [], "riskyTracks": []}
        recommendations: list[dict[str, Any]] = []
        if objects.get("stopRecommended"):
            recommendations.append({"action": "stop", "reason": "moving_object", "source": "lidar"})
        for hazard in ground.get("hazards", []):
            recommendations.append({"action": "inspect", "source": "kinect", **hazard})
        if ground.get("status") != "ok":
            recommendations.append({"action": "stop", "reason": "ground_unknown", "source": "kinect"})
        if not objects.get("ok"):
            recommendations.append({"action": "stop", "reason": "objects_unknown", "source": "lidar"})
        return {"ok": True, "ground": ground, "objects": objects,
                "groundAgeS": None if ground_age is None else round(ground_age, 3),
                "objectsAgeS": None if objects_age is None else round(objects_age, 3),
                "groundFresh": ground_fresh, "objectsFresh": objects_fresh,
                "recommendations": recommendations, "tractionConnected": False}

    def reset(self) -> None:
        self.tracker.reset()
        self.ground_tracker.reset()
        self._ground = {"status": "unknown", "reason": "no_frame", "hazards": [], "candidates": []}
        self._objects = {"ok": False, "reason": "no_scan", "tracks": [], "riskyTracks": []}
        self._ground_at = None
        self._objects_at = None
        self._ground_sequence = 0
