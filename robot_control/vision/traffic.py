"""Цвет сигнализатора светофора: красный / жёлтый / зелёный.

Классификатор считает яркие насыщенные пиксели внутри рамки светофора.
Он одинаково пригоден для классической трёхлинзовой головки и для
светодиодного табло с полигона: важен цвет самих диодов, а не форма корпуса.
HSV считается на numpy (без OpenCV), поэтому логику можно проверять в тестах
и на машинах без cv2. Диапазоны тонов — в градусах OpenCV (0…179):

* красный 0…12 и 173…179;
* жёлтый/оранжевый 13…38;
* зелёный 39…88 (гон 90…130 — синий экран-баннер, не сигнал).

Пурпурный баннер и зелёная стена полигона в эти маски почти не попадают:
у баннера тон 155…170, у стены яркость ниже порога диодов.
"""
from __future__ import annotations

import math
import time
from collections import Counter, deque
from typing import Any, Optional

import numpy as np

from .classes import TRAFFIC_LIGHT_LABELS, TRAFFIC_LIGHT_STATES

UNKNOWN = "unknown"

# Порог «это светящийся диод, а не фон».
MIN_VALUE = 185
MIN_SATURATION = 100
MIN_PIXELS = 8


def _bgr_to_hsv(frame: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Векторизованный BGR→HSV в шкале OpenCV (H 0…179, S/V 0…255)."""
    rgb = np.asarray(frame, dtype=np.float32) / 255.0
    b, g, r = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    mx = np.maximum(np.maximum(r, g), b)
    mn = np.minimum(np.minimum(r, g), b)
    diff = mx - mn
    hue = np.zeros_like(mx, dtype=np.float32)
    nonzero = diff > 1e-7
    r_max = nonzero & (mx == r)
    g_max = nonzero & (mx == g) & ~r_max
    b_max = nonzero & (mx == b) & ~r_max & ~g_max
    hue[r_max] = ((g - b)[r_max] / diff[r_max]) % 6.0
    hue[g_max] = (b - r)[g_max] / diff[g_max] + 2.0
    hue[b_max] = (r - g)[b_max] / diff[b_max] + 4.0
    h = hue * 30.0  # 60°/6 · 180/360 → 30
    s = np.where(mx > 1e-7, diff / np.maximum(mx, 1e-7), 0.0) * 255.0
    v = mx * 255.0
    return h, s, v


def color_counts(frame: np.ndarray) -> dict[str, int]:
    """Сколько пикселей каждого сигнального цвета в кадре/рамке."""
    arr = np.asarray(frame)
    if arr.ndim != 3 or arr.shape[-1] < 3 or not arr.size:
        return {state: 0 for state in TRAFFIC_LIGHT_STATES}
    h, s, v = _bgr_to_hsv(arr[..., :3])
    lamp = (v >= MIN_VALUE) & (s >= MIN_SATURATION)
    red = lamp & ((h <= 12.0) | (h >= 173.0))
    yellow = lamp & (h > 12.0) & (h <= 38.0)
    green = lamp & (h > 38.0) & (h <= 88.0)
    return {
        "red": int(np.count_nonzero(red)),
        "yellow": int(np.count_nonzero(yellow)),
        "green": int(np.count_nonzero(green)),
    }


def classify_traffic_light(roi_bgr: Any, *, prefer_red: bool = False) -> dict[str, Any]:
    """Определить текущий сигнал по рамке светофора.

    Возвращает ``state``, ``confidence`` (доля победившего цвета), ``counts`` и
    ``pixels``. При ``prefer_red=True`` даже небольшое число красных диодов
    даёт «красный» — безопасный режим для реальной езды.
    """
    counts = color_counts(roi_bgr)
    total = sum(counts.values())
    result: dict[str, Any] = {"state": UNKNOWN, "confidence": 0.0,
                              "counts": counts, "pixels": total}
    if total < MIN_PIXELS:
        return result
    dominant = max(TRAFFIC_LIGHT_STATES, key=lambda name: counts[name])
    if prefer_red and counts["red"] >= max(MIN_PIXELS, int(total * 0.12)):
        dominant = "red"
    if counts[dominant] < max(MIN_PIXELS, int(total * 0.30)):
        return result
    result["state"] = dominant
    result["confidence"] = round(counts[dominant] / total, 3)
    return result


class TrafficLightTracker:
    """Сглаживает мигание диодов: большинство за окно + гистерезис переключения.

    Светодиодная матрица обновляется построчно, поэтому одиночный кадр легко
    даёт ложный «зелёный». Решение меняется только после ``switch_votes``
    последовательных кадров с новым большинством.
    """

    def __init__(self, *, window: int = 7, switch_votes: int = 2,
                 stale_s: float = 2.0) -> None:
        self.window = max(1, int(window))
        self.switch_votes = max(1, int(switch_votes))
        self.stale_s = float(stale_s)
        self._votes: deque[tuple[float, str, float, int]] = deque(maxlen=self.window)
        self._state = UNKNOWN
        self._confidence = 0.0
        self._camera_id: Optional[int] = None
        self._candidate = UNKNOWN
        self._candidate_hits = 0
        self._updated_at: Optional[float] = None
        self._counts: dict[str, int] = {name: 0 for name in TRAFFIC_LIGHT_STATES}

    def set_counts(self, counts: dict[str, int]) -> None:
        """Суммарное число пикселей каждого цвета по всем камерам (для ламп UI)."""
        self._counts = {name: max(0, int(counts.get(name, 0))) for name in TRAFFIC_LIGHT_STATES}

    def update(self, state: str, confidence: float = 0.0, camera_id: int = 0,
               *, stamp: Optional[float] = None) -> dict[str, Any]:
        now = time.time() if stamp is None else float(stamp)
        if state not in TRAFFIC_LIGHT_STATES:
            state = UNKNOWN
        self._votes.append((now, state, max(0.0, min(1.0, float(confidence))), int(camera_id)))
        tally: Counter[str] = Counter()
        conf: dict[str, float] = {}
        cams: Counter[int] = Counter()
        last_seen: dict[str, float] = {}
        for stamp_value, vote, weight, cam in self._votes:
            if vote == UNKNOWN:
                continue
            tally[vote] += weight if weight > 0 else 0.25
            conf[vote] = max(conf.get(vote, 0.0), weight)
            cams[cam] += 1
            last_seen[vote] = stamp_value
        # При равенстве голосов берём более свежий цвет, а не тот, что
        # случайно оказался первым в Counter.
        winner = max(tally, key=lambda name: (tally[name], last_seen.get(name, 0.0))) \
            if tally else UNKNOWN
        if winner == self._state:
            self._candidate = UNKNOWN
            self._candidate_hits = 0
        elif winner == self._candidate:
            self._candidate_hits += 1
        else:
            self._candidate, self._candidate_hits = winner, 1
        if winner != self._state and (self._state == UNKNOWN or self._candidate_hits >= self.switch_votes):
            self._state = winner
            self._candidate, self._candidate_hits = UNKNOWN, 0
        self._confidence = round(conf.get(self._state, 0.0), 3)
        self._camera_id = cams.most_common(1)[0][0] if self._state != UNKNOWN and cams else None
        self._updated_at = now
        return self.snapshot(now=now)

    def snapshot(self, *, now: Optional[float] = None) -> dict[str, Any]:
        now = time.time() if now is None else float(now)
        age = None if self._updated_at is None else max(0.0, now - self._updated_at)
        fresh = age is not None and age <= self.stale_s
        state = self._state if fresh else UNKNOWN
        return {
            "state": state,
            "label": TRAFFIC_LIGHT_LABELS.get(state, TRAFFIC_LIGHT_LABELS[UNKNOWN]),
            "confidence": self._confidence if state != UNKNOWN else 0.0,
            "cameraId": self._camera_id if state != UNKNOWN else None,
            "ageS": None if age is None else round(age, 3),
            "fresh": fresh,
            "candidate": self._candidate,
            "counts": dict(self._counts) if state != UNKNOWN else {name: 0 for name in TRAFFIC_LIGHT_STATES},
            "votes": [vote[1] for vote in self._votes],
            "action": {"red": "stop", "yellow": "prepare", "green": "go"}.get(state, "wait"),
        }

    def reset(self) -> None:
        self._votes.clear()
        self._state = UNKNOWN
        self._confidence = 0.0
        self._camera_id = None
        self._candidate = UNKNOWN
        self._candidate_hits = 0
        self._updated_at = None
        self._counts = {name: 0 for name in TRAFFIC_LIGHT_STATES}


def hue_deg(rgb_fraction: tuple[float, float, float]) -> float:
    """Небольшая помощь для тестов/калибровки: тон RGB в градусах OpenCV."""
    arr = np.array([[rgb_fraction[::-1]]], dtype=np.float32)
    h, _, _ = _bgr_to_hsv(arr)
    value = float(h[0, 0])
    return value if math.isfinite(value) else 0.0
