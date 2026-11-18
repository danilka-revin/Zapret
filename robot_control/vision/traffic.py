"""Цвет сигнализатора светофора: красный / жёлтый / зелёный.

Классификатор считает яркие насыщенные пиксели внутри рамки светофора.
Он одинаково пригоден для классической трёхлинзовой головки и для
светодиодного табло с полигона: важен цвет самих диодов, а не форма корпуса.
HSV считается на numpy (без OpenCV), поэтому логику можно проверять в тестах
и на машинах без cv2. Диапазоны тонов — в градусах OpenCV (0…179):

* красный 0…12 и 173…179;
* жёлтый/оранжевый 13…38;
* зелёный 39…88 (гон 90…130 — синий экран-баннер, не сигнал).

Эти цветовые маски сами по себе не доказывают наличие светофора: яркая
окрашенная поверхность может дать ложный сигнал. Опциональная проверка
verify_traffic_light дополнительно проверяет корпус и положение секции,
а YOLO всё равно должен независимо подтвердить green.
"""
from __future__ import annotations

import math
import time
from collections import Counter, deque
from typing import TYPE_CHECKING, Any, Optional

if TYPE_CHECKING:
    import numpy as np

from .classes import TRAFFIC_LIGHT_LABELS, TRAFFIC_LIGHT_STATES

UNKNOWN = "unknown"

# Порог «это светящийся диод, а не фон».
MIN_VALUE = 185
MIN_SATURATION = 100
MIN_PIXELS = 8


def _bgr_to_hsv(frame: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Векторизованный BGR→HSV в шкале OpenCV (H 0…179, S/V 0…255)."""
    import numpy as np
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
    import numpy as np
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
    даёт «красный». Это приоритет цвета, а не гарантия безопасности движения.
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
    else:
        ranked = sorted(counts.values(), reverse=True)
        # Несколько одновременно горящих секций не означают «можно ехать».
        if ranked[0] / total < 0.65 or ranked[1] / total > 0.25:
            return result
    if counts[dominant] < MIN_PIXELS:
        return result
    result["state"] = dominant
    result["confidence"] = round(counts[dominant] / total, 3)
    return result


def verify_traffic_light(roi_bgr: Any, *, prefer_red: bool = False) -> dict[str, Any]:
    """Opt-in geometric/photometric evidence inside a candidate light box.

    Rejects wide/bright surfaces and requires emission in the appropriate
    third of a dark housing. This evidence alone must NOT grant green:
    YoloDetector also requires its learned green class. A dark/ambiguous
    panel stays unknown. These are bench heuristics, not a safety
    certification; the legacy production mode is unchanged.
    """
    import numpy as np
    arr = np.asarray(roi_bgr)
    result: dict[str, Any] = {"validBody": False, "state": UNKNOWN,
                              "confidence": 0.0, "counts": {s: 0 for s in TRAFFIC_LIGHT_STATES},
                              "pixels": 0, "reason": "invalid-roi"}
    if arr.ndim != 3 or arr.shape[-1] < 3 or arr.size == 0:
        return result
    height, width = arr.shape[:2]
    if height < 12 or width < 5 or not 1.6 <= height / width <= 5.5:
        result['reason'] = 'body-aspect'
        return result
    hue, saturation, value = _bgr_to_hsv(arr[..., :3])
    dark_fraction = float(np.mean(value < 140))
    result['darkFraction'] = round(dark_fraction, 4)
    if dark_fraction < .4:
        result['reason'] = 'no-dark-housing'
        return result
    result['validBody'] = True
    # Dark housings are exposed differently from frame to frame. Require
    # an absolute floor AND contrast over the local background, not V=185
    # for every camera. Low-saturation red LEDs can be almost pink/white.
    threshold = max(130.0, float(np.percentile(value, 30)) + 65.0)
    bright = value >= threshold
    masks = {
        'red': bright & (saturation >= 40) & ((hue <= 12) | (hue >= 173)),
        'yellow': bright & (saturation >= 65) & (hue > 12) & (hue <= 38),
        # Green never gets the relaxed red/yellow exposure floor.
        'green': bright & (value >= MIN_VALUE) & (saturation >= MIN_SATURATION) & (hue > 38) & (hue <= 88),
    }
    counts = {state: int(np.count_nonzero(mask)) for state, mask in masks.items()}
    total = sum(counts.values())
    result.update(counts=counts, pixels=total)
    minimum = max(5, int(math.ceil(height * width * .0002)))
    if total < minimum:
        result['reason'] = 'no-emission'
        return result
    if total / (height * width) > .3:
        result['reason'] = 'solid-coloured-surface'
        return result
    ranges = {'red': (0., .40), 'yellow': (.27, .73), 'green': (.60, 1.)}
    valid: dict[str, bool] = {}
    for state, mask in masks.items():
        ys, xs = np.nonzero(mask)
        lo, hi = ranges[state]
        spatial = ((ys / height >= lo) & (ys / height <= hi) &
                   (xs / width >= .08) & (xs / width <= .92))
        valid[state] = len(ys) >= minimum and float(spatial.mean()) >= .72
    dominant = max(TRAFFIC_LIGHT_STATES, key=lambda state: counts[state])
    if prefer_red and valid['red'] and counts['red'] >= max(minimum, total * .12):
        dominant = 'red'
    else:
        ranked = sorted(counts.values(), reverse=True)
        # Multiple illuminated sections and uncertain hue never permit green.
        if ranked[0] / total < .75 or ranked[1] / total > .15:
            result['reason'] = 'ambiguous-emission'
            return result
    if not valid[dominant]:
        result['reason'] = 'wrong-section-or-background'
        return result
    result.update(state=dominant, confidence=round(counts[dominant]/total, 3), reason='verified-section')
    return result


def fuse_signal(model_state: str | None, pixel_state: str, *,
                colour_model: bool) -> str:
    """Fail-closed согласование цвета YOLO и пикселей.

    Старые 5-классовые модели работают как раньше (ROI → HSV). У новых
    зелёный разрешён только при СОГЛАСИИ нейросети и HSV; generic/off,
    отсутствие пикселей или конфликт не превращаются в разрешение движения.
    Красный и жёлтый имеют консервативный приоритет, но не управляют моторами.
    """
    if not colour_model:
        return pixel_state
    if "red" in (model_state, pixel_state):
        return "red"
    if "yellow" in (model_state, pixel_state):
        return "yellow"
    if model_state == pixel_state == "green":
        return "green"
    return UNKNOWN


class TrafficLightTracker:
    """Сглаживает мигание диодов: большинство за окно + гистерезис переключения.

    Светодиодная матрица обновляется построчно, поэтому одиночный кадр легко
    даёт ложный «зелёный». Решение меняется только после ``switch_votes``
    последовательных кадров с новым большинством.
    """

    def __init__(self, *, window: int = 7, switch_votes: int = 2,
                 stale_s: float = 2.0, prefer_red: bool = False) -> None:
        self.prefer_red = bool(prefer_red)
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
               *, stamp: Optional[float] = None,
               evidence_stamp: Optional[float] = None) -> dict[str, Any]:
        now = time.time() if stamp is None else float(stamp)
        if state not in TRAFFIC_LIGHT_STATES:
            state = UNKNOWN
        # Старые голоса не участвуют, даже если новые кадры продолжают
        # приходить с unknown. Иначе однажды увиденный green жил бесконечно.
        self._votes = deque((vote for vote in self._votes if 0 <= now - vote[0] <= self.stale_s),
                            maxlen=self.window)
        observed = now if evidence_stamp is None else float(evidence_stamp)
        # Обработка другой камеры не делает старый кадр новым голосом и не
        # освежает его timestamp. Будущие/просроченные кадры тоже не голосуют.
        if not 0 <= now - observed <= self.stale_s:
            return self.snapshot(now=now)
        if evidence_stamp is not None and any(vote[0] == observed and vote[3] == camera_id
                                              for vote in self._votes):
            return self.snapshot(now=now)
        self._votes.append((observed, state, max(0.0, min(1.0, float(confidence))), int(camera_id)))
        tally: Counter[str] = Counter()
        conf: dict[str, float] = {}
        last_seen: dict[str, float] = {}
        for stamp_value, vote, weight, cam in self._votes:
            if vote == UNKNOWN:
                continue
            tally[vote] += weight if weight > 0 else 0.25
            conf[vote] = max(conf.get(vote, 0.0), weight)
            last_seen[vote] = max(last_seen.get(vote, stamp_value), stamp_value)
        # При равенстве голосов берём более свежий цвет, а не тот, что
        # случайно оказался первым в Counter.
        winner = max(tally, key=lambda name: (tally[name], last_seen.get(name, 0.0))) \
            if tally else UNKNOWN
        red_override = self.prefer_red and 'red' in tally
        if red_override:
            winner = 'red'  # свежий red не ждёт, пока истечёт очередь green
        if winner == self._state:
            self._candidate = UNKNOWN
            self._candidate_hits = 0
        elif winner == self._candidate:
            self._candidate_hits += 1
        else:
            self._candidate, self._candidate_hits = winner, 1
        if winner != self._state and (red_override or self._state == UNKNOWN or self._candidate_hits >= self.switch_votes):
            self._state = winner
            self._candidate, self._candidate_hits = UNKNOWN, 0
        if self._state not in last_seen:
            self._state = UNKNOWN
        self._confidence = round(conf.get(self._state, 0.0), 3)
        state_cameras = Counter(cam for _, vote, _, cam in self._votes if vote == self._state)
        self._camera_id = state_cameras.most_common(1)[0][0] if self._state != UNKNOWN and state_cameras else None
        # Свежесть — время реального свидетельства о текущем цвете, не
        # время очередного update(unknown) или старого кадра другой камеры.
        self._updated_at = last_seen.get(self._state)
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
    import numpy as np
    arr = np.array([[rgb_fraction[::-1]]], dtype=np.float32)
    h, _, _ = _bgr_to_hsv(arr)
    value = float(h[0, 0])
    return value if math.isfinite(value) else 0.0
