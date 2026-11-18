"""Офлайн-псевдоразметка. Это предложения рамок, НЕ эталон ground truth.

В отличие от бортовой эвристики ищет также красные STOP и весь корпус
трёхсекционного светофора. Цвета не синтезируются/не перекрашиваются.
Результаты нужно просмотреть и исправить через --annotations.
"""
from __future__ import annotations

import cv2
import numpy as np

from robot_control.vision.detector import Detection, HeuristicDetector, non_max_suppression
from robot_control.vision.traffic import classify_traffic_light


def propose(frame: np.ndarray, min_confidence: float = 0.5) -> list[Detection]:
    h, w = frame.shape[:2]
    scale = min(1.0, 960 / max(h, w))
    image = cv2.resize(frame, (round(w * scale), round(h * scale))) if scale < 1 else frame
    sh, sw = image.shape[:2]
    area = float(sh * sw)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, sat, val = cv2.split(hsv)
    teacher = HeuristicDetector()
    found: list[Detection] = []

    def contours(mask):
        return cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    def add(cls, x, y, bw, bh, confidence):
        found.append(Detection(cls, (x / sw, y / sh, bw / sw, bh / sh), confidence))

    # STOP: красная/пурпурная выпуклая табличка, не тонкий красный столбик.
    mask = (((hue <= 12) | (hue >= 145)) & (sat > 85) & (val > 65)).astype('uint8') * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    for contour in contours(mask):
        x, y, bw, bh = cv2.boundingRect(contour)
        a = cv2.contourArea(contour)
        if not area * 0.00035 < a < area * 0.4 or not 0.43 < bw / bh < 1.85:
            continue
        polygon = cv2.approxPolyDP(contour, 0.025 * cv2.arcLength(contour, True), True)
        vertices = len(polygon)
        patch = hsv[y:y + bh, x:x + bw]
        white = (patch[..., 1] < 85) & (patch[..., 2] > 150)
        if (min(bw, bh) >= sw * 0.025 and 0.6 < a / (bw * bh) < 0.96 and
                7 <= vertices <= 9 and cv2.isContourConvex(polygon) and white.mean() > 0.045):
            add('stop_sign', x, y, bw, bh, 0.75)

    mask = cv2.inRange(hsv, (95, 70, 55), (132, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    for contour in contours(mask):
        x, y, bw, bh = cv2.boundingRect(contour)
        a = cv2.contourArea(contour)
        if not area * 0.0003 < a < area * 0.3 or not 0.4 < bw / bh < 1.9:
            continue
        if a / (bw * bh) < 0.55:
            continue
        cls, confidence = teacher._classify_blue(image, contour)
        if confidence >= min_confidence:
            add(cls, x, y, bw, bh, confidence)

    # Склеиваем только выровненные красно-оранжевые полосы (не швы пола).
    mask = (((hue <= 24) | (hue >= 173)) & (sat > 130) & (val > 85)).astype('uint8') * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    pieces = []
    for contour in contours(mask):
        x, y, bw, bh = cv2.boundingRect(contour)
        if cv2.contourArea(contour) > area * 0.00008 and sw * 0.006 < bw < sw * 0.15:
            pieces.append([x, y, x + bw, y + bh, 1])
    changed = True
    while changed:
        changed = False
        for i, a in enumerate(pieces):
            for j in range(i):
                b = pieces[j]
                width = min(a[2] - a[0], b[2] - b[0])
                overlap = min(a[2], b[2]) - max(a[0], b[0])
                gap = max(a[1], b[1]) - min(a[3], b[3])
                if overlap > width * 0.55 and gap < width * 2.5:
                    pieces[j] = [min(a[0], b[0]), min(a[1], b[1]), max(a[2], b[2]),
                                 max(a[3], b[3]), a[4] + b[4]]
                    pieces.pop(i)
                    changed = True
                    break
            if changed:
                break
    for x, y, x2, y2, number in pieces:
        bw, bh = x2 - x, y2 - y
        if bh > sh * 0.05 and 2 < bh / bw < 25:
            if number >= 2 or bh / bw > 3:
                add('cone', x, y, bw, bh, 0.65)

    # Морфологическое открытие убирает тонкий штатив из рамки корпуса.
    for threshold in (85, 115, 145, 175):
        dark = (val < threshold).astype('uint8') * 255
        dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((3, 5), np.uint8))
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        for contour in contours(dark):
            x, y, bw, bh = cv2.boundingRect(contour)
            a = cv2.contourArea(contour)
            if not area * 0.001 < a < area * 0.55 or not 1.5 < bh / bw < 4.2:
                continue
            if a / (bw * bh) < 0.72:
                continue
            roi = image[y:y + bh, x:x + bw]
            signal = classify_traffic_light(roi)
            # Не помечаем дверь/спинку стула как светофор без светящихся диодов.
            if signal['pixels'] < 8 or signal['pixels'] > bw * bh * 0.2:
                continue
            hh, ss, vv = cv2.split(hsv[y:y + bh, x:x + bw])
            lamps = ((vv >= 185) & (ss >= 100) & ((hh <= 88) | (hh >= 173))).astype('uint8')
            components, _, stats, _ = cv2.connectedComponentsWithStats(lamps, 8)
            sizes = [stats[i, cv2.CC_STAT_AREA] for i in range(1, components)]
            small = [size for size in sizes if 1 <= size <= max(12, bw * bh * 0.004)]
            dots = len(small)
            if sum(small) < lamps.sum() * 0.7:
                continue
            ys, xs = np.nonzero(lamps)
            central = (xs > bw * 0.15) & (xs < bw * 0.85)
            # Диоды должны находиться ВНУТРИ корпуса, не на соседнем конусе.
            if (dots >= 8 and central.mean() > 0.90 and
                    np.ptp(xs) > bw * 0.25 and np.ptp(ys) < bh * 0.92):
                add('traffic_light', x, y, bw, bh, 0.65 + min(0.2, bw * bh / area))
    cones = [d for d in found if d.cls == 'cone']
    found = [d for d in found if d.cls != 'stop_sign' or not any(
        c.box[0] <= d.box[0] + d.box[2] / 2 <= c.box[0] + c.box[2] and
        c.box[1] <= d.box[1] + d.box[3] / 2 <= c.box[1] + c.box[3] and
        d.box[3] < c.box[3] * 0.8 for c in cones)]
    kept = non_max_suppression(found, 0.35)
    output = []
    for det in kept:
        if det.cls != 'traffic_light':
            output.append(det)
            continue
        x, y, bw, bh = det.box
        roi = frame[int(y * h):int((y + bh) * h), int(x * w):int((x + bw) * w)]
        signal = classify_traffic_light(roi)
        counts = signal['counts']
        total = sum(counts.values())
        ranked = sorted(counts.values(), reverse=True)
        # Два/три одновременно горящих сигнала — generic, а не ложный green.
        state = signal['state'] if total and ranked[0] / total >= 0.8 and ranked[1] / total < 0.15 else 'unknown'
        output.append(Detection(det.cls, det.box, det.confidence, state, counts, 'bootstrap'))
    return sorted(output, key=lambda d: (d.cls, d.box))
