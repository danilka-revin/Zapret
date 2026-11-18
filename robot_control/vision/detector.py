"""Детектор знаков: YOLO-совместимый формат + встроенный резерв по цвету/форме.

``HeuristicDetector`` нужен не «для галочки»: он

1. служит основой предложений разметки; актуальный расширенный датасет
   собирают ``scripts/vision_dataset.py`` и ``scripts/vision_bootstrap.py``;
2. работает на роботе прямо сейчас, даже если PyTorch/веса YOLO не установлены;
3. выдаёт ровно тот же список ``Detection``, что и обученная модель.

Кадр можно подавать любого размера: рамки возвращаются нормированными в 0…1,
поэтому масштабирование камеры не рассинхронизирует экран и поле карты.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable, Optional

import numpy as np

try:  # OpenCV уже используется camera.py; отсутствие — не падение импорта.
    import cv2
except ImportError:  # pragma: no cover - на роботе без cv2 сервис честно выключен
    cv2 = None  # type: ignore[assignment]

from .classes import CLASS_INFO, CLASS_NAMES, class_id, normalize_class, signal_from_class
from .traffic import classify_traffic_light, fuse_signal, verify_traffic_light


def cv2_available() -> bool:
    """Есть ли OpenCV в текущем процессе."""
    return cv2 is not None


@dataclass(frozen=True)
class Detection:
    """Одна рамка. ``box`` = (x, y, w, h) в долях ширины/высоты кадра."""

    cls: str
    box: tuple[float, float, float, float]
    confidence: float = 0.6
    color: Optional[str] = None
    color_counts: dict[str, int] = field(default_factory=dict)
    source: str = "heuristic"

    def __post_init__(self) -> None:
        if self.cls not in CLASS_NAMES:
            raise ValueError(f"неизвестный класс: {self.cls!r}")
        x, y, w, h = self.box
        if not all(math.isfinite(v) for v in (x, y, w, h)) or w <= 0 or h <= 0:
            raise ValueError(f"некорректная рамка: {self.box!r}")
        x, y = min(1.0, max(0.0, x)), min(1.0, max(0.0, y))
        w, h = min(1.0 - x, max(0.0, w)), min(1.0 - y, max(0.0, h))
        object.__setattr__(self, "box", tuple(round(v, 6) for v in (x, y, w, h)))
        object.__setattr__(self, "confidence", min(1.0, max(0.0, float(self.confidence))))

    def to_dict(self) -> dict[str, Any]:
        info = CLASS_INFO[self.cls]
        x, y, w, h = self.box
        return {
            "class": self.cls,
            "label": info["label"],
            "short": info["short"],
            "action": info["action"],
            "color": info["color"],
            "confidence": round(self.confidence, 3),
            "box": {"x": round(x, 4), "y": round(y, 4),
                    "w": round(w, 4), "h": round(h, 4)},
            "signal": self.color,
            "signalCounts": dict(self.color_counts),
            "source": self.source,
        }

    def yolo_line(self) -> str:
        """Строка разметки YOLO: ``cls cx cy w h``."""
        x, y, w, h = self.box
        return f"{class_id(self.cls)} {x + w / 2:.6f} {y + h / 2:.6f} {w:.6f} {h:.6f}"


def iou(a: Detection, b: Detection) -> float:
    """Пересечение / объединение двух нормированных рамок."""
    ax, ay, aw, ah = a.box
    bx, by, bw, bh = b.box
    x1, y1 = max(ax, bx), max(ay, by)
    x2, y2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


def non_max_suppression(items: Iterable[Detection], threshold: float = 0.45, *,
                        conservative_signals: bool = False) -> list[Detection]:
    """Подавить дубли одного класса; конфликтующие сигналы не выбирают green.

    YOLO делает NMS по восьми исходным классам. После свёртки в публичные
    пять один корпус может иметь и red, и более уверенный green. Для новой
    цветовой модели red/yellow/unknown должны пережить такой конфликт.
    """
    def priority(det: Detection) -> tuple[int, float]:
        signal = {"red": 3, "yellow": 2, "unknown": 1, "green": 0}
        rank = signal.get(det.color or "unknown", 1) if conservative_signals and det.cls == "traffic_light" else 0
        return rank, det.confidence

    kept: list[Detection] = []
    for candidate in sorted(items, key=priority, reverse=True):
        if any(candidate.cls == other.cls and iou(candidate, other) > threshold for other in kept):
            continue
        kept.append(candidate)
    return kept


class HeuristicDetector:
    """Цвет + геометрия: СТОП, два синих знака, табло светофора, конусы.

    Диапазоны HSV подобраны по фотографиям полигона (зелёные стены, пурпурный
    баннер и оранжевые столбики). Синий квадрат дополнительно перспективно
    выпрямляется и классифицируется по чёрному символу внутри белого
    треугольника: у «пешехода» тёмные пиксели поднимаются выше центра, у
    «неровности» это низкий широкий холмик.
    """

    def __init__(self, *, max_side: int = 720, min_confidence: float = 0.5,
                 prefer_red: bool = False) -> None:
        self.max_side = max(240, int(max_side))
        self.min_confidence = float(min_confidence)
        self.prefer_red = bool(prefer_red)

    def set_confidence(self, value: float) -> None:
        self.min_confidence = max(0.5, min(0.95, float(value)))

    # -------------------------------------------------------------- helpers
    @staticmethod
    def _contours(mask: np.ndarray):
        found = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return found[-2]

    @staticmethod
    def _box(x: float, y: float, w: float, h: float, sw: int, sh: int) -> tuple[float, float, float, float]:
        x1, y1 = max(0.0, x) / sw, max(0.0, y) / sh
        x2, y2 = min(sw, x + w) / sw, min(sh, y + h) / sh
        return x1, y1, max(0.0, x2 - x1), max(0.0, y2 - y1)

    @staticmethod
    def _merge_traffic_lights(detections: list[Detection]) -> list[Detection]:
        """Склеить секции одного табло: красная/жёлтая/зелёная зоны — один знак."""
        lights = [d for d in detections if d.cls == "traffic_light"]
        others = [d for d in detections if d.cls != "traffic_light"]
        if len(lights) < 2:
            return detections
        clusters: list[list[Detection]] = []
        for det in lights:
            cx, cy = det.box[0] + det.box[2] / 2, det.box[1] + det.box[3] / 2
            target = None
            for cluster in clusters:
                if any(iou(det, other) > 0.05 or
                       (abs(cx - (other.box[0] + other.box[2] / 2)) < 0.16 and
                        abs(cy - (other.box[1] + other.box[3] / 2)) < 0.30)
                       for other in cluster):
                    target = cluster
                    break
            if target is None:
                clusters.append([det])
            else:
                target.append(det)
        for cluster in clusters:
            x1 = min(d.box[0] for d in cluster)
            y1 = min(d.box[1] for d in cluster)
            x2 = max(d.box[0] + d.box[2] for d in cluster)
            y2 = max(d.box[1] + d.box[3] for d in cluster)
            others.append(Detection("traffic_light", (x1, y1, x2 - x1, y2 - y1),
                                    max(d.confidence for d in cluster)))
        return others

    @staticmethod
    def _order_quad(points: np.ndarray) -> np.ndarray:
        """Упорядочить 4 вершины: верх-лево, верх-право, низ-право, низ-лево."""
        pts = points.reshape(-1, 2).astype(np.float32)
        order = np.argsort(pts[:, 0] + pts[:, 1])
        tl, br = pts[order[0]], pts[order[-1]]
        rest = np.array([p for p in pts if not (np.array_equal(p, tl) or np.array_equal(p, br))],
                        dtype=np.float32)
        if rest.shape[0] != 2:
            rect = cv2.minAreaRect(points.astype(np.float32))
            return cv2.boxPoints(rect)
        tr, bl = (rest[0], rest[1]) if rest[0][1] < rest[1][1] else (rest[1], rest[0])
        return np.array([tl, tr, br, bl], dtype=np.float32)

    def _classify_blue(self, frame: np.ndarray, contour: np.ndarray) -> tuple[str, float]:
        """Вернуть (класс, уверенность) для синего квадрата с белым треугольником."""
        peri = cv2.arcLength(contour, True)
        quad = cv2.approxPolyDP(contour, 0.045 * peri, True)
        if len(quad) != 4:
            rect = cv2.minAreaRect(contour)
            quad = cv2.boxPoints(rect).reshape(-1, 1, 2).astype(np.float32)
        src = self._order_quad(quad)
        dst = np.array([[0, 0], [99, 0], [99, 99], [0, 99]], dtype=np.float32)
        try:
            matrix = cv2.getPerspectiveTransform(src, dst)
            patch = cv2.warpPerspective(frame, matrix, (100, 100))
        except cv2.error:
            return "pedestrian_crossing", 0.5
        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
        hue, sat, val = cv2.split(hsv)
        triangle = np.zeros((100, 100), dtype=np.uint8)
        cv2.fillConvexPoly(triangle, np.array([[50, 18], [83, 82], [17, 82]], dtype=np.int32), 1)
        tri = triangle.astype(bool)
        # У настоящего знака внутри треугольника белое поле, а не синий фон:
        # так отсекаются голубые участки большого LED-экрана и баннера.
        blue_inside = float(np.count_nonzero((hue >= 95) & (hue <= 132) & (sat > 60) & tri)) / tri.sum()
        if blue_inside > 0.45:
            return "pedestrian_crossing", 0.2
        dark = ((val < 115) & tri).astype(np.uint8)
        ys, xs = np.nonzero(dark)
        total = int(dark.sum())
        if total < 25:
            return "pedestrian_crossing", 0.2
        upper = float(np.count_nonzero(ys < 52)) / total
        centroid = float(ys.mean()) / 100.0
        width = float(xs.max() - xs.min()) / 100.0
        # Пешеход: «голова» и корпус тянутся в верхнюю половину треугольника.
        # Неровность: плотный холмик внизу, почти нет тёмного выше центра.
        if upper > 0.24 or centroid < 0.63:
            score = 0.55 + min(0.35, upper * 0.7 + (0.7 - centroid) * 0.3)
            return "pedestrian_crossing", score
        score = 0.58 + min(0.32, (centroid - 0.6) * 0.6 + width * 0.15)
        return "speed_bump", score

    # --------------------------------------------------------------- detect
    def detect(self, frame_bgr: Any) -> list[Detection]:
        """Найти знаки на BGR-кадре. Без OpenCV возвращает пустой список."""
        if cv2 is None:
            return []
        frame = np.asarray(frame_bgr)
        if frame.ndim != 3 or frame.shape[2] < 3 or frame.size == 0:
            return []
        h, w = frame.shape[:2]
        scale = min(1.0, self.max_side / max(h, w))
        small = cv2.resize(frame[..., :3], (max(1, int(w * scale)), max(1, int(h * scale))),
                           interpolation=cv2.INTER_AREA) if scale < 1 else frame[..., :3].copy()
        sh, sw = small.shape[:2]
        area = float(sh * sw)
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        hue, sat, val = cv2.split(hsv)
        found: list[Detection] = []

        # ---- STOP: пурпурный восьмиугольник ---------------------------------
        magenta = cv2.inRange(hsv, np.array((150, 70, 60), np.uint8),
                              np.array((179, 255, 255), np.uint8))
        magenta = cv2.morphologyEx(magenta, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        for contour in self._contours(magenta):
            a = cv2.contourArea(contour)
            if a < area * 0.0015 or a > area * 0.22:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            ratio = bw / max(1, bh)
            if not 0.55 <= ratio <= 1.8 or a / (bw * bh) < 0.55:
                continue
            peri = cv2.arcLength(contour, True)
            vertices = len(cv2.approxPolyDP(contour, 0.035 * peri, True))
            if vertices < 6 or vertices > 12:
                continue
            conf = 0.62 + min(0.3, abs(8 - vertices) * -0.04 + a / (bw * bh) * 0.3)
            found.append(Detection("stop_sign", self._box(x, y, bw, bh, sw, sh), conf))

        # ---- синие знаки: квадрат + белый треугольник ------------------------
        blue = cv2.inRange(hsv, np.array((95, 70, 55), np.uint8),
                           np.array((132, 255, 255), np.uint8))
        blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        for contour in self._contours(blue):
            a = cv2.contourArea(contour)
            if a < area * 0.0025 or a > area * 0.28:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            ratio = bw / max(1, bh)
            if not 0.55 <= ratio <= 1.8 or a / (bw * bh) < 0.62:
                continue
            cls, conf = self._classify_blue(small, contour)
            found.append(Detection(cls, self._box(x, y, bw, bh, sw, sh), conf))

        # ---- конусы / оградительные столбики --------------------------------
        orange = cv2.inRange(hsv, np.array((3, 110, 110), np.uint8),
                             np.array((28, 255, 255), np.uint8))
        orange = cv2.morphologyEx(orange, cv2.MORPH_CLOSE, np.ones((3, 11), np.uint8))
        for contour in self._contours(orange):
            a = cv2.contourArea(contour)
            if a < area * 0.0008 or a > area * 0.12:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            if bh < sw * 0.06 or bh / max(1, bw) < 1.5 or a / (bw * bh) < 0.32:
                continue
            found.append(Detection("cone", self._box(x, y, bw, bh, sw, sh), 0.55))

        # ---- светофор: скопление ярких диодов на тёмном корпусе -------------
        # Синий/голубой экран-баннер (H≈95…110) и пурпур (H≈155…170) сюда
        # не попадают: иначе их яркие пиксели принимались за диоды светофора.
        lamp = ((val > 205) & (sat > 110) &
                (((hue <= 38) | (hue >= 173)) | ((hue >= 39) & (hue <= 88)))).astype(np.uint8) * 255
        # Диоды идут строками: вертикальное ядро склеивает строки в одно табло,
        # а не в пять отдельных «светофоров».
        kv = max(25, int(sh / 14)) | 1
        kh = max(9, int(sw / 70)) | 1
        joined = cv2.dilate(lamp, np.ones((kv, kh), np.uint8), iterations=1)
        for contour in self._contours(joined):
            x, y, bw, bh = cv2.boundingRect(contour)
            if bw * bh < area * 0.002 or bw * bh > area * 0.4:
                continue
            if bw / max(1, bh) > 2.0 or bh / max(1, bw) > 3.5:
                continue
            pad_x, pad_y = int(bw * 0.35), int(bh * 0.60)
            x1, y1 = max(0, x - pad_x), max(0, y - pad_y)
            x2, y2 = min(sw, x + bw + pad_x), min(sh, y + bh + pad_y)
            roi = small[y1:y2, x1:x2]
            if not roi.size:
                continue
            region = val[y1:y2, x1:x2]
            dark_ratio = float(np.count_nonzero(region < 80)) / region.size
            if dark_ratio < 0.55:
                continue
            lamp_roi = lamp[y1:y2, x1:x2]
            dots = int(np.count_nonzero(lamp_roi))
            # Диоды занимают единицы процентов тёмного корпуса; большая яркая
            # область — это экран, стена или стол, а не светофор.
            if dots < 20 or dots > region.size * 0.06:
                continue
            components, _, stats, _ = cv2.connectedComponentsWithStats(lamp_roi, 8)
            blobs = sum(1 for i in range(1, components) if 4 <= stats[i, cv2.CC_STAT_AREA] <= 600)
            # Матрица — много мелких диодов. Одиночная линза допускается только
            # на очень тёмном корпусе и при малой площади свечения.
            if blobs < 8 and not (dark_ratio > 0.65 and dots < region.size * 0.015):
                continue
            found.append(Detection("traffic_light", self._box(x1, y1, x2 - x1, y2 - y1, sw, sh), 0.62))

        # Поиск «просто тёмного прямоугольника» намеренно не используется:
        # на полигоне полно тёмных дверей, штативов и мусорных корзин, и такой
        # контур давал ложные светофоры. Надёжный признак — именно сетка диодов.
        kept = [d for d in non_max_suppression(found) if d.confidence >= self.min_confidence]
        kept = self._merge_traffic_lights(kept)
        # Цвет сигнала считаем по той же уменьшенной картинке, по которой нашли рамку.
        coloured: list[Detection] = []
        for det in kept:
            if det.cls != "traffic_light":
                coloured.append(det)
                continue
            x, y, bw, bh = det.box
            roi = small[int(y * sh):int((y + bh) * sh), int(x * sw):int((x + bw) * sw)]
            signal = classify_traffic_light(roi, prefer_red=self.prefer_red)
            coloured.append(Detection(det.cls, det.box, det.confidence,
                                      signal["state"], signal["counts"], det.source))
        return sorted(coloured, key=lambda d: (d.cls, -d.confidence))


class YoloDetector:
    """Обёртка над Ultralytics YOLO; маппит ``model.names`` в наши классы."""

    def __init__(self, model_path: str, *, confidence: float = 0.45,
                 iou: float = 0.45, prefer_red: bool = False,
                 imgsz: int = 416, device: str = "cpu",
                 traffic_mode: str = "legacy") -> None:
        if traffic_mode not in ("legacy", "body", "verified"):
            raise ValueError("traffic_mode must be legacy, body or verified")
        self.traffic_mode = traffic_mode
        from ultralytics import YOLO  # импорт только когда веса реально есть

        self.model_path = str(model_path)
        self.confidence = float(confidence)
        self.iou = float(iou)
        self.prefer_red = bool(prefer_red)
        self.imgsz = max(160, min(1280, int(imgsz) // 32 * 32))
        self.device = device
        self.model = YOLO(self.model_path)
        self.names: dict[int, str] = {}
        self.signals: dict[int, str] = {}
        names = getattr(self.model, "names", None) or {}
        entries = names.items() if isinstance(names, dict) else enumerate(names)
        for key, value in entries:
            canonical = normalize_class(value)
            if canonical:
                self.names[int(key)] = canonical
            signal = signal_from_class(value)
            if signal:
                self.signals[int(key)] = signal
        self.has_signal_classes = bool(self.signals)
        if not self.names:
            raise ValueError("модель не содержит поддерживаемых классов знаков")

    def set_confidence(self, value: float) -> None:
        self.confidence = max(0.05, min(0.95, float(value)))

    def detect(self, frame_bgr: Any) -> list[Detection]:
        frame = np.asarray(frame_bgr)
        if frame.ndim != 3 or frame.shape[2] < 3 or not frame.size:
            return []
        frame = frame[..., :3]
        results = self.model.predict(frame, conf=self.confidence, iou=self.iou,
                                     imgsz=self.imgsz, verbose=False, device=self.device)
        h, w = frame.shape[:2]
        found: list[Detection] = []
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for box in boxes:
                cls_id = int(box.cls.item())
                cls = self.names.get(cls_id)
                if cls is None:
                    continue
                x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].tolist())
                confidence = float(box.conf.item())
                if not all(math.isfinite(v) for v in (x1, y1, x2, y2, confidence)):
                    continue
                x1, x2 = max(0.0, min(w, x1)), max(0.0, min(w, x2))
                y1, y2 = max(0.0, min(h, y1)), max(0.0, min(h, y2))
                if x2 <= x1 or y2 <= y1:
                    continue
                det = Detection(cls, (x1 / w, y1 / h, (x2 - x1) / w, (y2 - y1) / h),
                                confidence, color=self.signals.get(cls_id), source="yolo")
                found.append(det)
        coloured: list[Detection] = []
        for det in found:
            if det.cls != "traffic_light":
                coloured.append(det)
                continue
            x, y, bw, bh = det.box
            roi = frame[int(y * h):int((y + bh) * h), int(x * w):int((x + bw) * w)]
            if self.traffic_mode == "verified":
                signal = verify_traffic_light(roi, prefer_red=self.prefer_red)
                if not signal["validBody"]:
                    continue
                # Section geometry is additional evidence, not a replacement
                # for learned green agreement. A 5-class localizer cannot
                # grant green in this mode; dark/ambiguous evidence never
                # falls back to the colour head's guess either.
                state = (fuse_signal(det.color, signal["state"], colour_model=True)
                         if signal["state"] != "unknown" else "unknown")
            else:
                signal = classify_traffic_light(roi, prefer_red=self.prefer_red)
                state = fuse_signal(det.color, signal["state"], colour_model=self.has_signal_classes)
            coloured.append(Detection(det.cls, det.box, det.confidence,
                                      state, signal["counts"], "yolo"))
        merged = non_max_suppression(coloured, self.iou,
                                     conservative_signals=self.has_signal_classes or self.traffic_mode == "verified")
        if self.traffic_mode != "body":
            return merged
        # Body-only guard is a POST-NMS filter: it can only DELETE legacy
        # outputs, never add a green or let a previously suppressed green
        # escape a conflicting red/unknown. Colour fusion stays identical.
        guarded: list[Detection] = []
        for det in merged:
            if det.cls != "traffic_light":
                guarded.append(det)
                continue
            x, y, bw, bh = det.box
            roi = frame[int(y*h):int((y+bh)*h), int(x*w):int((x+bw)*w)]
            if verify_traffic_light(roi, prefer_red=self.prefer_red)["validBody"]:
                guarded.append(det)
        return guarded


class EnsembleDetector:
    """YOLO + встроенный детектор: берём сильное, не теряем дальние объекты.

    Эвристика дополняет пропуски (это не гарантия качества на новой сцене).
    При пересечении рамок одного класса побеждает YOLO; остальные находки
    добавляются как есть, но не обходят проверку согласия для green.
    """

    def __init__(self, yolo: YoloDetector, heuristic: HeuristicDetector) -> None:
        self.yolo = yolo
        self.heuristic = heuristic

    def set_confidence(self, value: float) -> None:
        self.yolo.set_confidence(value)
        self.heuristic.set_confidence(value)

    def detect(self, frame_bgr: Any) -> list[Detection]:
        primary = self.yolo.detect(frame_bgr)
        extra = self.heuristic.detect(frame_bgr)
        merged = list(primary)
        for candidate in extra:
            # Extra heuristic boxes cannot bypass verified body/section checks.
            if getattr(self.yolo, "traffic_mode", "legacy") in ("body", "verified") and candidate.cls == "traffic_light":
                continue
            threshold = 0.25 if candidate.cls == "traffic_light" else 0.40
            if any(other.cls == candidate.cls and iou(candidate, other) > threshold
                   for other in primary):
                continue
            # Эвристика не должна обходить проверку YOLO+HSV для green.
            if (getattr(self.yolo, "has_signal_classes", False) and
                    candidate.cls == "traffic_light" and candidate.color == "green"):
                candidate = Detection(candidate.cls, candidate.box, candidate.confidence,
                                      "unknown", candidate.color_counts, candidate.source)
            merged.append(candidate)
        return non_max_suppression(merged, 0.45)


def yolo_label_lines(detections: Iterable[Detection]) -> list[str]:
    """Разметка в формате YOLO для одного изображения."""
    return [d.yolo_line() for d in detections]
