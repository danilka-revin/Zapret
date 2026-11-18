#!/usr/bin/env python3
"""Проверка кандидата ПОСЛЕ выбора по val: рабочий порог + YOLO/HSV fusion.

Не заменяет Ultralytics mAP: считает TP/FP/FN при одном confidence и IoU≥0.5,
отдельно ошибки цвета и ложные green. Все метрики зависят от качества меток.
На роботе нужны новый независимый ролик и измерение FPS на бортовом железе.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import statistics
import sys
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_control.vision.classes import (  # noqa: E402
    CLASS_NAMES, TRAINING_CLASS_NAMES, normalize_class, signal_from_class,
)
from robot_control.vision.detector import (  # noqa: E402
    Detection, EnsembleDetector, HeuristicDetector, YoloDetector, iou,
)
from scripts.train_vision import inspect_dataset  # noqa: E402


def score_image(expected: list[Detection], predicted: list[Detection]) -> dict:
    """Confidence-greedy one-to-one matching, без оценки через саму эвристику."""
    remaining = set(range(len(expected)))
    counts = {name: Counter(tp=0, fp=0, fn=0) for name in CLASS_NAMES}
    signals: Counter[str] = Counter()
    false_green = 0
    for det in sorted(predicted, key=lambda item: item.confidence, reverse=True):
        candidates = [index for index in remaining if expected[index].cls == det.cls]
        best = max(candidates, key=lambda index: iou(det, expected[index])) if candidates else None
        match = best is not None and iou(det, expected[best]) >= .5
        counts[det.cls]['tp' if match else 'fp'] += 1
        if match:
            remaining.remove(best)
        if det.cls == 'traffic_light':
            truth = (expected[best].color or 'unknown') if match else 'background'
            signal = det.color or 'unknown'
            signals[f'{truth}->{signal}'] += 1
            false_green += int(signal == 'green' and truth != 'green')
    for index in remaining:
        det = expected[index]
        counts[det.cls]['fn'] += 1
        if det.cls == 'traffic_light':
            signals[f'{det.color or "unknown"}->missed'] += 1
    return {'counts': counts, 'signalConfusion': signals, 'falseGreen': false_green}


def _labels(path: Path) -> list[Detection]:
    items = []
    for row in path.read_text().splitlines():
        if not row.strip():
            continue
        cls, cx, cy, w, h = map(float, row.split())
        name = TRAINING_CLASS_NAMES[int(cls)]
        items.append(Detection(normalize_class(name), (cx - w / 2, cy - h / 2, w, h),
                               1.0, signal_from_class(name), source='ground-truth'))
    return items


def preview(rows: list[dict], directory: Path, target: Path) -> list[str]:
    """Примеры по классам + равномерная выборка; без отбора по успеху модели."""
    import cv2
    from PIL import Image, ImageDraw
    chosen = []
    for cls in TRAINING_CLASS_NAMES:
        row = next((row for row in rows if cls in row['groundTruthClasses'] and row not in chosen), None)
        if row:
            chosen.append(row)
    for row in rows[::max(1, len(rows) // 12)]:
        if len(chosen) >= 12:
            break
        if row not in chosen:
            chosen.append(row)
    width, height = 400, 300
    sheet = Image.new('RGB', (3 * width, ((len(chosen) + 2) // 3) * height), '#20262e')
    colors = {'stop_sign': '#ff5374', 'speed_bump': '#ffb020', 'pedestrian_crossing': '#3bb7ff',
              'cone': '#ffaa77', 'red': '#ff5374', 'yellow': '#ffe26b', 'green': '#62e681'}
    for index, row in enumerate(chosen):
        frame = cv2.imread(str(directory / row['image']))
        image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        image.thumbnail((width - 8, height - 42))
        draw = ImageDraw.Draw(image)
        w, h = image.size
        for det in row['predictions']:
            x, y, bw, bh = (det['box'][key] for key in ('x', 'y', 'w', 'h'))
            signal = det['signal']
            color = colors.get(signal if det['class'] == 'traffic_light' else det['class'], '#bfa3ff')
            tag = (signal or 'unknown') if det['class'] == 'traffic_light' else det['class']
            tag += f" {det['confidence']:.2f}"
            draw.rectangle((x*w, y*h, (x+bw)*w, (y+bh)*h), outline=color, width=2)
            label_y = max(0, y*h - 11)
            draw.text((x*w, label_y), tag, fill=color, stroke_width=1, stroke_fill='black')
        x, y = index % 3 * width, index // 3 * height
        sheet.paste(image, (x + (width - w) // 2, y))
        draw = ImageDraw.Draw(sheet)
        draw.text((x + 5, y + height - 36), row['image'], fill='white')
        draw.text((x + 5, y + height - 20), f"predictions: {len(row['predictions'])} | false green: {row['falseGreen']}", fill='white')
    target.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(target, quality=90)
    return [row['image'] for row in chosen]


def evaluate(*, weights: Path, data: Path, output: Path, split: str = 'test',
             confidence: float = .45, imgsz: int = 416, device: str = 'cpu',
             prefer_red: bool = True, ensemble: bool = False,
             preview_file: Path | None = None, traffic_mode: str = "legacy") -> dict:
    import cv2
    import torch
    if device == 'cpu':
        torch.set_num_threads(2)
    config, audit = inspect_dataset(data.resolve())
    if len(audit['classes']) > len(TRAINING_CLASS_NAMES):
        raise ValueError('Этот тест проверяет 5-классовый API робота, а не кочку на полу. '
                         'Для весов Drive используйте scripts/report_drive_vision.py.')
    directory = Path(config['path']) / config[split]
    yolo = YoloDetector(str(weights), confidence=confidence, imgsz=imgsz,
                        device=device, prefer_red=prefer_red, traffic_mode=traffic_mode)
    detector = EnsembleDetector(yolo, HeuristicDetector(min_confidence=max(.5, confidence),
                                                        prefer_red=prefer_red)) if ensemble else yolo
    files = sorted(path for path in directory.iterdir() if path.suffix.lower() in ('.jpg', '.jpeg', '.png'))
    for _ in range(2):
        detector.detect(cv2.imread(str(files[0])))
    totals = {name: Counter(tp=0, fp=0, fn=0) for name in CLASS_NAMES}
    confusion: Counter[str] = Counter()
    rows, latencies = [], []
    for image in files:
        frame = cv2.imread(str(image))
        if frame is None:
            raise ValueError(f'не удалось декодировать {image}')
        expected = _labels(Path(config['path']) / 'labels' / split / image.with_suffix('.txt').name)
        started = time.perf_counter()
        predicted = detector.detect(frame)
        latencies.append((time.perf_counter() - started) * 1000)
        scored = score_image(expected, predicted)
        for name, counts in scored['counts'].items():
            totals[name].update(counts)
        confusion.update(scored['signalConfusion'])
        classes = [(f'traffic_light_{det.color}' if det.color else det.cls) for det in expected]
        rows.append({'image': image.name, 'groundTruthClasses': classes,
                     'predictions': [det.to_dict() for det in predicted], **scored})
    for counts in totals.values():
        tp, fp, fn = counts['tp'], counts['fp'], counts['fn']
        counts['precision'] = round(tp / (tp + fp), 4) if tp + fp else 0.0
        counts['recall'] = round(tp / (tp + fn), 4) if tp + fn else 0.0
    report = {
        'weights': str(weights), 'sha256': hashlib.sha256(weights.read_bytes()).hexdigest(),
        'createdAt': datetime.now(timezone.utc).isoformat(), 'datasetFingerprint': audit['fingerprint'],
        'split': split, 'images': len(rows), 'backend': 'yolo+heuristic+fusion' if ensemble else 'yolo+fusion',
        'confidence': confidence, 'iouMatch': .5, 'imgsz': imgsz, 'device': device, 'preferRed': prefer_red,
        'trafficMode': traffic_mode,
        'counts': totals, 'signalConfusion': confusion,
        'falseGreen': sum(row['falseGreen'] for row in rows),
        'latencyMedianMs': round(statistics.median(latencies), 2),
        'latencyP95Ms': round(sorted(latencies)[int((len(latencies)-1)*.95)], 2),
        'warning': 'Bootstrap/agent-assisted labels, NOT independent ground truth or a safety guarantee. '
                   'Latency measured in this environment, not on robot hardware; excludes JPEG decode.',
        'frames': rows,
    }
    if preview_file:
        report['previewImages'] = preview(rows, directory, preview_file)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({key: value for key, value in report.items() if key != 'frames'}, ensure_ascii=False, indent=2))
    return report


def calibrate(*, weights: Path, data: Path, output: Path, imgsz: int = 512,
              device: str = 'cpu', prefer_red: bool = True, traffic_mode: str = 'legacy') -> dict:
    """Выбрать рабочий confidence ТОЛЬКО по val, не заглядывая в test.

    Цель: 0.7 × macro-F1 пяти объектов + 0.3 × balanced recall трёх
    сигналов. Конфигурация с false-green на val не проходит. Это маленькая
    псевдоразмеченная val, а не сертификация нулевого риска.
    """
    points = []
    for confidence in (.15, .25, .35, .45):
        report = evaluate(weights=weights, data=data, output=output.with_name(f'val-conf-{confidence:.2f}.json'),
                          split='val', confidence=confidence, imgsz=imgsz, device=device, prefer_red=prefer_red,
                          traffic_mode=traffic_mode)
        f1 = []
        for counts in report['counts'].values():
            tp, fp, fn = counts['tp'], counts['fp'], counts['fn']
            f1.append(2 * tp / (2 * tp + fp + fn) if 2 * tp + fp + fn else 0.0)
        signal_recall = []
        for state in ('red', 'yellow', 'green'):
            count = sum(value for key, value in report['signalConfusion'].items() if key.startswith(state + '->'))
            signal_recall.append(report['signalConfusion'].get(f'{state}->{state}', 0) / count if count else 0.0)
        macro_f1, balanced = statistics.mean(f1), statistics.mean(signal_recall)
        points.append({'confidence': confidence, 'macroF1': round(macro_f1, 6),
                       'signalBalancedRecall': round(balanced, 6), 'falseGreen': report['falseGreen'],
                       'score': round(.7 * macro_f1 + .3 * balanced, 6)})
    allowed = [point for point in points if point['falseGreen'] == 0]
    selected = max(allowed, key=lambda point: (point['score'], point['confidence'])) if allowed else None
    summary = {'split': 'val', 'sha256': report['sha256'], 'datasetFingerprint': report['datasetFingerprint'],
               'imgsz': imgsz, 'preferRed': prefer_red, 'backend': 'yolo+fusion', 'trafficMode': traffic_mode,
               'objective': '0.7*canonical_macro_F1 + 0.3*signal_balanced_recall; falseGreen==0 on val',
               'points': points, 'selected': selected,
               'warning': 'Tiny bootstrap/agent-assisted validation set; this constraint is NOT a safety guarantee.'}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(summary, indent=2))
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', type=Path, default=ROOT / 'models/vision/yolo-stop-candidate.pt')
    parser.add_argument('--data', type=Path, default=ROOT / 'datasets/yolo-stop/data.yaml')
    parser.add_argument('--output', type=Path, default=ROOT / 'models/vision/yolo-stop-candidate-evaluation.json')
    parser.add_argument('--split', choices=('val', 'test'), default='test')
    parser.add_argument('--confidence', type=float, default=.45)
    parser.add_argument('--imgsz', type=int, default=416)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--no-prefer-red', action='store_true')
    parser.add_argument('--ensemble', action='store_true')
    parser.add_argument('--traffic-mode', choices=('legacy','body','verified'), default='legacy')
    parser.add_argument('--preview', type=Path)
    parser.add_argument('--calibrate', action='store_true', help='подобрать confidence на val; test НЕ используется')
    args = parser.parse_args()
    if args.calibrate:
        if args.ensemble:
            parser.error('--calibrate проверяет YOLO+fusion, без ансамбля')
        calibrate(weights=args.weights, data=args.data, output=args.output, imgsz=args.imgsz,
                  device=args.device, prefer_red=not args.no_prefer_red, traffic_mode=args.traffic_mode)
        return
    evaluate(weights=args.weights, data=args.data, output=args.output, split=args.split,
             confidence=args.confidence, imgsz=args.imgsz, device=args.device,
             prefer_red=not args.no_prefer_red, ensemble=args.ensemble, preview_file=args.preview,
             traffic_mode=args.traffic_mode)


if __name__ == '__main__':
    main()
