#!/usr/bin/env python3
"""Что видит поставленный PT при фиксированном пороге, без HSV и без робота.

В дополнение к mAP: сколько объектов найдено при conf=0.25, и сколько
корпусов найдено независимо от правильности цвета. Матрица включает ложные
рамки и пропуски. Порог/размер те же, что в примере yolo predict.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot_control.vision.classes import DRIVE_TRAINING_CLASS_NAMES as CLASSES
from scripts.train_vision import inspect_dataset


def counts(matrix, ids):
    """Rows = predicted, columns = labelled; last index = background."""
    return {'correct': sum(int(matrix[i, j]) for i in ids for j in ids),
            'labelled': int(matrix[:, ids].sum()), 'predicted': int(matrix[ids, :].sum())}


def report(weights, data, *, imgsz=416, splits=('val', 'test')):
    import torch
    from ultralytics import YOLO
    from ultralytics.utils.metrics import ConfusionMatrix
    from ultralytics.utils.ops import xywh2xyxy

    torch.set_num_threads(2)
    config, audit = inspect_dataset(data.resolve())
    model = YOLO(weights)
    if tuple(audit['classes']) != CLASSES or tuple(model.names[i] for i in range(len(model.names))) != CLASSES:
        raise ValueError('ожидались девять классов Drive в порядке data.yaml')
    root = Path(config['path'])
    result = {'weights': weights.name, 'sha256': hashlib.sha256(weights.read_bytes()).hexdigest(),
              'datasetFingerprint': audit['fingerprint'], 'backend': 'Ultralytics predict (raw PT, no HSV)',
              'imgsz': imgsz, 'conf': 0.25, 'nmsIoU': 0.7, 'matchIoU': 0.5,
              'matrixOrder': [*CLASSES, 'background'], 'splits': {}}
    for split in splits:
        matrix = ConfusionMatrix(names=model.names)
        images = 0
        for pred in model.predict(source=str(root/config[split]), imgsz=imgsz, conf=0.25, iou=0.7,
                                  device='cpu', verbose=False, stream=True):
            rows = (root/'labels'/split/Path(pred.path).with_suffix('.txt').name).read_text().splitlines()
            gt = torch.tensor([list(map(float, row.split())) for row in rows if row.strip()]).reshape(-1, 5)
            matrix.process_batch({'cls': pred.boxes.cls, 'conf': pred.boxes.conf, 'bboxes': pred.boxes.xyxyn},
                                 {'cls': gt[:, 0], 'bboxes': xywh2xyxy(gt[:, 1:])}, conf=0.25, iou_thres=0.5)
            images += 1
        m = matrix.matrix
        result['splits'][split] = {'images': images, 'matrixPredictedByTrue': m.astype(int).tolist(),
                                   'perClass': {name: counts(m, [i]) for i, name in enumerate(CLASSES)},
                                   'lightAnyState': counts(m, [i for i, name in enumerate(CLASSES)
                                                              if name.startswith('traffic_light')])}
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=ROOT/'.cache/vision/yolo-drive/data.yaml')
    parser.add_argument('--weights', type=Path, default=ROOT/'models/vision/yolo-drive.pt')
    parser.add_argument('--imgsz', type=int, default=416)
    parser.add_argument('--split', choices=('val', 'test', 'both'), default='both')
    parser.add_argument('--output', type=Path, help='новый JSON-файл (по умолчанию stdout)')
    args = parser.parse_args()
    splits = ('val', 'test') if args.split == 'both' else (args.split,)
    text = json.dumps(report(args.weights, args.data, imgsz=args.imgsz, splits=splits), ensure_ascii=False, indent=2) + '\n'
    if args.output:
        with args.output.open('x', encoding='utf-8') as f:
            f.write(text)
    else:
        print(text, end='')
