#!/usr/bin/env python3
"""Дообучение YOLO для робота, без неявной замены рабочих весов.

    python scripts/train_vision.py --device 0 --epochs 150 --imgsz 640 --batch 16
    python scripts/train_vision.py --device cpu --epochs 80 --imgsz 416 --batch 8
    python scripts/train_vision.py --smoke --from-scratch
    python scripts/train_vision.py --weights yolov8n.pt --device 0 --onnx

По умолчанию warm start от имеющейся модели, результат — отдельный
models/vision/yolo-stop-candidate.pt. --deploy явно разрешает заменить
рабочий yolo-stop.pt. Smoke никогда не перезаписывает рабочие веса.
Выбирается best.pt по val, test оценивается только ПОСЛЕ обучения.
Цветовые классы запрещают hue shift, отражение знаков и вертикальный flip.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import shutil
import sys
import time
import uuid
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot_control.vision.classes import CLASS_NAMES, TRAINING_CLASS_NAMES  # noqa: E402

DATASET = ROOT / 'datasets' / 'yolo-stop'
MODEL_DIR = ROOT / 'models' / 'vision'
MODEL_FILE = MODEL_DIR / 'yolo-stop.pt'
SMOKE_MODEL_FILE = MODEL_DIR / 'yolo-stop-smoke.pt'
CANDIDATE_FILE = MODEL_DIR / 'yolo-stop-candidate.pt'
RUNS_DIR = ROOT / 'runs' / 'vision'


def ensure_dataset(no_download: bool) -> Path:
    data_yaml = DATASET / 'data.yaml'
    manifest = DATASET / 'dataset.json'
    samples = DATASET / 'samples.json'
    # A checkout intentionally omits heavy media, not its reviewed labels and
    # fixed source groups. Never silently regenerate those when pixels are absent.
    if any(path.exists() for path in (data_yaml, manifest, samples, DATASET / 'labels')):
        if (data_yaml.is_file() and manifest.is_file() and samples.is_file() and
                json.loads(manifest.read_text()).get('schemaVersion') == 2 and
                (DATASET / 'images' / 'train').is_dir()):
            return data_yaml
        raise ValueError('сохранённый датасет неполон или несовместим; автоматическая переразметка запрещена. '
                         'Для поставленного snapshot выполните scripts/restore_vision_source.py --restore-missing; '
                         'новый датасет создавайте явно и передавайте через --data')
    from scripts.vision_dataset import build
    build(zip_path=None, out=DATASET, no_download=no_download)
    return data_yaml


def inspect_dataset(source: Path) -> tuple[dict, dict]:
    """Проверить настоящие метки/хэши, а не только наличие папки train."""
    import yaml
    config = yaml.safe_load(source.read_text(encoding='utf-8'))
    names = config['names']
    names = [names[i] for i in range(len(names))] if isinstance(names, dict) else names
    if tuple(names) not in (CLASS_NAMES, TRAINING_CLASS_NAMES):
        raise ValueError('data.yaml: ожидаются 5 публичных или 8 цветовых классов в фиксированном порядке')
    root = Path(config.get('path', source.parent))
    if not root.is_absolute():
        # Внешний YAML с path: . относится к своей папке, а наш сохранённый
        # path: datasets/yolo-stop — к корню проекта. Проверяем путь к train,
        # а не просто существование '.' (оно верно в обеих папках).
        candidates = (source.parent / root, ROOT / root)
        root = next((item for item in candidates if (item / config['train']).is_dir()), candidates[0])
    root = root.resolve()
    config['path'] = str(root)
    hashes: dict[str, str] = {}
    groups: dict[str, str] = {}
    samples_file = source.parent / 'samples.json'
    samples = json.loads(samples_file.read_text()) if samples_file.is_file() else {}
    digest = hashlib.sha256(json.dumps({k: v for k, v in config.items() if k != 'path'}, sort_keys=True).encode())
    summary: dict[str, Any] = {'splits': {}, 'classes': names}
    for part in ('train', 'val', 'test'):
        if part not in config:
            continue
        folder = root / config[part]
        images = sorted(p for p in folder.iterdir() if p.suffix.lower() in ('.jpg', '.jpeg', '.png'))
        if not images:
            raise ValueError(f'{part}: нет изображений в {folder}')
        counts: Counter[str] = Counter()
        for image in images:
            # Этот пайплайн использует стандарт images/<split> ↔ labels/<split>.
            label = root / 'labels' / part / f'{image.stem}.txt'
            if not label.is_file():
                raise ValueError(f'нет меток: {label}; для фона нужен пустой .txt')
            content = label.read_text(encoding='utf-8')
            for line in content.splitlines():
                if not line.strip():
                    continue
                values = line.split()
                if len(values) != 5:
                    raise ValueError(f'{label}: ожидается cls cx cy w h')
                cls, cx, cy, w, h = map(float, values)
                if not all(math.isfinite(v) for v in (cls, cx, cy, w, h)) or not (
                    cls.is_integer() and 0 <= cls < len(names) and 0 < w <= 1 and 0 < h <= 1
                    and w / 2 - 1e-5 <= cx <= 1 - w / 2 + 1e-5
                    and h / 2 - 1e-5 <= cy <= 1 - h / 2 + 1e-5):
                    raise ValueError(f'{label}: некорректная рамка: {line}')
                counts[names[int(cls)]] += 1
            image_hash = hashlib.sha256(image.read_bytes()).hexdigest()
            if image_hash in hashes and hashes[image_hash] != part:
                raise ValueError(f'утечка train/val/test: дубликат {image}')
            hashes[image_hash] = part
            sample = samples.get(image.name, {})
            group = sample.get('group')
            if group and group in groups and groups[group] != part:
                raise ValueError(f'утечка train/val/test: группа {group}')
            if group:
                groups[group] = part
            digest.update(f'{part}/{image.name}:{image_hash}:{content}'.encode())
        summary['splits'][part] = {'images': len(images), 'counts': dict(counts),
                                  'missingClasses': [name for name in names if not counts[name]]}
    if not all(part in summary['splits'] for part in ('train', 'val')):
        raise ValueError('нужны непустые train и val')
    summary['fingerprint'] = digest.hexdigest()
    return config, summary


def runtime_data_yaml(source: Path, destination: Path | None = None) -> Path:
    import yaml
    config, _ = inspect_dataset(source)
    target = destination or RUNS_DIR / 'data.yaml'
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding='utf-8')
    return target


def load_model(weights: str | None, prefer_pretrained: bool):
    from ultralytics import YOLO
    if not prefer_pretrained:
        if weights:
            raise ValueError('--from-scratch нельзя совмещать с --weights')
        return YOLO('yolov8n.yaml'), 'yolov8n.yaml (random initialization)'
    base = weights or str(MODEL_FILE)
    if not weights and not MODEL_FILE.is_file():
        raise ValueError('нет начальных весов: укажите --weights yolov8n.pt или явно --from-scratch')
    if base.endswith(('.yaml', '.yml')):
        raise ValueError('случайная инициализация требует --from-scratch')
    return YOLO(base), base


def metrics_dict(result) -> dict[str, Any]:
    metrics = {key: round(float(value), 6) for key, value in result.results_dict.items()}
    # ap_class_index исключает классы без GT; нулевую/отсутствующую AP не прячем.
    metrics['perClass'] = [
        {'id': int(index), 'name': result.names[int(index)],
         'precision': round(float(result.box.p[i]), 6),
         'recall': round(float(result.box.r[i]), 6),
         'mAP50': round(float(result.box.ap50[i]), 6),
         'mAP50-95': round(float(result.box.ap[i]), 6)}
        for i, index in enumerate(result.box.ap_class_index)
    ]
    return metrics


def _atomic_copy(source: Path, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name + '.tmp')
    shutil.copy2(source, temporary)
    os.replace(temporary, target)


def train(*, epochs: int, imgsz: int, batch: int, weights: str | None,
          prefer_pretrained: bool, no_download: bool, smoke: bool,
          export_onnx: bool, output: Path | None = None, data: Path | None = None,
          device: str = 'auto', workers: int = 0, seed: int = 17,
          patience: int = 25, freeze: int = 0, cache: str = 'false',
          resume: Path | None = None, deploy: bool = False,
          lr0: float = 0.001, evaluate_test: bool = True,
          warmup_bias_lr: float = 0.0) -> dict[str, Any]:
    import torch
    import ultralytics
    from ultralytics import YOLO
    import yaml
    if epochs < 1 or imgsz < 160 or imgsz % 32 or batch < 1 or workers < 0 or patience < 0 or freeze < 0:
        raise ValueError('epochs/batch >= 1, imgsz >= 160 кратен 32, workers/patience/freeze >= 0')
    if not math.isfinite(lr0) or not 0 < lr0 <= .1:
        raise ValueError('lr0 должен быть конечным числом в (0, 0.1]')
    if not math.isfinite(warmup_bias_lr) or not 0 <= warmup_bias_lr <= 1:
        raise ValueError('warmup-bias-lr должен быть конечным числом в [0, 1]')
    if smoke and (deploy or (output and output.resolve() == MODEL_FILE.resolve())):
        raise ValueError('--smoke запрещено записывать в рабочую модель')
    if resume and (weights or not prefer_pretrained or smoke):
        raise ValueError('--resume нельзя совмещать с --weights/--from-scratch/--smoke')
    model_file = (output or (SMOKE_MODEL_FILE if smoke else MODEL_FILE if deploy else CANDIDATE_FILE)).resolve()
    if model_file.suffix.lower() != '.pt':
        raise ValueError('--output должен иметь расширение .pt; ONNX включается через --onnx')
    if model_file == MODEL_FILE.resolve() and not deploy:
        raise ValueError('для замены рабочей модели нужен --deploy')
    source = (data or ensure_dataset(no_download)).resolve()
    config, dataset_summary = inspect_dataset(source)
    if device == 'auto':
        device = '0' if torch.cuda.is_available() else 'cpu'
    if device == 'cpu':
        torch.set_num_threads(min(2, os.cpu_count() or 1))
    run_dir = RUNS_DIR / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:6])
    run_dir.mkdir(parents=True)
    data_yaml = run_dir / 'data.yaml'
    data_yaml.write_text(yaml.safe_dump(config, sort_keys=False), encoding='utf-8')
    (run_dir / 'dataset-audit.json').write_text(json.dumps(dataset_summary, indent=2), encoding='utf-8')
    if resume:
        model, base = YOLO(str(resume)), str(resume)
        if model.ckpt.get('epoch', -1) < 0 or model.ckpt.get('optimizer') is None:
            raise ValueError('resume: checkpoint завершён или не содержит optimizer; для нового запуска нужен --weights')
        original_data = Path(model.ckpt.get('train_args', {}).get('data', ''))
        audit = original_data.parent / 'dataset-audit.json'
        if not original_data.is_file() or not audit.is_file():
            raise ValueError('resume: нет dataset-audit.json исходного запуска; используйте --weights для нового обучения')
        _, original_summary = inspect_dataset(original_data)
        if not (json.loads(audit.read_text())['fingerprint'] == original_summary['fingerprint'] == dataset_summary['fingerprint']):
            raise ValueError('resume: изменились изображения, метки или разбиение')
    else:
        model, base = load_model(weights, prefer_pretrained)
    requested_epochs = 2 if smoke else epochs
    if resume:
        requested_epochs = int(model.ckpt['train_args']['epochs'])
    started = time.monotonic()
    print(f'Обучение: {requested_epochs} эпох, imgsz={imgsz}, batch={batch}, device={device}', flush=True)
    print('Псевдоразметка не заменяет независимую проверку на камере робота.', flush=True)
    result = model.train(
        data=str(data_yaml), epochs=requested_epochs, imgsz=imgsz, batch=batch,
        device=device, workers=workers, seed=seed, deterministic=True,
        project=str(run_dir), name='train', exist_ok=False,
        patience=patience, freeze=freeze, cache=False if cache == 'false' else cache,
        amp=device != 'cpu', plots=not smoke, verbose=False, save=True,
        optimizer='AdamW', lr0=lr0, cos_lr=True, warmup_epochs=min(3, requested_epochs),
        warmup_bias_lr=warmup_bias_lr,
        hsv_h=0.0, hsv_s=0.15, hsv_v=0.3, flipud=0.0, fliplr=0.0,
        degrees=8.0, translate=0.1, scale=0.35, perspective=0.0001,
        mosaic=0.5, close_mosaic=min(10, requested_epochs), mixup=0.0,
        resume=str(resume) if resume else False,
    )
    save_dir = Path(result.save_dir)
    best = save_dir / 'weights' / 'best.pt'
    if not best.is_file():
        raise RuntimeError(f'нет best.pt: {best}')
    _, final_summary = inspect_dataset(source)
    if final_summary['fingerprint'] != dataset_summary['fingerprint']:
        raise RuntimeError('датасет изменился во время обучения; best.pt сохранён, публикация отменена')
    best_model = YOLO(str(best))
    actual_names = [best_model.names[i] for i in range(len(best_model.names))]
    if actual_names != dataset_summary['classes']:
        raise RuntimeError('имена классов обученной модели не совпали с датасетом')
    evaluations = {}
    for split in ('val', 'test'):
        if split in config and (split != 'test' or evaluate_test):
            evaluated = best_model.val(data=str(data_yaml), split=split, imgsz=imgsz,
                                       batch=batch, device=device, workers=workers, plots=not smoke,
                                       project=str(run_dir), name=split, verbose=False)
            evaluations[split] = metrics_dict(evaluated)
    rows = []
    if (save_dir / 'results.csv').is_file():
        with (save_dir / 'results.csv').open() as handle:
            rows = [{key.strip(): value for key, value in row.items()} for row in csv.DictReader(handle)]
    best_row = max(rows, key=lambda row: float(row.get('metrics/mAP50-95(B)', 0))) if rows else {}
    manifest_file = source.parent / 'dataset.json'
    manifest = json.loads(manifest_file.read_text()) if manifest_file.is_file() else {}
    base_path = Path(base)
    provenance_path = base_path.with_suffix('.json')
    provenance = json.loads(provenance_path.read_text()) if provenance_path.is_file() else {}
    provenance = provenance.get('baseProvenance', provenance)
    provenance = {k: v for k, v in provenance.items() if k in ('source', 'revision', 'contentSha256', 'checkpointVersion', 'checkpointDate', 'originalTrainingData', 'licenseNote')}
    effective_args = yaml.safe_load((save_dir / 'args.yaml').read_text())
    metadata = {
        'model': model_file.name, 'sha256': hashlib.sha256(best.read_bytes()).hexdigest(),
        'createdAt': datetime.now(timezone.utc).isoformat(), 'base': base,
        'baseSha256': hashlib.sha256(base_path.read_bytes()).hexdigest() if base_path.is_file() else None,
        'baseProvenance': provenance,
        'ultralytics': ultralytics.__version__, 'torch': torch.__version__,
        'classes': actual_names, 'epochsRequested': requested_epochs, 'epochsCompleted': len(rows),
        'bestEpoch': int(best_row['epoch']) if best_row else None,
        'imgsz': imgsz, 'batch': batch, 'device': device, 'freeze': effective_args.get('freeze', freeze), 'seed': effective_args.get('seed', seed),
        'trainingArgs': {k: effective_args.get(k) for k in ('optimizer', 'lr0', 'lrf', 'cos_lr', 'warmup_bias_lr', 'cache', 'deterministic', 'patience', 'workers')},
        'durationS': round(time.monotonic() - started, 1), 'smoke': smoke,
        'validation': evaluations.get('val'), 'test': evaluations.get('test'),
        'dataset': {**dataset_summary, 'archiveSha256': manifest.get('archiveSha256'),
                    'sourceRevision': manifest.get('sourceRevision'),
                    'annotationWarning': manifest.get('warning', 'Labels not independently audited.')},
        'augmentation': {'hsv_h': 0.0, 'hsv_s': 0.15, 'hsv_v': 0.3,
                         'flipud': 0.0, 'fliplr': 0.0, 'mosaic': 0.5, 'close_mosaic': min(10, requested_epochs)},
        'runDirectory': str(save_dir.relative_to(ROOT)),
        'trafficLight': ('YOLO colour classes + conservative HSV agreement; generic = off/ambiguous'
                         if tuple(actual_names) == TRAINING_CLASS_NAMES else
                         'Unified traffic-light localizer; colour requires separate verified-ROI evaluation'),
        'deployment': 'candidate; validate on robot with wheels lifted and E-STOP available',
    }
    _atomic_copy(best, model_file)
    if export_onnx:
        try:
            exported = Path(best_model.export(format='onnx', imgsz=imgsz, simplify=False,
                                              dynamic=False, device='cpu'))
            _atomic_copy(exported, model_file.with_suffix('.onnx'))
            metadata['onnx'] = model_file.with_suffix('.onnx').name
        except Exception as exc:
            metadata['onnxError'] = str(exc)
            print(f'ONNX не экспортирован, PT сохранён: {exc}', flush=True)
    metadata_file = (MODEL_DIR / 'metadata.json') if model_file == MODEL_FILE.resolve() else model_file.with_suffix('.json')
    metadata_file.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(metadata, ensure_ascii=False, indent=2), flush=True)
    print(f'\nВеса: {model_file}', flush=True)
    return metadata


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--epochs', type=int, default=150)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--batch', type=int, default=8)
    parser.add_argument('--data', type=Path)
    parser.add_argument('--weights')
    parser.add_argument('--from-scratch', action='store_true')
    parser.add_argument('--no-download', action='store_true')
    parser.add_argument('--smoke', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--device', default='auto', help='auto, cpu, 0, 0,1')
    parser.add_argument('--workers', type=int, default=0)
    parser.add_argument('--seed', type=int, default=17)
    parser.add_argument('--patience', type=int, default=25)
    parser.add_argument('--lr0', type=float, default=.001, help='начальный learning rate AdamW')
    parser.add_argument('--warmup-bias-lr', type=float, default=0.0, help='learning rate bias в начале warmup')
    parser.add_argument('--freeze', type=int, default=0, help='число замороженных начальных слоёв')
    parser.add_argument('--cache', choices=('false', 'ram', 'disk'), default='false')
    parser.add_argument('--resume', type=Path, help='last.pt незавершённого запуска')
    parser.add_argument('--deploy', action='store_true', help='явно разрешить замену рабочих весов')
    parser.add_argument('--onnx', action='store_true')
    parser.add_argument('--validation-only', action='store_true', help='не оценивать test при подборе/дообучении')
    args = parser.parse_args(argv)
    try:
        train(epochs=args.epochs, imgsz=args.imgsz, batch=args.batch, weights=args.weights,
              prefer_pretrained=not args.from_scratch, no_download=args.no_download,
              smoke=args.smoke, export_onnx=args.onnx, output=args.output, data=args.data,
              device=args.device, workers=args.workers, seed=args.seed, patience=args.patience,
              freeze=args.freeze, cache=args.cache, resume=args.resume, deploy=args.deploy,
              lr0=args.lr0, evaluate_test=not args.validation_only, warmup_bias_lr=args.warmup_bias_lr)
    except ValueError as exc:
        parser.error(str(exc))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
