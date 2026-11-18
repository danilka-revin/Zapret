#!/usr/bin/env python3
"""Собрать отдельный датасет Drive, не меняя прежние изображения, метки и split.

CSV — визуально проверенные рамки XYXY на сетке 400 × 225 (весь видимый кадр).
Никакие объекты не генерируются по имени файла, цвету пикселя или ответу YOLO.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
from decimal import Decimal
import hashlib
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

import cv2
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot_control.vision.classes import DRIVE_TRAINING_CLASS_NAMES as CLASSES
from scripts.prepare_robot_vision_dataset import crop_labels, label_line
from scripts.train_vision import inspect_dataset
from scripts.vision_dataset import perceptual_hash

REVIEW = ROOT / 'datasets/yolo-drive'


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def rows(name):
    with (REVIEW / name).open(encoding='utf-8', newline='') as f:
        return list(csv.DictReader(f))


def corners(row):
    x, y, right, bottom = (int(row[k]) for k in ('x1', 'y1', 'x2', 'y2'))
    if not (0 <= x < right <= 400 and 0 <= y < bottom <= 225):
        raise ValueError(f'рамка вне сетки 400 × 225: {row}')
    return x / 400, y / 225, (right-x) / 400, (bottom-y) / 225


def jpeg(frame):
    ok, data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
    if not ok:
        raise ValueError('не удалось сохранить кадр')
    return data.tobytes()


def context_crop(image, box, labels):
    """Additional TRAIN view at a larger scale; transform all visible labels."""
    h, w = image.shape[:2]
    x, y, bw, bh = box
    cw = min(w, round(max(0.28, 2.5 * bw) * w))
    ch = min(h, round(max(cw * 0.65, bh * h * 2.5)))
    if cw * ch > 0.85 * w * h:
        return None
    left = min(w-cw, max(0, round((x+bw/2)*w - cw/2)))
    top = min(h-ch, max(0, round((y+bh/2)*h - ch/2)))
    pixels = left, top, left+cw, top+ch
    return jpeg(image[top:top+ch, left:left+cw]), crop_labels(labels, pixels, w, h)


def horizontal_labels(lines: list[str]) -> list[str]:
    """Mirror normalized YOLO xywh. Preserve class (including LED colour), cy, w, h."""
    result = []
    for line in lines:
        if line.strip():
            cls, x, y, w, h = line.split()
            result.append(f'{cls} {1 - Decimal(x):f} {y} {w} {h}')
    return result


def build(parent: Path, media: Path, output: Path, *, mirror_train: bool = False) -> dict:
    parent, media, output = (p.resolve() for p in (parent, media, output))
    if output.exists():
        raise ValueError(f'папка уже есть, выберите новую: {output}')
    spec = json.loads((REVIEW / 'source.json').read_text())
    _, previous = inspect_dataset(parent / 'data.yaml')
    if previous['fingerprint'] != spec['parentFingerprint']:
        raise ValueError('нужен неизменённый snapshot yolo-stop-robot; см. docs/VISION_DRIVE.md')
    samples = json.loads((parent / 'samples.json').read_text())
    if set(samples) != {p.name for split in ('train', 'val') for p in (parent/'images'/split).glob('*.jpg')}:
        raise ValueError('список исходных изображений не совпадает с samples.json')
    boards = {}
    for row in rows('ground_bumps.csv'):
        name = row['image']
        if name not in samples or name in boards:
            raise ValueError(f'неизвестный или повторный кадр с кочкой: {name}')
        boards[name] = corners(row)
    videos = defaultdict(lambda: defaultdict(list))
    for row in rows('video_boxes.csv'):
        name, number = row['video'], int(row['frame'])
        if name not in spec['videos'] or Path(name).name != name or number < 0:
            raise ValueError(f'неизвестное видео или неверный номер кадра: {row}')
        labels = videos[name][number]
        line = label_line(CLASSES.index(row['class']), corners(row))
        if line in labels:
            raise ValueError(f'повторная рамка в {name}/{number}')
        labels.append(line)
    if videos.keys() != spec['videos'].keys():
        raise ValueError('список размеченных видео не совпадает с источником')

    output.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(dir=output.parent) as temporary:
        dst = Path(temporary) / 'dataset'
        records, transports = {}, Counter()

        def put(name, data, labels, **info):
            if Path(name).name != name or name in records or info['split'] not in ('train', 'val', 'test'):
                raise ValueError(f'некорректное или повторное имя кадра: {name}')
            for folder, filename, payload in [('images', name, data),
                    ('labels', Path(name).stem + '.txt', ''.join(line+'\n' for line in labels).encode())]:
                target = dst / folder / info['split'] / filename
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(payload)
            records[name] = dict(info, sha256=digest(data))

        for name, sample in sorted(samples.items()):
            part = sample['split']
            if part not in ('train', 'val') or Path(name).name != name:
                raise ValueError('ожидается только TRAIN/VAL родительского snapshot')
            image = parent/'images'/part/name
            direct = sample['kind'] == 'photo' and (media/name).is_file()
            payload = (media/name if direct else image).read_bytes()
            if digest(payload) != sample['sha256']:
                raise ValueError(f'не совпал SHA-256 фото: {name}')
            transports['drivePhoto' if direct else 'parentSnapshot'] += 1
            labels = (parent/'labels'/part/Path(name).with_suffix('.txt')).read_text().splitlines()
            info = dict(split=part, group=sample['group'], source=sample['source'])
            if name in boards:
                labels.append(label_line(CLASSES.index('ground_bump'), boards[name]))
            put(name, payload, labels, **info)
            if name in boards and part == 'train' and sample['kind'] in ('photo', 'video'):
                view = context_crop(cv2.imread(str(image)), boards[name], labels)
                if view is not None:
                    put('context-'+name, *view, **info, parentImage=name)

        # Never split a video (or two takes of the same new drive session).
        # Also reject near-duplicates of original images across splits.
        hashes = [(int(v['phash'], 16), v['split']) for v in samples.values()
                  if v['kind'] in ('photo', 'video')]
        for name, frames in sorted(videos.items()):
            meta = spec['videos'][name]
            path = media/name
            if digest(path.read_bytes()) != meta['sha256']:
                raise ValueError(f'не совпал SHA-256 видео: {name}')
            cap = cv2.VideoCapture(str(path))
            try:
                for number, labels in sorted(frames.items()):
                    cap.set(cv2.CAP_PROP_POS_FRAMES, number)
                    ok, frame = cap.read()
                    if not ok:
                        raise ValueError(f'не читается {name}, кадр {number}')
                    ph = perceptual_hash(frame)
                    part = meta['split']
                    if any(p != part and (other ^ ph).bit_count() <= 8 for other, p in hashes):
                        raise ValueError(f'похожий кадр между выборками: {name}/{number}')
                    hashes.append((ph, part))
                    put(f'{path.stem}_f{number:06}.jpg', jpeg(frame), labels,
                        split=part, group=meta['group'], source=name, frame=number)
            finally:
                cap.release()

        if mirror_train:
            # Snapshot the originals *after* splitting. Never mirror a mirror or a held-out frame.
            train_names = sorted(name for name, info in records.items() if info['split'] == 'train')
            for name in train_names:
                image = cv2.imread(str(dst / 'images/train' / name))
                if image is None:
                    raise ValueError(f'не читается TRAIN для отражения: {name}')
                labels = (dst / 'labels/train' / Path(name).with_suffix('.txt')).read_text().splitlines()
                info = records[name]
                put('hflip-' + name, jpeg(cv2.flip(image, 1)), horizontal_labels(labels),
                    split='train', group=info['group'], source=info['source'], parentImage=name,
                    transform='horizontal-flip')

        config = {p: f'images/{p}' for p in ('train', 'val', 'test')}
        config['names'] = dict(enumerate(CLASSES))
        (dst/'data.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
        (dst/'samples.json').write_text(json.dumps(records, indent=2)+'\n')
        _, audit = inspect_dataset(dst/'data.yaml')
        manifest = {**audit, 'parentFingerprint': previous['fingerprint'], 'source': 'Google Drive / yolo + pinned parent',
                    'parentMediaRead': dict(transports), 'framesPerDriveVideo': {k: len(v) for k, v in videos.items()},
                    'reviewSha256': {p: digest((REVIEW/p).read_bytes()) for p in ('ground_bumps.csv', 'video_boxes.csv', 'source.json')},
                    'warning': 'Agent-reviewed boxes and inherited pseudo-labels. Same indoor course; not independent human ground truth.'}
        if mirror_train:
            manifest['mirrorTrain'] = {'axis': 'horizontal', 'originals': len(train_names),
                                       'copies': len(train_names), 'heldOutCopies': 0}
        (dst/'dataset.json').write_text(json.dumps(manifest, indent=2)+'\n')
        dst.rename(output)
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--media', type=Path, required=True, help='локальная папка yolo с исходными файлами Drive')
    parser.add_argument('--parent', type=Path, default=ROOT/'datasets/yolo-stop-robot')
    parser.add_argument('--mirror-train', action='store_true', help='удвоить только TRAIN горизонтальным отражением изображений и рамок')
    parser.add_argument('--output', type=Path, help='новая папка (по умолчанию .cache/vision/yolo-drive или yolo-drive-x2)')
    args = parser.parse_args()
    output = args.output or ROOT / '.cache/vision' / ('yolo-drive-x2' if args.mirror_train else 'yolo-drive')
    try:
        print(json.dumps(build(args.parent, args.media, output, mirror_train=args.mirror_train), indent=2))
    except (ValueError, OSError) as exc:
        parser.exit(1, f'{exc}\n')
