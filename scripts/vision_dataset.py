#!/usr/bin/env python3
"""Сборка датасета из ВСЕХ фото, вложенных ZIP и видео yolo-stop.

    python scripts/vision_dataset.py
    python scripts/vision_dataset.py --zip source-repo.zip --video-fps 1
    python scripts/vision_dataset.py --no-download --annotations reviewed.json

Исходник закреплён за commit, точные дубликаты исключаются по SHA-256.
Близкие фото (pHash) и все кадры одного видео попадают в одну группу;
train/val/test делятся ГРУППАМИ, не случайными соседними кадрами.
Авторазметка — только псевдометки; --annotations заменяет предложения
проверенными рамками. JPEG/видео/архивы/preview в Git не попадают.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
import shutil
import sys
import urllib.request
import zipfile
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot_control.vision.classes import TRAINING_CLASS_NAMES  # noqa: E402
from robot_control.vision.detector import Detection, cv2_available  # noqa: E402
from scripts.vision_bootstrap import propose  # noqa: E402

SOURCE_REPOSITORY = 'https://github.com/p25543770-lang/yolo-stop'
SOURCE_REVISION = '3cb277bfadf7d0034943117f9146d4b92bb773a9'
SOURCE_URL = f'https://codeload.github.com/p25543770-lang/yolo-stop/zip/{SOURCE_REVISION}'
# Оба ZIP содержат один commit: различаются корневая папка и SHA самого ZIP.
KNOWN_ARCHIVES = {
    '464f43a9042be1e802745b7f00c8c44074b51d5bea8c4611eb4553afa5f55c70',
    '762d6fa93f6f5b8103041cfced32de30b32d752399a37742e7f05d30499a0b88',
}
ARCHIVE_NAME = 'source-repo.zip'
DEFAULT_OUT = ROOT / 'datasets' / 'yolo-stop'
SCHEMA_VERSION = 2
IMAGE_EXTENSIONS = ('.jpg', '.jpeg', '.png')


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _read_archive(path: Path) -> dict[str, bytes]:
    """Совместимость для локального ZIP фотографий; читает и вложенные ZIP."""
    images, _ = read_media(path)
    return images


def read_media(path: Path) -> tuple[dict[str, bytes], dict[str, bytes]]:
    images: dict[str, bytes] = {}
    videos: dict[str, bytes] = {}

    def visit(data: bytes, depth: int = 0) -> None:
        if depth > 2:
            raise ValueError('слишком много вложенных ZIP')
        with zipfile.ZipFile(io.BytesIO(data)) as archive:
            for member in archive.infolist():
                if member.is_dir():
                    continue
                name = Path(member.filename).name  # Никакого extractall/zip slip.
                suffix = Path(name).suffix.lower()
                if suffix == '.zip':
                    visit(archive.read(member), depth + 1)
                elif suffix in IMAGE_EXTENSIONS or suffix == '.mp4':
                    target = videos if suffix == '.mp4' else images
                    payload = archive.read(member)
                    if name in target and target[name] != payload:
                        raise ValueError(f'разные файлы с одинаковым именем: {name}')
                    target[name] = payload
    visit(path.read_bytes())
    return images, videos


def download_archive(destination: Path) -> Path:
    destination.mkdir(parents=True, exist_ok=True)
    target = destination / ARCHIVE_NAME
    if not target.is_file():
        print(f'скачиваю {SOURCE_URL}', flush=True)
        temporary = target.with_suffix('.zip.part')
        try:
            with urllib.request.urlopen(SOURCE_URL, timeout=90) as reply:
                with temporary.open('wb') as output:
                    shutil.copyfileobj(reply, output)
            if _sha256(temporary.read_bytes()) not in KNOWN_ARCHIVES:
                raise ValueError('SHA-256 исходного архива не совпал с закреплённой версией')
            temporary.replace(target)
        finally:
            temporary.unlink(missing_ok=True)
    if _sha256(target.read_bytes()) not in KNOWN_ARCHIVES:
        raise ValueError('неизвестный кэш архива: передайте его явно через --zip или уберите старый source-repo.zip')
    return target


def perceptual_hash(frame) -> int:
    import cv2
    import numpy as np
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    dct = cv2.dct(cv2.resize(gray, (32, 32)).astype(np.float32))[:8, :8]
    bits = (dct > np.median(dct.flatten()[1:])).flatten()
    value = 0
    for bit in bits:
        value = (value << 1) | int(bit)
    return value


def group_samples(samples: dict[str, dict[str, Any]], distance: int = 8) -> None:
    """Union-find: видео целиком + визуально близкие фото/кадры между источниками."""
    names = sorted(samples)
    parents = list(range(len(names)))

    def root(i):
        while parents[i] != i:
            parents[i] = parents[parents[i]]
            i = parents[i]
        return i

    def union(a, b):
        ra, rb = root(a), root(b)
        parents[max(ra, rb)] = min(ra, rb)

    videos: dict[str, int] = {}
    for i, name in enumerate(names):
        sample = samples[name]
        if sample['kind'] == 'video':
            source = sample['source']
            if source in videos:
                union(i, videos[source])
            videos[source] = i
        for j in range(i):
            if (sample['_phash'] ^ samples[names[j]]['_phash']).bit_count() <= distance:
                union(i, j)
    for i, name in enumerate(names):
        samples[name]['group'] = _sha256(names[root(i)].encode())[:16]


def split_groups(samples: dict[str, dict[str, Any]], val_fraction: float,
                 test_fraction: float) -> dict[str, list[str]]:
    """Детерминированный поиск группового разбиения с покрытием классов.

    Метки нужны только для стратификации, не для подбора модели на test.
    Если редкий класс встречается менее чем в трёх группах, это отражается
    в classCoverage, а не скрывается синтетическими дубликатами.
    """
    if not 0 < val_fraction < 0.5 or not 0 <= test_fraction < 0.5 or val_fraction + test_fraction >= 0.8:
        raise ValueError('нужны 0 < val < 0.5, 0 <= test < 0.5 и val + test < 0.8')
    groups: dict[str, list[str]] = defaultdict(list)
    total_counts: Counter[str] = Counter()
    for name, sample in samples.items():
        groups[sample['group']].append(name)
        total_counts.update(sample['classes'])
    if len(groups) < (3 if test_fraction else 2):
        raise ValueError('недостаточно независимых групп для train/val/test')
    fractions = {'train': 1 - val_fraction - test_fraction, 'val': val_fraction}
    if test_fraction:
        fractions['test'] = test_fraction
    class_groups = {cls: sum(any(cls in samples[n]['classes'] for n in names)
                             for names in groups.values()) for cls in total_counts}
    best_score, best_parts = float('inf'), None
    for seed in range(512):
        parts: dict[str, list[str]] = {part: [] for part in fractions}
        for group, names in sorted(groups.items()):
            value = int(_sha256(f'{seed}:{group}'.encode())[:8], 16) / 0x100000000
            part = 'train' if value < fractions['train'] else (
                'val' if value < fractions['train'] + val_fraction else 'test')
            parts[part].extend(names)
        score = 0.0
        for part, names in parts.items():
            fraction = fractions[part]
            score += 5 * (len(names) / len(samples) - fraction) ** 2
            if not names:
                score += 100
            counts = Counter(cls for name in names for cls in samples[name]['classes'])
            for cls, total in total_counts.items():
                score += (counts[cls] / total - fraction) ** 2
                if not counts[cls] and (part == 'train' or class_groups[cls] >= len(parts)):
                    score += 20
        if score < best_score:
            best_score, best_parts = score, parts
    assert best_parts is not None
    return {part: sorted(names) for part, names in best_parts.items()}


def _reviewed_detections(items: list[dict]) -> list[Detection]:
    output = []
    for item in items:
        cls = item['class']
        signal = item.get('signal')
        if cls.startswith('traffic_light_'):
            signal, cls = cls.removeprefix('traffic_light_'), 'traffic_light'
        box = item['box']
        if isinstance(box, dict):
            box = [box[key] for key in ('x', 'y', 'w', 'h')]
        x, y, w, h = map(float, box)
        if not all(math.isfinite(v) for v in (x, y, w, h)) or not (
                0 <= x < 1 and 0 <= y < 1 and 0 < w <= 1 - x + 1e-6 and 0 < h <= 1 - y + 1e-6):
            raise ValueError(f'неверная проверенная рамка: {box}')
        if signal not in (None, 'unknown', 'red', 'yellow', 'green'):
            raise ValueError(f'неизвестный сигнал: {signal}')
        output.append(Detection(cls, (x, y, w, h), 1.0, signal, source='reviewed'))
    return output


def build(*, zip_path: Path | None, out: Path, val_fraction: float = 0.15,
          no_download: bool = False, min_confidence: float = 0.5,
          test_fraction: float = 0.15, video_fps: float = 1.0,
          annotations_path: Path | None = None, phash_distance: int = 8) -> dict[str, Any]:
    if not cv2_available():
        raise SystemExit('нужен OpenCV: pip install -r requirements-vision.txt')
    import cv2
    import numpy as np
    if not 0 <= video_fps <= 5 or not 0 <= phash_distance <= 16:
        raise ValueError('video-fps: 0..5; phash-distance: 0..16')
    out.mkdir(parents=True, exist_ok=True)
    archive = zip_path or (out / ARCHIVE_NAME if no_download else download_archive(out))
    if not archive.is_file():
        raise SystemExit('архив не найден: укажите --zip или разрешите скачивание')
    files, videos = read_media(archive)
    source_photo_count = len(files)
    raw = out / 'raw'
    raw.mkdir(exist_ok=True)
    origins: dict[str, dict[str, Any]] = {
        name: {'source': name, 'kind': 'photo'} for name in files
    }
    video_info = []
    for name, data in sorted(videos.items()):
        if not video_fps:
            break
        path = raw / name
        path.write_bytes(data)
        capture = cv2.VideoCapture(str(path))
        fps = capture.get(cv2.CAP_PROP_FPS)
        count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        if not capture.isOpened() or fps <= 0 or not math.isfinite(fps):
            capture.release()
            raise ValueError(f'не удалось прочитать видео: {name}')
        extracted = 0
        for frame_number in range(0, count, max(1, round(fps / video_fps))):
            capture.set(cv2.CAP_PROP_POS_FRAMES, frame_number)
            ok, frame = capture.read()
            if not ok:
                continue
            target = f'{Path(name).stem}_f{frame_number:06d}.jpg'
            ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
            if not ok:
                continue
            if target in files:
                raise ValueError(f'коллизия имени кадра: {target}')
            files[target] = jpeg.tobytes()
            origins[target] = {'source': name, 'kind': 'video', 'frame': frame_number,
                               'timeS': round(frame_number / fps, 4)}
            extracted += 1
        capture.release()
        video_info.append({'name': name, 'sha256': _sha256(data), 'fps': fps,
                           'frames': count, 'extracted': extracted})
    if not files:
        raise ValueError('в источнике нет изображений')

    reviewed_path = annotations_path or out / 'reviewed_annotations.json'
    reviewed = json.loads(reviewed_path.read_text(encoding='utf-8')) if reviewed_path.is_file() else {}
    if not isinstance(reviewed, dict):
        raise ValueError('annotations: ожидается {имя_кадра: [рамки]}')
    # Исправление под именем удаляемого SHA-дубликата относится и к
    # канонической копии. Иначе дедупликация молча теряла ручные правки.
    reviews_by_hash: dict[str, list[dict]] = {}
    for name, items in reviewed.items():
        if name not in files:
            continue
        digest = _sha256(files[name])
        if digest in reviews_by_hash and reviews_by_hash[digest] != items:
            raise ValueError(f'противоречивые исправления точных дубликатов: {name}')
        reviews_by_hash[digest] = items
    seen: dict[str, str] = {}
    samples: dict[str, dict[str, Any]] = {}
    annotations = {}
    labels = {}
    duplicates = {}
    rejected = []
    counts: Counter[str] = Counter()
    signals: Counter[str] = Counter()
    for name, data in sorted(files.items()):
        digest = _sha256(data)
        if digest in seen:
            duplicates[name] = seen[digest]
            continue
        seen[digest] = name
        frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            rejected.append(name)
            continue
        (raw / name).write_bytes(data)
        dets = _reviewed_detections(reviews_by_hash[digest]) if digest in reviews_by_hash else propose(frame, min_confidence)
        rows, items, classes = [], [], []
        for det in dets:
            cls = f'traffic_light_{det.color}' if det.cls == 'traffic_light' and det.color in (
                'red', 'yellow', 'green') else det.cls
            x, y, w, h = det.box
            rows.append(f'{TRAINING_CLASS_NAMES.index(cls)} {x + w / 2:.6f} {y + h / 2:.6f} {w:.6f} {h:.6f}')
            items.append({'class': det.cls, 'trainingClass': cls, 'box': list(det.box),
                          'signal': det.color, 'source': det.source})
            classes.append(cls)
            if det.cls == 'traffic_light':
                signals.update([det.color or 'unknown'])
        labels[name] = '\n'.join(rows) + ('\n' if rows else '')
        annotations[name] = items
        samples[name] = {**origins[name], 'sha256': digest, '_phash': perceptual_hash(frame),
                         'classes': classes, 'annotationSource': 'reviewed' if digest in reviews_by_hash else 'bootstrap'}
        counts.update(classes)
    if not samples:
        raise ValueError('нет декодируемых кадров')
    group_samples(samples, phash_distance)
    parts = split_groups(samples, val_fraction, test_fraction)
    for part in ('train', 'val', 'test'):
        # Ultralytics хэширует размеры/пути, а замена класса или координаты
        # может не изменить размер .txt. Не оставляем устаревший label cache.
        (out / 'labels' / f'{part}.cache').unlink(missing_ok=True)
        for kind in ('images', 'labels'):
            folder = out / kind / part
            folder.mkdir(parents=True, exist_ok=True)
            for old in folder.iterdir():
                if old.is_file():
                    old.unlink()
        for name in parts.get(part, []):
            shutil.copy2(raw / name, out / 'images' / part / name)
            (out / 'labels' / part / f'{Path(name).stem}.txt').write_text(labels[name], encoding='utf-8')
            samples[name]['split'] = part
    for sample in samples.values():
        sample['phash'] = f"{sample.pop('_phash'):016x}"
    try:
        relative = out.resolve().relative_to(ROOT).as_posix()
    except ValueError:
        relative = out.resolve().as_posix()
    yaml = '# scripts/vision_dataset.py; первые пять id совместимы со старыми весами\n'
    yaml += f'path: {relative}\ntrain: images/train\nval: images/val\n'
    if test_fraction:
        yaml += 'test: images/test\n'
    yaml += 'names:\n' + ''.join(f'  {i}: {name}\n' for i, name in enumerate(TRAINING_CLASS_NAMES))
    (out / 'data.yaml').write_text(yaml, encoding='utf-8')
    archive_hash = _sha256(archive.read_bytes())
    manifest = {
        'schemaVersion': SCHEMA_VERSION,
        'source': SOURCE_REPOSITORY if archive_hash in KNOWN_ARCHIVES else 'local archive',
        'sourceRevision': SOURCE_REVISION if archive_hash in KNOWN_ARCHIVES else None,
        'archive': archive.name, 'archiveSha256': archive_hash,
        'sourcePhotos': source_photo_count, 'sourceVideos': len(videos),
        'videoFps': video_fps, 'videos': video_info,
        'images': len(samples), **{part: len(names) for part, names in parts.items()},
        'duplicatesRemoved': len(duplicates), 'rejected': rejected,
        'valFraction': val_fraction, 'testFraction': test_fraction,
        'classes': list(TRAINING_CLASS_NAMES), 'counts': dict(counts),
        'trafficLightSignals': dict(signals),
        'splitStrategy': 'whole-video + transitive pHash groups; stratified hash seed search',
        'phashDistance': phash_distance, 'groups': len({s['group'] for s in samples.values()}),
        'classCoverage': {part: dict(Counter(cls for name in names for cls in samples[name]['classes']))
                          for part, names in parts.items()},
        'reviewedImages': sum(s['annotationSource'] == 'reviewed' for s in samples.values()),
        'reviewType': 'annotation overrides; shipped corrections are agent-assisted, not independent human ground truth',
        'autoLabel': 'scripts.vision_bootstrap.propose; pseudo-labels, NOT ground truth',
        'minConfidence': min_confidence,
        'warning': 'Metrics on bootstrap labels measure agreement with heuristics, not real-world safety. '
                   'Inspect annotations and record an independent robot-camera test before deployment.',
    }
    for filename, value in (('dataset.json', manifest), ('annotations.json', annotations),
                            ('samples.json', samples), ('duplicates.json', duplicates)):
        (out / filename).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(manifest, ensure_ascii=False, indent=2), flush=True)
    return manifest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--zip', type=Path)
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT)
    parser.add_argument('--val-fraction', type=float, default=0.15)
    parser.add_argument('--test-fraction', type=float, default=0.15)
    parser.add_argument('--video-fps', type=float, default=1)
    parser.add_argument('--phash-distance', type=int, default=8)
    parser.add_argument('--annotations', type=Path, help='проверенные рамки заменяют авторазметку')
    parser.add_argument('--no-download', action='store_true')
    parser.add_argument('--min-confidence', type=float, default=0.5)
    args = parser.parse_args(argv)
    try:
        build(zip_path=args.zip, out=args.out, val_fraction=args.val_fraction,
              test_fraction=args.test_fraction, video_fps=args.video_fps,
              phash_distance=args.phash_distance, annotations_path=args.annotations,
              no_download=args.no_download, min_confidence=args.min_confidence)
    except ValueError as exc:
        parser.error(str(exc))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
