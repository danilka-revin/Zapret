#!/usr/bin/env python3
"""Training-only cleanup for separate five/eight-class recognition candidates.

Never repartitions source families or changes the canonical dataset/test.
All removals/crops are logged. This is agent-assisted, not independent GT.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot_control.vision.classes import CLASS_NAMES, TRAINING_CLASS_NAMES


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def fold_label(line: str) -> str:
    values = line.split()
    cls = int(values[0])
    if cls in (5, 6, 7):
        values[0] = '3'
    return ' '.join(values)


def plausible_cone_crop(roi) -> bool:
    """Conservative training-label filter, NEVER a ground-truth evaluator."""
    import cv2
    import numpy as np
    if roi.size == 0:
        return False
    h, w = roi.shape[:2]
    if h < 22 or w < 4:
        return True  # Do not silently erase tiny objects based on color noise.
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    orange = (hsv[..., 0] <= 25) & (hsv[..., 1] >= 105) & (hsv[..., 2] >= 105)
    white = (hsv[..., 1] <= 75) & (hsv[..., 2] >= 130)
    return h / max(w, 1) >= 1.45 and float(orange.mean()) >= .06 and float(white.mean()) >= .07


def build(source: Path, output: Path, review_file: Path, *, keep_color_classes: bool = False) -> dict:
    import cv2
    import yaml
    source, output = source.resolve(), output.resolve()
    if output == source or output in source.parents or output.parent != ROOT / 'datasets':
        raise ValueError('output must be a separate sibling dataset directory')
    existing_manifest = output/'dataset.json'
    known_variants = {'five-class-localizer-training-only-cleanup', 'eight-class-reviewed-signals-training-only'}
    if ((output/'data.yaml').exists() or (output/'samples.json').exists()) and (
        not existing_manifest.is_file() or json.loads(existing_manifest.read_text()).get('variant') not in known_variants
    ):
        raise ValueError('output already contains an unrelated dataset; refusing to overwrite it')
    from scripts.train_vision import inspect_dataset
    _, parent_audit = inspect_dataset(source/'data.yaml')
    samples = json.loads((source/'samples.json').read_text())
    parent_manifest_file = source/'dataset.json'
    parent_manifest = json.loads(parent_manifest_file.read_text()) if parent_manifest_file.is_file() else {}
    review = json.loads(review_file.read_text())
    names = TRAINING_CLASS_NAMES if keep_color_classes else CLASS_NAMES
    signal_overrides = review.get('trainSignalClasses', {})
    if any(name not in samples or samples[name]['split'] != 'train' or cls not in (3,5,6,7)
           for name, cls in signal_overrides.items()):
        raise ValueError('Signal overrides must name TRAIN sources and a traffic class ID')
    for crop in review['negativeCrops']:
        sample = samples.get(crop['image'])
        if not sample or sample['split'] != 'train':
            raise ValueError('negative crop source must be TRAIN')
        if digest((source/'images/train'/crop['image']).read_bytes()) != sample['sha256']:
            raise ValueError(f"negative crop source pixels changed: {crop['image']}")
    keep_cones = {(entry['image'], entry['row']) for entry in review.get('keepTrainConeLabels', [])}
    for name, row in keep_cones:
        if name not in samples or samples[name]['split'] != 'train' or not isinstance(row, int) or row < 0:
            raise ValueError('Cone retention reviews must name a TRAIN image/row')
        lines = (source/'labels/train'/Path(name).with_suffix('.txt')).read_text().splitlines()
        if row >= len(lines) or lines[row].split()[0] != '4':
            raise ValueError('Reviewed cone row does not exist or has another class')
    excluded = set(review['excludeTrainImages'])
    if any(name not in samples or samples[name]['split'] != 'train' for name in excluded):
        raise ValueError('Only original TRAIN images may be excluded')
    records, cleanup, counts = {}, [], {'train': Counter(), 'val': Counter()}
    for split in ('train', 'val'):
        for folder in ('images', 'labels'):
            (output/folder/split).mkdir(parents=True, exist_ok=True)
        for name, sample in sorted(samples.items()):
            if sample['split'] != split or name in excluded:
                continue
            payload = (source/'images'/split/name).read_bytes()
            if digest(payload) != sample['sha256']:
                raise ValueError(f'source pixels changed: {name}')
            target = output/'images'/split/name
            target.write_bytes(payload)
            lines = (source/'labels'/split/Path(name).with_suffix('.txt')).read_text().splitlines()
            frame = cv2.imread(str(target))
            h, w = frame.shape[:2]
            kept = []
            for index, line in enumerate(lines):
                if not line.strip():
                    continue
                cls, cx, cy, bw, bh = map(float, line.split())
                if split == 'train' and int(cls) == 4 and (name, index) not in keep_cones:
                    x1, y1 = max(0, int((cx-bw/2)*w)), max(0, int((cy-bh/2)*h))
                    x2, y2 = min(w, math.ceil((cx+bw/2)*w)), min(h, math.ceil((cy+bh/2)*h))
                    if not plausible_cone_crop(frame[y1:y2, x1:x2]):
                        cleanup.append({'image': name, 'row': index, 'original': line,
                                        'reason': 'train-only orange/white shape quality filter; not independent GT'})
                        continue
                if int(cls) in (3, 5, 6, 7) and name in signal_overrides:
                    parts = line.split(); parts[0] = str(signal_overrides[name]); line = ' '.join(parts)
                kept.append(line if keep_color_classes else fold_label(line))
            text = '\n'.join(kept) + ('\n' if kept else '')
            (output/'labels'/split/Path(name).with_suffix('.txt')).write_text(text)
            for line in kept:
                counts[split][names[int(line.split()[0])]] += 1
            records[name] = {**sample, 'classes': [names[int(s.split()[0])] for s in kept],
                             'parentImage': name, 'annotationSource': 'train-only-filter-and-reviewed-signal' if name in signal_overrides else 'filtered-source-labels'}
    # Explicitly reviewed target-free crops. Parent families stay in TRAIN.
    for number, crop in enumerate(review['negativeCrops']):
        name = crop['image']; sample = samples[name]
        if sample['split'] != 'train':
            raise ValueError('negative crop source must be TRAIN')
        frame = cv2.imread(str(source/'images'/'train'/name))
        h, w = frame.shape[:2]
        x, y, bw, bh = crop['box']
        if not (0 <= x < 1 and 0 <= y < 1 and 0 < bw <= 1-x+1e-6 and 0 < bh <= 1-y+1e-6):
            raise ValueError('negative crop must stay within its TRAIN image')
        roi = frame[round(y*h):round((y+bh)*h), round(x*w):round((x+bw)*w)]
        if roi.size == 0:
            raise ValueError(f'empty negative crop: {crop}')
        ok, jpeg = cv2.imencode('.jpg', roi, [cv2.IMWRITE_JPEG_QUALITY, 92])
        if not ok:
            raise ValueError(f'empty negative crop: {crop}')
        filename = f'negative-{number:02}-{Path(name).stem}.jpg'
        payload = jpeg.tobytes()
        (output/'images'/'train'/filename).write_bytes(payload)
        (output/'labels'/'train'/Path(filename).with_suffix('.txt')).write_text('')
        records[filename] = {'source': sample['source'], 'kind': 'train-negative-crop',
                             'parentImage': name, 'crop': crop['box'], 'group': sample['group'],
                             'split': 'train', 'classes': [], 'sha256': digest(payload),
                             'annotationSource': 'agent-reviewed-negative', 'review': crop['reason']}
    # Reweight the scarce traffic body using a bounded number of train-only crops.
    positives = 0
    crop_counts = Counter()
    # Balanced scarce signal crops instead of taking only the first file names.
    for name, sample in sorted(samples.items()):
        if sample['split'] != 'train' or name in excluded or positives >= 40:
            continue
        lines = (output/'labels'/'train'/Path(name).with_suffix('.txt')).read_text().splitlines()
        frame = cv2.imread(str(source/'images'/'train'/name)); h, w = frame.shape[:2]
        for line in lines:
            cls, cx, cy, bw, bh = map(float, line.split())
            if int(cls) not in (3,5,6,7) or bh*h < 70 or not 1.5 <= bh*h/(bw*w) <= 5.5:
                continue
            if keep_color_classes and crop_counts[int(cls)] >= 12:
                continue
            x1 = max(0, int((cx-bw*.65)*w)); x2 = min(w, math.ceil((cx+bw*.65)*w))
            y1 = max(0, int((cy-bh*.56)*h)); y2 = min(h, math.ceil((cy+bh*.56)*h))
            roi = frame[y1:y2, x1:x2]
            ok, jpeg = cv2.imencode('.jpg', roi, [cv2.IMWRITE_JPEG_QUALITY, 92])
            if not ok:
                continue
            filename = f'traffic-crop-{positives:02}-{Path(name).stem}.jpg'
            # Transform every intersecting source label, not just the chosen light.
            transformed = []
            for original in lines:
                c, a, b, cw, ch = map(float, original.split())
                left, right = max(x1,(a-cw/2)*w), min(x2,(a+cw/2)*w)
                top, bottom = max(y1,(b-ch/2)*h), min(y2,(b+ch/2)*h)
                if right-left < 2 or bottom-top < 2:
                    continue
                transformed.append(f'{int(c)} {((left+right)/2-x1)/(x2-x1):.6f} {((top+bottom)/2-y1)/(y2-y1):.6f} {(right-left)/(x2-x1):.6f} {(bottom-top)/(y2-y1):.6f}')
            payload = jpeg.tobytes()
            (output/'images'/'train'/filename).write_bytes(payload)
            (output/'labels'/'train'/Path(filename).with_suffix('.txt')).write_text('\n'.join(transformed)+'\n')
            for s in transformed:
                counts['train'][names[int(s.split()[0])]] += 1
            records[filename] = {'source': sample['source'], 'kind': 'train-traffic-crop', 'parentImage':name,
                                 'cropPixels': [x1,y1,x2,y2], 'group': sample['group'], 'split':'train',
                                 'sha256':digest(payload), 'classes':[names[int(s.split()[0])] for s in transformed],
                                 'annotationSource':'transformed-source-labels'}
            positives += 1
            crop_counts[int(cls)] += 1
            break
    # Ultralytics caches by file sizes/paths: replacing a class ID can keep
    # exactly the same byte count. Always invalidate caches after a rebuild.
    for split in ('train', 'val', 'test'):
        (output/'labels'/f'{split}.cache').unlink(missing_ok=True)
    # Remove stale generated images/labels only within this dedicated output.
    for split in ('train','val'):
        valid = {n for n,s in records.items() if s['split']==split}
        for path in (output/'images'/split).iterdir():
            if path.name not in valid:
                path.unlink()
        valid_labels = {str(Path(n).with_suffix('.txt')) for n in valid}
        for path in (output/'labels'/split).glob('*.txt'):
            if path.name not in valid_labels:
                path.unlink()
    config = {'path': str(output.relative_to(ROOT)), 'train':'images/train', 'val':'images/val',
              'names':dict(enumerate(names))}
    (output/'data.yaml').write_text(yaml.safe_dump(config,sort_keys=False,allow_unicode=True))
    (output/'samples.json').write_text(json.dumps(records,ensure_ascii=False,indent=2)+'\n')
    (output/'cleanup.json').write_text(json.dumps(cleanup,ensure_ascii=False,indent=2)+'\n')
    _, audit = inspect_dataset(output/'data.yaml')
    manifest = {'schemaVersion':3,
                'variant':'eight-class-reviewed-signals-training-only' if keep_color_classes else 'five-class-localizer-training-only-cleanup',
                'parentFingerprint':parent_audit['fingerprint'],
                'fingerprint':audit['fingerprint'], 'sourceRevision':parent_manifest.get('sourceRevision'),
                'archiveSha256':parent_manifest.get('archiveSha256'),
                'reviewSha256':digest(review_file.read_bytes()),
                'recipeSha256':digest(Path(__file__).read_bytes()),
                'excludedTrainImages':sorted(excluded),'removedTrainConeLabels':len(cleanup),
                'negativeCrops':len(review['negativeCrops']),'trafficCrops':positives,
                'trafficCropCounts':dict(crop_counts), 'reviewedTrainSignalImages':len(signal_overrides),
                'keepColorClasses':keep_color_classes, 'retainedReviewedConeLabels':len(keep_cones),
                'splits':audit['splits'],'test':'Original canonical test is untouched and absent from this training variant.',
                'warning':'Agent-assisted filtering and TRAIN-only signal review of pre-existing pseudo-labels, NOT independent human ground truth. Val has no coordinate/instance edits; color IDs are folded only for the five-class variant.'}
    (output/'dataset.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(manifest,ensure_ascii=False,indent=2))
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=ROOT/'datasets/yolo-stop')
    parser.add_argument('--output',type=Path,default=ROOT/'datasets/yolo-stop-refined')
    parser.add_argument('--review',type=Path,default=ROOT/'datasets/yolo-stop-refined/review.json')
    parser.add_argument('--keep-color-classes', action='store_true', help='keep the learned signal head; no relaxed green agreement')
    args=parser.parse_args();build(args.source,args.output,args.review,keep_color_classes=args.keep_color_classes)


if __name__ == '__main__':
    main()
