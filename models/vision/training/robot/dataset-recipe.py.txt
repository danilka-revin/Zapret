#!/usr/bin/env python3
"""Build a separate robot-training variant from the reviewed colour dataset.

Only TRAIN housing coordinates and explicitly reviewed TRAIN-video negatives
may change. VAL is copied byte-for-byte; TEST is neither copied nor consumed.
No model predictions are used as truth. Reviews are agent-assisted, not human GT.
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
from robot_control.vision.classes import TRAINING_CLASS_NAMES
from scripts.train_vision import inspect_dataset

VARIANT = 'robot-reviewed-housings-and-train-backgrounds-v1'


def sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def checked_box(box) -> tuple[float, float, float, float]:
    if not isinstance(box, (list, tuple)) or len(box) != 4:
        raise ValueError('box must be [x,y,w,h]')
    x, y, w, h = map(float, box)
    if not all(math.isfinite(v) for v in (x, y, w, h)) or not (
        0 <= x < 1 and 0 <= y < 1 and w > 0 and h > 0 and x+w <= 1+1e-7 and y+h <= 1+1e-7
    ):
        raise ValueError('box must stay inside its image')
    return x, y, w, h


def label_line(cls: int, box) -> str:
    x, y, w, h = checked_box(box)
    return f'{cls} {x+w/2:.6f} {y+h/2:.6f} {w:.6f} {h:.6f}'


def crop_labels(lines: list[str], pixels, width: int, height: int) -> list[str]:
    """Transform ALL intersecting labels, including partially visible cones/signs."""
    x1, y1, x2, y2 = pixels
    transformed = []
    for line in lines:
        cls, cx, cy, w, h = map(float, line.split())
        left, right = max(x1, (cx-w/2)*width), min(x2, (cx+w/2)*width)
        top, bottom = max(y1, (cy-h/2)*height), min(y2, (cy+h/2)*height)
        if right-left < 2 or bottom-top < 2:
            continue
        transformed.append(label_line(int(cls), [(left-x1)/(x2-x1), (top-y1)/(y2-y1),
                                                (right-left)/(x2-x1), (bottom-top)/(y2-y1)]))
    return transformed


def build(source: Path, output: Path, review_file: Path, video_dir: Path) -> dict:
    import cv2
    import yaml
    source, output = source.resolve(), output.resolve()
    if source == output or source in output.parents or output in source.parents or output.parent != ROOT/'datasets':
        raise ValueError('output must be a separate sibling dataset')
    if output.exists() and any(output.iterdir()):
        manifest_file = output/'dataset.json'
        # review.json / README may be prepared before the first build.
        if not manifest_file.is_file():
            if any(p.name not in {'review.json', 'README.md'} for p in output.iterdir()):
                raise ValueError('output contains an unrelated dataset')
        elif json.loads(manifest_file.read_text()).get('variant') != VARIANT:
            raise ValueError('output contains an unrelated dataset')
    _, parent_audit = inspect_dataset(source/'data.yaml')
    samples = json.loads((source/'samples.json').read_text())
    parent = json.loads((source/'dataset.json').read_text())
    review = json.loads(review_file.read_text())
    if review.get('parentFingerprint') != parent_audit['fingerprint']:
        raise ValueError('review parent fingerprint changed')
    if tuple(parent_audit['classes']) != TRAINING_CLASS_NAMES:
        raise ValueError('eight colour classes required')
    # Preflight BEFORE mutating output. Source and family assignment are immutable.
    label_map, frames = {}, {}
    for name, sample in samples.items():
        split = sample['split']
        if split not in ('train', 'val') or Path(name).name != name:
            raise ValueError('source variant must contain TRAIN/VAL only and safe filenames')
        payload = (source/'images'/split/name).read_bytes()
        if sha256(payload) != sample['sha256']:
            raise ValueError(f'source pixels changed: {name}')
        label_map[name] = (source/'labels'/split/Path(name).with_suffix('.txt')).read_text().splitlines()
    seen = set()
    for entry in review['housingBoxes']:
        name, row = entry['image'], entry['row']
        sample = samples.get(name, {})
        if sample.get('split') != 'train' or sample.get('kind') not in ('photo', 'video'):
            raise ValueError('housing corrections require an original TRAIN image')
        if type(row) is not int or row < 0 or row >= len(label_map[name]) or (name, row) in seen:
            raise ValueError('invalid or duplicate reviewed label row')
        seen.add((name, row))
        original = label_map[name][row]
        if entry.get('original') != original or int(original.split()[0]) not in (3, 5, 6, 7):
            raise ValueError('reviewed original traffic row changed')
        if entry.get('approved') is not True or not entry.get('reason'):
            raise ValueError('unapproved housing proposal')
        label_map[name][row] = label_line(int(original.split()[0]), entry['box'])
    source_groups = {}
    for sample in samples.values():
        if sample.get('kind') == 'video':
            source_groups.setdefault(sample['source'], set()).add((sample['split'], sample['group']))
    video_hashes, negatives_seen = {}, set()
    for entry in review['videoNegatives']:
        name, frame = entry['source'], entry['frame']
        if Path(name).name != name or not name.endswith('.mp4'):
            raise ValueError('video source must be a safe MP4 basename')
        if source_groups.get(name) != {('train', entry.get('group'))}:
            raise ValueError('negative video must belong exclusively to its original TRAIN family')
        if type(frame) is not int or frame < 0:
            raise ValueError('negative frame must be a nonnegative integer')
        box = checked_box(entry['box'])
        identity = (name, frame, box)
        if identity in negatives_seen:
            raise ValueError('duplicate negative crop')
        negatives_seen.add(identity)
        if entry.get('approved') is not True or not entry.get('reason'):
            raise ValueError('unapproved negative proposal')
        if name not in video_hashes:
            video_hashes[name] = sha256((video_dir/name).read_bytes())
        if video_hashes[name] != entry.get('sourceSha256'):
            raise ValueError('negative video source SHA changed')
    # Decode each video once, in original frame order, independent of model output.
    for name in video_hashes:
        cap = cv2.VideoCapture(str(video_dir/name))
        wanted = {entry['frame'] for entry in review['videoNegatives'] if entry['source'] == name}
        index = 0
        try:
            while wanted:
                ok, frame = cap.read()
                if not ok:
                    raise ValueError(f'could not decode reviewed frame from {name}')
                if index in wanted:
                    frames[name, index] = frame
                    wanted.remove(index)
                index += 1
        finally:
            cap.release()
    records, changes = {}, []
    for split in ('train', 'val'):
        for folder in ('images', 'labels'):
            (output/folder/split).mkdir(parents=True, exist_ok=True)
    for name, sample in sorted(samples.items()):
        split = sample['split']
        payload = (source/'images'/split/name).read_bytes()
        lines = label_map[name]
        record = dict(sample)
        if sample.get('kind') == 'train-traffic-crop':
            parent_name = sample['parentImage']
            if split != 'train' or samples[parent_name]['split'] != 'train' or samples[parent_name]['group'] != sample['group']:
                raise ValueError('traffic crop parent must remain in its TRAIN family')
            image = cv2.imread(str(source/'images/train'/parent_name))
            height, width = image.shape[:2]
            parent_lines = label_map[parent_name]
            traffic = [line for line in parent_lines if int(line.split()[0]) in (3,5,6,7)]
            if len(traffic) != 1:
                raise ValueError('traffic recrop requires exactly one reviewed parent housing')
            _, cx, cy, w, h = map(float, traffic[0].split())
            x1, x2 = max(0, int((cx-w*.65)*width)), min(width, math.ceil((cx+w*.65)*width))
            y1, y2 = max(0, int((cy-h*.56)*height)), min(height, math.ceil((cy+h*.56)*height))
            ok, jpeg = cv2.imencode('.jpg', image[y1:y2,x1:x2], [cv2.IMWRITE_JPEG_QUALITY,92])
            if not ok:
                raise ValueError('cannot encode corrected traffic crop')
            payload = jpeg.tobytes()
            lines = crop_labels(parent_lines, (x1,y1,x2,y2), width, height)
            record.update(cropPixels=[x1,y1,x2,y2], annotationSource='transformed-reviewed-train-housings')
        text = '\n'.join(lines) + ('\n' if lines else '')
        original_text = (source/'labels'/split/Path(name).with_suffix('.txt')).read_text()
        if split == 'val':
            # No normalization, roundoff, coordinate, class or instance edits.
            text = original_text
        if text != original_text:
            changes.append({'image':name,'split':split,'before':original_text,'after':text})
            record['annotationSource'] = 'agent-reviewed-train-housing-coordinates'
        (output/'images'/split/name).write_bytes(payload)
        (output/'labels'/split/Path(name).with_suffix('.txt')).write_text(text)
        record.update(sha256=sha256(payload), classes=[TRAINING_CLASS_NAMES[int(s.split()[0])] for s in lines])
        records[name] = record
    for number, entry in enumerate(review['videoNegatives']):
        image = frames[entry['source'], entry['frame']]
        height, width = image.shape[:2]
        x,y,w,h = checked_box(entry['box'])
        roi = image[round(y*height):round((y+h)*height),round(x*width):round((x+w)*width)]
        if roi.size == 0:
            raise ValueError('empty reviewed negative crop')
        ok, jpeg = cv2.imencode('.jpg', roi, [cv2.IMWRITE_JPEG_QUALITY,92])
        if not ok:
            raise ValueError('cannot encode reviewed negative')
        payload = jpeg.tobytes()
        name = f'robot-negative-{number:03}-{Path(entry["source"]).stem}-f{entry["frame"]:06}.jpg'
        (output/'images/train'/name).write_bytes(payload)
        (output/'labels/train'/Path(name).with_suffix('.txt')).write_text('')
        records[name] = {**entry, 'kind':'train-reviewed-video-negative', 'split':'train',
                         'sha256':sha256(payload),'classes':[], 'annotationSource':'agent-visually-reviewed-target-free-crop'}
    for split in ('train','val'):
        valid = {n for n,s in records.items() if s['split']==split}
        for file in (output/'images'/split).iterdir():
            if file.name not in valid:
                file.unlink()
        valid_labels = {Path(n).with_suffix('.txt').name for n in valid}
        for file in (output/'labels'/split).glob('*.txt'):
            if file.name not in valid_labels:
                file.unlink()
    for split in ('train','val','test'):
        (output/'labels'/f'{split}.cache').unlink(missing_ok=True)
    config = {'path':str(output.relative_to(ROOT)),'train':'images/train','val':'images/val',
              'names':dict(enumerate(TRAINING_CLASS_NAMES))}
    (output/'data.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    (output/'samples.json').write_text(json.dumps(records,ensure_ascii=False,indent=2)+'\n')
    (output/'changes.json').write_text(json.dumps(changes,ensure_ascii=False,indent=2)+'\n')
    _, audit = inspect_dataset(output/'data.yaml')
    _, after = inspect_dataset(source/'data.yaml')
    if parent_audit['fingerprint'] != after['fingerprint']:
        raise RuntimeError('source changed during build')
    manifest = {'schemaVersion':4,'variant':VARIANT,'parentFingerprint':parent_audit['fingerprint'],
                'fingerprint':audit['fingerprint'],'sourceRevision':parent['sourceRevision'],
                'archiveSha256':parent.get('archiveSha256'),'reviewSha256':sha256(review_file.read_bytes()),
                'recipeSha256':sha256(Path(__file__).read_bytes()),'reviewedHousingBoxes':len(seen),
                'videoNegatives':len(review['videoNegatives']),'videoSourceHashes':video_hashes,
                'changedLabels':len(changes),'splits':audit['splits'],
                'test':'Absent. Canonical TEST and all VAL pixels/labels unchanged.',
                'warning':'TRAIN-only agent-assisted coordinate review and visually reviewed negatives, not independent human ground truth. Inherits remaining pseudo-label noise and already-reused validation/test from its parent.'}
    (output/'dataset.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    return manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=ROOT/'datasets/yolo-stop-refined-color')
    parser.add_argument('--output',type=Path,default=ROOT/'datasets/yolo-stop-robot')
    parser.add_argument('--review',type=Path,default=ROOT/'datasets/yolo-stop-robot/review.json')
    parser.add_argument('--video-dir',type=Path,default=ROOT/'.cache/vision/video-review')
    args=parser.parse_args()
    print(json.dumps(build(args.source,args.output,args.review,args.video_dir),ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
