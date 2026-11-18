#!/usr/bin/env python3
"""Verify/restore the pinned yolo-stop media WITHOUT rebuilding labels or splits.

    python scripts/restore_vision_source.py --restore-missing
    python scripts/restore_vision_source.py --zip source-repo.zip --no-download

Only known upstream archives are accepted. Every canonical JPEG (including
JPEG-92 video frames) and MP4 must match its saved SHA before any media writes.
Conflicting existing files are never overwritten. No upstream code is run,
no annotations are generated, and no training or model selection takes place.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts import vision_dataset as source  # noqa: E402
from scripts.train_vision import inspect_dataset  # noqa: E402

CACHE = ROOT / '.cache/vision/source'
VIDEO_DIR = ROOT / '.cache/vision/video-review'
REPORT = ROOT / 'docs/vision-source-verification.json'
REPORT_KIND = 'pinned-yolo-stop-source-verification-v1'


def digest(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def basename(value: str) -> str:
    if not isinstance(value, str) or value in ('', '.', '..') or '/' in value or '\\' in value:
        raise ValueError(f'unsafe source filename: {value!r}')
    return value


def metadata_hashes(dataset: Path) -> dict[str, str]:
    files = list(dataset.glob('*.json')) + list(dataset.glob('*.yaml')) + list(dataset.glob('labels/*/*.txt'))
    return {path.relative_to(dataset).as_posix(): digest(path.read_bytes()) for path in sorted(files)}


def media_path(root: Path, *parts: str) -> Path:
    path = root.joinpath(*parts)
    if not path.resolve().is_relative_to(root.resolve()) or any(
        parent.is_symlink() for parent in (path, *path.parents) if parent != root.parent
    ):
        raise ValueError(f'symlink/escaped media destination: {path}')
    return path


def validate_layout(dataset: Path, manifest: dict) -> None:
    import yaml

    config = yaml.safe_load((dataset / 'data.yaml').read_text())
    root = Path(config.get('path', '.'))
    candidates = (root.resolve(),) if root.is_absolute() else (
        (dataset / root).resolve(), (ROOT / root).resolve())
    existing = next((path for path in candidates if (path / 'images/train').is_dir()), dataset)
    if dataset not in candidates or existing != dataset:
        raise ValueError('data.yaml must refer to the snapshot being restored')
    for split in ('train', 'val', 'test'):
        expected = f'images/{split}' if manifest.get(split, 0) else None
        if config.get(split) != expected:
            raise ValueError('data.yaml split layout differs from the saved manifest')
    names = config.get('names')
    if isinstance(names, dict):
        names = [names[index] for index in range(len(names))]
    if names != list(source.TRAINING_CLASS_NAMES) or manifest.get('classes') != names:
        raise ValueError('data.yaml must retain the eight saved training classes')


def verify_source(*, archive: Path, dataset: Path, video_dir: Path,
                  restore_missing: bool = False) -> dict:
    import cv2

    dataset, video_dir = dataset.resolve(), video_dir.absolute()
    if video_dir.resolve().is_relative_to(dataset) or dataset.is_relative_to(video_dir.resolve()):
        raise ValueError('video cache must be separate from the dataset snapshot')
    archive_sha = digest(archive.read_bytes())
    if archive_sha not in source.KNOWN_ARCHIVES:
        raise ValueError('archive SHA-256 is not a pinned p25543770-lang/yolo-stop archive')
    before = metadata_hashes(dataset)
    manifest = json.loads((dataset / 'dataset.json').read_text())
    samples = json.loads((dataset / 'samples.json').read_text())
    if (manifest.get('source') != source.SOURCE_REPOSITORY or
            manifest.get('sourceRevision') != source.SOURCE_REVISION or
            manifest.get('archiveSha256') not in source.KNOWN_ARCHIVES):
        raise ValueError('dataset must retain the pinned upstream source and revision')
    if not samples or manifest['images'] != len(samples):
        raise ValueError('canonical sample count differs from the saved manifest')
    validate_layout(dataset, manifest)
    images, videos = source.read_media(archive)
    if (manifest['sourcePhotos'], manifest['sourceVideos']) != (len(images), len(videos)):
        raise ValueError('upstream media inventory differs from the saved manifest')
    info = {item['name']: item for item in manifest['videos']}
    if len(info) != len(manifest['videos']) or set(info) != set(videos):
        raise ValueError('upstream video list differs from the saved manifest')
    for name, payload in videos.items():
        basename(name)
        if digest(payload) != info[name]['sha256']:
            raise ValueError(f'video SHA-256 mismatch: {name}')

    plan: dict[Path, bytes] = {}
    requested = defaultdict(list)
    split_counts, kinds = Counter(), Counter()
    families, groups = {}, {}
    for name, sample in samples.items():
        basename(name)
        split, kind, origin = sample['split'], sample['kind'], basename(sample['source'])
        if split not in ('train', 'val', 'test'):
            raise ValueError('invalid saved split')
        if groups.setdefault(sample['group'], split) != split:
            raise ValueError('source group leaks across TRAIN/VAL/TEST')
        split_counts[split] += 1
        kinds[kind] += 1
        target = media_path(dataset, 'images', split, name)
        if kind == 'photo':
            if origin != name or origin not in images or digest(images[origin]) != sample['sha256']:
                raise ValueError(f'photo SHA-256/source mismatch: {name}')
            plan[target] = images[origin]
        elif kind == 'video':
            number = sample['frame']
            if (origin not in videos or type(number) is not int or number < 0 or
                    number >= info[origin]['frames'] or
                    name != f'{Path(origin).stem}_f{number:06d}.jpg'):
                raise ValueError(f'invalid saved video frame/source: {name}')
            if families.setdefault(origin, split) != split:
                raise ValueError('one source video leaks across TRAIN/VAL/TEST')
            requested[origin].append((number, name, sample['sha256'], target))
        else:
            raise ValueError(f'not a canonical photo/video sample: {kind}')
    if any(split_counts[part] != manifest.get(part, 0) for part in ('train', 'val', 'test')):
        raise ValueError('split counts differ from the saved manifest')

    # Decode the archived MP4 itself, not an unrelated local video. Temporary
    # media are ignored, never extracted with archive-controlled paths.
    with tempfile.TemporaryDirectory(prefix='yolo-source-') as temporary:
        for name, frames in sorted(requested.items()):
            path = Path(temporary) / name
            path.write_bytes(videos[name])
            capture = cv2.VideoCapture(str(path))
            try:
                fps = capture.get(cv2.CAP_PROP_FPS)
                count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
                if (not capture.isOpened() or count != info[name]['frames'] or
                        not math.isfinite(fps) or not math.isclose(fps, info[name]['fps'], rel_tol=1e-6, abs_tol=1e-5)):
                    raise ValueError(f'video decoder/metadata mismatch: {name}')
                for number, image_name, expected, target in sorted(frames):
                    capture.set(cv2.CAP_PROP_POS_FRAMES, number)
                    ok, frame = capture.read()
                    if not ok:
                        raise ValueError(f'cannot decode saved video frame: {image_name}')
                    ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
                    if not ok or digest(jpeg.tobytes()) != expected:
                        raise ValueError(f'video-frame SHA-256 mismatch: {image_name}; use the recorded OpenCV version')
                    plan[target] = jpeg.tobytes()
            finally:
                capture.release()
    for name, payload in videos.items():
        plan[media_path(video_dir, name)] = payload
    if len(plan) != len(samples) + len(videos):
        raise ValueError('colliding media destinations')

    # Validate ALL source media and existing destinations before media writes.
    missing = []
    for path, payload in plan.items():
        if path.exists():
            if not path.is_file() or digest(path.read_bytes()) != digest(payload):
                raise ValueError(f'existing media conflict; refusing to overwrite: {path}')
        else:
            missing.append(path)
    if missing and not restore_missing:
        raise ValueError(f'{len(missing)} missing media files; use --restore-missing (first: {missing[0]})')
    for path in missing:
        path.parent.mkdir(parents=True, exist_ok=True)
        # Exclusive create also refuses a file/symlink introduced after preflight.
        with path.open('xb') as output:
            output.write(plan[path])
    config, audit = inspect_dataset(dataset / 'data.yaml')
    if Path(config['path']) != dataset:
        raise ValueError('data.yaml resolved to a different dataset')
    if metadata_hashes(dataset) != before:
        raise RuntimeError('labels, splits or provenance unexpectedly changed')
    return {
        'kind': REPORT_KIND,
        'verifiedAt': datetime.now(timezone.utc).isoformat(),
        'source': source.SOURCE_REPOSITORY,
        'sourceRevision': source.SOURCE_REVISION,
        'downloadUrl': source.SOURCE_URL,
        'verifiedArchiveSha256': archive_sha,
        'datasetRecordedArchiveSha256': manifest['archiveSha256'],
        'archiveNote': 'Pinned-commit ZIP and historical main-ref ZIP have different root names/hashes; every restored media byte is verified against the saved sample/video SHA.',
        'sourcePhotos': len(images), 'sourceVideos': len(videos),
        'canonicalPhotosVerified': kinds['photo'], 'canonicalVideoFramesVerified': kinds['video'],
        'canonicalImagesVerified': len(samples), 'splits': dict(sorted(split_counts.items())),
        'datasetFingerprint': audit['fingerprint'],
        'restoredImages': sum(path.is_relative_to(dataset / 'images') for path in missing),
        'restoredVideos': sum(path.is_relative_to(video_dir) for path in missing),
        'sourceMetadataSha256': before,
        'labelsAndSplitsUnchanged': True,
        'trainingOrInferencePerformed': False,
        'warning': 'Source-byte provenance only, not independent annotation truth, accuracy or robot-safety acceptance. No upstream code/weights executed and no production model changed.',
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--zip', type=Path, help='known pinned upstream ZIP, never an arbitrary training archive')
    parser.add_argument('--dataset', type=Path, default=source.DEFAULT_OUT)
    parser.add_argument('--video-dir', type=Path, default=VIDEO_DIR)
    parser.add_argument('--restore-missing', action='store_true', help='create only absent, SHA-verified media')
    parser.add_argument('--no-download', action='store_true')
    parser.add_argument('--report', type=Path, default=REPORT)
    args = parser.parse_args(argv)
    try:
        target = args.report.resolve()
        if (target.suffix != '.json' or target.is_relative_to(args.dataset.resolve()) or
                (target.exists() and json.loads(target.read_text()).get('kind') != REPORT_KIND)):
            raise ValueError('report must be a separate source-verification JSON, not dataset/model metadata')
        archive = args.zip or (CACHE / source.ARCHIVE_NAME if args.no_download else source.download_archive(CACHE))
        result = verify_source(archive=archive, dataset=args.dataset, video_dir=args.video_dir,
                               restore_missing=args.restore_missing)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps({key: value for key, value in result.items() if key != 'sourceMetadataSha256'}, ensure_ascii=False, indent=2))
    except (ValueError, OSError, zipfile.BadZipFile) as exc:
        parser.error(str(exc))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
