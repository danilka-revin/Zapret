"""Pinned upstream media provenance; never rerun annotation/group selection."""
from __future__ import annotations

import io
import json
from pathlib import Path
from types import SimpleNamespace
import zipfile

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')
from scripts import restore_vision_source as restore, vision_dataset as source  # noqa: E402


def jpeg(seed):
    image = np.random.default_rng(seed).integers(0, 256, (48, 64, 3), dtype=np.uint8)
    ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 92])
    assert ok
    return encoded.tobytes()


@pytest.fixture
def snapshot(tmp_path, monkeypatch):
    movie = tmp_path / 'clip.mp4'
    writer = cv2.VideoWriter(str(movie), cv2.VideoWriter_fourcc(*'mp4v'), 5, (64, 48))
    if not writer.isOpened():
        pytest.skip('OpenCV MP4 encoder unavailable')
    for index in range(3):
        image = np.random.default_rng(index + 20).integers(0, 256, (48, 64, 3), dtype=np.uint8)
        writer.write(image)
    writer.release()
    cap = cv2.VideoCapture(str(movie))
    fps, count = cap.get(cv2.CAP_PROP_FPS), int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.set(cv2.CAP_PROP_POS_FRAMES, 1)
    ok, frame = cap.read()
    cap.release()
    assert ok
    ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
    assert ok
    media = {'one.jpg': jpeg(1), 'two.jpg': jpeg(2), 'nested.jpg': jpeg(3),
             'clip_f000001.jpg': encoded.tobytes()}
    nested = io.BytesIO()
    with zipfile.ZipFile(nested, 'w') as archive:
        archive.writestr('old/nested.jpg', media['nested.jpg'])
    upstream = tmp_path / 'upstream.zip'
    with zipfile.ZipFile(upstream, 'w') as archive:
        archive.writestr('repo/one.jpg', media['one.jpg'])
        archive.writestr('repo/two.jpg', media['two.jpg'])
        archive.writestr('repo/old.zip', nested.getvalue())
        archive.writestr('repo/clip.mp4', movie.read_bytes())
        archive.writestr('repo/untrusted.py', 'raise RuntimeError("must never execute")')
    archive_sha = restore.digest(upstream.read_bytes())
    monkeypatch.setattr(source, 'KNOWN_ARCHIVES', {archive_sha})
    monkeypatch.setattr(source, 'propose', lambda *_a, **_k: pytest.fail('no new annotations'))
    monkeypatch.setattr(source, 'group_samples', lambda *_a, **_k: pytest.fail('no new source groups'))
    monkeypatch.setattr(source, 'split_groups', lambda *_a, **_k: pytest.fail('no new split selection'))
    dataset = tmp_path / 'dataset'
    dataset.mkdir()
    samples = {}
    for name, payload in media.items():
        video = name.startswith('clip_')
        split = 'val' if name == 'two.jpg' else 'train'
        samples[name] = {'kind': 'video' if video else 'photo', 'source': 'clip.mp4' if video else name,
                         'sha256': restore.digest(payload), 'split': split, 'group': 'clip' if video else name,
                         **({'frame': 1} if video else {})}
        label = dataset / 'labels' / split / Path(name).with_suffix('.txt')
        label.parent.mkdir(parents=True, exist_ok=True)
        label.write_text('4 0.5 0.5 0.1 0.6\n')
    manifest = {'schemaVersion': 2, 'source': source.SOURCE_REPOSITORY, 'sourceRevision': source.SOURCE_REVISION,
                'archiveSha256': archive_sha, 'sourcePhotos': 3, 'sourceVideos': 1,
                'images': 4, 'train': 3, 'val': 1, 'test': 0,
                'classes': list(source.TRAINING_CLASS_NAMES),
                'videos': [{'name': 'clip.mp4', 'sha256': restore.digest(movie.read_bytes()),
                            'fps': fps, 'frames': count}]}
    (dataset / 'samples.json').write_text(json.dumps(samples))
    (dataset / 'dataset.json').write_text(json.dumps(manifest))
    (dataset / 'reviewed_annotations.json').write_text('{"one.jpg": []}')
    (dataset / 'data.yaml').write_text('path: .\ntrain: images/train\nval: images/val\nnames:\n' +
                                     ''.join(f'  {index}: {name}\n' for index, name in enumerate(source.TRAINING_CLASS_NAMES)))
    return SimpleNamespace(archive=upstream, dataset=dataset, video_dir=tmp_path / 'videos',
                           media=media, samples=samples, manifest=manifest)


def run(snapshot, **kwargs):
    return restore.verify_source(archive=snapshot.archive, dataset=snapshot.dataset,
                                 video_dir=snapshot.video_dir, **kwargs)


def save_samples(snapshot):
    (snapshot.dataset / 'samples.json').write_text(json.dumps(snapshot.samples))


def save_manifest(snapshot):
    (snapshot.dataset / 'dataset.json').write_text(json.dumps(snapshot.manifest))


def assert_no_media(snapshot):
    assert not (snapshot.dataset / 'images').exists()
    assert not snapshot.video_dir.exists()


def test_default_source_is_the_requested_pinned_repository():
    assert source.SOURCE_REPOSITORY == 'https://github.com/p25543770-lang/yolo-stop'
    assert source.SOURCE_REVISION == '3cb277bfadf7d0034943117f9146d4b92bb773a9'
    assert source.SOURCE_URL == ('https://codeload.github.com/p25543770-lang/yolo-stop/zip/' +
                                 source.SOURCE_REVISION)
    assert 'main' not in source.SOURCE_URL
    assert '464f43a9042be1e802745b7f00c8c44074b51d5bea8c4611eb4553afa5f55c70' in source.KNOWN_ARCHIVES


def test_restore_matches_root_nested_and_video_bytes_without_metadata_edits(snapshot):
    before = restore.metadata_hashes(snapshot.dataset)
    result = run(snapshot, restore_missing=True)
    assert result['canonicalPhotosVerified'] == 3 and result['canonicalVideoFramesVerified'] == 1
    assert result['restoredImages'] == 4 and result['restoredVideos'] == 1
    assert result['labelsAndSplitsUnchanged'] and not result['trainingOrInferencePerformed']
    assert restore.metadata_hashes(snapshot.dataset) == before == result['sourceMetadataSha256']
    for name, payload in snapshot.media.items():
        assert (snapshot.dataset / 'images' / snapshot.samples[name]['split'] / name).read_bytes() == payload
    assert (snapshot.video_dir / 'clip.mp4').read_bytes() == (snapshot.archive.parent / 'clip.mp4').read_bytes()
    assert not (snapshot.dataset / 'untrusted.py').exists()
    repeat = run(snapshot)
    assert repeat['restoredImages'] == repeat['restoredVideos'] == 0
    assert repeat['datasetFingerprint'] == result['datasetFingerprint']


def test_verify_only_never_creates_missing_media(snapshot):
    with pytest.raises(ValueError, match='restore-missing'):
        run(snapshot)
    assert_no_media(snapshot)


def test_unknown_archive_is_not_accepted_even_with_matching_dataset_claim(snapshot):
    snapshot.archive.write_bytes(snapshot.archive.read_bytes() + b'changed ZIP')
    snapshot.manifest['archiveSha256'] = restore.digest(snapshot.archive.read_bytes())
    save_manifest(snapshot)
    with pytest.raises(ValueError, match='archive SHA-256'):
        run(snapshot, restore_missing=True)
    assert_no_media(snapshot)


@pytest.mark.parametrize('field,value', [
    ('source', 'https://github.com/unrelated/repo'), ('sourceRevision', 'main'),
    ('archiveSha256', '0' * 64), ('sourcePhotos', 99), ('sourceVideos', 0),
    ('images', 5), ('train', 2), ('classes', ['different']),
])
def test_changed_source_revision_inventory_or_split_is_rejected(snapshot, field, value):
    snapshot.manifest[field] = value
    save_manifest(snapshot)
    with pytest.raises(ValueError):
        run(snapshot, restore_missing=True)
    assert_no_media(snapshot)


@pytest.mark.parametrize('name', ['one.jpg', 'clip_f000001.jpg'])
def test_every_sample_hash_is_checked_before_writes(snapshot, name):
    snapshot.samples[name]['sha256'] = '0' * 64
    save_samples(snapshot)
    with pytest.raises(ValueError, match='SHA-256'):
        run(snapshot, restore_missing=True)
    assert_no_media(snapshot)


def test_video_hash_is_checked_before_writes(snapshot):
    snapshot.manifest['videos'][0]['sha256'] = '0' * 64
    save_manifest(snapshot)
    with pytest.raises(ValueError, match='video SHA-256'):
        run(snapshot, restore_missing=True)
    assert_no_media(snapshot)


@pytest.mark.parametrize('number', [-1, 8, 1.5, True, '1'])
def test_invalid_frame_indices_do_not_restore_another_frame(snapshot, number):
    snapshot.samples['clip_f000001.jpg']['frame'] = number
    save_samples(snapshot)
    with pytest.raises(ValueError, match='video frame/source'):
        run(snapshot, restore_missing=True)
    assert_no_media(snapshot)


@pytest.mark.parametrize('name', ['../escape.jpg', '/escape.jpg', 'dir\\escape.jpg', '..'])
def test_manifest_names_cannot_escape_destinations(snapshot, name):
    snapshot.samples[name] = snapshot.samples.pop('one.jpg')
    save_samples(snapshot)
    with pytest.raises(ValueError, match='unsafe source filename'):
        run(snapshot, restore_missing=True)
    assert_no_media(snapshot)


def test_one_video_cannot_cross_splits_even_with_different_group_ids(snapshot):
    snapshot.samples['clip_f000002.jpg'] = {**snapshot.samples['clip_f000001.jpg'],
                                           'frame': 2, 'split': 'val', 'group': 'another-group'}
    snapshot.manifest.update(images=5, val=2)
    save_samples(snapshot)
    save_manifest(snapshot)
    with pytest.raises(ValueError, match='source video leaks'):
        run(snapshot, restore_missing=True)
    assert_no_media(snapshot)


def test_existing_conflicting_pixels_are_never_overwritten(snapshot):
    image = snapshot.dataset / 'images/train/one.jpg'
    image.parent.mkdir(parents=True)
    image.write_bytes(b'keep existing user pixels')
    with pytest.raises(ValueError, match='refusing to overwrite'):
        run(snapshot, restore_missing=True)
    assert image.read_bytes() == b'keep existing user pixels'
    assert len(list((snapshot.dataset / 'images').rglob('*.jpg'))) == 1
    assert not snapshot.video_dir.exists()


def test_symlink_destination_does_not_write_outside_snapshot(snapshot, tmp_path):
    outside = tmp_path / 'outside'
    outside.mkdir()
    (snapshot.dataset / 'images').symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError, match='symlink/escaped'):
        run(snapshot, restore_missing=True)
    assert not list(outside.iterdir())
    assert not snapshot.video_dir.exists()


def test_wrong_yaml_root_cannot_claim_a_different_training_dataset(snapshot, tmp_path):
    yaml = snapshot.dataset / 'data.yaml'
    yaml.write_text(yaml.read_text().replace('path: .', f'path: {tmp_path / "other"}'))
    with pytest.raises(ValueError, match='snapshot being restored'):
        run(snapshot, restore_missing=True)
    assert_no_media(snapshot)


def test_video_cache_must_not_overlap_the_label_snapshot(snapshot):
    snapshot.video_dir = snapshot.dataset / 'raw'
    with pytest.raises(ValueError, match='separate'):
        run(snapshot, restore_missing=True)
    assert_no_media(snapshot)


def test_cli_offline_archive_does_not_access_network(snapshot, tmp_path, monkeypatch):
    monkeypatch.setattr(source, 'download_archive', lambda *_: pytest.fail('offline archive must not download'))
    report = tmp_path / 'report.json'
    assert restore.main(['--zip', str(snapshot.archive), '--no-download', '--restore-missing',
                         '--dataset', str(snapshot.dataset), '--video-dir', str(snapshot.video_dir),
                         '--report', str(report)]) == 0
    result = json.loads(report.read_text())
    assert result['kind'] == restore.REPORT_KIND
    assert result['source'] == source.SOURCE_REPOSITORY and result['canonicalImagesVerified'] == 4


def test_cli_wont_overwrite_existing_model_metadata(snapshot, tmp_path):
    metadata = tmp_path / 'model.json'
    metadata.write_text('{"sha256":"keep frozen metadata"}')
    with pytest.raises(SystemExit) as exc:
        restore.main(['--zip', str(snapshot.archive), '--restore-missing', '--dataset', str(snapshot.dataset),
                      '--video-dir', str(snapshot.video_dir), '--report', str(metadata)])
    assert exc.value.code == 2
    assert json.loads(metadata.read_text()) == {'sha256': 'keep frozen metadata'}
    assert_no_media(snapshot)


@pytest.mark.parametrize('damage', ['missing-images', 'unknown-schema', 'missing-samples'])
def test_training_never_rebuilds_an_existing_snapshot(snapshot, monkeypatch, damage):
    from scripts import train_vision
    monkeypatch.setattr(train_vision, 'DATASET', snapshot.dataset)
    monkeypatch.setattr(source, 'build', lambda **_: pytest.fail('must not regenerate labels or splits'))
    if damage != 'missing-images':
        (snapshot.dataset / 'images/train').mkdir(parents=True)
    if damage == 'unknown-schema':
        snapshot.manifest['schemaVersion'] = 1
        save_manifest(snapshot)
    elif damage == 'missing-samples':
        (snapshot.dataset / 'samples.json').unlink()
    before = restore.metadata_hashes(snapshot.dataset)
    with pytest.raises(ValueError, match='restore_vision_source.py --restore-missing'):
        train_vision.ensure_dataset(no_download=True)
    assert restore.metadata_hashes(snapshot.dataset) == before


def test_training_accepts_the_restored_snapshot_without_rebuilding(snapshot, monkeypatch):
    from scripts import train_vision
    run(snapshot, restore_missing=True)
    monkeypatch.setattr(train_vision, 'DATASET', snapshot.dataset)
    monkeypatch.setattr(source, 'build', lambda **_: pytest.fail('must not rebuild restored source'))
    before = restore.metadata_hashes(snapshot.dataset)
    assert train_vision.ensure_dataset(no_download=True) == snapshot.dataset / 'data.yaml'
    assert restore.metadata_hashes(snapshot.dataset) == before


def test_training_initial_dataset_still_uses_the_pinned_source_builder(tmp_path, monkeypatch):
    from scripts import train_vision
    dataset = tmp_path / 'brand-new-dataset'
    monkeypatch.setattr(train_vision, 'DATASET', dataset)
    calls = []
    monkeypatch.setattr(source, 'build', lambda **kwargs: calls.append(kwargs))
    assert train_vision.ensure_dataset(no_download=False) == dataset / 'data.yaml'
    assert calls == [{'zip_path': None, 'out': dataset, 'no_download': False}]
