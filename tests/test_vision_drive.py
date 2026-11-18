"""Drive: real dataset contract, without Drive credentials, network, or torch."""
import json

import pytest

np = pytest.importorskip('numpy')
cv2 = pytest.importorskip('cv2')
yaml = pytest.importorskip('yaml')

from robot_control.vision.classes import DRIVE_TRAINING_CLASS_NAMES, TRAINING_CLASS_NAMES, normalize_class
from scripts import prepare_drive_vision_dataset as prep
from scripts.report_drive_vision import counts
from scripts.train_vision import inspect_dataset
from scripts.vision_dataset import perceptual_hash


def test_floor_is_not_a_sign_or_a_new_robot_action():
    assert DRIVE_TRAINING_CLASS_NAMES == (*TRAINING_CLASS_NAMES, 'ground_bump')
    assert normalize_class('ground_bump') is None
    assert normalize_class('speed_bump') == 'speed_bump'


@pytest.fixture
def tiny(tmp_path, monkeypatch):
    parent, review, media, out = (tmp_path / s for s in ('parent', 'review', 'media', 'out'))
    for path in (parent, review, media):
        path.mkdir()
    monkeypatch.setattr(prep, 'REVIEW', review)
    rng = np.random.default_rng(73)
    samples = {}
    for split in ('train', 'val'):
        img = rng.integers(0, 255, (96, 160, 3), dtype=np.uint8)
        name = f'{split}.jpg'
        image = parent/'images'/split/name
        label = parent/'labels'/split/f'{split}.txt'
        for path in (image, label):
            path.parent.mkdir(parents=True)
        image.write_bytes(prep.jpeg(img))
        label.write_text('2 0.5 0.5 0.2 0.2\n')
        samples[name] = dict(split=split, kind='photo', source=name, group=split,
                             sha256=prep.digest(image.read_bytes()), phash=f'{perceptual_hash(cv2.imread(str(image))):016x}')
        if split == 'train':
            (media/name).write_bytes(image.read_bytes())
    (parent/'data.yaml').write_text(yaml.safe_dump(dict(train='images/train', val='images/val', names=list(TRAINING_CLASS_NAMES))))
    (parent/'samples.json').write_text(json.dumps(samples))
    video = media/'drive.mp4'
    sink = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'mp4v'), 5, (160, 96))
    if not sink.isOpened():
        pytest.skip('no MP4 encoder in local OpenCV')
    for _ in range(3):
        sink.write(rng.integers(0, 255, (96, 160, 3), dtype=np.uint8))
    sink.release()
    spec = {'parentFingerprint': inspect_dataset(parent/'data.yaml')[1]['fingerprint'],
            'videos': {'drive.mp4': {'sha256': prep.digest(video.read_bytes()), 'split': 'test', 'group': 'whole-clip'}}}
    (review/'source.json').write_text(json.dumps(spec))
    (review/'ground_bumps.csv').write_text('image,x1,y1,x2,y2\ntrain.jpg,160,100,240,127\n')
    (review/'video_boxes.csv').write_text('video,frame,class,x1,y1,x2,y2\ndrive.mp4,1,ground_bump,15,70,100,90\n')
    return parent, media, out


def test_build_uses_real_frames_extends_only_own_labels_and_keeps_groups(tiny):
    parent, media, out = tiny
    data = prep.build(parent, media, out)
    assert data['parentMediaRead'] == {'drivePhoto': 1, 'parentSnapshot': 1}
    assert data['splits']['train']['images'] == 2
    assert data['splits']['test']['counts'] == {'ground_bump': 1}
    assert set((out/'labels/train/context-train.txt').read_text().split()[::5]) == {'2', '8'}
    assert (out/'labels/train/train.txt').read_text().splitlines()[-1] == '8 0.500000 0.504444 0.200000 0.120000'
    assert (parent/'labels/train/train.txt').read_text() == '2 0.5 0.5 0.2 0.2\n'
    records = json.loads((out/'samples.json').read_text())
    assert records['context-train.jpg']['group'] == records['train.jpg']['group']
    assert records['drive_f000001.jpg']['group'] == 'whole-clip'
    assert inspect_dataset(out/'data.yaml')[1]['fingerprint'] == data['fingerprint']
    with pytest.raises(ValueError, match='уже есть'):
        prep.build(parent, media, out)


@pytest.mark.parametrize('bad_file', ['drive.mp4', 'train.jpg'])
def test_changed_drive_bytes_do_not_create_partial_dataset(tiny, bad_file):
    parent, media, out = tiny
    (media/bad_file).write_bytes(b'wrong media with the right name')
    with pytest.raises(ValueError, match='SHA-256'):
        prep.build(parent, media, out)
    assert not out.exists()


@pytest.mark.parametrize('x,y,right,bottom', [(3, 1, 3, 10), (-1, 0, 20, 5), (1, 0, 401, 15), (0, 0, 21, 226)])
def test_invalid_reviewed_box_rejected(x, y, right, bottom):
    with pytest.raises(ValueError):
        prep.corners(dict(x1=x, y1=y, x2=right, y2=bottom))


def test_committed_annotations_and_session_holdout():
    spec = json.loads((prep.REVIEW/'source.json').read_text())
    reviews = prep.rows('video_boxes.csv')
    groups = {}
    for row in reviews:
        info = spec['videos'][row['video']]
        assert groups.setdefault(info['group'], info['split']) == info['split']
        assert int(row['frame']) >= 0 and row['class'] in DRIVE_TRAINING_CLASS_NAMES
        prep.corners(row)
    held = {r['video'] for r in reviews if spec['videos'][r['video']]['split'] == 'test'}
    assert len(held) == 2 and len({spec['videos'][n]['group'] for n in held}) == 1
    assert spec['excludedVideo']['name'] not in {r['video'] for r in reviews}


def test_confusion_report_separates_finding_a_light_from_its_colour():
    # The off-diagonal is a located housing with the WRONG colour; the last
    # row/column is a missed target / background prediction, not a light.
    m = np.array([[1, 2, 1], [0, 3, 0], [1, 0, 0]])
    assert counts(m, [0]) == {'correct': 1, 'labelled': 2, 'predicted': 4}
    assert counts(m, [0, 1]) == {'correct': 6, 'labelled': 7, 'predicted': 7}


def test_near_duplicate_of_a_different_split_is_rejected(tiny):
    parent, media, out = tiny
    p = parent/'samples.json'
    entries = json.loads(p.read_text())
    cap = cv2.VideoCapture(str(media/'drive.mp4'))
    try:
        cap.set(cv2.CAP_PROP_POS_FRAMES, 1)
        ok, frame = cap.read()
        assert ok
    finally:
        cap.release()
    entries['train.jpg']['phash'] = f'{perceptual_hash(frame):016x}'
    p.write_text(json.dumps(entries))
    with pytest.raises(ValueError, match='похожий кадр'):
        prep.build(parent, media, out)
    assert not out.exists()


def test_drive_dataset_cannot_replace_the_production_checkpoint(tiny, monkeypatch):
    import sys
    from types import SimpleNamespace
    from scripts.train_vision import train

    parent, media, out = tiny
    prep.build(parent, media, out)
    monkeypatch.setitem(sys.modules, 'torch', SimpleNamespace())
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=object))
    with pytest.raises(ValueError, match='офлайн-кандидат'):
        train(epochs=1, imgsz=416, batch=1, weights=None, prefer_pretrained=True,
              no_download=True, smoke=False, export_onnx=False, data=out/'data.yaml', deploy=True)


def test_horizontal_labels_preserve_classes_sizes_and_empty_backgrounds():
    labels = ['0 0.18561234 0.5 0.25 0.4', '6 0.9 0.7 0.2 0.1']
    mirrored = prep.horizontal_labels(labels)
    assert mirrored == ['0 0.81438766 0.5 0.25 0.4', '6 0.1 0.7 0.2 0.1']
    assert prep.horizontal_labels(mirrored) == labels
    assert prep.horizontal_labels(['', ' ']) == []


def test_mirror_variant_doubles_only_train_and_preserves_original_bytes(tiny):
    parent, media, base = tiny
    out = base.with_name('x2')
    original = prep.build(parent, media, base)
    double = prep.build(parent, media, out, mirror_train=True)
    assert double['mirrorTrain'] == dict(axis='horizontal', originals=2, copies=2, heldOutCopies=0)
    assert double['splits']['train']['counts'] == {k: 2 * n for k, n in original['splits']['train']['counts'].items()}
    old = json.loads((base/'samples.json').read_text())
    new = json.loads((out/'samples.json').read_text())
    train = {n for n, info in old.items() if info['split'] == 'train'}
    assert set(new) == set(old) | {'hflip-' + n for n in train}
    for name, info in old.items():
        split = info['split']
        image = base / 'images' / split / name
        label = base / 'labels' / split / image.with_suffix('.txt').name
        for p in (image, label):
            assert (out / p.relative_to(base)).read_bytes() == p.read_bytes()
        assert new[name] == info
        if name not in train:
            assert double['splits'][split] == original['splits'][split]
            continue
        mirrored = new['hflip-' + name]
        assert (mirrored['parentImage'], mirrored['group'], mirrored['split']) == (name, info['group'], 'train')
        assert (out/'images/train'/('hflip-' + name)).read_bytes() == prep.jpeg(cv2.flip(cv2.imread(str(image)), 1))
        assert (out/'labels/train'/('hflip-' + label.name)).read_text().splitlines() == prep.horizontal_labels(label.read_text().splitlines())


def test_mirror_album_preserves_filenames_and_verifies_input_bytes(tmp_path):
    import hashlib
    from zipfile import ZipFile
    from scripts.mirror_drive_photos import mirror_album

    media = tmp_path/'media'
    media.mkdir()
    orig = prep.jpeg(np.random.default_rng(3).integers(0, 255, (30, 60, 3), dtype=np.uint8))
    (media/'1.jpg').write_bytes(orig)
    source = tmp_path/'source.zip'
    with ZipFile(source, 'x') as z:
        z.writestr('pinned/2.jpg', orig)
    md5 = hashlib.md5(orig).hexdigest()
    manifest, out = tmp_path/'photos.md5', tmp_path/'flips.zip'
    manifest.write_text(f'{md5}  1.jpg\n{md5}  2.jpg\n')
    assert mirror_album(media, out, archive=source, manifest=manifest) == 2
    with ZipFile(out) as z:
        assert z.read('hflip-1.jpg') == z.read('hflip-2.jpg') == prep.jpeg(cv2.flip(cv2.imread(str(media/'1.jpg')), 1))
    assert (media/'1.jpg').read_bytes() == orig
    with pytest.raises(FileExistsError):
        mirror_album(media, out, archive=source, manifest=manifest)
    (media/'1.jpg').write_bytes(b'wrong')
    wrong = tmp_path/'wrong.zip'
    with pytest.raises(ValueError, match='манифестом'):
        mirror_album(media, wrong, archive=source, manifest=manifest)
    assert not wrong.exists()
