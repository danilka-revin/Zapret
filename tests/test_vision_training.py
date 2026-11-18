"""Контракты разметки/обучения. Сеть, GPU и реальные веса не нужны."""
from __future__ import annotations

import hashlib
import io
import json
import sys
import types
import zipfile
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from robot_control.vision.classes import (  # noqa: E402
    CLASS_NAMES, TRAINING_CLASS_NAMES, normalize_class, signal_from_class,
)
from robot_control.vision.detector import (  # noqa: E402
    Detection, EnsembleDetector, YoloDetector,
)
from robot_control.vision.traffic import (  # noqa: E402
    TrafficLightTracker, classify_traffic_light, fuse_signal,
)
from scripts import train_vision, vision_dataset  # noqa: E402


def jpeg(seed: int) -> bytes:
    image = np.random.default_rng(seed).integers(0, 255, (72, 96, 3), dtype=np.uint8)
    ok, encoded = cv2.imencode('.jpg', image)
    assert ok
    return encoded.tobytes()


def make_dataset(tmp_path, monkeypatch):
    archive = tmp_path / 'repo.zip'
    nested = io.BytesIO()
    with zipfile.ZipFile(nested, 'w') as z:
        z.writestr('old/old.jpg', jpeg(20))
    with zipfile.ZipFile(archive, 'w') as z:
        for i in range(12):
            z.writestr(f'repo/{i:02d}.jpg', jpeg(i))
        z.writestr('repo/duplicate.jpg', jpeg(0))
        z.writestr('repo/old.zip', nested.getvalue())
        z.writestr('repo/broken.jpg', b'not a jpeg')
    monkeypatch.setattr(vision_dataset, 'propose', lambda *_: [
        Detection('stop_sign', (.1, .1, .3, .3)),
        Detection('traffic_light', (.5, .1, .2, .6), color='green'),
    ])
    out = tmp_path / 'dataset'
    out.mkdir()
    (out / 'reviewed_annotations.json').write_text(json.dumps({
        '01.jpg': [{'class': 'traffic_light_red', 'box': [.1, .1, .2, .5]}],
        '02.jpg': [],  # Явно проверенный фон не заменяется эвристикой.
    }))
    manifest = vision_dataset.build(zip_path=archive, out=out, video_fps=0,
                                   val_fraction=.2, test_fraction=.2)
    return out, manifest


def test_training_classes_extend_public_ids():
    assert TRAINING_CLASS_NAMES[:5] == CLASS_NAMES
    assert TRAINING_CLASS_NAMES[5:] == ('traffic_light_red', 'traffic_light_yellow', 'traffic_light_green')
    for name in TRAINING_CLASS_NAMES[5:]:
        assert normalize_class(name) == 'traffic_light'
        assert signal_from_class(name) in ('red', 'yellow', 'green')
    assert normalize_class('red_light') == 'traffic_light'
    assert normalize_class('car') is None


def test_dataset_reads_root_nested_deduplicates_and_preserves_review(tmp_path, monkeypatch):
    out, manifest = make_dataset(tmp_path, monkeypatch)
    assert manifest['images'] == 13
    assert manifest['duplicatesRemoved'] == 1
    assert manifest['rejected'] == ['broken.jpg']
    assert manifest['reviewedImages'] == 2
    assert sum(manifest[p] for p in ('train', 'val', 'test')) == 13
    samples = json.loads((out / 'samples.json').read_text())
    assert samples['01.jpg']['classes'] == ['traffic_light_red']
    assert samples['02.jpg']['classes'] == []
    assert json.loads((out / 'annotations.json').read_text())['02.jpg'] == []
    assert (out / 'labels' / samples['02.jpg']['split'] / '02.txt').read_text() == ''
    seen_hashes, seen_groups = {}, {}
    for sample in samples.values():
        for seen, key in ((seen_hashes, 'sha256'), (seen_groups, 'group')):
            assert seen.setdefault(sample[key], sample['split']) == sample['split']
    _, summary = train_vision.inspect_dataset(out / 'data.yaml')
    assert set(summary['splits']) == {'train', 'val', 'test'}
    assert len(summary['fingerprint']) == 64


def test_nested_basename_collision_is_not_silently_overwritten(tmp_path):
    archive = tmp_path / 'bad.zip'
    with zipfile.ZipFile(archive, 'w') as z:
        z.writestr('first/a.jpg', jpeg(1))
        z.writestr('second/a.jpg', jpeg(2))
    with pytest.raises(ValueError, match='одинаковым именем'):
        vision_dataset.read_media(archive)


def test_group_split_keeps_whole_videos_and_transitive_near_duplicates():
    samples = {
        'a.jpg': {'kind': 'photo', 'source': 'a.jpg', '_phash': 0, 'classes': ['cone']},
        'b.jpg': {'kind': 'photo', 'source': 'b.jpg', '_phash': 3, 'classes': ['cone']},
        'c.jpg': {'kind': 'video', 'source': 'v.mp4', '_phash': 15, 'classes': ['cone']},
        'd.jpg': {'kind': 'video', 'source': 'v.mp4', '_phash': 0xFFFFFFFF, 'classes': ['cone']},
        'e.jpg': {'kind': 'photo', 'source': 'e.jpg', '_phash': 0x3333333333333333, 'classes': ['cone']},
        'f.jpg': {'kind': 'photo', 'source': 'f.jpg', '_phash': 0x5555555555555555, 'classes': ['cone']},
        'g.jpg': {'kind': 'photo', 'source': 'g.jpg', '_phash': 0x7777777777777777, 'classes': ['cone']},
    }
    vision_dataset.group_samples(samples, distance=2)
    assert len({samples[n]['group'] for n in ('a.jpg', 'b.jpg', 'c.jpg', 'd.jpg')}) == 1
    parts = vision_dataset.split_groups(samples, .2, .2)
    assert parts == vision_dataset.split_groups(dict(reversed(list(samples.items()))), .2, .2)
    assert len({part for part, names in parts.items() if any(n in names for n in ('a.jpg', 'd.jpg'))}) == 1


@pytest.mark.parametrize('line', [
    '8 .5 .5 .2 .2', '0 nan .5 .2 .2', '0 .5 .5 0 .2',
    '0 .99 .5 .3 .2', '0 .5 .5 .2 .2 .7',
])
def test_preflight_rejects_invalid_yolo_labels(tmp_path, monkeypatch, line):
    out, _ = make_dataset(tmp_path, monkeypatch)
    label = next((out / 'labels' / 'train').glob('*.txt'))
    label.write_text(line)
    with pytest.raises(ValueError):
        train_vision.inspect_dataset(out / 'data.yaml')


def test_preflight_rejects_split_leakage(tmp_path, monkeypatch):
    out, _ = make_dataset(tmp_path, monkeypatch)
    first = next((out / 'images' / 'train').glob('*.jpg'))
    (out / 'images' / 'val' / 'leaked.jpg').write_bytes(first.read_bytes())
    (out / 'labels' / 'val' / 'leaked.txt').write_text('')
    with pytest.raises(ValueError, match='утечка'):
        train_vision.inspect_dataset(out / 'data.yaml')


def test_from_scratch_never_uses_existing_production_weights(monkeypatch):
    calls = []
    monkeypatch.setitem(sys.modules, 'ultralytics', types.SimpleNamespace(YOLO=lambda p: calls.append(p) or p))
    model, _ = train_vision.load_model(None, prefer_pretrained=False)
    assert model == 'yolov8n.yaml' and calls == ['yolov8n.yaml']
    with pytest.raises(ValueError):
        train_vision.load_model('best.pt', prefer_pretrained=False)


@pytest.mark.parametrize('kwargs', [
    {'smoke': True, 'output': train_vision.MODEL_FILE},
    {'smoke': True, 'deploy': True},
    {'smoke': False, 'output': train_vision.MODEL_FILE},
])
def test_training_refuses_accidental_production_overwrite(kwargs):
    with pytest.raises(ValueError):
        train_vision.train(epochs=2, imgsz=320, batch=1, weights=None, prefer_pretrained=False,
                           no_download=True, export_onnx=False, **kwargs)


@pytest.mark.parametrize('model,pixels,expected', [
    ('green', 'green', 'green'), ('green', 'unknown', 'unknown'),
    ('green', 'red', 'red'), ('green', 'yellow', 'yellow'),
    ('red', 'green', 'red'), ('yellow', 'green', 'yellow'),
    (None, 'green', 'unknown'), (None, 'unknown', 'unknown'),
])
def test_colour_fusion_fails_closed(model, pixels, expected):
    assert fuse_signal(model, pixels, colour_model=True) == expected
    assert fuse_signal(model, pixels, colour_model=False) == pixels


def test_ambiguous_lights_are_not_green():
    image = np.zeros((30, 30, 3), dtype=np.uint8)
    image[:15] = (20, 20, 250)
    image[15:] = (20, 250, 20)
    assert classify_traffic_light(image)['state'] == 'unknown'
    assert classify_traffic_light(image, prefer_red=True)['state'] == 'red'


def test_unknown_frames_do_not_refresh_old_green_forever():
    tracker = TrafficLightTracker(window=100, stale_s=1.0)
    assert tracker.update('green', .9, stamp=0)['state'] == 'green'
    for stamp in (.2, .4, .6, .8, 1.01, 1.2):
        state = tracker.update('unknown', 0, stamp=stamp)
    assert state['state'] == 'unknown' and state['action'] == 'wait'
    assert not state['fresh']


def test_ensemble_heuristic_cannot_bypass_green_agreement():
    yolo = types.SimpleNamespace(has_signal_classes=True, detect=lambda _: [])
    heuristic = types.SimpleNamespace(detect=lambda _: [Detection(
        'traffic_light', (.2, .2, .2, .6), color='green')])
    output = EnsembleDetector(yolo, heuristic).detect(None)
    assert output[0].color == 'unknown'


def test_yolo_maps_coloured_head_and_passes_inference_size(monkeypatch):
    class Scalar:
        def __init__(self, value): self.value = value
        def item(self): return self.value
    class Coords:
        def tolist(self): return [5, 5, 25, 35]
    box = types.SimpleNamespace(cls=Scalar(7), conf=Scalar(.9), xyxy=[Coords()])
    calls = []
    class Model:
        names = dict(enumerate(TRAINING_CLASS_NAMES))
        def __init__(self, _): pass
        def predict(self, image, **kwargs):
            calls.append(kwargs)
            return [types.SimpleNamespace(boxes=[box])]
    monkeypatch.setitem(sys.modules, 'ultralytics', types.SimpleNamespace(YOLO=Model))
    detector = YoloDetector('fake.pt', imgsz=480)
    image = np.full((40, 40, 3), (20, 240, 20), dtype=np.uint8)
    result = detector.detect(image)
    assert detector.has_signal_classes
    assert result[0].cls == 'traffic_light' and result[0].color == 'green'
    assert calls[0]['imgsz'] == 480 and calls[0]['device'] == 'cpu'
    image[:] = 0
    assert detector.detect(image)[0].color == 'unknown'
    assert detector.detect(np.zeros((0, 0, 3), dtype=np.uint8)) == []


def test_checkpoint_loader_verifies_cached_tensor_archive(tmp_path, monkeypatch):
    from scripts import fetch_vision_weights as fetcher
    path = tmp_path / 'base.pt'
    with zipfile.ZipFile(path, 'w') as archive:
        archive.writestr('checkpoint/data.pkl', b'not executed')
        archive.writestr('checkpoint/data/0', b'tensor bytes')
    expected = fetcher.content_digest(path)
    monkeypatch.setattr(fetcher, 'CONTENT_SHA256', expected)
    monkeypatch.setattr(fetcher.urllib.request, 'urlopen', lambda *_a, **_k: pytest.fail('cache must not use network'))
    assert fetcher.fetch(path) == path
    assert json.loads(path.with_suffix('.json').read_text())['contentSha256'] == expected
    with zipfile.ZipFile(path, 'w') as archive:
        archive.writestr('checkpoint/data.pkl', b'changed')
    with pytest.raises(ValueError, match='неожиданное содержимое'):
        fetcher.fetch(path)


def test_fingerprint_is_portable_and_resume_yaml_uses_absolute_path(tmp_path, monkeypatch):
    import shutil
    out, _ = make_dataset(tmp_path, monkeypatch)
    _, first = train_vision.inspect_dataset(out / 'data.yaml')
    copied = tmp_path / 'copy'
    shutil.copytree(out, copied)
    source = copied / 'data.yaml'
    source.write_text(source.read_text().replace(str(out), str(copied)))
    _, second = train_vision.inspect_dataset(source)
    assert first['fingerprint'] == second['fingerprint']
    runtime = train_vision.runtime_data_yaml(source, tmp_path / 'run' / 'data.yaml')
    _, resumed = train_vision.inspect_dataset(runtime)
    assert resumed['fingerprint'] == second['fingerprint']


def test_train_publishes_best_not_last_and_records_real_epochs(tmp_path, monkeypatch):
    import yaml
    out, _ = make_dataset(tmp_path, monkeypatch)
    calls = []
    class Model:
        def __init__(self, path):
            self.path = Path(path)
            self.names = dict(enumerate(TRAINING_CLASS_NAMES))
        def train(self, **kwargs):
            calls.append(('train', kwargs))
            save_dir = Path(kwargs['project']) / kwargs['name']
            (save_dir / 'weights').mkdir(parents=True)
            (save_dir / 'weights' / 'best.pt').write_bytes(b'BEST MODEL')
            (save_dir / 'weights' / 'last.pt').write_bytes(b'LAST MODEL')
            (save_dir / 'args.yaml').write_text(yaml.safe_dump(kwargs))
            (save_dir / 'results.csv').write_text('epoch,metrics/mAP50-95(B)\n1,0.4\n2,0.8\n')
            return types.SimpleNamespace(save_dir=save_dir)
        def val(self, **kwargs):
            calls.append(('val', self.path.name, kwargs['split']))
            return types.SimpleNamespace(results_dict={'metrics/mAP50(B)': .9}, names=self.names,
                                         box=types.SimpleNamespace(ap_class_index=[]))
        def export(self, **kwargs):
            calls.append(('export', self.path.name))
            raise RuntimeError('optional exporter missing')
    monkeypatch.setitem(sys.modules, 'ultralytics', types.SimpleNamespace(YOLO=Model, __version__='test'))
    monkeypatch.setattr(train_vision, 'ROOT', tmp_path)
    monkeypatch.setattr(train_vision, 'RUNS_DIR', tmp_path / 'runs')
    production = tmp_path / 'production.pt'
    production.write_bytes(b'UNCHANGED PRODUCTION')
    monkeypatch.setattr(train_vision, 'MODEL_FILE', production)
    output = tmp_path / 'candidate.pt'
    meta = train_vision.train(epochs=99, imgsz=320, batch=2, weights='base.pt',
                              prefer_pretrained=True, no_download=True, smoke=False,
                              export_onnx=True, output=output, data=out / 'data.yaml', device='cpu')
    assert output.read_bytes() == b'BEST MODEL'
    assert production.read_bytes() == b'UNCHANGED PRODUCTION'
    assert meta['epochsRequested'] == 99 and meta['epochsCompleted'] == 2 and meta['bestEpoch'] == 2
    assert meta['sha256'] == hashlib.sha256(b'BEST MODEL').hexdigest()
    assert meta['validation'] and meta['test']
    assert 'onnxError' in meta and 'onnx' not in meta
    assert ('val', 'best.pt', 'test') in calls
    assert ('export', 'best.pt') in calls
    args = calls[0][1]
    assert args['hsv_h'] == args['fliplr'] == args['flipud'] == 0
    assert args['device'] == 'cpu' and args['seed'] == 17
    assert json.loads(output.with_suffix('.json').read_text())['epochsCompleted'] == 2


def test_prefer_red_arbitrates_between_cameras():
    import time
    from robot_control.vision.service import VisionConfig, VisionService
    service = VisionService(config=VisionConfig(prefer_red=True), camera_provider=lambda: None,
                            layout_provider=lambda: [])
    now = time.time()
    for cid, signal, confidence in [(0, 'green', .99), (1, 'red', .6)]:
        camera = service._cameras[cid]
        camera.stamp = now
        camera.detections = [Detection('traffic_light', (.2, .2, .2, .5), confidence, signal,
                                       {signal: 100})]
    service._update_traffic(now)
    assert service._traffic.snapshot(now=now)['state'] == 'red'


@pytest.mark.parametrize('other,expected', [('red', 'red'), ('yellow', 'yellow'), ('unknown', 'unknown')])
def test_colour_head_nms_does_not_hide_conflicts_behind_confident_green(other, expected):
    from robot_control.vision.detector import non_max_suppression
    green = Detection('traffic_light', (.2, .2, .2, .5), .95, 'green')
    conflict = Detection('traffic_light', (.21, .2, .2, .5), .65, other)
    assert non_max_suppression([green, conflict])[0].color == 'green'  # legacy unchanged
    result = non_max_suppression([green, conflict], conservative_signals=True)
    assert len(result) == 1 and result[0].color == expected


def test_reused_camera_evidence_cannot_refresh_or_multiply_green():
    tracker = TrafficLightTracker(stale_s=1.0)
    first = tracker.update('green', .9, 2, stamp=10, evidence_stamp=10)
    assert first['state'] == 'green'
    reused = tracker.update('green', .9, 2, stamp=10.8, evidence_stamp=10)
    assert reused['votes'] == ['green'] and reused['ageS'] == .8
    assert tracker.update('green', .9, 2, stamp=11.1, evidence_stamp=10)['state'] == 'unknown'
    assert tracker.update('green', .9, 2, stamp=12, evidence_stamp=13)['state'] == 'unknown'


def test_service_counts_each_camera_frame_once():
    from robot_control.vision.service import VisionConfig, VisionService
    service = VisionService(config=VisionConfig(), camera_provider=lambda: None,
                            layout_provider=lambda: [])
    service._cameras[0].stamp = 100
    service._cameras[0].detections = [Detection('traffic_light', (.2, .2, .2, .5), .9,
                                                'green', {'green': 100})]
    for now in (100, 100.2, 100.4, 100.6):
        service._update_traffic(now)
    snapshot = service._traffic.snapshot(now=100.6)
    assert snapshot['votes'] == ['green'] and snapshot['ageS'] == .6


def test_operating_point_evaluation_matches_once_and_counts_false_green():
    from scripts.evaluate_vision import score_image
    box = (.2, .2, .2, .5)
    expected = [Detection('traffic_light', box, 1, 'red'),
                Detection('cone', (.7, .2, .1, .5))]
    predicted = [Detection('traffic_light', box, .9, 'green'),
                 Detection('traffic_light', box, .6, 'green'),
                 Detection('stop_sign', (.05, .05, .1, .1))]
    result = score_image(expected, predicted)
    assert result['counts']['traffic_light'] == {'tp': 1, 'fp': 1, 'fn': 0}
    assert result['counts']['cone']['fn'] == 1
    assert result['counts']['stop_sign']['fp'] == 1
    assert result['signalConfusion'] == {'red->green': 1, 'background->green': 1}
    assert result['falseGreen'] == 2


def test_rebuild_invalidates_label_cache_and_keeps_duplicate_alias_review(tmp_path, monkeypatch):
    out, _ = make_dataset(tmp_path, monkeypatch)
    reviewed = json.loads((out / 'reviewed_annotations.json').read_text())
    reviewed['duplicate.jpg'] = []  # Same bytes as 00.jpg, which remains canonical.
    (out / 'reviewed_annotations.json').write_text(json.dumps(reviewed))
    (out / 'labels' / 'train.cache').write_bytes(b'stale cached labels')
    vision_dataset.build(zip_path=tmp_path / 'repo.zip', out=out, video_fps=0,
                         val_fraction=.2, test_fraction=.2)
    assert not (out / 'labels' / 'train.cache').exists()
    samples = json.loads((out / 'samples.json').read_text())
    assert samples['00.jpg']['annotationSource'] == 'reviewed'
    assert json.loads((out / 'annotations.json').read_text())['00.jpg'] == []
    assert not samples.get('duplicate.jpg')


def test_red_priority_interrupts_accumulated_green_without_waiting():
    tracker = TrafficLightTracker(prefer_red=True, stale_s=1)
    for n in range(6):
        assert tracker.update('green', .99, stamp=10 + n * .05)['state'] == 'green'
    assert tracker.update('red', .6, stamp=10.3)['state'] == 'red'
    # Не бесконечный приоритет: просроченный red больше не разрешает vote.
    assert tracker.update('unknown', 0, stamp=11.31)['state'] == 'unknown'


def test_external_dataset_yaml_relative_dot_resolves_at_yaml_parent(tmp_path, monkeypatch):
    out, _ = make_dataset(tmp_path, monkeypatch)
    source = out / 'data.yaml'
    _, first = train_vision.inspect_dataset(source)
    source.write_text(source.read_text().replace(str(out), '.'))
    config, second = train_vision.inspect_dataset(source)
    assert config['path'] == str(out)
    assert first['fingerprint'] == second['fingerprint']


def test_slow_inference_cannot_make_old_camera_green_fresh():
    from robot_control.vision.service import VisionConfig, VisionService
    service = VisionService(config=VisionConfig(traffic_stale_s=1), camera_provider=lambda: None,
                            layout_provider=lambda: [])
    camera = service._cameras[0]
    camera.stamp = 100  # inference just completed
    camera.evidence_stamp = 95  # image was captured five seconds ago
    camera.detections = [Detection('traffic_light', (.2, .2, .2, .5), .9,
                                    'green', {'green': 100})]
    service._update_traffic(100)
    assert service._traffic.snapshot(now=100)['state'] == 'unknown'
    assert camera.to_dict(100)['captureAgeS'] == 5


def test_threshold_calibration_uses_only_val_and_rejects_false_green(tmp_path, monkeypatch):
    from scripts import evaluate_vision
    calls = []
    def evaluate(**kwargs):
        calls.append(kwargs)
        confidence = kwargs['confidence']
        tp = 9 if confidence <= .25 else 5
        return {'counts': {name: {'tp': tp, 'fp': 1, 'fn': 10 - tp} for name in CLASS_NAMES},
                'signalConfusion': {'red->red': tp, 'green->green': tp, 'yellow->yellow': tp,
                                    'green->missed': 10 - tp},
                'falseGreen': int(confidence == .15), 'sha256': 'hash', 'datasetFingerprint': 'fingerprint'}
    monkeypatch.setattr(evaluate_vision, 'evaluate', evaluate)
    output = tmp_path / 'calibration.json'
    report = evaluate_vision.calibrate(weights=tmp_path / 'fake.pt', data=tmp_path / 'data.yaml', output=output)
    assert all(call['split'] == 'val' for call in calls)
    assert report['selected']['confidence'] == .25
    assert report['selected']['falseGreen'] == 0
    assert json.loads(output.read_text()) == report
