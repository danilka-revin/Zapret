"""Opt-in verified signal pipeline and training-only dataset refinement."""
from pathlib import Path
import sys
import types

import numpy as np
import pytest

from robot_control.vision.classes import CLASS_NAMES, TRAINING_CLASS_NAMES
from robot_control.vision.detector import Detection, EnsembleDetector, YoloDetector
from robot_control.vision.service import VisionConfig, VisionService
from robot_control.vision.traffic import classify_traffic_light, verify_traffic_light
from scripts.refine_vision_dataset import fold_label, plausible_cone_crop


def panel(*signals):
    image = np.full((120, 40, 3), 25, np.uint8)
    starts = {'red': 8, 'yellow': 48, 'green': 88}
    bgr = {'red': (40, 40, 230), 'yellow': (30, 230, 245), 'green': (30, 215, 45)}
    for signal in signals:
        for y in range(starts[signal], starts[signal]+20, 4):
            for x in range(9, 32, 4):
                image[y:y+2, x:x+2] = bgr[signal]
    return image


@pytest.mark.parametrize('state', ['red', 'yellow', 'green'])
def test_verified_emission_requires_matching_section(state):
    result = verify_traffic_light(panel(state))
    assert result['validBody']
    assert result['state'] == state
    assert result['reason'] == 'verified-section'


def test_green_in_top_section_is_not_permission():
    image = panel()
    image[8:28, 10:30] = (20, 230, 40)
    assert verify_traffic_light(image)['state'] == 'unknown'


def test_dark_and_conflicting_lamps_are_unknown():
    assert verify_traffic_light(panel())['state'] == 'unknown'
    assert verify_traffic_light(panel('red', 'green'))['state'] == 'unknown'
    assert verify_traffic_light(panel('red', 'green'), prefer_red=True)['state'] == 'red'


@pytest.mark.parametrize('shape', [(120, 40, 3), (20, 20, 3)])
def test_painted_green_surface_is_not_a_signal(shape):
    image = np.full(shape, (20, 230, 40), np.uint8)
    assert classify_traffic_light(image)['state'] == 'green'  # legacy contract
    assert verify_traffic_light(image)['state'] == 'unknown'
    assert not verify_traffic_light(image)['validBody']


def test_wide_foot_and_door_boxes_rejected():
    for image in [np.zeros((80, 100, 3), np.uint8), np.zeros((0, 0, 3), np.uint8),
                  np.zeros((9, 3, 3), np.uint8)]:
        result = verify_traffic_light(image)
        assert not result['validBody'] and result['state'] == 'unknown'


def fake_yolo(monkeypatch, *, names=CLASS_NAMES, coords=(10, 10, 50, 130), class_id=3):
    class Scalar:
        def __init__(self, value): self.value = value
        def item(self): return self.value
    class Coords:
        def tolist(self): return list(coords)
    box = types.SimpleNamespace(cls=Scalar(class_id), conf=Scalar(.9), xyxy=[Coords()])
    class Model:
        def __init__(self, _): self.names = dict(enumerate(names))
        def predict(self, image, **kwargs): return [types.SimpleNamespace(boxes=[box])]
    monkeypatch.setitem(sys.modules, 'ultralytics', types.SimpleNamespace(YOLO=Model))


def test_verified_five_class_head_cannot_grant_green(monkeypatch):
    fake_yolo(monkeypatch)
    image = np.full((150, 80, 3), 230, np.uint8)
    image[10:130, 10:50] = panel('green')
    assert verify_traffic_light(image[10:130, 10:50])['state'] == 'green'
    result = YoloDetector('fake.pt', traffic_mode='verified').detect(image)
    assert len(result) == 1 and result[0].color == 'unknown'
    image[10:130, 10:50] = panel()
    assert YoloDetector('fake.pt', traffic_mode='verified').detect(image)[0].color == 'unknown'


@pytest.mark.parametrize('class_id,signal,expected', [
    (7, 'green', 'green'), (3, 'green', 'unknown'), (5, 'green', 'red'),
    (7, 'red', 'red'), (6, 'green', 'yellow'),
])
def test_verified_preserves_learned_green_agreement(monkeypatch, class_id, signal, expected):
    fake_yolo(monkeypatch, names=TRAINING_CLASS_NAMES, class_id=class_id)
    image = np.zeros((150, 80, 3), np.uint8)
    image[10:130, 10:50] = panel(signal)
    assert YoloDetector('fake.pt', traffic_mode='verified').detect(image)[0].color == expected


@pytest.mark.parametrize('class_id', [3, 5, 6, 7])
def test_verified_dark_panel_has_no_colour_head_fallback(monkeypatch, class_id):
    fake_yolo(monkeypatch, names=TRAINING_CLASS_NAMES, class_id=class_id)
    image = np.zeros((150, 80, 3), np.uint8)
    image[10:130, 10:50] = panel()
    assert YoloDetector('fake.pt', traffic_mode='verified').detect(image)[0].color == 'unknown'


def test_legacy_eight_class_consensus_is_unchanged(monkeypatch):
    fake_yolo(monkeypatch, names=TRAINING_CLASS_NAMES)
    image = np.full((150, 80, 3), 230, np.uint8)
    image[10:130, 10:50] = panel('green')
    # A generic/off output of the old eight-class head still cannot grant green.
    assert YoloDetector('fake.pt').detect(image)[0].color == 'unknown'


def test_verified_rejects_wide_false_traffic_box(monkeypatch):
    fake_yolo(monkeypatch, coords=(0, 0, 80, 60))
    image = np.zeros((100, 100, 3), np.uint8)
    assert YoloDetector('fake.pt', traffic_mode='verified').detect(image) == []


def test_invalid_detector_mode_fails_before_model_import():
    with pytest.raises(ValueError, match='traffic_mode'):
        YoloDetector('missing.pt', traffic_mode='verifed')


@pytest.mark.parametrize('state', ['red', 'yellow', 'green', 'unknown'])
def test_heuristic_cannot_bypass_verified_traffic(state):
    yolo = types.SimpleNamespace(traffic_mode='verified', has_signal_classes=False, detect=lambda _: [])
    heuristic = types.SimpleNamespace(detect=lambda _: [Detection('traffic_light', (.1,.1,.2,.6), color=state)])
    assert EnsembleDetector(yolo, heuristic).detect(None) == []


def test_verified_service_does_not_fall_back_when_weights_missing():
    service = VisionService(config=VisionConfig(model_path=Path('/missing-verified.pt'), traffic_mode='verified'),
                            camera_provider=lambda: None, layout_provider=lambda: [])
    assert service.initialize() == 'unavailable'
    assert service._detector is None
    assert service.state()['trafficMode'] == 'verified'


def test_service_default_remains_legacy(monkeypatch):
    monkeypatch.delenv('RC_VISION_TRAFFIC_MODE', raising=False)
    assert VisionConfig.from_env().traffic_mode == 'legacy'
    monkeypatch.setenv('RC_VISION_TRAFFIC_MODE', 'verified')
    assert VisionConfig.from_env().traffic_mode == 'verified'


def test_verified_service_has_no_extra_heuristic_detector(tmp_path, monkeypatch):
    import robot_control.vision.service as module
    weights = tmp_path/'candidate.pt'; weights.write_bytes(b'fake')
    detector = types.SimpleNamespace()
    calls = []
    def model(*args, **kwargs):
        calls.append(kwargs); return detector
    monkeypatch.setattr(module, 'YoloDetector', model)
    service = VisionService(config=VisionConfig(model_path=weights, traffic_mode='verified'),
                            camera_provider=lambda: None, layout_provider=lambda: [])
    assert service.initialize() == 'yolo+verified'
    assert service._detector is detector
    assert calls[0]['traffic_mode'] == 'verified'


@pytest.mark.parametrize('cls', [3, 5, 6, 7])
def test_label_folding_preserves_box_coordinates(cls):
    assert fold_label(f'{cls} .5 .4 .2 .6') == '3 .5 .4 .2 .6'


def test_train_cone_filter_does_not_accept_black_trousers():
    assert not plausible_cone_crop(np.full((160, 35, 3), 35, np.uint8))
    image = np.full((160, 35, 3), (20, 95, 235), np.uint8)
    image[20:40] = 230; image[70:90] = 230; image[120:140] = 230
    assert plausible_cone_crop(image)


@pytest.fixture
def refinement_source(tmp_path, monkeypatch):
    import hashlib
    import json
    import yaml
    cv2 = pytest.importorskip('cv2')
    from scripts import refine_vision_dataset as module, train_vision
    monkeypatch.setattr(module, 'ROOT', tmp_path)
    monkeypatch.setattr(train_vision, 'ROOT', tmp_path)
    source = tmp_path/'datasets/canonical'
    records = {}
    for i, (split, state) in enumerate([('train', 'red'), ('train', 'yellow'),
                                         ('val', 'green'), ('test', 'green')]):
        for folder in ('images', 'labels'):
            (source/folder/split).mkdir(parents=True, exist_ok=True)
        image = panel(state)
        image[0:4, 0:4] = 40+i  # distinct SHA in each split, no fake leakage
        ok, payload = cv2.imencode('.jpg', image)
        assert ok
        name = f'{split}-{i}.jpg'
        (source/'images'/split/name).write_bytes(payload.tobytes())
        cls = {'red': 5, 'yellow': 6, 'green': 7}[state]
        (source/'labels'/split/Path(name).with_suffix('.txt')).write_text(f'{cls} .5 .5 .8 .8\n')
        records[name] = {'split':split,'sha256':hashlib.sha256(payload).hexdigest(),
                          'source':name,'group':str(i),'kind':'photo','classes':[]}
    (source/'samples.json').write_text(json.dumps(records))
    (source/'dataset.json').write_text(json.dumps({'sourceRevision':'fixture-revision'}))
    config = {'path':str(source), 'train':'images/train','val':'images/val','test':'images/test',
              'names':dict(enumerate(TRAINING_CLASS_NAMES))}
    (source/'data.yaml').write_text(yaml.safe_dump(config))
    review = tmp_path/'review.json'
    review.write_text(json.dumps({'excludeTrainImages':[], 'negativeCrops':[],
                                   'trainSignalClasses':{'train-0.jpg':3}}))
    return source, tmp_path/'datasets/refined', review


@pytest.mark.parametrize('keep_color_classes', [False, True])
def test_refinement_is_separate_repeatable_and_preserves_val_and_test(refinement_source, keep_color_classes):
    from scripts import refine_vision_dataset as module
    import json
    source, output, review = refinement_source
    before = {p.relative_to(source):p.read_bytes() for p in source.rglob('*') if p.is_file()}
    manifest = module.build(source, output, review, keep_color_classes=keep_color_classes)
    assert 'test' not in manifest['splits']
    assert not (output/'images/test').exists()
    assert (output/'labels/train/train-0.txt').read_text() == '3 .5 .5 .8 .8\n'
    expected = '7' if keep_color_classes else '3'
    assert (output/'labels/val/val-2.txt').read_text() == f'{expected} .5 .5 .8 .8\n'
    assert {p.relative_to(source):p.read_bytes() for p in source.rglob('*') if p.is_file()} == before
    assert module.build(source, output, review, keep_color_classes=keep_color_classes)['fingerprint'] == manifest['fingerprint']
    records = json.loads((output/'samples.json').read_text())
    assert all(s['split']=='train' for s in records.values() if s.get('kind')=='train-traffic-crop')
    assert manifest['sourceRevision']=='fixture-revision'


@pytest.mark.parametrize('field,value,match', [
    ('excludeTrainImages', ['val-2.jpg'], 'TRAIN'),
    ('negativeCrops', [{'image':'test-3.jpg','box':[0,0,.5,.5]}], 'TRAIN'),
    ('trainSignalClasses', {'val-2.jpg':3}, 'TRAIN'),
    ('trainSignalClasses', {'train-0.jpg':4}, 'traffic class ID'),
])
def test_refinement_rejects_non_train_or_invalid_review(refinement_source, field, value, match):
    import json
    from scripts.refine_vision_dataset import build
    source, output, review = refinement_source
    data = json.loads(review.read_text()); data[field] = value; review.write_text(json.dumps(data))
    with pytest.raises(ValueError, match=match):
        build(source, output, review)


def test_refinement_rejects_changed_source_sha(refinement_source):
    from scripts.refine_vision_dataset import build
    source, output, review = refinement_source
    (source/'images/train/train-0.jpg').write_bytes(b'changed image')
    with pytest.raises(ValueError, match='pixels changed'):
        build(source, output, review)


def test_refinement_does_not_overwrite_source_or_ancestor(refinement_source):
    from scripts.refine_vision_dataset import build
    source, output, review = refinement_source
    for destination in [source, source.parent, source/'nested']:
        with pytest.raises(ValueError, match='separate sibling'):
            build(source, destination, review)


def test_cone_filter_honours_visual_retention_override(refinement_source):
    import json
    from scripts.refine_vision_dataset import build
    source, output, review = refinement_source
    label = source/'labels/train/train-0.txt'
    label.write_text('4 .5 .5 .8 .8\n')  # synthetic dark crop fails the colour heuristic
    data = json.loads(review.read_text())
    data['keepTrainConeLabels'] = [{'image':'train-0.jpg','row':0}]
    review.write_text(json.dumps(data))
    manifest = build(source, output, review)
    assert (output/'labels/train/train-0.txt').read_text() == label.read_text()
    assert manifest['retainedReviewedConeLabels'] == 1
    assert manifest['removedTrainConeLabels'] == 0


def test_unrelated_output_dataset_is_never_overwritten(refinement_source):
    from scripts.refine_vision_dataset import build
    source, output, review = refinement_source
    output.mkdir(); (output/'data.yaml').write_text('do not overwrite')
    with pytest.raises(ValueError, match='unrelated'):
        build(source, output, review)
    assert (output/'data.yaml').read_text() == 'do not overwrite'


def test_compare_keeps_precise_rates_and_counts_signal_misses():
    from copy import deepcopy
    from scripts.compare_vision import compare, summarize
    report = {'weights':'old.pt','sha256':'old','datasetFingerprint':'same','split':'val',
               'images':1,'confidence':.25,'imgsz':512,'device':'cpu','preferRed':False,
               'iouMatch':.5,'backend':'yolo+fusion','falseGreen':0,
               'counts':{name:dict(tp=2,fp=1,fn=1) for name in CLASS_NAMES},
               'signalConfusion':{'red->red':1,'red->missed':2,'green->unknown':1},
               'frames':[{'image':'same.jpg','groundTruthClasses':['traffic_light_red']}]}
    summary = summarize(report)
    assert summary['micro'] == dict(tp=10, fp=5, fn=5, precision=2/3, recall=2/3, f1=2/3)
    assert summary['signals']['red']['recallIncludingMisses'] == 1/3
    after = deepcopy(report); after['weights']='new.pt'; after['trafficMode']='verified'
    result = compare(report, after)
    assert result['delta']['macroF1'] == 0
    assert result['before']['trafficMode'] == 'legacy'
    assert result['after']['trafficMode'] == 'verified'
    for key, value in [('split','test'),('confidence',.35),('datasetFingerprint','other'),('imgsz',640)]:
        changed = deepcopy(after); changed[key] = value
        with pytest.raises(ValueError, match='must match'):
            compare(report, changed)
    changed = deepcopy(after); changed['frames'][0]['groundTruthClasses'] = []
    with pytest.raises(ValueError, match='ground truth differ'):
        compare(report, changed)


def test_yolo_only_profile_disables_extras_and_fallback(tmp_path, monkeypatch):
    import robot_control.vision.service as module
    weights = tmp_path/'candidate.pt'; weights.write_bytes(b'fake')
    detector = types.SimpleNamespace()
    monkeypatch.setattr(module, 'YoloDetector', lambda *a, **kw: detector)
    service = VisionService(config=VisionConfig(model_path=weights, heuristic_enabled=False),
                            camera_provider=lambda: None, layout_provider=lambda: [])
    assert service.initialize() == 'yolo'
    assert service._detector is detector
    assert service.state()['heuristicEnabled'] is False
    weights.unlink()
    assert service.initialize() == 'unavailable'
    assert service._detector is None
    monkeypatch.delenv('RC_VISION_HEURISTIC', raising=False)
    assert VisionConfig.from_env().heuristic_enabled is True
    monkeypatch.setenv('RC_VISION_HEURISTIC','0')
    assert VisionConfig.from_env().heuristic_enabled is False


def test_refinement_invalidates_equal_size_label_cache(refinement_source):
    from scripts.refine_vision_dataset import build
    source, output, review = refinement_source
    build(source, output, review)
    cache = output/'labels/train.cache'; cache.write_bytes(b'old cached class IDs')
    build(source, output, review)
    assert not cache.exists()


@pytest.mark.parametrize('class_id,signal', [(7,'green'),(3,'green'),(5,'green'),(6,'red')])
def test_body_guard_preserves_legacy_fusion_exactly(monkeypatch, class_id, signal):
    fake_yolo(monkeypatch, names=TRAINING_CLASS_NAMES, class_id=class_id)
    image = np.zeros((150,80,3),np.uint8); image[10:130,10:50]=panel(signal)
    before = YoloDetector('fake.pt').detect(image)
    after = YoloDetector('fake.pt', traffic_mode='body').detect(image)
    assert before == after  # no changed colour or extra permissive green


def test_body_guard_only_removes_implausible_boxes(monkeypatch):
    fake_yolo(monkeypatch, names=TRAINING_CLASS_NAMES, coords=(0,0,80,60), class_id=5)
    image = np.zeros((100,100,3),np.uint8)
    assert YoloDetector('fake.pt').detect(image)
    assert YoloDetector('fake.pt', traffic_mode='body').detect(image) == []


def test_body_guard_cannot_create_green_after_nms(monkeypatch):
    import robot_control.vision.detector as module
    fake_yolo(monkeypatch, names=TRAINING_CLASS_NAMES, class_id=7)
    # Simulate a red having suppressed a green. Rejecting the final red
    # must not rerun NMS and resurrect the suppressed green.
    monkeypatch.setattr(module,'non_max_suppression',lambda *a, **kw:[Detection('traffic_light',(0,0,.8,.6),.9,'red')])
    image=np.zeros((100,100,3),np.uint8)
    assert YoloDetector('fake.pt',traffic_mode='body').detect(image)==[]


def test_heuristic_cannot_bypass_body_guard():
    yolo=types.SimpleNamespace(traffic_mode='body',has_signal_classes=True,detect=lambda _:[])
    extra=types.SimpleNamespace(detect=lambda _:[Detection('traffic_light',(.1,.1,.2,.6),color='red')])
    assert EnsembleDetector(yolo,extra).detect(None)==[]
