"""Robot fine-tuning must not move validation/test families into TRAIN."""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from robot_control.vision.classes import TRAINING_CLASS_NAMES
from scripts import prepare_robot_vision_dataset as module
from scripts import train_vision


@pytest.fixture
def robot_source(tmp_path, monkeypatch):
    cv2 = pytest.importorskip('cv2')
    yaml = pytest.importorskip('yaml')
    monkeypatch.setattr(module, 'ROOT', tmp_path)
    monkeypatch.setattr(train_vision, 'ROOT', tmp_path)
    source = tmp_path/'datasets/refined'
    output = tmp_path/'datasets/robot'
    videos = tmp_path/'videos'; videos.mkdir()
    train = np.full((120,80,3), 40, np.uint8)
    train[14:35,25:40] = (30,30,230)
    val = np.full((120,80,3), 115, np.uint8)
    crop = train[10:100,12:65]
    samples = {}
    labels = '5 .5 .5 .5 .75\n4 .45 .3 .04 .2\n'
    for name, split, image, kind in [('train.jpg','train',train,'video'),
                                       ('crop.jpg','train',crop,'train-traffic-crop'),
                                       ('val.jpg','val',val,'video')]:
        for folder in ('images','labels'):
            (source/folder/split).mkdir(parents=True,exist_ok=True)
        ok, jpeg = cv2.imencode('.jpg',image)
        assert ok
        payload=jpeg.tobytes()
        (source/'images'/split/name).write_bytes(payload)
        (source/'labels'/split/Path(name).with_suffix('.txt')).write_text(labels)
        samples[name]={'source':'train.mp4' if split=='train' else 'val.mp4', 'kind':kind,
                       'group':split+'-family','split':split,'sha256':hashlib.sha256(payload).hexdigest()}
    samples['crop.jpg'].update(parentImage='train.jpg',cropPixels=[12,10,65,100])
    (source/'samples.json').write_text(json.dumps(samples))
    (source/'dataset.json').write_text(json.dumps({'sourceRevision':'fixture-source','archiveSha256':'fixture-archive'}))
    (source/'data.yaml').write_text(yaml.safe_dump({'path':str(source),'train':'images/train','val':'images/val',
                                                    'names':dict(enumerate(TRAINING_CLASS_NAMES))}))
    writer=cv2.VideoWriter(str(videos/'train.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),10.,(80,120))
    if not writer.isOpened():
        pytest.skip('MP4 encoder unavailable')
    for i in range(4):
        writer.write(np.full((120,80,3),170+i,np.uint8))
    writer.release()
    review={'parentFingerprint':train_vision.inspect_dataset(source/'data.yaml')[1]['fingerprint'],
            'housingBoxes':[{'image':'train.jpg','row':0,'original':labels.splitlines()[0],
                             'box':[.15,.1,.3,.65],'approved':True,'reason':'Visually reviewed fixture'}],
            'videoNegatives':[{'source':'train.mp4','frame':2,'box':[0,.5,.5,.5],
                               'group':'train-family','sourceSha256':hashlib.sha256((videos/'train.mp4').read_bytes()).hexdigest(),
                               'approved':True,'reason':'Visually reviewed target-free fixture'}]}
    review_file=tmp_path/'review.json'; review_file.write_text(json.dumps(review))
    return source,output,review_file,videos


def inventory(path):
    return {p.relative_to(path):p.read_bytes() for p in path.rglob('*') if p.is_file()}


def test_separate_repeatable_build_preserves_source_and_val(robot_source):
    source,output,review,videos=robot_source
    before=inventory(source)
    first=module.build(source,output,review,videos)
    assert inventory(source)==before
    assert first['splits']['train']['images']==3
    assert 'test' not in first['splits']
    assert not (output/'images/test').exists()
    assert (output/'labels/val/val.txt').read_bytes()==(source/'labels/val/val.txt').read_bytes()
    assert (output/'images/val/val.jpg').read_bytes()==(source/'images/val/val.jpg').read_bytes()
    lines=(output/'labels/train/train.txt').read_text().splitlines()
    assert lines[0]=='5 0.300000 0.425000 0.300000 0.650000'
    assert lines[1]=='4 .45 .3 .04 .2'  # no other class/instance edits
    crop_lines=(output/'labels/train/crop.txt').read_text().splitlines()
    assert {int(line.split()[0]) for line in crop_lines}=={4,5}
    negatives=list((output/'labels/train').glob('robot-negative-*.txt'))
    assert len(negatives)==1 and negatives[0].read_bytes()==b''
    records=json.loads((output/'samples.json').read_text())
    assert all(r['group']=='train-family' for r in records.values() if r['split']=='train')
    (output/'labels/train.cache').write_bytes(b'stale cache')
    second=module.build(source,output,review,videos)
    assert not (output/'labels/train.cache').exists()
    assert first['fingerprint']==second['fingerprint']
    assert inventory(source)==before


@pytest.mark.parametrize('mutation,match', [
    ('val-housing','original TRAIN'), ('wrong-row','row'), ('changed-row','row changed'),
    ('wrong-class','row changed'), ('unapproved-housing','unapproved housing'),
    ('outside-box','inside'), ('val-video','original TRAIN family'),
    ('wrong-group','original TRAIN family'), ('video-sha','SHA changed'),
    ('unapproved-negative','unapproved negative'), ('negative-frame','nonnegative'),
    ('fractional-frame','nonnegative'), ('duplicate-negative','duplicate negative'),
    ('duplicate-housing','duplicate reviewed'), ('path-video','safe MP4'),
    ('fingerprint','parent fingerprint'),
])
def test_rejects_unreviewed_or_leaking_data_before_output(robot_source,mutation,match):
    source,output,review,videos=robot_source
    data=json.loads(review.read_text());h=data['housingBoxes'][0];n=data['videoNegatives'][0]
    if mutation=='val-housing': h['image']='val.jpg'
    elif mutation=='wrong-row': h['row']=100
    elif mutation=='changed-row': h['original']='5 .51 .5 .5 .75'
    elif mutation=='wrong-class': h['row']=1; h['original']='4 .45 .3 .04 .2'
    elif mutation=='unapproved-housing': h['approved']=False
    elif mutation=='outside-box': h['box']=[.9,0,.5,.5]
    elif mutation=='val-video': n['source']='val.mp4';n['group']='val-family'
    elif mutation=='wrong-group': n['group']='val-family'
    elif mutation=='video-sha': n['sourceSha256']='changed'
    elif mutation=='unapproved-negative': n['approved']=False
    elif mutation=='negative-frame': n['frame']=-1
    elif mutation=='fractional-frame': n['frame']=1.5
    elif mutation=='duplicate-negative': data['videoNegatives'].append(copy.deepcopy(n))
    elif mutation=='duplicate-housing': data['housingBoxes'].append(copy.deepcopy(h))
    elif mutation=='path-video': n['source']='../train.mp4'
    elif mutation=='fingerprint': data['parentFingerprint']='changed'
    review.write_text(json.dumps(data))
    with pytest.raises(ValueError,match=match):
        module.build(source,output,review,videos)
    assert not output.exists()


def test_changed_source_pixels_are_not_silently_relabelled(robot_source):
    source,output,review,videos=robot_source
    path=source/'images/train/train.jpg';path.write_bytes(path.read_bytes()+b'changed')
    # A review is pinned both to the full dataset fingerprint and source SHAs.
    with pytest.raises(ValueError,match='parent fingerprint'):
        module.build(source,output,review,videos)
    data=json.loads(review.read_text())
    data['parentFingerprint']=train_vision.inspect_dataset(source/'data.yaml')[1]['fingerprint']
    review.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='pixels changed'):
        module.build(source,output,review,videos)


def test_source_and_unrelated_output_never_overwritten(robot_source):
    source,output,review,videos=robot_source
    for destination in (source,source.parent,source/'nested'):
        with pytest.raises(ValueError,match='separate sibling'):
            module.build(source,destination,review,videos)
    output.mkdir();(output/'data.yaml').write_text('unrelated')
    with pytest.raises(ValueError,match='unrelated dataset'):
        module.build(source,output,review,videos)
    assert (output/'data.yaml').read_text()=='unrelated'


@pytest.mark.parametrize('box', [[0,0,0,1],[0,0,1,float('nan')],[0,0,1],[0,-.1,.2,.2]])
def test_invalid_boxes_cannot_become_training_labels(box):
    with pytest.raises(ValueError,match='box'):
        module.checked_box(box)


def test_crop_labels_preserves_partial_objects_not_just_traffic():
    result=module.crop_labels(['5 .5 .5 .6 .8','4 .8 .7 .2 .2','0 .1 .1 .1 .1'],(30,20,80,90),100,100)
    assert {int(line.split()[0]) for line in result}=={4,5}
    assert all(0<float(line.split()[3])<=1 for line in result)
