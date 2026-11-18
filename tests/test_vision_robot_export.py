"""Portable exports must retain the actual adapter's signals, not just boxes."""
import pytest

from robot_control.vision.detector import Detection
from scripts.export_robot_vision import compare_detections


def det(*, cls='traffic_light', signal='green', box=(.1,.1,.2,.6), confidence=.8):
    return Detection(cls,box,confidence,color=signal,source='yolo')


def test_parity_matches_independently_of_order_and_preserves_unknown():
    items=[det(),det(cls='cone',signal=None,box=(.6,.3,.1,.5)),det(signal='unknown',box=(.5,.1,.1,.2))]
    result=compare_detections(items,list(reversed(items)))
    assert result['passed'] and result['matched']==3
    assert result['minimumIoU']==pytest.approx(1.)


@pytest.mark.parametrize('changed', [
    [], [det(signal='red')], [det(signal='unknown')],
    [det(cls='cone',signal=None)], [det(box=(.3,.3,.2,.6))],
    [det(confidence=.79)], [det(),det(box=(.7,.1,.1,.4))],
])
def test_parity_fails_for_missing_extra_or_changed_detections(changed):
    assert not compare_detections([det()],changed)['passed']


def test_tiny_fp32_roundoff_is_not_a_new_detection():
    result=compare_detections([det()],[det(box=(.100001,.1,.2,.6),confidence=.800001)])
    assert result['passed']
    assert result['maximumConfidenceError']<1e-4


def test_empty_output_parity_does_not_claim_box_iou_or_accuracy():
    result=compare_detections([],[])
    assert result['passed'] and result['matched']==0
    assert result['minimumIoU'] is None
    assert not compare_detections([],[det()])['passed']
