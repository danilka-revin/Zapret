"""Selection cannot quietly accept false greens or the historical shoe regression."""
import pytest

from scripts.select_robot_vision_profile import CONFIDENCES, MODES, eligible, select_new


def test_bounded_predeclared_grid():
    assert CONFIDENCES==(.15,.25,.35,.45)
    assert MODES==('legacy','body')


@pytest.mark.parametrize('false_green,frames,colours,passes', [
    (0,21,1,True),(0,0,0,True),(1,0,0,False),(0,22,1,False),(0,11,11,False),
])
def test_regression_gates_include_specific_false_signals(false_green,frames,colours,passes):
    assert eligible({'falseGreen':false_green},{'framesWithTrafficFalsePositive':frames,
                 'specificColourFalsePositiveBoxes':colours},frame_limit=21,colour_limit=1) is passes


def point(name,score,passes=True,variant='robot',false_green=0):
    return {'name':name,'variant':variant,'score':score,'eligible':passes,'falseGreen':false_green,
            'macroF1':score,'confidence':.25,'trafficMode':'body'}


def test_best_score_does_not_override_a_failed_safety_regression_gate():
    selected,passes=select_new([point('fail',.99,False),point('ok',.7),point('control',1.,variant='baseline')])
    assert selected['name']=='ok' and passes


def test_no_eligible_candidate_explicitly_blocks_promotion():
    selected,passes=select_new([point('fail',.8,False),point('green-fail',.99,False,false_green=1)])
    assert selected['name']=='fail' and not passes


def test_all_false_green_does_not_become_an_eligible_profile():
    _,passes=select_new([point('fail',.8,False,false_green=3)])
    assert not passes


def test_controls_are_not_mistaken_for_newly_trained_weights():
    with pytest.raises(ValueError,match='no candidate'):
        select_new([point('control',.9,variant='baseline')])
