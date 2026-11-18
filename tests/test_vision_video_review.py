"""Output-only temporal summaries must not silently become accuracy metrics."""
import hashlib
import math

import pytest

from scripts.review_vision_video import output_state, sha256, state_runs


def test_no_traffic_output_is_not_called_a_miss():
    assert output_state([]) == 'no_detection'
    assert output_state([{'class': 'cone', 'signal': None}]) == 'no_detection'


def test_unknown_color_is_not_absence_or_green():
    assert output_state([{'class': 'traffic_light', 'signal': None}]) == 'unknown'
    assert output_state([{'class': 'traffic_light', 'signal': 'unknown'}]) == 'unknown'


def test_multiple_traffic_outputs_are_preserved_and_deduplicated():
    predictions = [{'class': 'traffic_light', 'signal': value}
                   for value in ('green', 'red', 'red', 'unknown')]
    assert output_state(predictions) == 'green|red|unknown'


def test_state_runs_use_half_open_contiguous_frame_intervals():
    states = ['no_detection', 'no_detection', 'red', 'unknown', 'unknown', 'red']
    runs = state_runs(states, 2)
    assert [(r['state'], r['startFrame'], r['endFrameExclusive']) for r in runs] == [
        ('no_detection', 0, 2), ('red', 2, 3), ('unknown', 3, 5), ('red', 5, 6)]
    assert [r['frames'] for r in runs] == [2, 1, 2, 1]
    assert [r['startS'] for r in runs] == [0, 1, 1.5, 2.5]
    assert [r['endS'] for r in runs] == [1, 1.5, 2.5, 3]
    assert sum(r['frames'] for r in runs) == len(states)


def test_empty_sequence_has_no_fabricated_spans():
    assert state_runs([], 29.97) == []


@pytest.mark.parametrize('fps', [0, -1, math.inf, -math.inf, math.nan])
def test_invalid_frame_rates_rejected(fps):
    with pytest.raises(ValueError, match='fps'):
        state_runs(['red'], fps)


def test_frame_hash_is_of_exact_bytes(tmp_path):
    image = tmp_path / 'source.mp4'
    data = b'not a real video\0' * 100000
    image.write_bytes(data)
    assert sha256(image) == hashlib.sha256(data).hexdigest()
