"""Closed-course UNKNOWN advice is opt-in, fresh, non-actuating and not green."""
from pathlib import Path
import time

import pytest

from robot_control.vision.closed_course import ClosedCourseWaitPolicy
from robot_control.vision.detector import Detection
from robot_control.vision.service import VisionConfig, VisionService

BOX = (.2, .1, .1, .5)


def observe(policy, second, **changes):
    args = dict(camera_id=0, box=BOX, frame_stamp=1000 + second,
                frame_age_s=0., signal='unknown', confirmed_signal='unknown', now=second)
    args.update(changes)
    return policy.update(**args)


def waiting(policy, stop=5.):
    result = None
    for tick in range(int(stop * 2) + 1):
        result = observe(policy, tick / 2)
    return result


def test_disabled_by_default_even_with_many_unknown_frames():
    policy = ClosedCourseWaitPolicy()
    result = waiting(policy, 10)
    assert result['phase'] == 'disabled' and result['recommendation'] == 'none'


def test_exact_five_seconds_on_distinct_fresh_frames_yields_only_separate_advice():
    policy = ClosedCourseWaitPolicy(enabled=True)
    assert waiting(policy, 4.5)['recommendation'] == 'wait'
    assert observe(policy, 4.999)['recommendation'] == 'wait'
    result = observe(policy, 5.)
    assert result['phase'] == 'timeout_advisory'
    assert result['recommendation'] == 'proceed_closed_course' and result['elapsedS'] == 5
    assert not result['controlsMotors']
    assert 'state' not in result and 'action' not in result  # never fabricate green/go


def test_one_frame_followed_by_polling_never_finishes_the_countdown():
    policy = ClosedCourseWaitPolicy(enabled=True)
    observe(policy, 0)
    for second in (.1, .5, 1., 2., 5., 100.):
        assert policy.snapshot(now=second)['recommendation'] != 'proceed_closed_course'
    assert observe(policy, 101)['elapsedS'] == 0


def test_duplicate_capture_stamp_cannot_advance_waiting_or_renew_advice():
    policy = ClosedCourseWaitPolicy(enabled=True)
    waiting(policy)
    for second in (5.5, 6., 6.1, 8.):
        result = observe(policy, second, frame_stamp=1005.)
    assert result['phase'] == 'stale' and result['recommendation'] == 'wait'
    assert observe(policy, 8.5)['elapsedS'] == 0


def test_frame_gap_cancels_previous_observation_time():
    policy = ClosedCourseWaitPolicy(enabled=True)
    waiting(policy, 4.5)
    assert observe(policy, 6.)['elapsedS'] == 0
    assert observe(policy, 6.5)['recommendation'] == 'wait'


@pytest.mark.parametrize('age', [-.01, 1.01, float('nan'), float('inf')])
def test_stale_future_or_invalid_frame_age_never_permits_timeout(age):
    policy = ClosedCourseWaitPolicy(enabled=True)
    waiting(policy, 4.5)
    result = observe(policy, 5., frame_age_s=age)
    assert result['phase'] == 'invalid_or_stale_frame' and result['recommendation'] == 'wait'


@pytest.mark.parametrize('box', [(0., 0., 0., .5), (0., 0., .5),
                                 (float('nan'), .1, .1, .5), (.9, .1, .2, .5)])
def test_invalid_body_box_cannot_continue_a_wait(box):
    policy = ClosedCourseWaitPolicy(enabled=True)
    waiting(policy, 4.5)
    assert observe(policy, 5., box=box)['recommendation'] == 'wait'


@pytest.mark.parametrize('changes', [{'camera_id': 1}, {'box': (.7, .1, .1, .5)}])
def test_different_camera_or_body_restarts_the_wait(changes):
    policy = ClosedCourseWaitPolicy(enabled=True)
    waiting(policy, 4.5)
    assert observe(policy, 5., **changes)['elapsedS'] == 0


@pytest.mark.parametrize('signal', ['red', 'yellow'])
def test_known_stop_colour_is_latched_and_unknown_cannot_time_it_out(signal):
    policy = ClosedCourseWaitPolicy(enabled=True)
    waiting(policy)
    assert observe(policy, 5.5, signal=signal)['stopLatched']
    policy.invalidate('camera_lost')  # losing the camera does NOT erase a red
    for step in range(12, 40):
        result = observe(policy, step / 2)
        assert result['recommendation'] == 'wait' and result['stopLatched']


def test_other_camera_red_blocks_even_a_duplicate_selected_unknown_frame():
    policy = ClosedCourseWaitPolicy(enabled=True)
    waiting(policy)
    result = observe(policy, 5.1, frame_stamp=1005., stop_signal_present=True)
    assert result['stopLatched'] and result['recommendation'] == 'wait'


def test_only_new_confirmed_green_clears_the_stop_latch():
    policy = ClosedCourseWaitPolicy(enabled=True)
    observe(policy, 0, signal='red')
    assert observe(policy, .5, signal='green')['stopLatched']
    result = observe(policy, 1., signal='green', confirmed_signal='green')
    assert not result['stopLatched'] and result['phase'] == 'confirmed_green'
    assert result['recommendation'] == 'none'  # normal tracker alone publishes green/go
    assert observe(policy, 1.5)['elapsedS'] == 0


def test_unconfirmed_green_does_not_count_as_unknown_waiting():
    policy = ClosedCourseWaitPolicy(enabled=True)
    waiting(policy, 4.5)
    assert observe(policy, 5., signal='green')['phase'] == 'awaiting_signal_confirmation'
    assert observe(policy, 5.5)['elapsedS'] == 0


@pytest.mark.parametrize('change', ['wall_jump', 'out_of_order', 'monotonic_backwards'])
def test_time_jumps_and_reordered_frames_cannot_complete_waiting(change):
    policy = ClosedCourseWaitPolicy(enabled=True)
    waiting(policy, 4.5)
    if change == 'wall_jump':
        result = observe(policy, 5., frame_stamp=1200.)
    elif change == 'out_of_order':
        result = observe(policy, 5., frame_stamp=1001.)
    else:
        result = observe(policy, 5., now=3.)
    assert result['recommendation'] == 'wait' and result['elapsedS'] == 0


def test_snapshot_stales_after_at_most_one_second_even_with_long_colour_ttl():
    policy = ClosedCourseWaitPolicy(enabled=True, stale_s=30)
    waiting(policy)
    assert policy.snapshot(now=6.01)['recommendation'] == 'wait'


def make_service():
    clock = [time.monotonic() - 5]
    wall_start = time.time() - 5
    service = VisionService(config=VisionConfig(model_path=Path('/missing.pt'),
                            closed_course_unknown_timeout=True),
                            camera_provider=lambda: None, layout_provider=lambda: [])
    service._detector = object()  # no model/camera/USB initialization
    service._backend = 'test'
    service._closed_course = ClosedCourseWaitPolicy(enabled=True, clock=lambda: clock[0])
    monotonic_start = clock[0]

    def tick(second, signal='unknown', detections=None):
        clock[0] = monotonic_start + second
        wall = wall_start + second
        camera = service._cameras[0]
        camera.jpeg = b'frame'
        camera.stamp = camera.evidence_stamp = wall
        camera.detections = detections if detections is not None else [Detection('traffic_light', BOX, .9, signal)]
        service._update_traffic(wall)
        return service._traffic.snapshot(now=wall), service._closed_course.snapshot()
    return service, tick


def service_wait(tick):
    for step in range(11):
        traffic, course = tick(step / 2)
    return traffic, course


def test_real_service_keeps_unknown_wait_and_separately_exposes_timeout_in_api():
    from flask import Flask
    from robot_control.vision.api import register_vision_routes
    service, tick = make_service()
    traffic, course = service_wait(tick)
    assert traffic['state'] == 'unknown' and traffic['action'] == 'wait'
    assert course['recommendation'] == 'proceed_closed_course'
    app = Flask(__name__)
    register_vision_routes(app, service=service, autostart=False)
    for endpoint in ('state', 'field'):
        payload = app.test_client().get('/api/vision/' + endpoint).get_json()['trafficLight']
        assert payload['state'] == 'unknown' and payload['action'] == 'wait'
        assert payload['closedCourse']['recommendation'] == 'proceed_closed_course'
        assert payload['closedCourse']['controlsMotors'] is False


def test_service_no_detection_resets_the_countdown():
    service, tick = make_service()
    service_wait(tick)
    _, result = tick(5.5, detections=[])
    assert result['recommendation'] == 'wait'
    assert tick(6.)[1]['elapsedS'] == 0


def test_service_ignores_a_reprocessed_old_capture_despite_new_processing_time():
    service, tick = make_service()
    service_wait(tick)
    camera = service._cameras[0]
    camera.evidence_stamp -= 10
    service._update_traffic(camera.stamp)
    assert service._closed_course.snapshot()['recommendation'] == 'wait'


def test_lower_confidence_red_from_another_camera_still_blocks_timeout():
    service, tick = make_service()
    service_wait(tick)
    state = service._cameras[1]
    state.jpeg = b'frame'
    state.stamp = state.evidence_stamp = service._cameras[0].stamp + .5
    state.detections = [Detection('traffic_light', BOX, .5, 'red')]
    _, result = tick(5.5)
    assert result['stopLatched'] and result['recommendation'] == 'wait'


def test_broken_jpeg_cancels_existing_timeout_advice_immediately():
    pytest.importorskip('cv2')
    service, tick = make_service()
    service_wait(tick)
    result = service.process_frame(0, b'not a JPEG')
    assert result['error']
    assert service.state()['trafficLight']['closedCourse']['recommendation'] == 'wait'


def test_disabling_and_stopping_cancel_advice_but_do_not_erase_red(monkeypatch):
    service, tick = make_service()
    tick(0, 'red')
    service.configure(enabled=False)
    assert service._closed_course.snapshot()['stopLatched']
    monkeypatch.setattr(service, 'start', lambda: None)
    service.configure(enabled=True)
    assert service._closed_course.snapshot()['stopLatched']
    assert tick(5.)[1]['recommendation'] == 'wait'
    service.stop()
    assert service._closed_course.snapshot()['stopLatched']
    service.reset()  # explicit full observation reset, not a drive command
    assert not service._closed_course.snapshot()['stopLatched']


def test_env_is_explicit_opt_in_and_not_enabled_by_http(monkeypatch):
    from flask import Flask
    from robot_control.vision.api import register_vision_routes
    monkeypatch.delenv('RC_VISION_CLOSED_COURSE_TIMEOUT', raising=False)
    assert not VisionConfig.from_env().closed_course_unknown_timeout
    monkeypatch.setenv('RC_VISION_CLOSED_COURSE_TIMEOUT', '1')
    assert VisionConfig.from_env().closed_course_unknown_timeout
    service = VisionService(config=VisionConfig(closed_course_unknown_timeout=False),
                            camera_provider=lambda: None, layout_provider=lambda: [])
    service._detector = object()
    monkeypatch.setattr(service, 'start', lambda: None)
    app = Flask(__name__)
    register_vision_routes(app, service=service, autostart=False)
    payload = app.test_client().post('/api/vision/config', json={'closedCourseTimeout': True}).get_json()
    assert payload['trafficLight']['closedCourse']['enabled'] is False
