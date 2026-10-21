"""Shared circular wheel control: events, actual motor UI integration and assets."""
import shutil
import subprocess
from pathlib import Path

import pytest

from conftest import login

ROOT = Path(__file__).resolve().parents[1]


def test_shared_dial_gestures_keyboard_and_geometry():
    node = shutil.which('node')
    if not node:
        pytest.skip('node is not installed')
    result = subprocess.run([node, 'tests/js/steer_dial.js'], cwd=ROOT,
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr


def test_onboard_and_remote_pages_load_same_dial(client, operator, kiosk_client):
    login(client)
    for page in (kiosk_client.get('/robot.html'), client.get('/controls')):
        assert page.status_code == 200
        assert 'steer_dial.css' in page.text
        assert page.text.index('steer_dial.js') < page.text.index('src="robot.js"')
    for app_client in (client, kiosk_client):
        for asset in ('steer_dial.css', 'steer_dial.js'):
            assert app_client.get('/' + asset).status_code == 200
    for route in ('/', '/console'):
        page = client.get(route)
        assert 'steer_dial.css' in page.text
        assert page.text.index('steer_dial.js') < page.text.index('src="steering.js"')


@pytest.mark.parametrize('remote', [False, True])
def test_dial_drives_only_selected_wheel_once_on_release(tmp_path, remote):
    from test_steering_screen import payload, run_screen, angles_sent
    body = payload(dials=True, actions=[{'type':'dial', 'module':'FR', 'angle':20, 'move':35}])
    if remote:
        body['remoteView'] = 'motors'
    report = run_screen(tmp_path, body)
    assert not report['renderErrors']
    assert angles_sent(report) == [('FR', 35)]
    assert not report['drives']


@pytest.mark.parametrize('extra', [{'cancel': True}, {'disable': True}])
def test_cancelled_dial_never_sends_motor_command(tmp_path, extra):
    from test_steering_screen import payload, run_screen, angles_sent
    report = run_screen(tmp_path, payload(dials=True, actions=[{'type':'dial', 'angle':40, **extra}]))
    assert not report['renderErrors']
    assert not angles_sent(report)


@pytest.mark.parametrize('case', ['offline', 'unhomed', 'traction', 'busy'])
def test_existing_safety_interlocks_disable_dial(tmp_path, case):
    from test_steering_screen import payload, motor, run_screen, angles_sent
    motors = [motor(m) for m in ('FL', 'FR', 'RL', 'RR')]
    if case == 'offline': motors[0]['online'] = False
    if case == 'unhomed': motors[0]['homed'] = False
    if case == 'busy': motors[0]['moving'] = True
    body = payload(dials=True, motors=motors, actions=[{'type':'dial', 'angle':35}])
    if case == 'traction': body['data']['drive']['active'] = True
    report = run_screen(tmp_path, body)
    assert not report['renderErrors']
    assert not angles_sent(report)
    assert report['dialStates'][0]['disabled'] == 'true'


def test_dial_honors_server_angle_limit(tmp_path):
    from test_steering_screen import payload, run_screen, angles_sent
    report = run_screen(tmp_path, payload(dials=True, limit=45, actions=[{'type':'dial', 'angle':85}]))
    assert not report['renderErrors']
    assert angles_sent(report) == [('FL', 45)]
