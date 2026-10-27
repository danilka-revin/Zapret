"""Authenticated remote motor control reaches the one onboard USB owner."""
import io
import json
import time
from urllib.error import URLError

import pytest

from conftest import login
from robot_control import map_peer, slam


class Reply(io.BytesIO):
    def __init__(self, body, code=200):
        super().__init__(json.dumps(body).encode())
        self.code = code


class Motors:
    supports_individual_drive = True
    name = 'master'
    supports_individual_steer = supports_drive = supports_calibration = True
    MAX_STEER_DEG = 100

    def __init__(self):
        self.calls = []

    def read(self):
        data = slam.load_backend().NoSource().read()
        data.update(linkOk=True, drive={'available': True, 'canDrive': True})
        return data

    def steer_module(self, module, angle):
        self.calls.append(('steer', module, angle))
        return {'ok': True}

    def zero_module(self, module):
        self.calls.append(('zero', module))
        return {'ok': True}

    def calibrate_module(self, module):
        self.calls.append(('calibrate', module))
        return {'ok': True}

    def set_pot_mode(self, action):
        self.calls.append(('pot_mode', action))
        return {'ok': True}

    def drive(self, percent):
        self.calls.append(('drive', percent))
        return {'ok': True, 'percent': percent}

    def drive_module(self, module, percent):
        self.calls.append(('drive', module, percent))
        return {'ok': True, 'queued': True, 'module': module, 'percent': round(percent)}

    def set_drive_config(self, action, value=None, module=None):
        self.calls.append(('drive_config', action, value, module))
        return {'ok': True, 'queued': True}


@pytest.fixture
def transport(kiosk_client, monkeypatch):
    motor = Motors()
    kiosk_client.application.extensions['rc_slam'].source = motor
    calls = []

    def forward(req, timeout):
        from urllib.parse import urlsplit
        calls.append((req.full_url, req.method, dict(req.header_items())))
        response = kiosk_client.open(urlsplit(req.full_url).path, method=req.method,
            data=req.data, headers=dict(req.header_items()), content_type='application/json')
        return Reply(response.json, response.status_code)

    monkeypatch.setattr(map_peer, 'urlopen', forward)
    return motor, calls


def headers(client, marker='steering'):
    return {'X-Robot-Control': marker,
            'X-CSRF-Token': client.get('/api/control/session').json['csrfToken']}


def test_controls_require_operator_and_csrf(client, operator, transport):
    assert client.get('/controls').status_code == 302
    assert client.get('/api/control/state').status_code in (302, 401)
    assert client.post('/api/control/drive', json={'percent': 8}).status_code in (302, 401)
    login(client)
    assert client.post('/api/control/drive', json={'percent': 8}).json['reason'] == 'request_header_required'
    assert client.post('/api/control/drive', json={'percent': 8},
                       headers={'X-Robot-Control': 'drive'}).json['reason'] == 'csrf_required'
    assert client.get('/api/control/steer').status_code == 405
    assert client.post('/api/control/arbitrary').status_code == 404
    assert transport[1] == []


def test_controls_page_reuses_full_motor_ui(client, operator):
    login(client)
    assert 'id="nav-controls"' in client.get('/').text
    assert 'id="console-controls-link"' in client.get('/console').text
    page = client.get('/controls')
    assert page.status_code == 200
    for name in ('swipe-motors-view', 'steer-slider-FL', 'steer-preset-grid',
                 'drive-forward', 'drive-back', 'drive-stop', 'wheel-port-settings', 'remote-estop'):
        assert f'id="{name}"' in page.text
    assert page.text.index('web_controls.js') < page.text.index('src="robot.js"')
    assert client.get('/web_controls.js').status_code == 200


@pytest.mark.parametrize('operation,body,marker,expected', [
    ('steer', {'module': 'FR', 'angle': 12}, 'steering', ('steer', 'FR', 12)),
    ('zero', {'module': 'FL'}, 'steering', ('zero', 'FL')),
    ('calibrate', {'module': 'ALL'}, 'steering', ('calibrate', 'ALL')),
    ('pot_mode', {'action': 'on'}, 'steering', ('pot_mode', 'on')),
    ('drive_config', {'action': 'tach', 'value': 'off'}, 'steering',
     ('drive_config', 'tach', 'off', None)),
    ('drive_config', {'action': 'trim', 'value': 1620}, 'steering',
     ('drive_config', 'trim', 1620, None)),
    ('drive_config', {'action': 'span', 'value': 300}, 'steering',
     ('drive_config', 'span', 300, None)),
    ('drive_config', {'action': 'stop', 'value': 'off'}, 'steering',
     ('drive_config', 'stop', 'off', None)),
    # Сторона вращения — настройка одного колеса: имя доезжает до бортового
    # сервера, иначе реверс получили бы все четыре платы сразу.
    ('drive_config', {'action': 'direction', 'module': 'FR', 'value': 'reverse'}, 'steering',
     ('drive_config', 'direction', 'reverse', 'FR')),
    ('drive', {'percent': -8}, 'drive', ('drive', -8)),
    ('drive', {'percent': 0}, 'drive', ('drive', 0)),
    # Тяга одного колеса идёт тем же маршрутом и тем же заголовком команды.
    ('drive', {'module': 'FL', 'percent': 6}, 'drive', ('drive', 'FL', 6)),
    ('drive', {'module': 'fr', 'percent': -3}, 'drive', ('drive', 'FR', -3)),
    ('drive', {'module': 'RL', 'percent': 0}, 'drive', ('drive', 'RL', 0)),
])
def test_all_commands_reach_onboard_owner(client, operator, transport, operation, body, marker, expected):
    login(client)
    response = client.post('/api/control/' + operation, json=body, headers=headers(client, marker))
    assert response.status_code == 200, response.json
    assert transport[0].calls[-1] == expected
    assert all(url.startswith('http://127.0.0.1:8090/api/') for url, _, _ in transport[1])
    assert not hasattr(client.application.extensions['rc_slam'].source, 'links')
    telemetry = client.get('/api/control/state').json['data']
    assert telemetry['steeringControl']['available'] is True


def test_onboard_rejections_remain_effective(client, operator, transport):
    login(client)
    assert client.post('/api/control/drive', json={'percent': 21},
                       headers=headers(client, 'drive')).status_code == 400
    assert client.post('/api/control/steer', json={'module': 'FL', 'angle': 101},
                       headers=headers(client)).status_code == 400
    stop = client.post('/api/control/estop', json={}, headers=headers(client, 'drive'))
    assert stop.status_code == 200 and stop.json['ok']
    response = client.post('/api/control/drive', json={'percent': 8}, headers=headers(client, 'drive'))
    assert response.status_code == 409 and response.json['reason'] == 'estop'
    assert client.post('/api/control/drive', json={'percent': 0},
                       headers=headers(client, 'drive')).status_code == 200


def test_offline_owner_fails_closed(client, operator, monkeypatch):
    login(client)
    def failed(*args, **kwargs):
        raise URLError('offline')
    monkeypatch.setattr(map_peer, 'urlopen', failed)
    assert client.get('/api/control/state').status_code == 503
    response = client.post('/api/control/drive', json={'percent': 8}, headers=headers(client, 'drive'))
    assert response.status_code == 503 and not response.json['ok']


def test_expired_remote_heartbeat_cannot_start_drive(kiosk_client, transport):
    hdr = {'X-Robot-Control': 'drive', 'X-Robot-Command-Deadline': str(time.time() - 1)}
    response = kiosk_client.post('/api/kiosk_drive', json={'percent': 8}, headers=hdr)
    assert response.status_code == 408 and response.json['reason'] == 'command_expired'
    assert transport[0].calls == []
    assert kiosk_client.post('/api/kiosk_drive', json={'percent': 0}, headers=hdr).status_code == 200


def test_browser_transport_and_unload_stop():
    import shutil
    import subprocess
    from pathlib import Path
    node = shutil.which('node')
    if not node:
        pytest.skip('node needed for browser transport test')
    result = subprocess.run([node, str(Path(__file__).parent / 'js/web_controls.js')],
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr


def test_manual_ports_can_be_saved_from_web_without_autodetection(client, operator, transport, monkeypatch, tmp_path):
    from robot_control import wheel_settings
    target = tmp_path / '.env'
    monkeypatch.setattr(wheel_settings, 'env_file_path', lambda: target)
    monkeypatch.setattr(wheel_settings, 'reserved_ports', lambda: set())
    login(client)
    assert client.get('/api/control/wheels/settings').json['canEdit']
    result = client.post('/api/control/wheels/settings', json={'FR': '/dev/ttyUSB3'},
                         headers=headers(client, 'wheel-settings'))
    assert result.status_code == 200 and result.json['restartRequired']
    assert 'RC_WHEEL_FR_PORT=/dev/ttyUSB3' in target.read_text()
    assert client.get('/api/control/wheels/settings').json['settings']['FR'] == '/dev/ttyUSB3'


def test_proxy_generates_deadline_and_does_not_forward_browser_auth_headers(client, operator, transport):
    login(client)
    hdr = headers(client, 'drive')
    hdr.update({'Authorization': 'do-not-forward', 'X-Robot-Command-Deadline': '99999999999999'})
    assert client.post('/api/control/drive', json={'percent': 8}, headers=hdr).status_code == 200
    forwarded = {key.lower(): val for key, val in transport[1][-1][2].items()}
    assert 'authorization' not in forwarded and 'x-csrf-token' not in forwarded
    deadline = float(forwarded['x-robot-command-deadline'])
    assert time.time() - 0.5 <= deadline <= time.time() + 0.5


def test_battery_tab_auth_full_ui_and_navigation(client, operator, transport):
    for path in ('/battery', '/api/control/battery', '/api/control/battery/report.csv'):
        assert client.get(path).status_code in (302, 401)
    assert not transport[1]
    login(client)
    assert 'id="nav-battery"' in client.get('/').text
    assert 'id="console-battery-link"' in client.get('/console').text
    page = client.get('/battery')
    assert page.status_code == 200
    assert 'data-remote-view="battery"' in page.text
    assert 'id="bms-setup-open" hidden' in page.text
    assert 'id="remote-battery-link"' in client.get('/controls').text
    for name in ('bms-soc', 'bms-summary', 'bms-cells', 'bms-cell-tiles', 'bms-spread',
                 'bms-fets', 'bms-faults', 'bms-counters', 'bms-temps', 'bms-passport',
                 'bms-thresholds', 'bms-report'):
        assert f'id="{name}"' in page.text
    assert page.text.index('web_controls.js') < page.text.index('src="robot.js"')


def test_battery_reads_only_existing_owner(client, operator, transport, kiosk_client, monkeypatch):
    from robot_control.battery import parse_snapshot
    # A complete 32-bank sample, no serial transport created on either server.
    basic = bytearray(27)
    basic[0:2] = (10240).to_bytes(2, 'big')
    basic[2:4] = (-1250).to_bytes(2, 'big', signed=True)
    basic[4:6] = (2500).to_bytes(2, 'big')
    basic[6:8] = (5000).to_bytes(2, 'big')
    basic[18:23] = bytes([0x21, 50, 3, 32, 0])
    cells = b''.join((3200 + i).to_bytes(2, 'big') for i in range(32))
    manager = kiosk_client.application.extensions['rc_battery']
    manager.port = '/dev/onboard-bms'
    manager._sample = parse_snapshot(bytes(basic), cells)
    manager._error = ''
    manager._device_name = 'JBD remote test'
    manager._thresholds = {'covp': (3.65, 'В')}
    manager._threshold_queue = []
    login(client)
    monkeypatch.setenv('RC_SLAM_PEER_URL', 'http://onboard.example:8090/api/kiosk_state')
    response = client.get('/api/control/battery')
    assert response.status_code == 200
    snapshot = response.json['battery']
    assert snapshot['connected']
    assert snapshot['data']['cellCount'] == 32
    assert snapshot['data']['deviceName'] == 'JBD remote test'
    assert snapshot['data']['thresholds'][0]['value'] == 3.65
    assert snapshot['data']['cells'][-1]['mv'] == 3231
    assert transport[1][-1][0] == 'http://onboard.example:8090/api/battery'
    report = client.get('/api/control/battery/report.csv')
    assert report.status_code == 200
    assert report.content_type.startswith('text/csv')
    text = report.data.decode('utf-8-sig')
    assert '32;3231' in text and 'JBD remote test' in text
    assert manager._thread is None
    assert transport[0].calls == []
    assert client.post('/api/control/battery', json={}).status_code == 405
    for operation in ('setup', 'port', 'probe', 'scan'):
        assert client.post('/api/control/battery/' + operation, json={}).status_code == 404


def test_battery_report_without_data_or_owner(client, operator, transport, monkeypatch):
    login(client)
    assert client.get('/api/control/battery/report.csv').status_code == 503
    def offline(*args, **kwargs):
        raise URLError('owner offline')
    monkeypatch.setattr(map_peer, 'urlopen', offline)
    assert client.get('/api/control/battery').status_code == 503
    assert client.get('/api/control/battery/report.csv').status_code == 503
    monkeypatch.setattr(map_peer, 'urlopen', lambda *args, **kwargs: Reply({'ok': True, 'battery': {}}))
    assert client.get('/api/control/battery/report.csv').status_code == 503


def test_remote_battery_renderer_and_startup(tmp_path):
    import shutil
    import subprocess
    from pathlib import Path
    node = shutil.which('node')
    if not node:
        pytest.skip('node is not installed')
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run([node, str(root / 'tests/js/battery_screen.js'), '--remote'],
                            cwd=root, capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
    fixture = tmp_path / 'remote-battery.json'
    fixture.write_text(json.dumps({'ok': True, 'data': {}, 'remoteView': 'battery'}))
    result = subprocess.run([node, str(root / 'tests/js/steering_screen.js'), str(fixture)],
                            cwd=root, capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
    rendered = json.loads(result.stdout)
    assert rendered['batteryVisible'] and not rendered['motorsVisible']
    assert not rendered['drives']
    assert not rendered['renderErrors']


def test_remote_logs_read_board_journal_with_bounded_limit(client, operator, transport):
    assert client.get('/api/control/logs').status_code in (302, 401)
    login(client)
    from robot_control import eventlog
    eventlog.record('warn', 'Нет связи с FR', source='мастер')
    response = client.get('/api/control/logs?limit=400')
    assert response.status_code == 200
    data = response.json['data']
    assert data['entries'][-1]['group'] == 'wheels'
    assert data['groups']['wheels']['warnings'] == 1
    assert transport[1][-1][0].endswith('/api/kiosk_logs?limit=400')
    client.get('/api/control/logs?limit=999999&url=https://unrelated.example')
    assert transport[1][-1][0].endswith('/api/kiosk_logs?limit=5000')
    assert client.post('/api/control/logs', json={}).status_code == 405
    assert transport[0].calls == []
