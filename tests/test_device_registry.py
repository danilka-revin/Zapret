"""Device registry tests: synthetic sysfs, untrusted hints and onboard API.

No physical devices are opened by the registry. PTY/Chromium tests elsewhere
exercise the live wheel owner; these tests cannot validate physical wiring.
"""
from __future__ import annotations

import copy
import io
import json
import os
import stat
import threading
from pathlib import Path
from urllib.parse import urlsplit

import pytest
from flask import Flask

from conftest import login
from robot_control import map_peer
from robot_control.device_registry import DeviceRegistry, UsbTopology, _resolve_identity


def node(path, kind, *, interface='1.0', index='', aliases=()):
    return {'path': path, 'kind': kind, 'aliases': list(aliases),
            'interface': interface, 'index': index, 'name': ''}


def usb(topology, vidpid, serial='', nodes=(), *, printer=False, product='USB device'):
    return {'topology': topology, 'vidPid': vidpid, 'serial': serial,
            'manufacturer': 'Maker', 'product': product, 'class': '00', 'interfaces': ['07'] if printer else [],
            'busPath': '', 'nodes': list(nodes), 'connection': 'через хаб · 1-2', 'parent': '1-2',
            'isHub': False, 'printerClass': printer}


class MutableTopology:
    def __init__(self, devices):
        self.devices = devices
        self.calls = 0

    def collect(self):
        self.calls += 1
        return {'available': True, 'devices': copy.deepcopy(self.devices), 'collectedAt': 1000}


@pytest.fixture
def fake_topology():
    return MutableTopology([
        usb('1-2.1', '1a86:7523', '', [node('/dev/ttyUSB0', 'serial')], product='Wheel USB'),
        usb('1-2.2', '1a86:7523', '', [node('/dev/ttyUSB1', 'serial')], product='Wheel USB'),
        usb('1-4', '046d:0843', 'CAM-123', [node('/dev/video0', 'video', index='0'),
            node('/dev/video1', 'video', index='1')], product='Camera'),
        usb('1-5', '046d:0843', 'CAM-456', [node('/dev/video2', 'video', index='0')], product='Camera'),
        usb('1-6', '045e:02ae', 'K-1', product='Kinect'),
        usb('1-7', '1234:5678', 'P-1', [node('/dev/usb/lp0', 'printer')], printer=True, product='Printer'),
    ])


def _attributes(path: Path, **values):
    path.mkdir(parents=True, exist_ok=True)
    for key, value in values.items():
        (path / key).write_text(str(value), encoding='utf-8')


def test_sysfs_maps_hub_interfaces_and_aliases_without_opening_devices(tmp_path, monkeypatch):
    root, dev = tmp_path / 'sys', tmp_path / 'dev'
    actual = root / 'devices/pci/usb1'
    hub = actual / '1-2'
    wheel = hub / '1-2.3'
    printer = actual / '1-4'
    _attributes(actual, idVendor='1d6b', idProduct='0002', bDeviceClass='09', product='Root hub')
    _attributes(hub, idVendor='1234', idProduct='5678', bDeviceClass='09', product='Powered hub')
    _attributes(wheel, idVendor='1a86', idProduct='7523', product='CH340', serial='WHEEL', busnum='1', devnum='7')
    _attributes(printer, idVendor='1111', idProduct='2222', product='Receipt printer', busnum='1', devnum='9')
    _attributes(wheel / '1-2.3:1.0', bInterfaceClass='ff')
    _attributes(printer / '1-4:1.0', bInterfaceClass='07')
    bus = root / 'bus/usb/devices'; bus.mkdir(parents=True)
    for path in (actual, hub, wheel, wheel / '1-2.3:1.0', printer, printer / '1-4:1.0'):
        (bus / path.name).symlink_to(path)
    tty = root / 'class/tty/ttyUSB0'; tty.mkdir(parents=True)
    (tty / 'device').symlink_to(wheel / '1-2.3:1.0')
    (dev / 'serial/by-path').mkdir(parents=True)
    (dev / 'ttyUSB0').touch()
    (dev / 'serial/by-path/usb-hub-port3').symlink_to(dev / 'ttyUSB0')

    import serial
    monkeypatch.setattr(serial, 'Serial', lambda *a, **kw: pytest.fail('registry must not open serial ports'))
    original = io.open
    def no_device_open(file, *args, **kwargs):
        assert not str(file).startswith(str(dev)), f'device opened: {file}'
        return original(file, *args, **kwargs)
    monkeypatch.setattr(io, 'open', no_device_open)
    result = UsbTopology(root, dev).collect()
    devices = {item['topology']: item for item in result['devices']}
    assert result['available'] is True
    assert devices['1-2.3']['connection'] == 'через Powered hub · 1-2'
    assert devices['1-2.3']['busPath'] == str(dev / 'bus/usb/001/007')
    tty_node = devices['1-2.3']['nodes'][0]
    assert tty_node['path'] == str(dev / 'ttyUSB0') and tty_node['interface'] == '1.0'
    assert tty_node['aliases'] == [str(dev / 'serial/by-path/usb-hub-port3')]
    assert devices['1-4']['printerClass'] is True
    assert devices['usb1']['isHub'] and devices['1-2']['isHub']


def test_hints_are_persisted_atomically_validated_and_never_classify(fake_topology, tmp_path, monkeypatch):
    path = tmp_path / 'data/registry.json'
    reg = DeviceRegistry(path, topology=fake_topology, clock=lambda: 1000)
    reg.set_hint('wheel_FR', '/dev/ttyUSB1')
    assert stat.S_IMODE(path.stat().st_mode) == 0o600
    assert not list(path.parent.glob('.devices-*.tmp'))
    assert reg.order_serial(['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB9']) == [
        '/dev/ttyUSB1', '/dev/ttyUSB0', '/dev/ttyUSB9']
    snapshot = reg.snapshot()
    assert len(snapshot['devices']) == 12
    by_id = {d['id']: d for d in snapshot['devices']}
    assert by_id['wheel_FR']['port'] is None and by_id['wheel_FR']['status'] != 'verified'
    assert by_id['wheel_FR']['hint']['port'] == '/dev/ttyUSB1'
    assert not snapshot['wheelDiscovery']['ready']
    reloaded = DeviceRegistry(path, topology=fake_topology)
    assert reloaded.records()[0]['wheel_FR']['port'] == '/dev/ttyUSB1'
    for role, value in [('wheel_FL', '/dev/ttyUSB0;touch /tmp/x'), ('wheel_FL', '/etc/passwd'),
                        ('camera_FL', '/dev/ttyUSB0'), ('bms', '/dev/../etc/passwd'),
                        ('sensors', '/dev/ttyUSB0'), ('printer', '/dev/ttyUSB0\n')]:
        with pytest.raises(ValueError):
            reg.set_hint(role, value)
    before = path.read_bytes()
    monkeypatch.setattr(os, 'replace', lambda *a: (_ for _ in ()).throw(OSError('disk full')))
    with pytest.raises(OSError):
        reg.set_hint('wheel_RR', '/dev/ttyUSB0')
    assert path.read_bytes() == before and 'wheel_RR' not in reg.records()[0]
    assert not list(path.parent.glob('.devices-*.tmp'))


def test_camera_identity_rechecks_renumber_swap_duplicates_and_visual_confirmation(fake_topology, tmp_path):
    reg = DeviceRegistry(tmp_path / 'registry.json', topology=fake_topology, clock=lambda: 1000)
    reg.set_hint('camera_FL', '/dev/video0')
    assert reg.camera_target('front_left', '/dev/video9') == '/dev/video0'
    # A guessed/default second role cannot steal this camera's second V4L2 node.
    assert reg.camera_target('rear_left', '/dev/video1') == ''
    with pytest.raises(ValueError, match='уже указана'):
        reg.set_hint('camera_FR', '/dev/video1')
    with pytest.raises(ValueError, match='камер'):
        reg.confirm_camera('camera_FL', [])
    camera = {'id': 0, 'device': '/dev/video9', 'openedPath': '/dev/video0',
              'frameSource': 'capture', 'frameAgeS': 0.1}
    item = lambda: {x['id']: x for x in reg.snapshot(cameras=[camera], refresh=True)['devices']}['camera_FL']
    assert item()['status'] == 'warning' and item()['canConfirm']
    reg.confirm_camera('camera_FL', [camera])
    assert item()['status'] == 'verified' and reg.camera_role_confirmed('front_left') is True
    reg = DeviceRegistry(reg.path, topology=fake_topology, clock=lambda: 1001)
    assert reg.camera_role_confirmed('front_left') is True
    # Same unique USB serial after port renumbering: resolve current node anew.
    camera_device = fake_topology.devices[2]
    camera_device['topology'] = '1-9'
    camera_device['nodes'][0]['path'] = '/dev/video6'
    reg.usb(refresh=True)
    assert reg.camera_target('front_left', '/dev/video0') == '/dev/video6'
    camera['openedPath'] = '/dev/video6'
    assert item()['port'] == '/dev/video6'
    # Device replacement with another serial: never use the old numeric path.
    camera_device['serial'] = 'REPLACEMENT'
    reg.usb(refresh=True)
    assert reg.camera_target('front_left', '/dev/video0') == ''
    assert item()['port'] is None and item()['status'] == 'warning'
    # A cloned serial can match the socket but cannot prove a physical role.
    camera_device['serial'] = 'CAM-123'; camera_device['topology'] = '1-4'
    fake_topology.devices[3]['serial'] = 'CAM-123'
    reg.usb(refresh=True)
    assert reg.camera_target('front_left', '/dev/video0') == '/dev/video6'
    assert reg.camera_role_confirmed('front_left') is False
    assert item()['status'] == 'warning'
    # A different V4L2 interface is not silently accepted.
    camera_device['nodes'][0]['index'] = '7'
    reg.usb(refresh=True)
    assert reg.camera_target('front_left', '/dev/video0') == ''


def test_camera_missing_hint_and_clear_are_fail_closed(fake_topology, tmp_path):
    reg = DeviceRegistry(tmp_path / 'registry.json', topology=fake_topology)
    reg.set_hint('camera_RR', '/dev/video99')
    assert reg.camera_target('rear_right', '/dev/video3') == ''
    # Plugging an unobserved port in later is not enough: the operator must
    # associate it while present, because there was no identity to save.
    fake_topology.devices.append(usb('1-9', '1111:9999', 'fresh', [node('/dev/video99', 'video', index='0')]))
    reg.usb(refresh=True)
    assert reg.camera_target('rear_right', '/dev/video3') == ''
    reg.set_hint('camera_RR', '/dev/video99')
    assert reg.camera_target('rear_right', '/dev/video3') == '/dev/video99'
    reg.set_hint('camera_RR', '')
    assert reg.camera_target('rear_right', '/dev/video3') == ''
    assert DeviceRegistry(reg.path, topology=fake_topology).camera_target('rear_right', '/dev/video3') == ''


def test_snapshot_observes_live_producers_not_cache_or_usb_presence(fake_topology, tmp_path):
    reg = DeviceRegistry(tmp_path / 'registry.json', topology=fake_topology, clock=lambda: 1000)
    reg.set_hint('wheel_FL', '/dev/ttyUSB1')  # intentionally wrong
    wheels = {'masterPort': {'auto': True, 'ready': False, 'error': 'другой USB занят',
                             'unclassified': ['/dev/ttyUSB9'], 'candidates': ['/dev/ttyUSB0', '/dev/ttyUSB9']},
              'motors': [{'id': 'FL', 'port': '/dev/ttyUSB0', 'online': True, 'firmware': 24, 'telemetryAgeS': .1},
                         {'id': 'FR', 'port': '/dev/ttyUSB1', 'online': True, 'firmware': 21, 'telemetryAgeS': .1}]}
    cameras = [{'id': 0, 'device': '/dev/video0', 'openedPath': '/dev/video0',
                'frameSource': 'external', 'frameAgeS': .2}]
    lidar = {'connected': True, 'source': 'api', 'scan_trusted': False, 'age_s': .1,
             'detected': {'port': '/dev/ttyUSB1'}}
    bms = {'connected': True, 'port': '/dev/ttyUSB1', 'ageS': 30}
    # Fresh arrival but ancient capture is not a current Kinect frame.
    kinect = {'connected': True, 'lastStamp': 1.0, 'ageS': .1, 'spec': {'max_age_s': 1.5}}
    snapshot = reg.snapshot(wheels=wheels, cameras=cameras, lidar=lidar, bms=bms, kinect=kinect)
    items = {d['id']: d for d in snapshot['devices']}
    assert items['wheel_FL']['port'] == '/dev/ttyUSB0' and items['wheel_FL']['status'] == 'verified'
    assert items['wheel_FR']['port'] is None
    assert items['camera_FL']['status'] == 'warning' and items['camera_FL']['port'] is None
    assert items['lidar']['status'] == 'missing' and items['lidar']['port'] is None
    assert items['bms']['status'] == 'missing' and items['bms']['port'] is None
    assert items['kinect']['status'] == 'warning' and 'устарел' in items['kinect']['label']
    assert items['kinect']['usbCandidate']['topology'] == '1-6'
    assert items['printer']['status'] == 'warning' and 'печать не проверена' in items['printer']['check']
    assert snapshot['wheelDiscovery']['unclassified'] == ['/dev/ttyUSB9']
    # Restart/cache is not an online signal. Only current producer snapshots count.
    again = DeviceRegistry(reg.path, topology=fake_topology, clock=lambda: 1001).snapshot()
    remembered = {d['id']: d for d in again['devices']}['wheel_FL']
    assert remembered['port'] is None and remembered['status'] != 'verified'
    assert remembered['remembered']['port'] == '/dev/ttyUSB0'


def test_live_lidar_bms_and_ros_reported_port_stay_distinct(fake_topology, tmp_path):
    reg = DeviceRegistry(tmp_path / 'registry.json', topology=fake_topology, clock=lambda: 1000)
    real_lidar = {'connected': True, 'source': 'serial:lds01', 'scan_trusted': True, 'age_s': .2,
                  'detected': {'port': '/dev/ttyUSB0'}}
    bms = {'connected': True, 'port': '/dev/ttyUSB1', 'ageS': .3, 'reason': 'ok'}
    items = {d['id']: d for d in reg.snapshot(lidar=real_lidar, bms=bms)['devices']}
    assert items['lidar']['port'] == '/dev/ttyUSB0' and items['lidar']['status'] == 'verified'
    assert items['bms']['port'] == '/dev/ttyUSB1' and items['bms']['status'] == 'verified'
    ros = {'connected': True, 'source': 'ros', 'scan_trusted': True, 'age_s': .2,
           'detected': {'port': '/dev/ttyUSB0'}, 'ros': {'supervisor': {
               'backend': 'zapret_lidar', 'port': '/dev/ttyUSB1'}}}
    item = {d['id']: d for d in reg.snapshot(lidar=ros)['devices']}['lidar']
    assert item['port'] is None and item['reportedPort'] == '/dev/ttyUSB1'
    assert 'не доказана' in item['detail']


@pytest.mark.parametrize('bad', [True, False, float('nan'), float('inf'), -1.0, 10 ** 400])
def test_untrusted_numeric_values_never_confirm_a_device(fake_topology, tmp_path, bad):
    path = tmp_path / 'registry.json'
    reg = DeviceRegistry(path, topology=fake_topology, clock=lambda: 1000)
    reg.set_hint('camera_FL', '/dev/video0')
    data = json.loads(path.read_text())
    data['hints']['camera_FL']['confirmedAt'] = bad
    data['verified']['wheel_FL'] = {'port': '/dev/ttyUSB0', 'verifiedAt': bad}
    path.write_text(json.dumps(data))
    reg = DeviceRegistry(path, topology=fake_topology, clock=lambda: 1000)
    camera = {'id': 0, 'device': '/dev/video0', 'openedPath': '/dev/video0',
              'frameSource': 'capture', 'frameAgeS': .1}
    assert not reg.camera_role_confirmed('front_left')
    wheels = {'masterPort': {'auto': True}, 'motors': [
        {'id': 'FL', 'online': True, 'firmware': 24, 'port': '/dev/ttyUSB0', 'telemetryAgeS': bad}]}
    kinect = {'connected': True, 'lastStamp': bad, 'ageS': bad, 'spec': {'max_age_s': bad}}
    items = {d['id']: d for d in reg.snapshot(wheels=wheels, cameras=[camera], kinect=kinect)['devices']}
    assert items['camera_FL']['status'] == 'warning'
    assert items['wheel_FL']['status'] != 'verified'
    assert items['kinect']['status'] == 'warning' and 'устарел' in items['kinect']['label']
    camera['frameAgeS'] = bad
    with pytest.raises(ValueError):
        reg.confirm_camera('camera_FL', [camera])


def test_corrupt_cache_and_simultaneous_hint_saves_are_safe(fake_topology, tmp_path):
    path = tmp_path / 'registry.json'
    path.write_text('{bad json', encoding='utf-8')
    reg = DeviceRegistry(path, topology=fake_topology)
    assert reg.error and not reg.records()[0]
    threads = [threading.Thread(target=reg.set_hint, args=(f'wheel_{wheel}', f'/dev/ttyUSB{i}'))
               for i, wheel in enumerate(('FL', 'FR', 'RL', 'RR'))]
    for thread in threads: thread.start()
    for thread in threads: thread.join()
    assert set(json.loads(path.read_text())['hints']) == {'wheel_FL', 'wheel_FR', 'wheel_RL', 'wheel_RR'}
    assert reg.error == ''


def test_registry_disk_stall_never_blocks_wheel_order_or_camera_readers(fake_topology, tmp_path, monkeypatch):
    import time
    reg = DeviceRegistry(tmp_path / 'registry.json', topology=fake_topology)
    reg.set_hint('camera_FL', '/dev/video0')
    entered, release = threading.Event(), threading.Event()
    original = reg._save_locked
    def stalled():
        entered.set()
        assert release.wait(3)
        return original()
    monkeypatch.setattr(reg, '_save_locked', stalled)
    thread = threading.Thread(target=reg.set_hint, args=('wheel_FL', '/dev/ttyUSB1'))
    thread.start()
    try:
        assert entered.wait(2)
        started = time.monotonic()
        assert reg.order_serial(['/dev/ttyUSB0', '/dev/ttyUSB1']) == ['/dev/ttyUSB0', '/dev/ttyUSB1']
        assert reg.camera_target('front_left', '/dev/video0') == '/dev/video0'
        assert time.monotonic() - started < .2
    finally:
        release.set()
        thread.join(timeout=4)
    assert not thread.is_alive()
    assert reg.order_serial(['/dev/ttyUSB0', '/dev/ttyUSB1'])[0] == '/dev/ttyUSB1'


def test_onboard_device_api_requires_marker_validates_and_does_not_touch_motor_routes(kiosk_client):
    assert kiosk_client.get('/devices').status_code == 200
    assert kiosk_client.get('/devices.js').status_code == 200
    snap = kiosk_client.get('/api/devices').json
    assert snap['ok'] and len(snap['devices']) == 12
    assert 'k-devices-open' in kiosk_client.get('/kiosk').text
    assert kiosk_client.post('/api/devices/hint', json={'role': 'wheel_FL', 'port': '/dev/ttyUSB88'}).status_code == 400
    header = {'X-Robot-Control': 'devices'}
    assert kiosk_client.post('/api/devices/hint', json={'role': 'wheel_FL', 'port': '/dev/ttyUSB88'}, headers=header).status_code == 200
    assert kiosk_client.post('/api/devices/hint', json={'role': 'wheel_FL', 'port': '/etc/passwd'}, headers=header).status_code == 400
    assert kiosk_client.post('/api/devices/hint', json={'role': 'wheel_FL', 'port': '', 'extra': 1}, headers=header).status_code == 400
    assert kiosk_client.post('/api/devices/hint', data='x' * 5000, headers=header).status_code == 413
    assert kiosk_client.post('/api/devices/camera/confirm', json={'role': 'camera_FL'}, headers=header).status_code == 400
    assert kiosk_client.get('/api/battery/scan').status_code == 405


def test_web_device_api_auth_csrf_and_fixed_onboard_proxy(client, operator, kiosk_client, monkeypatch):
    assert client.get('/devices').status_code in (302, 401)
    assert client.get('/api/devices').status_code in (302, 401)
    assert client.post('/api/devices/hint', json={}).status_code in (302, 401)
    assert login(client).status_code == 302
    assert 'nav-devices' in client.get('/').text
    assert 'console-devices-link' in client.get('/console').text
    assert client.get('/devices').status_code == 200
    calls = []
    def forward(req, timeout):
        calls.append((req.full_url, req.get_method(), dict(req.header_items())))
        parsed = urlsplit(req.full_url)
        response = kiosk_client.open(parsed.path + ('?' + parsed.query if parsed.query else ''),
                                     method=req.get_method(), data=req.data, headers=dict(req.header_items()))
        result = io.BytesIO(response.data)
        result.code = response.status_code
        result.headers = response.headers
        return result
    monkeypatch.setenv('RC_SLAM_PEER_URL', 'http://owner.example:8090/api/kiosk_state')
    monkeypatch.setattr(map_peer, 'urlopen', forward)
    assert client.get('/api/devices?refresh=1').json['ok']
    token = client.get('/api/control/session').json['csrfToken']
    payload = {'role': 'wheel_FL', 'port': '/dev/ttyUSB7'}
    assert client.post('/api/devices/hint', json=payload).status_code == 400
    assert client.post('/api/devices/hint', json=payload, headers={'X-Robot-Control': 'devices'}).status_code == 400
    good = {'X-Robot-Control': 'devices', 'X-CSRF-Token': token}
    assert client.post('/api/devices/hint', json=payload, headers=good).status_code == 200
    assert client.post('/api/devices/camera/confirm', json={'role': 'camera_FL'}, headers=good).status_code == 400
    assert calls[0][0] == 'http://owner.example:8090/api/devices?refresh=1'
    assert calls[-2][0] == 'http://owner.example:8090/api/devices/hint'
    assert calls[-1][0] == 'http://owner.example:8090/api/devices/camera/confirm'
    assert all('Cookie' not in headers for _, _, headers in calls)


def test_web_peer_camera_paths_never_construct_a_second_owner(monkeypatch):
    from types import SimpleNamespace
    from robot_control import slam
    app = Flask(__name__)
    state = SimpleNamespace(source=SimpleNamespace(name='master-peer'),
                            snapshot=lambda: {'cameras': [{'id': 0}], 'motors': [], 'lidar': []})
    app.register_blueprint(slam.create_blueprint(state, csrf_header_ok=lambda: True))
    monkeypatch.setattr(slam.CameraManager, 'get_instance', lambda: pytest.fail('peer opened a camera'))
    monkeypatch.setattr(slam, 'peer_kiosk_url', lambda: 'http://owner:8090/api/kiosk_state')
    calls = []
    def forward(req, timeout):
        calls.append(req.full_url)
        frame = req.full_url.endswith('/frame')
        result = io.BytesIO(b'jpeg' if frame else b'{"ok":true,"cameras":[]}')
        result.code = 200
        result.headers = {'Content-Type': 'image/jpeg' if frame else 'application/json'}
        return result
    monkeypatch.setattr(map_peer, 'urlopen', forward)
    c = app.test_client()
    assert c.get('/api/cameras').json['ok']
    assert c.get('/api/camera/0/frame').data == b'jpeg'
    assert c.get('/api/camera/8/frame').status_code == 404
    assert c.get('/api/camera/settings').json['ok']
    assert calls == ['http://owner:8090/api/cameras', 'http://owner:8090/api/camera/0/frame',
                     'http://owner:8090/api/camera/settings']


@pytest.mark.skipif(not os.environ.get('ZAPRET_BROWSER_NODE_MODULES'), reason='optional Chromium is not installed')
@pytest.mark.parametrize('remote', [False, True], ids=['onboard', 'authenticated-web'])
def test_camera_devices_page_end_to_end_with_synthetic_v4l2(kiosk_client, fake_topology, app, operator, monkeypatch, remote):
    """Browser workflow with a synthetic frame/USB tree, not a physical camera."""
    import subprocess
    from flask import jsonify, request
    from werkzeug.serving import WSGIRequestHandler, make_server
    from robot_control.camera import CameraManager

    kiosk = kiosk_client.application
    registry = kiosk.extensions['rc_devices']
    registry.topology = fake_topology
    registry.usb(refresh=True)
    frame = {'data': b''}
    class Cameras:
        cameras = {}
        def refresh(self): pass
        def live(self):
            return registry.camera_target('front_left', '/dev/video0') == '/dev/video0'
        def list_cameras(self):
            return [{'id': 0, 'device': '/dev/video0', 'openedPath': '/dev/video0' if self.live() else None,
                     'frameAgeS': .1 if self.live() else None, 'frameSource': 'capture' if self.live() else ''}]
        def get_frame(self, cam_id): return frame['data'] if cam_id == 0 and self.live() else None
        def current_settings(self): return {}
    previous = CameraManager._instance
    CameraManager._instance = Cameras()

    @kiosk.post('/__test/frame')
    def synthetic_frame():
        frame['data'] = request.get_data()
        return jsonify(ok=True)

    @kiosk.post('/__test/swap-camera')
    def swap():
        fake_topology.devices[2]['serial'] = 'OTHER-CAMERA'
        registry.usb(refresh=True)
        return jsonify(ok=True)

    @kiosk.post('/__test/restore-camera')
    def restore():
        fake_topology.devices[2]['serial'] = 'CAM-123'
        registry.usb(refresh=True)
        return jsonify(ok=True)

    class Quiet(WSGIRequestHandler):
        def log_request(self, *args, **kwargs): pass
    server = make_server('127.0.0.1', 0, kiosk, threaded=True, request_handler=Quiet)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    web_server = web_thread = None
    try:
        board = f'http://127.0.0.1:{server.server_port}'
        origin = board
        if remote:
            monkeypatch.setenv('RC_SLAM_PEER_URL', board + '/api/kiosk_state')
            web_server = make_server('127.0.0.1', 0, app, threaded=True, request_handler=Quiet)
            web_thread = threading.Thread(target=web_server.serve_forever, daemon=True)
            web_thread.start()
            origin = f'http://127.0.0.1:{web_server.server_port}'
        result = subprocess.run(['node', str(Path(__file__).parent / 'js/devices_browser.mjs'),
                                 origin, board, 'remote' if remote else 'onboard'],
                                capture_output=True, text=True, timeout=90)
        assert result.returncode == 0, result.stdout + result.stderr
        assert 'camera confirmation, replacement and reset passed' in result.stdout
    finally:
        if web_server is not None:
            web_server.shutdown()
            web_server.server_close()
        if web_thread is not None:
            web_thread.join(timeout=3)
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)
        CameraManager._instance = previous
