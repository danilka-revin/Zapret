"""Passive onboard inventory, cached reads and authenticated web forwarding."""
import io
import json
import shutil
import subprocess
import threading
from pathlib import Path
from types import SimpleNamespace
from urllib.error import URLError
from urllib.parse import urlsplit

import pytest

from conftest import login
from robot_control import connections, map_peer
from robot_control.connections import ConnectionsMonitor, Inventory


@pytest.fixture
def inventory(tmp_path, monkeypatch):
    root, dev = tmp_path / 'sys', tmp_path / 'dev'
    dev.mkdir()
    for directory in ('bus/usb/devices', 'class/tty', 'class/net', 'class/bluetooth',
                      'class/video4linux', 'class/input', 'class/block', 'class/drm'):
        (root / directory).mkdir(parents=True)
    calls = []
    outputs = {
        ('ip', '-j', 'address', 'show'): json.dumps([
            {'ifname': 'wlan0', 'addr_info': [{'local': '192.168.4.1', 'prefixlen': 24},
                                          {'local': 'fe80::1', 'prefixlen': 64}]}]),
        ('ip', '-j', 'neigh', 'show'): json.dumps([
            {'dst': '192.168.4.2', 'dev': 'wlan0', 'lladdr': 'aa:bb:cc:dd:ee:ff', 'state': ['STALE']}]),
        ('nmcli', '-t', '-f', 'DEVICE,TYPE,STATE,CONNECTION', 'device', 'status'):
            'wlan0:wifi:connected:Robot\\: Wi-Fi\neth0:ethernet:disconnected:--\n',
        ('bluetoothctl', 'list'): 'Controller 11:22:33:44:55:66 Robot BT [default]\n',
        ('bluetoothctl', 'devices'): 'Device AA:BB:CC:DD:EE:01 Keyboard\nDevice AA:BB:CC:DD:EE:02 Mouse\n',
        ('bluetoothctl', 'devices', 'Connected'): 'Device AA:BB:CC:DD:EE:01 Keyboard\n',
        ('bluetoothctl', 'devices', 'Paired'): 'Device AA:BB:CC:DD:EE:02 Mouse\n',
    }

    def runner(args, **kwargs):
        calls.append((args, kwargs))
        assert tuple(args) in outputs, f'Unexpected command: {args}'
        result = outputs[tuple(args)]
        if isinstance(result, Exception):
            raise result
        return SimpleNamespace(returncode=0, stdout=result)

    inv = Inventory(root, dev, runner)
    inv.calls, inv.outputs = calls, outputs
    for key in ('RC_BMS_PORT', 'RC_LIDAR_PORT', 'RC_WHEEL_FL_PORT', 'RC_WHEEL_FR_PORT',
                'RC_WHEEL_RL_PORT', 'RC_WHEEL_RR_PORT'):
        monkeypatch.delenv(key, raising=False)
    return inv


def attributes(path, **values):
    path.mkdir(parents=True, exist_ok=True)
    for key, value in values.items():
        target = path / key
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(str(value))


def test_usb_serial_and_explicit_roles_without_opening_devices(inventory, monkeypatch):
    import serial
    monkeypatch.setattr(serial, 'Serial', lambda *a, **kw: pytest.fail('must never open UART'))
    attributes(inventory.sys / 'bus/usb/devices/1-2', idVendor='1a86', idProduct='7523',
               product='<script>FL board</script>', manufacturer='CH340', serial='123', speed='12', authorized='1')
    attributes(inventory.sys / 'bus/usb/devices/1-2:1.0', bInterfaceClass='ff')
    attributes(inventory.sys / 'class/tty/ttyUSB0')
    attributes(inventory.sys / 'class/tty/ttyUSB1')
    (inventory.dev / 'ttyUSB0').touch()
    (inventory.dev / 'ttyUSB1').touch()
    missing = str(inventory.dev / 'ttyUSB3')
    monkeypatch.setenv('RC_WHEEL_FR_PORT', str(inventory.dev / 'ttyUSB0'))
    monkeypatch.setenv('RC_BMS_PORT', missing)
    data = inventory.collect()['sections']
    assert len(data['usb']['items']) == 1
    assert data['usb']['items'][0]['vidPid'] == '1a86:7523'
    ports = {item['path']: item for item in data['serial']['items']}
    assert ports[str(inventory.dev / 'ttyUSB0')]['roles'] == ['Колесо FR']
    assert ports[str(inventory.dev / 'ttyUSB1')]['roles'] == []
    assert ports[missing]['roles'] == ['BMS'] and not ports[missing]['present']


def test_network_bluetooth_peers_and_peripherals(inventory):
    attributes(inventory.sys / 'class/net/wlan0', operstate='up', carrier='1', address='11:22:33:44:55:66')
    attributes(inventory.sys / 'class/net/wlan0/wireless')
    attributes(inventory.sys / 'class/net/eth0', type='1', operstate='down', carrier='0')
    attributes(inventory.sys / 'class/net/eth0/device')
    attributes(inventory.sys / 'class/video4linux/video0', name='USB camera')
    attributes(inventory.sys / 'class/input/event0', **{'device/name': 'Keyboard'})
    attributes(inventory.sys / 'class/block/sda', **{'device/model': 'SSD'})
    attributes(inventory.sys / 'class/drm/card0-HDMI-A-1', status='connected')
    attributes(inventory.sys / 'class/drm/card0-DP-1', status='disconnected')
    sections = inventory.collect()['sections']
    net = {item['name']: item for item in sections['network']['items']}
    assert net['wlan0']['kind'] == 'Wi-Fi'
    assert net['wlan0']['addresses'] == ['192.168.4.1/24', 'fe80::1/64']
    assert net['wlan0']['profile'] == 'Robot: Wi-Fi'
    assert net['eth0']['kind'] == 'Ethernet'
    assert net['eth0']['state'] == 'down'
    bt = sections['bluetooth']['items']
    assert bt['adapters'] == [{'name': 'Robot BT', 'address': '11:22:33:44:55:66'}]
    keyboard, mouse = bt['devices']
    assert keyboard['connected'] and not keyboard['paired']
    assert mouse['paired'] and not mouse['connected']
    assert len(sections['peripherals']['items']) == 4
    assert sections['peers']['items'][0]['state'] == ['STALE']
    for args, kwargs in inventory.calls:
        assert not kwargs.get('shell')
        assert kwargs['timeout'] <= 0.8
        assert kwargs['env']['LC_ALL'] == 'C'
        assert not {'scan', 'connect', 'pair', '--show-secrets'} & set(args)


def test_missing_tools_permission_errors_and_old_bluez_are_explicit(inventory):
    inventory.outputs[('nmcli', '-t', '-f', 'DEVICE,TYPE,STATE,CONNECTION', 'device', 'status')] = PermissionError()
    inventory.outputs[('bluetoothctl', 'devices', 'Connected')] = 'Invalid command in menu main: devices\n'
    inventory.outputs[('ip', '-j', 'neigh', 'show')] = 'not JSON'
    sections = inventory.collect()['sections']
    assert sections['network']['status'] == 'partial'
    assert sections['bluetooth']['status'] == 'partial'
    assert sections['bluetooth']['items']['devices'][0]['connected'] is None
    assert sections['peers']['status'] == 'unavailable'
    assert sections['usb']['status'] == 'ok'
    shutil.rmtree(inventory.sys / 'bus/usb/devices')
    assert inventory.collect()['sections']['usb']['status'] == 'unavailable'


def test_tools_timeout_and_total_collection_budget(inventory, monkeypatch):
    inventory.outputs[('bluetoothctl', 'list')] = subprocess.TimeoutExpired('bluetoothctl', 0.8)
    assert inventory.collect()['sections']['bluetooth']['status'] == 'partial'
    inventory._deadline = 1
    monkeypatch.setattr(connections.time, 'monotonic', lambda: 2)
    count = len(inventory.calls)
    with pytest.raises(RuntimeError, match='Лимит времени'):
        inventory.command(['ip', '-j', 'address', 'show'])
    assert len(inventory.calls) == count


def test_cache_single_background_reader_nonblocking_and_copy(monkeypatch):
    entered, release = threading.Event(), threading.Event()
    workers, calls = [], []
    original_thread = threading.Thread

    def thread(*args, **kwargs):
        worker = original_thread(*args, **kwargs)
        workers.append(worker)
        return worker

    def collect():
        calls.append(1)
        entered.set()
        assert release.wait(3)
        return {'hostname': 'robot', 'sections': {}}

    monkeypatch.setattr(connections.threading, 'Thread', thread)
    monitor = ConnectionsMonitor(collect)
    first = monitor.snapshot()
    assert first['refreshing'] and first['data'] is None
    assert entered.wait(1)
    for _ in range(20):
        assert monitor.snapshot()['refreshing']
    assert len(workers) == 1
    release.set()
    workers[0].join(2)
    snapshot = monitor.snapshot()
    assert snapshot['data']['hostname'] == 'robot'
    snapshot['data']['hostname'] = 'not the robot'
    assert monitor.snapshot()['data']['hostname'] == 'robot'
    assert calls == [1]
    monitor.collector = lambda: (_ for _ in ()).throw(OSError('no access'))
    monitor._refresh()
    assert monitor.snapshot()['error']
    assert monitor.snapshot()['data']['hostname'] == 'robot'


def test_web_auth_proxy_peer_selection_and_read_only(client, operator, kiosk_client, monkeypatch):
    for path in ('/connections', '/api/connections'):
        assert client.get(path).status_code in (302, 401)
    login(client)
    assert 'id="nav-connections"' in client.get('/').text
    assert 'id="console-connections-link"' in client.get('/console').text
    page = client.get('/connections')
    assert page.status_code == 200
    assert page.text.index('slam_auth.js') < page.text.index('connections.js')
    assert client.get('/connections.js').status_code == 200
    assert client.get('/connections.css').status_code == 200
    assert client.post('/api/connections').status_code == 405
    snapshot = {'ok': True, 'data': {'hostname': 'onboard'}, 'refreshing': False, 'error': None}
    monkeypatch.setattr(kiosk_client.application.extensions['rc_connections'], 'snapshot', lambda: snapshot)
    calls = []

    def forward(req, timeout):
        calls.append(req.full_url)
        assert not req.data
        assert req.get_header('Cookie') is None
        response = kiosk_client.get(urlsplit(req.full_url).path)
        assert response.headers['Cache-Control'] == 'no-store'
        reply = io.BytesIO(response.data)
        reply.code = response.status_code
        return reply

    monkeypatch.setenv('RC_SLAM_PEER_URL', 'http://onboard.example:8090/api/kiosk_state')
    monkeypatch.setattr(map_peer, 'urlopen', forward)
    response = client.get('/api/connections')
    assert response.status_code == 200
    assert response.json == snapshot
    assert response.headers['Cache-Control'] == 'no-store'
    assert calls == ['http://onboard.example:8090/api/connections']
    monkeypatch.setattr(map_peer, 'urlopen', lambda *a, **kw: (_ for _ in ()).throw(URLError('offline')))
    assert client.get('/api/connections').status_code == 503


def test_connections_javascript():
    node = shutil.which('node')
    if not node:
        pytest.skip('node is not installed')
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run([node, 'tests/js/connections.js'], cwd=root,
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
