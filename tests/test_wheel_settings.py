"""Manual USB routing, persistence and UI without wheel hardware."""
import queue
import threading
from types import SimpleNamespace

import pytest

from robot_control import slam, wheel_settings


@pytest.fixture
def settings_file(tmp_path, monkeypatch):
    target = tmp_path / '.env'
    monkeypatch.setattr(wheel_settings, 'env_file_path', lambda: target)
    monkeypatch.setattr(wheel_settings, 'reserved_ports', lambda: set())
    for key in wheel_settings.ENV_KEYS.values():
        monkeypatch.delenv(key, raising=False)
    return target


def test_save_roundtrip_without_live_rerouting(settings_file):
    settings_file.write_text('# keep\nRC_SECRET_KEY=secret\nRC_WHEEL_FR_PORT=old\nRC_WHEEL_FR_PORT=duplicate\n')
    result = wheel_settings.save_settings({'FR': '/dev/ttyUSB2'})
    assert result['restartRequired']
    text = settings_file.read_text()
    assert '# keep\nRC_SECRET_KEY=secret' in text
    assert text.count('RC_WHEEL_FR_PORT=') == 1
    assert wheel_settings.current_settings()['FR'] == ''
    assert wheel_settings.settings_payload()['settings']['FR'] == '/dev/ttyUSB2'
    wheel_settings.save_settings({})
    assert not any(wheel_settings.settings_payload()['settings'].values())


@pytest.mark.parametrize('payload', [None, [], {'foo': ''}, {'FL': 5},
    {'FL': '/tmp/file'}, {'FL': '/dev/ttyUSB0\nBAD=1'},
    {'FL': '/dev/ttyUSB0', 'FR': '/dev/ttyUSB0'}])
def test_invalid_settings_do_not_write(settings_file, payload):
    with pytest.raises(ValueError):
        wheel_settings.save_settings(payload)
    assert not settings_file.exists()


def test_aliases_and_reserved_ports_are_rejected(settings_file, monkeypatch, tmp_path):
    monkeypatch.setattr(wheel_settings.os.path, 'realpath', lambda p: '/dev/ttyUSB0')
    with pytest.raises(ValueError, match='двум'):
        wheel_settings.save_settings({'FL': '/dev/ttyUSB0', 'FR': '/dev/serial/by-path/hub:1.0'})
    monkeypatch.setattr(wheel_settings, 'reserved_ports', lambda: {'/dev/ttyUSB0'})
    with pytest.raises(ValueError, match='занят'):
        wheel_settings.save_settings({'FR': '/dev/serial/by-path/hub:1.0'})


def test_kiosk_api_and_assets(kiosk_client, settings_file):
    assert 'wheel-port-settings' in kiosk_client.get('/kiosk').text
    assert kiosk_client.get('/wheel_settings.js').status_code == 200
    assert kiosk_client.get('/api/wheels/settings').json['canEdit']
    assert kiosk_client.post('/api/wheels/settings', json={}).status_code == 400
    response = kiosk_client.post('/api/wheels/settings', json={'FL': '/dev/ttyACM0'},
                                 headers={'X-Robot-Control': 'wheel-settings'})
    assert response.status_code == 200
    assert response.json['restartRequired']
    assert kiosk_client.get('/api/wheels/settings').json['settings']['FL'] == '/dev/ttyACM0'


def source_without_threads():
    backend = slam.load_backend()
    src = object.__new__(backend.MasterSource)
    src.module_ports = {'FR': '/dev/right', 'RL': '/dev/back'}
    src.lock = threading.Lock()
    src.modules, src.frames, src.links = {}, {}, {}
    src.port = None
    src.serial_ready = threading.Event()
    src.commands = queue.Queue()
    src.messages = []
    src.on_status = src.on_message = None
    src._last_status = None
    src._conflict_noted = {}
    return src


def test_same_firmware_name_can_be_manually_assigned_and_commanded():
    src = source_without_threads()
    for name, path in src.module_ports.items():
        link = SimpleNamespace(path=path, module=None, commands=queue.Queue())
        src.links[path] = link
        src._link_line(link, '@TLM mod=FL deg=12 tgt=0 cal=1')
        assert link.module == name
        assert src.frames[name][0]['mod'] == name
    assert 'FL' not in src.frames
    src.commands.put(b'w FR 25\n')
    src._fanout_commands()
    assert src.links['/dev/right'].commands.get_nowait() == b'w 25\n'
    assert src.links['/dev/back'].commands.empty()
    src.commands.put(b'w RL calib\n')
    src._fanout_commands()
    assert src.links['/dev/back'].commands.get_nowait() == b'w calib\n'
    src.commands.put(b'b\n')
    src._fanout_commands()
    assert all(link.commands.get_nowait() == b'b\n' for link in src.links.values())


def test_unidentified_ports_still_never_receive_commands():
    src = source_without_threads()
    link = SimpleNamespace(path='/dev/right', module=None, commands=queue.Queue())
    src.links[link.path] = link
    src._link_line(link, 'garbage')
    src.commands.put(b'w FR 20\n')
    src._fanout_commands()
    assert link.module is None and link.commands.empty()
    assert not src.frames


def test_auto_port_cannot_take_manually_reserved_role():
    src = source_without_threads()
    link = SimpleNamespace(path='/dev/other', module=None)
    src._link_line(link, '@TLM mod=FR deg=0')
    assert link.module is None
    assert not src.frames
    src._link_line(link, '@TLM mod=FL deg=0')
    assert link.module is None
    assert not src.frames


def test_manual_ports_excluded_from_lidar_but_active_lidar_keeps_ownership(monkeypatch):
    from robot_control import lidar
    monkeypatch.setenv('RC_WHEEL_FR_PORT', '/dev/ttyUSB9')
    monkeypatch.setattr(lidar, 'module_port_confirmed', lambda p: False)
    monkeypatch.setattr(lidar, 'lidar_active_port', lambda: None)
    monkeypatch.setattr(lidar, 'lidar_search_priority', lambda: 60)
    assert '/dev/ttyUSB9' in lidar.excluded_ports()
    assert not lidar.lidar_holds_port('/dev/ttyUSB9')
    assert lidar.lidar_holds_port('/dev/ttyUSB8')
    monkeypatch.setattr(lidar, 'lidar_active_port', lambda: '/dev/ttyUSB9')
    assert lidar.lidar_holds_port('/dev/ttyUSB9')


def test_master_source_receives_assignments_and_scans_them_first(monkeypatch):
    from robot_control import master_port, battery
    monkeypatch.setenv('RC_WHEEL_FR_PORT', '/dev/serial/by-path/hub:1.2')
    monkeypatch.setattr(master_port, 'reserved_ports', lambda: set())
    monkeypatch.setattr(battery, 'reserved_ports', lambda: set())
    backend = slam.load_backend()
    captured = {}

    def source(**kwargs):
        captured.update(**kwargs)
        return captured

    monkeypatch.setattr(backend, 'MasterSource', source)
    slam.make_source('master')
    assert captured['module_ports']['FR'] == '/dev/serial/by-path/hub:1.2'
    assert 'port_provider' not in captured
    assert 'ports' not in captured


def test_fragmented_telemetry_is_only_parsed_after_newline():
    src = source_without_threads()
    link = slam.load_backend()._WheelLink('/dev/right')
    assert not src._link_bytes(link, b'@TLM mod=FL de')
    assert not src._link_bytes(link, b'g=12 tgt=0 cal=1 drive=0\r')
    assert not src.frames and link.module is None
    assert src._link_bytes(link, b'\n')
    assert src.frames['FR'][0]['deg'] == 12
    assert src.frames['FR'][0]['cal']
    assert link.module == 'FR'


def test_noisy_stream_is_bounded_and_resynchronizes():
    src = source_without_threads()
    link = slam.load_backend()._WheelLink('/dev/right')
    assert not src._link_bytes(link, b'x' * 10000 + b'@TLM mod=FL deg=9\n')
    assert not src.frames
    assert len(link.rx) == 0
    assert src._link_bytes(link, b'@TLM mod=FL deg=1\n@TLM mod=FL deg=2\n')
    assert src.frames['FR'][0]['deg'] == 2


def test_identity_lines_cannot_enable_an_assigned_port():
    src = source_without_threads()
    link = slam.load_backend()._WheelLink('/dev/right')
    assert not src._link_bytes(link, b'@ID mod=FR fw=4\n')
    assert link.module is None and not src.frames
    assert src._link_bytes(link, b'@TLM deg=7 tgt=0 cal=1\n')
    assert link.module == 'FR'
    assert src.frames['FR'][0]['deg'] == 7


def test_old_unknown_firmware_name_cannot_override_manual_assignment():
    src = source_without_threads()
    link = slam.load_backend()._WheelLink('/dev/right')
    assert src._link_bytes(link, b'@TLM mod=UNKNOWN deg=7 tgt=0 cal=1\n')
    assert link.module == 'FR'


def test_port_list_uses_real_names_and_never_opens_devices(settings_file, monkeypatch):
    monkeypatch.setattr(wheel_settings.glob, 'glob', lambda pattern:
                        ['/dev/ttyUSB0'] if 'USB' in pattern else ['/dev/ttyACM2'])
    monkeypatch.setattr(wheel_settings.os.path, 'realpath',
                        lambda p: '/dev/ttyUSB0' if '/serial/' in p else p)
    settings_file.write_text('RC_WHEEL_FL_PORT=/dev/serial/by-path/hub:1.0\n')
    data = wheel_settings.settings_payload()
    assert [p['path'] for p in data['ports']] == ['/dev/ttyACM2', '/dev/ttyUSB0']
    assert data['settings']['FL'] == '/dev/ttyUSB0'


def test_wheel_ui_has_no_auto_option(kiosk_client):
    script = kiosk_client.get('/wheel_settings.js').text
    assert 'Авто · по имени платы' not in script
    assert "new Option('Не подключено · порт не выбран', '')" in script
    assert 'new Option(port.path, port.path)' in script


def test_duplicate_assignments_fail_before_opening_serial(monkeypatch):
    backend = slam.load_backend()
    with pytest.raises(ValueError, match='двум'):
        backend.MasterSource(module_ports={'FL': '/dev/ttyUSB0', 'FR': '/dev/ttyUSB0'})
