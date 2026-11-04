"""Battery receipts, passive USB selection and real PTY print transport."""
from __future__ import annotations

import copy
import json
import os
import re
import select
import time
from pathlib import Path

import pytest

from robot_control.battery import parse_snapshot
from robot_control.battery_receipt import encode_receipt, render_receipt
from robot_control import printer as printer_mod


def sample(missing=False):
    basic = bytearray(27)
    basic[0:2] = (5120).to_bytes(2, 'big')
    basic[2:4] = (-1250).to_bytes(2, 'big', signed=True)
    basic[4:6] = (2500).to_bytes(2, 'big')
    basic[6:8] = (5000).to_bytes(2, 'big')
    basic[8:10] = (123).to_bytes(2, 'big')
    basic[18:23] = bytes([0x21, 50, 3, 12, 2])
    basic[23:25] = (2981).to_bytes(2, 'big')
    basic[25:27] = (3201).to_bytes(2, 'big')
    cells = b''.join((3200 + i).to_bytes(2, 'big') for i in range(11 if missing else 12))
    data = parse_snapshot(basic, cells)
    data.update(deviceName='JBD 12S', errorCounters=None, thresholds=[])
    return {'connected': True, 'complete': not missing, 'reason': 'ОК', 'ageS': 0.0, 'data': data}


class Battery:
    def __init__(self):
        self.port = '/dev/bms-distinct'
        self.value = sample()

    def snapshot(self):
        return copy.deepcopy(self.value)


class Registry:
    def __init__(self, path):
        self.port = path
        self.device = {'topology': '1-7', 'vidPid': '1a86:7523', 'serial': 'TEST-1',
                       'product': 'USB-UART', 'manufacturer': 'maker', 'connection': 'порт',
                       'nodes': [{'path': path, 'kind': 'serial', 'interface': '1.0', 'index': '', 'aliases': []}]}
        self.hints = []

    def usb(self, *, refresh=False):
        return {'available': True, 'devices': [copy.deepcopy(self.device)]}

    def set_hint(self, role, port):
        self.hints.append((role, port))


class WheelOwner:
    auto_discovery = True
    allowed = True
    def manual_printer_port_allowed(self, port):
        return self.allowed


@pytest.fixture
def printer_fixture(tmp_path, monkeypatch):
    import pty
    from robot_control import lidar
    monkeypatch.setenv('RC_PRINTER_CONFIG', str(tmp_path / 'printer.json'))
    monkeypatch.delenv('RC_BMS_PORT', raising=False)
    monkeypatch.delenv('RC_LIDAR_PORT', raising=False)
    monkeypatch.delenv('RC_LIDAR_EXCLUDE', raising=False)
    monkeypatch.setattr(lidar, 'DATA_DIR', tmp_path / 'locks')
    monkeypatch.setattr(printer_mod, 'SERIAL_RE', re.compile(r'^/dev/pts/\d+$'))
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    os.close(slave)
    registry, battery, source = Registry(path), Battery(), WheelOwner()
    service = printer_mod.PrinterService(registry, battery, lambda: source, path=tmp_path / 'printer.json')
    yield service, master, path, registry, battery, source
    service.cancel()
    deadline = time.monotonic() + 4
    while service._job_lock.locked() and time.monotonic() < deadline:
        time.sleep(.01)
    os.close(master)


def configuration(path, **changes):
    value = {'port': path, 'baudRate': 9600, 'framing': '8N1', 'width': 32,
             'profile': 'plain', 'language': 'en', 'charset': 'cp866',
             'lineDelayMs': 0, 'confirmed': True}
    value.update(changes)
    return value


def wait_job(service, timeout=8):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        job = service.status(include_ports=False)['job']
        if job and job['state'] in ('sent', 'error', 'cancelled') and not service._job_lock.locked():
            return job
        time.sleep(.01)
    pytest.fail('printer job did not finish')


def read_available(fd):
    output = bytearray()
    while select.select([fd], [], [], .1)[0]:
        try:
            output.extend(os.read(fd, 8192))
        except OSError as exc:
            if exc.errno == 5:  # PTY slave closed after the job
                break
            raise
    return bytes(output)


@pytest.mark.parametrize('language', ['en', 'ru'])
@pytest.mark.parametrize('width', [24, 32, 48])
def test_report_levels_are_bounded_and_include_real_measurements(language, width):
    snap = sample()
    reports = {level: render_receipt(snap, level, language=language, width=width)
               for level in ('short', 'medium', 'full')}
    assert len(reports['short']) < len(reports['medium']) < len(reports['full'])
    assert all(len(line) <= width for lines in reports.values() for line in lines)
    short = '\n'.join(reports['short'])
    full = '\n'.join(reports['full'])
    assert '51.20' in short and '-12.50' in short and '50 %' in short
    assert ('нет данных' if language == 'ru' else 'no data') in full
    assert '#12' in full and '3211' in full
    if language == 'en':
        assert all(ord(c) < 128 for line in reports['full'] for c in line)


def test_incomplete_receipt_never_fabricates_missing_cells_or_faults():
    snap = sample(missing=True)
    snap['data']['faults'] = None
    text = '\n'.join(render_receipt(snap, 'full', language='ru'))
    assert 'НЕ ВСЕ БАНКИ' in text and '11 / 12' in text
    assert '#12' in text and 'нет данных' in text
    assert 'Активные защиты' in text
    with pytest.raises(ValueError):
        render_receipt({'connected': False, 'data': snap['data']})
    with pytest.raises(ValueError):
        render_receipt(snap, 'raw')


def test_printer_escpos_encoding_and_escape_injection():
    snap = sample()
    snap['data']['deviceName'] = 'X\x1b@\x1dp\x00BAD'
    lines = render_receipt(snap, 'full', language='ru')
    assert all('\x1b' not in line and '\x1d' not in line and '\x00' not in line for line in lines)
    for charset, page in [('cp866', 17), ('cp1251', 46), ('utf8', None)]:
        init, chunks = encode_receipt(lines, profile='escpos', language='ru', charset=charset)
        assert init.startswith(b'\x1b@')
        assert init == b'\x1b@' + (bytes([27, 116, page]) if page is not None else b'')
        assert not any(b'\x1b' in chunk or b'\x1d' in chunk for chunk in chunks)
        assert chunks[-4:] == [b'\r\n'] * 4
    init, chunks = encode_receipt(lines, profile='plain', language='ru')
    assert init == b'' and chunks[0].startswith(b' '), 'paper alignment matches preview'


def test_print_uses_real_pty_and_one_snapshot_without_replay(printer_fixture):
    service, master, path, registry, battery, source = printer_fixture
    service.configure(configuration(path, profile='escpos', language='ru'))
    assert registry.hints == [('printer', path)]
    status = service.status(include_ports=False)
    assert status['ready'] and status['confidence'] == 'serial'
    first = service.start({'requestId': 'print-123456789012345', 'level': 'short'})
    assert first['ok']
    job = wait_job(service)
    assert job['state'] == 'sent' and job['sentLines'] == job['totalLines']
    receipt = read_available(master)
    assert receipt.startswith(b'\x1b@\x1bt\x11')
    assert 'Напряжение'.encode('cp866') in receipt and b'51.20' in receipt
    assert b'\x1dV' not in receipt and b'\x1b7' not in receipt
    assert service.start({'requestId': 'print-123456789012345', 'level': 'short'})['replayed']
    assert not read_available(master), 'replayed request must never print a second copy'
    assert 'проверить на принтере' in job['message']


def test_port_rechecked_no_print_on_stale_battery_or_wrong_owner(printer_fixture):
    service, master, path, registry, battery, source = printer_fixture
    source.allowed = False
    with pytest.raises(printer_mod.PrinterError, match='проверяется'):
        service.configure(configuration(path))
    assert not read_available(master)
    source.allowed = True
    service.configure(configuration(path))
    battery.value['data']['updatedAt'] -= 20
    with pytest.raises(printer_mod.PrinterError, match='свежих'):
        service.start({'requestId': 'request-1234567890123', 'level': 'short'})
    battery.value['data']['updatedAt'] = time.time()
    battery.port = path
    assert not service.status(include_ports=False)['ready']
    with pytest.raises(printer_mod.PrinterError, match='BMS'):
        service.start({'requestId': 'request-1234567890124', 'level': 'short'})
    assert not read_available(master)


def test_changed_adapter_or_restart_requires_confirmation(printer_fixture):
    service, master, path, registry, battery, source = printer_fixture
    service.configure(configuration(path))
    restarted = printer_mod.PrinterService(registry, battery, lambda: source, path=service.path)
    assert restarted.settings()['port'] == path and not restarted.status(include_ports=False)['ready']
    assert 'Подтвердите' in restarted.status(include_ports=False)['detail']
    registry.device['serial'] = 'A-DIFFERENT-DEVICE'
    assert not service.status(include_ports=False)['ready']
    assert not read_available(master)


def test_busy_cancel_and_no_parallel_or_invalid_serial_commands(printer_fixture):
    service, master, path, registry, battery, source = printer_fixture
    for patch in ({'confirmed': False}, {'port': '/etc/passwd'}, {'baudRate': 1234}, {'profile': 'drawer'}):
        with pytest.raises(printer_mod.PrinterError):
            service.configure(configuration(path, **patch))
    service.configure(configuration(path, lineDelayMs=150))
    service.start({'requestId': 'cancel-123456789012345', 'level': 'full'})
    with pytest.raises(printer_mod.PrinterError, match='занят'):
        service.start({'requestId': 'other-1234567890123456', 'level': 'short'})
    with pytest.raises(printer_mod.PrinterError, match='печати'):
        service.configure(configuration(path))
    time.sleep(.25)
    service.cancel()
    job = wait_job(service)
    assert job['state'] == 'cancelled' and job['sentLines'] < job['totalLines']
    assert b'\x1b' not in read_available(master), 'plain profile sends no control commands'


def test_config_is_reserved_from_bms_and_lidar(printer_fixture, monkeypatch):
    from robot_control import bms_setup, lidar
    service, master, path, registry, battery, source = printer_fixture
    service.configure(configuration(path))
    assert path in printer_mod.printer_reserved_ports()
    assert 'принтера' in bms_setup.probe_port(path)['error']
    with pytest.raises(ValueError, match='принтера'):
        bms_setup.apply_port(battery, path)
    assert path not in lidar.discover_ports(configured=path, autoprobe=False)


def test_api_rejects_unknown_body_and_header(kiosk_client):
    client = kiosk_client
    assert client.post('/api/printer/print', json={'requestId': 'test-123456789012345', 'level': 'short'}).status_code == 400
    assert client.post('/api/printer/print', json={'requestId': 'test-123456789012345', 'level': 'short', 'bytes': 'ESC'},
                       headers={'X-Robot-Control': 'printer'}).status_code == 400
    assert client.get('/api/printer/preview?level=short').status_code == 503
    assert client.get('/receipt.js').status_code == 200
    assert client.get('/receipt.css').status_code == 200


def test_master_passive_check_does_not_classify_printer_or_relax_motion():
    import threading
    from types import SimpleNamespace
    from robot_control.slam import load_backend
    cls = load_backend().MasterSource
    source = cls.__new__(cls)
    source.lock = threading.RLock()
    source._unknown_devices = {'/dev/ttyUSB9': (1, 2, 3, 4)}
    source._bms_results = {}
    source.links = {}
    source._spawning = set()
    source._device_fingerprint = lambda path: (1, 2, 3, 4)
    assert source.manual_printer_port_allowed('/dev/ttyUSB9')
    assert source._unknown_devices == {'/dev/ttyUSB9': (1, 2, 3, 4)}
    assert source._bms_results == {}, 'a weak printer hint must not classify the port'
    source._bms_results['/dev/ttyUSB9'] = ((1, 2, 3, 4), True)
    assert not source.manual_printer_port_allowed('/dev/ttyUSB9')
    source._bms_results['/dev/ttyUSB9'] = ((1, 2, 3, 4), False)
    assert not source.manual_printer_port_allowed('/dev/ttyUSB9')
    source._bms_results = {}
    source.links['wheel'] = SimpleNamespace(device='/dev/ttyUSB9')
    assert not source.manual_printer_port_allowed('/dev/ttyUSB9')
    source.links = {}
    source._device_fingerprint = lambda path: (8, 2, 3, 4)
    assert not source.manual_printer_port_allowed('/dev/ttyUSB9')


@pytest.mark.skipif(
    os.name != 'posix' or not __import__('shutil').which('node') or
    not os.environ.get('RC_TEST_CHROMIUM') or not os.environ.get('RC_TEST_PLAYWRIGHT_CORE'),
    reason='для browser -> UART проверки нужны Chromium и playwright-core',
)
@pytest.mark.parametrize('remote', [False, True], ids=['onboard', 'authenticated-web'])
def test_receipt_in_real_browser_through_simulated_uart(
        printer_fixture, kiosk_client, app, operator, monkeypatch, remote):
    """Real Chromium and real PTY: no claim about the physical printer or BMS."""
    import shutil
    import subprocess
    import threading
    from flask import jsonify
    from werkzeug.serving import make_server
    from robot_control import bms_setup

    _, master, path, registry, battery, owner = printer_fixture
    application = kiosk_client.application
    service = application.extensions['rc_printer']
    onboard_battery = application.extensions['rc_battery']
    service.registry = registry
    service.source_provider = lambda: owner
    onboard_battery.port = battery.port
    monkeypatch.setattr(onboard_battery, 'snapshot', lambda: sample())
    # A by-id alias represents a production USB-UART, while the underlying
    # node is a PTY. Only this test extends Python's port whitelist for PTYs.
    alias = '/dev/serial/by-id/simulated-printer'
    registry.device['nodes'][0]['aliases'] = [alias]
    monkeypatch.setattr(printer_mod, 'SERIAL_RE', re.compile(r'^(?:/dev/pts/\d+|/dev/serial/by-id/simulated-printer)$'))
    monkeypatch.setattr(bms_setup, 'describe_ports', lambda *args, **kwargs: [
        {'port': alias, 'device': path, 'role': 'free', 'note': 'Simulated UART'}])
    observed = bytearray()

    @application.get('/__test_printer_bytes')
    def printer_bytes():
        observed.extend(read_available(master))
        return jsonify(data=observed.decode('latin-1'))

    board = web = None
    threads = []
    try:
        board = make_server('127.0.0.1', 0, application, threaded=True)
        thread = threading.Thread(target=board.serve_forever, daemon=True)
        thread.start()
        threads.append(thread)
        kiosk = f'http://127.0.0.1:{board.server_port}'
        origin = kiosk
        if remote:
            monkeypatch.setenv('RC_SLAM_PEER_URL', kiosk + '/api/kiosk_state')
            web = make_server('127.0.0.1', 0, app, threaded=True)
            thread = threading.Thread(target=web.serve_forever, daemon=True)
            thread.start()
            threads.append(thread)
            origin = f'http://127.0.0.1:{web.server_port}'
        result = subprocess.run([
            shutil.which('node'), str(Path(__file__).parent / 'js/receipt_browser.mjs'),
            origin, kiosk, 'remote' if remote else 'onboard', alias,
        ], capture_output=True, text=True, timeout=70)
        assert result.returncode == 0, result.stdout + '\n' + result.stderr
        assert 'Browser -> onboard USB-UART receipt: OK' in result.stdout
    finally:
        service.cancel()
        for server in (web, board):
            if server is not None:
                server.shutdown()
                server.server_close()
        for thread in threads:
            thread.join(timeout=3)


def test_replayed_id_between_lookup_and_lock_never_leaks_job_lock(printer_fixture, monkeypatch):
    service, master, path, _, _, _ = printer_fixture
    service.configure(configuration(path))
    job_id = 'racing-1234567890123456789'
    original = service.status

    def raced_status(*args, **kwargs):
        result = original(*args, **kwargs)
        # A previous identical request completed after start()'s first lookup.
        service._jobs[job_id] = {'id': job_id, 'state': 'sent'}
        return result

    monkeypatch.setattr(service, 'status', raced_status)
    replay = service.start({'requestId': job_id, 'level': 'short'})
    assert replay['replayed'] and replay['job']['state'] == 'sent'
    assert not service._job_lock.locked()
    assert not read_available(master)


def test_wait_for_uart_queue_can_be_cancelled_and_never_blindly_succeeds(printer_fixture):
    import threading
    service, _, path, _, _, source = printer_fixture
    service.configure(configuration(path))
    fingerprint = printer_mod._fingerprint(path)
    settings = service.settings()

    class Stalled:
        out_waiting = 12

    errors = []
    def wait():
        try:
            service._wait_for_output(Stalled(), path, fingerprint, source, settings, 50)
        except printer_mod.PrinterError as exc:
            errors.append(str(exc))

    thread = threading.Thread(target=wait)
    thread.start()
    service._cancel.set()
    thread.join(timeout=1)
    assert not thread.is_alive() and errors and 'Отменено' in errors[0]
    service._cancel.clear()

    class Broken:
        @property
        def out_waiting(self):
            raise OSError('disconnected')

    with pytest.raises(printer_mod.PrinterError, match='результат печати неизвестен'):
        service._wait_for_output(Broken(), path, fingerprint, source, settings, 50)


def test_real_master_passive_discovery_and_printer_share_the_uart_lock(printer_fixture, monkeypatch):
    """A real auto-discovery owner rechecking a silent PTY cannot steal a print job."""
    from robot_control import lidar, slam
    backend = slam.load_backend()
    service, master, path, _, battery, _ = printer_fixture
    claims = []
    original_claim = lidar.try_lock_port

    def claim(candidate):
        token = original_claim(candidate)
        claims.append(token)
        return token

    # The production probe is 5 s; shorten only this simulated test.
    monkeypatch.setattr(backend.MasterSource, 'PROBE_S', .2)
    source = backend.MasterSource(auto_discovery=True, port_provider=lambda: [path],
                                  blocked_ports=lambda: (), claim=claim, release=lidar.unlock_port)
    service.source_provider = lambda: source
    try:
        until = time.monotonic() + 5
        while time.monotonic() < until and not source.manual_printer_port_allowed(path):
            time.sleep(.02)
        assert source.manual_printer_port_allowed(path), source.read()
        assert not source.serial_ready.is_set(), 'unknown printer must not authorize motion'
        assert not read_available(master), 'passive discovery must not write to the printer'
        service.configure(configuration(path, lineDelayMs=100))
        battery.value['data']['updatedAt'] = time.time()
        service.start({'requestId': 'discovery-12345678901234', 'level': 'short'})
        until = time.monotonic() + 3
        while time.monotonic() < until and (service._job or {}).get('state') != 'printing':
            time.sleep(.01)
        assert (service._job or {}).get('state') == 'printing'
        with source.lock:
            source._unknown_checked_at[os.path.realpath(path)] = time.monotonic() - 11
            source._cooldown[path] = 0
        source._dispatch_wakeup.set()
        job = wait_job(service, timeout=8)
        assert job['state'] == 'sent', job
        assert any(token is None for token in claims), 'discovery must be denied the held printer lock'
        receipt = read_available(master)
        assert b'BATTERY REPORT' in receipt and b'51.20' in receipt
        assert not source.serial_ready.is_set()
        assert not source._bms_results, 'USB-UART is still not a positively identified peripheral'
    finally:
        source._stop = True
        source._dispatch_wakeup.set()
        until = time.monotonic() + 3
        while time.monotonic() < until:
            with source.lock:
                if not source.links and not source._spawning:
                    break
            time.sleep(.02)


def test_changed_fingerprint_mid_job_stops_before_further_lines(printer_fixture, monkeypatch):
    service, master, path, _, battery, _ = printer_fixture
    service.configure(configuration(path, lineDelayMs=100))
    battery.value['data']['updatedAt'] = time.time()
    service.start({'requestId': 'changed-1234567890123456', 'level': 'full'})
    until = time.monotonic() + 3
    while time.monotonic() < until and (service._job or {}).get('sentLines', 0) < 1:
        time.sleep(.01)
    assert service._job['sentLines'] >= 1
    monkeypatch.setattr(printer_mod, '_fingerprint', lambda selected: (999, 999, 999, 999))
    job = wait_job(service)
    assert job['state'] == 'error' and 'изменился' in job['message']
    assert job['sentLines'] < job['totalLines'] and 'Часть чека' in job['message']
    assert read_available(master), 'only the already-sent part may have reached the UART'


def test_partial_write_failure_warns_even_before_first_complete_line(printer_fixture, monkeypatch):
    service, master, path, _, _, _ = printer_fixture
    service.configure(configuration(path))
    open_serial = service._open

    class PartiallyFailing:
        def __init__(self, device):
            self.device, self.calls = device, 0
        def fileno(self):
            return self.device.fileno()
        def write(self, data):
            self.calls += 1
            if self.calls == 1:
                return self.device.write(data[:2])
            raise TimeoutError('serial write timeout')
        def close(self):
            self.device.close()

    monkeypatch.setattr(service, '_open', lambda port, settings: PartiallyFailing(open_serial(port, settings)))
    service.start({'requestId': 'partial-1234567890123456', 'level': 'short'})
    job = wait_job(service)
    assert job['state'] == 'error' and job['sentLines'] == 0
    assert 'Часть чека могла напечататься' in job['message']
    assert len(read_available(master)) == 2


@pytest.mark.parametrize('level', ['short', 'medium', 'full'])
def test_all_report_lengths_reach_the_uart(printer_fixture, level):
    service, master, path, _, _, _ = printer_fixture
    service.configure(configuration(path))
    service.start({'requestId': 'format-' + level + '-1234567890123456', 'level': level})
    job = wait_job(service)
    assert job['state'] == 'sent', job
    content = read_available(master).decode('ascii')
    assert '51.20' in content and ('Report' in content and level in content)
    assert ('\r\nCELLS\r\n' in content) == (level != 'short')
    assert ('CELL DETAILS' in content) == (level == 'full')
    assert content.endswith('\r\n' * 5)  # last text line + four feed lines
