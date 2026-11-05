"""Состояние бортового компьютера: только чтение /proc и /sys, честные «нет данных»."""
import io
import os
import shutil
import subprocess
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from robot_control.connections import ConnectionsMonitor
from robot_control.system_monitor import SystemMonitor, SystemState, _celsius

CPUINFO = """processor\t: 0
model name\t: ARMv8 Processor rev 3
cpu MHz\t\t: 1500.000
processor\t: 1
model name\t: ARMv8 Processor rev 3
cpu MHz\t\t: 1500.000
"""


def build(tmp_path, *, zones=(('cpu-thermal', '54000'),), hwmon=(),
          stat_busy=100, stat_total=400, mem_used_mb=1800, mem_total_mb=3900,
          load='0.42 0.30 0.20 1/123 4567', uptime='7200.55\n'):
    proc, sysroot, root = tmp_path / 'proc', tmp_path / 'sys', tmp_path / 'root'
    for directory in (proc, sysroot / 'class/thermal', sysroot / 'class/hwmon',
                      sysroot / 'devices/system/cpu/cpu0/cpufreq', root):
        directory.mkdir(parents=True, exist_ok=True)
    if stat_busy is not None:
        idle = stat_total - stat_busy
        (proc / 'stat').write_text('cpu  %d 0 %d %d 0 0 0 0 0 0\ncpu0 %d 0 %d %d 0 0 0 0 0 0\n'
                                   % (stat_busy, 0, idle, stat_busy, 0, idle))
    if mem_total_mb is not None:
        total = mem_total_mb * 1024
        (proc / 'meminfo').write_text(
            'MemTotal: %d kB\nMemFree: 200000 kB\nMemAvailable: %d kB\n'
            'SwapTotal: 102400 kB\nSwapFree: 51200 kB\n' % (total, (mem_total_mb - mem_used_mb) * 1024))
    (proc / 'loadavg').write_text(load + '\n')
    (proc / 'uptime').write_text(uptime)
    (proc / 'cpuinfo').write_text(CPUINFO)
    (sysroot / 'devices/system/cpu/cpu0/cpufreq/scaling_cur_freq').write_text('1500000\n')
    for index, (name, millis) in enumerate(zones):
        zone = sysroot / 'class/thermal' / ('thermal_zone%d' % index)
        zone.mkdir(parents=True, exist_ok=True)
        (zone / 'type').write_text(name + '\n')
        (zone / 'temp').write_text(millis + '\n')
    for index, (name, millis) in enumerate(hwmon):
        chip = sysroot / 'class/hwmon' / ('hwmon%d' % index)
        chip.mkdir(parents=True, exist_ok=True)
        (chip / 'name').write_text(name + '\n')
        (chip / 'temp1_input').write_text(millis + '\n')
        (chip / 'temp1_label').write_text('Package id 0\n')
    return proc, sysroot, root


def state(tmp_path, clock=None, **kwargs):
    proc, sysroot, root = build(tmp_path, **kwargs)
    return SystemState(procroot=proc, sysroot=sysroot, root=root,
                       hostname=lambda: 'robot-pc', clock=clock or (lambda: 1000.0))


def test_temperature_cpu_memory_disk_and_uptime_are_real_readings(tmp_path):
    snapshot = state(tmp_path).collect()
    assert snapshot['hostname'] == 'robot-pc'
    assert snapshot['status'] == 'ok', snapshot['warnings']
    assert snapshot['warnings'] == []

    temp = snapshot['temperature']
    assert temp['available'] and temp['maxC'] == 54.0 and temp['hottest'] == 'cpu-thermal'
    assert temp['zones'] == [{'name': 'cpu-thermal', 'celsius': 54.0, 'source': 'thermal'}]

    cpu = snapshot['cpu']
    assert cpu['cores'] == 2 and cpu['model'] == 'ARMv8 Processor rev 3'
    assert cpu['load1'] == 0.42 and cpu['frequencyMHz'] == 1500
    assert cpu['usagePercent'] is None  # первый замер не выдаём за проценты
    assert cpu['runningProcesses'] == 1

    memory = snapshot['memory']
    assert memory['totalMB'] == 3900 and memory['usedMB'] == 1800
    assert memory['usedPercent'] == pytest.approx(46.2, abs=0.2)
    assert memory['swapUsedMB'] == 50
    assert snapshot['disk']['available'] and snapshot['disk']['usedPercent'] >= 0
    assert snapshot['uptimeS'] == pytest.approx(7200.55)


def test_cpu_usage_needs_two_samples_and_ignores_garbage(tmp_path):
    reader = state(tmp_path)
    assert reader.collect()['cpu']['usagePercent'] is None
    (reader.proc / 'stat').write_text('cpu  300 0 0 500 0 0 0 0 0 0\n')
    # 200 из 400 новых тиков = 50 %.
    assert reader.collect()['cpu']['usagePercent'] == 50.0
    (reader.proc / 'stat').write_text('мусор\n')
    assert reader.collect()['cpu']['usagePercent'] is None


def test_hwmon_is_used_when_thermal_zones_are_missing(tmp_path):
    snapshot = state(tmp_path, zones=(), hwmon=(('coretemp', '61000'), ('k10temp', '4000'))).collect()
    temp = snapshot['temperature']
    assert temp['available'] and temp['maxC'] == 61.0
    assert temp['hottest'] == 'coretemp Package id 0'
    assert [zone['source'] for zone in temp['zones']] == ['hwmon', 'hwmon']


def test_hot_processor_memory_and_disk_raise_status_and_warnings(tmp_path, monkeypatch):
    snapshot = state(tmp_path, zones=(('cpu-thermal', '86000'),)).collect()
    assert snapshot['status'] == 'critical'
    assert any('86' in warning for warning in snapshot['warnings'])

    warn = state(tmp_path / 'warn', zones=(('cpu-thermal', '78000'),)).collect()
    assert warn['status'] == 'warn' and any('78' in warning for warning in warn['warnings'])

    mem = state(tmp_path / 'mem', mem_used_mb=3750).collect()
    assert mem['status'] == 'warn' and any('памяти' in warning for warning in mem['warnings'])

    full = state(tmp_path / 'disk')
    monkeypatch.setattr(os, 'statvfs', lambda path: SimpleNamespace(
        f_blocks=100, f_frsize=1_000_000_000, f_bavail=2))
    disk = full.collect()
    assert disk['disk']['usedPercent'] == 98.0
    assert disk['status'] == 'critical'
    assert any('Диск' in warning for warning in disk['warnings'])


def test_missing_sensors_are_reported_not_invented(tmp_path):
    snapshot = state(tmp_path, zones=(), stat_busy=None, mem_total_mb=None).collect()
    temp = snapshot['temperature']
    assert temp['available'] is False and temp['maxC'] is None and temp['zones'] == []
    assert 'датчики температуры не найдены' in temp['reason']
    assert snapshot['cpu']['usagePercent'] is None
    assert snapshot['memory']['available'] is False
    assert snapshot['status'] == 'ok'
    assert snapshot['warnings'] and 'Температура недоступна' in snapshot['warnings'][0]
    # Диск всё равно читается: /proc и /sys могут быть неполными.
    assert snapshot['disk']['available'] is True


def test_celsius_rejects_impossible_values():
    assert _celsius(54000) == 54.0
    assert _celsius(54) == 54.0
    assert _celsius(0) == 0.0  # ноль — возможное показание, не «нет данных»
    assert _celsius(-273000) is None
    assert _celsius(None) is None


def test_threshold_overrides_are_sane(tmp_path, monkeypatch):
    monkeypatch.setenv('RC_PC_TEMP_WARN_C', '60')
    monkeypatch.setenv('RC_PC_TEMP_CRIT_C', '70')
    hot = state(tmp_path, zones=(('cpu-thermal', '65000'),)).collect()
    assert hot['thresholds']['warnTempC'] == 60.0 and hot['status'] == 'warn'
    # Порог «критично» ниже «внимание» — настройка бессмысленна, берём штатные значения.
    monkeypatch.setenv('RC_PC_TEMP_WARN_C', '90')
    monkeypatch.setenv('RC_PC_TEMP_CRIT_C', '80')
    reader = state(tmp_path / 'swapped', zones=(('cpu-thermal', '86000'),))
    assert reader.warn_temp_c == 75.0 and reader.crit_temp_c == 85.0


def test_monitor_serves_cached_snapshot_without_blocking(tmp_path):
    release, entered = threading.Event(), threading.Event()

    class Slow:
        def collect(self):
            entered.set()
            assert release.wait(3)
            return {'hostname': 'robot-pc', 'collectedAt': 1000.0, 'status': 'ok'}

    monitor = SystemMonitor(Slow(), interval=0.0)
    first = monitor.snapshot()
    assert first['data'] is None and first['refreshing'] is True
    assert entered.wait(1)
    assert monitor.snapshot()['data'] is None  # запрос не ждёт чтения датчиков
    release.set()
    for _ in range(100):
        data = monitor.snapshot()['data']
        if data is not None:
            break
        threading.Event().wait(0.02)
    assert data['hostname'] == 'robot-pc'
    data['hostname'] = 'changed by browser'
    assert monitor.snapshot()['data']['hostname'] == 'robot-pc'

    class Broken:
        def collect(self):
            raise OSError('proc недоступен')

    monitor.reader = Broken()
    monitor._last_attempt = float('-inf')
    monitor.interval = 0.0
    monitor.refresh()
    assert monitor.snapshot()['error']
    assert monitor.snapshot()['data']['hostname'] == 'robot-pc'


def test_connections_monitor_merges_system_and_keeps_it_when_inventory_fails(tmp_path, monkeypatch):
    readings = []

    class Reader:
        def collect(self):
            readings.append(1)
            return {'hostname': 'robot-pc', 'collectedAt': time.time(), 'status': 'ok',
                    'temperature': {'available': True, 'maxC': 54.0}}

    monitor = ConnectionsMonitor(lambda: {'hostname': 'robot-pc', 'updatedAt': 1.0,
                                         'sections': {}}, interval=0.0,
                                system=Reader(), system_interval=0.0)
    # This test explicitly drives synchronous refreshes. Otherwise snapshot()
    # schedules another interval=0 background refresh, which can finish after
    # the intended failing one and nondeterministically clear its error.
    # Separate tests cover the real asynchronous scheduler.
    monkeypatch.setattr(monitor, '_maybe_start', lambda: None)
    monitor._refresh()
    data = monitor.snapshot()['data']
    assert data['sections'] == {} and data['system']['temperature']['maxC'] == 54.0
    assert data['systemError'] is None and data['system']['stale'] is False

    monitor.collector = lambda: (_ for _ in ()).throw(OSError('занято'))
    monitor._refresh()
    snapshot = monitor.snapshot()
    assert snapshot['error'] and snapshot['data']['system']['temperature']['maxC'] == 54.0
    assert snapshot['data']['hostname'] == 'robot-pc'

    class Broken:
        def collect(self):
            raise OSError('нет датчиков')

    monitor.system = Broken()
    monitor._refresh_system()
    assert monitor.snapshot()['data']['systemError']


def test_api_system_is_read_only_and_never_cached(kiosk_client, monkeypatch):
    monitor = kiosk_client.application.extensions['rc_system']
    payload = {'ok': True, 'data': {'hostname': 'onboard', 'status': 'ok', 'stale': False},
               'refreshing': False, 'error': None}
    monkeypatch.setattr(monitor, 'snapshot', lambda: payload)
    response = kiosk_client.get('/api/pc_state')
    assert response.status_code == 200 and response.json == payload
    assert response.headers['Cache-Control'] == 'no-store'
    assert kiosk_client.post('/api/pc_state').status_code == 405


def test_connections_payload_includes_temperature_for_the_screen(kiosk_client, monkeypatch):
    monitor = kiosk_client.application.extensions['rc_connections']
    snapshot = {'ok': True, 'data': {'hostname': 'onboard', 'updatedAt': 1.0, 'sections': {},
                                     'system': {'status': 'warn', 'temperature': {'maxC': 80.0}},
                                     'systemError': None},
                'refreshing': False, 'error': None}
    monkeypatch.setattr(monitor, 'snapshot', lambda: snapshot)
    response = kiosk_client.get('/api/connections')
    assert response.json['data']['system']['temperature']['maxC'] == 80.0


def test_kiosk_screen_has_a_quiet_pc_chip(kiosk_client):
    page = kiosk_client.get('/robot.html').get_data(as_text=True)
    assert 'id="pc-chip"' in page and 'id="data-chip"' in page
    assert page.index('id="pc-chip"') < page.index('id="data-chip"')
    # Модуль чипа подключён раньше robot.js и отдаётся как статика.
    assert page.index('pc_chip.js') < page.index('src="robot.js"')
    assert kiosk_client.get('/pc_chip.js').status_code == 200


def test_connections_screen_has_the_pc_panel(client, operator):
    from conftest import login
    login(client)
    page = client.get('/connections').get_data(as_text=True)
    assert 'id="system-facts"' in page and 'id="system-warning"' in page
    # Панель компьютера стоит до списка устройств: состояние — первым делом.
    assert page.index('id="system-facts"') < page.index('id="usb-items"')


def test_operator_app_serves_and_proxies_the_pc_state(client, operator, kiosk_client, monkeypatch):
    from conftest import login
    from robot_control import map_peer
    from urllib.error import URLError
    monkeypatch.setenv('RC_SLAM_PEER_URL', 'http://onboard.example:8090/api/kiosk_state')
    login(client)
    assert client.get('/pc_chip.js').status_code == 200
    payload = {'ok': True, 'data': {'status': 'ok', 'temperature': {'maxC': 54.0}},
               'refreshing': False, 'error': None}
    monkeypatch.setattr(kiosk_client.application.extensions['rc_system'], 'snapshot', lambda: payload)
    calls = []

    def forward(req, timeout):
        from urllib.parse import urlsplit
        calls.append(req.full_url)
        assert not req.data
        response = kiosk_client.get(urlsplit(req.full_url).path)
        reply = io.BytesIO(response.data)
        reply.code = response.status_code
        return reply

    monkeypatch.setattr(map_peer, 'urlopen', forward)
    response = client.get('/api/pc_state')
    assert response.status_code == 200 and response.json == payload
    assert response.headers['Cache-Control'] == 'no-store'
    assert calls == ['http://onboard.example:8090/api/pc_state']
    # Офлайн-борт: прокси отвечает 503, а не пустыми нулями.
    monkeypatch.setattr(map_peer, 'urlopen', lambda *a, **kw: (_ for _ in ()).throw(URLError('offline')))
    assert client.get('/api/pc_state').status_code == 503
    assert client.post('/api/pc_state').status_code == 405


def test_pc_chip_javascript():
    node = shutil.which('node')
    if not node:
        pytest.skip('node is not installed')
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run([node, 'tests/js/pc_chip.js'], cwd=root,
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
