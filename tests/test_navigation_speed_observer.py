import importlib.util
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
spec = importlib.util.spec_from_file_location('navigation_speed_observer', Path(__file__).resolve().parents[1] / 'scripts/navigation_speed_observer.py')
observer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(observer)


def sample(t, x, scans):
    return {'ok': True, 'matched': True, 'quality': 0.9, 'settings': {'matching': True},
            'updatedAt': t, 'scans': scans, 'pose': {'x': x, 'y': 0.0}}


def wheels():
    return {'ok': True, 'data': {'linkOk': True, 'wheelSpeedMps': 0.3,
            'motors': [{'id': i, 'online': True, 'telemetryAgeS': 0.1, 'scRpm': 20, 'scPpr': 90}
                       for i in ('FL', 'FR', 'RL', 'RR')]}}


def test_map_estimate_and_invalid_updates():
    a, b = sample(10, 0, 1), sample(11, 0.5, 2)
    assert observer.map_speed(a, b, 11.5) == 0.5
    assert observer.map_speed(a, b, 14) is None
    assert observer.map_speed(b, b, 11.5) is None
    b['matched'] = False
    assert observer.map_speed(a, b, 11.5) is None
    b['matched'] = True
    b['pose']['x'] = float('nan')
    assert observer.map_speed(a, b, 11.5) is None


def test_wheel_estimate_requires_all_fresh_wheels():
    state = wheels()
    assert observer.wheel_speed(state) == 0.3
    state['data']['motors'][0]['telemetryAgeS'] = 1
    assert observer.wheel_speed(state) is None
    state = wheels()
    state['data']['motors'][0]['scPpr'] = 0
    assert observer.wheel_speed(state) is None
    state = wheels()
    state['data']['motors'][0]['id'] = 'FR'
    assert observer.wheel_speed(state) is None


def test_observer_only_reads_api(monkeypatch, capsys):
    calls = []
    clock = iter([0, 0.1, 0.6, 1.1])
    monkeypatch.setattr(observer.time, 'monotonic', lambda: next(clock))
    monkeypatch.setattr(observer.time, 'time', lambda: 11.0)
    monkeypatch.setattr(observer.time, 'sleep', lambda _: None)

    def fetch(url):
        calls.append(url)
        return sample(10, 0, 1) if url.endswith('/api/kiosk_map') else wheels()

    monkeypatch.setattr(observer, 'fetch', fetch)
    assert observer.main(['--url', 'http://robot', '--seconds', '1']) == 0
    assert set(calls) == {'http://robot/api/kiosk_map', 'http://robot/api/kiosk_state'}
    assert 'не калибровка' in capsys.readouterr().out
