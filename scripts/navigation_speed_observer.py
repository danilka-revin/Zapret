#!/usr/bin/env python3
"""Read-only, experimental comparison of map displacement and wheel SC speed.

Never sends motion or STOP commands, changes configuration, or enables navigation.
Map jumps, slip, SC scaling and telemetry latency can invalidate these estimates.
"""
from __future__ import annotations

import argparse
import math
import sys
import time

from navigation_check import fetch


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def map_speed(previous, current, now):
    """Return displacement/dt only for consecutive fresh, matched map updates."""
    for sample in (previous, current):
        if (not isinstance(sample, dict) or sample.get('ok') is not True or
                not number(sample.get('matched')) or sample['matched'] <= 0 or
                (sample.get('settings') or {}).get('matching') is not True or
                not number(sample.get('quality')) or sample['quality'] < 0.55 or
                not number(sample.get('updatedAt')) or type(sample.get('scans')) is not int):
            return None
        pose = sample.get('pose')
        if not isinstance(pose, dict) or not all(number(pose.get(k)) for k in ('x', 'y')):
            return None
    age = now - current['updatedAt']
    dt = current['updatedAt'] - previous['updatedAt']
    if not 0 <= age <= 2 or not 0.2 <= dt <= 2 or current['scans'] <= previous['scans']:
        return None
    dx = current['pose']['x'] - previous['pose']['x']
    dy = current['pose']['y'] - previous['pose']['y']
    return math.hypot(dx, dy) / dt


def wheel_speed(response, now):
    """Read the backend's configured-diameter SC estimate only if telemetry is fresh."""
    if not isinstance(response, dict) or response.get('ok') is not True:
        return None
    data = response.get('data')
    if (not isinstance(data, dict) or data.get('linkOk') is not True or
            not number(data.get('ts')) or not 0 <= now - data['ts'] <= 1):
        return None
    motors = data.get('motors')
    if not isinstance(motors, list) or len(motors) != 4:
        return None
    if {m.get('id') for m in motors if isinstance(m, dict)} != {'FL', 'FR', 'RL', 'RR'}:
        return None
    for m in motors:
        age, rpm, ppr = m.get('telemetryAgeS'), m.get('scRpm'), m.get('scPpr')
        if (m.get('online') is not True or not number(age) or not 0 <= age < 0.75 or
                not number(rpm) or rpm < 0 or type(ppr) is not int or ppr <= 0):
            return None
    value = data.get('wheelSpeedMps')
    return value if number(value) and value >= 0 else None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8090')
    parser.add_argument('--seconds', type=float, default=15.0)
    args = parser.parse_args(argv)
    if not number(args.seconds) or not 1 <= args.seconds <= 600:
        parser.error('--seconds должен быть от 1 до 600')
    if not args.url.startswith(('http://', 'https://')):
        parser.error('--url должен начинаться с http:// или https://')
    origin = args.url.rstrip('/')
    print('Только чтение: оценка скорости, не калибровка и не разрешение движения.', flush=True)
    previous = None
    deadline = time.monotonic() + args.seconds
    any_valid = False
    while time.monotonic() < deadline:
        current = fetch(origin + '/api/kiosk_map')
        wheels = fetch(origin + '/api/kiosk_state')
        estimate = map_speed(previous, current, time.time()) if previous is not None else None
        wheel = wheel_speed(wheels, time.time())
        any_valid |= estimate is not None or wheel is not None
        fmt = lambda v: 'нет данных' if v is None else f'{v:.3f} м/с'
        print(f'Лидар/карта: {fmt(estimate)}; окружность колёс SC: {fmt(wheel)}', flush=True)
        previous = current
        time.sleep(0.5)
    print('SC может ошибаться из-за диаметра, проскальзывания и настройки PPR; карта может прыгать.')
    print('Не использовать показания как гарантию торможения или для отключения защит.')
    return 0 if any_valid else 1


if __name__ == '__main__':
    sys.exit(main())
