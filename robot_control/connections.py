"""Passive Linux connection inventory. Never opens devices or changes network state."""
from __future__ import annotations

import copy
import json
import os
import re
import shutil
import subprocess
import threading
import time
from pathlib import Path

MAX_ITEMS = 256


def _text(path):
    try:
        return path.read_text(errors='replace')[:2048].strip()
    except OSError:
        return ''  # unplugged devices and optional sysfs attributes are normal


def _children(path):
    return sorted(path.iterdir())[:MAX_ITEMS]


def _terse(line):
    """NetworkManager's backslash-escaped terse fields, including colons in names."""
    fields, field, escaped = [], '', False
    for char in line:
        if escaped:
            field += char
            escaped = False
        elif char == '\\':
            escaped = True
        elif char == ':':
            fields.append(field)
            field = ''
        else:
            field += char
    return fields + [field]


class Inventory:
    def __init__(self, sysroot='/sys', devroot='/dev', runner=None):
        self.sys = Path(sysroot)
        self.dev = Path(devroot)
        self.runner = runner or subprocess.run
        self._deadline = None

    def command(self, args):
        # Fixed read-only argv; no shell, sudo, scan, connect, pairing or secrets.
        if self.runner is subprocess.run and not shutil.which(args[0]):
            raise RuntimeError(f'{args[0]} не установлен')
        remaining = self._deadline - time.monotonic() if self._deadline is not None else 0.8
        if remaining <= 0:
            raise RuntimeError('Лимит времени сбора сведений исчерпан')
        try:
            result = self.runner(args, capture_output=True, text=True, timeout=min(0.8, remaining),
                                 env={**os.environ, 'LC_ALL': 'C', 'LANG': 'C'})
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise RuntimeError(f'{args[0]} недоступен или не ответил') from exc
        if result.returncode:
            raise RuntimeError(f'{args[0]}: нет доступа или служба недоступна')
        output = result.stdout[:512_000]
        if args[0] == 'bluetoothctl' and any(message in output for message in (
                'Invalid command', 'Invalid argument', 'No default controller', 'not available')):
            raise RuntimeError('Bluetooth: команда или адаптер недоступны')
        return output

    def usb(self):
        devices = []
        for path in _children(self.sys / 'bus/usb/devices'):
            vendor, product = _text(path / 'idVendor'), _text(path / 'idProduct')
            if not vendor or not product:
                continue  # interfaces are children of the physical device
            devices.append({'name': _text(path / 'product') or 'USB-устройство',
                            'manufacturer': _text(path / 'manufacturer'),
                            'serial': _text(path / 'serial'), 'path': path.name,
                            'vidPid': f'{vendor}:{product}',
                            'speed': _text(path / 'speed'),
                            'authorized': _text(path / 'authorized') != '0'})
        return devices, []

    def serial(self):
        # Actual kernel names only, and annotations from explicit assignments.
        assignments = {}
        for key, role in [('RC_BMS_PORT', 'BMS'), ('RC_LIDAR_PORT', 'Лидар')] + [
                (f'RC_WHEEL_{wheel}_PORT', f'Колесо {wheel}') for wheel in ('FL', 'FR', 'RL', 'RR')]:
            for value in os.environ.get(key, '').split(','):
                if value.strip():
                    assignments.setdefault(os.path.realpath(value.strip()), []).append(role)
        paths = {str(self.dev / p.name) for p in _children(self.sys / 'class/tty')
                 if re.fullmatch(r'(tty(?:USB|ACM|S|AMA)\d+|rfcomm\d+)', p.name)
                 and (self.dev / p.name).exists()}
        paths.update(assignments)
        return [{'path': path, 'name': Path(path).name,
                 'present': Path(path).exists(), 'roles': assignments.get(path, [])}
                for path in sorted(paths)[:MAX_ITEMS]], []

    def network(self):
        warnings, addresses, profiles = [], {}, {}
        try:
            payload = json.loads(self.command(['ip', '-j', 'address', 'show']))
            for item in payload[:MAX_ITEMS]:
                addresses[item['ifname']] = [f"{a['local']}/{a['prefixlen']}"
                    for a in item.get('addr_info', []) if a.get('local')]
        except (RuntimeError, ValueError, KeyError, TypeError, AttributeError) as exc:
            warnings.append(f'IP-адреса недоступны: {exc}')
        try:
            output = self.command(['nmcli', '-t', '-f', 'DEVICE,TYPE,STATE,CONNECTION', 'device', 'status'])
            for line in output.splitlines()[:MAX_ITEMS]:
                fields = _terse(line)
                if len(fields) == 4:
                    profiles[fields[0]] = fields[1:]
        except RuntimeError as exc:
            warnings.append(str(exc))
        devices = []
        for path in _children(self.sys / 'class/net'):
            profile = profiles.get(path.name, [])
            wireless = (path / 'wireless').exists() or (profile and profile[0] == 'wifi')
            kind = ('Wi-Fi' if wireless else 'Loopback' if path.name == 'lo' else
                    'Ethernet' if _text(path / 'type') == '1' and (path / 'device').exists()
                    else 'Виртуальный / другой')
            devices.append({'name': path.name, 'kind': kind,
                'state': _text(path / 'operstate') or 'unknown',
                'carrier': _text(path / 'carrier'), 'mac': _text(path / 'address'),
                'speed': _text(path / 'speed') if not wireless else '',
                'addresses': addresses.get(path.name, []),
                'profile': profile[2] if profile and profile[2] != '--' else '',
                'connectionState': profile[1] if profile else ''})
        return devices, warnings

    def bluetooth(self):
        adapters = [{'name': p.name, 'address': _text(p / 'address')}
                    for p in _children(self.sys / 'class/bluetooth') if re.fullmatch(r'hci\d+', p.name)]
        devices, warnings, successful = {}, [], set()
        try:
            output = self.command(['bluetoothctl', 'list'])
            for line in output.splitlines()[:MAX_ITEMS]:
                match = re.fullmatch(r'Controller ([0-9A-Fa-f:]{17})\s*(.*?)(?: \[default\])?', line.strip())
                if match:
                    address, name = match.groups()
                    adapters.append({'name': name or 'Bluetooth', 'address': address})
            # BlueZ supplies the real adapter name/address; sysfs is a fallback.
            if any(item['address'] for item in adapters):
                adapters = [item for item in adapters if item['address']]
        except RuntimeError as exc:
            warnings.append(str(exc))
        # Query BlueZ's known/connected/paired lists only. Never start discovery.
        for filter_name, flag in [('', 'known'), ('Connected', 'connected'), ('Paired', 'paired')]:
            try:
                output = self.command(['bluetoothctl', 'devices'] + ([filter_name] if filter_name else []))
                successful.add(flag)
                for line in output.splitlines()[:MAX_ITEMS]:
                    match = re.fullmatch(r'Device ([0-9A-Fa-f:]{17})\s*(.*)', line.strip())
                    if not match:
                        continue
                    address, name = match.groups()
                    item = devices.setdefault(address, {'name': name or address, 'address': address,
                                                          'connected': None, 'paired': None})
                    item[flag] = True
            except RuntimeError as exc:
                warnings.append(f'{filter_name or "Список устройств"}: {exc}')
        for item in devices.values():
            for flag in ('connected', 'paired'):
                if flag in successful and item[flag] is None:
                    item[flag] = False
        unique = {item['address'] or item['name']: item for item in adapters}
        return {'adapters': list(unique.values()), 'devices': list(devices.values())}, warnings

    def peers(self):
        # ARP/NDP cache only: an entry is NOT proof the peer is still connected.
        payload = json.loads(self.command(['ip', '-j', 'neigh', 'show']))
        if not isinstance(payload, list):
            raise RuntimeError('Не удалось прочитать таблицу соседей')
        devices = []
        for item in payload[:MAX_ITEMS]:
            if not isinstance(item, dict) or not item.get('dst'):
                continue
            devices.append({'address': item['dst'], 'interface': item.get('dev', ''),
                            'mac': item.get('lladdr', ''), 'state': item.get('state', [])})
        return devices, []

    def peripherals(self):
        devices, warnings = [], []
        for directory, kind, pattern, attribute in [
            ('video4linux', 'Камера', r'video\d+', 'name'),
            ('input', 'Устройство ввода', r'event\d+', 'device/name'),
            ('block', 'Накопитель', r'(sd[a-z]+|nvme\d+n\d+|mmcblk\d+)', 'device/model'),
            ('drm', 'Монитор', r'card\d+-.+', 'status')]:
            root = self.sys / 'class' / directory
            if not root.exists():
                continue
            for path in _children(root):
                if not re.fullmatch(pattern, path.name):
                    continue
                value = _text(path / attribute)
                if directory == 'drm' and value != 'connected':
                    continue
                devices.append({'name': value if directory != 'drm' else path.name,
                                'path': (str(self.dev / ('input/' if directory == 'input' else '') / path.name)
                                         if directory != 'drm' else path.name),
                                'kind': kind})
        return devices[:MAX_ITEMS], warnings

    def collect(self):
        sections = {}
        self._deadline = time.monotonic() + 6
        for name in ('usb', 'serial', 'network', 'bluetooth', 'peripherals', 'peers'):
            try:
                items, warnings = getattr(self, name)()
                sections[name] = {'items': items, 'warnings': warnings,
                                  'status': 'partial' if warnings else 'ok'}
            except (OSError, RuntimeError, ValueError, TypeError, KeyError, AttributeError) as exc:
                sections[name] = {'items': {} if name == 'bluetooth' else [],
                                  'status': 'unavailable', 'warnings': [str(exc)]}
        return {'hostname': os.uname().nodename, 'updatedAt': time.time(), 'sections': sections}


class ConnectionsMonitor:
    """One cached reader per onboard process. HTTP and motor threads never wait on tools."""
    def __init__(self, collector=None, interval=5):
        self.collector = collector or Inventory().collect
        self.interval = interval
        self._lock = threading.Lock()
        self._data = None
        self._refreshing = False
        self._last_attempt = float('-inf')
        self._error = None

    def snapshot(self):
        with self._lock:
            if not self._refreshing and time.monotonic() - self._last_attempt >= self.interval:
                self._refreshing = True
                self._last_attempt = time.monotonic()
                threading.Thread(target=self._refresh, name='connections-inventory', daemon=True).start()
            return {'ok': True, 'data': copy.deepcopy(self._data),
                    'refreshing': self._refreshing, 'error': self._error}

    def _refresh(self):
        try:
            data = self.collector()
            with self._lock:
                self._data = data
                self._error = None
        except Exception:
            with self._lock:
                self._error = 'Не удалось обновить список подключений'
        finally:
            with self._lock:
                self._refreshing = False
