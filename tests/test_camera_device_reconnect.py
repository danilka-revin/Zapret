"""Camera hotplug/identity regression tests without a physical V4L2 device."""
from __future__ import annotations

import os
import sys
import threading
import time
from dataclasses import replace
from types import SimpleNamespace

import pytest

from robot_control import camera_settings, device_registry
from robot_control.camera import CameraDevice, CameraManager
from robot_control.config import Config
from robot_control.kiosk_app import create_kiosk_app
from test_device_registry import MutableTopology, node, usb


def wait_for(predicate, timeout=3):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        if predicate():
            return True
        time.sleep(.01)
    return bool(predicate())


def _fake_cv2(opened, properties):
    class Cap:
        def __init__(self, target, backend):
            opened.append(target)
            self.active = True
        def isOpened(self): return self.active
        def set(self, *args): properties.append(args); return True
        def read(self): return True, object()
        def release(self): self.active = False
    class Bytes:
        def tobytes(self): return b'jpeg'
    return SimpleNamespace(VideoCapture=Cap, CAP_V4L2=1, CAP_ANY=0, CAP_PROP_FOURCC=4,
                           VideoWriter_fourcc=lambda *letters: "".join(letters), CAP_PROP_FRAME_WIDTH=1,
                           CAP_PROP_FRAME_HEIGHT=2, CAP_PROP_FPS=3, IMWRITE_JPEG_QUALITY=1,
                           imencode=lambda *args: (True, Bytes()))


def test_missing_camera_never_falls_back_to_index_and_reconnects_safely(tmp_path, monkeypatch):
    path = tmp_path / 'video-device'
    selected = {'value': str(path)}
    opened, properties = [], []
    fake = _fake_cv2(opened, properties)
    monkeypatch.setitem(sys.modules, 'cv2', fake)
    dev = CameraDevice(3, str(path), role='rear_right', target_provider=lambda: selected['value'])
    dev.start()
    try:
        time.sleep(.15)
        assert opened == [], 'absent device must not open numeric index 3'
        path.touch()
        assert wait_for(lambda: dev.get_frame() == b'jpeg')
        assert opened == [str(path)]
        assert properties[0] == (fake.CAP_PROP_FOURCC, "MJPG")
        # Reused pathname/inode replacement must invalidate the old frame
        # before a new camera can be presented as the old role.
        replacement = tmp_path / 'replacement'
        replacement.touch()
        os.replace(replacement, path)
        selected['value'] = ''
        assert dev.get_frame() is None
        assert wait_for(lambda: not dev.is_active)
        time.sleep(.6)
        assert opened == [str(path)], 'identity mismatch must not fall back to an index'
        selected['value'] = str(path)
        assert wait_for(lambda: len(opened) >= 2 and dev.get_frame() == b'jpeg')
    finally:
        dev.stop()
        dev._thread.join(timeout=2)


def test_rechecks_live_usb_before_and_after_open_not_just_cached_path(tmp_path, monkeypatch):
    path = tmp_path / 'video-device'; path.touch()
    identity = {'live': False, 'swap_on_open': False}
    opened, released = [], []
    class Cap:
        def __init__(self, target, backend):
            opened.append(target)
            if identity['swap_on_open']:
                identity['live'] = False
        def isOpened(self): return True
        def release(self): released.append(True)
    fake = SimpleNamespace(VideoCapture=Cap, CAP_V4L2=1, CAP_ANY=0)
    monkeypatch.setitem(sys.modules, 'cv2', fake)
    dev = CameraDevice(0, str(path), target_provider=lambda: str(path),
                       revalidate_provider=lambda: str(path) if identity['live'] else '')
    dev.start()
    try:
        time.sleep(.65)
        assert not opened, 'replacement at cached /dev/videoN must not be opened'
        identity.update(live=True, swap_on_open=True)
        assert wait_for(lambda: opened and released)
        assert dev.get_frame() is None and not dev.is_active
    finally:
        dev.stop()
        dev._thread.join(timeout=2)


def test_manager_singleton_is_constructed_once_under_concurrent_requests(monkeypatch):
    CameraManager.reset_instance()
    entered = []
    def init(_self):
        entered.append(1)
        time.sleep(.05)
    monkeypatch.setattr(CameraManager, '_init_devices', init)
    results = []
    threads = [threading.Thread(target=lambda: results.append(CameraManager.get_instance())) for _ in range(12)]
    try:
        for thread in threads: thread.start()
        for thread in threads: thread.join()
        assert len(entered) == 1 and len({id(result) for result in results}) == 1
    finally:
        CameraManager.reset_instance()


def test_open_port_without_a_frame_is_not_live(monkeypatch, tmp_path):
    path = tmp_path / 'video'; path.touch()
    class EmptyCapture:
        def __init__(self, *args): pass
        def isOpened(self): return True
        def set(self, *args): return False
        def read(self): return False, None
        def release(self): pass
    fake = SimpleNamespace(VideoCapture=EmptyCapture, CAP_V4L2=1, CAP_PROP_FOURCC=6,
                           VideoWriter_fourcc=lambda *args: 0, CAP_PROP_FRAME_WIDTH=3,
                           CAP_PROP_FRAME_HEIGHT=4, CAP_PROP_FPS=5)
    monkeypatch.setitem(sys.modules, 'cv2', fake)
    dev = CameraDevice(0, str(path)); dev.start()
    try:
        assert wait_for(lambda: 'нет кадра' in dev.capture_error)
        item, = CameraManager.list_cameras(SimpleNamespace(cameras={0: dev}))
        assert item['exists'] and not item['active']
        assert not item['frameSource'] and item['frameAgeS'] is None
        assert str(path) in item['captureError']
    finally:
        dev.stop(); dev._thread.join(timeout=2)


def test_cold_kiosk_waits_for_saved_usb_camera_and_restores_raw_frame(tmp_path, monkeypatch):
    """A replacement at the old number is not the assigned front camera."""
    opened = []
    monkeypatch.setitem(sys.modules, 'cv2', _fake_cv2(opened, []))
    for keys in camera_settings.ENV_KEYS_BY_ROLE.values():
        for key in keys:
            monkeypatch.setenv(key, '')
    saved_port = '/dev/video0'
    old_node, new_node = tmp_path / 'video0', tmp_path / 'video8'
    old_node.touch()

    def camera(socket, serial, path, aliases=()):
        return usb(socket, '046d:0843', serial, [node(str(path), 'video', index='0', aliases=aliases)])

    previous = device_registry.DeviceRegistry(topology=MutableTopology([
        camera('1-4', 'CAM-FL', old_node, [saved_port]),
    ]))
    previous.set_hint('camera_FL', saved_port)
    previous.confirm_camera('camera_FL', [{'id': 0, 'openedPath': str(old_node),
                                          'frameSource': 'capture', 'frameAgeS': 0.1}])
    previous.set_hint('camera_RL', '')
    camera_settings.save_camera_settings({'front': saved_port, 'rear': ''})

    # New process state: only the two persistent files survive, not the
    # capture object, the previous topology, or the live os.environ changes.
    replacement = camera('1-5', 'OTHER-CAMERA', old_node, [saved_port])
    boot_usb = MutableTopology([replacement])
    monkeypatch.setattr(device_registry, 'UsbTopology', lambda: boot_usb)
    monkeypatch.setenv('RC_CAMERA_RL_PORT', '/dev/video1')  # stale env from an older install
    cfg = replace(Config.from_env(tmp_path), testing=True, slam_source='none')
    CameraManager.reset_instance()
    app = create_kiosk_app(cfg)
    client = app.test_client()
    manager = CameraManager.get_instance()
    try:
        assert wait_for(lambda: manager.get_camera(0).capture_error)
        assert client.get('/api/camera/0/frame').status_code == 404
        assert not opened

        new_node.touch()
        boot_usb.devices = [replacement, camera('1-4', 'CAM-FL', new_node)]
        # No inventory API or forced rescan is called after the new USB node
        # appears. The capture worker must refresh the cached topology itself.
        assert wait_for(lambda: manager.get_frame(0) == b'jpeg')
        frame = client.get('/api/camera/front/frame')
        assert (frame.status_code, frame.mimetype, frame.data) == (200, 'image/jpeg', b'jpeg')
        front, rear, *_ = client.get('/api/cameras').json['cameras']
        assert (front['device'], front['openedPath'], front['role']) == (saved_port, str(new_node), 'front_left')
        assert front['active'] and front['captureError'] is None
        assert manager.get_camera(0).role_verified()
        assert not rear['device'] and not rear['active']
        assert client.get('/api/camera/rear/frame').status_code == 404
        assert opened == [str(new_node)]
    finally:
        CameraManager.reset_instance()
        for dev in manager.cameras.values():
            dev._thread.join(timeout=2)


@pytest.mark.parametrize('module', [None, SimpleNamespace()])
def test_missing_or_incomplete_opencv_reports_a_capture_error(monkeypatch, module):
    monkeypatch.setitem(sys.modules, 'cv2', module)
    dev = CameraDevice(0, '/dev/video0')
    dev.start()
    dev._thread.join(timeout=2)
    assert 'OpenCV' in dev.capture_error
    assert './scripts/ensure_deps.sh --camera' in dev.capture_error
    assert dev.get_frame() is None
    dev.feed_frame(b'external-jpeg')
    assert dev.get_frame() == b'external-jpeg'


def test_mjpeg_stream_follows_reconfigured_device():
    manager = CameraManager.__new__(CameraManager)
    first = CameraDevice(0, '')
    second = CameraDevice(0, '')
    manager.cameras = {0: first}
    first.feed_frame(b'first')
    second.feed_frame(b'second')
    stream = manager.generate_mjpeg_stream(0)
    assert next(stream).endswith(b'first\r\n')
    manager.cameras[0] = second
    assert next(stream).endswith(b'second\r\n')
    stream.close()
