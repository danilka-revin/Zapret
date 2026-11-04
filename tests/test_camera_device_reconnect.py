"""Camera hotplug/identity regression tests without a physical V4L2 device."""
from __future__ import annotations

import os
import sys
import threading
import time
from types import SimpleNamespace

from robot_control.camera import CameraDevice, CameraManager


def wait_for(predicate, timeout=3):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        if predicate():
            return True
        time.sleep(.01)
    return bool(predicate())


def test_missing_camera_never_falls_back_to_index_and_reconnects_safely(tmp_path, monkeypatch):
    path = tmp_path / 'video-device'
    selected = {'value': str(path)}
    opened = []
    class Cap:
        def __init__(self, target, backend):
            opened.append(target)
            self.active = True
        def isOpened(self): return self.active
        def set(self, *args): return True
        def read(self): return True, object()
        def release(self): self.active = False
    class Bytes:
        def tobytes(self): return b'jpeg'
    fake = SimpleNamespace(VideoCapture=Cap, CAP_V4L2=1, CAP_ANY=0, CAP_PROP_FRAME_WIDTH=1,
                           CAP_PROP_FRAME_HEIGHT=2, CAP_PROP_FPS=3, IMWRITE_JPEG_QUALITY=1,
                           imencode=lambda *args: (True, Bytes()))
    monkeypatch.setitem(sys.modules, 'cv2', fake)
    dev = CameraDevice(3, str(path), role='rear_right', target_provider=lambda: selected['value'])
    dev.start()
    try:
        time.sleep(.15)
        assert opened == [], 'absent device must not open numeric index 3'
        path.touch()
        assert wait_for(lambda: dev.get_frame() == b'jpeg')
        assert opened == [str(path)]
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
