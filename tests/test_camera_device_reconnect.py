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
    class Pixels:
        size, shape = 12, (2, 2, 3)
        def __getitem__(self, key): return self
        def max(self): return 100
    class Cap:
        def __init__(self, target, backend):
            opened.append(target)
            self.active = True
        def isOpened(self): return self.active
        def set(self, *args): properties.append(args); return True
        def get(self, key): return 15
        def read(self): time.sleep(.02); return True, Pixels()
        def release(self): self.active = False
    class Bytes:
        size = 4
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
    monkeypatch.setattr('robot_control.camera.load_opencv', lambda: (fake, None, ''))
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
    monkeypatch.setattr('robot_control.camera.load_opencv', lambda: (fake, None, ''))
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
        def get(self, key): return 0
        def read(self): return False, None
        def release(self): pass
    fake = SimpleNamespace(VideoCapture=EmptyCapture, CAP_V4L2=1, CAP_PROP_FOURCC=6,
                           VideoWriter_fourcc=lambda *args: 0, CAP_PROP_FRAME_WIDTH=3,
                           CAP_PROP_FRAME_HEIGHT=4, CAP_PROP_FPS=5)
    monkeypatch.setattr('robot_control.camera.load_opencv', lambda: (fake, None, ''))
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
    fake = _fake_cv2(opened, [])
    monkeypatch.setattr('robot_control.camera.load_opencv', lambda: (fake, None, ''))
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
    monkeypatch.setenv('RC_CAMERA_BACKEND', 'opencv')
    monkeypatch.setitem(sys.modules, 'cv2', module)
    dev = CameraDevice(0, '/dev/video0')
    dev.start()
    dev._thread.join(timeout=2)
    assert 'Не установлен модуль cv2' in dev.capture_error or 'OpenCV/NumPy не загрузились' in dev.capture_error
    assert '.venv/bin/python scripts/ensure_opencv.py' in dev.capture_error
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


def test_capture_negotiates_then_serves_decodable_jpeg(kiosk_client, tmp_path, monkeypatch):
    """Real VideoCapture.read + imencode + HTTP; recorded MJPG replaces only V4L2.

    The first profile opens but returns *no* frame; the second works. No HTTP
    feed/injected JPEG can make this test pass. A covered lens is still live.
    """
    cv = pytest.importorskip('cv2')
    np = pytest.importorskip('numpy')
    path = str(tmp_path / 'selected.avi')
    writer = cv.VideoWriter(path, cv.VideoWriter_fourcc(*'MJPG'), 10, (64, 48))
    assert writer.isOpened()
    for _ in range(20):
        writer.write(np.zeros((48, 64, 3), dtype=np.uint8))
    writer.release()
    opened, released = [], []
    capture = cv.VideoCapture

    class Device:
        def __init__(self, target, backend):
            assert target == path and backend == cv.CAP_V4L2
            self.reader = capture(target, cv.CAP_FFMPEG)
            self.settings = {}
            opened.append(self.settings)
        def isOpened(self): return self.reader.isOpened()
        def set(self, key, value): self.settings[key] = value; return False
        def get(self, key): return self.reader.get(key)
        def read(self):
            if self.settings[cv.CAP_PROP_FRAME_WIDTH] != 640:
                return False, None
            time.sleep(.02)
            return self.reader.read()
        def release(self): self.reader.release(); released.append(True)

    monkeypatch.setattr(cv, 'VideoCapture', Device)
    dev = CameraDevice(2, path, role='front_right')
    mgr = object.__new__(CameraManager)
    mgr.cameras = {2: dev}
    monkeypatch.setattr(CameraManager, 'get_instance', lambda: mgr)
    dev.start()
    try:
        assert wait_for(lambda: dev.frames_received >= 2), dev.capture_error
        snap, = kiosk_client.get('/api/cameras').json['cameras']
        assert snap['openedPath'] == path and snap['frameSource'] == 'capture'
        assert snap['active'] and snap['darkFrame'] is True and '640×360' in snap['captureMode']
        assert (snap['width'], snap['height'], snap['fps']) == (64, 48, 10)
        res = kiosk_client.get('/api/camera/2/frame')
        assert res.status_code == 200 and res.mimetype == 'image/jpeg'
        image = cv.imdecode(np.frombuffer(res.data, dtype=np.uint8), cv.IMREAD_COLOR)
        assert image.shape == (48, 64, 3) and image.max() == 0
        assert len(released) == 1 and len(opened) == 2
    finally:
        dev.stop(); dev._thread.join(timeout=2)
    assert len(released) == 2
    assert kiosk_client.get('/api/camera/2/frame').status_code == 404


@pytest.mark.parametrize('backend, cv2_module', [
    pytest.param('auto', None, id='missing-opencv'),
    pytest.param('auto', SimpleNamespace(), id='incomplete-opencv'),
    pytest.param('ffmpeg', None, id='forced-ffmpeg'),
])
def test_ffmpeg_reader_releases_child_on_stop(tmp_path, monkeypatch, backend, cv2_module):
    """Exercise the real FFmpeg pipe/process, substituting a test source for the USB input."""
    import io
    import shutil
    import subprocess
    from robot_control import camera_capture

    image = pytest.importorskip('PIL.Image')
    binary = shutil.which('ffmpeg') or pytest.importorskip('imageio_ffmpeg').get_ffmpeg_exe()
    path = tmp_path / 'selected'; path.touch()
    real_popen = subprocess.Popen
    children = []

    def with_test_input(args, **kwargs):
        assert args[args.index('-i')+1] == str(path) and args[args.index('-f')+1] == 'v4l2'
        out_args = args[args.index('-i')+2:]
        proc = real_popen([binary, '-loglevel', 'error', '-re', '-f', 'lavfi', '-i',
                           'color=c=red:size=1280x720:rate=10', *out_args], **kwargs)
        children.append(proc)
        return proc

    monkeypatch.setattr(camera_capture.subprocess, 'Popen', with_test_input)
    monkeypatch.setattr(shutil, 'which', lambda name: binary if name == 'ffmpeg' else None)
    monkeypatch.setenv('RC_CAMERA_BACKEND', backend)
    if backend == 'auto':
        monkeypatch.setitem(sys.modules, 'cv2', cv2_module)
    else:
        monkeypatch.setattr('robot_control.camera.load_opencv', lambda: pytest.fail('forced FFmpeg loaded OpenCV'))
    dev = CameraDevice(0, str(path)); dev.start()
    try:
        assert wait_for(lambda: dev.frames_received >= 2, timeout=5), dev.capture_error
        jpeg = image.open(io.BytesIO(dev.get_frame()))
        assert jpeg.size == (1280, 720) and jpeg.getpixel((32, 32))[0] >= 240
        assert dev.frame_source == 'capture' and 'FFmpeg' in dev.capture_mode
    finally:
        dev.stop(); dev._thread.join(timeout=2)
    assert len(children) in (1, 2) and all(child.poll() is not None for child in children)
    if len(children) == 2:
        # Old FFmpeg exits on the unsupported flag; both children are reaped.
        assert '-fps_mode' in children[0].args and children[0].returncode != 0
        assert '-vsync' in children[1].args
    assert not dev.get_frame()


def test_legacy_ffmpeg_retries_with_passthrough_without_changing_the_port(monkeypatch):
    from robot_control import camera_capture as cc

    attempted = []
    jpeg = cc.Frame(b'jpeg', 640, 480, None, None, None)

    def old_ffmpeg(args, size, current):
        attempted.append(args)
        assert args[args.index('-i') + 1] == '/dev/selected'
        if '-fps_mode' in args:
            assert args[args.index('-fps_mode') + 1] == 'passthrough'
            raise OSError("Unrecognized option 'fps_mode'.")
        assert args[args.index('-vsync') + 1] == '0'
        yield jpeg

    monkeypatch.setattr(cc, '_ffmpeg_pipe', old_ffmpeg)
    assert list(cc.ffmpeg_frames('ffmpeg', '/dev/selected', cc.modes(False, True)[0], lambda: True)) == [jpeg]
    assert len(attempted) == 2


def test_silent_ffmpeg_child_has_a_deadline_and_is_reaped(monkeypatch):
    import subprocess
    from robot_control import camera_capture as cc
    popen, children = subprocess.Popen, []

    def silent(*args, **kwargs):
        children.append(popen([sys.executable, '-c', 'import time; time.sleep(60)'], **kwargs))
        return children[-1]

    monkeypatch.setattr(subprocess, 'Popen', silent)
    monkeypatch.setattr(cc, 'FFMPEG_FRAME_TIMEOUT_S', .2)
    reader = cc.ffmpeg_frames('ffmpeg', '/dev/chosen', cc.modes(False, True)[0], lambda: True)
    with pytest.raises(OSError, match='нет кадра'):
        next(reader)
    assert len(children) == 1 and children[0].poll() is not None


def test_opt_in_physical_camera_reads_multiple_frames():
    """Run manually with the owner stopped, RC_TEST_V4L2 set to ONE selected video node."""
    path = os.environ.get('RC_TEST_V4L2')
    if not path:
        pytest.skip('no explicitly selected physical camera (RC_TEST_V4L2)')
    import stat
    assert path.startswith('/dev/') and stat.S_ISCHR(os.stat(path).st_mode)
    dev = CameraDevice(0, path); dev.start()
    try:
        assert wait_for(lambda: dev.frames_received >= 3 and dev.get_frame(), timeout=60), dev.capture_error
        assert dev.opened_path == path and dev.frame_source == 'capture'
        print(f'V4L2 {path}: {dev.capture_mode}, JPEG {dev.width}x{dev.height}, count={dev.frames_received}')
    finally:
        dev.stop(); dev._thread.join(timeout=2)
