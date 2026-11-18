"""OpenCV provision/diagnostics on the onboard interpreter, no network or camera needed."""
from __future__ import annotations

import json
import os
import subprocess
import sys
import zipfile
from pathlib import Path
from textwrap import dedent

import pytest

from scripts import ensure_opencv
from test_lidar_deploy import offline_install, run_deps  # noqa: F401 (shared deployment fixture)


def test_auto_install_uses_camera_settings_not_geometry(tmp_path, monkeypatch):
    for key in ("RC_CAMERA_BACKEND", "RC_CAMERA_ENV_FILE", *(k for keys in ensure_opencv.CAMERA_ENV_KEYS.values() for k in keys)):
        monkeypatch.delenv(key, raising=False)
    dev = tmp_path / "dev"
    dev.mkdir()
    (tmp_path / ".env").write_text('RC_CAMERA_FRONT=\nRC_CAMERA_FR_YAW_DEG=25\n')
    assert not ensure_opencv.needs_camera(tmp_path, dev)
    overlay = tmp_path / 'data/camera-ports.env'
    overlay.parent.mkdir()
    overlay.write_text('RC_CAMERA_FRONT="/dev/v4l/by-path/assigned"\n')
    monkeypatch.setenv('RC_CAMERA_FRONT', '')  # stale web-unit value, not the writable kiosk overlay
    assert ensure_opencv.needs_camera(tmp_path, dev)
    overlay.write_text('RC_CAMERA_FRONT=\n')
    assert not ensure_opencv.needs_camera(tmp_path, dev)
    (dev / 'video1').touch()
    assert ensure_opencv.needs_camera(tmp_path, dev)
    (tmp_path / '.env').write_text('RC_CAMERA_BACKEND=ffmpeg\n')
    assert not ensure_opencv.needs_camera(tmp_path, dev)


def _fixture_wheel(vendor: Path, dist: str, version: str, module: str, code: str):
    """Tiny import-only surrogate wheels to exercise actual pip install/reinstall offline."""
    name = dist.replace('-', '_')
    info = f'{name}-{version}.dist-info'
    files = {
        f'{module}/__init__.py': f'__version__ = "{version}"\n{code}',
        f'{info}/METADATA': f'Metadata-Version: 2.1\nName: {dist}\nVersion: {version}\n',
        f'{info}/WHEEL': 'Wheel-Version: 1.0\nRoot-Is-Purelib: true\nTag: py3-none-any\n',
    }
    record = f'{info}/RECORD'
    files[record] = ''.join(f'{path},,\n' for path in [*files, record])
    with zipfile.ZipFile(vendor / f'{name}-{version}-py3-none-any.whl', 'w') as wheel:
        for path, content in files.items():
            wheel.writestr(path, content)


def test_existing_install_recovers_after_base_hash_hit_and_offline_failure(offline_install):
    project, env = offline_install
    env.update(RC_DEPS_OFFLINE='1', RC_CAMERA_BACKEND='ffmpeg')
    first = run_deps(project, env)
    assert first.returncode == 0 and 'Устанавливаю OpenCV' not in first.stdout
    python = project / '.venv/bin/python'
    manual = [str(python), 'scripts/ensure_opencv.py']
    env['RC_CAMERA_BACKEND'] = 'opencv'
    # Mimic the updater from the PREVIOUS release, passing no new options.
    without = run_deps(project, env)
    assert without.returncode == 0 and 'зависимости уже установлены' in without.stdout
    assert 'OpenCV недоступен' in without.stdout and 'ModuleNotFoundError' in without.stderr
    assert subprocess.run([*manual, '--check'], cwd=project, env=env).returncode != 0
    # A later USB update brings architecture-compatible wheels. A previous
    # failed attempt or matching base hash must not hide them.
    _fixture_wheel(project / 'vendor', 'numpy', '2.2.6' if sys.version_info >= (3, 13) else '1.26.4', 'numpy', '')
    _fixture_wheel(project / 'vendor', 'opencv-python-headless', '4.11.0.86', 'cv2',
                   'VideoCapture = imdecode = imencode = object()\n')
    fixed = run_deps(project, env)
    assert fixed.returncode == 0 and 'OpenCV 4.11.0.86' in fixed.stdout, fixed.stderr
    healthy = run_deps(project, env)
    assert 'OpenCV 4.11.0.86' in healthy.stdout and 'Устанавливаю OpenCV' not in healthy.stdout
    # Metadata still satisfies requirements, but native import is broken. A
    # successful pip exit must NOT be considered a repair unless cv2 loads.
    cv, = (project / '.venv/lib').glob('python*/site-packages/cv2/__init__.py')
    cv.write_text("raise OSError('libcamera_test.so: cannot open shared object file')\n")
    broken = run_deps(project, env)
    assert 'libcamera_test.so' in broken.stderr and 'OpenCV 4.11.0.86' in broken.stdout
    assert broken.returncode == 0 and 'OpenCV недоступен' not in broken.stdout
    assert subprocess.run([*manual, '--check'], cwd=project, env=env).returncode == 0


@pytest.mark.parametrize(('broken_module', 'exception', 'detail'), [
    ('cv2', "ModuleNotFoundError(\"No module named 'cv2'\", name='cv2')", 'Не установлен модуль cv2'),
    ('cv2', "ImportError('libGL.so.1: not found')", 'ImportError: libGL.so.1'),
    ('numpy', "ValueError('numpy.dtype size changed')", 'ValueError: numpy.dtype size changed'),
])
def test_capture_and_recognition_keep_native_import_error(tmp_path, broken_module, exception, detail):
    # A new process is important: the recognition package imports cv2 during
    # app initialization, whereas the camera owner imports it on its worker.
    (tmp_path / 'cv2.py').write_text('VideoCapture = imencode = imdecode = None\n')
    (tmp_path / (broken_module + '.py')).write_text('raise ' + exception + '\n')
    code = dedent('''
        import json
        from robot_control.camera import CameraDevice
        from robot_control.vision.service import VisionConfig, VisionService
        dev = CameraDevice(0, '/dev/assigned')
        dev._capture_loop()
        service = VisionService(config=VisionConfig(enabled=True, traffic_mode='verified'), layout_provider=lambda: [])
        assert service.initialize() == 'unavailable'
        assert not dev.is_active and dev.get_frame() is None
        print(json.dumps([dev.capture_error, service.state()['reason']]))
    ''')
    result = subprocess.run([sys.executable, '-c', code], cwd=ensure_opencv.ROOT,
                            env={**os.environ, 'PYTHONPATH': str(tmp_path), 'RC_CAMERA_BACKEND': 'opencv'},
                            capture_output=True, text=True, timeout=15, check=True)
    capture, vision = json.loads(result.stdout)
    assert all(detail in text and 'scripts/ensure_opencv.py' in text for text in (capture, vision))
    assert 'FFmpeg' not in capture  # explicitly forced OpenCV, not a missing ffmpeg binary
