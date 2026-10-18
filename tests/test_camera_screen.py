"""Camera UI does not need hardware to test zoom, reconnects or teardown."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_camera_lifecycle_and_viewport_gestures():
    node = shutil.which('node')
    if not node:
        pytest.skip('node is needed for browser-script tests')
    result = subprocess.run([node, str(ROOT / 'tests/js/camera_view.js')],
                            capture_output=True, text=True, cwd=ROOT, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr


def test_camera_button_and_controls_are_served(kiosk_client):
    page = kiosk_client.get('/kiosk').text
    assert 'id="k-cameras-open"' in page
    assert 'camera_view.js' in page and 'viewport.js' in page
    for n in (1, 2):
        for control in ('in', 'out', 'reset', 'expand', 'zoom'):
            assert f'id="cam{n}-{control}"' in page
    script = kiosk_client.get('/camera_view.js').text
    assert 'getUserMedia' not in script
    assert "'/frame?t='" in script
