"""Панель автообследования и форма калибровки проверяются в Node/JS DOM."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_explore_panel_dom_logic():
    node = shutil.which("node")
    if not node:
        pytest.skip("node is needed for browser-script tests")
    result = subprocess.run([node, str(ROOT / "tests/js/explore_screen.js")],
                            capture_output=True, text=True, cwd=ROOT, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "DOM: ok" in result.stdout


def test_legacy_screens_show_four_c930e_cameras():
    main = (ROOT / "slam_gui" / "main.html").read_text(encoding="utf-8")
    index = (ROOT / "slam_gui" / "index.html").read_text(encoding="utf-8")
    for marker in ("cam-canvas", "cam-rear-canvas", "cam-right-canvas", "cam-rear-right-canvas"):
        assert 'id="%s"' % marker in main, marker
    assert "C930e" in main and "C930e" in index
    for marker in ("CAM-FL", "CAM-RL", "CAM-FR", "CAM-RR"):
        assert marker in index, marker
    script = (ROOT / "slam_gui" / "main.js").read_text(encoding="utf-8")
    poller = (ROOT / "slam_gui" / "camera_frames.js").read_text(encoding="utf-8")
    assert "backendCams" in script and "api/camera/' + camera.id + '/frame" in poller
    assert main.index('src="camera_frames.js"') < main.index('src="main.js"')


def test_bounded_camera_frame_polling():
    node = shutil.which("node")
    if not node:
        pytest.skip("node is needed for browser-script tests")
    result = subprocess.run([node, str(ROOT / "tests/js/camera_frames.js")],
                            capture_output=True, text=True, cwd=ROOT, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "bounded camera frame polling: OK" in result.stdout


def test_map_and_route_pages_serve_calibration_form(kiosk_client, client, operator):
    from conftest import login
    login(client)
    for web in (kiosk_client, client):
        for page_path in ("/map", "/route"):
            page = web.get(page_path).text
            for marker in ('id="explore-blockers"', 'id="explore-nav"',
                           'id="explore-cal-form"', 'id="explore-cal-fields"',
                           'id="explore-cal-state"'):
                assert marker in page, (page_path, marker)
