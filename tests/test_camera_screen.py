"""Camera UI does not need hardware to test zoom, reconnects or teardown."""
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_camera_lifecycle_and_viewport_gestures():
    node = shutil.which('node')
    if not node:
        pytest.skip('node is needed for browser-script tests')
    for test_file in ('camera_view.js', 'camera_stitch.js'):
        result = subprocess.run([node, str(ROOT / 'tests/js' / test_file)],
                                capture_output=True, text=True, cwd=ROOT, timeout=30)
        assert result.returncode == 0, result.stdout + result.stderr


def test_main_camera_frame_poller_is_served(client):
    response = client.get('/camera_frames.js')
    assert response.status_code == 200
    assert 'MAX_IN_FLIGHT = 2' in response.text
    assert "'/frame?t='" in response.text


def test_camera_button_and_controls_are_served(kiosk_client):
    page = kiosk_client.get('/kiosk').text
    assert 'id="k-cameras-open"' in page
    assert 'camera_view.js' in page and 'camera_stitch.js' in page and 'viewport.js' in page
    assert 'id="cam-stitch-toggle"' in page and 'id="cam-stitch-canvas"' in page
    assert 'id="cam-stitch-left-mirror"' in page and 'id="cam-stitch-right-crop"' in page
    assert 'cam-stitch-panel' in page and 'cam-stitch-reset' in page
    stitch_script = kiosk_client.get('/camera_stitch.js')
    assert stitch_script.status_code == 200
    assert 'robot-camera-stitch-v1' in stitch_script.text
    for n in (1, 2):
        for control in ('in', 'out', 'reset', 'expand', 'zoom'):
            assert f'id="cam{n}-{control}"' in page
    script = kiosk_client.get('/camera_view.js').text
    assert 'getUserMedia' not in script
    assert "'/frame?t='" in script


def test_camera_panel_is_laid_out_front_first(kiosk_client):
    """Передние камеры сверху, задние снизу; слева левые, справа правые."""
    page = kiosk_client.get("/kiosk").text
    sides = re.findall(r'class="cam-card" data-camera="([A-Z]{2})"', page)
    assert sides == ["FL", "FR", "RL", "RR"], "порядок блоков: передние сверху"

    # Место в сетке 2×2 задано явно: раскладка не зависит от порядка разметки.
    css = kiosk_client.get("/robot.css").text
    places = (("FL", "1 / 1"), ("FR", "1 / 2"), ("RL", "2 / 1"), ("RR", "2 / 2"))
    for side, area in places:
        rule = '.cam-stack > .cam-card[data-camera="%s"] { grid-area: %s; }'
        rule %= (side, area)
        assert rule in css, rule
    # Развёрнутая камера занимает весь блок, а не свой угол.
    assert ".camera-expanded .cam-stack > .cam-card[data-camera] { grid-area: auto; }" in css

    # Подписи говорят, где камера, а не какой у неё номер: номера в портах
    # (0…3) не совпадают с порядком на экране и только путали.
    labels = ("Передняя левая", "Передняя правая", "Задняя левая", "Задняя правая")
    for label in labels:
        assert "<h3>%s</h3>" % label in page, label
    assert "кормовая" not in page


def test_web_console_camera_feeds_are_ordered_the_same_way():
    """Пульт 8080 показывает камеры в том же порядке: передние сверху."""
    root = Path(__file__).resolve().parent.parent / "slam_gui"
    page = (root / "index.html").read_text(encoding="utf-8")
    assert re.findall(r"CAM-([A-Z]{2})", page) == ["FL", "FR", "RL", "RR"]
    # .cams — две колонки, поэтому строка 1 = передние, строка 2 = задние.
    styles = (root / "styles.css").read_text(encoding="utf-8")
    assert ".cams{display:grid;grid-template-columns:1fr 1fr" in styles
