"""Interactive Kinect view and 3D frame projection logic."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent


@pytest.mark.parametrize('page', ['robot.html', 'index.html', 'scan.html'])
def test_kinect_fov_toggle_is_on_every_3d_overview(page):
    html = (ROOT / 'slam_gui' / page).read_text()
    for token in ('id="kinect-3d-canvas"', 'id="kinect-fov-toggle"', 'kinect_view.js'):
        assert token in html
    assert 'голубая пирамида' in html.lower()
    css = (ROOT / 'slam_gui' / ('robot.css' if page == 'robot.html'
                               else 'scan.css' if page == 'scan.html'
                               else 'styles.css')).read_text()
    assert 'button[hidden]' in css, 'кнопка зоны не должна оставаться в режимах 2D/только Kinect'


def test_kinect_panel_is_included_on_robot_home():
    html = (ROOT / 'slam_gui/robot.html').read_text()
    for token in ('id="sensor-lidar"', 'id="sensor-kinect"', 'id="sensor-combined"',
                  'id="kinect-3d-canvas"', 'id="kinect-zoom-in"',
                  'id="kinect-reset-view"', 'id="kinect-fov-toggle"',
                  'голубая пирамида', 'kinect_view.js'):
        assert token in html


@pytest.mark.parametrize('page', ['robot.html', 'index.html', 'scan.html'])
def test_vision_state_feeds_3d_overview(page):
    html = (ROOT / 'slam_gui' / page).read_text()
    assert 'vision_panel.js' in html
    assert 'kinect_view.js' in html
    assert html.index('vision_panel.js') < html.index('kinect_view.js'), \
        'RCVision должен быть зарегистрирован до подписки 3D-обзора'


@pytest.mark.skipif(not shutil.which('node'), reason='node is required')
def test_kinect_javascript_decodes_frames():
    subprocess.run(['node', str(ROOT / 'tests/js/kinect_view.js')], check=True, timeout=10)
