"""Interactive Kinect view and 3D frame projection logic."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent


def test_kinect_panel_is_included_on_robot_home():
    html = (ROOT / 'slam_gui/robot.html').read_text()
    for token in ('id="sensor-lidar"', 'id="sensor-kinect"', 'id="kinect-3d-canvas"',
                  'id="kinect-zoom-in"', 'id="kinect-reset-view"', 'kinect_view.js'):
        assert token in html


@pytest.mark.skipif(not shutil.which('node'), reason='node is required')
def test_kinect_javascript_decodes_frames():
    subprocess.run(['node', str(ROOT / 'tests/js/kinect_view.js')], check=True, timeout=10)
