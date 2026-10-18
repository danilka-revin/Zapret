"""Shared map/lidar chassis: no synthetic wheel angles or stale live status."""
import shutil
import subprocess
from pathlib import Path

import pytest

from conftest import login


@pytest.mark.skipif(shutil.which("node") is None, reason="node is required")
def test_robot_model_telemetry_and_wheel_transforms():
    root = Path(__file__).resolve().parent.parent
    result = subprocess.run(["node", "tests/js/robot_model.js"], cwd=root,
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr


def test_shared_model_asset_available_on_both_servers(client, operator, kiosk_client):
    login(client)
    for browser in (client, kiosk_client):
        response = browser.get('/robot_model.js')
        assert response.status_code == 200
        assert b'window.RCRobotModel' in response.data
