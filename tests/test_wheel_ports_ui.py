"""Wheel-port presentation never guesses an ID or keeps a stale connection."""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_shared_port_presentation():
    node = shutil.which("node")
    if not node:
        pytest.skip("node is required for the UI test")
    result = subprocess.run([node, str(ROOT / "tests/js/wheel_ports.js")],
                            capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stderr
    assert "wheel ports UI passed" in result.stdout


def test_wheel_port_assets_are_served_and_all_cards_have_placeholders(client, kiosk_client):
    assert client.get("/wheel_ports.js").status_code == 200
    assert kiosk_client.get("/wheel_ports.js").status_code == 200
    page = kiosk_client.get("/kiosk").get_data(as_text=True)
    assert page.index('src="wheel_ports.js"') < page.index('src="robot.js"')
    for wheel in ("FL", "FR", "RL", "RR"):
        assert f'id="usb-port-{wheel}"' in page
        assert f'id="steer-usb-port-{wheel}"' in page
