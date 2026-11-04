"""Shared calibration UI: static routes, components and optional Chromium.

The browser scenario injects synthetic telemetry and intercepts every command;
no physical robot or serial device is exercised.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import threading
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
NODE = shutil.which('node')


def test_calibration_styles_and_markup_are_served_on_both_surfaces(kiosk_client, client, operator):
    kiosk_css = kiosk_client.get('/calibration.css')
    web_css = client.get('/calibration.css')
    assert kiosk_css.status_code == web_css.status_code == 200
    assert kiosk_css.mimetype == web_css.mimetype == 'text/css'
    assert kiosk_css.data == web_css.data
    assert b'grid-template-areas' in kiosk_css.data

    # /controls remains authenticated; making the shared stylesheet public does
    # not make the operator screen or its commands public.
    assert client.get('/controls').status_code == 302
    with client.session_transaction() as session:
        session['operator'] = 'operator'
        session['role'] = 'operator'
    kiosk = kiosk_client.get('/kiosk').get_data(as_text=True)
    web = client.get('/controls').get_data(as_text=True)
    for page in (kiosk, web):
        assert '<link rel="stylesheet" href="calibration.css"' in page
        assert 'id="steer-calibration"' in page
        assert 'id="steer-calib-all"' in page and 'id="full-zero-open"' in page
        assert 'calibration-safety' in page
        assert page.index('calibration-safety') < page.index('id="steer-calib-all"')
    for path in ('/', '/console'):
        assert 'href="calibration.css"' in client.get(path).get_data(as_text=True)


@pytest.mark.skipif(NODE is None, reason='node needed for the potentiometer component test')
def test_pot_mode_component_contract():
    result = subprocess.run([NODE, str(ROOT / 'tests/js/pot_mode.js')], cwd=ROOT,
                            capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'confirmation and telemetry passed' in result.stdout


@pytest.mark.skipif(not os.environ.get('ZAPRET_BROWSER_NODE_MODULES') or NODE is None,
                    reason='optional Chromium/playwright-core not installed')
def test_calibration_onboard_and_web_in_browser(kiosk_client, app, operator, monkeypatch):
    """Real pages/transport with fake telemetry, no actual motion commands."""
    from werkzeug.serving import WSGIRequestHandler, make_server

    class Quiet(WSGIRequestHandler):
        def log_request(self, *args, **kwargs):
            pass

    onboard = make_server('127.0.0.1', 0, kiosk_client.application,
                          threaded=True, request_handler=Quiet)
    monkeypatch.setenv('RC_SLAM_PEER_URL', f'http://127.0.0.1:{onboard.server_port}/api/kiosk_state')
    remote = make_server('127.0.0.1', 0, app, threaded=True, request_handler=Quiet)
    threads = [threading.Thread(target=server.serve_forever, daemon=True) for server in (onboard, remote)]
    for thread in threads:
        thread.start()
    try:
        result = subprocess.run([NODE, str(ROOT / 'tests/js/calibration_browser.mjs'),
                                 f'http://127.0.0.1:{onboard.server_port}',
                                 f'http://127.0.0.1:{remote.server_port}'],
                                cwd=ROOT, capture_output=True, text=True, timeout=120)
        assert result.returncode == 0, result.stdout + result.stderr
        assert 'calibration layout and simulated controls passed' in result.stdout
    finally:
        remote.shutdown()
        onboard.shutdown()
        for thread in threads:
            thread.join(timeout=3)
