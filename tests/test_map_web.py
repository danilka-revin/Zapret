"""Web map shares the onboard owner and remains behind operator login."""
import io
import json
from urllib.error import HTTPError, URLError

import pytest

from conftest import login
from robot_control import map_peer


class Reply(io.BytesIO):
    def __init__(self, payload, code=200):
        super().__init__(json.dumps(payload).encode())
        self.code = code


def test_map_requires_login(client):
    assert client.get('/map').status_code == 302
    for path in ['', '/settings', '/save', '/reset']:
        method = 'get' if path in ('', '/settings') else 'post'
        assert getattr(client, method)('/api/kiosk_map' + path).status_code in (302, 401)


def test_web_map_navigation_and_assets(client, operator, kiosk_client):
    login(client)
    assert 'id="nav-map"' in client.get('/').text
    assert 'id="console-map-link"' in client.get('/console').text
    page = client.get('/map')
    assert page.status_code == 200
    assert 'id="map-canvas"' in page.text
    assert 'id="map-zoom-in"' in page.text
    for name in ('map.css', 'map_view.js', 'viewport.js', 'camera_view.js'):
        assert client.get('/' + name).status_code == 200
        assert kiosk_client.get('/' + name).status_code == 200


def test_map_requests_reach_same_onboard_service(client, operator, kiosk_client, monkeypatch):
    login(client)
    calls = []

    def forward(req, timeout):
        calls.append((req.full_url, req.method, timeout))
        # Real kiosk handlers, not an independent grid in the web application.
        path = '/api/kiosk_map' + req.full_url.split('/api/kiosk_map', 1)[1]
        response = kiosk_client.open(path, method=req.method, data=req.data,
                                     content_type='application/json')
        return Reply(response.json, response.status_code)

    monkeypatch.setenv('RC_KIOSK_PORT', '18090')
    monkeypatch.setattr(map_peer, 'urlopen', forward)
    kiosk_client.application.extensions['rc_map'].ingest([
        {'a': 0, 'r': 2}, {'a': 1, 'r': 3}])
    response = client.get('/api/kiosk_map')
    assert response.status_code == 200
    assert response.json == kiosk_client.get('/api/kiosk_map').json
    assert response.headers['Cache-Control'] == 'no-store'
    settings = client.post('/api/kiosk_map/settings', json={'settings': {'max_range': 6}})
    assert settings.json['settings']['max_range'] == 6
    assert kiosk_client.get('/api/kiosk_map/settings').json['settings']['max_range'] == 6
    assert client.post('/api/kiosk_map/save', json={}).json['ok']
    assert client.post('/api/kiosk_map/reset', json={}).json['ok']
    assert kiosk_client.get('/api/kiosk_map').json['scans'] == 0
    assert all(url.startswith('http://127.0.0.1:18090/api/kiosk_map') for url, _, _ in calls)
    assert [method for _, method, _ in calls] == ['GET', 'POST', 'POST', 'POST']


@pytest.mark.parametrize('reason', ['offline', 'html', 'oversized', 'invalid'])
def test_map_failure_is_reported_without_creating_another_map(client, operator, monkeypatch, reason):
    login(client)

    def failed(req, timeout):
        if reason == 'offline':
            raise URLError('refused')
        if reason == 'oversized':
            return io.BytesIO(b' ' * (map_peer.MAX_REPLY_BYTES + 1))
        return io.BytesIO(b'<html>login</html>' if reason == 'html' else b'[]')

    monkeypatch.setattr(map_peer, 'urlopen', failed)
    response = client.get('/api/kiosk_map')
    assert response.status_code == 503
    assert response.json['ok'] is False
    assert 'бортов' in response.json['error']
    assert 'rc_map' not in client.application.extensions


def test_map_preserves_onboard_save_error(client, operator, monkeypatch):
    login(client)

    def failed(req, timeout):
        raise HTTPError(req.full_url, 500, 'disk full', {},
                        io.BytesIO(b'{"ok":false,"error":"disk full"}'))

    monkeypatch.setattr(map_peer, 'urlopen', failed)
    response = client.post('/api/kiosk_map/save', json={})
    assert response.status_code == 500
    assert response.json == {'ok': False, 'error': 'disk full'}


def test_map_rejects_nonobject_settings(client, operator):
    login(client)
    assert client.post('/api/kiosk_map/settings', json=[]).status_code == 400


def test_onboard_map_is_a_standalone_page_without_login(kiosk_client):
    # The map can be reached without a successful robot.js boot or a swipe.
    home = kiosk_client.get('/kiosk').text
    assert 'id="k-map-open" class="k-ctrl-btn" href="/map"' in home
    assert 'id="swipe-map-open"' in home
    response = kiosk_client.get('/map')
    assert response.status_code == 200
    assert 'id="map-canvas"' in response.text
    assert 'robot.js' not in response.text
    for name in ('map.css', 'map_view.js', 'viewport.js', 'slam_auth.js'):
        assert kiosk_client.get('/' + name).status_code == 200
    assert kiosk_client.get('/api/kiosk_map').json['ok'] is True
