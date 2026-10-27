"""Forward web map/navigation operations to the onboard owner."""
from __future__ import annotations

import json
from http.client import HTTPException
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from flask import Response, jsonify, request

MAX_REPLY_BYTES = 8 * 1024 * 1024
MAX_NAV_REPLY_BYTES = 2 * 1024 * 1024


def _forward_json(origin: str, path: str, *, methods: set[str], limit: int,
                  timeout: float = 4.0, control_header: str = "", extra_headers=None):
    if request.method not in methods:
        return jsonify(ok=False, error="Метод не поддерживается"), 405
    body = None
    if request.method == "POST":
        payload = request.get_json(silent=True)
        if payload is not None and not isinstance(payload, dict):
            return jsonify(ok=False, error="Ожидается JSON-объект"), 400
        body = json.dumps(payload or {}).encode("utf-8")
    headers = {"Content-Type": "application/json", "Accept": "application/json"}
    # Не прокидываем произвольные заголовки браузера на борт. Только этот
    # постоянный маркер команды распознаёт операторская и бортовая страницы.
    if control_header and request.headers.get("X-Robot-Control") == control_header:
        headers["X-Robot-Control"] = control_header
    headers.update(extra_headers or {})
    upstream = Request(origin + path, data=body, headers=headers, method=request.method)
    try:
        try:
            reply = urlopen(upstream, timeout=timeout)
        except HTTPError as exc:
            reply = exc
        with reply:
            raw = reply.read(limit + 1)
            if len(raw) > limit:
                raise ValueError("reply too large")
            payload = json.loads(raw)
            if not isinstance(payload, dict) or "ok" not in payload:
                raise ValueError("invalid JSON response")
            return Response(raw, status=reply.code, mimetype="application/json",
                            headers={"Cache-Control": "no-store"})
    except (OSError, URLError, ValueError, HTTPException):
        return jsonify(ok=False, error="Нет связи с бортовым сервером. Проверьте, что киоск запущен."), 503


def proxy_map(origin: str, suffix: str):
    """Only fixed internal routes; neither a URL nor a path comes from the browser."""
    if suffix not in {"", "/settings", "/reset", "/save"}:
        return jsonify(ok=False, error="Неизвестная операция карты"), 404
    methods = {"": {"GET"}, "/settings": {"GET", "POST"},
               "/reset": {"POST"}, "/save": {"POST"}}[suffix]
    return _forward_json(origin, "/api/kiosk_map" + suffix,
                         methods=methods, limit=MAX_REPLY_BYTES)


def proxy_navigation(origin: str, suffix: str):
    """Прокси только фиксированных маршрутов навигации на бортовой сервер."""
    routes = {
        "/status": ({"GET"}, ""),
        "/plan": ({"POST"}, ""),
        "/start": ({"POST"}, "navigation"),
        "/cancel": ({"POST"}, "navigation"),
        "/estop": ({"POST"}, "navigation"),
    }
    if suffix not in routes:
        return jsonify(ok=False, error="Неизвестная операция навигации"), 404
    methods, marker = routes[suffix]
    return _forward_json(origin, "/api/navigation" + suffix,
                         methods=methods, limit=MAX_NAV_REPLY_BYTES,
                         timeout=8.0, control_header=marker)


def proxy_estop(origin: str, release: bool = False):
    """E-STOP и PIN-снятие для read-only master-peer веб-пульта."""
    suffix = "/release" if release else ""
    return _forward_json(origin, "/api/estop" + suffix,
                         methods={"POST"}, limit=MAX_NAV_REPLY_BYTES,
                         timeout=4.0, control_header="drive")


# Routes are fixed: the browser cannot pick an upstream URL or serial port.
CONTROL_ROUTES = {
    'state': ('/api/kiosk_state', {'GET'}, ''),
    'logs': ('/api/kiosk_logs', {'GET'}, ''),
    'battery': ('/api/battery', {'GET'}, ''),
    'lock': ('/api/kiosk_lock', {'GET'}, ''),
    'lock/open': ('/api/kiosk_lock/open', {'POST'}, 'steering'),
    'lock/close': ('/api/kiosk_lock/close', {'POST'}, 'steering'),
    'steer': ('/api/kiosk_steer', {'POST'}, 'steering'),
    'zero': ('/api/kiosk_zero', {'POST'}, 'steering'),
    'calibrate': ('/api/kiosk_calibrate', {'POST'}, 'steering'),
    'pot_mode': ('/api/kiosk_pot_mode', {'POST'}, 'steering'),
    # Настройка привода тяги (серво-ШИМ: нейтраль/размах, тахометр, тормоз, стоп, предел).
    'drive_config': ('/api/kiosk_drive_config', {'POST'}, 'steering'),
    # Манёвры тяги: «метр вперёд», «90° на месте» и их настройка.
    'maneuver': ('/api/kiosk_maneuver', {'POST'}, 'steering'),
    'maneuver_config': ('/api/kiosk_maneuver_config', {'POST'}, 'steering'),
    'drive': ('/api/kiosk_drive', {'POST'}, 'drive'),
    'estop': ('/api/estop', {'POST'}, 'drive'),
    'estop/release': ('/api/estop/release', {'POST'}, 'drive'),
    'wheels/settings': ('/api/wheels/settings', {'GET', 'POST'}, 'wheel-settings'),
    # Прошивка плат колёс кнопкой с бортового экрана: состояние и запуск.
    # arduino-cli не нужен — сборка avr-gcc, заливка avrdude (см. avr_upload.py).
    'firmware_flash': ('/api/kiosk_firmware_flash', {'GET', 'POST'}, 'firmware-flash'),
}


def proxy_control(origin: str, operation: str):
    route = CONTROL_ROUTES.get(operation)
    if route is None:
        return jsonify(ok=False, error='Неизвестная команда управления'), 404
    path, methods, marker = route
    extra = None
    if operation == 'logs':
        from .eventlog import clamp_limit
        path += '?limit=' + str(clamp_limit(request.args.get('limit')))
    if operation == 'drive':
        import time
        # Web and onboard servers run on the robot. Do not execute a heartbeat
        # that sat in a server/network queue after the operator released it.
        extra = {'X-Robot-Command-Deadline': str(time.time() + 0.5)}
    return _forward_json(origin, path, methods=methods, limit=MAX_NAV_REPLY_BYTES,
                         timeout=0.6 if operation == 'drive' else 1.0,
                         control_header=marker, extra_headers=extra)


def proxy_battery_report(origin: str):
    """Build a read-only CSV from the same onboard snapshot as the battery tab."""
    from .battery import report_response
    response = proxy_control(origin, 'battery')
    if isinstance(response, tuple) or response.status_code != 200:
        return response
    payload = response.get_json(silent=True) or {}
    snapshot = payload.get('battery')
    if not isinstance(snapshot, dict):
        return jsonify(ok=False, error='Бортовой сервер не вернул данные АКБ'), 503
    try:
        return report_response(snapshot)
    except (KeyError, TypeError, ValueError):
        return jsonify(ok=False, error='Неполный ответ BMS от бортового сервера'), 503


def proxy_connections(origin: str):
    """Inventory belongs to the onboard computer, not the browser or web peer."""
    return _forward_json(origin, '/api/connections', methods={'GET'},
                         limit=MAX_NAV_REPLY_BYTES, timeout=1.0)
