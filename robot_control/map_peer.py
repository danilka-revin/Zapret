"""Forward web map/navigation operations to the onboard owner."""
from __future__ import annotations

import json
import re
from http.client import HTTPException
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

from flask import Response, jsonify, request

MAX_REPLY_BYTES = 8 * 1024 * 1024
MAX_NAV_REPLY_BYTES = 2 * 1024 * 1024
MAX_LANDMARK_BYTES = 2 * 1024 * 1024
LANDMARK_NAME_RE = re.compile(r"step\d{3}-cam[0-3]\.jpg")


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
        "/resume": ({"POST"}, "navigation"),
        "/cancel": ({"POST"}, "navigation"),
        "/estop": ({"POST"}, "navigation"),
    }
    if suffix not in routes:
        return jsonify(ok=False, error="Неизвестная операция навигации"), 404
    methods, marker = routes[suffix]
    return _forward_json(origin, "/api/navigation" + suffix,
                         methods=methods, limit=MAX_NAV_REPLY_BYTES,
                         timeout=8.0, control_header=marker)


def proxy_exploration(origin: str, suffix: str):
    """Прокси фиксированных маршрутов автообследования на бортовой сервер."""
    routes = {
        "/status": ({"GET"}, ""),
        "/start": ({"POST"}, "exploration"),
        "/stop": ({"POST"}, "exploration"),
        "/landmarks": ({"GET"}, ""),
        "/landmarks/clear": ({"POST"}, "exploration"),
    }
    if suffix not in routes:
        return jsonify(ok=False, error="Неизвестная операция обследования"), 404
    methods, marker = routes[suffix]
    return _forward_json(origin, "/api/exploration" + suffix,
                         methods=methods, limit=MAX_NAV_REPLY_BYTES,
                         timeout=8.0, control_header=marker)


def proxy_landmark_file(origin: str, name: str):
    """Отдать JPEG визуального ориентира с борта (не JSON)."""
    if not LANDMARK_NAME_RE.fullmatch(name):
        return jsonify(ok=False, error="Неверное имя файла ориентира"), 400
    request_obj = Request(origin + "/api/exploration/landmark/" + quote(name),
                          method="GET", headers={"Accept": "image/jpeg"})
    try:
        with urlopen(request_obj, timeout=4.0) as reply:
            raw = reply.read(MAX_LANDMARK_BYTES + 1)
            if len(raw) > MAX_LANDMARK_BYTES:
                raise ValueError("landmark too large")
            return Response(raw, status=reply.status, mimetype="image/jpeg",
                            headers={"Cache-Control": "no-store"})
    except (OSError, URLError, ValueError, HTTPException):
        return jsonify(ok=False, error="Ориентир недоступен на бортовом сервере"), 503


def proxy_sensors(origin: str, suffix: str):
    """Только чтение геометрии сенсоров и статуса Kinect с борта."""
    routes = {
        "/layout": ("/api/sensors/layout", {"GET"}),
        "/kinect": ("/api/kinect/status", {"GET"}),
    }
    if suffix not in routes:
        return jsonify(ok=False, error="Неизвестная операция сенсоров"), 404
    path, methods = routes[suffix]
    return _forward_json(origin, path, methods=methods, limit=MAX_NAV_REPLY_BYTES,
                         timeout=4.0)


def proxy_calibration(origin: str, suffix: str):
    """Чтение и сохранение замеров геометрии на бортовом сервере."""
    routes = {
        "/geometry": ({"GET"}, ""),
        "/geometry/save": ({"POST"}, "calibration"),
    }
    if suffix not in routes:
        return jsonify(ok=False, error="Неизвестная операция калибровки"), 404
    methods, marker = routes[suffix]
    return _forward_json(origin, "/api/calibration/geometry",
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
    'wheel_state': ('/api/kiosk_wheel_state', {'GET'}, ''),
    'logs': ('/api/kiosk_logs', {'GET'}, ''),
    'battery': ('/api/battery', {'GET'}, ''),
    'printer': ('/api/printer', {'GET'}, ''),
    'printer/preview': ('/api/printer/preview', {'GET'}, ''),
    'printer/config': ('/api/printer/config', {'POST'}, 'printer'),
    'printer/print': ('/api/printer/print', {'POST'}, 'printer'),
    'printer/test': ('/api/printer/test', {'POST'}, 'printer'),
    'printer/cancel': ('/api/printer/cancel', {'POST'}, 'printer'),
    'lock': ('/api/kiosk_lock', {'GET'}, ''),
    'lock/open': ('/api/kiosk_lock/open', {'POST'}, 'steering'),
    'lock/close': ('/api/kiosk_lock/close', {'POST'}, 'steering'),
    'steer': ('/api/kiosk_steer', {'POST'}, 'steering'),
    'zero': ('/api/kiosk_zero', {'POST'}, 'steering'),
    'calibrate': ('/api/kiosk_calibrate', {'POST'}, 'steering'),
    'full_zero': ('/api/kiosk_full_zero', {'POST'}, 'steering'),
    'pot_mode': ('/api/kiosk_pot_mode', {'POST'}, 'steering'),
    # Активное удержание угла (ph 0/1) — та же команда руления.
    'pot_hold': ('/api/kiosk_pot_hold', {'POST'}, 'steering'),
    # Манёвры тяги: «метр вперёд», «90° на месте» и их настройка.
    'maneuver': ('/api/kiosk_maneuver', {'POST'}, 'steering'),
    'maneuver_config': ('/api/kiosk_maneuver_config', {'POST'}, 'steering'),
    'keyboard': ('/api/kiosk_keyboard', {'POST'}, 'drive'),
    'neutral': ('/api/kiosk_neutral', {'POST'}, 'drive'),
    'drive': ('/api/kiosk_drive', {'POST'}, 'drive'),
    'estop': ('/api/estop', {'POST'}, 'drive'),
    'estop/release': ('/api/estop/release', {'POST'}, 'drive'),
    'wheels/settings': ('/api/wheels/settings', {'GET', 'POST'}, 'wheel-settings'),
}


def proxy_control(origin: str, operation: str):
    route = CONTROL_ROUTES.get(operation)
    if route is None:
        return jsonify(ok=False, error='Неизвестная команда управления'), 404
    path, methods, marker = route
    extra = None
    if operation == 'neutral' and request.method == 'POST':
        # The web proxy may be updated before the onboard server. Never
        # forward a freewheel command to an older server that still accepts it.
        payload = request.get_json(silent=True)
        if not isinstance(payload, dict):
            return jsonify(ok=False, reason='payload'), 400
        action = payload.get('action')
        if isinstance(action, str) and action in {'begin', 'heartbeat'}:
            return jsonify(ok=False, reason='neutral_disabled'), 410
        if payload != {'action': 'restore'}:
            return jsonify(ok=False, reason='payload'), 400
    if operation == 'logs':
        from .eventlog import clamp_limit
        path += '?limit=' + str(clamp_limit(request.args.get('limit')))
    elif operation == 'printer/preview':
        level = request.args.get('level', 'medium')
        if level not in {'short', 'medium', 'full'}:
            return jsonify(ok=False, error='Неверный вид отчёта'), 400
        path += '?level=' + level
    if operation == 'drive':
        import time
        # Web and onboard servers run on the robot. Do not execute a heartbeat
        # that sat in a server/network queue after the operator released it.
        extra = {'X-Robot-Command-Deadline': str(time.time() + 0.5)}
    elif operation in {'keyboard', 'neutral'}:
        import math
        import time
        # Only the known deadline is forwarded, never arbitrary client headers.
        # Preserve the browser's earlier deadline and cap it at 0.5 s on board.
        deadline = request.headers.get('X-Robot-Command-Deadline')
        if deadline is not None:
            try:
                value = float(deadline)
                now = time.time()
                if not math.isfinite(value) or value > now + 0.75:
                    raise ValueError('invalid deadline or clock skew')
                deadline = str(min(value, now + 0.5))
            except ValueError:
                deadline = 'invalid'
            extra = {'X-Robot-Command-Deadline': deadline}
    return _forward_json(origin, path, methods=methods, limit=MAX_NAV_REPLY_BYTES,
                         timeout=(0.6 if operation in {'drive', 'keyboard', 'neutral'} else
                                  4.0 if operation.startswith('printer') else 1.0),
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


def proxy_devices(origin: str, *, operation: str = 'snapshot'):
    """Only the fixed onboard inventory endpoints, with a small POST body."""
    paths = {'snapshot': '/api/devices', 'hint': '/api/devices/hint',
             'ports': '/api/devices/ports', 'save-ports': '/api/devices/ports',
             'confirm': '/api/devices/camera/confirm'}
    if operation not in paths:
        return jsonify(ok=False, error='Неизвестная операция'), 404
    mutate = operation != 'snapshot'
    limit = 8192 if operation == 'save-ports' else 4096
    if mutate:
        if request.content_length is not None and request.content_length > limit:
            return jsonify(ok=False, error="Слишком большой запрос"), 413
        request.max_content_length = limit
    path = paths[operation]
    if not mutate and request.args.get('refresh') == '1':
        path += '?refresh=1'
    return _forward_json(origin, path, methods={'POST'} if mutate else {'GET'},
                         limit=MAX_NAV_REPLY_BYTES, timeout=5.0,
                         control_header='devices' if mutate else '')


def proxy_camera_json(origin: str, operation: str):
    routes = {'list': ('/api/cameras', {'GET'}),
              'settings': ('/api/camera/settings', {'GET', 'POST'}),
              'rescan': ('/api/camera/rescan', {'POST'})}
    if operation not in routes:
        return jsonify(ok=False, error='Неизвестная операция камер'), 404
    path, methods = routes[operation]
    return _forward_json(origin, path, methods=methods, limit=MAX_NAV_REPLY_BYTES, timeout=5.0)


def _camera_frame(origin: str, cam_id: int) -> bytes | None:
    if cam_id not in range(4):
        return None
    try:
        with urlopen(Request(origin + f'/api/camera/{cam_id}/frame',
                             headers={'Accept': 'image/jpeg'}), timeout=2.0) as reply:
            if reply.code != 200 or not str(reply.headers.get('Content-Type', '')).lower().startswith('image/jpeg'):
                return None
            raw = reply.read(MAX_LANDMARK_BYTES + 1)
            return raw if raw and len(raw) <= MAX_LANDMARK_BYTES else None
    except (OSError, URLError, ValueError, HTTPException):
        return None


def proxy_camera_frame(origin: str, cam_id: int):
    """Bounded still frame only; a peer never opens V4L2 on the computer."""
    if cam_id not in range(4):
        return Response(status=404)
    if request.method == 'GET':
        frame = _camera_frame(origin, cam_id)
        return (Response(frame, mimetype='image/jpeg', headers={'Cache-Control': 'no-store'})
                if frame else Response(status=404, headers={'Cache-Control': 'no-store'}))
    if request.method != 'POST':
        return Response(status=405)
    if request.content_length is not None and request.content_length > MAX_LANDMARK_BYTES:
        return jsonify(ok=False, error='Слишком большой кадр'), 413
    raw = request.stream.read(MAX_LANDMARK_BYTES + 1)
    if not raw or len(raw) > MAX_LANDMARK_BYTES:
        return jsonify(ok=False, error='Пустой или слишком большой кадр'), 400
    try:
        upstream = Request(origin + f'/api/camera/{cam_id}/frame', data=raw,
                           headers={'Content-Type': 'image/jpeg'}, method='POST')
        with urlopen(upstream, timeout=2.0) as reply:
            body = reply.read(4097)
            parsed = json.loads(body)
            if len(body) > 4096 or not isinstance(parsed, dict) or 'ok' not in parsed:
                raise ValueError('invalid reply')
            return Response(body, status=reply.code, mimetype='application/json',
                            headers={'Cache-Control': 'no-store'})
    except (OSError, URLError, ValueError, HTTPException):
        return jsonify(ok=False, error='Камера на борту недоступна'), 503


def proxy_camera_stream(origin: str, cam_id: int):
    """Compatibility MJPEG generated from bounded owner frames, no V4L2 open."""
    if cam_id not in range(4):
        return Response(status=404)
    import time
    def generate():
        while True:
            frame = _camera_frame(origin, cam_id)
            if frame:
                yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
            time.sleep(0.1 if frame else 1.0)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame',
                    headers={'Cache-Control': 'no-store'})


def proxy_system(origin: str):
    """Temperature and load of the onboard computer, read-only, cached on board."""
    return _forward_json(origin, '/api/pc_state', methods={'GET'},
                         limit=MAX_REPLY_BYTES, timeout=2.0)
