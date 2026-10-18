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
                  timeout: float = 4.0, control_header: str = ""):
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
