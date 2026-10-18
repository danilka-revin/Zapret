"""Forward web map operations to the onboard owner, never maintain a second grid."""
from __future__ import annotations

import json
from http.client import HTTPException
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from flask import Response, jsonify, request

MAX_REPLY_BYTES = 8 * 1024 * 1024


def proxy_map(origin: str, suffix: str):
    """Only fixed internal routes; neither a URL nor a path comes from the browser."""
    if suffix not in {"", "/settings", "/reset", "/save"}:
        return jsonify(ok=False, error="Неизвестная операция карты"), 404
    body = None
    if request.method == "POST":
        payload = request.get_json(silent=True)
        if payload is not None and not isinstance(payload, dict):
            return jsonify(ok=False, error="Ожидается JSON-объект"), 400
        body = json.dumps(payload or {}).encode("utf-8")
    upstream = Request(origin + "/api/kiosk_map" + suffix, data=body,
                       headers={"Content-Type": "application/json", "Accept": "application/json"},
                       method=request.method)
    try:
        try:
            reply = urlopen(upstream, timeout=4)
        except HTTPError as exc:
            reply = exc
        with reply:
            raw = reply.read(MAX_REPLY_BYTES + 1)
            if len(raw) > MAX_REPLY_BYTES:
                raise ValueError("map response too large")
            payload = json.loads(raw)
            if not isinstance(payload, dict) or "ok" not in payload:
                raise ValueError("invalid map response")
            return Response(raw, status=reply.code, mimetype="application/json",
                            headers={"Cache-Control": "no-store"})
    except (OSError, URLError, ValueError, HTTPException):
        return jsonify(ok=False, error="Нет связи с бортовым сервером карты. "
                       "Проверьте, что бортовой экран запущен."), 503
