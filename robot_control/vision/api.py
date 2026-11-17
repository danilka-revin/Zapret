"""HTTP-маршруты распознавания, общие для веб-пульта и бортового киоска.

Оба Flask-приложения регистрируют один и тот же набор путей на одном
синглтоне :class:`VisionService`. В режиме ``master-peer`` веб-пульт не
открывает свои камеры, а проксирует состояние борта (см. ``map_peer.py``).
"""
from __future__ import annotations

from typing import Any, Callable, Optional

from flask import Response, jsonify, request

from .service import VisionService, get_vision_service


def _json(payload: dict[str, Any]):
    response = jsonify(payload)
    response.headers["Cache-Control"] = "no-store"
    return response


def register_vision_routes(router, *, service: Optional[VisionService] = None,
                           guard: Optional[Callable] = None,
                           peer_origin: Optional[Callable[[], Optional[str]]] = None,
                           csrf_header_ok: Optional[Callable[[], bool]] = None,
                           autostart: bool = True) -> VisionService:
    """Добавить ``/api/vision/*`` к Flask-приложению или Blueprint.

    ``router`` — ``Flask`` или ``Blueprint``; ``guard`` — декоратор авторизации
    пульта; ``peer_origin`` возвращает адрес борта в peer-режиме или ``None``.
    """
    service = service or get_vision_service()
    decorate = guard or (lambda view: view)

    def origin() -> Optional[str]:
        if peer_origin is None:
            return None
        return peer_origin()

    @router.get("/api/vision/state")
    @decorate
    def api_vision_state():
        peer = origin()
        if peer:
            from ..map_peer import proxy_vision_json
            return proxy_vision_json(peer, "state")
        if autostart:
            service.start()
        return _json(service.state())

    @router.get("/api/vision/field")
    @decorate
    def api_vision_field():
        peer = origin()
        if peer:
            from ..map_peer import proxy_vision_json
            return proxy_vision_json(peer, "field")
        payload = service.state()
        return _json({
            "ok": payload["ok"], "backend": payload["backend"],
            "trafficLight": payload["trafficLight"],
            "field": payload["field"], "updatedAt": payload["updatedAt"],
        })

    @router.get("/api/vision/camera/<int:cam_id>/frame")
    @decorate
    def api_vision_frame(cam_id: int):
        peer = origin()
        if peer:
            from ..map_peer import proxy_vision_frame
            return proxy_vision_frame(peer, cam_id)
        jpeg = service.annotated_frame(cam_id)
        if not jpeg:
            return Response(status=404, headers={"Cache-Control": "no-store"})
        return Response(jpeg, mimetype="image/jpeg",
                        headers={"Cache-Control": "no-store"})

    @router.post("/api/vision/camera/<int:cam_id>/frame")
    @decorate
    def api_vision_feed(cam_id: int):
        """Принять кадр от внешнего источника и сразу разметить его."""
        if csrf_header_ok is not None and not csrf_header_ok():
            return jsonify(ok=False, reason="csrf_required"), 400
        if request.content_length is not None and request.content_length > 2_000_000:
            return jsonify(ok=False, reason="frame_too_large"), 413
        raw = request.get_data()
        if not raw:
            return jsonify(ok=False, reason="empty_frame"), 400
        try:
            result = service.process_frame(cam_id, raw)
        except ValueError as exc:
            return jsonify(ok=False, reason=str(exc)), 400
        return _json({"ok": True, "camera": result})

    @router.post("/api/vision/config")
    @decorate
    def api_vision_config():
        """Включить/выключить разметку и подкрутить порог уверенности."""
        if csrf_header_ok is not None and not csrf_header_ok():
            return jsonify(ok=False, reason="csrf_required"), 400
        body = request.get_json(silent=True) or {}
        if not isinstance(body, dict):
            return jsonify(ok=False, reason="bad_json"), 400
        enabled = body.get("enabled")
        confidence = body.get("confidence")
        if enabled is not None and not isinstance(enabled, bool):
            return jsonify(ok=False, reason="bad_enabled"), 400
        if confidence is not None:
            try:
                confidence = float(confidence)
            except (TypeError, ValueError):
                return jsonify(ok=False, reason="bad_confidence"), 400
        snapshot = service.configure(enabled=enabled, confidence=confidence)
        return _json(snapshot)

    @router.get("/api/vision/classes")
    @decorate
    def api_vision_classes():
        from .classes import CLASS_INFO, CLASS_NAMES, TRAFFIC_LIGHT_LABELS
        return _json({
            "ok": True,
            "classes": [{"name": name, **CLASS_INFO[name]} for name in CLASS_NAMES],
            "signals": TRAFFIC_LIGHT_LABELS,
        })

    return service
