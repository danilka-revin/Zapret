#!/usr/bin/env python3
"""Локальный предпросмотр экрана доставки без железа и без автоустановки ROS 2.

    .venv/bin/python scripts/delivery_preview.py --host 0.0.0.0 --port 8090

Поднимает то же приложение киоска в testing-режиме с источником `none`, поэтому
лидар/камеры/колёса не открываются и фоновые установки не запускаются. Корень
перенаправляется на `/delivery`. Это инструмент разработки, не боевой запуск.
"""
from __future__ import annotations

import argparse
import os
import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_control.config import PROJECT_ROOT, Config
from robot_control.kiosk_app import create_kiosk_app
from werkzeug.serving import make_server


class RootRedirect:
    def __init__(self, app):
        self.app = app

    def __call__(self, environ, start_response):
        if environ.get("PATH_INFO") == "/":
            start_response("302 Found", [("Location", "/delivery"), ("Cache-Control", "no-store")])
            return [b""]
        return self.app(environ, start_response)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    args = parser.parse_args(argv)
    os.environ.setdefault("RC_DELIVERY_DIR", str(PROJECT_ROOT / "var" / "delivery-preview"))
    cfg = replace(Config.from_env(), testing=True, slam_source="none")
    app = create_kiosk_app(cfg)
    server = make_server(args.host, args.port, RootRedirect(app), threaded=True)
    print(f"Предпросмотр доставки: http://{args.host}:{args.port}/delivery (тяга отключена)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
