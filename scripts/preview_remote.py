#!/usr/bin/env python3
"""Preview both kiosk (8090) and remote (8080) for Arena."""
import os, sys, threading, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dataclasses import replace
from robot_control.config import Config, PROJECT_ROOT
from robot_control.kiosk_app import create_kiosk_app
from robot_control.app import create_app
from werkzeug.serving import make_server

os.environ.setdefault("RC_DELIVERY_DIR", str(PROJECT_ROOT / "var" / "preview"))
cfg_kiosk = replace(Config.from_env(), testing=True, slam_source="none", port=8090)
cfg_remote = replace(Config.from_env(), testing=True, slam_source="none", port=8080)

kiosk_app = create_kiosk_app(cfg_kiosk)
remote_app = create_app(cfg_remote)

def run(app, host, port, name):
    srv = make_server(host, port, app, threaded=True)
    print(f"{name} http://{host}:{port}/", flush=True)
    srv.serve_forever()

t1 = threading.Thread(target=run, args=(kiosk_app, "0.0.0.0", 8090, "KIOSK"), daemon=True)
t2 = threading.Thread(target=run, args=(remote_app, "0.0.0.0", 8080, "REMOTE"), daemon=True)
t1.start()
t2.start()
print("Both running: REMOTE http://0.0.0.0:8080/console  KIOSK http://0.0.0.0:8090/kiosk", flush=True)
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass
