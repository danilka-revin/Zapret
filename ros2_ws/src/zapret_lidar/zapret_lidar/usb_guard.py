"""Shared USB classification/lock used by the ROS lidar supervisor.

In automatic wheel mode, no port is probed until the onboard owner has
passively excluded a wheel and completed a negative read-only JBD probe.
The same per-device flock is held while a probe or ROS driver owns the UART.
"""
from __future__ import annotations

import json
import os
import re
import time
from pathlib import Path

try:
    import fcntl
except ImportError:  # pragma: no cover
    fcntl = None

TTL_S = 8.0


def enabled() -> bool:
    return os.environ.get("RC_USB_DISCOVERY_MODE") == "1"


def checked_lidar_ports() -> set[str]:
    directory = Path(os.environ.get("RC_USB_DISCOVERY_DIR") or "data/ports")
    ports: set[str] = set()
    for record in directory.glob("usb-scan-*.json"):
        try:
            data = json.loads(record.read_text("utf-8"))
            pid = int(data["pid"])
            if pid <= 0 or time.time() - float(data["at"]) > TTL_S:
                continue
            os.kill(pid, 0)
            for item in data.get("ports") or []:
                if not isinstance(item, dict) or item.get("bms") is not False:
                    continue
                port = item.get("port")
                if not isinstance(port, str) or not port:
                    continue
                try:
                    st = os.stat(port)
                except OSError:
                    continue
                if (item.get("device") == os.path.realpath(port) and
                        item.get("fingerprint") == [st.st_dev, st.st_ino, st.st_rdev, st.st_ctime_ns]):
                    ports.add(os.path.realpath(port))
        except (OSError, ValueError, KeyError, TypeError):
            continue
    return ports


def permitted(port: str) -> bool:
    return not enabled() or os.path.realpath(port) in checked_lidar_ports()


def claim(port: str):
    """-1 in standalone mode; None on contention or any lock failure."""
    if not enabled():
        return -1
    if not port or fcntl is None:
        return None
    real = os.path.realpath(port)
    name = re.sub(r"[^A-Za-z0-9_.-]", "_", real.strip("/").replace("/", "-")) or "port"
    directory = Path(os.environ.get("RC_USB_LOCK_DIR") or "data")
    try:
        directory.mkdir(parents=True, exist_ok=True)
        fd = os.open(str(directory / ("lidar-%s.lock" % name)), os.O_RDWR | os.O_CREAT, 0o644)
    except OSError:
        return None
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        os.close(fd)
        return None
    return fd


def release(fd) -> None:
    if fd is None or fd == -1:
        return
    try:
        fcntl.flock(fd, fcntl.LOCK_UN)
    except OSError:
        pass
    try:
        os.close(fd)
    except OSError:
        pass
