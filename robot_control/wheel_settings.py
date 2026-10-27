"""Manual wheel-to-USB assignments. Changes take effect after restart."""
from __future__ import annotations

import glob
import os
import re
import secrets
import tempfile
from pathlib import Path

from .config import PROJECT_ROOT
from .master_port import reserved_ports

MODULES = {"FL": "Переднее левое", "FR": "Переднее правое",
           "RL": "Заднее левое", "RR": "Заднее правое"}
ENV_KEYS = {module: f"RC_WHEEL_{module}_PORT" for module in MODULES}


def current_settings():
    return {module: os.environ.get(key, "").strip() for module, key in ENV_KEYS.items()}


def env_file_path():
    return PROJECT_ROOT / ".env"


def settings_payload():
    from .lidar import read_env_file
    from .master_port import serial_ports
    saved = read_env_file(env_file_path())
    settings = {m: saved.get(k, current_settings()[m]) for m, k in ENV_KEYS.items()}
    blocked = reserved_ports()
    # Тот же список, что видит остальная программа: /dev/serial/by-id/… тоже
    # годится — такие имена не меняются после переподключения USB. Раньше
    # панель показывала только /dev/ttyUSB* и /dev/ttyACM*, и на машине, где
    # кабели видны лишь как by-id, выбрать порт было не из чего.
    paths = sorted(set(serial_ports()
                       + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
                       + [os.path.realpath(p) for p in settings.values() if p]))
    # Show kernel port names, not a guessed board identity or preferred alias.
    settings = {m: os.path.realpath(p) if p else "" for m, p in settings.items()}
    return {"ok": True, "settings": settings, "restartRequired": True,
            "ports": [{"path": p, "present": os.path.exists(p),
                       "reserved": p in blocked or os.path.realpath(p) in blocked}
                      for p in paths]}


def save_settings(settings, env_path=None):
    if not isinstance(settings, dict) or set(settings) - set(MODULES):
        raise ValueError("Ожидаются порты FL, FR, RL, RR")
    normalized, seen = {}, set()
    blocked = reserved_ports()
    for module in MODULES:
        value = settings.get(module, "")
        if not isinstance(value, str):
            raise ValueError(f"{module}: порт должен быть строкой")
        value = value.strip()
        if value and not re.fullmatch(r"/dev/(?:tty(?:USB|ACM)[0-9]+|serial/(?:by-id|by-path)/[A-Za-z0-9_.:+-]+)", value):
            raise ValueError(f"{module}: выберите USB-порт /dev/ttyUSB…, /dev/ttyACM… или /dev/serial/…")
        if value:
            real = os.path.realpath(value)
            if value in blocked or real in blocked:
                raise ValueError(f"{module}: порт занят лидаром, BMS или исключён из шасси")
            if real in seen:
                raise ValueError("Один USB-порт нельзя назначить двум колёсам")
            seen.add(real)
        normalized[module] = value
    target = Path(env_path) if env_path is not None else env_file_path()
    lines = target.read_text(encoding="utf-8").splitlines() if target.exists() else []
    values = {ENV_KEYS[m]: p for m, p in normalized.items()}
    out, written = [], set()
    for line in lines:
        key = line.strip().split("=", 1)[0].strip()
        if key not in values:
            out.append(line)
        elif key not in written:
            out.append(f"{key}={values[key]}")
            written.add(key)
    out.extend(f"{k}={v}" for k, v in values.items() if k not in written)
    if not target.exists() and not any(line.startswith("RC_SECRET_KEY=") for line in out):
        out.insert(0, f"RC_SECRET_KEY={secrets.token_urlsafe(48)}")
    target.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=".env-wheels-", dir=target.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write("\n".join(out).rstrip() + "\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(name, target.stat().st_mode & 0o777 if target.exists() else 0o600)
        os.replace(name, target)
    finally:
        if os.path.exists(name):
            os.unlink(name)
    # Do not update os.environ: running sources keep their old routing until restart.
    return {"ok": True, "settings": normalized, "restartRequired": True}
