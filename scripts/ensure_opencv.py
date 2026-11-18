#!/usr/bin/env python3
"""Проверить/восстановить OpenCV в .venv борта, не открывая камеры и не вызывая sudo."""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot_control.config import dotenv_items
from robot_control.opencv_runtime import REPAIR_COMMAND
from robot_control.sensor_layout import CAMERA_ENV_KEYS


def needs_camera(root: Path, dev: Path = Path("/dev")) -> bool:
    # The updater may run in the web process, not the kiosk unit which loaded
    # the writable camera overlay. Read settings as data, never source .env.
    settings = dict(dotenv_items(root / ".env"))
    settings.update(os.environ)
    overlay = settings.get("RC_CAMERA_ENV_FILE") or "data/camera-ports.env"
    settings.update(dotenv_items(root / overlay))
    backend = settings.get("RC_CAMERA_BACKEND", "auto").strip().lower()
    if backend == "ffmpeg":
        return False  # explicit FFmpeg-only capture; manual repair still works for recognition
    return (backend == "opencv" or any(settings.get(k) for keys in CAMERA_ENV_KEYS.values() for k in keys)
            or any(dev.glob("video[0-9]*")))


def ensure(root: Path, *, auto: bool, check: bool) -> int:
    if auto and not needs_camera(root):
        return 0
    python = root / ".venv/bin/python"
    if not python.is_file():
        print("Нет .venv: сначала запустите ./scripts/ensure_deps.sh", file=sys.stderr)
        return 1
    # The updater can inherit ROS PYTHONPATH. Do not let system pip/NumPy
    # satisfy a check or install intended for the isolated project interpreter.
    probe = [str(python), "-I", str(root / "robot_control/opencv_runtime.py")]
    if subprocess.run(probe).returncode == 0:
        return 0
    if check:
        return 1
    offline = os.environ.get("RC_DEPS_OFFLINE") == "1"
    failed = root / ".venv/.opencv-install-failed"
    try:
        recently_failed = 0 <= time.time() - failed.stat().st_mtime < 86400
    except FileNotFoundError:
        recently_failed = False
    if auto and not offline and recently_failed:
        print(f"Недавно поставить OpenCV не удалось. Повторить сейчас: {REPAIR_COMMAND}", flush=True)
        return 1
    args = [str(python), "-I", "-m", "pip", "install", "--disable-pip-version-check", "--no-input",
            "--only-binary=:all:", "--retries", "1", "--timeout", "8", "--find-links", str(root / "vendor")]
    env = os.environ.copy()
    if offline:
        # No remote --find-links from pip.conf or the service environment either.
        args.append("--no-index")
        env = {k: v for k, v in env.items() if not k.startswith("PIP_")}
        env["PIP_CONFIG_FILE"] = os.devnull
    args.extend(["-r", str(root / "requirements-camera.txt")])
    print("Устанавливаю OpenCV/NumPy в .venv" + (" только из vendor/…" if offline else "…"), flush=True)
    try:
        # Normally keep satisfied dependencies. If pip returns success but the
        # .so still won't load, metadata may have survived a damaged install.
        for flags in ([[], ["--force-reinstall"]] if auto else [["--force-reinstall"]]):
            if subprocess.run([*args, *flags], env=env, timeout=180).returncode != 0:
                break
            if subprocess.run(probe).returncode == 0:
                failed.unlink(missing_ok=True)
                return 0
            if not flags:
                print("pip завершился успешно, но импорт не работает; переустанавливаю колёса…", flush=True)
    except subprocess.TimeoutExpired:
        print("Истёк таймаут установки OpenCV.", file=sys.stderr)
    if not offline:
        failed.touch()
    print(f"OpenCV не восстановлен. Проверка: {REPAIR_COMMAND} --check; повтор установки: {REPAIR_COMMAND}",
          file=sys.stderr)
    return 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="только проверить импорт, без изменений")
    mode.add_argument("--auto", action="store_true", help="необязательная установка только для видеовходов борта")
    opts = parser.parse_args()
    sys.exit(ensure(ROOT, auto=opts.auto, check=opts.check))
