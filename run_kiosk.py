#!/usr/bin/env python3
"""Экран робота (киоск) одной командой.

    ./run_kiosk.py                 # сервер на 8090 + экран в браузере (если есть дисплей)
    ./run_kiosk.py --no-open       # только сервер (для systemd/без дисплея)
    ./run_kiosk.py --source sim    # стенд: данные симуляции

Отдаёт экран киоска: телеметрия и функции доступны без PIN; код запрашивается
только для открытия грузового кейса. Руление и тяга доступны только с
``--source master``. Порт по умолчанию — 8090 (``RC_KIOSK_PORT``).
Подробности — ``--help``.
"""
import os
import signal
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.kiosk_app import main


def _restart_after_update(_signum, _frame):
    """The main web app signals this process after an atomic Git update."""
    entry = os.path.abspath(__file__)
    os.execv(sys.executable, [sys.executable, entry, *sys.argv[1:]])


if hasattr(signal, "SIGHUP"):
    signal.signal(signal.SIGHUP, _restart_after_update)

if __name__ == "__main__":
    sys.exit(main())
