#!/usr/bin/env python3
"""Помощник обязательного замера RC_NAV_MPS_PER_PERCENT.

Навигация не угадывает скорость: без измеренного коэффициента она откажет
фразой «Нет калибровки скорости». Замер делает человек на ограждённой
площадке (колёса вывешены/робот под наблюдением, E-STOP под рукой):

  1. Подайте одну и ту же команду тяги t=<процент> на несколько секунд.
  2. Измерьте пройденное расстояние (рулеткой) и время (секундомером).
  3. Запустите:

       ./scripts/measure_nav_speed.py --distance 2.0 --seconds 8.0 --percent 4
       ./scripts/measure_nav_speed.py --distance 2.0 --seconds 8.0 --percent 4 --save-env

`--save-env` атомарно пишет RC_NAV_MPS_PER_PERCENT в .env через тот же
механизм, что и «Калибровка геометрии» на /map. После изменения перезапустите
бортовой сервер. Значение — м/с на один процент команды t, НЕ процент Servo.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_control import calibration  # noqa: E402


def compute(distance_m: float, seconds: float, percent: float) -> float:
    """м/с на 1% команды: (пройдено / время) / процент."""
    if not 0 < distance_m <= 1000:
        raise ValueError("расстояние должно быть в пределах 0…1000 м")
    if not 0.1 <= seconds <= 3600:
        raise ValueError("время должно быть в пределах 0,1…3600 с")
    if not 0.1 <= percent <= 100:
        raise ValueError("процент тяги должен быть в пределах 0,1…100")
    return (distance_m / seconds) / percent


def run(distance_m: float, seconds: float, percent: float,
        save_env: bool = False) -> dict:
    value = compute(distance_m, seconds, percent)
    report = {
        "ok": True,
        "distanceM": distance_m, "seconds": seconds, "percent": percent,
        "speedMps": round(distance_m / seconds, 6),
        "mpsPerPercent": round(value, 8),
        "envKey": "RC_NAV_MPS_PER_PERCENT",
        "saved": False,
    }
    if save_env:
        calibration.save_geometry({"RC_NAV_MPS_PER_PERCENT": value})
        report["saved"] = True
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--distance", type=float, required=True, help="пройдено, м")
    parser.add_argument("--seconds", type=float, required=True, help="время, с")
    parser.add_argument("--percent", type=float, required=True, help="команда t, %%")
    parser.add_argument("--save-env", action="store_true", help="записать в .env")
    parser.add_argument("--json", action="store_true", help="машиночитаемый отчёт")
    args = parser.parse_args(argv)
    try:
        report = run(args.distance, args.seconds, args.percent, args.save_env)
    except (ValueError, calibration.CalibrationError, OSError) as exc:
        print("Замер не принят: %s" % exc, file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, ensure_ascii=False))
    else:
        print("Скорость:        %.4f м/с при t=%g%%" % (report["speedMps"], args.percent))
        print("RC_NAV_MPS_PER_PERCENT = %.8f" % report["mpsPerPercent"])
        print("Сохранено в .env: %s" % ("да" if report["saved"] else "нет (добавьте --save-env)"))
        print("После сохранения перезапустите бортовой сервер и проверьте чек-лист на /map.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
