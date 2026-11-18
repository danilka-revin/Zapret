#!/usr/bin/env python3
"""Сухой прогон автономности на трассе с конусами (без робота и без тяги).

Тот же автомат, что на вкладке /autonomy, но восприятие подменено скриптом:
коридор с левым и правым поворотами, знак СТОП, кочка (для проезда нужна
добавка мощности), красный→зелёный светофор. Ни камеры, ни Kinect, ни шасси,
ни файлы состояния не открываются; ``RC_TRACK_ENABLE`` не нужен.

    ./scripts/track_autonomy_sim.py
    ./scripts/track_autonomy_sim.py --scenario blocked_gate
    ./scripts/track_autonomy_sim.py --scenario full --json

Отчёт ``passed: true`` говорит только о результате сценария, а не о безопасности
движения. Для blocked_gate ожидается остановка без движения; перед реальным
пуском см. docs/TRACK_AUTONOMY.md.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_control.track_autonomy import TRACK_SCENARIOS, simulate_track  # noqa: E402


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=TRACK_SCENARIOS, default="full",
                        help="какие элементы трассы включить")
    parser.add_argument("--json", action="store_true", help="машиночитаемый отчёт")
    args = parser.parse_args(argv)
    report = simulate_track(args.scenario)
    counters = report["counters"]
    report["ok"] = report["passed"]
    if args.json:
        print(json.dumps(report, ensure_ascii=False))
    else:
        print("Сухой прогон автономности (без железа), сценарий: %s" % report["scenario"])
        print("  фаза:              %s" % report["phase"])
        print("  пройдено:          %.2f м за %.1f с (%d тактов)" %
              (report["distanceM"], report["elapsedS"], report["steps"]))
        print("  СТОП / кочки / светофоры: %d / %d / %d" %
              (counters["stop"], counters["bump"], counters["traffic"]))
        print("  добавок тяги:      %d (макс. +%.1f %%)" %
              (counters["powerBoosts"], report["maxBoostPercent"]))
        print("  событий:           %d" % len(report["events"]))
        for event in report["events"]:
            print("    %6.1f с  %5.1f м  %-14s %s" %
                  (event["atS"], event["distanceM"], event["phase"], event["message"]))
        result = ("проезд заблокирован как ожидалось" if report["expectedStop"] else "сценарий завершён")
        print("  итог:              %s" % (result if report["ok"] else "ПРОБЛЕМА: " + str(report["message"])))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
