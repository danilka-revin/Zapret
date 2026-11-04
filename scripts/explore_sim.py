#!/usr/bin/env python3
"""Сухой прогон автообследования без железа.

Проверяет весь цикл `ExplorationService` на синтетической карте: поиск
границы → «поездка» → рост карты → автосохранение → визуальные ориентиры →
повтор, включая все штатные остановы. Ни лидар, ни шасси, ни Kinect не
открываются; «движение» имитирует FakeNavigation, который расширяет известную
область карты ровно так, как это сделал бы новый скан после переезда.

    ./scripts/explore_sim.py            # отчёт человеку
    ./scripts/explore_sim.py --json     # машиночитаемый отчёт
    ./scripts/explore_sim.py --steps 8

Это НЕ заменяет проверку на реальном роботе: физические замеры геометрии,
свежесть глубины и поведение привода проверяются по docs/AUTONOMY.md.
"""
from __future__ import annotations

import argparse
import json
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_control.exploration import (ExplorationService, LandmarkRecorder,
                                      coverage_pct, known_cells)
from robot_control.slam_map import UNKNOWN

WIDTH = HEIGHT = 60
RES = 0.1
ORIGIN = -3.0


class SyntheticMap:
    """Карта, где робот «видит» всё левее front, а справа — неизвестность."""

    def __init__(self, free_until: float = 1.0, reveal: float = 0.6):
        self.free_until = free_until
        self.reveal = reveal
        self.saved = 0

    def navigation_snapshot(self) -> dict:
        data = bytearray([UNKNOWN]) * (WIDTH * HEIGHT)
        for iy in range(HEIGHT):
            for ix in range(WIDTH):
                if ORIGIN + ix * RES < self.free_until:
                    data[iy * WIDTH + ix] = 0
        return {
            "data": bytes(data), "width": WIDTH, "height": HEIGHT,
            "resolution": RES, "origin": {"x": ORIGIN, "y": ORIGIN},
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "scans": 3, "quality": 1.0, "updatedAt": time.time(),
            "revision": 1, "trusted": True, "navigationIntegrity": True,
            "settings": {"matching": True, "decay": 0.0, "min_range": 0.3},
        }

    def advance(self) -> None:
        self.free_until = min(ORIGIN + WIDTH * RES - 0.2, self.free_until + self.reveal)

    def save(self) -> dict:
        self.saved += 1
        return {"ok": True}


class SimNavigation:
    """Имитирует поездку: сразу «приезжает» и добавляет новый участок карты."""

    def __init__(self, mapping: SyntheticMap):
        self.enabled = True
        self.active = False
        self.mapping = mapping
        self.starts: list[dict] = []
        self.cancels: list[str] = []

    def start(self, body, *, autonomous=False):
        assert autonomous is True, "автоцикл не имеет права обходить проверенный план"
        assert body.get("blindZoneChecked") is True, "слепая зона должна быть подтверждена"
        self.starts.append(body["goal"])
        self.mapping.advance()

    def cancel(self, reason):
        self.cancels.append(reason)
        self.active = False


class SimKinect:
    def fresh(self):
        return True


class SimCameras:
    def get_camera(self, cam_id):
        return type("Dev", (), {"role": f"cam{cam_id}",
                                "spec": {"yaw_deg": float(cam_id * 10)}})()

    def get_frame(self, cam_id, max_age_s=2.0):
        return b"\xff\xd8sim\xff\xd9" if 0 <= cam_id < 4 else None


def run_simulation(steps: int = 6, timeout_s: float = 15.0) -> dict:
    mapping = SyntheticMap()
    navigation = SimNavigation(mapping)
    before = {"known": known_cells(mapping.navigation_snapshot()),
              "coverage": coverage_pct(mapping.navigation_snapshot())}
    with tempfile.TemporaryDirectory(prefix="explore-sim-") as tmp:
        # camera_provider — это функция, возвращающая менеджер камер, а не сам менеджер.
        cameras = SimCameras()
        landmarks = LandmarkRecorder(lambda: cameras, directory=Path(tmp))
        service = ExplorationService(
            mapping, navigation, kinect=SimKinect(), camera_provider=lambda: cameras,
            enabled=True, landmarks=landmarks, estop_check=lambda: None,
        )
        service.max_steps = steps
        service.min_gain = 1
        service.max_no_gain = steps + 1
        service.max_failures = 3
        service.poll_s = 0.01
        service.max_duration_s = timeout_s
        service.start({"blindZoneChecked": True})
        deadline = time.monotonic() + timeout_s
        while service.active and time.monotonic() < deadline:
            time.sleep(0.01)
        if service.active:
            service.stop("симуляция превысила таймаут")
        status = service.status()
        after = {"known": known_cells(mapping.navigation_snapshot()),
                 "coverage": coverage_pct(mapping.navigation_snapshot())}
        report = {
            "ok": False,
            "steps": status.get("steps", 0),
            "phase": status.get("phase"),
            "message": status.get("message"),
            "knownCellsBefore": before["known"],
            "knownCellsAfter": after["known"],
            "coverageBeforePct": round(before["coverage"], 2),
            "coverageAfterPct": round(after["coverage"], 2),
            "goals": navigation.starts,
            "mapSaves": mapping.saved,
            "landmarks": len(landmarks.list(limit=200)),
        }
        report["ok"] = (report["steps"] > 0
                        and after["known"] > before["known"]
                        and report["mapSaves"] >= report["steps"]
                        and report["landmarks"] > 0)
        return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=6, help="предел шагов цикла")
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--json", action="store_true", help="машиночитаемый отчёт")
    args = parser.parse_args(argv)
    report = run_simulation(max(1, min(50, args.steps)), args.timeout)
    if args.json:
        print(json.dumps(report, ensure_ascii=False))
    else:
        print("Сухой прогон автообследования (без железа)")
        print("  шагов:            %d" % report["steps"])
        print("  фаза:             %s" % report["phase"])
        print("  известных клеток: %d → %d" % (report["knownCellsBefore"], report["knownCellsAfter"]))
        print("  покрытие:         %.1f%% → %.1f%%" % (report["coverageBeforePct"], report["coverageAfterPct"]))
        print("  целей выбрано:    %d" % len(report["goals"]))
        print("  сохранений карты: %d" % report["mapSaves"])
        print("  ориентиров:       %d" % report["landmarks"])
        print("  итог:             %s" % ("карта выросла, цикл исправен" if report["ok"]
                                          else "ПРОБЛЕМА: " + str(report["message"])))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
