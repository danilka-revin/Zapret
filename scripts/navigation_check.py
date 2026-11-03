#!/usr/bin/env python3
"""Read-only preflight report from the onboard lidar/map/navigation API.

This tool never sends motion commands or changes configuration. A green report
is not proof of physical safety, correct braking or reliable localization.
"""
from __future__ import annotations

import argparse
import json
import sys
import urllib.error
import urllib.request

REQUIRED_CHECKS = {
    "enabled": "автотяга включена конфигурацией",
    "speedCalibrated": "указана измеренная калибровка скорости",
    "speedWithinLimit": "скорость в лимите тяги",
    "masterReady": "шасси и четыре колеса готовы",
    "driveStopped": "посторонняя тяга выключена",
    "mapFresh": "карта свежая",
    "mapReady": "не менее двух сканов карты",
    "mapTrusted": "карта построена из физического лидара/ROS",
    "mapIntegrity": "история карты пригодна для навигации",
    "mapDecayDisabled": "забывание препятствий выключено",
    "scanReady": "свежий скан с полным обзором",
    "poseReady": "сопоставление сканов включено и принято",
    "footprintSufficient": "габарит не меньше диагонали корпуса",
    "estopClear": "защёлка E-STOP снята",
}


def fetch(url: str) -> dict:
    try:
        with urllib.request.urlopen(url, timeout=3) as response:  # noqa: S310
            value = json.loads(response.read(10_000_001))
            if not isinstance(value, dict):
                raise ValueError("ответ не JSON-объект")
            return value
    except (urllib.error.URLError, OSError, ValueError) as exc:
        return {"error": str(getattr(exc, "reason", exc))}


def inspect(base_url: str) -> dict:
    base = base_url.rstrip("/")
    lidar = fetch(base + "/api/lidar")
    mapping = fetch(base + "/api/kiosk_map")
    nav = fetch(base + "/api/navigation/status")
    errors = [name for name, value in (("лидар", lidar), ("карта", mapping), ("навигация", nav))
              if value.get("error") or value.get("ok") is not True]
    checks = (nav.get("diagnostics") or {}).get("checks") or {}
    missing = [key for key in REQUIRED_CHECKS if key not in checks]
    failed = [key for key in REQUIRED_CHECKS if checks.get(key) is not True]
    return {"ok": not errors and not failed, "errors": errors,
            "failedChecks": failed, "missingChecks": missing,
            "checks": checks, "lidar": lidar, "map": {key: mapping.get(key) for key in
                ("scans", "points", "quality", "pose", "settings", "updatedAt", "lidarOffsetM")},
            "navigation": {key: nav.get(key) for key in
                ("enabled", "canStart", "reason", "robotDimensions", "diagnostics")},
            "warning": "Это только программные проверки; физические E-STOP, торможение, обзор и локализацию нужно проверить на роботе."}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Проверить бортовые API без движения или изменения настроек")
    parser.add_argument("--url", default="http://127.0.0.1:8090", help="адрес бортового экрана (по умолчанию %(default)s)")
    parser.add_argument("--json", action="store_true", help="выдать JSON")
    args = parser.parse_args(argv)
    report = inspect(args.url)
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        print("Диагностика навигации (только чтение, без команды движения)\n")
        if report["errors"]:
            print("Не отвечают API: " + ", ".join(report["errors"]))
        lidar = report["lidar"]
        mapping = report["map"]
        print("Лидар: источник %s, точек %s, возраст %s с" %
              (lidar.get("source", "—"), lidar.get("points_count", "—"), lidar.get("age_s", "—")))
        print("Карта: сканов %s, качество позы %s, поза %s" %
              (mapping.get("scans", "—"), mapping.get("quality", "—"), mapping.get("pose", "—")))
        for key, label in REQUIRED_CHECKS.items():
            value = report["checks"].get(key)
            print("  %s %s%s" % ("✓" if value is True else "✗", label,
                                   " (нет проверки в ответе)" if key in report["missingChecks"] else ""))
        if report["navigation"].get("reason"):
            print("Причина: " + report["navigation"]["reason"])
        print("\n" + ("Все программные проверки пройдены." if report["ok"] else
                       "Программные проверки не пройдены; автономно не запускать."))
        print(report["warning"])
    return 2 if report["errors"] else (0 if report["ok"] else 1)


if __name__ == "__main__":
    sys.exit(main())
