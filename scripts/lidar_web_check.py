#!/usr/bin/env python3
"""Проверка, что реальный лидар дошёл до веб-экранов (пульт 8080 и киоск 8090).

``scripts/lidar_check.py`` отвечает на вопрос «видит ли железо сам Python».
Этот скрипт проверяет остаток пути — то, что обычно и ломается:

  * веб-приложения запущены и отвечают;
  * запущены они тем интерпретатором, где есть pyserial (``.venv``);
  * ``GET /api/lidar`` отдаёт свежий скан с подтверждённым приложением
    происхождением и временем (``scan_trusted``, ``connected``, ``points_count``);
  * скан не подменён демо (``demo``) или API-сканом без provenance;
    программная проверка не доказывает подлинность датчика, корректность
    измерений, обзор и безопасность движения;
  * второй процесс робота берёт скан у первого (``source == "peer"``), то есть
    пульт и бортовой экран не воюют за один USB-порт;
  * если скана нет — печатается причина из API вместе с USB-диагностикой
    (порты, права, исключения, конфликт настроек).

Запуск на роботе (приложения должны работать):

    .venv/bin/python scripts/lidar_web_check.py
    .venv/bin/python scripts/lidar_web_check.py --wait 20     # ждать свежий скан
    .venv/bin/python scripts/lidar_web_check.py --json        # для скриптов

Коды возврата: 0 — оба экрана показывают реальный скан; 1 — приложение работает,
но реального скана нет (в выводе причина); 2 — приложение не запущено/не отвечает.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import urllib.error
import urllib.request

#: Оба веб-приложения робота: название, адрес /api/lidar и чем перезапустить.
DEFAULT_TARGETS = (
    ("пульт оператора", "http://127.0.0.1:8080/api/lidar", "./scripts/start.sh"),
    ("бортовой экран", "http://127.0.0.1:8090/api/lidar", "./scripts/kiosk.sh"),
)

#: Сколько секунд скан считается свежим (как в robot_control/lidar.py).
MAX_AGE_S = 1.0


def log(text: str = "") -> None:
    print(text, flush=True)


def fetch(url: str, timeout: float = 2.0) -> dict:
    """Ответ ``GET /api/lidar`` как dict; ``error`` — если приложение не отвечает."""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:  # noqa: S310
            if response.status != 200:
                return {"error": "HTTP %s" % response.status}
            data = json.loads(response.read().decode("utf-8", "replace"))
            return data if isinstance(data, dict) else {"error": "ответ не объект"}
    except urllib.error.HTTPError as exc:
        return {"error": "HTTP %s" % exc.code}
    except (urllib.error.URLError, OSError, ValueError) as exc:
        return {"error": str(getattr(exc, "reason", exc))}


def real_scan(status: dict) -> bool:
    """Есть ли в ответе свежий настоящий скан (не демо)."""
    if status.get("error") or status.get("demo"):
        return False
    if not status.get("connected") or status.get("scan_trusted") is not True:
        return False
    age = status.get("age_s")
    if (not isinstance(age, (int, float)) or isinstance(age, bool)
            or not math.isfinite(age) or not 0 <= age <= MAX_AGE_S):
        return False
    points = status.get("points_count")
    return isinstance(points, int) and not isinstance(points, bool) and points > 0


def describe(name: str, url: str, status: dict, restart: str = "") -> dict:
    """Отчёт по одному приложению: что видно в его /api/lidar."""
    report = {
        "name": name,
        "url": url,
        "restart": restart,
        "reachable": not status.get("error"),
        "real_scan": real_scan(status),
        "demo": bool(status.get("demo")),
        "connected": bool(status.get("connected")),
        "scan_trusted": status.get("scan_trusted") is True,
        "points": status.get("points_count") or 0,
        "source": status.get("source"),
        "detected": status.get("detected"),
        "age_s": status.get("age_s"),
        "python": status.get("python"),
        "pyserial": status.get("pyserial"),
        "reason": status.get("reason"),
        "hint": status.get("hint"),
        "error": status.get("error"),
    }
    diagnostics = status.get("usb_diagnostics") or {}
    report["usb_summary"] = diagnostics.get("summary")
    report["usb_ports"] = diagnostics.get("ports") or []
    return report


def print_report(report: dict) -> None:
    log("%s — %s" % (report["name"], report["url"]))
    if not report["reachable"]:
        log("  ✘ не отвечает: %s" % report["error"])
        log("    запусти: %s" % (report.get("restart") or "./scripts/start.sh"))
        return
    log("  интерпретатор: %s · pyserial: %s"
        % (report["python"], report["pyserial"] or "НЕТ — лидар не читается"))
    if not report["pyserial"]:
        log("  ✘ веб-приложение запущено не из .venv: ./scripts/ensure_deps.sh "
            "и перезапуск (см. README)")
    if report["real_scan"]:
        detected = report["detected"] or {}
        log("  ✔ реальный скан: %s точек, источник %s" % (report["points"], report["source"]))
        if detected:
            log("    устройство: %s · %s · %s бод · мотор %s"
                % (detected.get("port"), detected.get("protocol"),
                   detected.get("baud"), detected.get("motor")))
        if report["source"] == "peer":
            log("    порт держит второй процесс робота — скан получен по HTTP (это штатно)")
        return
    if report["demo"]:
        log("  ✘ ДЕМО-скан вместо реального лидара (%s точек): настоящий скан не читается"
            % report["points"])
    elif report["connected"] and not report["scan_trusted"]:
        log("  ✘ скан виден, но источник или время не подтверждены (scan_trusted не true).")
        log("    Проверь источник, время ROS и обнови оба приложения до одной версии.")
    else:
        log("  ✘ подтверждённого свежего скана нет (точек: %s)" % report["points"])
    if report["reason"]:
        log("    причина: %s" % report["reason"])
    if report["hint"]:
        log("    подсказка: %s" % report["hint"])
    if report["usb_summary"]:
        log("    USB: %s" % report["usb_summary"])
    for port in report["usb_ports"]:
        log("      • %s → %s%s%s" % (
            port.get("port"), port.get("device"),
            "" if port.get("accessible") else " (НЕТ ДОСТУПА: нужна группа dialout)",
            " (исключён настройками мастера/шасси)" if port.get("excluded") else "",
        ))


def check(targets, wait: float = 0.0, pause: float = 0.5) -> list[dict]:
    """Опрашивает приложения; ``wait`` — сколько секунд ждать свежий скан."""
    deadline = time.monotonic() + max(0.0, wait)
    reports = []
    while True:
        reports = [describe(target[0], target[1], fetch(target[1]), target[2])
                   for target in targets]
        alive = [r for r in reports if r["reachable"]]
        if alive and all(r["real_scan"] for r in alive):
            return reports
        if time.monotonic() >= deadline:
            return reports
        time.sleep(pause)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Проверка: доходит ли реальный скан лидара до веб-экранов робота",
    )
    parser.add_argument("--console-url", default=DEFAULT_TARGETS[0][1],
                        help="адрес /api/lidar пульта (по умолчанию %(default)s)")
    parser.add_argument("--kiosk-url", default=DEFAULT_TARGETS[1][1],
                        help="адрес /api/lidar бортового экрана (по умолчанию %(default)s)")
    parser.add_argument("--only", choices=["console", "kiosk"], default=None,
                        help="проверить только одно приложение")
    parser.add_argument("--wait", type=float, default=5.0,
                        help="сколько секунд ждать свежий скан (по умолчанию 5)")
    parser.add_argument("--json", action="store_true", help="вывод в JSON (для скриптов)")
    args = parser.parse_args(argv)

    targets = []
    if args.only != "kiosk":
        name, _url, restart = DEFAULT_TARGETS[0]
        targets.append((name, args.console_url, restart))
    if args.only != "console":
        name, _url, restart = DEFAULT_TARGETS[1]
        targets.append((name, args.kiosk_url, restart))

    reports = check(targets, wait=args.wait)

    if args.json:
        log(json.dumps({
            "ok": all(r["real_scan"] for r in reports),
            "reports": reports,
        }, ensure_ascii=False, indent=2))
    else:
        log("Проверка веб-лидара: реальный скан должен доходить до обоих экранов.\n")
        for report in reports:
            print_report(report)
            log()

    missing = [r for r in reports if not r["reachable"]]
    if missing:
        if not args.json:
            log("Итог: не все веб-приложения запущены — проверка не полная.")
        return 2
    if all(r["real_scan"] for r in reports):
        if not args.json:
            sources = {r["source"] for r in reports}
            log("Итог: свежий подтверждённый скан виден на проверенных экранах (источники: %s)."
                % ", ".join(sorted(str(s) for s in sources)))
            log("Это программная проверка: датчик, геометрию и обзор проверьте физически.")
        return 0
    if not args.json:
        log("Итог: веб-приложения работают, но реального скана нет — причина выше.")
        log("Что делать дальше:")
        log("  1) ./scripts/ros2_lidar.sh --status                    # состояние драйвера")
        log("  2) ls -l /dev/serial/by-id/                           # посмотреть имена устройств")
        log("  3) journalctl -u robot-control -f  (или окно start.sh) # журнал лидара")
        log("  ВНИМАНИЕ: активный перебор портов может сбросить платы; не запускайте его на действующем роботе.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
