#!/usr/bin/env python3
"""Поездка A → B по проверенному маршруту одной командой (с гейтами).

Скрипт говорит с *бортовым* API (по умолчанию ``http://127.0.0.1:8090``) и
делает три вещи по порядку:

1. **Проверка.** Читает ``/api/navigation/status`` и показывает все 14 условий
   готовности из ``docs/navigation.md`` — с причиной отказа, если что-то не так.
   Ничего не двигает и не меняет настройки.
2. **Маршрут.** С ``--goal X,Y`` строит план (``POST /api/navigation/plan``),
   печатает длину, число точек, признак частичного пути и радиус слепой зоны.
   Движения всё ещё нет.
3. **Поездка.** Только с ``--go`` **и** ``--yes``: ``POST
   /api/navigation/start``, затем опрос статуса, пока робот не остановится.
   Ctrl+C отменяет маршрут обычным API (робот тормозит).

Примеры::

    # 1. Где робот и почему нельзя ехать (безопасно, только чтение)
    ./scripts/nav_test_run.py

    # 2. Построить маршрут A → B и посмотреть, что получилось
    ./scripts/nav_test_run.py --goal 1.5,0.0

    # 3. Разрешить сам проезд (площадка проверена, E-STOP в руке)
    ./scripts/nav_test_run.py --goal 1.5,0.0 --go --yes

    # ...и подтвердить осмотр слепой зоны, если план этого требует
    ./scripts/nav_test_run.py --goal 1.5,0.0 --go --yes --blind-zone-checked

Что скрипт **не** делает:

* не включает ``RC_NAV_ENABLE`` и не придумывает ``RC_NAV_MPS_PER_PERCENT`` —
  калибровка вписывается в ``.env`` отдельно, после замера;
* не обходит проверки: если они красные, сервер откажет, и скрипт покажет
  причину, а не будет «продавливать» старт;
* не заменяет физический E-STOP и наблюдателя. Автономная тяга — движение
  настоящего робота.

Если сервер вернул ``plan_changed``, поза или карта изменились между планом и
стартом: маршрут показан заново, нужно посмотреть его глазами и запустить
команду ещё раз. Автоматического повтора здесь нет намеренно.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request

HEADER = {"X-Robot-Control": "navigation"}
#: Проверки, без которых автономный старт запрещён (как в navigation_check.py).
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
ACTIVE_PHASES = {"starting", "executing", "stopping"}
DONE_PHASES = {"succeeded", "blocked", "cancelled", "error", "idle"}


class ApiError(RuntimeError):
    def __init__(self, message: str, status: int = 0, payload: dict | None = None):
        super().__init__(message)
        self.status = status
        self.payload = payload or {}


def call(base: str, path: str, body: dict | None = None, *, headers: dict | None = None,
         timeout: float = 6.0) -> dict:
    """Запрос к бортовому API. Возвращает JSON или бросает ApiError с причиной."""
    url = base.rstrip("/") + path
    data = None if body is None else json.dumps(body).encode("utf-8")
    request = urllib.request.Request(
        url, data=data, method="GET" if body is None else "POST",
        headers={"Content-Type": "application/json", "Connection": "close", **(headers or {})},
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:  # noqa: S310
            raw = response.read(10_000_001).decode("utf-8", "replace")
            return json.loads(raw) if raw else {}
    except urllib.error.HTTPError as exc:
        raw = exc.read(10_000_001).decode("utf-8", "replace") if exc.fp else ""
        try:
            payload = json.loads(raw) if raw else {}
        except ValueError:
            payload = {"error": raw[:400]}
        raise ApiError(str(payload.get("error") or payload.get("message") or exc.reason),
                       exc.code, payload) from None
    except (urllib.error.URLError, OSError, ValueError) as exc:
        raise ApiError(str(getattr(exc, "reason", exc)), 0, {}) from None


def pose_text(pose: dict | None) -> str:
    if not isinstance(pose, dict):
        return "поза неизвестна"
    return "x=%.2f y=%.2f θ=%.0f°" % (
        float(pose.get("x") or 0.0), float(pose.get("y") or 0.0),
        float(pose.get("yaw") or 0.0) * 180.0 / 3.141592653589793)


def current_pose(base: str) -> dict | None:
    """Поза робота по карте: в статусе навигации её нет, в /api/kiosk_map есть."""
    try:
        mapping = call(base, "/api/kiosk_map", timeout=4.0)
    except ApiError:
        return None
    pose = mapping.get("pose")
    return pose if isinstance(pose, dict) else None


def report_readiness(status: dict) -> bool:
    """Печатает 14 проверок; возвращает True, если старт разрешён."""
    diagnostics = status.get("diagnostics") or {}
    checks = diagnostics.get("checks") or {}
    print("Готовность навигации")
    failed = []
    for key, label in REQUIRED_CHECKS.items():
        value = checks.get(key)
        if value is True:
            print("  ✓ %s" % label)
        else:
            failed.append(key)
            mark = "✗" if value is False else "?"
            print("  %s %s%s" % (mark, label,
                                 " (проверки нет в ответе)" if key not in checks else ""))
    print("  карта обновлена %.1f с назад, качество позы %.0f%%, точек скана %s" % (
        float(diagnostics.get("mapAgeS") or 0.0),
        float(diagnostics.get("poseQuality") or 0.0) * 100.0,
        diagnostics.get("scanPoints", "—")))
    if failed:
        print("  причина: " + str(status.get("reason") or "проверки не пройдены"))
    return not failed


def format_plan(plan: dict) -> str:
    path = plan.get("path") or []
    text = ("маршрут: %.2f м, точек пути %d, сегментов %s" % (
        float(plan.get("distanceM") or 0.0), len(path), plan.get("waypoints", "—")))
    if plan.get("partial"):
        text += "\n  маршрут частичный: дальше робот досканирует и перепланирует сам"
    if not plan.get("executable", True):
        text += "\n  ВНИМАНИЕ: подтверждённого свободного участка нет — старт невозможен"
    return text


def wait_for_finish(base: str, goal: dict, *, poll: float, timeout: float) -> dict:
    """Опрос статуса до остановки робота. Ctrl+C отменяет маршрут."""
    started = time.monotonic()
    last_line = ""
    print("\nПоездка началась. Отмена — Ctrl+C (робот тормозит), при сомнении — "
          "физический E-STOP.")
    try:
        while True:
            time.sleep(poll)
            elapsed = time.monotonic() - started
            status = call(base, "/api/navigation/status", timeout=4.0)
            phase = str(status.get("phase") or "?")
            line = "  %5.1f с  %-9s %3.0f%%  точка %s/%s  обход %s/%s  %s" % (
                elapsed, phase, float(status.get("progress") or 0.0) * 100.0,
                status.get("waypoint", "—"), status.get("totalWaypoints", "—"),
                status.get("detour", "—"), status.get("detourLimit", "—"),
                str(status.get("message") or "")[:90])
            if line != last_line:
                print(line, flush=True)
                last_line = line
            if phase in DONE_PHASES:
                return status
            if elapsed > timeout:
                print("\nтаймаут ожидания (%.0f с) — отменяю маршрут" % timeout)
                call(base, "/api/navigation/cancel", {}, headers=HEADER, timeout=4.0)
                return call(base, "/api/navigation/status", timeout=4.0)
    except KeyboardInterrupt:
        print("\nCtrl+C: отменяю маршрут…")
        try:
            call(base, "/api/navigation/cancel", {}, headers=HEADER, timeout=4.0)
        except ApiError as exc:
            print("  не удалось отправить отмену (%s). НАЖМИТЕ ФИЗИЧЕСКИЙ E-STOP." % exc,
                  file=sys.stderr)
            raise
        status = call(base, "/api/navigation/status", timeout=4.0)
        print("  фаза после отмены: %s" % status.get("phase"))
        return status


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Проверка готовности и тест A → B на бортовом API робота",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8090",
                        help="бортовой экран (по умолчанию %(default)s)")
    parser.add_argument("--goal", metavar="X,Y",
                        help="точка Б в метрах карты, например 1.5,0.0")
    parser.add_argument("--go", action="store_true",
                        help="разрешить движение (требует --yes)")
    parser.add_argument("--yes", action="store_true",
                        help="подтверждение: площадка проверена, наблюдатель и E-STOP на месте")
    parser.add_argument("--blind-zone-checked", action="store_true",
                        help="ближняя слепая зона лидара физически осмотрена и безопасна")
    parser.add_argument("--poll", type=float, default=1.0, help="период опроса статуса, с")
    parser.add_argument("--timeout", type=float, default=180.0,
                        help="предохранитель: отменить маршрут через N секунд")
    args = parser.parse_args(argv)

    if args.go and not args.yes:
        print("--go без --yes не запускаю: подтвердите площадку и E-STOP флагом --yes.",
              file=sys.stderr)
        return 2

    try:
        status = call(args.url, "/api/navigation/status")
    except ApiError as exc:
        print("Бортовой API %s недоступен: %s" % (args.url, exc), file=sys.stderr)
        print("На борту это ./run_kiosk.py (источник master). Снаружи — через "
              "./scripts/serve.sh.", file=sys.stderr)
        return 3

    pose = (status.get("plan") or {}).get("pose") or current_pose(args.url)
    print("Робот: %s, фаза %r" % (pose_text(pose), status.get("phase")))
    ready = report_readiness(status)

    if not args.goal:
        if ready:
            print("\nПроверки зелёные. Укажите точку Б: --goal X,Y "
                  "(координаты видны на /map).")
        return 0 if ready else 1

    try:
        x_text, y_text = args.goal.replace(" ", "").split(",")
        goal = {"x": float(x_text), "y": float(y_text)}
    except ValueError:
        print("--goal ждёт два числа через запятую: X,Y", file=sys.stderr)
        return 2

    print("\nСтрою маршрут к (%.2f, %.2f)…" % (goal["x"], goal["y"]))
    try:
        planned = call(args.url, "/api/navigation/plan", {"goal": goal})
    except ApiError as exc:
        print("Маршрут не построен [%s]: %s" % (exc.payload.get("error") or exc.status, exc),
              file=sys.stderr)
        if exc.payload.get("status"):
            report_readiness(exc.payload["status"])
        return 1
    plan = planned.get("plan") or {}
    print("  " + format_plan(plan))

    manual_blind = bool(plan.get("manualBlindZone"))
    radius = plan.get("blindInspectionRadiusM")
    if manual_blind:
        print("  Слепая зона: осмотреть круг радиусом %s м вокруг лидара на месте старта "
              "и убедиться, что там пусто." % (radius if radius is not None else "?"))
        if not args.blind_zone_checked:
            print("  Старт потребует --blind-zone-checked (это осмотр, а не показание лидара).")

    if not args.go:
        print("\nДвижения не было. Чтобы поехать, повторите с --go --yes"
              + (" --blind-zone-checked" if manual_blind else "") + ".")
        return 0 if ready else 1
    if not ready:
        print("\nСтарт не отправляю: проверки красные (см. выше).", file=sys.stderr)
        return 1

    body = {"goal": goal, "blindZoneChecked": bool(args.blind_zone_checked)}
    try:
        started = call(args.url, "/api/navigation/start", body, headers=HEADER)
    except ApiError as exc:
        code = str(exc.payload.get("error") or "")
        print("Старт отклонён [%s]: %s" % (code or exc.status, exc), file=sys.stderr)
        if code == "plan_changed":
            new_plan = (exc.payload.get("details") or {}).get("plan") or {}
            print("  " + format_plan(new_plan))
            print("  Посмотрите новый маршрут на /map и запустите команду ещё раз.")
        elif code == "blind_zone_confirmation_required":
            print("  Осмотрите слепую зону и добавьте --blind-zone-checked.")
        return 1
    print("  сервер принял старт: %s" % ((started.get("status") or {}).get("message") or "ok"))

    final = wait_for_finish(args.url, goal, poll=args.poll, timeout=args.timeout)
    phase = str(final.get("phase") or "?")
    print("\nИтог: %s, прогресс %.0f%%, точек пройдено %s/%s, обходов %s" % (
        phase, float(final.get("progress") or 0.0) * 100.0,
        final.get("waypoint", "—"), final.get("totalWaypoints", "—"), final.get("detour", 0)))
    if final.get("message"):
        print("Сообщение: %s" % final["message"])
    if phase == "succeeded":
        print("Цель достигнута по плану %.2f м. Проверьте фактическое положение робота "
              "и площадку вокруг — программа не заменяет осмотр." % float(plan.get("distanceM") or 0.0))
        return 0
    print("Маршрут не завершён. Не повторяйте вслепую: осмотрите площадку, затем "
          "«Продолжить» на /route или новый маршрут.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
