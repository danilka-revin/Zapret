#!/usr/bin/env python3
"""Синтетический скан лидара для демонстрации карты без железа.

Скрипт **не открывает порты и не двигает робота**. Он раз в несколько
десятых секунды отправляет на уже запущенный бортовой экран (8090) обычный
``POST /api/lidar`` с точками комнаты: так карта, планировщик маршрута и
предпросмотр пути видны в браузере без настоящего лидара.

    ./scripts/demo_map.sh                  # экран 8090 + этот скан одной командой
    .venv/bin/python scripts/demo_scan.py  # только скан к уже запущенному экрану

Границы честности:

* точки помечаются сервером как ``api``-скан, а не как физический лидар,
  поэтому автономная тяга («Поехать») по-прежнему заблокирована;
* это НЕ проверка навигации и НЕ замена испытаниям из ``docs/navigation.md``;
* не запускайте скрипт на реальном роботе рядом с настоящим лидаром: скан
  смешается с физическим и карта станет неверной.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
import time
import urllib.error
import urllib.request

#: Комната вокруг робота: стены и несколько препятствий, метры.
DEFAULT_WIDTH = 6.0
DEFAULT_DEPTH = 4.0
OBSTACLES = (
    # Прямоугольная коробка перед роботом справа и колонна слева.
    ((-1.4, 0.9), (0.2, 0.9), (0.2, 1.6), (-1.4, 1.6)),
    ((-2.0, -1.2), (-1.5, -1.2), (-1.5, -0.7), (-2.0, -0.7)),
)


def _segments(width: float, depth: float) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """Стены комнаты (робот в центре) и препятствия как отрезки."""
    hx, hy = width / 2.0, depth / 2.0
    walls = [
        ((-hx, -hy), (hx, -hy)),
        ((hx, -hy), (hx, hy)),
        ((hx, hy), (-hx, hy)),
        ((-hx, hy), (-hx, -hy)),
    ]
    boxes: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for corners in OBSTACLES:
        for index, start in enumerate(corners):
            boxes.append((start, corners[(index + 1) % len(corners)]))
    return walls + boxes


def _ray_cast(angle: float, segments, limit: float) -> float:
    """Расстояние до ближайшего отрезка вдоль луча из (0, 0)."""
    dx, dy = math.cos(angle), math.sin(angle)
    best = limit
    for (x1, y1), (x2, y2) in segments:
        ex, ey = x2 - x1, y2 - y1
        denominator = dx * ey - dy * ex
        if abs(denominator) < 1e-9:
            continue
        t = (x1 * ey - y1 * ex) / denominator
        u = (x1 * dy - y1 * dx) / denominator
        if t > 0.02 and 0.0 <= u <= 1.0 and t < best:
            best = t
    return best


def scan_points(width: float, depth: float, rays: int, noise: float,
                rng: random.Random) -> list[dict[str, float]]:
    """Виток лидара: угол от «вперёд» против часовой, расстояние в метрах."""
    segments = _segments(width, depth)
    limit = max(width, depth)  # стены комнаты всегда ближе этого значения
    points = []
    for index in range(rays):
        angle = -math.pi + 2.0 * math.pi * index / rays
        distance = _ray_cast(angle, segments, limit)
        if noise > 0:
            distance = max(0.05, distance + rng.gauss(0.0, noise))
        points.append({"a": round(angle, 4), "r": round(distance, 3)})
    return points


def post_scan(url: str, points: list[dict[str, float]], timeout: float) -> dict:
    body = json.dumps({"points": points}).encode("utf-8")
    request = urllib.request.Request(
        url.rstrip("/") + "/api/lidar", data=body, method="POST",
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--url", default="http://127.0.0.1:8090",
                        help="адрес бортового экрана (по умолчанию http://127.0.0.1:8090)")
    parser.add_argument("--rate", type=float, default=5.0, help="витков в секунду (по умолчанию 5)")
    parser.add_argument("--count", type=int, default=0,
                        help="сколько витков отправить, 0 — до Ctrl+C (по умолчанию 0)")
    parser.add_argument("--width", type=float, default=DEFAULT_WIDTH, help="ширина комнаты, м")
    parser.add_argument("--depth", type=float, default=DEFAULT_DEPTH, help="глубина комнаты, м")
    parser.add_argument("--rays", type=int, default=360, help="лучей в витке (по умолчанию 360)")
    parser.add_argument("--noise", type=float, default=0.01, help="шум дальности, м")
    parser.add_argument("--seed", type=int, default=7, help="зерно шума (для повторов)")
    args = parser.parse_args(argv)

    if args.rate <= 0:
        print("частота должна быть больше нуля", file=sys.stderr)
        return 2
    if min(args.width, args.depth) < 1.0:
        print("комната меньше метра — проверьте --width/--depth", file=sys.stderr)
        return 2

    rng = random.Random(args.seed)
    period = 1.0 / args.rate
    print("демонстрационный скан (не настоящий лидар): комната "
          f"{args.width:g}×{args.depth:g} м, {args.rays} лучей → {args.url}/api/lidar")
    print("автономная тяга этим НЕ включается; это только карта и планировщик.")
    sent = 0
    try:
        while args.count <= 0 or sent < args.count:
            started = time.monotonic()
            points = scan_points(args.width, args.depth, args.rays, args.noise, rng)
            try:
                answer = post_scan(args.url, points, timeout=2.0)
            except (urllib.error.URLError, OSError, ValueError) as exc:
                print(f"экран {args.url} недоступен: {exc}\n"
                      "запустите его: ./run_kiosk.py --source none --no-open", file=sys.stderr)
                return 1
            if not answer.get("ok"):
                print(f"сервер отклонил скан: {answer}", file=sys.stderr)
                return 1
            sent += 1
            if sent % int(max(1.0, args.rate * 5)) == 0:
                print(f"отправлено витков: {sent}", end="\r", flush=True)
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    except KeyboardInterrupt:
        print()
    print(f"готово: отправлено витков {sent}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
