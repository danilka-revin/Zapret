#!/usr/bin/env python3
"""Measure STOP-to-four-ACK latency via the running onboard server.

This sends real STOP commands. Run only on a secured, stationary robot with
wheels lifted, an independent E-STOP and nobody else controlling it. STOP also
interrupts any keyboard session; never run during a task or on a slope.

    python3 scripts/measure_wheel_latency.py --confirm-stop --count 10

The result includes local HTTP, server scheduling, polling, USB and firmware;
it is NOT an isolated UART measurement or proof that a mechanical brake works.
The program never sends non-zero traction or steering commands and never opens
wheel serial ports itself (the onboard process remains the sole port owner).
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
import urllib.error
import urllib.request

MODULES = ("FL", "FR", "RL", "RR")


class MeasurementError(RuntimeError):
    pass


def request_json(origin, path, body=None):
    data = None if body is None else json.dumps(body).encode("utf-8")
    headers = {"Accept": "application/json"}
    if data is not None:
        headers.update({"Content-Type": "application/json", "X-Robot-Control": "drive",
                        "X-Robot-Command-Deadline": str(time.time() + 0.5)})
    request = urllib.request.Request(origin.rstrip("/") + path, data=data, headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=2) as response:
            result = json.load(response)
    except (OSError, ValueError, urllib.error.HTTPError) as exc:
        raise MeasurementError(f"HTTP {path}: {exc}") from exc
    if not isinstance(result, dict) or result.get("ok") is not True:
        raise MeasurementError(f"HTTP {path}: {result}")
    return result


def validate_state(data):
    """Refuse missing, stale, busy, neutral or plausibly moving telemetry."""
    if not isinstance(data, dict) or data.get("linkOk") is not True or data.get("estop") is not None:
        raise MeasurementError("Нет связи или включён E-STOP")
    neutral, drive = data.get("neutral") or {}, data.get("drive") or {}
    if neutral.get("locked") is not False or neutral.get("phase") != "idle":
        raise MeasurementError("Сначала выключите нейтраль и подтвердите тормоз")
    if (drive.get("available") is not True or drive.get("active") is not False or
            drive.get("commandedPercent") not in (None, 0) or
            any(v not in (None, 0) for v in (drive.get("commandedModules") or {}).values()) or
            (drive.get("maneuver") or {}).get("active") is not False):
        raise MeasurementError("Тяга или манёвр активны, либо их состояние неизвестно")
    items = data.get("motors")
    if not isinstance(items, list):
        raise MeasurementError("Нет четырёх колёс")
    motors = {m.get("id"): m for m in items if isinstance(m, dict)}
    if len(items) != 4 or set(motors) != set(MODULES):
        raise MeasurementError("Нет четырёх колёс")
    for module, m in motors.items():
        age, rpm, firmware = m.get("telemetryAgeS"), m.get("scRpm"), m.get("firmware")
        number = lambda x: isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
        if (m.get("online") is not True or not number(age) or not 0 <= age < 0.75 or
                type(firmware) is not int or firmware < 17 or type(m.get("brakeId")) is not int or
                type(m.get("drivePercent")) is not int or m["drivePercent"] != 0 or
                type(m.get("driveServoValue")) is not int or m["driveServoValue"] != 0 or
                m.get("moving") is not False or m.get("calibrating") is not False or
                m.get("cycling") is not False or not number(rpm) or not 0 <= rpm <= 1 or
                (firmware >= 19 and m.get("scRecent") is not False) or
                (firmware >= 18 and (m.get("neutralId") != 0 or m.get("neutralLocked") is not False))):
            raise MeasurementError(f"{module}: колесо занято, движется или телеметрия неизвестна/устарела")
    return motors


def measure(origin, count, poll_interval=0.01):
    before = request_json(origin, "/api/kiosk_wheel_state")["data"]
    validate_state(before)
    navigation = request_json(origin, "/api/navigation/status")
    if navigation.get("phase") not in {"idle", "planned", "blocked", "cancelled", "succeeded", "error"}:
        raise MeasurementError("Навигация активна или её состояние неизвестно")
    reply = request_json(origin, "/api/kiosk_keyboard", {"action": "begin"})
    session = reply.get("session")
    if not isinstance(session, str) or not session:
        raise MeasurementError("Сервер не выдал сессию STOP")
    sequence = 0
    samples = {m: [] for m in MODULES}
    samples["all"] = []
    try:
        for _ in range(count):
            sequence += 1
            started = time.monotonic()
            response = request_json(origin, "/api/kiosk_keyboard",
                                    {"action": "stop", "session": session, "sequence": sequence})
            brake_id = response.get("brakeId")
            if type(brake_id) is not int or brake_id <= 0:
                raise MeasurementError("Сервер не выдал новый идентификатор тормоза")
            seen = {}
            deadline = started + 2.0
            while len(seen) < len(MODULES):
                if time.monotonic() > deadline:
                    raise MeasurementError(f"Не все колёса подтвердили STOP {brake_id}")
                motors = validate_state(request_json(origin, "/api/kiosk_wheel_state")["data"])
                received = time.monotonic()
                for module in MODULES:
                    if module not in seen and motors[module].get("brakeId") == brake_id:
                        seen[module] = (received - started) * 1000.0
                if len(seen) < len(MODULES):
                    time.sleep(poll_interval)
            for module in MODULES:
                samples[module].append(seen[module])
            samples["all"].append(max(seen.values()))
            time.sleep(0.20)
    finally:
        try:
            request_json(origin, "/api/kiosk_keyboard",
                         {"action": "end", "session": session, "sequence": sequence + 1})
        except MeasurementError as exc:
            raise MeasurementError(f"Не удалось закрыть сессию STOP: {exc}") from exc
    return samples


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--url", default="http://127.0.0.1:8090", help="бортовой сервер; по умолчанию localhost:8090")
    parser.add_argument("--count", type=int, default=10, help="число STOP (1–30)")
    parser.add_argument("--confirm-stop", action="store_true", help="подтверждаю безопасный стенд и настоящие STOP")
    args = parser.parse_args(argv)
    if not args.confirm_stop:
        parser.error("требуется --confirm-stop: программа отправит реальные STOP всем колёсам")
    if not 1 <= args.count <= 30:
        parser.error("--count должен быть от 1 до 30")
    if not args.url.startswith(("http://", "https://")):
        parser.error("--url должен начинаться с http:// или https://")
    print("Внимание: измерение отправляет реальные STOP. Ноль SC не доказывает, что колесо стоит.")
    try:
        samples = measure(args.url, args.count)
    except (MeasurementError, KeyError) as exc:
        print(f"Измерение прервано: {exc}", file=sys.stderr)
        return 1
    print("STOP → получены подтверждения (HTTP + опрос + сервер + связь, не физическое торможение):")
    for module in (*MODULES, "all"):
        values = sorted(samples[module])
        p90 = values[max(0, math.ceil(len(values) * 0.9) - 1)]
        print(f"{module:>3}: n={len(values)} median={statistics.median(values):.1f} ms "
              f"p90={p90:.1f} ms max={values[-1]:.1f} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
