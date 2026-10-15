"""Сводка работы робота: рейсы, пробег, энергия, замок и неисправности.

Всё считается только из реальной телеметрии ``snapshot()`` (источник данных:
UART-мастер, модули, ROS 2 или соседний процесс) и из журнала аудита замка.
Если источник не отдаёт величину, она не выдумывается: в сводке она честно
помечается как недоступная (см. :data:`UNAVAILABLE_METRICS`).

Что считается:
  * пробег — интеграл ``speedMps`` по времени;
  * энергия — интеграл ``battery.watts`` отдельно на разряд и на заряд;
  * рейс — одно движение: начинается с первого движения после стоянки и
    завершается, когда робот стоит дольше :data:`STOP_GRACE_S` секунд
    (итог ``ok``) или когда связь с бортом пропала дольше
    :data:`LINK_GRACE_S` (итог ``aborted``);
  * циклы заряда — переходы в состояние «заряд» с защитой от дребезга;
  * неисправности — переход любого модуля в ``fault``;
  * замок — новые записи аудита (открытие, отказ, смена PIN) с момента
    запуска сводки.

Данные хранятся в JSON (по умолчанию ``data/stats.json``, каталог ``data/``
не попадает в git). Запись атомарная: временный файл и ``os.replace``.
"""

from __future__ import annotations

import csv
import io
import json
import logging
import os
import threading
import time
from pathlib import Path
from typing import Callable, Iterable, Optional

log = logging.getLogger("robot_control.stats")

STATS_VERSION = 1
#: Ниже этой скорости (м/с) робот считается стоящим.
MOVING_MPS = 0.05
#: Сколько секунд стоянки завершают рейс.
STOP_GRACE_S = 10.0
#: Сколько секунд без связи с бортом рейс считается прерванным.
LINK_GRACE_S = 5.0
#: Повторный «цикл заряда» не чаще, чем раз в эти секунды.
CHARGE_DEBOUNCE_S = 60.0
#: Максимальный шаг интеграции: после паузы процесса шаг не растягивается.
MAX_STEP_S = 5.0
#: Периодичность записи на диск, если данные менялись.
SAVE_INTERVAL_S = 15.0
#: Сколько последних рейсов хранить в файле.
TRIP_CAPACITY = 200

#: Показатели, для которых в телеметрии робота пока нет источника.
#: Их нет в сводке как цифр — только как честная пометка «нет данных».
UNAVAILABLE_METRICS = (
    "вес груза (нет датчика)",
    "объезды препятствий (нет события от перцепции)",
    "знаки и светофор (нет события от перцепции)",
    "ручные сервисные команды (журнал пульта, не борт)",
)


def _empty_totals() -> dict:
    return {
        "trips": 0,
        "tripsAborted": 0,
        "distanceM": 0.0,
        "energyUsedWh": 0.0,
        "energyChargedWh": 0.0,
        "movingSec": 0.0,
        "maxSpeedMps": 0.0,
        "chargeCycles": 0,
        "faults": 0,
        "lockOpen": 0,
        "lockDenied": 0,
        "pinChanged": 0,
    }


class RobotStats:
    """Накопитель сводки. Потокобезопасен: sampler и HTTP-запросы работают параллельно."""

    def __init__(
        self,
        path: Optional[str | Path] = None,
        *,
        clock: Callable[[], float] = time.time,
        trip_capacity: int = TRIP_CAPACITY,
    ) -> None:
        self.path = Path(path) if path else None
        self.clock = clock
        self.trip_capacity = max(1, int(trip_capacity))
        self._lock = threading.RLock()
        self.data = self._load()
        # Состояние интеграции — только в памяти: после перезапуска сессия новая.
        self._last_t: Optional[float] = None
        self._last_valid_t: Optional[float] = None
        self._idle_since: Optional[float] = None
        self._trip: Optional[dict] = None
        self._charging = False
        self._last_charge_start: Optional[float] = None
        self._fault_active = False
        self._dirty = False
        self._last_save = 0.0
        self._last_sample_at: Optional[float] = None
        self._link_ok = False
        self._source = None

    # ------------------------------------------------------------ хранение
    def _fresh(self) -> dict:
        now = self.clock()
        return {
            "version": STATS_VERSION,
            "since": now,
            "updatedAt": now,
            "totals": _empty_totals(),
            "trips": [],
            "nextTripId": 1,
            "auditSeenTs": None,
        }

    def _load(self) -> dict:
        if not self.path or not self.path.is_file():
            return self._fresh()
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            log.warning("сводка: файл %s не прочитан (%s) — начинаю заново", self.path, exc)
            return self._fresh()
        if not isinstance(raw, dict) or raw.get("version") != STATS_VERSION:
            log.warning("сводка: неизвестная версия файла %s — начинаю заново", self.path)
            return self._fresh()
        fresh = self._fresh()
        totals = _empty_totals()
        totals.update({k: v for k, v in (raw.get("totals") or {}).items() if k in totals})
        fresh["totals"] = totals
        trips = raw.get("trips")
        fresh["trips"] = [t for t in trips if isinstance(t, dict)] if isinstance(trips, list) else []
        fresh["since"] = raw.get("since") or fresh["since"]
        fresh["nextTripId"] = int(raw.get("nextTripId") or (len(fresh["trips"]) + 1))
        fresh["auditSeenTs"] = raw.get("auditSeenTs")
        return fresh

    def save(self, force: bool = False) -> None:
        with self._lock:
            now = self.clock()
            if not force and not self._dirty:
                return
            if not force and now - self._last_save < SAVE_INTERVAL_S:
                return
            self.data["updatedAt"] = now
            self._last_save = now
            self._dirty = False
            if not self.path:
                return
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                tmp = self.path.with_suffix(self.path.suffix + ".tmp")
                tmp.write_text(json.dumps(self.data, ensure_ascii=False, indent=1), encoding="utf-8")
                os.replace(tmp, self.path)
            except OSError as exc:
                log.error("сводка: не удалось записать %s: %s", self.path, exc)

    def flush(self) -> None:
        self.save(force=True)

    # ------------------------------------------------------------ рейсы
    def _begin_trip(self, now: float, route: str) -> None:
        self._trip = {
            "id": self.data["nextTripId"],
            "startedAt": now,
            "endedAt": None,
            "route": route,
            "distanceM": 0.0,
            "energyUsedWh": 0.0,
            "movingSec": 0.0,
            "maxSpeedMps": 0.0,
            "avgSpeedMps": 0.0,
            "result": None,
            "reason": "",
        }
        self.data["nextTripId"] += 1

    def _end_trip(self, now: float, result: str, reason: str = "") -> None:
        trip = self._trip
        self._trip = None
        self._idle_since = None
        if trip is None:
            return
        trip["endedAt"] = now
        trip["result"] = result
        trip["reason"] = reason
        if trip["movingSec"] > 0.5:
            trip["avgSpeedMps"] = trip["distanceM"] / trip["movingSec"]
        totals = self.data["totals"]
        totals["trips"] += 1
        if result != "ok":
            totals["tripsAborted"] += 1
        trips = self.data["trips"]
        trips.append(trip)
        if len(trips) > self.trip_capacity:
            del trips[: len(trips) - self.trip_capacity]
        self._dirty = True
        self.save(force=True)

    # ------------------------------------------------------------ выборка
    def sample(self, snap: dict, now: Optional[float] = None) -> None:
        """Учесть один снимок телеметрии. Вызывается раз в секунду из sampler."""
        if not isinstance(snap, dict):
            return
        with self._lock:
            now = self.clock() if now is None else float(now)
            dt = 0.0 if self._last_t is None else min(max(now - self._last_t, 0.0), MAX_STEP_S)
            self._last_t = now
            self._last_sample_at = now
            self._source = snap.get("source")

            battery = snap.get("battery") if isinstance(snap.get("battery"), dict) else {}
            volts = battery.get("volts")
            link_ok = bool(snap.get("linkOk")) and isinstance(volts, (int, float))
            self._link_ok = link_ok

            if not link_ok:
                # Связь пропала: данные не интегрируем, рейс живёт до LINK_GRACE_S.
                if self._trip and self._last_valid_t is not None and now - self._last_valid_t >= LINK_GRACE_S:
                    self._end_trip(now, "aborted", "связь с бортом пропала")
                return

            self._last_valid_t = now
            totals = self.data["totals"]
            trip = self._trip

            # Энергия: разряд и заряд раздельно.
            watts = battery.get("watts")
            state = battery.get("state")
            if isinstance(watts, (int, float)) and dt > 0:
                wh = float(watts) * dt / 3600.0
                if state == "разряд":
                    totals["energyUsedWh"] += wh
                    if trip:
                        trip["energyUsedWh"] += wh
                    self._dirty = True
                elif state == "заряд":
                    totals["energyChargedWh"] += wh
                    self._dirty = True

            # Циклы заряда: переход в «заряд» с защитой от дребезга.
            charging = state == "заряд"
            if charging and not self._charging:
                if self._last_charge_start is None or now - self._last_charge_start >= CHARGE_DEBOUNCE_S:
                    totals["chargeCycles"] += 1
                    self._last_charge_start = now
                    self._dirty = True
            self._charging = charging

            # Неисправности: учитываем переход «нет неисправности → есть».
            motors = snap.get("motors") if isinstance(snap.get("motors"), list) else []
            fault_now = any(bool(m.get("fault")) for m in motors if isinstance(m, dict))
            if fault_now and not self._fault_active:
                totals["faults"] += 1
                self._dirty = True
            self._fault_active = fault_now

            # Движение, пробег, рейсы.
            speed = snap.get("speedMps")
            if isinstance(speed, (int, float)) and speed >= 0:
                speed = float(speed)
                moving = speed > MOVING_MPS
                if dt > 0:
                    totals["distanceM"] += speed * dt
                    if moving:
                        totals["movingSec"] += dt
                    self._dirty = True
                totals["maxSpeedMps"] = max(totals["maxSpeedMps"], speed)
                if moving:
                    self._idle_since = None
                    if self._trip is None:
                        route = snap.get("route")
                        self._begin_trip(now, str(route) if isinstance(route, str) and route.strip() else "—")
                    trip = self._trip
                    trip["distanceM"] += speed * dt
                    if dt > 0:
                        trip["movingSec"] += dt
                    trip["maxSpeedMps"] = max(trip["maxSpeedMps"], speed)
                    self._dirty = True
                elif trip is not None:
                    if self._idle_since is None:
                        self._idle_since = now
                    elif now - self._idle_since >= STOP_GRACE_S:
                        self._end_trip(now, "ok")

            self.save(force=False)

    def note_lock_audit(self, entries: Iterable[dict]) -> None:
        """Учесть новые записи аудита замка (entries — от новых к старым)."""
        with self._lock:
            items = [e for e in entries if isinstance(e, dict) and isinstance(e.get("ts"), (int, float))]
            if not items:
                return
            seen = self.data.get("auditSeenTs")
            if seen is None:
                # Первый запуск сводки: отмечаем текущий хвост журнала и не
                # пересчитываем историю, которой не вели.
                self.data["auditSeenTs"] = max(float(e["ts"]) for e in items)
                self._dirty = True
                return
            totals = self.data["totals"]
            newest = float(seen)
            for entry in items:
                ts = float(entry["ts"])
                if ts <= float(seen):
                    continue
                newest = max(newest, ts)
                action = entry.get("action")
                if action == "unlock":
                    if entry.get("ok"):
                        totals["lockOpen"] += 1
                    else:
                        totals["lockDenied"] += 1
                elif action == "pin_change" and entry.get("ok"):
                    totals["pinChanged"] += 1
            if newest > float(seen):
                self.data["auditSeenTs"] = newest
                self._dirty = True
            self.save(force=False)

    # ------------------------------------------------------------ выдача
    def summary(self) -> dict:
        with self._lock:
            t = self.data["totals"]
            km = t["distanceM"] / 1000.0
            trips = self.data["trips"]
            return {
                "since": self.data.get("since"),
                "updatedAt": self.data.get("updatedAt"),
                "live": {
                    "linkOk": self._link_ok,
                    "lastSampleAt": self._last_sample_at,
                    "source": self._source,
                    "tripActive": self._trip is not None,
                    "currentTrip": (
                        {"id": self._trip["id"], "route": self._trip["route"],
                         "distanceM": round(self._trip["distanceM"], 1)}
                        if self._trip else None
                    ),
                },
                "totals": dict(t),
                "distanceKm": round(km, 3),
                "avgSpeedKmh": round((t["distanceM"] / t["movingSec"]) * 3.6, 2) if t["movingSec"] > 1 else 0.0,
                "whPerKm": round(t["energyUsedWh"] / km, 1) if km > 0.05 else None,
                "recentTrips": list(reversed(trips[-20:])),
                "energySeries": [round(tr.get("energyUsedWh", 0.0), 2) for tr in trips[-20:]],
                "unavailable": list(UNAVAILABLE_METRICS),
            }

    @property
    def trips(self) -> list:
        with self._lock:
            return list(self.data["trips"])

    def to_csv(self) -> str:
        s = self.summary()
        t = s["totals"]
        out = io.StringIO()
        writer = csv.writer(out, delimiter=";", lineterminator="\n")
        writer.writerow(["# RUS SLAM — сводка работы робота (реальная телеметрия)"])
        writer.writerow(["# сформировано", time.strftime("%Y-%m-%dT%H:%M:%S")])
        writer.writerow([])
        writer.writerow(["Показатель", "Значение"])
        writer.writerow(["Рейсов", t["trips"]])
        writer.writerow(["Прервано рейсов", t["tripsAborted"]])
        writer.writerow(["Пробег, м", f"{t['distanceM']:.1f}"])
        writer.writerow(["Энергия на разряд, Вт·ч", f"{t['energyUsedWh']:.1f}"])
        writer.writerow(["Энергия на заряд, Вт·ч", f"{t['energyChargedWh']:.1f}"])
        writer.writerow(["Время в движении, с", f"{t['movingSec']:.0f}"])
        writer.writerow(["Средняя скорость, км/ч", f"{s['avgSpeedKmh']:.2f}"])
        writer.writerow(["Максимальная скорость, м/с", f"{t['maxSpeedMps']:.2f}"])
        writer.writerow(["Расход, Вт·ч/км", "" if s["whPerKm"] is None else f"{s['whPerKm']:.1f}"])
        writer.writerow(["Циклов заряда", t["chargeCycles"]])
        writer.writerow(["Неисправностей модулей", t["faults"]])
        writer.writerow(["Открытий замка", t["lockOpen"]])
        writer.writerow(["Отказов доступа", t["lockDenied"]])
        writer.writerow(["Смен PIN", t["pinChanged"]])
        writer.writerow([])
        writer.writerow(["Нет источника данных (не выдумано):"])
        for item in UNAVAILABLE_METRICS:
            writer.writerow([item])
        writer.writerow([])
        writer.writerow(["# Рейсы"])
        writer.writerow(["id", "начало", "окончание", "маршрут", "дистанция_м", "энергия_Втч",
                         "движение_с", "ср_скорость_мс", "макс_скорость_мс", "результат", "причина"])
        for tr in self.trips:
            writer.writerow([
                tr["id"], _iso(tr["startedAt"]), _iso(tr.get("endedAt")), tr.get("route", ""),
                f"{tr['distanceM']:.1f}", f"{tr['energyUsedWh']:.1f}", f"{tr['movingSec']:.0f}",
                f"{tr.get('avgSpeedMps', 0.0):.2f}", f"{tr['maxSpeedMps']:.2f}",
                tr.get("result") or "", tr.get("reason") or "",
            ])
        return out.getvalue()

    def to_json(self) -> str:
        return json.dumps(
            {
                "generatedAt": _iso(self.clock()),
                "device": "RUS SLAM 4WIS/4WID, НТЦ АО «АВТОВАЗ»",
                "summary": self.summary(),
                "trips": self.trips,
                "unavailable": list(UNAVAILABLE_METRICS),
            },
            ensure_ascii=False,
            indent=2,
        )

    def reset(self) -> None:
        """Обнулить накопленное. Журнал аудита замка сам по себе не трогаем."""
        with self._lock:
            self.data = self._fresh()
            self._trip = None
            self._idle_since = None
            self._charging = False
            self._fault_active = False
            self._dirty = True
            self.save(force=True)


def _iso(ts: Optional[float]) -> str:
    if not isinstance(ts, (int, float)):
        return ""
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(ts))


class StatsSampler:
    """Фоновый поток: раз в секунду снимает телеметрию и передаёт её сводке."""

    def __init__(
        self,
        stats: RobotStats,
        snapshot: Callable[[], dict],
        audit: Optional[Callable[[], list]] = None,
        interval: float = 1.0,
    ) -> None:
        self.stats = stats
        self.snapshot = snapshot
        self.audit = audit
        self.interval = max(0.2, float(interval))
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="rc-stats", daemon=True)

    def start(self) -> "StatsSampler":
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)
        self.stats.flush()

    def tick(self) -> None:
        try:
            snap = self.snapshot()
            self.stats.sample(snap)
        except Exception:  # noqa: BLE001 — сбой одного снимка не должен гасить сводку
            log.exception("сводка: снимок телеметрии не получен")
        if self.audit is not None:
            try:
                self.stats.note_lock_audit(self.audit())
            except Exception:  # noqa: BLE001
                log.exception("сводка: журнал замка не прочитан")

    def _run(self) -> None:
        while not self._stop.wait(self.interval):
            self.tick()
