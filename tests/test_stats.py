"""Сводка работы (robot_control/stats.py): интеграция телеметрии, рейсы, заряд,
неисправности, аудит замка, файл и сброс. Часы подменяются, поэтому тесты детерминированы."""

import json

import pytest

from robot_control.stats import (
    CHARGE_DEBOUNCE_S,
    LINK_GRACE_S,
    STOP_GRACE_S,
    UNAVAILABLE_METRICS,
    RobotStats,
    StatsSampler,
)


class Clock:
    def __init__(self, t: float = 0.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t


def snap(speed=0.0, watts=0.0, state="покой", link=True, route="склад", motors=None):
    return {
        "source": "master",
        "linkOk": link,
        "speedMps": speed,
        "route": route,
        "battery": {"volts": 48.0 if link else None, "watts": watts, "state": state},
        "motors": motors or [],
    }


def run(stats, seconds, make, start=0.0):
    """Подаёт снимок раз в секунду за указанный интервал."""
    for i in range(int(seconds) + 1):
        stats.sample(make(), now=start + i)
    return start + seconds


# ---------------------------------------------------------------- интеграция
def test_distance_and_discharge_energy_are_integrated():
    stats = RobotStats(None, clock=Clock())
    run(stats, 10, lambda: snap(speed=1.0, watts=100.0, state="разряд"))
    t = stats.summary()["totals"]
    assert t["distanceM"] == pytest.approx(10.0)
    assert t["movingSec"] == pytest.approx(10.0)
    assert t["energyUsedWh"] == pytest.approx(100.0 * 10 / 3600.0)
    assert t["energyChargedWh"] == 0.0
    assert t["maxSpeedMps"] == pytest.approx(1.0)


def test_charge_energy_is_counted_separately():
    stats = RobotStats(None, clock=Clock())
    run(stats, 5, lambda: snap(watts=200.0, state="заряд"))
    t = stats.summary()["totals"]
    assert t["energyChargedWh"] == pytest.approx(200.0 * 5 / 3600.0)
    assert t["energyUsedWh"] == 0.0
    assert t["distanceM"] == 0.0


def test_no_link_means_no_integration_and_no_invented_values():
    stats = RobotStats(None, clock=Clock())
    run(stats, 3, lambda: snap(speed=1.0, watts=100.0, state="разряд", link=False))
    s = stats.summary()
    assert s["totals"]["distanceM"] == 0.0
    assert s["totals"]["energyUsedWh"] == 0.0
    assert s["live"]["linkOk"] is False
    assert s["whPerKm"] is None


def test_bad_snapshot_is_ignored():
    stats = RobotStats(None, clock=Clock())
    stats.sample("не словарь", now=1.0)
    stats.sample({"linkOk": True}, now=2.0)  # без battery — связи нет
    assert stats.summary()["totals"]["distanceM"] == 0.0


def test_missing_speed_does_not_count_distance():
    stats = RobotStats(None, clock=Clock())
    run(stats, 5, lambda: dict(snap(watts=50.0, state="разряд"), speedMps=None))
    assert stats.summary()["totals"]["distanceM"] == 0.0


def test_whpkm_is_computed_only_on_enough_distance():
    stats = RobotStats(None, clock=Clock())
    # 100 с по 10 м/с → 1000 м, 100 Вт × 100 с = 2.78 Вт·ч → ≈2.8 Вт·ч/км
    run(stats, 100, lambda: snap(speed=10.0, watts=100.0, state="разряд"))
    s = stats.summary()
    assert s["distanceKm"] == pytest.approx(1.0, abs=0.01)
    assert s["whPerKm"] == pytest.approx(2.8, abs=0.1)
    assert s["avgSpeedKmh"] == pytest.approx(36.0, abs=0.1)


def test_huge_pause_does_not_stretch_integration_step():
    clock = Clock()
    stats = RobotStats(None, clock=clock)
    stats.sample(snap(speed=1.0, watts=3600.0, state="разряд"), now=0.0)
    stats.sample(snap(speed=1.0, watts=3600.0, state="разряд"), now=1000.0)  # процесс «спал»
    # шаг ограничен MAX_STEP_S = 5 с: 3600 Вт × 5 с = 5 Вт·ч
    assert stats.summary()["totals"]["energyUsedWh"] == pytest.approx(5.0)


# -------------------------------------------------------------------- рейсы
def test_trip_ends_ok_after_stop_grace():
    stats = RobotStats(None, clock=Clock())
    t = run(stats, 5, lambda: snap(speed=1.0, route="зона выгрузки"))
    run(stats, STOP_GRACE_S + 2, lambda: snap(speed=0.0), start=t + 1)
    s = stats.summary()
    assert s["totals"]["trips"] == 1
    assert s["totals"]["tripsAborted"] == 0
    trip = s["recentTrips"][0]
    assert trip["result"] == "ok"
    assert trip["route"] == "зона выгрузки"
    assert trip["avgSpeedMps"] == pytest.approx(1.0)
    assert s["live"]["currentTrip"] is None


def test_trip_is_aborted_when_link_lost_too_long():
    stats = RobotStats(None, clock=Clock())
    t = run(stats, 3, lambda: snap(speed=1.0))
    run(stats, LINK_GRACE_S + 3, lambda: snap(speed=0.0, link=False), start=t + 1)
    s = stats.summary()
    assert s["totals"]["trips"] == 1
    assert s["totals"]["tripsAborted"] == 1
    assert s["recentTrips"][0]["result"] == "aborted"


def test_short_link_drop_keeps_trip_alive():
    stats = RobotStats(None, clock=Clock())
    t = run(stats, 3, lambda: snap(speed=1.0))
    stats.sample(snap(speed=1.0, link=False), now=t + 1)
    stats.sample(snap(speed=1.0), now=t + 2)
    s = stats.summary()
    assert s["totals"]["trips"] == 0
    assert s["live"]["currentTrip"] is not None


def test_trip_history_is_capped():
    stats = RobotStats(None, clock=Clock(), trip_capacity=3)
    t = 0.0
    for _ in range(5):
        t = run(stats, 2, lambda: snap(speed=1.0), start=t + 1)
        t = run(stats, STOP_GRACE_S + 1, lambda: snap(speed=0.0), start=t + 1)
    assert stats.summary()["totals"]["trips"] == 5
    assert len(stats.trips) == 3
    assert [tr["id"] for tr in stats.trips] == [3, 4, 5]


# ------------------------------------------------------------- заряд, модули
def test_charge_cycles_are_debounced():
    stats = RobotStats(None, clock=Clock())
    stats.sample(snap(state="заряд"), now=0.0)
    run(stats, 5, lambda: snap(state="заряд"), start=1.0)
    assert stats.summary()["totals"]["chargeCycles"] == 1
    stats.sample(snap(state="покой"), now=10.0)
    stats.sample(snap(state="заряд"), now=30.0)  # дребезг, меньше минуты
    assert stats.summary()["totals"]["chargeCycles"] == 1
    stats.sample(snap(state="покой"), now=70.0)
    stats.sample(snap(state="заряд"), now=70.0 + 1)  # давно от первого начала
    assert stats.summary()["totals"]["chargeCycles"] == 2
    assert CHARGE_DEBOUNCE_S == 60.0


def test_fault_counts_each_episode_once():
    stats = RobotStats(None, clock=Clock())

    def m(fault):
        return snap(motors=[{"id": "FL", "fault": fault}, {"id": "FR", "fault": False}])

    stats.sample(m(True), now=0.0)
    stats.sample(m(True), now=1.0)
    stats.sample(m(False), now=2.0)
    stats.sample(m(True), now=3.0)
    assert stats.summary()["totals"]["faults"] == 2


# ----------------------------------------------------------------- аудит замка
def test_lock_audit_counts_only_new_entries():
    stats = RobotStats(None, clock=Clock())
    # первый запуск: история не пересчитывается, отмечается хвост
    stats.note_lock_audit([{"ts": 100, "action": "unlock", "ok": True}])
    assert stats.summary()["totals"]["lockOpen"] == 0
    stats.note_lock_audit([
        {"ts": 300, "action": "pin_change", "ok": True},
        {"ts": 200, "action": "unlock", "ok": True},
        {"ts": 150, "action": "unlock", "ok": False},
        {"ts": 50, "action": "unlock", "ok": True},  # старее хвоста — не считаем
        {"ts": 120, "action": "pin_change", "ok": False},
    ])
    t = stats.summary()["totals"]
    assert (t["lockOpen"], t["lockDenied"], t["pinChanged"]) == (1, 1, 1)
    stats.note_lock_audit([{"ts": 300, "action": "unlock", "ok": True}])  # повтор
    assert stats.summary()["totals"]["lockOpen"] == 1


def test_lock_audit_ignores_broken_entries():
    stats = RobotStats(None, clock=Clock())
    stats.note_lock_audit([{"ts": 1, "action": "unlock", "ok": True}])
    stats.note_lock_audit([{"action": "unlock"}, "мусор", {"ts": "x"}])
    assert stats.summary()["totals"]["lockOpen"] == 0


# ------------------------------------------------------------ хранение и сброс
def test_state_survives_restart(tmp_path):
    path = tmp_path / "stats.json"
    clock = Clock()
    first = RobotStats(path, clock=clock)
    run(first, 4, lambda: snap(speed=1.0, watts=100.0, state="разряд"))
    first.flush()
    assert path.is_file()
    second = RobotStats(path, clock=clock)
    assert second.summary()["totals"]["distanceM"] == pytest.approx(first.summary()["totals"]["distanceM"])
    assert second.summary()["totals"]["energyUsedWh"] == pytest.approx(first.summary()["totals"]["energyUsedWh"])


def test_file_is_written_atomically_without_tmp_leftover(tmp_path):
    path = tmp_path / "stats.json"
    stats = RobotStats(path, clock=Clock())
    stats.flush()
    assert json.loads(path.read_text(encoding="utf-8"))["version"] == 1
    assert not (tmp_path / "stats.json.tmp").exists()


def test_corrupt_or_foreign_file_starts_fresh(tmp_path):
    path = tmp_path / "stats.json"
    path.write_text("{не json", encoding="utf-8")
    assert RobotStats(path, clock=Clock()).summary()["totals"]["trips"] == 0
    path.write_text(json.dumps({"version": 99, "totals": {"trips": 7}}), encoding="utf-8")
    assert RobotStats(path, clock=Clock()).summary()["totals"]["trips"] == 0


def test_reset_clears_totals_but_not_lock_journal(tmp_path):
    path = tmp_path / "stats.json"
    stats = RobotStats(path, clock=Clock())
    run(stats, 4, lambda: snap(speed=1.0, watts=100.0, state="разряд"))
    stats.note_lock_audit([{"ts": 1, "action": "unlock", "ok": True}])
    stats.note_lock_audit([{"ts": 2, "action": "unlock", "ok": True}])
    stats.reset()
    s = stats.summary()
    assert s["totals"]["distanceM"] == 0.0
    assert s["totals"]["lockOpen"] == 0
    assert s["recentTrips"] == []
    assert json.loads(path.read_text(encoding="utf-8"))["totals"]["distanceM"] == 0.0


def test_unavailable_metrics_are_listed_not_zeroed():
    s = RobotStats(None, clock=Clock()).summary()
    assert s["unavailable"] == list(UNAVAILABLE_METRICS)
    assert "вес груза (нет датчика)" in s["unavailable"]
    assert "weight" not in json.dumps(s["totals"]).lower()


# ------------------------------------------------------------------- выгрузка
def test_csv_and_json_reports_match_summary():
    stats = RobotStats(None, clock=Clock())
    run(stats, 4, lambda: snap(speed=1.0, watts=100.0, state="разряд"))
    csv_text = stats.to_csv()
    assert "Пробег, м" in csv_text
    assert "Энергия на разряд, Вт·ч" in csv_text
    doc = json.loads(stats.to_json())
    assert doc["summary"]["totals"]["distanceM"] == pytest.approx(4.0)
    assert doc["unavailable"] == list(UNAVAILABLE_METRICS)
    assert "НТЦ" in doc["device"]


# ------------------------------------------------------------------- sampler
def test_sampler_tick_survives_failures():
    stats = RobotStats(None, clock=Clock())

    def broken():
        raise RuntimeError("источник упал")

    def audit_broken(*_):
        raise RuntimeError("журнал недоступен")

    sampler = StatsSampler(stats, snapshot=broken, audit=lambda: [], interval=1.0)
    sampler.tick()  # не должно бросить исключение
    sampler2 = StatsSampler(stats, snapshot=lambda: snap(speed=1.0), audit=audit_broken, interval=1.0)
    sampler2.tick()
    # снимок с напряжением принят, сбой журнала замка не остановил сводку
    assert stats.summary()["live"]["linkOk"] is True
    assert stats.summary()["totals"]["distanceM"] == 0.0  # первый снимок — точка отсчёта
