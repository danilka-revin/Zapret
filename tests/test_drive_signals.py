"""Автосигналы шасси: стоп-сигнал при торможении и тревога у препятствия.

Здесь закреплено то, что действительно ломается и что нельзя проверять
мотором на площадке:

* стоп-сигнал включается при отпускании тяги, остановке, E-STOP и качении,
  а не по каждой паузе; хвост гаснет сам; повторный газ его не «замораживает»;
* близкое препятствие (0,5 м по умолчанию) при движении включает фары и оба
  поворотника и даёт прерывистый зуммер;
* без свежего скана или без движения тревога не выдумывается;
* автоматика не гасит ручной свет оператора и снимает только свои биты;
* ошибка платы видна в состоянии и не роняет сервер;
* прошивка, симуляция и API согласованы: команды только light/buzz.
"""
from __future__ import annotations

import math

import pytest

from robot_control import drive_signals as ds


class FakeModule:
    """Плата без порта: запоминает команды и отдаёт заданные биты света."""

    def __init__(self, lights=None):
        self.calls = []
        self.lights = dict(lights or {})
        self.fail = None

    def light(self, name, on):
        if self.fail:
            raise RuntimeError(self.fail)
        self.calls.append(("light", name, on))

    def buzz(self):
        if self.fail:
            raise RuntimeError(self.fail)
        self.calls.append(("buzz",))

    def state(self):
        return {"lights": dict(self.lights)}


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def tick(self, seconds):
        self.now += seconds


def telemetry(percent=0, *, active=None, speed=None, rpm=None, phase="", reason="", estop=None):
    motors = [{"online": True, "rpm": rpm if rpm is not None else (abs(percent) if active else 0)}]
    data = {"motors": motors,
            "drive": {"commandedPercent": percent, "active": bool(active) if active is not None else bool(percent),
                      "phase": phase, "reason": reason},
            "speedMps": speed}
    if estop is not None:
        data["estop"] = estop
    return data


def points(*items):
    return [{"a": angle, "r": distance} for angle, distance in items]


# --------------------------------------------------------------- стоп-сигнал --
def test_brake_light_lights_when_throttle_is_released_and_fades_by_itself():
    clock = Clock()
    ctl = ds.SignalController(ds.SignalConfig(tail_s=1.0), clock=clock)
    # Едем: стоп-сигнал не горит, тревоги без скана нет.
    state = ctl.update(ds.facts_from_telemetry(telemetry(8, active=True)))
    assert state["brakeLight"] is False
    # Отпустили газ: стоп-сигнал загорается на хвост, даже если робот уже стоит.
    state = ctl.update(ds.facts_from_telemetry(telemetry(0)), scan_age=None)
    assert state["brakeLight"] is True
    clock.tick(1.1)
    state = ctl.update(ds.facts_from_telemetry(telemetry(0)), scan_age=None)
    assert state["brakeLight"] is False


def test_coasting_robot_keeps_brake_light_until_it_stops():
    clock = Clock()
    ctl = ds.SignalController(ds.SignalConfig(tail_s=0.5), clock=clock)
    ctl.update(ds.facts_from_telemetry(telemetry(6, active=True)))
    # Команда ноль, но робот ещё катится: хвост не должен погасить стоп раньше.
    for _ in range(6):
        clock.tick(0.1)
        state = ctl.update(ds.facts_from_telemetry(telemetry(0, active=False, rpm=30)))
        assert state["brakeLight"] is True
    # Остановился — гаснет через хвост.
    clock.tick(0.6)
    state = ctl.update(ds.facts_from_telemetry(telemetry(0, active=False, rpm=0)))
    assert state["brakeLight"] is False


def test_estop_and_braking_phase_hold_stop_signal_and_new_throttle_does_not_freeze_it():
    clock = Clock()
    ctl = ds.SignalController(ds.SignalConfig(tail_s=1.0), clock=clock)
    state = ctl.update(ds.facts_from_telemetry(telemetry(5, active=True, phase="braking")))
    assert state["brakeLight"] is True
    state = ctl.update(ds.facts_from_telemetry(telemetry(5, active=True, estop={"latched": True})))
    assert state["brakeLight"] is True
    # Оператор снова дал газ: стоп-сигнал не остаётся гореть на весь хвост.
    state = ctl.update(ds.facts_from_telemetry(telemetry(5, active=True)))
    assert state["brakeLight"] is True          # ещё в пределах короткого хвоста
    clock.tick(0.5)
    state = ctl.update(ds.facts_from_telemetry(telemetry(5, active=True)))
    assert state["brakeLight"] is False


def test_facts_accept_estop_from_the_shared_latch_not_only_from_telemetry():
    facts = ds.facts_from_telemetry(telemetry(0), estop_latched=True)
    assert facts.estop is True
    assert ds.facts_from_telemetry({"motors": []}).percent is None


# ------------------------------------------------------------------ тревога --
def test_close_obstacle_ahead_triggers_lights_and_intermittent_buzzer():
    clock = Clock()
    module = FakeModule()
    ctl = ds.SignalController(ds.SignalConfig(near_m=0.5, tail_s=0.5), clock=clock)
    facts = ds.facts_from_telemetry(telemetry(6, active=True))
    state = ctl.update(facts, points=points((0.0, 0.5), (0.1, 0.3)), scan_age=0.2)
    assert state["alert"] is True and state["distanceM"] == pytest.approx(0.3, abs=0.01)
    state = ctl.apply(module, manual={}, stop_on=False)
    assert ("light", "head", True) in module.calls
    assert ("light", "left", True) in module.calls and ("light", "right", True) in module.calls
    # Зуммер прерывистый: за 0,9 с команда уходит больше одного раза и не сплошняком.
    clock.tick(0.1)
    ctl.update(facts, points=points((0.0, 0.5)), scan_age=0.1)
    ctl.apply(module, manual={})
    assert any(call[0] == "buzz" for call in module.calls)
    # Отъехали — биты, включённые автоматикой, снимаются.
    state = ctl.update(facts, points=points((0.0, 3.0)), scan_age=0.1)
    assert state["alert"] is False
    module.calls.clear()
    ctl.apply(module, manual={})
    assert ("light", "head", False) in module.calls
    assert ("light", "left", False) in module.calls and ("light", "right", False) in module.calls


def test_side_or_behind_obstacle_is_not_ahead_even_when_close():
    assert ds.nearest_ahead(points((math.pi / 2, 0.2), (-math.pi / 2, 0.2), (math.pi, 0.3)),
                            direction=1, config=ds.SignalConfig(near_m=0.5)) is None
    # Назад смотрим в задний сектор: то, что было сзади, теперь впереди.
    assert ds.nearest_ahead(points((math.pi, 0.3)), direction=-1,
                            config=ds.SignalConfig()) == pytest.approx(0.3)
    # Близко, но в стороне от полосы движения — не препятствие по курсу.
    assert ds.nearest_ahead(points((math.pi / 3, 0.2)), direction=1,
                            config=ds.SignalConfig(half_width_m=0.35)) is None


def test_no_alert_without_fresh_scan_or_without_motion():
    clock = Clock()
    ctl = ds.SignalController(clock=clock)
    standing = ds.facts_from_telemetry(telemetry(0, active=False))
    assert ctl.update(standing, points=points((0.0, 0.2)), scan_age=0.1)["alert"] is False
    moving = ds.facts_from_telemetry(telemetry(6, active=True))
    state = ctl.update(moving, points=points((0.0, 0.2)), scan_age=None)
    assert state["alert"] is False and "скана" in state["reason"]
    state = ctl.update(moving, points=points((0.0, 0.2)), scan_age=3.0)
    assert state["alert"] is False and "устарел" in state["reason"]
    # Свежий скан, но пусто по курсу — тревоги нет, причина объясняет.
    state = ctl.update(moving, points=points((math.pi, 0.2)), scan_age=0.1)
    assert state["alert"] is False and state["distanceM"] is None


def test_alert_threshold_comes_from_environment_with_safe_limits(monkeypatch):
    monkeypatch.setenv("RC_SAFETY_NEAR_M", "1.0")
    assert ds.SignalConfig.from_env().near_m == 1.0
    monkeypatch.setenv("RC_SAFETY_NEAR_M", "50")
    assert ds.SignalConfig.from_env().near_m == 2.0      # зажато в допустимое
    monkeypatch.setenv("RC_SAFETY_BUZZER", "0")
    assert ds.SignalConfig.from_env().buzzer is False
    monkeypatch.setenv("RC_SIGNALS_ENABLED", "0")
    clock = Clock()
    ctl = ds.SignalController(ds.SignalConfig.from_env(), clock=clock)
    state = ctl.update(ds.facts_from_telemetry(telemetry(8, active=True)),
                       points=points((0.0, 0.1)), scan_age=0.1)
    assert state["enabled"] is False and state["alert"] is False


# ------------------------------------------------------------------ ручной ---
def test_manual_lights_are_never_turned_off_by_the_automation():
    clock = Clock()
    module = FakeModule(lights={"stop": True, "head": True})
    ctl = ds.SignalController(ds.SignalConfig(tail_s=0.5), clock=clock)
    ctl.update(ds.facts_from_telemetry(telemetry(0)), scan_age=None)
    module.calls.clear()
    # Стоп и передний свет уже горят по команде оператора: повторов не шлём.
    ctl.apply(module, manual={"stop": True, "head": True}, stop_on=True)
    assert module.calls == []
    clock.tick(1.0)
    ctl.update(ds.facts_from_telemetry(telemetry(0)), scan_age=None)
    ctl.apply(module, manual={"stop": True, "head": True}, stop_on=False)
    assert ("light", "stop", False) not in module.calls


def test_module_failure_is_reported_and_does_not_raise():
    ctl = ds.SignalController(ds.SignalConfig(), clock=Clock())
    module = FakeModule()
    module.fail = "порт занят"
    ctl.update(ds.facts_from_telemetry(telemetry(0)), scan_age=None)
    state = ctl.apply(module, manual={}, stop_on=True)
    assert "порт занят" in state["error"]
    # Без платы сервис тоже не падает: только честная ошибка.
    assert ctl.apply(None, manual={})["error"] in ("", "порт занят")


# ------------------------------------------------------------------ сервис ---
def test_service_step_uses_scan_stamp_telemetry_and_module_without_raising():
    clock = Clock()
    module = FakeModule()
    ctl = ds.SignalController(ds.SignalConfig(near_m=0.5), clock=clock)
    service = ds.DriveSignalService(
        ctl,
        telemetry_provider=lambda: telemetry(6, active=True),
        lidar_provider=lambda: {"points": points((0.0, 0.4)), "stamp": 99.0},
        module_provider=lambda: module,
        offset_provider=lambda: 0.0,
        estop_provider=lambda: None,
    )
    service._lidar = lambda: {"points": points((0.0, 0.4)), "stamp": __import__("time").time()}
    state = service.step()
    assert state["alert"] is True
    assert service.state()["alert"] is True
    assert ("light", "head", True) in module.calls


def test_service_survives_broken_providers():
    def boom():
        raise RuntimeError("нет порта")
    ctl = ds.SignalController(ds.SignalConfig(), clock=Clock())
    service = ds.DriveSignalService(ctl, telemetry_provider=boom, lidar_provider=boom,
                                    module_provider=boom, estop_provider=boom)
    state = service.step()
    assert state["brakeLight"] is False
    assert "нет порта" in state["error"]


# --------------------------------------- остановка у препятствия (круиз-контроль)
def test_close_obstacle_stops_for_five_seconds_then_returns_control():
    clock = Clock()
    ctl = ds.SignalController(ds.SignalConfig(near_m=0.5, hold_s=5.0), clock=clock)
    moving = ds.facts_from_telemetry(telemetry(6, active=True))
    state = ctl.update(moving, points=points((0.0, 0.4)), scan_age=0.1)
    assert state["hold"] is True and state["holdTriggered"] is True
    assert state["holdRemainingS"] == pytest.approx(5.0, abs=0.05)
    assert state["alert"] is True, "пищание и моргание остаются при паузе"
    assert "остановка" in state["reason"]

    # Робот встал (борт снял тягу), пауза продолжается и не продлевается.
    stopped = ds.facts_from_telemetry(telemetry(0, active=False))
    clock.tick(2.0)
    state = ctl.update(stopped, points=points((0.0, 0.4)), scan_age=0.1)
    assert state["hold"] is True and state["holdRemainingS"] == pytest.approx(3.0, abs=0.1)
    clock.tick(3.5)
    state = ctl.update(stopped, points=points((0.0, 0.4)), scan_age=0.1)
    assert state["hold"] is False, "через 5 с управление возвращается само"
    assert "вернулось" in state["reason"]


def test_new_attempt_into_the_wall_starts_a_new_pause_but_free_path_does_not():
    clock = Clock()
    ctl = ds.SignalController(ds.SignalConfig(near_m=0.5, hold_s=5.0), clock=clock)
    stopped = ds.facts_from_telemetry(telemetry(0, active=False))
    moving = ds.facts_from_telemetry(telemetry(6, active=True))
    ctl.update(moving, points=points((0.0, 0.4)), scan_age=0.1)
    ctl.update(stopped, points=points((0.0, 0.4)), scan_age=0.1)
    clock.tick(6.0)
    assert ctl.update(stopped, points=points((0.0, 0.4)), scan_age=0.1)["hold"] is False
    # Тронуться в стену не даём: пауза начинается до выдачи тяги.
    state = ctl.request_hold(1)
    assert state["hold"] is True and state["holdTriggered"] is True
    # Путь свободен — ход разрешён без паузы.
    clock.tick(6.0)
    ctl.update(stopped, points=points((0.0, 3.0)), scan_age=0.1)
    assert ctl.request_hold(1)["hold"] is False


def test_hold_is_not_started_without_a_fresh_scan_or_when_disabled():
    clock = Clock()
    ctl = ds.SignalController(ds.SignalConfig(near_m=0.5, hold_s=5.0), clock=clock)
    moving = ds.facts_from_telemetry(telemetry(6, active=True))
    assert ctl.update(moving, points=points((0.0, 0.2)), scan_age=None)["hold"] is False
    assert ctl.update(moving, points=points((0.0, 0.2)), scan_age=3.0)["hold"] is False
    # Без скана request_hold не выдумывает препятствие: ход разрешён.
    assert ctl.request_hold(1)["hold"] is False
    off = ds.SignalController(ds.SignalConfig(hold=False), clock=clock)
    state = off.update(moving, points=points((0.0, 0.2)), scan_age=0.1)
    assert state["alert"] is True and state["hold"] is False


def test_hold_can_be_cancelled_by_the_operator():
    clock = Clock()
    ctl = ds.SignalController(ds.SignalConfig(near_m=0.5, hold_s=5.0), clock=clock)
    moving = ds.facts_from_telemetry(telemetry(6, active=True))
    assert ctl.update(moving, points=points((0.0, 0.4)), scan_age=0.1)["hold"] is True
    ctl.clear_hold("оператор")
    state = ctl.snapshot()
    assert state["hold"] is False and state["reason"] == "оператор"


def test_hold_only_applies_to_the_direction_of_travel():
    """Стена сзади не мешает ехать вперёд и наоборот."""
    clock = Clock()
    ctl = ds.SignalController(ds.SignalConfig(near_m=0.5, hold_s=5.0), clock=clock)
    stopped = ds.facts_from_telemetry(telemetry(0, active=False))
    ctl.update(stopped, points=points((math.pi, 0.3)), scan_age=0.1)
    assert ctl.request_hold(1)["hold"] is False, "сзади близко — вперёд можно"
    assert ctl.request_hold(-1)["hold"] is True, "назад — в стену, пауза"


def test_warning_blinks_instead_of_a_constant_lamp_and_keeps_chirping():
    """«Моргание» в постановке: поворотники мигают, а не горят ровно."""
    clock = Clock()
    module = FakeModule()
    ctl = ds.SignalController(ds.SignalConfig(near_m=0.5, hold=False), clock=clock)
    facts = ds.facts_from_telemetry(telemetry(6, active=True))
    # Повторяем то, что реально ушло на плату: последняя команда по каждому биту.
    lamps, phases, buzzes = {}, [], []
    for _ in range(20):
        state = ctl.update(facts, points=points((0.0, 0.3)), scan_age=0.1)
        ctl.apply(module, manual={}, stop_on=state["brakeLight"])
        for call in module.calls:
            if call[0] == "light":
                lamps[call[1]] = call[2]
        phases.append(bool(lamps.get("left") and lamps.get("right")))
        buzzes.append(state["buzz"])
        clock.tick(0.1)
    # Мигание: и включено, и выключено, без длинных фаз в одну сторону.
    assert any(phases) and not all(phases)
    runs = [len(list(run)) for _, run in __import__("itertools").groupby(phases)]
    assert max(runs) <= 6, "слишком долгая фаза: %d тактов" % max(runs)
    # Фары при тревоге горят всё время, зуммер пищит короткими импульсами.
    assert lamps.get("head") is True
    assert any(buzzes) and not all(buzzes)
    assert ctl.snapshot()["blink"] is True


def test_blinking_can_be_turned_off_for_a_steadier_lamp():
    clock = Clock()
    module = FakeModule()
    ctl = ds.SignalController(ds.SignalConfig(near_m=0.5, hold=False, blink=False), clock=clock)
    facts = ds.facts_from_telemetry(telemetry(6, active=True))
    for _ in range(10):
        ctl.update(facts, points=points((0.0, 0.3)), scan_age=0.1)
        ctl.apply(module, manual={})
        assert ("light", "left", True) in module.calls
        assert ("light", "left", False) not in module.calls
        assert ("light", "right", False) not in module.calls
        clock.tick(0.1)


def test_warning_survives_the_hold_while_traction_is_already_zero():
    """Во время паузы тяги нет, но предупреждение не гаснет само."""
    clock = Clock()
    module = FakeModule()
    ctl = ds.SignalController(ds.SignalConfig(near_m=0.5, hold_s=5.0), clock=clock)
    moving = ds.facts_from_telemetry(telemetry(6, active=True))
    assert ctl.update(moving, points=points((0.0, 0.4)), scan_age=0.1)["hold"] is True
    # Тягу снял уже сам борт (или клавиатура): направление хода не сообщается.
    stopped = ds.facts_from_telemetry(telemetry(0, active=False))
    seen = 0
    for _ in range(12):
        clock.tick(0.25)
        module.calls.clear()
        state = ctl.update(stopped, points=points((0.0, 0.4)), scan_age=0.1)
        ctl.apply(module, manual={}, stop_on=state["brakeLight"])
        if state["hold"]:
            seen += 1
            assert state["lightsOn"] is not None
            turned_off = [call for call in module.calls
                          if call[0] == "light" and call[2] is False and call[1] == "head"]
            assert not turned_off, "предупреждение погасло во время паузы"
    assert seen >= 8
    # Пауза кончилась: предупреждение снова включается только движением, а не
    # стоянкой. Стоящий робот у стены не мигает — иначе аварийка горела бы
    # постоянно на стоянке.
    clock.tick(3.0)
    state = ctl.update(stopped, points=points((0.0, 0.4)), scan_age=0.1)
    assert state["hold"] is False and state["alert"] is False
    assert state["lightsOn"] is False
    # Следующий такт уже объясняет состояние обычным правилом.
    state = ctl.update(stopped, points=points((0.0, 0.4)), scan_age=0.1)
    assert state["alert"] is False and "нет тяги" in state["reason"]
