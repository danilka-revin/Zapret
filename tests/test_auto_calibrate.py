"""Автокалибровка по ярлыку: порядок колёса → потенциометры, все четыре разом.

Проверяем именно последовательность и гейты, а не «перемолола ли прошивка»:
прошивку имитирует фейковый источник, время — виртуальные часы.
"""

from __future__ import annotations

import threading

from robot_control import auto_calibrate

MODULES = ("FL", "FR", "RL", "RR")


class FakeClock:
    def __init__(self, source=None):
        self.t = 0.0
        self.source = source

    def now(self):
        return self.t

    def monotonic(self):
        return self.t

    def sleep(self, dt):
        self.t += dt
        if self.source is not None:
            self.source.tick()


class FakeSource:
    """Четыре платы с кадрами @TLM и очередью команд, как у мастера."""

    name = "master"
    supports_calibration = True
    STALE_S = 1.0

    def __init__(self, clock, modules=MODULES, *, wheel_converge=True, pot_ok=True,
                 never_finish_wheel=False, wheel_silent=False, pot_silent=False):
        self.clock = clock
        self.lock = threading.Lock()
        self.frames = {mid: (self._blank_frame(), clock.t) for mid in modules}
        self.commands = []
        self.wheel_converge = wheel_converge
        self.pot_ok = pot_ok
        self.never_finish_wheel = never_finish_wheel
        self.wheel_silent = wheel_silent
        self.pot_silent = pot_silent
        self.calibrate_error = None
        self.pot_error = None
        self.pm_error = None
        self.wheel_duration = 5.0
        self.pot_duration = 3.0
        self._wheel_end = None
        self._pot_end = None
        clock.source = self

    @staticmethod
    def _blank_frame():
        # potMode=True: pm по умолчанию включён в прошивке (bool pot_enabled = true).
        return {"cal": False, "calib": False, "moving": False, "cycle": False,
                "drivePercent": 0, "potMode": True, "pot": False}

    # --- команды, как их вызывает автокалибровка --------------------------- #
    def calibrate_module(self, module, now=None):
        self.commands.append(("calibrate_module", str(module).upper()))
        if self.calibrate_error:
            return {"ok": False, "reason": self.calibrate_error}
        if self.wheel_silent:
            return {"ok": True, "queued": True}
        with self.lock:
            for mid, entry in self.frames.items():
                frame = dict(entry[0])
                frame.update(calib=True, moving=True)
                self.frames[mid] = (frame, self.clock.t)
        self._wheel_end = self.clock.t + self.wheel_duration
        return {"ok": True, "queued": True}

    def set_pot_mode(self, action, now=None):
        self.commands.append(("set_pot_mode", action))
        if action == "on":
            if self.pm_error:
                return {"ok": False, "reason": self.pm_error}
            with self.lock:
                for mid, entry in self.frames.items():
                    frame = dict(entry[0])
                    frame["potMode"] = True
                    self.frames[mid] = (frame, self.clock.t)
            return {"ok": True, "queued": True}
        if action == "calibrate":
            if self.pot_error:
                return {"ok": False, "reason": self.pot_error}
            if self.pot_silent:
                return {"ok": True, "queued": True}
            with self.lock:
                for mid, entry in self.frames.items():
                    frame = dict(entry[0])
                    frame.update(calib=True, moving=True)
                    self.frames[mid] = (frame, self.clock.t)
            self._pot_end = self.clock.t + self.pot_duration
            return {"ok": True, "queued": True}
        return {"ok": False, "reason": "invalid_action"}

    # --- виртуальное время двигается — платы отвечают ---------------------- #
    def tick(self):
        with self.lock:
            for mid, entry in self.frames.items():
                frame = dict(entry[0])
                if (self._wheel_end is not None and not self.never_finish_wheel
                        and self.clock.t >= self._wheel_end):
                    frame.update(calib=False, moving=False, cal=self.wheel_converge)
                if self._pot_end is not None and self.clock.t >= self._pot_end:
                    frame.update(calib=False, moving=False, pot=self.pot_ok)
                self.frames[mid] = (frame, self.clock.t)
            if self._wheel_end is not None and not self.never_finish_wheel \
                    and self.clock.t >= self._wheel_end:
                self._wheel_end = None
            if self._pot_end is not None and self.clock.t >= self._pot_end:
                self._pot_end = None


def make_rig(modules=MODULES, **kwargs):
    clock = FakeClock()
    source = FakeSource(clock, modules, **kwargs)
    entries = []

    def record(level, message, *, source=""):
        entries.append((level, message))

    estop = {"on": False}

    def latched():
        return {"since": 1} if estop["on"] else None

    return clock, source, entries, estop, latched, record


def run(rig):
    clock, source, entries, estop, latched, record = rig
    outcome = auto_calibrate.run(
        source, latched,
        sleep=clock.sleep, monotonic=clock.monotonic, now=clock.now, record=record,
    )
    return outcome, entries, source


def test_happy_path_wheels_first_then_pots_for_all_four():
    rig = make_rig()
    outcome, entries, source = run(rig)
    assert outcome["ok"] is True and outcome["stage"] == "done"
    assert source.commands == [
        ("calibrate_module", "ALL"),
        ("set_pot_mode", "on"),
        ("set_pot_mode", "calibrate"),
    ]
    texts = [msg for _, msg in entries]
    assert any("все четыре модуля подключены" in msg for msg in texts)
    assert any("калибровка руля завершена" in msg for msg in texts)
    assert any("калибровка потенциометров завершена" in msg for msg in texts)


def test_waits_for_the_fourth_module_before_sending_anything():
    modules = MODULES[:-1]
    clock = FakeClock()
    source = FakeSource(clock, modules)
    entries = []
    record = lambda level, message, *, source="": entries.append((level, message))
    latched = lambda: None

    sleeps = {"n": 0}

    def sleep(dt):
        sleeps["n"] += 1
        clock.sleep(dt)
        # на третьей паузе «подключаем» RR
        if sleeps["n"] == 3 and "RR" not in source.frames:
            source.frames["RR"] = (source._blank_frame(), clock.t)

    outcome = auto_calibrate.run(source, latched, sleep=sleep, monotonic=clock.monotonic,
                                 now=clock.now, record=record)
    assert outcome["ok"] is True
    assert source.commands[0] == ("calibrate_module", "ALL")


def test_estop_latched_before_start_cancels_everything():
    rig = make_rig()
    rig_estop = rig[3]
    rig_estop["on"] = True
    outcome, entries, source = run(rig)
    assert outcome["ok"] is False and outcome["reason"] == "estop"
    assert source.commands == []


def test_estop_during_wheel_calibration_skips_potentiometers():
    rig = make_rig()
    clock, source, entries, estop, latched, record = rig

    original = source.calibrate_module

    def tripping_calibrate(module, now=None):
        result = original(module, now)
        estop["on"] = True
        return result

    source.calibrate_module = tripping_calibrate
    outcome, entries, source = run(rig)
    assert outcome["ok"] is False and outcome["reason"] == "estop"
    assert ("set_pot_mode", "calibrate") not in source.commands


def test_wheel_command_refused_retries_then_gives_up_without_pots():
    rig = make_rig()
    rig[1].calibrate_error = "module_busy"
    outcome, entries, source = run(rig)
    assert outcome["ok"] is False and outcome["stage"] == "wheel"
    assert source.commands.count(("calibrate_module", "ALL")) == auto_calibrate.QUEUE_ATTEMPTS
    assert ("set_pot_mode", "calibrate") not in source.commands


def test_wheel_timeout_skips_potentiometers():
    rig = make_rig(never_finish_wheel=True)
    outcome, entries, source = run(rig)
    assert outcome["ok"] is False and outcome["stage"] == "wheel" and outcome["reason"] == "timeout"
    assert ("set_pot_mode", "calibrate") not in source.commands


def test_wheel_finish_without_home_skips_potentiometers():
    rig = make_rig(wheel_converge=False)
    outcome, entries, source = run(rig)
    assert outcome["ok"] is False and outcome["stage"] == "wheel" and outcome["reason"] == "not_homed"
    assert outcome["modules"] == list(MODULES)
    assert ("set_pot_mode", "calibrate") not in source.commands


def test_wheel_never_starts_is_reported_and_pots_skipped():
    rig = make_rig(wheel_silent=True)
    outcome, entries, source = run(rig)
    assert outcome["ok"] is False and outcome["stage"] == "wheel" and outcome["reason"] == "not_started"
    assert ("set_pot_mode", "calibrate") not in source.commands


def test_pot_never_starts_is_reported():
    rig = make_rig(pot_silent=True)
    outcome, entries, source = run(rig)
    assert outcome["ok"] is False and outcome["stage"] == "pot" and outcome["reason"] == "not_started"
    assert source.commands[0] == ("calibrate_module", "ALL")


def test_pot_not_confirmed_after_q_is_reported():
    rig = make_rig(pot_ok=False)
    outcome, entries, source = run(rig)
    assert outcome["ok"] is False and outcome["stage"] == "pot" and outcome["reason"] == "not_calibrated"
    assert outcome["modules"] == list(MODULES)


def test_pot_calibrate_refused_is_reported():
    rig = make_rig()
    rig[1].pot_error = "zero_required"
    outcome, entries, source = run(rig)
    assert outcome["ok"] is False and outcome["stage"] == "pot" and outcome["reason"] == "zero_required"
    assert source.commands[0] == ("calibrate_module", "ALL")


def test_start_refused_when_disabled_or_unsupported():
    assert auto_calibrate.autostart_enabled({"RC_AUTO_CALIBRATE": "0"}) is False
    assert auto_calibrate.autostart_enabled({"RC_AUTO_CALIBRATE": "off"}) is False
    assert auto_calibrate.autostart_enabled({}) is True
    assert auto_calibrate.autostart_enabled({"RC_AUTO_CALIBRATE": "1"}) is True

    class Bare:
        pass

    class State:
        pass

    state = State()
    state.source = Bare()  # источник без supports_calibration
    assert auto_calibrate.start(state, env={}) is False

    class NoFrames:
        name = "master"
        supports_calibration = True
        frames = None

        def calibrate_module(self, module):
            return {"ok": True}

        def set_pot_mode(self, action):
            return {"ok": True}

    state = State()
    state.source = NoFrames()
    assert auto_calibrate.start(state, env={}) is False


def test_start_launches_daemon_thread_for_master_source():
    clock = FakeClock()
    source = FakeSource(clock, MODULES)

    class State:
        pass

    state = State()
    state.source = source
    # env пуст → по умолчанию включено; E-STOP берём из файла, но он не взведён
    assert auto_calibrate.start(state, env={}, latched=lambda: None) is True
