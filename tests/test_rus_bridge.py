"""Мост rus_core → мастер Zapret: план движения и последовательность шагов.

Железа нет: мастер подменён ``FakeMaster`` с тем же интерфейсом, что ``MasterSource``
(``steer_module``, ``drive``, ``frames``, ``lock``, ``STALE_S``).
"""

import math
import threading

import pytest

from robot_control import estop
from robot_control.rus_bridge import MasterBridge, plan_motion
from robot_control.rus_core.kinematics import SwerveKinematics

CAL = 0.01  # калибровка для тестов: 0.01 м/с на 1 % тяги


class FakeMaster:
    STALE_S = 1.0

    def __init__(self):
        self.lock = threading.Lock()
        self.frames = {}
        self.steers = []
        self.drives = []
        self.refuse_steer = None
        self.refuse_drive = None

    def set_module(self, name, deg, now=0.0, moving=False, cal=True):
        with self.lock:
            self.frames[name] = ({"mod": name, "deg": deg, "moving": moving,
                                  "cal": cal, "calib": False}, now)

    def settle(self, target, now=0.0):
        """Имитация прошивки: модули доехали до цели и стоят."""
        for name, deg in target.items():
            self.set_module(name, deg, now=now)

    def steer_module(self, module, angle, now=None):
        if self.refuse_steer:
            return {"ok": False, "reason": self.refuse_steer}
        self.steers.append((module, round(angle, 1)))
        return {"ok": True, "queued": True}

    def drive(self, percent, now=None):
        if percent != 0 and self.refuse_drive:
            return {"ok": False, "reason": self.refuse_drive}
        self.drives.append(int(percent))
        return {"ok": True, "percent": int(percent)}


@pytest.fixture
def master():
    return FakeMaster()


def _bridge(master, **kw):
    kw.setdefault("mps_per_percent", CAL)
    return MasterBridge(master, SwerveKinematics(), **kw)


# ---------------------------------------------------------------- план --- #

def test_plan_straight_forward_maps_to_percent():
    plan = plan_motion(SwerveKinematics(), 0.1, 0.0, 0.0, mps_per_percent=CAL)
    assert plan.ok and plan.percent == 10
    assert set(plan.steer_deg) == {"FL", "FR", "RL", "RR"}
    assert all(abs(v) < 1e-9 for v in plan.steer_deg.values())


def test_plan_reverse_gives_negative_percent_with_zero_angles():
    plan = plan_motion(SwerveKinematics(), -0.1, 0.0, 0.0, mps_per_percent=CAL)
    assert plan.ok and plan.percent == -10
    assert all(abs(v) < 1e-9 for v in plan.steer_deg.values())


def test_plan_crab_within_steer_limit_is_feasible():
    # Ход под 20°: все колёса на 20°, тяга одна на все
    v = 0.1
    plan = plan_motion(SwerveKinematics(), v * math.cos(math.radians(20)),
                       v * math.sin(math.radians(20)), 0.0, mps_per_percent=CAL)
    assert plan.ok
    assert all(abs(d - 20.0) < 0.1 for d in plan.steer_deg.values())
    assert plan.percent == 10


def test_plan_crab_45_is_within_steer_limit():
    # Ход руля ±90°: диагональ 45° больше не упирается в предел.
    v = 0.1
    plan = plan_motion(SwerveKinematics(), v * math.cos(math.radians(45)),
                       v * math.sin(math.radians(45)), 0.0, mps_per_percent=CAL)
    assert plan.ok
    assert all(abs(d - 45.0) < 0.1 for d in plan.steer_deg.values())


def test_plan_pure_side_step_at_90_degrees_is_feasible():
    # 90° — чистый боковой ход: все колёса перпендикулярно корпусу, тяга одна.
    v = 0.1
    plan = plan_motion(SwerveKinematics(), v * math.cos(math.radians(90)),
                       v * math.sin(math.radians(90)), 0.0, mps_per_percent=CAL)
    assert plan.ok
    assert all(abs(abs(d) - 90.0) < 0.1 for d in plan.steer_deg.values())


def test_plan_still_refuses_angle_beyond_an_explicit_limit():
    # Предел — параметр моста: на старом железе (±32°) диагональ 45° по-прежнему отказ,
    # то есть защита не исчезла вместе с расширением хода.
    v = 0.1
    plan = plan_motion(SwerveKinematics(), v * math.cos(math.radians(45)),
                       v * math.sin(math.radians(45)), 0.0, mps_per_percent=CAL,
                       max_steer_deg=32.0)
    assert not plan.ok and plan.reason == "steer_limit"


def test_plan_rotation_in_place_is_unsupported():
    plan = plan_motion(SwerveKinematics(), 0.0, 0.0, 0.5, mps_per_percent=CAL)
    assert not plan.ok and plan.reason == "unsupported_motion"


def test_plan_without_calibration_refuses_drive():
    plan = plan_motion(SwerveKinematics(), 0.1, 0.0, 0.0, mps_per_percent=None)
    assert not plan.ok and plan.reason == "speed_not_calibrated"


def test_plan_speed_above_20_percent_is_refused():
    plan = plan_motion(SwerveKinematics(), 0.5, 0.0, 0.0, mps_per_percent=CAL)
    assert not plan.ok and plan.reason == "drive_limit"


def test_plan_stop_when_no_motion():
    plan = plan_motion(SwerveKinematics(), 0.0, 0.0, 0.0, mps_per_percent=CAL)
    assert plan.ok and plan.percent == 0


# ----------------------------------------------------- последовательность --- #

def test_first_step_steers_then_waits_for_alignment_then_drives(master):
    # модули стоят под 10°, цель — 0°: до поворота тяги быть не должно
    for name in ("FL", "FR", "RL", "RR"):
        master.set_module(name, 10.0, now=0.0)
    bridge = _bridge(master)
    r = bridge.step(0.1, 0.0, 0.0, now=0.0)
    assert r["phase"] == "steering" and master.steers
    assert master.drives == []

    # телеметрия свежая, но модули ещё не довернулись — тяги нет
    for name in ("FL", "FR", "RL", "RR"):
        master.set_module(name, 10.0, now=0.2)
    r = bridge.step(0.1, 0.0, 0.0, now=0.2)
    assert r["phase"] == "aligning" and master.drives == []

    master.settle({name: 0.0 for name in bridge.steer_target}, now=0.4)
    r = bridge.step(0.1, 0.0, 0.0, now=0.4)
    assert r["phase"] == "driving" and r["percent"] == 10
    assert master.drives == [10]


def test_heartbeat_resends_drive_every_step(master):
    bridge = _bridge(master)
    bridge.step(0.1, 0.0, 0.0, now=0.0)
    master.settle({n: 0.0 for n in bridge.steer_target}, now=0.1)
    bridge.step(0.1, 0.0, 0.0, now=0.1)
    bridge.step(0.1, 0.0, 0.0, now=0.3)
    bridge.step(0.1, 0.0, 0.0, now=0.5)
    assert master.drives == [10, 10, 10]


def test_steer_change_while_driving_brakes_first(master):
    bridge = _bridge(master)
    bridge.step(0.1, 0.0, 0.0, now=0.0)
    master.settle({n: 0.0 for n in bridge.steer_target}, now=0.1)
    bridge.step(0.1, 0.0, 0.0, now=0.1)  # едем прямо
    steers_before = len(master.steers)

    v = 0.1
    r = bridge.step(v * math.cos(math.radians(20)), v * math.sin(math.radians(20)), 0.0, now=0.3)
    assert r["phase"] == "braking"
    assert master.drives[-1] == 0
    assert len(master.steers) == steers_before  # руль не трогаем на ходу


def test_after_brake_steer_is_sent_and_waits_for_alignment(master):
    bridge = _bridge(master)
    bridge.step(0.1, 0.0, 0.0, now=0.0)
    master.settle({n: 0.0 for n in bridge.steer_target}, now=0.1)
    bridge.step(0.1, 0.0, 0.0, now=0.1)
    v = 0.1
    vx, vy = v * math.cos(math.radians(20)), v * math.sin(math.radians(20))
    bridge.step(vx, vy, 0.0, now=0.3)            # тормоз
    r = bridge.step(vx, vy, 0.0, now=0.5)        # теперь руль
    assert r["phase"] == "steering"
    assert any(abs(deg - 20.0) < 0.2 for _, deg in master.steers[-4:])


def test_alignment_timeout_stops_the_robot(master):
    bridge = _bridge(master, align_timeout_s=1.0)
    bridge.step(0.1, 0.0, 0.0, now=0.0)
    # руль застрял под 15°, телеметрия идёт (кадры свежие на каждом шаге)
    results = {}
    for t in (0.1, 1.0, 2.0):
        master.settle({n: 15.0 for n in bridge.steer_target}, now=t)
        results[t] = bridge.step(0.1, 0.0, 0.0, now=t)
    assert results[1.0]["phase"] == "aligning"
    assert results[2.0]["ok"] is False and results[2.0]["reason"] == "steer_timeout"
    assert master.drives == []  # тяга так и не подавалась
    # после таймаута цель забыта: следующий шаг снова отправляет поворот
    steers_before = len(master.steers)
    r = bridge.step(0.1, 0.0, 0.0, now=2.5)
    assert r["phase"] == "steering" and len(master.steers) > steers_before


def test_moving_module_blocks_drive(master):
    bridge = _bridge(master)
    bridge.step(0.1, 0.0, 0.0, now=0.0)
    master.settle({n: 0.0 for n in bridge.steer_target}, now=0.1)
    master.set_module("FR", 0.0, now=0.2, moving=True)
    r = bridge.step(0.1, 0.0, 0.0, now=0.2)
    assert r["phase"] == "aligning" and master.drives == []


def test_missing_telemetry_refuses_and_stops(master):
    bridge = _bridge(master)
    bridge.step(0.1, 0.0, 0.0, now=0.0)
    master.settle({n: 0.0 for n in bridge.steer_target}, now=0.1)
    bridge.step(0.1, 0.0, 0.0, now=0.1)  # едем
    # телеметрия пропала: кадры старше STALE_S
    r = bridge.step(0.1, 0.0, 0.0, now=10.0)
    assert r["ok"] is False and r["reason"] == "module_offline"
    assert master.drives[-1] == 0


def test_unsupported_motion_brakes_when_driving(master):
    bridge = _bridge(master)
    bridge.step(0.1, 0.0, 0.0, now=0.0)
    master.settle({n: 0.0 for n in bridge.steer_target}, now=0.1)
    bridge.step(0.1, 0.0, 0.0, now=0.1)
    r = bridge.step(0.0, 0.0, 0.5, now=0.3)      # разворот на месте
    assert r["ok"] is False and r["reason"] == "unsupported_motion"
    assert master.drives[-1] == 0


def test_master_refusal_is_reported_and_brakes(master):
    bridge = _bridge(master)
    bridge.step(0.1, 0.0, 0.0, now=0.0)
    master.settle({n: 0.0 for n in bridge.steer_target}, now=0.1)
    master.refuse_drive = "module_busy"
    r = bridge.step(0.1, 0.0, 0.0, now=0.1)
    assert r["ok"] is False and r["reason"] == "module_busy"


def test_stop_command_brakes_and_is_ok(master):
    bridge = _bridge(master)
    bridge.step(0.1, 0.0, 0.0, now=0.0)
    master.settle({n: 0.0 for n in bridge.steer_target}, now=0.1)
    bridge.step(0.1, 0.0, 0.0, now=0.1)
    r = bridge.step(0.0, 0.0, 0.0, now=0.3)
    assert r["ok"] is True and r["phase"] == "stopped"
    assert master.drives[-1] == 0


def test_estop_forces_brake_even_when_bridge_thinks_idle(master, monkeypatch):
    bridge = _bridge(master)
    estop.latch("тест")
    r = bridge.step(0.1, 0.0, 0.0, now=0.0)
    assert r["ok"] is False and r["reason"] == "estop"
    assert master.drives == [0]
    assert master.steers == []


def test_estop_while_driving_brakes(master):
    bridge = _bridge(master)
    bridge.step(0.1, 0.0, 0.0, now=0.0)
    master.settle({n: 0.0 for n in bridge.steer_target}, now=0.1)
    bridge.step(0.1, 0.0, 0.0, now=0.1)
    estop.latch("тест")
    r = bridge.step(0.1, 0.0, 0.0, now=0.3)
    assert r["reason"] == "estop" and master.drives[-1] == 0
    estop.release()


def test_brake_method_stops_motion(master):
    bridge = _bridge(master)
    bridge.step(0.1, 0.0, 0.0, now=0.0)
    master.settle({n: 0.0 for n in bridge.steer_target}, now=0.1)
    bridge.step(0.1, 0.0, 0.0, now=0.1)
    bridge.brake("пульт отпущен", now=0.2)
    assert master.drives[-1] == 0
