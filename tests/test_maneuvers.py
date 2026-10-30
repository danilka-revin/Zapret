# -*- coding: utf-8 -*-
"""Манёвры тяги без железа: геометрия, рампа, измерение и остановка на цели.

Независимая модель считает реальный путь в каждом шаге, генерирует
квантованные SC-переходы и проверяет метр, обратный ход, дугу и отказы.
Датчик без знака и без измерения пробуксовки не гарантирует движение корпуса.
"""
from __future__ import annotations

import math

import pytest

from robot_control import maneuvers as m

MODULES = m.MODULES


class World:
    """Игрушечное шасси: колесо крутится ровно тем процентом, что ему дали."""

    #: Скорость рулевых модулей: 180 °/с — реальный порядок для TB6600.
    STEER_DEG_PER_S = 180.0

    def __init__(self, *, tach=True, rpm_per_percent=2.0, stuck_steering=False):
        self.tach = tach
        self.rpm_per_percent = rpm_per_percent
        self.stuck_steering = stuck_steering
        self.percent = {module: 0 for module in MODULES}
        self.angle = {module: 0.0 for module in MODULES}
        self.target_angle = {module: 0.0 for module in MODULES}
        self.travel = {module: 0.0 for module in MODULES}
        self.edges = {module: 0.0 for module in MODULES}
        self.quiet = set()
        self.time = 0.0
        self.commands = []
        self.brakes = 0
        self.dt = 0.05
        self.circumference = m.wheel_circumference_m(m.DEFAULT_CHASSIS["wheelDiameterMm"])

    def step(self):
        """Путь считается непрерывно, а не только при отправке команд."""
        self.time += self.dt
        limit = self.STEER_DEG_PER_S * self.dt
        for module in MODULES:
            diff = self.target_angle[module] - self.angle[module]
            if not self.stuck_steering:
                if abs(diff) <= limit:
                    self.angle[module] = self.target_angle[module]
                else:
                    self.angle[module] += limit if diff > 0 else -limit
            turns = abs(self.percent[module]) * self.rpm_per_percent / 60.0 * self.dt
            self.travel[module] += turns * self.circumference
            if module not in self.quiet:
                self.edges[module] += turns * 90

    def frames(self):
        return {module: {
            "deg": self.angle[module],
            "moving": abs(self.target_angle[module] - self.angle[module]) > 0.5,
            "firmware": 17 if self.tach else 16,
            "scRpm": abs(self.percent[module] * self.rpm_per_percent),
            "scPpr": 90,
            "scCount": int(self.edges[module]) % (2 ** 32) if self.tach else None,
            "uptimeMs": round(self.time * 1000) % (2 ** 32),
            "drivePercent": self.percent[module],
            "cal": True,
        } for module in MODULES}

    def steer(self, module, angle):
        self.commands.append(("steer", module, angle))
        self.target_angle[module] = angle

    def drive(self, module, percent):
        self.commands.append(("drive", module, percent))
        targets = MODULES if module is None else (module,)
        for item in targets:
            self.percent[item] = percent

    def brake(self):
        self.brakes += 1
        for module in MODULES:
            self.percent[module] = 0


def runner_for(world, **kwargs):
    config = m.default_config()
    return m.ManeuverRunner(config, brake=world.brake, **kwargs)


def run(world, runner, seconds, start_now=0.0):
    now = start_now
    for _ in range(int(seconds / world.dt)):
        world.step()
        now += world.dt
        runner.tick(now, world.frames(), world.steer, world.drive)
    return now


# --------------------------------------------------------------- геометрия # #
def test_wheel_circumference_matches_the_wheel_diameter():
    # 8,5″ = 215,9 мм; один оборот ≈ 0,678 м.
    assert m.DEFAULT_CHASSIS["wheelDiameterMm"] == 215.9
    assert round(m.wheel_circumference_m(215.9), 3) == 0.678


def test_turn_radius_uses_base_and_track():
    chassis = {"wheelbaseMm": 600.0, "trackMm": 800.0}
    # Колесо на углу 600×800 стоит в 500 мм от центра.
    assert round(m.turn_radius_m(chassis), 3) == 0.500


def test_measured_pivot_axes_determine_the_default_spin_geometry():
    """Продольная база 500 мм, поперечная колея 380 мм — не наоборот."""
    chassis = m.default_config()["chassis"]
    assert chassis["wheelbaseMm"] == 500.0
    assert chassis["trackMm"] == 380.0
    assert m.turn_radius_m(chassis) == pytest.approx(0.31400636936215165)
    assert m.pivot_angle_deg(chassis) == pytest.approx(52.76516601842533)

    angles, signs = m.steering_targets("spin_left", chassis)
    assert angles == pytest.approx({
        "FL": 52.76516601842533, "FR": -52.76516601842533,
        "RL": -52.76516601842533, "RR": 52.76516601842533,
    })
    assert signs == {"FL": -1, "FR": 1, "RL": -1, "RR": 1}

    # Для цели 90° каждый контакт проходит примерно 493,24 мм при
    # идеальном вращении без смещения и проскальзывания.
    runner = m.ManeuverRunner()
    assert runner.start("spin_left", 0.0)["ok"]
    assert runner._target_wheel_path() == pytest.approx(0.49324005158426937)
    runner._travel = {module: 0.49324005158426937 for module in MODULES}
    assert runner._odometry_value() == pytest.approx(90.0)


def test_straight_maneuvers_keep_wheels_straight():
    angles, signs = m.steering_targets("forward")
    assert set(angles.values()) == {0.0}
    assert set(signs.values()) == {1}
    angles, signs = m.steering_targets("backward")
    assert set(angles.values()) == {0.0}
    assert set(signs.values()) == {-1}


def test_side_maneuvers_turn_wheels_perpendicular():
    left, left_signs = m.steering_targets("left")
    right, right_signs = m.steering_targets("right")
    assert set(left.values()) == {-m.SIDE_DEG}
    assert set(right.values()) == {m.SIDE_DEG}
    assert set(left_signs.values()) == {1} and set(right_signs.values()) == {1}


def test_pivot_turns_with_opposite_board_directions_within_the_steering_limit():
    """Разворот: углы зеркальные и в пределах руля, борта едут в разные стороны."""
    angles, signs = m.steering_targets("spin_left")
    pivot = m.pivot_angle_deg(m.DEFAULT_CHASSIS)
    assert set(round(value, 6) for value in angles.values()) == {round(pivot, 6),
                                                                 -round(pivot, 6)}
    # Касательная к окружности поворота всегда круче 45°, но мельче 90° —
    # значит, в ход руля ±100° она укладывается при любой геометрии шасси.
    assert 45 <= pivot < 90
    # База длиннее колеи — колесо доворачивается сильнее.
    assert m.pivot_angle_deg({"wheelbaseMm": 900, "trackMm": 400}) > pivot
    assert all(abs(angle) <= 100 for angle in angles.values())
    assert signs["FL"] == -signs["FR"]
    assert signs["FL"] == signs["RL"] and signs["FR"] == signs["RR"]
    # Вправо — те же углы с обратными знаками тяги.
    right_angles, right_signs = m.steering_targets("spin_right")
    assert right_angles == angles
    assert right_signs == {module: -sign for module, sign in signs.items()}


def test_pivot_wheels_push_the_body_tangentially():
    """Проверка физики: скорость колеса направлена по касательной к окружности."""
    angles, signs = m.steering_targets("spin_left")
    chassis = m.DEFAULT_CHASSIS
    half_base, half_track = chassis["wheelbaseMm"] / 2, chassis["trackMm"] / 2
    positions = {"FL": (half_base, half_track), "FR": (half_base, -half_track),
                 "RL": (-half_base, half_track), "RR": (-half_base, -half_track)}

    def unit(module):
        theta = math.radians(angles[module])
        return (math.cos(theta), -math.sin(theta))     # знак как в прошивке: + = вправо

    for module, (x, y) in positions.items():
        direction = unit(module)
        speed = signs[module]
        velocity = (direction[0] * speed, direction[1] * speed)
        # Скорость колеса должна быть строго касательной, без радиальной
        # составляющей (иначе колёса «разъезжаются», а корпус не крутится),
        # и направленной против часовой стрелки: r × v > 0.
        tangent = (-y, x)
        cross = velocity[0] * tangent[1] - velocity[1] * tangent[0]
        along_radius = velocity[0] * x + velocity[1] * y
        spin = x * velocity[1] - y * velocity[0]
        assert abs(cross) < 1e-9, module
        assert abs(along_radius) < 1e-9, module
        assert spin > 0, module


# ------------------------------------------------------------------ конфиг # #
def test_default_config_is_complete_and_editable():
    config = m.default_config()
    assert set(config["presets"]) == set(m.MANEUVER_IDS)
    assert config["presets"]["forward"]["distanceM"] == 1.0
    assert config["presets"]["spin_left"]["angleDeg"] == 90.0
    assert {key: config["chassis"][key] for key in ("wheelDiameterMm", "wheelbaseMm", "trackMm")}


@pytest.mark.parametrize("name, values", [
    ("forward", {"distanceM": 2.5}),
    ("backward", {"distanceM": 0.3, "percent": 8}),
    ("spin_left", {"angleDeg": 180}),
    ("right", {"toleranceM": 0.08}),
    ("chassis", {"wheelDiameterMm": 254}),
])
def test_preset_updates_are_accepted(name, values):
    config, error = m.apply_preset_update(m.default_config(), name, values)
    assert error is None
    section = config["chassis"] if name == "chassis" else config["presets"][name]
    for key, value in values.items():
        assert section[key] == value


@pytest.mark.parametrize("name, values", [
    ("forward", {"distanceM": 0}),
    ("forward", {"distanceM": "далеко"}),
    ("spin_left", {"angleDeg": 400}),
    ("forward", {"percent": 0}),
    ("chassis", {"wheelDiameterMm": 5}),
    ("нет-такого", {"distanceM": 1}),
    ("forward", {"title": "своё имя"}),
])
def test_preset_updates_reject_nonsense(name, values):
    config, error = m.apply_preset_update(m.default_config(), name, values)
    assert error is not None


def test_config_survives_a_round_trip_on_disk(tmp_path):
    path = tmp_path / "maneuvers.json"
    config = m.default_config()
    config["presets"]["forward"]["distanceM"] = 1.35
    config["chassis"]["wheelDiameterMm"] = 254.0
    config["learned"]["forward"] = {"rate": 0.28, "percent": 12.0}
    m.save_config(config, path)
    loaded = m.load_config(path)
    assert loaded["presets"]["forward"]["distanceM"] == 1.35
    assert loaded["chassis"]["wheelDiameterMm"] == 254.0
    assert loaded["learned"]["forward"] == {"rate": 0.28, "percent": 12.0}


def test_missing_config_uses_measured_axes_without_creating_a_file(tmp_path):
    path = tmp_path / "maneuvers.json"
    config = m.load_config(path)
    assert config["chassis"]["wheelbaseMm"] == 500.0
    assert config["chassis"]["trackMm"] == 380.0
    assert not path.exists()


def test_saved_geometry_is_preserved_until_operator_edits_it(tmp_path):
    path = tmp_path / "maneuvers.json"
    original = m.default_config()
    original["chassis"].update(wheelDiameterMm=207.5, trackMm=450.0)
    original["presets"]["forward"]["distanceM"] = 1.35
    original["learned"]["spin_left"] = {"rate": 20.0, "percent": 10.0}
    m.save_config(original, path)
    contents = path.read_bytes()

    loaded = m.load_config(path)
    runner = m.ManeuverRunner(loaded)
    assert path.read_bytes() == contents  # чтение и создание раннера ничего не мигрируют
    assert runner.config_payload()["chassis"]["trackMm"] == 450.0
    assert m.turn_radius_m(loaded["chassis"]) == pytest.approx(0.3363406011768428)
    assert m.pivot_angle_deg(loaded["chassis"]) == pytest.approx(48.01278750418334)
    assert runner.config_payload()["learned"]["spin_left"]["rate"] == 20.0

    # Только явное сохранение оператором меняет колею; его другие настройки целы.
    result = runner.update_config("chassis", {"trackMm": 380.0})
    assert result["ok"]
    m.save_config(runner.config, path)
    updated = m.load_config(path)
    assert updated["chassis"]["trackMm"] == 380.0
    assert updated["chassis"]["wheelDiameterMm"] == 207.5
    assert updated["presets"]["forward"]["distanceM"] == 1.35
    assert updated["learned"] == {}   # оценки скорости пересчитывают при смене шасси
    assert m.pivot_angle_deg(updated["chassis"]) == pytest.approx(52.76516601842533)


def test_broken_config_file_returns_factory_defaults(tmp_path):
    path = tmp_path / "maneuvers.json"
    path.write_text("{не json", encoding="utf-8")
    assert m.load_config(path) == m.default_config()


# ------------------------------------------------------------------ раннер # #
def test_maneuver_turns_wheels_first_and_drives_only_after_they_arrive():
    world = World()
    runner = runner_for(world)
    assert runner.start("spin_left", 0.0)["ok"] is True
    assert runner.status()["state"] == runner.STEERING
    run(world, runner, 0.15)
    assert any(command[0] == "steer" for command in world.commands)
    # Пока колёса не встали в углы, тяги нет вовсе.
    assert not [c for c in world.commands if c[0] == "drive"]
    assert runner.status()["state"] == runner.STEERING
    run(world, runner, 0.5, start_now=0.15)
    assert runner.status()["state"] == runner.DRIVING
    assert [c for c in world.commands if c[0] == "drive"]


def test_straight_maneuver_does_not_steer_when_wheels_already_straight():
    """Колёса уже смотрят вперёд — лишние команды рулю не нужны."""
    world = World()
    runner = runner_for(world)
    runner.start("forward", 0.0)
    run(world, runner, 0.3)
    assert not [c for c in world.commands if c[0] == "steer"]
    assert runner.status()["state"] == runner.DRIVING


def test_forward_one_metre_stops_within_tolerance():
    world = World()
    runner = runner_for(world)
    runner.start("forward", 0.0)
    run(world, runner, 30.0)
    status = runner.status()
    assert status["state"] == runner.DONE, status
    assert status["reason"] == "reached"
    assert abs(status["value"] - 1.0) <= 0.05, status
    assert all(abs(distance - 1.0) <= 0.06 for distance in world.travel.values())
    assert status["progress"] == 1.0
    # Тормоз — всем колёсам, и проценты на нуле.
    assert world.brakes >= 1
    assert set(world.percent.values()) == {0}


def test_maneuver_is_gentle_at_both_ends():
    """Рампа: тяга не прыгает на рабочий процент и гаснет до нуля, а не рывком."""
    world = World()
    runner = runner_for(world)
    runner.start("forward", 0.0)
    run(world, runner, 30.0)
    sent = [abs(command[2]) for command in world.commands if command[0] == "drive"]
    assert max(sent) >= 10
    # Первая команда тяги — заметно ниже рабочего процента...
    assert sent[0] <= 2
    # ...а последняя перед остановкой — снова маленькая: рампа гасит, а не рубит.
    assert sent[-1] <= 3
    assert runner.status()["state"] == runner.DONE


def test_backward_maneuver_drives_with_minus():
    world = World()
    runner = runner_for(world)
    runner.start("backward", 0.0)
    run(world, runner, 30.0)
    drive = [command for command in world.commands if command[0] == "drive"]
    assert drive and all(command[2] <= 0 for command in drive)
    assert runner.status()["state"] == runner.DONE
    assert all(abs(distance - 0.5) < 0.06 for distance in world.travel.values())


def test_spin_ninety_degrees_is_measured_by_arc_length():
    world = World()
    runner = runner_for(world)
    runner.start("spin_left", 0.0)
    run(world, runner, 60.0)
    status = runner.status()
    assert status["state"] == runner.DONE, status
    assert abs(status["value"] - 90.0) <= 3.0, status
    expected_arc = math.pi / 2 * m.turn_radius_m(m.DEFAULT_CHASSIS)
    assert all(abs(distance - expected_arc) < 0.04 for distance in world.travel.values())
    # Борта ехали в разные стороны.
    per_module = {}
    for command in world.commands:
        if command[0] == "drive" and command[1]:
            per_module[command[1]] = command[2]
    assert per_module["FL"] * per_module["FR"] <= 0


def test_successful_maneuver_learns_rate_for_ramp_and_timeout_only():
    """Измеренная скорость не разрешает движение без SC."""
    world = World()
    runner = runner_for(world)
    runner.start("forward", 0.0)
    run(world, runner, 30.0)
    learned = runner.status()["learned"]
    assert learned and learned["rate"] > 0
    assert learned["percent"] == 12.0


def test_without_counter_the_maneuver_never_drives_by_time():
    world = World(tach=False)
    runner = runner_for(world)
    runner.start("forward", 0.0)
    run(world, runner, 0.3)
    status = runner.status()
    assert status["state"] == runner.FAILED
    assert status["reason"] == "sc_firmware_required"
    assert not [c for c in world.commands if c[0] == "drive"]
    assert world.brakes >= 1


def test_manual_stop_cancels_and_brakes():
    world = World()
    runner = runner_for(world)
    runner.start("forward", 0.0)
    run(world, runner, 1.0)
    status = runner.cancel(1.0)
    assert status["state"] == runner.CANCELLED
    assert status["reason"] == "cancelled"
    assert world.brakes >= 1


def test_second_maneuver_is_refused_while_the_first_runs():
    world = World()
    runner = runner_for(world)
    assert runner.start("forward", 0.0)["ok"] is True
    second = runner.start("spin_left", 0.1)
    assert second == {"ok": False, "reason": "maneuver_busy"}
    assert runner.cancel(0.2)["state"] == runner.CANCELLED
    assert runner.start("spin_left", 0.3)["ok"] is True


def test_wheel_without_steering_feedback_fails_instead_of_driving():
    world = World(stuck_steering=True)
    runner = runner_for(world)
    runner.start("spin_left", 0.0)
    run(world, runner, 15.0)
    status = runner.status()
    assert status["state"] == runner.FAILED
    assert status["reason"] == "steering_timeout"
    assert not [c for c in world.commands if c[0] == "drive"]
    assert world.brakes >= 1


def test_unknown_maneuver_is_rejected():
    world = World()
    runner = runner_for(world)
    assert runner.start("телепорт", 0.0) == {"ok": False, "reason": "invalid_maneuver"}


def test_work_percent_never_exceeds_the_board_limit():
    world = World()
    runner = runner_for(world)
    runner.start("forward", 0.0, limit_percent=5)
    assert runner.status()["workPercent"] == 5


def test_config_cannot_change_mid_maneuver():
    world = World()
    runner = runner_for(world)
    runner.start("forward", 0.0)
    result = runner.update_config("forward", {"distanceM": 5.0})
    assert result == {"ok": False, "reason": "module_busy"}
    runner.cancel(0.1)
    assert runner.update_config("forward", {"distanceM": 5.0})["ok"] is True
    assert runner.status()["target"] == 5.0 or runner.config["presets"]["forward"]["distanceM"] == 5.0


def test_changing_percent_forgets_the_learned_speed():
    world = World()
    runner = runner_for(world)
    runner.start("forward", 0.0)
    run(world, runner, 30.0)
    assert "forward" in runner.config["learned"]
    runner.update_config("forward", {"percent": 15})
    assert "forward" not in runner.config["learned"]


def test_status_payload_has_no_active_maneuver_when_idle():
    world = World()
    runner = runner_for(world)
    status = runner.status()
    assert status["active"] is False and status["state"] == runner.IDLE
    payload = runner.config_payload()
    assert set(payload["presets"]) == set(m.MANEUVER_IDS)
    assert payload["limits"]["distanceM"][0] > 0


def test_realistic_250ms_frames_measure_pulses_not_rpm_or_command_count():
    world = World()
    runner = runner_for(world)
    assert runner.start("forward", 0.0)["ok"]
    frames = world.frames()
    for tick in range(600):
        world.step()
        if tick % 5 == 0:  # @TLM раз в 250 мс, не каждый тик сервера
            frames = world.frames()
            # Ошибочная скорость не должна добавлять путь: только счётчик.
            for frame in frames.values():
                frame["scRpm"] = 0.0
        runner.tick(world.time, frames, world.steer, world.drive)
        if not runner.status()["active"]:
            break
    assert runner.status()["state"] == runner.DONE, runner.status()
    assert all(abs(value - 1.0) < 0.10 for value in world.travel.values())


@pytest.mark.parametrize("quiet", ["FL", "FR", "RL", "RR"])
def test_each_silent_sc_sensor_stops_every_wheel(quiet):
    world = World()
    world.quiet.add(quiet)
    runner = runner_for(world)
    runner.start("forward", 0.0)
    run(world, runner, 8.0)
    assert runner.status()["state"] == runner.FAILED
    assert runner.status()["reason"] in ("sc_stalled", "sc_mismatch")
    assert set(world.percent.values()) == {0} and world.brakes
    assert runner.status()["mode"] == "odometry"


def test_short_target_cannot_succeed_on_three_wheels_only():
    world = World(rpm_per_percent=20)
    world.quiet.add("FR")
    runner = runner_for(world)
    assert runner.update_config("forward", {"distanceM": 0.05})["ok"]
    runner.start("forward", 0.0)
    run(world, runner, 5.0)
    assert runner.status()["state"] == runner.FAILED
    assert runner.status()["reason"] == "sc_mismatch"
    assert set(world.percent.values()) == {0}


def test_missing_telemetry_aborts_immediately_without_time_fallback():
    world = World()
    runner = runner_for(world)
    runner.start("forward", 0.0)
    now = run(world, runner, 0.8)
    frames = world.frames()
    del frames["RL"]
    runner.tick(now + 0.05, frames, world.steer, world.drive)
    assert runner.status()["reason"] == "module_offline"
    assert set(world.percent.values()) == {0}


@pytest.mark.parametrize("damage, reason", [
    (lambda frame: frame.update(scCount=None), "sc_invalid"),
    (lambda frame: frame.update(scPpr=91), "sc_changed"),
    (lambda frame: frame.update(scCount=4000000), "sc_changed"),
    (lambda frame: frame.update(uptimeMs=0), "sc_changed"),
    (lambda frame: frame.update(firmware=16), "sc_firmware_required"),
    (lambda frame: frame.update(deg=20), "steering_changed"),
    (lambda frame: frame.update(drivePercent=-5), "drive_mismatch"),
])
def test_corrupt_or_changed_feedback_stops_immediately(damage, reason):
    world = World()
    runner = runner_for(world)
    runner.start("forward", 0.0)
    now = run(world, runner, 1.0)
    frames = world.frames()
    damage(frames["FR"])
    runner.tick(now + 0.05, frames, world.steer, world.drive)
    assert runner.status()["reason"] == reason
    assert set(world.percent.values()) == {0}


def test_counter_rollover_counts_small_positive_delta():
    world = World()
    for module in MODULES:
        world.edges[module] = 2 ** 32 - 5
    runner = runner_for(world)
    runner.start("forward", 0.0)
    run(world, runner, 30.0)
    assert runner.status()["state"] == runner.DONE, runner.status()
    assert all(abs(value - 1.0) < 0.06 for value in world.travel.values())


def test_drive_command_rejection_brakes_everything():
    world = World()
    runner = runner_for(world)
    runner.start("forward", 0.0)
    world.step()
    runner.tick(world.time, world.frames(), world.steer,
                lambda module, value: {"ok": False, "reason": "command_queue_full"})
    assert runner.status()["reason"] == "command_failed"
    assert world.brakes


def test_maneuver_waits_for_exact_brake_ack_from_every_board():
    world = World()
    runner = runner_for(world)
    assert runner.start("forward", 0.0, brake_id=42)["ok"]
    frames = world.frames()
    for frame in frames.values():
        frame["brakeId"] = 41  # an earlier STOP, not this one
    for module in MODULES[:-1]:
        frames[module]["brakeId"] = 42
    runner.tick(0.05, frames, world.steer, world.drive)
    assert runner.status()["state"] == runner.STEERING and not world.commands
    frames[MODULES[-1]]["brakeId"] = 42
    runner.tick(0.10, frames, world.steer, world.drive)
    assert runner.status()["state"] == runner.DRIVING
    assert any(command[0] == "drive" for command in world.commands)


def test_maneuver_missing_brake_ack_fails_closed():
    world = World()
    runner = runner_for(world)
    runner.start("right", 0.0, brake_id=42)
    frames = world.frames()
    for frame in frames.values():
        frame["brakeId"] = 42
    frames["FR"]["brakeId"] = None
    runner.tick(0.05, frames, world.steer, world.drive)
    assert runner.status()["reason"] == "brake_firmware_required"
    assert not world.commands and world.brakes == 1
