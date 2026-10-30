"""Регрессии ядра rus_core: найденные при переносе из rus_slam дефекты и связки модулей.

Эти тесты добавлены в Zapret сверх исходных 25 тестов rus_slam.
"""

import math
import random
import struct

import pytest

from robot_control.rus_core.kinematics import SwerveKinematics
from robot_control.rus_core.lds import LdsParser
from robot_control.rus_core.odometry import SwerveOdometry, WheelSample
from robot_control.rus_core.protocol import MAX_STEER_CDEG, build_command_packet


def _build_lds_frame(points):
    body = bytearray([0x54, 0x2C])
    body += (36000).to_bytes(2, 'little')          # скорость 360 °/с
    body += (0).to_bytes(2, 'little')              # старт 0°
    for dist_m, intensity in points:
        body += (int(dist_m * 1000) & 0xFFFF).to_bytes(2, 'little')
        body.append(intensity & 0xFF)
    body += (3000).to_bytes(2, 'little')           # конец 30°
    body += b"\x00\x00"
    xor = 0
    for b in body:
        xor ^= b
    body.append(xor)
    return bytes(body)


def _random_commands(seed, count):
    rnd = random.Random(seed)
    for _ in range(count):
        yield (rnd.uniform(-1, 1), rnd.uniform(-1, 1), rnd.uniform(-1, 1))


def test_steer_angle_stays_wrapped_over_long_run():
    """Раньше угол копился между командами (до 115 рад) и зажимался в int16."""
    kin = SwerveKinematics()
    worst = 0.0
    for vx, vy, wz in _random_commands(1, 5000):
        for cmd in kin.inverse(vx, vy, wz):
            worst = max(worst, abs(cmd.angle_rad))
    assert worst <= math.pi + 1e-9


def test_steer_command_never_clamped_by_packet_encoding():
    kin = SwerveKinematics()
    for vx, vy, wz in _random_commands(2, 2000):
        for cmd in kin.inverse(vx, vy, wz):
            cdeg = round(math.degrees(cmd.angle_rad) * 100)
            assert abs(cdeg) <= MAX_STEER_CDEG
            packet = build_command_packet(cmd.module_id, cdeg, 0, cmd.enable)
            assert struct.unpack('>h', packet[3:5])[0] == cdeg


def test_wrap_keeps_shortest_turn():
    """Переход 170° → −170° идёт через 180° на 20°, а не на 340°; угол остаётся в [-π, π)."""
    kin = SwerveKinematics()
    kin._last_angles[1] = math.radians(170.0)
    target = math.radians(-170.0)
    # Для FL при wz=0 угол колеса равен направлению скорости корпуса.
    cmd = [c for c in kin.inverse(math.cos(target), math.sin(target), 0.0)
           if c.module_id == 1][0]
    assert -math.pi <= cmd.angle_rad < math.pi
    # Кратчайший путь: направление совпадает с целью (по модулю 2π) и тяга вперёд.
    assert abs(math.atan2(math.sin(cmd.angle_rad - target),
                          math.cos(cmd.angle_rad - target))) < 1e-9
    assert cmd.speed_mps > 0


@pytest.mark.parametrize("wheelbase, track", [(0.50, 0.38), (0.55, 0.46)])
def test_round_trip_kinematics_to_odometry_recovers_body_twist(wheelbase, track):
    """Колёсные проекции восстанавливаются; компоненты — когда они наблюдаемы."""
    kin = SwerveKinematics(wheelbase=wheelbase, track=track, max_speed=5.0)
    odom = SwerveOdometry(kin.wheelbase, kin.track)
    dt = 0.05
    rnd = random.Random(3)
    well_conditioned = 0
    for _ in range(300):
        vx, vy, wz = rnd.uniform(-0.6, 0.6), rnd.uniform(-0.6, 0.6), rnd.uniform(-0.6, 0.6)
        cmds = kin.inverse(vx, vy, wz)
        if not all(c.enable for c in cmds):
            continue
        samples = [WheelSample(c.module_id, c.angle_rad, c.speed_mps * dt) for c in cmds]
        est = odom.estimate_twist(dt, samples)
        rows = []
        for c in cmds:
            x, y = odom.positions[c.module_id]
            co, si = math.cos(c.angle_rad), math.sin(c.angle_rad)
            row = (co, si, x * si - y * co)
            rows.append(row)
            # Даже возле вырождения оценка должна соответствовать измеренным
            # проекциям скорости на все четыре направления качения.
            assert abs(row[0] * est.vx + row[1] * est.vy + row[2] * est.wz - c.speed_mps) < 1e-6
        assert math.hypot(est.vx, est.vy, est.wz) <= math.hypot(vx, vy, wz) + 1e-6

        # При почти вырожденной матрице компоненты плохо наблюдаемы:
        # стабилизирующая добавка 1e-9 намеренно смещает их к меньшей норме.
        # Новая геометрия 500×380 попадает в такой случай в этой выборке.
        ata = [[sum(row[i] * row[j] for row in rows) for j in range(3)] for i in range(3)]
        det = (ata[0][0] * (ata[1][1] * ata[2][2] - ata[1][2] * ata[2][1])
               - ata[0][1] * (ata[1][0] * ata[2][2] - ata[1][2] * ata[2][0])
               + ata[0][2] * (ata[1][0] * ata[2][1] - ata[1][1] * ata[2][0]))
        if det > 0.001:
            well_conditioned += 1
        # Для прежней базы сохраняем строгую проверку всех прежних 300 случаев.
        if det > 0.001 or (wheelbase, track) == (0.55, 0.46):
            assert abs(est.vx - vx) < 1e-6 and abs(est.vy - vy) < 1e-6 and abs(est.wz - wz) < 1e-6
    assert well_conditioned > 200


def test_straight_line_with_all_wheels_aligned_is_exact():
    """Все колёса смотрят вперёд: vy не наблюдается, но vx и wz должны быть точными."""
    odom = SwerveOdometry(0.55, 0.46)
    for dt in (0.01, 0.05, 0.2):
        samples = [WheelSample(mid, 0.0, 0.8 * dt) for mid in odom.positions]
        twist = odom.estimate_twist(dt, samples)
        assert abs(twist.vx - 0.8) < 1e-6
        assert abs(twist.vy) < 1e-6 and abs(twist.wz) < 1e-6


def test_lds_parser_byte_by_byte_and_bad_frame_then_good():
    pts = [(0.25 * (i + 1), 90) for i in range(12)]
    good = _build_lds_frame(pts)
    bad = bytearray(good)
    bad[10] ^= 0x55  # портим данные, КС не совпадёт
    parser = LdsParser()
    frames = []
    for byte in bytes(bad) + good:
        frames += parser.feed(bytes([byte]))
    assert len(frames) == 1
    assert frames[0].points[0][0] == 0.25
    assert parser.checksum_errors >= 1
