"""Блокировка по красному светофору и снижение скорости у знака «неровность»."""
from __future__ import annotations

import pytest

from robot_control.drive_guard import DriveGuard, GuardConfig


class Clock:
    def __init__(self, now: float = 100.0):
        self.now = now

    def __call__(self) -> float:
        return self.now


def guard(**overrides) -> DriveGuard:
    config = GuardConfig(**{"enabled": True, "block_red": True, "slow_bump": True,
                            "red_frames": 2, "green_frames": 2,
                            "slowdown": 0.20, "bump_hold_s": 4.0, "stale_s": 2.0,
                            **overrides})
    return DriveGuard(config, clock=Clock())


def test_single_red_frame_never_blocks():
    g = guard()
    assert g.observe("red", camera_id=0)["blocked"] is False
    # Второй красный кадр подряд — вот теперь блокировка.
    assert g.observe("red", camera_id=0)["blocked"] is True


def test_red_must_be_consecutive_on_the_same_camera():
    g = guard()
    g.observe("red", camera_id=0)
    g.observe("unknown", camera_id=0)          # пропуск кадра обрывает серию
    assert g.observe("red", camera_id=0)["blocked"] is False
    # Кадры другой камеры не подтверждают красный этой.
    g.observe("red", camera_id=2)
    assert g.observe("red", camera_id=3)["blocked"] is False
    assert g.observe("red", camera_id=3)["blocked"] is True


def test_yellow_neither_blocks_nor_releases():
    g = guard()
    g.observe("red", camera_id=0)
    g.observe("red", camera_id=0)
    g.observe("yellow", camera_id=0)
    state = g.observe("yellow", camera_id=0)
    assert state["blocked"] is True
    # Жёлтый сбрасывает серию красного: красный после него считается заново.
    g2 = guard()
    g2.observe("red", camera_id=0)
    g2.observe("yellow", camera_id=0)
    assert g2.observe("red", camera_id=0)["blocked"] is False


def test_only_two_confirmed_green_frames_release_the_block():
    g = guard()
    g.observe("red", camera_id=0)
    g.observe("red", camera_id=0)
    assert g.observe("green", camera_id=0)["blocked"] is True
    # Мигание: один зелёный кадр не открывает движение.
    g.observe("unknown", camera_id=0)
    assert g.observe("green", camera_id=0)["blocked"] is True
    assert g.observe("green", camera_id=0)["blocked"] is False


def test_block_survives_camera_loss_and_never_times_out():
    g = guard()
    g.observe("red", camera_id=0)
    g.observe("red", camera_id=0)
    # Камера ослепла/пропала: наблюдений нет, блокировка не снимается сама.
    for _ in range(50):
        assert g.observe("unknown", camera_id=0)["blocked"] is True
    assert g.snapshot()["stale"] is False
    g._clock = Clock(1000.0)                   # прошло много времени
    assert g.snapshot()["blocked"] is True
    assert g.snapshot()["stale"] is True


def test_red_observed_on_one_camera_is_not_cancelled_by_green_elsewhere():
    g = guard()
    g.observe("red", camera_id=0)
    g.observe("red", camera_id=0)
    g.observe("green", camera_id=1)          # один зелёный кадр — не подтверждение
    assert g.snapshot()["blocked"] is True
    # А два зелёных подряд с одной камеры — снимают: светофор переключился.
    g.observe("green", camera_id=1)
    assert g.snapshot()["blocked"] is False


def test_speed_bump_lowers_the_traction_ceiling_by_twenty_percent():
    g = guard()
    assert g.observe_signs(["stop_sign"])["speedCapPercent"] == 100
    state = g.observe_signs(["speed_bump"])
    assert state["bump"] is True and state["speedCapPercent"] == 80
    assert state["slowdown"] == pytest.approx(0.20)


def test_bump_stays_for_the_hold_window_after_the_sign_leaves():
    clock = Clock()
    g = DriveGuard(GuardConfig(bump_hold_s=4.0), clock=clock)
    g.observe_signs(["speed_bump"])
    assert g.snapshot()["bump"] is True
    clock.now += 3.0
    assert g.snapshot()["bump"] is True
    clock.now += 2.0
    assert g.snapshot()["bump"] is False

    # Знак вернулся в кадр — окно продлено.
    g.observe_signs(["speed_bump"])
    clock.now += 2.0
    g.observe_signs(["speed_bump"])
    clock.now += 3.5
    assert g.snapshot()["bump"] is True


def test_disabled_guard_neither_blocks_nor_slows():
    g = guard(enabled=False)
    for _ in range(5):
        assert g.observe("red", camera_id=0)["blocked"] is False
    assert g.observe_signs(["speed_bump"])["speedCapPercent"] == 100
    assert g.snapshot()["enabled"] is False


def test_independent_switches_for_blocking_and_slowing():
    red_off = guard(block_red=False)
    for _ in range(5):
        assert red_off.observe("red", camera_id=0)["blocked"] is False

    slow_off = guard(slow_bump=False)
    assert slow_off.observe_signs(["speed_bump"])["speedCapPercent"] == 100

    # Отключённое снижение не мешает блокировке и наоборот.
    a = guard(block_red=False)
    a.observe("red", camera_id=0)
    assert a.observe_signs(["speed_bump"])["speedCapPercent"] == 80


def test_release_and_reset_clear_everything():
    g = guard()
    g.observe("red", camera_id=0)
    g.observe("red", camera_id=0)
    g.observe_signs(["speed_bump"])
    g.release("оператор")
    state = g.snapshot()
    assert state["blocked"] is False and state["reason"] == "оператор"
    assert state["bump"] is True                 # снижение живёт своей жизнью
    g.reset()
    state = g.snapshot()
    assert state["blocked"] is False and state["bump"] is False
    assert state["speedCapPercent"] == 100
    assert state["redFrames"] == 0


def test_snapshot_is_json_safe_and_carries_the_reason():
    g = guard()
    g.observe("red", camera_id=1)
    g.observe("red", camera_id=1)
    state = g.snapshot()
    assert state["blocked"] is True
    assert "красный" in state["blockedReason"]
    assert state["blockedCamera"] == 1
    assert isinstance(state["blockedS"], float)
    assert set(state) >= {"enabled", "blocked", "blockedReason", "blockedS", "bump",
                          "speedCapPercent", "slowdown", "reason", "stale"}


def test_signs_are_display_only_by_default():
    """По умолчанию знаки и светофор только показываются, тягу не трогают."""
    guard_config = GuardConfig.from_env()
    assert guard_config.enabled is False, "ограничитель по знакам включён без спроса"
    g = DriveGuard(guard_config)
    for _ in range(5):                      # даже длинная серия красного
        state = g.observe("red", camera_id=0)
        assert state["blocked"] is False
    state = g.observe_signs(["speed_bump", "stop_sign"])
    assert state["speedCapPercent"] == 100 and state["bump"] is False
    # Распознавание при этом никуда не делось: цвет и знаки видны в состоянии.
    assert state["enabled"] is False
    assert "выключены" in state["reason"]


def test_keyboard_without_guard_drives_regardless_of_detected_signs():
    """Клавиатура без подключённого ограничителя всегда даёт полную тягу."""
    from robot_control.keyboard import KeyboardControl

    config = GuardConfig.from_env()
    assert config.enabled is False
    # Ровно то, что делает борт: ограничитель не подключается вовсе.
    ctl = KeyboardControl(lambda: None, lambda *a: ({"ok": True}, 200),
                          lambda *a: ({"ok": True}, 200), lambda: None)
    state = ctl.guard_state()
    assert state["blocked"] is False and state["speedCapPercent"] == 100
    assert ctl._cap_percent(100) == 100
    assert ctl._cap_percent(-100) == -100
    assert ctl._guard_error() is None
