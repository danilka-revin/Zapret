"""Ограничение движения по знакам и светофору с камер.

Модуль отвечает на два требования оператора:

* **Красный светофор, увиденный два кадра подряд**, блокирует управление с
  клавиатуры до тех пор, пока в кадре не появится зелёный. Один кадр красного
  ничего не блокирует: одиночные вспышки, отражения и чужие красные предметы
  попадают в кадр регулярно, и блокировать по ним управление нельзя.
* **Знак «искусственная неровность» (кочка)** снижает скорость управления с
  клавиатуры на 20 % — пока знак виден и ещё короткое время после того, как он
  ушёл из кадра, чтобы робот успел проехать саму неровность.

Здесь только чистая логика: ни камер, ни моторов, ни потоков. Сервис
распознавания передаёт сюда по одному наблюдению за кадр, а ``KeyboardControl``
спрашивает готовое решение перед каждой командой тяги. Поэтому блокировку
нельзя обойти устаревшей вкладкой или ручным запросом: проверка живёт на борту.

Границы ответственности:

* **По умолчанию выключен** (``RC_GUARD_ENABLED=0``): знаки и светофор
  распознаются и показываются, но тягу не ограничивают. Движение — только из
  ручного управления. Включайте осознанно, если правила полигона требуют
  реакции на знаки.
* Это не сертифицированная защита и не замена E-STOP. Блокировка снимается
  только зелёным сигналом (или явным ``release`` оператором/сервисом), а не
  таймаутом: «красный пропал из кадра» не значит «можно ехать».
* Снижение скорости — потолок процентов тяги, а не отдельная команда приводу.
  Оно действует только на ручное управление с клавиатуры; автономные режимы
  пользуются своими пределами (``RC_NAV_MAX_PERCENT`` и т. п.).
* Знак в кадре — факт наблюдения. Если камера ослепла, пропала или детектор
  выключен, наблюдений нет: ни блокировки, ни снижения не выдумывается.
"""
from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

#: Сколько подтверждающих кадров нужно, чтобы красный заблокировал управление.
DEFAULT_RED_FRAMES = 2
#: Сколько подтверждающих кадров нужно, чтобы зелёный снял блокировку.
DEFAULT_GREEN_FRAMES = 2
#: Сколько секунд после последнего кадра знака «неровность» снижение остаётся:
#: робот успевает доехать до кочки, которая уже вышла из кадра.
DEFAULT_BUMP_HOLD_S = 4.0
#: Насколько снижается потолок тяги при знаке «неровность».
DEFAULT_SLOWDOWN = 0.20
#: Сколько секунд живёт наблюдение: сервис может перестать присылать кадры.
DEFAULT_STALE_S = 2.0

BUMP_CLASSES = ("speed_bump",)


def _env_float(name: str, default: float, lo: float, hi: float) -> float:
    try:
        value = float((os.environ.get(name) or "").strip() or default)
    except (TypeError, ValueError):
        return default
    return max(lo, min(hi, value)) if math.isfinite(value) else default


def _env_int(name: str, default: int, lo: int, hi: int) -> int:
    return int(_env_float(name, default, lo, hi))


def _env_flag(name: str, default: bool) -> bool:
    raw = (os.environ.get(name) or "").strip().lower()
    if not raw:
        return default
    return raw in {"1", "true", "yes", "on"}


@dataclass(frozen=True)
class GuardConfig:
    """Пороги подтверждения и снижения. Проверяются при сборке, не на ходу."""

    enabled: bool = True
    #: Блокировка по красному (сама по себе; красный без неё только индикация).
    block_red: bool = True
    #: Снижение потолка тяги при знаке «искусственная неровность».
    slow_bump: bool = True
    red_frames: int = DEFAULT_RED_FRAMES
    green_frames: int = DEFAULT_GREEN_FRAMES
    slowdown: float = DEFAULT_SLOWDOWN
    bump_hold_s: float = DEFAULT_BUMP_HOLD_S
    stale_s: float = DEFAULT_STALE_S

    @classmethod
    def from_env(cls) -> "GuardConfig":
        return cls(
            # По умолчанию знаки только распознаются и показываются: тягу
            # ограничивает человек с клавиатуры, а не картинка с камеры.
            # Включение — осознанное решение стенда/полигона.
            enabled=_env_flag("RC_GUARD_ENABLED", False),
            block_red=_env_flag("RC_GUARD_RED_BLOCK", True),
            slow_bump=_env_flag("RC_GUARD_BUMP_SLOW", True),
            red_frames=_env_int("RC_GUARD_RED_FRAMES", DEFAULT_RED_FRAMES, 1, 20),
            green_frames=_env_int("RC_GUARD_GREEN_FRAMES", DEFAULT_GREEN_FRAMES, 1, 20),
            slowdown=_env_float("RC_GUARD_SLOWDOWN", DEFAULT_SLOWDOWN, 0.05, 0.80),
            bump_hold_s=_env_float("RC_GUARD_BUMP_HOLD_S", DEFAULT_BUMP_HOLD_S, 0.0, 30.0),
            stale_s=_env_float("RC_GUARD_STALE_S", DEFAULT_STALE_S, 0.2, 10.0),
        )


class DriveGuard:
    """Подтверждение красного/зелёного и снижение скорости у «кочки».

    Наблюдения приходят покадрово, по одному на камеру: ``observe()`` для
    цвета светофора и ``observe_signs()`` для классов знаков. Порядок внутри
    кадра не важен — важен порядок кадров.
    """

    def __init__(self, config: Optional[GuardConfig] = None, *,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.config = config or GuardConfig.from_env()
        self._clock = clock
        #: Серии подтверждений считаются по каждой камере отдельно: кадры с
        #: другой камеры не обрывают серию, иначе при четырёх камерах красный
        #: никогда не набрал бы двух подтверждений (соседние кадры «пустые»).
        self._red_run: dict[int, int] = {}
        self._green_run: dict[int, int] = {}
        self._blocked = False
        self._blocked_by: Optional[int] = None
        self._bump_run = 0
        self._bump_until: Optional[float] = None
        self._bump_last_seen: Optional[float] = None
        self._last_observation: Optional[float] = None
        self._reason = "наблюдений ещё не было"

    # ------------------------------------------------------------ наблюдения
    def observe(self, state: Any, *, camera_id: int = 0,
                now: Optional[float] = None) -> dict[str, Any]:
        """Один кадр светофора. ``state`` — ``red``/``yellow``/``green``/``unknown``.

        Красный подтверждается только подряд идущими кадрами: последовательность
        red → unknown → red блокировкой не становится. Зелёный снимает
        блокировку так же — по двум кадрам подряд, чтобы мигание диодов не
        открывало движение на один кадр.
        """
        now = self._clock() if now is None else float(now)
        if not self.config.enabled:
            self.reset()
            return self.snapshot(now=now)
        state = str(state or "unknown").strip().lower()
        camera = int(camera_id) if isinstance(camera_id, int) else 0
        self._last_observation = now
        if state == "red":
            self._red_run[camera] = self._red_run.get(camera, 0) + 1
            self._green_run[camera] = 0
            if (self.config.block_red and not self._blocked and
                    self._red_run[camera] >= self.config.red_frames):
                self._blocked = True
                self._blocked_by = camera
                self._blocked_at = now
                self._reason = ("красный подтверждён %d кадрами подряд (CAM %d)"
                                % (self._red_run[camera], camera))
        elif state == "green":
            self._green_run[camera] = self._green_run.get(camera, 0) + 1
            self._red_run[camera] = 0
            if (self._blocked and
                    self._green_run[camera] >= self.config.green_frames):
                self._blocked = False
                self._blocked_by = None
                self._blocked_at = None
                self._reason = ("зелёный подтверждён %d кадрами подряд (CAM %d)"
                                % (self._green_run[camera], camera))
        elif state == "yellow":
            # Жёлтый не блокирует сам и не снимает красный: «приготовиться».
            self._red_run[camera] = 0
            self._green_run[camera] = 0
        else:
            # Кадр без распознанного сигнала. Если красного не было подряд,
            # серия обрывается; уже подтверждённая блокировка остаётся: «красный
            # пропал из кадра» не значит «можно ехать».
            self._red_run[camera] = 0
            self._green_run[camera] = 0
            if self._blocked:
                self._reason = "красный подтверждён; ждём зелёный"
            else:
                self._reason = "светофор не распознан"
        return self.snapshot(now=now)

    def observe_signs(self, names: Any, *, now: Optional[float] = None) -> dict[str, Any]:
        """Классы знаков в одном кадре (любая камера): влияет только «неровность»."""
        now = self._clock() if now is None else float(now)
        names = {str(name) for name in (names or ())}
        if not self.config.enabled or not self.config.slow_bump:
            self._bump_run = 0
            self._bump_until = None
            self._bump_last_seen = None
            return self.snapshot(now=now)
        if names & set(BUMP_CLASSES):
            self._bump_run += 1
            self._bump_last_seen = now
            self._bump_until = now + self.config.bump_hold_s
        else:
            self._bump_run = 0
            # Знак ушёл из кадра — держим снижение до конца «хвоста».
            if self._bump_last_seen is not None:
                self._bump_until = min(self._bump_until or 0.0,
                                       self._bump_last_seen + self.config.bump_hold_s)
        return self.snapshot(now=now)

    # ------------------------------------------------------------- решение
    def snapshot(self, *, now: Optional[float] = None) -> dict[str, Any]:
        now = self._clock() if now is None else float(now)
        bump = bool(self._bump_until is not None and now <= self._bump_until)
        bump_left = None
        if bump and self._bump_until is not None:
            bump_left = round(max(0.0, self._bump_until - now), 2)
        if bump:
            cap = max(0, int(round(100 * (1.0 - self.config.slowdown))))
        else:
            cap = 100
        blocked = bool(self._blocked)
        reason = self._reason
        if not self.config.enabled:
            reason = "ограничения по знакам выключены (RC_GUARD_ENABLED=0)"
        return {
            "enabled": self.config.enabled,
            "blocked": blocked,
            "blockedReason": reason if blocked else None,
            "blockedS": (round(max(0.0, now - self._blocked_at), 1)
                         if blocked and self._blocked_at is not None else None),
            "bump": bump,
            "bumpS": bump_left,
            #: Потолок тяги WASD в процентах, 0…100: его и применяет борт.
            "speedCapPercent": cap,
            "slowdown": self.config.slowdown if bump else 0.0,
            "blockedCamera": self._blocked_by,
            "redFrames": max(self._red_run.values(), default=0),
            "greenFrames": max(self._green_run.values(), default=0),
            "reason": reason,
            #: Ограничитель не управляет моторами: он лишь запрещает тягу и
            #: сужает потолок. Моторы ведёт ``KeyboardControl``/``slam``.
            "controlsMotors": False,
            "stale": (self._last_observation is not None and
                      now - self._last_observation > self.config.stale_s),
        }

    def release(self, reason: str = "оператор снял блокировку") -> None:
        """Явное снятие блокировки (оператор/перезапуск сервиса), не таймаут."""
        self._blocked = False
        self._blocked_at = None
        self._blocked_by = None
        self._red_run.clear()
        self._green_run.clear()
        self._reason = reason

    def reset(self) -> None:
        """Полный сброс наблюдений: включение сервиса, новая сессия распознавания."""
        self._red_run.clear()
        self._green_run.clear()
        self._blocked = False
        self._blocked_at = None
        self._blocked_by = None
        self._bump_run = 0
        self._bump_until = None
        self._bump_last_seen = None
        self._last_observation = None
        self._reason = "наблюдений ещё не было"
