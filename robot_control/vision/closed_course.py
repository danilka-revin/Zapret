"""Opt-in UNKNOWN timeout advice for a supervised, closed test course ONLY.

This is not a signal classifier, movement permission or actuator controller.
It never changes a detected colour/action, clears E-STOP or commands motors.
Five seconds means distinct, fresh observations of the same candidate light,
not five seconds after one frame or proof that the vehicle has stopped.
"""
from __future__ import annotations

import math
import time
from typing import Any, Callable


class ClosedCourseWaitPolicy:
    WAIT_S = 5.0

    def __init__(self, *, enabled: bool = False, stale_s: float = 1.0,
                 clock: Callable[[], float] | None = None) -> None:
        self.enabled = bool(enabled)
        self._clock = clock or time.monotonic
        self.stale_s = min(1.0, max(0.2, float(stale_s)))
        self.reset()

    def reset(self) -> None:
        """Explicit full observation reset; not a motor/E-STOP operation."""
        self._stop_latched = False
        self._frames: dict[int, float] = {}
        self.invalidate('no_detection')

    def invalidate(self, reason: str = 'no_detection') -> None:
        """Cancel waiting/advice but retain any previously seen stop signal."""
        self._phase = reason
        self._first: float | None = None
        self._last: float | None = None
        self._camera: int | None = None
        self._anchor: tuple | None = None
        self._box: tuple | None = None
        self._elapsed = 0.0

    @staticmethod
    def _iou(a: tuple, b: tuple) -> float:
        x, y = max(a[0], b[0]), max(a[1], b[1])
        w = max(0., min(a[0] + a[2], b[0] + b[2]) - x)
        h = max(0., min(a[1] + a[3], b[1] + b[3]) - y)
        intersection = w * h
        return intersection / max(1e-12, a[2] * a[3] + b[2] * b[3] - intersection)

    def update(self, *, camera_id: int, box: tuple, frame_stamp: float,
               frame_age_s: float, signal: str = 'unknown',
               confirmed_signal: str = 'unknown', stop_signal_present: bool = False,
               now: float | None = None) -> dict[str, Any]:
        """``now`` uses a monotonic clock; capture stamps only deduplicate frames."""
        now = self._clock() if now is None else float(now)
        if not self.enabled:
            return self.snapshot(now=now)
        if (not all(math.isfinite(value) for value in (now, frame_stamp, frame_age_s)) or
                not 0 <= frame_age_s <= self.stale_s or camera_id not in range(4) or
                len(box) != 4 or not all(math.isfinite(v) for v in box) or
                min(box) < 0 or box[2] <= 0 or box[3] <= 0 or
                box[0] + box[2] > 1.00001 or box[1] + box[3] > 1.00001):
            self.invalidate('invalid_or_stale_frame')
            return self.snapshot(now=now)
        previous = self._frames.get(camera_id)
        # A fresh red/yellow from ANY camera blocks immediately, even when the
        # selected light/frame has not changed. Unknown cannot clear this latch.
        if stop_signal_present or signal in ('red', 'yellow') or confirmed_signal in ('red', 'yellow'):
            self._stop_latched = True
            self.invalidate('stop_latched')
            self._frames[camera_id] = max(frame_stamp, previous if previous is not None else frame_stamp)
            return self.snapshot(now=now)
        if previous is not None and frame_stamp <= previous:
            if frame_stamp < previous:
                self.invalidate('out_of_order_frame')
            return self.snapshot(now=now)
        self._frames[camera_id] = frame_stamp
        if signal == confirmed_signal == 'green':
            self._stop_latched = False
            self.invalidate('confirmed_green')
            return self.snapshot(now=now)
        if self._stop_latched:
            self.invalidate('stop_latched')
            return self.snapshot(now=now)
        if signal not in ('unknown', None) or confirmed_signal != 'unknown':
            self.invalidate('awaiting_signal_confirmation')
            return self.snapshot(now=now)
        continuous = (self._camera == camera_id and self._last is not None and
                      0 <= now - self._last <= self.stale_s and previous is not None and
                      0 < frame_stamp - previous <= self.stale_s and
                      self._iou(self._box, box) >= .3 and self._iou(self._anchor, box) >= .3)
        if not continuous:
            self._first, self._anchor = now, box
        self._last, self._camera, self._box = now, camera_id, box
        self._elapsed = max(0., now - self._first)
        self._phase = 'timeout_advisory' if self._elapsed >= self.WAIT_S else 'waiting'
        return self.snapshot(now=now)

    def snapshot(self, *, now: float | None = None) -> dict[str, Any]:
        now = self._clock() if now is None else float(now)
        phase = self._phase if self.enabled else 'disabled'
        fresh = self._last is not None and math.isfinite(now) and 0 <= now - self._last <= self.stale_s
        if phase in ('waiting', 'timeout_advisory') and not fresh:
            phase = 'stale'
        proceed = phase == 'timeout_advisory' and fresh and not self._stop_latched
        elapsed = self._elapsed if fresh else 0.0
        return {
            'enabled': self.enabled,
            'mode': 'closed_course_advisory',
            'phase': phase,
            'waitS': self.WAIT_S,
            'elapsedS': round(elapsed, 3),
            'remainingS': round(max(0., self.WAIT_S - elapsed), 3) if phase == 'waiting' else None,
            'stopLatched': self._stop_latched,
            'recommendation': 'proceed_closed_course' if proceed else (
                'none' if phase in ('disabled', 'confirmed_green') else 'wait'),
            'controlsMotors': False,
        }
