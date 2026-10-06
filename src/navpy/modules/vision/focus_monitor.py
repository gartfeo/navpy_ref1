"""Post-zoom autofocus recovery for the SIYI camera.

A zoom change shifts the lens focal plane, so the image comes up soft until
autofocus runs — and through real optics (e.g. a window) a single AF often
fails to lock, needing two or three tries. The SIYI zoom controller
deliberately does NOT autofocus, so nothing recovers focus automatically.

``FocusMonitor`` observes the reported zoom value and, after any change,
drives autofocus and KEEPS re-triggering it until the focus metric stops
improving (or an attempt cap is hit). This needs no scene-dependent absolute
threshold: AF "helped" iff sharpness rose, so the loop self-terminates exactly
when the image is sharp. A secondary steady-state guard re-runs the same cycle
if sharpness later collapses relative to its recent peak at an unchanged zoom.

The monitor owns no hardware and no frames: it depends only on an injected
``request_autofocus`` callable and a focus metric, so it is fully unit-testable.
"""
from __future__ import annotations

import time
from typing import Callable, Optional, Sequence

import cv2
import numpy as np

from navpy.modules.vision.focus_policy import FocusPolicy as _FocusPolicy
from navpy.modules.vision.focus_state import (
    _State,
    FocusDriftState as _FocusDriftState,
    FocusRecoveryState as _FocusRecoveryState,
)


def laplacian_focus(frame: np.ndarray, bbox_cxcywh: Optional[Sequence[float]] = None) -> float:
    """Variance-of-Laplacian sharpness. Higher = sharper. Scene-dependent
    absolute scale, but monotonic for a fixed scene+zoom — which is all the
    monitor relies on (it compares sharpness before/after AF at one zoom).

    When ``bbox_cxcywh`` is given the metric is computed on that crop (the
    region the operator cares about); otherwise on the full frame.
    """
    if frame is None:
        return 0.0
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    h, w = gray.shape[:2]
    if bbox_cxcywh is not None:
        cx, cy, bw, bh = bbox_cxcywh
        x0, y0 = int(max(0, cx - bw / 2)), int(max(0, cy - bh / 2))
        x1, y1 = int(min(w, cx + bw / 2)), int(min(h, cy + bh / 2))
        if x1 - x0 >= 8 and y1 - y0 >= 8:
            gray = gray[y0:y1, x0:x1]
    if gray.size == 0:
        return 0.0
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


class FocusMonitor:
    def __init__(
            self,
            request_autofocus: Callable[[], None],
            *,
            focus_metric: Callable[..., float] = laplacian_focus,
            clock: Callable[[], float] = time.monotonic,
            settle_delay: float = 0.7,     # lens transit after a zoom command
            af_settle: float = 0.8,        # time for one AF to act before re-measuring
            improve_ratio: float = 1.10,   # AF "helped" iff sharpness rose >10%
            max_attempts: int = 4,         # AF tries per recovery cycle
            # Zoom delta that counts as a change. Sized ABOVE the SIYI hardware
            # readback quantization (~0.1) + its jitter so sensor noise on a
            # steady zoom does not thrash autofocus; a real zoom step is >= 1.0
            # (level) or a deliberate continuous nudge, both well above this.
            zoom_epsilon: float = 0.15,
            drift_ratio: float = 0.45,     # steady-state: blurry below this * peak
            drift_seconds: float = 1.5,    # sustained blur before a drift recovery
            drift_window: float = 6.0,     # rolling window for the steady-state peak
    ):
        self._af = request_autofocus
        self._focus = focus_metric
        self._clock = clock
        self._policy = _FocusPolicy.from_values(
            settle_delay=settle_delay,
            af_settle=af_settle,
            improve_ratio=improve_ratio,
            max_attempts=max_attempts,
            zoom_epsilon=zoom_epsilon,
            drift_ratio=drift_ratio,
            drift_seconds=drift_seconds,
            drift_window=drift_window,
        )
        self._recovery = _FocusRecoveryState()
        self._drift = _FocusDriftState()
        self.reset()

    @property
    def state(self) -> str:
        return self._recovery.phase.value

    def reset(self) -> None:
        self._recovery.reset()
        self._drift.reset()

    def tick(
            self,
            frame: Optional[np.ndarray],
            current_zoom: Optional[float],
            now: Optional[float] = None,
            bbox_cxcywh: Optional[Sequence[float]] = None,
    ) -> bool:
        """Advance one step. Returns True iff autofocus was requested now.

        Never propagates an exception (the focus metric runs on the operator
        main loop): a transient frame/cv2 error degrades to "no AF this tick".
        """
        try:
            return self._tick(frame, current_zoom, now, bbox_cxcywh)
        except Exception:  # noqa: BLE001 — monitoring must not break the loop
            return False

    def _tick(self, frame, current_zoom, now, bbox_cxcywh) -> bool:
        now = self._clock() if now is None else float(now)
        if frame is None:
            return False
        zoom = _as_float(current_zoom)

        # First time we see a zoom, establish the baseline WITHOUT a cycle —
        # only an actual change should drive autofocus.
        if self._recovery.record_zoom(zoom, self._policy.zoom_epsilon):
            self._begin_settle(now)
            return False

        if self._recovery.phase is _State.SETTLING:
            if self._recovery.ready(now):
                self._recovery.record_focus_before(
                    self._focus(frame, bbox_cxcywh)
                )
                return self._fire_af(now)
            return False

        if self._recovery.phase is _State.VERIFY:
            if self._recovery.ready(now):
                focus_now = self._focus(frame, bbox_cxcywh)
                improved = (
                    focus_now
                    > self._recovery.focus_before * self._policy.improve_ratio
                )
                if (
                        improved
                        and self._recovery.attempts < self._policy.max_attempts
                ):
                    self._recovery.record_focus_before(focus_now)
                    return self._fire_af(now)
                # converged (sharp) or out of tries: settle into steady state
                self._recovery.finish()
                self._seed_peak(now, focus_now)
            return False

        # IDLE: steady-state drift guard (focus collapse without a zoom change).
        return self._drift_guard(frame, bbox_cxcywh, now)

    # -- internals -----------------------------------------------------------
    def _begin_settle(self, now: float) -> None:
        self._recovery.begin_settle(now, self._policy.settle_delay)
        self._drift.reset()

    def _fire_af(self, now: float) -> bool:
        self._af()
        self._recovery.record_autofocus(now, self._policy.af_settle)
        return True

    def _seed_peak(self, now: float, focus: float) -> None:
        self._drift.seed(now, focus)

    def _drift_guard(self, frame, bbox, now: float) -> bool:
        focus = self._focus(frame, bbox)
        if self._drift.observe(
                now,
                focus,
                window=self._policy.drift_window,
                ratio=self._policy.drift_ratio,
                seconds=self._policy.drift_seconds,
        ):
            # treat as a fresh recovery: AF until sharp again
            self._begin_settle(now)
            self._recovery.make_ready(now)  # no lens transit to wait on here
        return False


def _as_float(value) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None
