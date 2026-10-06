"""Mutable recovery-cycle and steady-state drift state."""

from __future__ import annotations

from enum import Enum


class _State(Enum):
    IDLE = "idle"
    SETTLING = "settling"
    VERIFY = "verify"


class FocusRecoveryState:
    """Own zoom baselining and autofocus-cycle transitions."""

    def __init__(self) -> None:
        self.reset()

    @property
    def phase(self) -> _State:
        return self._phase

    @property
    def attempts(self) -> int:
        return self._attempts

    @property
    def focus_before(self) -> float:
        return self._focus_before

    def reset(self) -> None:
        self._phase = _State.IDLE
        self._last_zoom: float | None = None
        self._deadline = 0.0
        self._attempts = 0
        self._focus_before = 0.0

    def record_zoom(self, zoom: float | None, epsilon: float) -> bool:
        if zoom is None:
            return False
        if self._last_zoom is None:
            self._last_zoom = zoom
            return False
        if abs(zoom - self._last_zoom) > epsilon:
            self._last_zoom = zoom
            return True
        return False

    def begin_settle(self, now: float, settle_delay: float) -> None:
        self._phase = _State.SETTLING
        self._deadline = now + settle_delay
        self._attempts = 0

    def make_ready(self, now: float) -> None:
        self._deadline = now

    def ready(self, now: float) -> bool:
        return now >= self._deadline

    def record_focus_before(self, focus: float) -> None:
        self._focus_before = focus

    def record_autofocus(self, now: float, af_settle: float) -> None:
        self._attempts += 1
        self._phase = _State.VERIFY
        self._deadline = now + af_settle

    def finish(self) -> None:
        self._phase = _State.IDLE


class FocusDriftState:
    """Own rolling focus samples and sustained-low debounce state."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._peak: list[tuple[float, float]] = []
        self._low_since: float | None = None

    def seed(self, now: float, focus: float) -> None:
        self._peak = [(now, focus)]
        self._low_since = None

    def observe(
            self,
            now: float,
            focus: float,
            *,
            window: float,
            ratio: float,
            seconds: float,
    ) -> bool:
        self._peak.append((now, focus))
        self._peak = [
            (sample_time, sample_focus)
            for sample_time, sample_focus in self._peak
            if now - sample_time <= window
        ]
        peak = max((sample_focus for _, sample_focus in self._peak), default=0.0)
        if peak > 0.0 and focus < ratio * peak:
            if self._low_since is None:
                self._low_since = now
            elif now - self._low_since >= seconds:
                return True
        else:
            self._low_since = None
        return False


__all__ = ["FocusDriftState", "FocusRecoveryState"]
