"""Transactional vertical visual-rate estimator."""

from __future__ import annotations

import math
from dataclasses import dataclass

from navpy.modules.navigation.nav.vision_nav.frame import TerminalVisionFrame


@dataclass(frozen=True)
class VerticalRateState:
    continuity_key: tuple[str, int, int, int]
    timestamp_s: float
    delta_rad: float
    filtered_rate_rad_s: float


@dataclass(frozen=True)
class VerticalRatePlan:
    # Interval mean consumed by the pitch integrator. The endpoint filter
    # state is stored separately in next_state for the next observation.
    rate_rad_s: float
    next_state: VerticalRateState


class VerticalRateFilter:
    """Calculate state changes without committing them before actuation."""

    def __init__(self) -> None:
        self._state: VerticalRateState | None = None

    def reset(self) -> None:
        self._state = None

    def seed(self, frame: TerminalVisionFrame) -> None:
        delta = _vertical_angle(frame)
        self._state = VerticalRateState(
            frame.continuity_key,
            frame.source_timestamp_s,
            delta,
            0.0,
        )

    def plan(self, frame: TerminalVisionFrame, tau_s: float | None) -> VerticalRatePlan:
        delta = _vertical_angle(frame)
        previous = self._state
        filtered = 0.0
        interval_mean = 0.0
        if previous is not None and previous.continuity_key == frame.continuity_key:
            dt_s = frame.source_timestamp_s - previous.timestamp_s
            if dt_s <= 0.0:
                return VerticalRatePlan(0.0, previous)
            raw_rate = (delta - previous.delta_rad) / dt_s
            filtered = _low_pass(raw_rate, previous.filtered_rate_rad_s, dt_s, tau_s)
            if tau_s is not None and tau_s > 0.0:
                # The measured derivative is constant over this frame interval.
                # Integrate tau*f' + f = raw exactly: endpoint*dt would add a
                # cadence-dependent command offset, even after a closed visual
                # excursion settles. Keep the same endpoint for filter history.
                ratio = dt_s / tau_s
                mean_weight = 1.0 + math.expm1(-ratio) / ratio
                interval_mean = previous.filtered_rate_rad_s + mean_weight * (
                    raw_rate - previous.filtered_rate_rad_s
                )
        state = VerticalRateState(
            frame.continuity_key,
            frame.source_timestamp_s,
            delta,
            filtered,
        )
        return VerticalRatePlan(interval_mean, state)

    def commit(self, plan: VerticalRatePlan) -> None:
        self._state = plan.next_state


def _vertical_angle(frame: TerminalVisionFrame) -> float:
    return math.atan2(
        frame.control_z,
        math.hypot(frame.control_x, frame.control_y),
    )


def _low_pass(raw: float, previous: float, dt_s: float, tau_s: float | None) -> float:
    if tau_s is None or tau_s <= 0.0:
        return 0.0
    alpha = math.exp(-dt_s / tau_s)
    return alpha * previous + (1.0 - alpha) * raw


__all__ = ["VerticalRateFilter", "VerticalRatePlan", "VerticalRateState", "_low_pass"]
