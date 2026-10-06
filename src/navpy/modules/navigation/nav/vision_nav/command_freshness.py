"""Raw producer-clock freshness policy for final-approach commands."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

from navpy.modules.navigation.nav.vision_nav.source_time_ports import SourceNow


FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S = 1.0


@dataclass(frozen=True)
class FinalApproachCommandTiming:
    source_timestamp_s: float
    source_now_s: SourceNow | None
    receipt_timestamp_s: float | None
    receipt_now_s: SourceNow | None


class FinalApproachCommandFreshness:
    """Require both producer progress and speedup-aware receipt liveness."""

    def __init__(self, wall_period_s: Callable[[float], float]) -> None:
        self._wall_period_s = wall_period_s

    def is_fresh(self, timing: FinalApproachCommandTiming) -> bool:
        return _age_is_fresh(
            timing.source_timestamp_s,
            timing.source_now_s,
            FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S,
        ) and _age_is_fresh(
            timing.receipt_timestamp_s,
            timing.receipt_now_s,
            self._receipt_max_age_s(),
        )

    def _receipt_max_age_s(self) -> float:
        try:
            allowance_s = float(
                self._wall_period_s(FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S)
            )
        except Exception:  # noqa: BLE001 - invalid cadence fails closed
            return math.nan
        return allowance_s if allowance_s > 0.0 else math.nan


def _age_is_fresh(
    timestamp_value: float | None,
    now_provider: SourceNow | None,
    maximum_age_s: float,
) -> bool:
    """Compare one clock domain without rewriting, scaling, or extrapolation."""
    if timestamp_value is None or now_provider is None:
        return False
    try:
        raw_now_s = now_provider()
    except Exception:  # noqa: BLE001 - invalid source clock fails closed
        return False
    if isinstance(timestamp_value, bool) or isinstance(raw_now_s, bool):
        return False
    try:
        timestamp_s = float(timestamp_value)
        now_s = float(raw_now_s)
        limit_s = float(maximum_age_s)
    except (TypeError, ValueError):
        return False
    age_s = now_s - timestamp_s
    return (
        math.isfinite(timestamp_s)
        and math.isfinite(now_s)
        and math.isfinite(age_s)
        and math.isfinite(limit_s)
        and limit_s > 0.0
        and 0.0 <= age_s <= limit_s
    )


__all__ = [
    "FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S",
    "FinalApproachCommandFreshness",
    "FinalApproachCommandTiming",
]
