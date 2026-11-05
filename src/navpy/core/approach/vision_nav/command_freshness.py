"""Raw producer-clock freshness policy for final-approach commands."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

from navpy.core.approach.vision_nav.source_time_ports import SourceNow


FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S = 1.0


@dataclass(frozen=True)
class FinalApproachCommandTiming:
    source_timestamp_s: float
    source_now_s: SourceNow | None
    receipt_timestamp_s: float | None
    receipt_now_s: SourceNow | None


@dataclass(frozen=True)
class FinalApproachBoundCheck:
    """One freshness bound with the values sampled while it was checked.

    ``problem`` is ``None`` when the bound held; otherwise it names why not.
    ``timestamp`` and ``now`` are the raw values read, before conversion.
    """

    name: str
    timestamp: object
    now: object
    age_s: float | None
    limit_s: float
    problem: str | None

    @property
    def fresh(self) -> bool:
        return self.problem is None

    def describe(self) -> str:
        age = "None" if self.age_s is None else f"{self.age_s:.4f}"
        return (
            f"{self.name}: {self.problem or 'ok'} age_s={age} "
            f"limit_s={self.limit_s:.4f} stamp={self.timestamp!r} "
            f"now={self.now!r}"
        )


@dataclass(frozen=True)
class FinalApproachFreshnessCheck:
    """Both bounds; ``receipt`` is ``None`` when the source bound already failed."""

    source: FinalApproachBoundCheck
    receipt: FinalApproachBoundCheck | None

    @property
    def fresh(self) -> bool:
        return (
            self.source.fresh
            and self.receipt is not None
            and self.receipt.fresh
        )

    def describe(self) -> str:
        receipt = (
            "receipt: not checked" if self.receipt is None
            else self.receipt.describe()
        )
        return f"{self.source.describe()}; {receipt}"


class FinalApproachCommandFreshness:
    """Require both producer progress and speedup-aware receipt liveness."""

    def __init__(self, wall_period_s: Callable[[float], float]) -> None:
        self._wall_period_s = wall_period_s

    def is_fresh(
        self,
        timing: FinalApproachCommandTiming,
        on_stale: Callable[[FinalApproachFreshnessCheck], None] | None = None,
    ) -> bool:
        """``on_stale`` names the failed bound (todo 230); it never decides."""
        check = self.check(timing)
        if not check.fresh and on_stale is not None:
            try:
                on_stale(check)
            except Exception:  # noqa: BLE001 - a diagnostic sink must not decide
                pass
        return check.fresh

    def check(
        self,
        timing: FinalApproachCommandTiming,
    ) -> FinalApproachFreshnessCheck:
        """Check each clock once; the receipt bound only if the source bound held."""
        source = _check_age(
            "source",
            timing.source_timestamp_s,
            timing.source_now_s,
            FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S,
        )
        if not source.fresh:
            return FinalApproachFreshnessCheck(source, None)
        receipt = _check_age(
            "receipt",
            timing.receipt_timestamp_s,
            timing.receipt_now_s,
            self._receipt_max_age_s(),
        )
        return FinalApproachFreshnessCheck(source, receipt)

    def _receipt_max_age_s(self) -> float:
        try:
            allowance_s = float(
                self._wall_period_s(FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S)
            )
        except Exception:  # noqa: BLE001 - invalid cadence fails closed
            return math.nan
        return allowance_s if allowance_s > 0.0 else math.nan


def _check_age(
    name: str,
    timestamp_value: float | None,
    now_provider: SourceNow | None,
    maximum_age_s: float,
) -> FinalApproachBoundCheck:
    """Compare one clock domain without rewriting, scaling, or extrapolation."""
    def result(
        problem: str | None,
        now: object = None,
        age_s: float | None = None,
    ) -> FinalApproachBoundCheck:
        return FinalApproachBoundCheck(
            name, timestamp_value, now, age_s, maximum_age_s, problem,
        )

    if timestamp_value is None:
        return result("missing_timestamp")
    if now_provider is None:
        return result("missing_clock")
    try:
        raw_now_s = now_provider()
    except Exception as error:  # noqa: BLE001 - invalid source clock fails closed
        return result(f"clock_error:{type(error).__name__}")
    if isinstance(timestamp_value, bool) or isinstance(raw_now_s, bool):
        return result("invalid_value", raw_now_s)
    try:
        timestamp_s = float(timestamp_value)
        now_s = float(raw_now_s)
        limit_s = float(maximum_age_s)
    except (TypeError, ValueError):
        return result("invalid_value", raw_now_s)
    age_s = now_s - timestamp_s
    if not (math.isfinite(timestamp_s) and math.isfinite(now_s)):
        return result("non_finite_clock", raw_now_s)
    if not math.isfinite(age_s):
        return result("non_finite_age", raw_now_s, age_s)
    if not (math.isfinite(limit_s) and limit_s > 0.0):
        return result("invalid_limit", raw_now_s, age_s)
    if age_s < 0.0:
        return result("backward_clock", raw_now_s, age_s)
    if age_s > limit_s:
        return result("too_old", raw_now_s, age_s)
    return result(None, raw_now_s, age_s)


__all__ = [
    "FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S",
    "FinalApproachBoundCheck",
    "FinalApproachCommandFreshness",
    "FinalApproachFreshnessCheck",
    "FinalApproachCommandTiming",
]
