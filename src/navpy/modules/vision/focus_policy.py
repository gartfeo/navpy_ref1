"""Immutable thresholds for autofocus recovery and drift detection."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FocusPolicy:
    settle_delay: float
    af_settle: float
    improve_ratio: float
    max_attempts: int
    zoom_epsilon: float
    drift_ratio: float
    drift_seconds: float
    drift_window: float

    @classmethod
    def from_values(
            cls,
            *,
            settle_delay: float,
            af_settle: float,
            improve_ratio: float,
            max_attempts: int,
            zoom_epsilon: float,
            drift_ratio: float,
            drift_seconds: float,
            drift_window: float,
    ) -> "FocusPolicy":
        return cls(
            settle_delay=float(settle_delay),
            af_settle=float(af_settle),
            improve_ratio=float(improve_ratio),
            max_attempts=max(1, int(max_attempts)),
            zoom_epsilon=float(zoom_epsilon),
            drift_ratio=float(drift_ratio),
            drift_seconds=float(drift_seconds),
            drift_window=float(drift_window),
        )


__all__ = ["FocusPolicy"]
