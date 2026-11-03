"""Explicit integration timing for encoded HITL video (stage 1 step 11): config and limits.

An uncertified, HITL-only estimate: it never builds or stands in for a
``CaptureLookupCalibration``, and ``is_certified`` is False for everything it
admits. The map from frame PTS to receiver read time is ``pts_read_map.py``,
its slip checks ``integration_slip.py``.

Each other source is placed with its own receiver-local correction
C = Ds - Dc (source-link delay minus video delay, fitted offline by step 12):
``lookup = mapped + C``. No absolute delay is known or claimed: ``mapped`` is
later than exposure by the smallest video delay, so ages taken from it are
underestimated by that much. Visual rates use the frame's PTS (``interval``),
never mapped or read differences.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

AUTHORITY_CERTIFIED = "certified"
AUTHORITY_INTEGRATION = "integration_uncertified"
# The run modes that may use integration timing (tools/hitl/approach_records.MODE_SUBSTITUTIONS). Flight and
# Qualification are never in it.
INTEGRATION_MODES = frozenset({"pc-sitl"})
NS_PER_S = 1_000_000_000

STATUS_MAPPED = "mapped"
STATUS_UNAVAILABLE = "real_frame_integration_source_unavailable"
STATUS_UNSETTLED = "real_frame_integration_unsettled"
STATUS_UNKNOWN_FRAME = "real_frame_integration_unknown_frame"
STATUS_ABORTED = "real_frame_integration_aborted"


def _positive(value: object) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(value)
        and value > 0.0
    )


@dataclass(frozen=True)
class IntegrationTimingLimits:
    """Frozen per run in the run record. My values, synthetic candidates, not measured limits."""

    window_s: float = 2.0
    update_limit_s: float = 0.0001
    slip_limit_s: float = 0.001
    slip_window_s: float = 1.0
    # The clock rate is fitted over this much older history: a 2 s window
    # under ms jitter cannot tell 300 ppm from noise.
    rate_window_s: float = 10.0
    min_frames: int = 30
    max_frame_gap_s: float = 0.5
    max_frame_age_s: float = 0.5
    max_source_age_s: float = 0.2
    # Two SIYI poll periods (plugins/gimbal/siyi/hardware/polling.py, 20 Hz):
    # a wider gap between adjacent gimbal readbacks is not a bracket.
    gimbal_bracket_max_s: float = 0.1

    def problems(self) -> list[str]:
        names = (
            "window_s", "update_limit_s", "slip_limit_s", "slip_window_s", "rate_window_s",
            "max_frame_gap_s", "max_frame_age_s", "max_source_age_s", "gimbal_bracket_max_s",
        )
        found = [f"{name} must be a positive finite number" for name in names if not _positive(getattr(self, name))]
        if isinstance(self.min_frames, bool) or not isinstance(self.min_frames, int) or self.min_frames < 2:
            found.append("min_frames must be an integer >= 2")
        if found:
            return found
        if self.update_limit_s >= self.slip_limit_s:
            found.append("update_limit_s must be below slip_limit_s")
        if 2 * self.slip_window_s > self.window_s:
            found.append("slip_window_s must be at most half of window_s")
        if self.rate_window_s < self.window_s:
            found.append("rate_window_s must be at least window_s")
        return found


@dataclass(frozen=True)
class IntegrationTimingConfig:
    """One run's integration timing: who may use it, its limits and the corrections C = Ds - Dc."""

    run_id: str
    mode: str
    source_id: str
    link_id: str
    limits: IntegrationTimingLimits
    attitude_correction_s: float
    gimbal_correction_s: float

    def problems(self) -> list[str]:
        found = []
        if self.mode not in INTEGRATION_MODES:
            found.append(f"mode {self.mode!r} may not use integration timing")
        for name in ("run_id", "source_id", "link_id"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                found.append(f"{name} must be a non-empty string")
        if not isinstance(self.limits, IntegrationTimingLimits):
            found.append("limits must be IntegrationTimingLimits")
        else:
            found.extend(self.limits.problems())
        for name in ("attitude_correction_s", "gimbal_correction_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                found.append(f"{name} must be a finite number")
        return found

    def record(self) -> dict[str, object]:
        """The section frozen into the run record."""
        limits = self.limits
        return {
            "authority": AUTHORITY_INTEGRATION,
            "source_id": self.source_id,
            "link_id": self.link_id,
            "attitude_correction_s": self.attitude_correction_s,
            "gimbal_correction_s": self.gimbal_correction_s,
            "limits": asdict(limits),
        }


def is_certified(authority: str | None) -> bool:
    """The one check certified-only gates use: integration timing is never certified."""
    return authority == AUTHORITY_CERTIFIED


__all__ = [
    "AUTHORITY_CERTIFIED",
    "AUTHORITY_INTEGRATION",
    "INTEGRATION_MODES",
    "IntegrationTimingConfig",
    "IntegrationTimingLimits",
    "NS_PER_S",
    "STATUS_ABORTED",
    "STATUS_MAPPED",
    "STATUS_UNAVAILABLE",
    "STATUS_UNKNOWN_FRAME",
    "STATUS_UNSETTLED",
    "is_certified",
]
