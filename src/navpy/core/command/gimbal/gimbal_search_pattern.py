"""The search scan's profile-defined pattern (HITL step 21, docs/design/hitl_guidance.md section 9).

A raster of gimbal targets in the readback frame (yaw right-positive and pitch up-positive, relative to the
aircraft body, as the SIYI 0x0D readback is in FOLLOW mode): each pitch row is swept from one yaw end to the other,
the rows are visited down and back up, and the cycle repeats. Nothing in it depends on a dock, the aircraft's
position, heading, altitude or speed: the same profile gives the same targets in every scenario.

Profile (a device's ``gimbal.search``; every value is the profile's, none is defaulted here):

    "search": {"yaw_deg": [-60, 60], "pitch_rows_deg": [-15, -35], "rate_dps": 20.0, "tolerance_deg": 1.0,
               "approach_s": 0.5, "readback_max_age_s": 0.5, "frame_max_age_s": 1.0, "clear_s": 2.0}
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

# The scan stays inside the ZR10's body-relative mechanical range (siyi_sdk.ZR10); a profile outside it is wrong.
SEARCH_YAW_RANGE_DEG = (-135.0, 135.0)
SEARCH_PITCH_RANGE_DEG = (-90.0, 25.0)


@dataclass(frozen=True)
class GimbalSearchTiming:
    """When the scan may move: fresh readback, fresh frames, and nothing detected for ``clear_s``."""

    readback_max_age_s: float
    frame_max_age_s: float
    clear_s: float


@dataclass(frozen=True)
class GimbalSearchPattern:
    yaw_deg: tuple[float, float]
    pitch_rows_deg: tuple[float, ...]
    rate_dps: float
    tolerance_deg: float
    approach_s: float
    timing: GimbalSearchTiming

    def __post_init__(self) -> None:
        problems = _problems(self)
        if problems:
            raise ValueError("gimbal search: " + "; ".join(problems))

    @property
    def targets(self) -> tuple[tuple[float, float], ...]:
        """(yaw, pitch) targets of one cycle: each row visit moves to the row at the current yaw end, then sweeps
        to the other end; rows go down and back up, so consecutive cycles join without a jump."""
        rows = len(self.pitch_rows_deg)
        order = list(range(rows)) + list(range(rows - 2, 0, -1))
        ends = (self.yaw_deg[0], self.yaw_deg[1])
        side, targets = 0, []
        for _ in range(2 if len(order) % 2 else 1):    # an odd visit count needs two passes to return to the start
            for row in order:
                pitch = self.pitch_rows_deg[row]
                targets += [(ends[side], pitch), (ends[1 - side], pitch)]
                side = 1 - side
        return tuple(targets)

    @classmethod
    def from_profile(cls, spec: Mapping[str, object]) -> "GimbalSearchPattern":
        try:
            yaw = _numbers(spec["yaw_deg"])
            rows = _numbers(spec["pitch_rows_deg"])
            if len(yaw) != 2:
                raise ValueError(f"yaw_deg needs [min, max], got {spec['yaw_deg']!r}")
            return cls(
                (yaw[0], yaw[1]), rows, _number(spec["rate_dps"]), _number(spec["tolerance_deg"]),
                _number(spec["approach_s"]),
                GimbalSearchTiming(_number(spec["readback_max_age_s"]), _number(spec["frame_max_age_s"]),
                                   _number(spec["clear_s"])),
            )
        except KeyError as exc:
            raise ValueError(f"gimbal search: profile misses {exc.args[0]!r}") from None


def _number(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"gimbal search: {value!r} is not a number")
    return float(value)


def _numbers(value: object) -> tuple[float, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"gimbal search: {value!r} is not a list of numbers")
    return tuple(_number(item) for item in value)


def _problems(pattern: GimbalSearchPattern) -> list[str]:
    timing = pattern.timing
    values = (*pattern.yaw_deg, *pattern.pitch_rows_deg, pattern.rate_dps, pattern.tolerance_deg,
              pattern.approach_s, timing.readback_max_age_s, timing.frame_max_age_s, timing.clear_s)
    if not all(math.isfinite(value) for value in values):
        return ["every value must be finite"]
    problems = []
    low, high = pattern.yaw_deg
    if not SEARCH_YAW_RANGE_DEG[0] <= low < high <= SEARCH_YAW_RANGE_DEG[1]:
        problems.append(f"yaw_deg {pattern.yaw_deg} must rise inside {SEARCH_YAW_RANGE_DEG}")
    if not pattern.pitch_rows_deg:
        problems.append("pitch_rows_deg is empty")
    if any(not SEARCH_PITCH_RANGE_DEG[0] <= row <= SEARCH_PITCH_RANGE_DEG[1] for row in pattern.pitch_rows_deg):
        problems.append(f"pitch_rows_deg {pattern.pitch_rows_deg} must lie inside {SEARCH_PITCH_RANGE_DEG}")
    if len(set(pattern.pitch_rows_deg)) != len(pattern.pitch_rows_deg):
        problems.append("pitch_rows_deg repeats a row")
    if not 0.0 < pattern.tolerance_deg < (high - low) / 2.0:
        problems.append("tolerance_deg must be above 0 and below half the yaw span")
    for name, value in (("rate_dps", pattern.rate_dps), ("approach_s", pattern.approach_s),
                        ("readback_max_age_s", timing.readback_max_age_s),
                        ("frame_max_age_s", timing.frame_max_age_s)):
        if value <= 0.0:
            problems.append(f"{name} must be above 0")
    if timing.clear_s < 0.0:
        problems.append("clear_s must not be negative")
    return problems


__all__ = ["GimbalSearchPattern", "GimbalSearchTiming", "SEARCH_PITCH_RANGE_DEG", "SEARCH_YAW_RANGE_DEG"]
