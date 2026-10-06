"""Strictly bracketed attitude interpolation on the receipt axis.

Split from `capture_lookup.py`, which owns the calibration contract and the
clock algebra; this module owns turning a lookup instant plus the retained
ATTITUDE history into an attitude — or a named refusal. Everything that
cannot be done exactly fails closed (decision doc §3c).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Sequence

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vehicle.pose_streams import (
    POSE_FRAME_ASSOCIATION_MAX_PERIODS,
    POSE_STREAM_RATE_HZ,
)

# Seam detection compares receipt-time progression against boot-time
# progression. Per-sample link/scheduler jitter must stay inside one ATTITUDE
# period for the stream to hold cadence at all, while clock steps and reboots
# exceed it by orders of magnitude. Independent of the lookup budget.
CLOCK_SEAM_TOLERANCE_S = 1.0 / POSE_STREAM_RATE_HZ

# A valid bracket must be two ADJACENT retained samples no further apart than
# the stream's own two-period association bound; a packet-loss gap is not a
# bracket.
BRACKET_MAX_WIDTH_S = POSE_FRAME_ASSOCIATION_MAX_PERIODS / POSE_STREAM_RATE_HZ

STATUS_UNBRACKETED = "real_frame_pose_unbracketed"
STATUS_GAP = "real_frame_pose_gap"
STATUS_CLOCK_SEAM = "real_frame_pose_clock_seam"


@dataclass(frozen=True)
class AttitudeAtLookup:
    attitude: Attitude
    body_rates_rad_s: tuple[float, float, float] | None
    time_boot_s: float | None


def _finite(value: object) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _lerp_angle_deg(first: float, second: float, fraction: float) -> float:
    delta = (second - first + 180.0) % 360.0 - 180.0
    return first + delta * fraction


def _pair_is_seam(
    earlier: SimpleNamespace,
    later: SimpleNamespace,
    seam_tolerance_s: float = CLOCK_SEAM_TOLERANCE_S,
) -> bool:
    receipt_delta = later.receipt_time_s - earlier.receipt_time_s
    if receipt_delta < 0.0:
        return True
    earlier_boot = _finite(earlier.time_boot_s)
    later_boot = _finite(later.time_boot_s)
    if earlier_boot is None or later_boot is None:
        # A sample that cannot be seam-checked cannot anchor a bracket.
        return True
    boot_delta = later_boot - earlier_boot
    if boot_delta < 0.0:
        return True
    return abs(receipt_delta - boot_delta) > seam_tolerance_s


def interpolate_attitude_at(
    history: Sequence[SimpleNamespace],
    lookup_wall_s: float,
    quarantine_s: float,
    bracket_max_width_s: float = BRACKET_MAX_WIDTH_S,
    seam_tolerance_s: float = CLOCK_SEAM_TOLERANCE_S,
) -> tuple[AttitudeAtLookup | None, str]:
    """Strictly bracketed interpolation on the receipt axis, or a refusal.

    Refusals (decision doc §3c): no bracket (including a lookup newer than
    the newest sample by ANY margin), a bracket wider than the stream's
    two-period cadence bound, and any clock seam either inside the bracket's
    epoch path or inside the quarantine window before the lookup. The width
    and seam bounds default to the 40 Hz fallback but MUST be supplied from
    the scheduler-derived runtime rate by the association builder -- an
    80 Hz stream's two-period ceiling is 25 ms, not 50 ms.
    """
    if len(history) < 2:
        return None, STATUS_UNBRACKETED

    seam_receipts: list[float] = []
    for index in range(1, len(history)):
        if _pair_is_seam(history[index - 1], history[index], seam_tolerance_s):
            seam_receipts.append(history[index].receipt_time_s)

    bracket = None
    for index in range(1, len(history)):
        earlier, later = history[index - 1], history[index]
        if earlier.receipt_time_s <= lookup_wall_s <= later.receipt_time_s:
            bracket = (earlier, later)
    if bracket is None:
        return None, STATUS_UNBRACKETED
    earlier, later = bracket
    if _pair_is_seam(earlier, later, seam_tolerance_s):
        return None, STATUS_CLOCK_SEAM
    for seam_receipt in seam_receipts:
        if lookup_wall_s - quarantine_s < seam_receipt <= lookup_wall_s:
            return None, STATUS_CLOCK_SEAM
    width = later.receipt_time_s - earlier.receipt_time_s
    if width > bracket_max_width_s:
        return None, STATUS_GAP

    if width <= 0.0:
        fraction = 0.0
    else:
        fraction = (lookup_wall_s - earlier.receipt_time_s) / width
    first_att, second_att = earlier.attitude, later.attitude
    attitude = Attitude(
        pitch=_lerp_angle_deg(first_att.pitch, second_att.pitch, fraction),
        yaw=_lerp_angle_deg(first_att.yaw, second_att.yaw, fraction),
        roll=_lerp_angle_deg(first_att.roll, second_att.roll, fraction),
    )
    rates = None
    if (
        earlier.body_rates_rad_s is not None
        and later.body_rates_rad_s is not None
    ):
        rates = tuple(
            first + (second - first) * fraction
            for first, second in zip(
                earlier.body_rates_rad_s, later.body_rates_rad_s
            )
        )
    earlier_boot = _finite(earlier.time_boot_s)
    later_boot = _finite(later.time_boot_s)
    boot = None
    if earlier_boot is not None and later_boot is not None:
        boot = earlier_boot + (later_boot - earlier_boot) * fraction
    return AttitudeAtLookup(attitude, rates, boot), "bracketed"


__all__ = [
    "AttitudeAtLookup",
    "BRACKET_MAX_WIDTH_S",
    "CLOCK_SEAM_TOLERANCE_S",
    "STATUS_CLOCK_SEAM",
    "STATUS_GAP",
    "STATUS_UNBRACKETED",
    "interpolate_attitude_at",
]
