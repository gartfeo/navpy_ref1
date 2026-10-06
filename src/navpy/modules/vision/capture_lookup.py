"""Capture-time attitude lookup: calibration contract and interpolation.

Implements the vision-nav capture-time association decision
(v7, approved). One clock domain: the ATTITUDE receipt wall axis. An attitude
generated at exposure time E appears there at E + Da, so the frame's stamp is
mapped to `attitude_lookup_wall_s` and the retained ATTITUDE history is
interpolated strictly inside a bracket at that instant. Everything that cannot
be done exactly fails closed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from navpy.modules.vehicle.pose_streams import (
    POSE_FRAME_ASSOCIATION_ABSOLUTE_MAX_SKEW_S,
)
from navpy.modules.vision.capture_interpolation import (
    AttitudeAtLookup,
    BRACKET_MAX_WIDTH_S,
    CLOCK_SEAM_TOLERANCE_S,
    STATUS_CLOCK_SEAM,
    STATUS_GAP,
    STATUS_UNBRACKETED,
    interpolate_attitude_at,
)
from navpy.modules.vision.frame_capture_ports import (
    CaptureStamp,
    CaptureStampKind,
)

# Derived from measured evidence, not interpolation structure: the SITL bench
# measured the sign-definite de-rotation mechanism becoming damaging from
# ~16-24 ms of systematic ray/attitude skew (arrival-offset instrumentation
# plus the delay-sweep minimum). The budget is that onset floor with a
# factor-2 safety margin. At the certified ~100 deg/s roll-rate envelope this
# bounds the de-rotation attitude error at ~0.8 deg.
TIMING_RESIDUAL_BUDGET_S = 0.008

# The differentiators divide by the estimated frame interval, so with true
# interval T and worst permitted interval error 2*camera_variation_bound_s the
# estimated LOS rate scales by A(T) = T / (T - 2*b). Admission (below) keeps
# A(T) <= this limit for every admitted source: a transient, sign-varying
# rate error of at most 25%, an order below the removed systematic sign-
# definite feedback. Provisional; the calibration follow-up refines it from
# flight evidence.
RATE_AMPLIFICATION_LIMIT = 1.25

STATUS_UNCALIBRATED = "real_frame_capture_uncalibrated"


@dataclass(frozen=True)
class CaptureLookupCalibration:
    """Certified timing artifact for one frame source on one link.

    Every bound is a certified MAXIMUM ABSOLUTE value over the stated
    certification envelope (see the decision doc §2); the lag-scan tooling
    refuses to write an over-budget artifact, and `validate_calibration`
    refuses a declared-over-budget one at runtime.
    """

    source_id: str
    link_id: str
    capture_lookup_bias_s: float      # measured Dc0 - Da0 (signed)
    attitude_link_delay_s: float      # measured Da0, >= 0
    residual_bound_s: float           # certified max |dc - da|
    camera_variation_bound_s: float   # certified max |dc|
    attitude_variation_bound_s: float # certified max |da|
    min_frame_interval_s: float       # certified minimum true frame interval
    calibrated_at: str = ""


def _finite(value: object) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def validate_calibration(
    calibration: CaptureLookupCalibration | None,
    active_source_id: str,
    active_link_id: str | None,
) -> str | None:
    """Return a refusal reason, or None when the artifact admits the source.

    The certified artifact is the admission authority; field presence alone
    admits nothing, and the live-link identity must come from the open
    connection (`PoseTelemetry.link_identity`), never from configuration.
    A malformed or partially deserialized artifact must REFUSE, not raise:
    the caller maps any refusal to `real_frame_capture_uncalibrated`.
    """
    if calibration is None:
        return "calibration absent"
    try:
        return _validate_calibration_fields(
            calibration, active_source_id, active_link_id,
        )
    except (AttributeError, TypeError, ValueError):
        return "calibration record is malformed"


def _validate_calibration_fields(
    calibration: CaptureLookupCalibration,
    active_source_id: str,
    active_link_id: str | None,
) -> str | None:
    if calibration.source_id != active_source_id:
        return "calibration source_id does not match the active frame source"
    if active_link_id is None or calibration.link_id != active_link_id:
        return "calibration link_id does not match the live link identity"
    bias = _finite(calibration.capture_lookup_bias_s)
    if bias is None or abs(bias) > POSE_FRAME_ASSOCIATION_ABSOLUTE_MAX_SKEW_S:
        return "capture_lookup_bias_s outside the physical ceiling"
    link_delay = _finite(calibration.attitude_link_delay_s)
    if (
        link_delay is None
        or link_delay < 0.0
        or link_delay > POSE_FRAME_ASSOCIATION_ABSOLUTE_MAX_SKEW_S
    ):
        return "attitude_link_delay_s outside the physical ceiling"
    for name, bound in (
        ("residual_bound_s", calibration.residual_bound_s),
        ("camera_variation_bound_s", calibration.camera_variation_bound_s),
        ("attitude_variation_bound_s", calibration.attitude_variation_bound_s),
    ):
        value = _finite(bound)
        if value is None or value < 0.0 or value > TIMING_RESIDUAL_BUDGET_S:
            return f"{name} outside the timing budget"
    interval = _finite(calibration.min_frame_interval_s)
    variation = float(calibration.camera_variation_bound_s)
    required = (
        RATE_AMPLIFICATION_LIMIT
        / (RATE_AMPLIFICATION_LIMIT - 1.0)
        * 2.0
        * variation
    )
    if interval is None or interval <= 0.0 or interval < required:
        return (
            "min_frame_interval_s violates the rate-amplification admission"
        )
    return None


def attitude_lookup_wall_s(
    stamp: CaptureStamp,
    calibration: CaptureLookupCalibration,
) -> float:
    """Map a frame stamp onto the ATTITUDE receipt axis (decision doc §2).

    Exhaustive over the stamp kinds ON PURPOSE: an unrecognized kind must
    never silently ride the EXPOSURE branch.
    """
    if stamp.kind is CaptureStampKind.READ:
        return stamp.captured_at_s - calibration.capture_lookup_bias_s
    if stamp.kind is CaptureStampKind.EXPOSURE:
        return stamp.captured_at_s + calibration.attitude_link_delay_s
    raise ValueError(f"unrecognized capture stamp kind: {stamp.kind!r}")


def capture_estimate_s(
    lookup_wall_s: float,
    calibration: CaptureLookupCalibration,
) -> float:
    """Best available exposure-time estimate (geometry/measurement time)."""
    return lookup_wall_s - calibration.attitude_link_delay_s


def quarantine_window_s(
    stamp_kind: CaptureStampKind,
    calibration: CaptureLookupCalibration,
) -> float:
    """Exposure-to-lookup upper bound: seams inside it invalidate the frame.

    The interval from true exposure to lookup is Da0 + dc (§2), so EXPOSURE
    needs Da0 and READ needs Da0 + max positive dc. Nonnegative by
    `validate_calibration`.
    """
    window = calibration.attitude_link_delay_s
    if stamp_kind is CaptureStampKind.READ:
        window += calibration.camera_variation_bound_s
    return window


__all__ = [
    "AttitudeAtLookup",
    "BRACKET_MAX_WIDTH_S",
    "CLOCK_SEAM_TOLERANCE_S",
    "CaptureLookupCalibration",
    "RATE_AMPLIFICATION_LIMIT",
    "STATUS_CLOCK_SEAM",
    "STATUS_GAP",
    "STATUS_UNBRACKETED",
    "STATUS_UNCALIBRATED",
    "TIMING_RESIDUAL_BUDGET_S",
    "attitude_lookup_wall_s",
    "capture_estimate_s",
    "interpolate_attitude_at",
    "quarantine_window_s",
    "validate_calibration",
]
