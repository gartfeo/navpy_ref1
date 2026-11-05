"""One cached ATTITUDE sample as the association's pose fallback."""

from __future__ import annotations

import math

from navpy.core.kernel.attitude import Attitude
from navpy.core.perception.pipeline.real_detector_ports import AttitudeSampleView


def frame_pose_from_sample(
    attitude_sample: AttitudeSampleView | None,
) -> tuple[
    Attitude,
    tuple[float, float, float] | None,
    float | None,
    float | None,
] | None:
    """Extract one cached ATTITUDE value and its capture-clock receipt."""
    if attitude_sample is None:
        return None
    uas_att = attitude_sample.attitude
    rates = attitude_sample.body_rates_rad_s
    body_rates = None
    if rates is not None:
        try:
            body_rates = (float(rates[0]), float(rates[1]), float(rates[2]))
        except (IndexError, TypeError, ValueError):
            body_rates = None
    pose_time_s = attitude_sample.time_boot_s
    try:
        pose_time_s = None if pose_time_s is None else float(pose_time_s)
    except (TypeError, ValueError):
        pose_time_s = None
    receipt_time_s = getattr(attitude_sample, "capture_receipt_s", None)
    try:
        receipt_time_s = None if receipt_time_s is None else float(receipt_time_s)
    except (TypeError, ValueError):
        receipt_time_s = None
    if receipt_time_s is not None and not math.isfinite(receipt_time_s):
        receipt_time_s = None
    return uas_att, body_rates, pose_time_s, receipt_time_s


__all__ = ["frame_pose_from_sample"]
