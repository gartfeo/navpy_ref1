"""Pose stream rate and the frame/pose association skew bound it implies."""

from __future__ import annotations

import math


# ArduPilot caps MAVLink messages at 0.8 * SCHED_LOOP_RATE. ArduPilot 4.6
# rejects a faster SET_MESSAGE_INTERVAL request instead of clamping it, and
# SIM_STATE has no default Plane stream, so request the firmware-achievable
# rate directly. These are firmware constraints, not operator settings.
SCHED_LOOP_RATE_FALLBACK_HZ = 50.0
ARDUPILOT_MESSAGE_RATE_SCHEDULER_FRACTION = 0.8
POSE_STREAM_RATE_HZ = (
    SCHED_LOOP_RATE_FALLBACK_HZ
    * ARDUPILOT_MESSAGE_RATE_SCHEDULER_FRACTION
)
# A frame may arrive anywhere within one ATTITUDE period; allow one additional
# period for link/scheduler jitter, then reject the association as mixed-time.
POSE_FRAME_ASSOCIATION_MAX_PERIODS = 2.0
# Even a malformed/unusually low scheduler rate must not bless a pose that is
# more than half a second away from its camera frame as frame-atomic.
POSE_FRAME_ASSOCIATION_ABSOLUTE_MAX_SKEW_S = 0.5


def pose_frame_association_max_skew_s(rate_hz: object) -> float:
    """Return a validated two-scheduler-period frame/pose skew bound."""
    if isinstance(rate_hz, bool):
        rate_hz = POSE_STREAM_RATE_HZ
    try:
        rate_hz = float(rate_hz)
    except (TypeError, ValueError):
        rate_hz = POSE_STREAM_RATE_HZ
    if not math.isfinite(rate_hz) or rate_hz <= 0.0:
        rate_hz = POSE_STREAM_RATE_HZ
    return min(
        POSE_FRAME_ASSOCIATION_MAX_PERIODS / rate_hz,
        POSE_FRAME_ASSOCIATION_ABSOLUTE_MAX_SKEW_S,
    )


POSE_FRAME_ASSOCIATION_MAX_SKEW_S = pose_frame_association_max_skew_s(
    POSE_STREAM_RATE_HZ,
)
