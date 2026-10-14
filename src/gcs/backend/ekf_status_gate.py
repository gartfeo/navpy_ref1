"""EKF launch gate: the estimator report a launch needs, and its stream.

Launching before ArduPlane's EKF has an absolute solution lets the filter set
its origin in flight; the reported altitude then jumps and the approach is
flown on it (docs/design/ekf_launch_gate.md). The demo pins ARMING_CHECK=0,
so the firmware does not refuse that arm itself: this gate does.

The frontend launch readiness and EKF row mirror these values
(``frontend/src/utils/prearmChecks.js``); a test keeps the two equal.
"""
from __future__ import annotations

import logging
from collections.abc import Mapping

from pymavlink.dialects.v20.ardupilotmega import (
    EKF_ATTITUDE,
    EKF_CONST_POS_MODE,
    EKF_GPS_GLITCHING,
    EKF_POS_HORIZ_ABS,
    EKF_POS_VERT_ABS,
    EKF_UNINITIALIZED,
    EKF_VELOCITY_HORIZ,
    EKF_VELOCITY_VERT,
    MAV_CMD_SET_MESSAGE_INTERVAL,
    MAVLINK_MSG_ID_EKF_STATUS_REPORT,
)

log = logging.getLogger(__name__)

# Rate the GCS asks each autopilot to send EKF_STATUS_REPORT on its link.
EKF_STATUS_RATE_HZ = 2.0
# A report older than four intervals is stale: one lost report on a lossy
# radio must not block, a stopped stream must.
EKF_STATUS_STALE_S = 4.0 / EKF_STATUS_RATE_HZ

# EKF_STATUS_FLAGS a launch needs, with what each one means is missing.
EKF_REQUIRED_BITS: dict[int, str] = {
    EKF_ATTITUDE: "attitude",
    EKF_VELOCITY_HORIZ: "horizontal velocity",
    EKF_VELOCITY_VERT: "vertical velocity",
    EKF_POS_HORIZ_ABS: "absolute horizontal position",
    # Baro-only height also sets this bit; ArduPlane accepts that, so do we.
    EKF_POS_VERT_ABS: "absolute vertical position",
}
# EKF_STATUS_FLAGS that block a launch while reported. A GPS glitch blocks
# only while reported: the next fresh report without it clears it.
EKF_FORBIDDEN_BITS: dict[int, str] = {
    EKF_UNINITIALIZED: "not initialised",
    EKF_CONST_POS_MODE: "constant-position mode (no GPS aiding yet)",
    EKF_GPS_GLITCHING: "GPS glitch",
}


def _mask(bits: Mapping[int, str]) -> int:
    mask = 0
    for bit in bits:
        mask |= bit
    return mask


EKF_REQUIRED_MASK = _mask(EKF_REQUIRED_BITS)    # 0x37
EKF_FORBIDDEN_MASK = _mask(EKF_FORBIDDEN_BITS)  # 0x8480

EKF_ISSUE_NO_REPORT = "waiting for EKF status"
EKF_ISSUE_STALE = "EKF status stale"


def ekf_status_issue(ekf: Mapping[str, float] | None) -> str | None:
    """Why this EKF report blocks a launch, or None when it allows one.

    ``ekf`` is the vehicle's ``ekf_status`` (flags plus ``age_s``); None means
    no report has been received from the vehicle's own autopilot.
    """
    if ekf is None:
        return EKF_ISSUE_NO_REPORT
    age_s = ekf.get("age_s")
    if age_s is None or not age_s <= EKF_STATUS_STALE_S:
        return EKF_ISSUE_STALE
    flags = int(ekf.get("flags", 0))
    reasons = [
        reason for bit, reason in EKF_FORBIDDEN_BITS.items() if flags & bit
    ] + [
        f"no {reason}"
        for bit, reason in EKF_REQUIRED_BITS.items() if not flags & bit
    ]
    if not reasons:
        return None
    return f"EKF not ready ({', '.join(reasons)})"


def request_ekf_status_stream(vehicle: object, sys_id: int) -> None:
    """Ask the vehicle's autopilot for EKF_STATUS_REPORT at the gate's rate.

    SET_MESSAGE_INTERVAL applies per link, so each GCS connection asks on its
    own. Fire-and-forget: a lost request leaves the firmware's default stream,
    and a missing or stale report blocks launch instead of passing it.
    """
    try:
        vehicle.send_command_long(
            MAV_CMD_SET_MESSAGE_INTERVAL,
            p1=MAVLINK_MSG_ID_EKF_STATUS_REPORT,
            p2=int(round(1_000_000.0 / EKF_STATUS_RATE_HZ)),
        )
    except Exception:
        log.exception("V%d: EKF_STATUS_REPORT interval request failed", sys_id)
