"""Hold the ground until the EKF owns the NED origin it will fly against."""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Any

from pymavlink import mavutil

# Siblings are imported as top-level modules, which only resolves when this
# directory is on the path.  Same guard as eval_navigation_telemetry: without it
# the module and its test fail standalone with ModuleNotFoundError.
_SCRIPTS = str(Path(__file__).resolve().parent)
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from eval_navigation_telemetry import (  # noqa: E402
    MESSAGE_INTERVAL_ACK_TIMEOUT_S,
    RECV_SLICE_S,
    recv_target_message,
    request_message_interval_stream,
)


# EKF_STATUS_REPORT bits that together mean the filter has an absolute
# position solution.  For a GPS-aided filter -- the eval default; an explicit
# --sitl-param EK3_SRC1_POSXY override voids this -- it doubles as an origin-established
# signal: horiz_pos_abs needs AID_ABSOLUTE, and the GPS route into
# AID_ABSOLUTE requires validOrigin (AP_NavEKF3_Control.cpp:591; EKF2 agrees,
# AP_NavEKF2_Control.cpp:391).  The external-nav and beacon routes do NOT
# carry that requirement (AP_NavEKF3_Control.cpp:601-620), so these flags are
# not an origin proof on a vehicle aided by those sources.
NAV_SOLUTION_FLAGS = (
    mavutil.mavlink.EKF_ATTITUDE
    | mavutil.mavlink.EKF_POS_HORIZ_ABS
    | mavutil.mavlink.EKF_POS_VERT_ABS
)
NAV_SOLUTION_RATE_HZ = 5.0
# Longest single wait for one report, so the loop notices an expired
# budget promptly instead of after a full report interval.
NAV_SOLUTION_POLL_S = 1.0
# pymavlink checks its own timeout before reading and not after, so a receive
# that started inside the budget can hand back a report once the budget has
# passed.  Accept that one overrun -- the vehicle did answer, and failing a
# flight over it trades a timing imprecision for an aborted run -- but bound
# it to the receive it can span so the advertised timeout stays true.
NAV_SOLUTION_GRACE_S = RECV_SLICE_S
NAV_SOLUTION_TIMEOUT_S = 120.0


def nav_solution_ready(flags: int) -> bool:
    """Return whether EKF_STATUS_REPORT flags show an absolute solution."""
    return (
        flags & NAV_SOLUTION_FLAGS == NAV_SOLUTION_FLAGS
        and not flags & mavutil.mavlink.EKF_UNINITIALIZED
    )


def wait_for_nav_solution(
    master: Any,
    *,
    timeout_s: float | None = None,
) -> bool:
    """Block until the selected vehicle's EKF publishes an absolute solution.

    Arming before this point lets the EKF take its NED origin while the
    aircraft is already climbing, and an origin cannot be moved once set
    (``NavEKF3_core::setOrigin`` rejects a second one).  Measured on the
    2026-09-05 fleet run: the origin landed at +1.830 m over a +0.100 m
    field, putting a fixed +1.66 m bias on every altitude the vehicle
    reported for the rest of the flight and costing 1.05 m of scored CPA
    against 0.017 m of simulator truth.  ArduPilot's own pre-arm check would
    refuse to arm here, but eval runs pin ARMING_CHECK to 0, so the harness
    has to hold the ground itself.
    """
    if timeout_s is None:
        timeout_s = NAV_SOLUTION_TIMEOUT_S
    # Every wait below is clamped to what is left of this deadline, the ACK
    # exchange included.  That bounds when a wait may START, not the runtime:
    # a blocking receive already under way can return after the deadline, and
    # NAV_SOLUTION_GRACE_S decides whether such a report still counts.
    deadline = time.monotonic() + timeout_s
    # Best-effort acceleration only: ArduPlane already streams
    # EKF_STATUS_REPORT in the EXTRA3 group (1 Hz default), so a rejected or
    # lost interval request must not fail the wait -- keep listening for the
    # reports that arrive anyway.
    request_message_interval_stream(
        master,
        mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
        NAV_SOLUTION_RATE_HZ,
        timeout_s=min(
            MESSAGE_INTERVAL_ACK_TIMEOUT_S,
            max(0.0, deadline - time.monotonic()),
        ),
    )
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False
        message = recv_target_message(
            master, "EKF_STATUS_REPORT", min(NAV_SOLUTION_POLL_S, remaining)
        )
        if message is not None and nav_solution_ready(
            int(getattr(message, "flags", 0))
        ):
            return time.monotonic() <= deadline + NAV_SOLUTION_GRACE_S


def require_nav_solution(
    master: Any,
    sys_id: object = None,
    *,
    timeout_s: float | None = None,
) -> None:
    """Raise unless the selected vehicle is safe to arm."""
    if not wait_for_nav_solution(master, timeout_s=timeout_s):
        who = "vehicle" if sys_id is None else f"vehicle {sys_id}"
        raise RuntimeError(
            f"{who} did not report an absolute EKF solution; refusing to arm "
            "(an origin captured in flight is permanent)"
        )


def require_fleet_nav_solution(
    master: Any,
    sys_ids: Any,
    *,
    select: Any,
    timeout_s: float | None = None,
) -> None:
    """Clear every vehicle's origin before any of them is allowed to arm.

    Checking inside the arming loop instead would let the first vehicle fly
    while the last was still initialising, which is the stagger the
    co-located launch geometry exists to avoid.
    """
    for sys_id in sys_ids:
        select(master, sys_id)
        require_nav_solution(master, sys_id, timeout_s=timeout_s)
