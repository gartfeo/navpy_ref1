"""Request a high-rate pose stream (ATTITUDE + position) from the autopilot.

Final approach treats a detection as a body-fixed visual direction only when
it carries a frame-atomic pose. The staleness of that pose is bounded by the
ATTITUDE stream period, so simulated pose delivery follows the autopilot's own
scheduler-derived MAVLink limit instead of a detector or wall-clock setting.

``SET_MESSAGE_INTERVAL`` is applied by ArduPilot PER-LINK, on the channel the
command arrives on, so this raises the rate only on our own (companion) MAVLink
link -- it does NOT touch the GCS telemetry-radio stream.
"""
from __future__ import annotations

import math
from typing import Protocol, runtime_checkable

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_SET_MESSAGE_INTERVAL,
    MAVLINK_MSG_ID_ATTITUDE,
    MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    MAVLINK_MSG_ID_SIM_STATE,
)

from navpy.core.ports.pose_stream_rate import (
    ARDUPILOT_MESSAGE_RATE_SCHEDULER_FRACTION,
    POSE_STREAM_RATE_HZ,
    SCHED_LOOP_RATE_FALLBACK_HZ,
)


@runtime_checkable
class SchedulerParameterReader(Protocol):
    def get_param_or_default(self, name: str, default: float) -> object: ...


@runtime_checkable
class CommandLongSender(Protocol):
    def send_command_long(
        self,
        command: int,
        **parameters: object,
    ) -> object: ...


class PoseStreamLogger(Protocol):
    def warning(self, message: str) -> None: ...


POSE_STREAM_MESSAGE_IDS = (
    MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    MAVLINK_MSG_ID_ATTITUDE,
)


def pose_stream_rate_for_scheduler_rate_hz(scheduler_rate_hz: float) -> float:
    """The achievable pose rate for an ALREADY-resolved scheduler rate.

    Split from the resolver so a caller that needs both rates reads the
    parameter once. Two reads can answer differently, and callers derive
    grids from them -- the association skew window from the pose rate, a
    scheduler-slot grid from the scheduler rate -- which must agree.
    """
    return scheduler_rate_hz * ARDUPILOT_MESSAGE_RATE_SCHEDULER_FRACTION


def resolve_pose_stream_rate_hz(vehicle: object) -> float:
    """Return ArduPilot's scheduler-derived achievable MAVLink pose rate."""
    return pose_stream_rate_for_scheduler_rate_hz(
        resolve_ardupilot_scheduler_rate_hz(vehicle)
    )


def resolve_ardupilot_scheduler_rate_hz(vehicle: object) -> float:
    """Return the validated ArduPilot main scheduler rate."""
    if not isinstance(vehicle, SchedulerParameterReader):
        return SCHED_LOOP_RATE_FALLBACK_HZ
    try:
        value = vehicle.get_param_or_default(
            "SCHED_LOOP_RATE",
            SCHED_LOOP_RATE_FALLBACK_HZ,
        )
        scheduler_rate_hz = float(value)
    except (TypeError, ValueError):
        return SCHED_LOOP_RATE_FALLBACK_HZ
    if (
        isinstance(value, bool)
        or not math.isfinite(scheduler_rate_hz)
        or scheduler_rate_hz <= 0.0
    ):
        return SCHED_LOOP_RATE_FALLBACK_HZ
    return scheduler_rate_hz


def request_pose_streams(
        vehicle: object,
        logger: PoseStreamLogger | None = None,
        rate_hz: float = POSE_STREAM_RATE_HZ,
) -> None:
    """Ask the autopilot for ATTITUDE + position at ``rate_hz`` on our link.

    No-op when the vehicle cannot send a command_long (e.g. a non-MAVLink
    vehicle or a test double); each message request is best-effort.
    """
    if not isinstance(vehicle, CommandLongSender):
        return
    if not (isinstance(rate_hz, (int, float)) and math.isfinite(rate_hz) and rate_hz > 0.0):
        if logger is not None:
            logger.warning(
                f"request_pose_streams: ignoring non-positive/non-finite rate_hz={rate_hz!r}"
            )
        return
    interval_us = int(round(1_000_000.0 / float(rate_hz)))
    for message_id in POSE_STREAM_MESSAGE_IDS:
        try:
            vehicle.send_command_long(
                MAV_CMD_SET_MESSAGE_INTERVAL,
                p1=message_id,
                p2=interval_us,
            )
        except Exception as exc:  # pragma: no cover - best-effort telemetry
            if logger is not None:
                logger.warning(
                    f"request_pose_streams: failed for message {message_id}: {exc}"
                )

def request_simulator_truth_pose_stream(
        vehicle: object,
        logger: PoseStreamLogger | None = None,
        rate_hz: float = POSE_STREAM_RATE_HZ,
) -> None:
    """Request SIM_STATE at the pose rate for the ideal simulator renderer.

    SIM_STATE has no source timestamp, so ATTITUDE still supplies cadence and
    boot-clock time. This request only keeps the cached truth pose fresh enough
    to pair with each advancing ATTITUDE sample.
    """
    if not isinstance(vehicle, CommandLongSender):
        return
    if not (
            isinstance(rate_hz, (int, float))
            and math.isfinite(rate_hz)
            and rate_hz > 0.0
    ):
        if logger is not None:
            logger.warning(
                "request_simulator_truth_pose_stream: ignoring "
                f"non-positive/non-finite rate_hz={rate_hz!r}"
            )
        return
    interval_us = int(round(1_000_000.0 / float(rate_hz)))
    try:
        vehicle.send_command_long(
            MAV_CMD_SET_MESSAGE_INTERVAL,
            p1=MAVLINK_MSG_ID_SIM_STATE,
            p2=interval_us,
        )
    except Exception as exc:  # pragma: no cover - best-effort telemetry
        if logger is not None:
            logger.warning(
                f"request_simulator_truth_pose_stream: failed for SIM_STATE: {exc}"
            )
