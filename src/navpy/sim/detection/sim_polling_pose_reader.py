"""Narrow polling-pose adapter for finite-FOV simulation.

A frame is rendered only from the latest ATTITUDE and the latest
GLOBAL_POSITION_INT when both describe the same autopilot instant: equal
integer ``time_boot_ms`` and receipts within one association budget.
Position feeds only the simulator's rendering, never the vision law.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional, Protocol, Tuple

from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.location import Location
from navpy.sim.detection.pose_associator import (
    copy_attitude,
    receipts_associated,
)
from navpy.sim.detection.sim_runtime_ports import (
    AttitudeSampleReader,
    AttitudeSampleView,
    OptionalFloatReader,
    PositionSampleReader,
    PositionSampleView,
)


@dataclass(frozen=True)
class PollingPoseSample:
    location: Location
    attitude: Attitude
    time_boot_s: float
    receipt_time_s: float
    body_rates_rad_s: Optional[Tuple[float, float, float]]
    air_speed_mps: Optional[float]


@dataclass(frozen=True)
class PollingPoseRead:
    """One poll: the latest attitude boot time and the exact pair, if any."""

    attitude_time_boot_s: Optional[float]
    sample: Optional[PollingPoseSample]


class PollingPoseReaderPort(Protocol):
    def read(self) -> PollingPoseRead: ...


def whole_boot_ms(time_boot_s: object) -> Optional[int]:
    """Integer ``time_boot_ms`` behind a ``float(ms) * 1e-3`` boot time.

    Both telemetry views produce ``time_boot_s`` with that conversion, so the
    inverse is exact; any other value is not a whole autopilot millisecond.
    """
    if isinstance(time_boot_s, bool):
        return None
    try:
        value_s = float(time_boot_s)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value_s):
        return None
    milliseconds = round(value_s * 1000.0)
    return milliseconds if float(milliseconds) * 1e-3 == value_s else None


def _finite_receipt_s(value: object) -> Optional[float]:
    if isinstance(value, bool):
        return None
    try:
        receipt_s = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    return receipt_s if math.isfinite(receipt_s) else None


def _location(position: PositionSampleView) -> Optional[Location]:
    try:
        values = (float(position.lat), float(position.lon), float(position.alt))
    except (AttributeError, TypeError, ValueError):
        return None
    if not all(map(math.isfinite, values)):
        return None
    return Location(*values, is_absolute=True)


def _finite_attitude(
    attitude: AttitudeSampleView,
) -> Optional[Tuple[Attitude, Optional[Tuple[float, float, float]]]]:
    """A copy of the attitude and body rates, or None when any is malformed."""
    try:
        orientation = copy_attitude(attitude.attitude)
        rates = attitude.body_rates_rad_s
        body_rates = (
            None if rates is None else tuple(float(rate) for rate in rates)
        )
    except (AttributeError, TypeError, ValueError):
        return None
    values = (orientation.pitch, orientation.yaw, orientation.roll, *(body_rates or ()))
    if not all(map(math.isfinite, values)):
        return None
    if body_rates is not None and len(body_rates) != 3:
        return None
    return orientation, body_rates


class SimPollingPoseReader:
    """Read one exact attitude/position pair through vehicle-field readers."""

    def __init__(
        self,
        *,
        attitude_sample: AttitudeSampleReader,
        position_sample: PositionSampleReader,
        air_speed: OptionalFloatReader,
        maximum_receipt_skew_s: Callable[[], float],
    ) -> None:
        self._attitude_sample = attitude_sample
        self._position_sample = position_sample
        self._air_speed = air_speed
        self._maximum_receipt_skew_s = maximum_receipt_skew_s

    def read(self) -> PollingPoseRead:
        attitude = self._attitude_sample()
        position = self._position_sample()
        attitude_ms = (
            whole_boot_ms(attitude.time_boot_s) if attitude is not None else None
        )
        if attitude is None or attitude_ms is None:
            return PollingPoseRead(None, None)
        attitude_boot_s = float(attitude_ms) * 1e-3
        if position is None:
            return PollingPoseRead(attitude_boot_s, None)
        return PollingPoseRead(
            attitude_boot_s,
            self._pair(attitude, attitude_ms, attitude_boot_s, position),
        )

    def _pair(
        self,
        attitude: AttitudeSampleView,
        attitude_ms: int,
        attitude_boot_s: float,
        position: PositionSampleView,
    ) -> Optional[PollingPoseSample]:
        if whole_boot_ms(position.time_boot_s) != attitude_ms:
            return None
        attitude_receipt_s = _finite_receipt_s(attitude.receipt_time_s)
        position_receipt_s = _finite_receipt_s(position.receipt_time_s)
        if attitude_receipt_s is None or position_receipt_s is None:
            return None
        if not receipts_associated(
            attitude_receipt_s,
            position_receipt_s,
            self._maximum_receipt_skew_s,
        ):
            return None
        location = _location(position)
        orientation = _finite_attitude(attitude)
        if location is None or orientation is None:
            return None
        rendered_attitude, body_rates = orientation
        return PollingPoseSample(
            location,
            rendered_attitude,
            attitude_boot_s,
            # The older constituent's age governs receipt freshness.
            min(attitude_receipt_s, position_receipt_s),
            body_rates,
            self._air_speed(),
        )


__all__ = [
    "PollingPoseRead",
    "PollingPoseReaderPort",
    "PollingPoseSample",
    "SimPollingPoseReader",
    "whole_boot_ms",
]
