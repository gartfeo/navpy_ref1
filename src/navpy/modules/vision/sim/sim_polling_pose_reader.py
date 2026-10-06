"""Narrow polling-pose adapter for finite-FOV simulation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol, Tuple

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.sim.sim_runtime_ports import (
    AttitudeReader,
    AttitudeSampleReader,
    LocationReader,
    OptionalFloatReader,
)


@dataclass(frozen=True)
class PollingPoseSample:
    location: Location
    attitude: Attitude
    time_boot_s: Optional[float]
    receipt_time_s: Optional[float]
    body_rates_rad_s: Optional[Tuple[float, float, float]]
    air_speed_mps: Optional[float]


class PollingPoseReaderPort(Protocol):
    def read(self) -> PollingPoseSample: ...


class SimPollingPoseReader:
    """Read one polling sample through exact vehicle-field readers."""

    def __init__(
        self,
        *,
        location: LocationReader,
        attitude: AttitudeReader,
        attitude_sample: AttitudeSampleReader,
        air_speed: OptionalFloatReader,
    ) -> None:
        self._location = location
        self._attitude = attitude
        self._attitude_sample = attitude_sample
        self._air_speed = air_speed

    def read(self) -> PollingPoseSample:
        sample = self._attitude_sample()
        if sample is None:
            attitude = self._attitude()
            boot_s = receipt_s = body_rates = None
        else:
            attitude = sample.attitude
            boot_s = sample.time_boot_s
            receipt_s = sample.receipt_time_s
            body_rates = sample.body_rates_rad_s
        return PollingPoseSample(
            self._location(),
            attitude,
            boot_s,
            receipt_s,
            body_rates,
            self._air_speed(),
        )


__all__ = ["PollingPoseReaderPort", "PollingPoseSample", "SimPollingPoseReader"]
