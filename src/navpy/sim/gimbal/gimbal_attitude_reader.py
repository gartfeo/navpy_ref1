"""Exact aircraft-attitude input used by simulated gimbals."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from navpy.core.kernel.attitude import Attitude


class AircraftAttitudeReader(Protocol):
    def read(self) -> Attitude | None: ...


@runtime_checkable
class AircraftClockReader(Protocol):
    """Optional: the latest attitude with its aircraft boot time (ms).

    Both come from one telemetry sample, so they always belong together.
    """

    def read_timed(self) -> tuple[Attitude, int] | None: ...


class AircraftAttitudeSource(Protocol):
    @property
    def attitude(self) -> Attitude | None: ...


class VehicleAttitudeReader:
    """One-field adapter that exposes attitude without the vehicle host."""

    def __init__(self, source: AircraftAttitudeSource) -> None:
        self._source = source

    def read(self) -> Attitude | None:
        return self._source.attitude

    def read_timed(self) -> tuple[Attitude, int] | None:
        sample = getattr(self._source, "attitude_sample", None)
        attitude = getattr(sample, "attitude", None)
        boot_s = getattr(sample, "time_boot_s", None)
        if (
            not isinstance(attitude, Attitude)
            or not isinstance(boot_s, (int, float))
            or isinstance(boot_s, bool)
        ):
            return None
        return attitude, int(round(float(boot_s) * 1000.0))


__all__ = [
    "AircraftAttitudeReader",
    "AircraftClockReader",
    "AircraftAttitudeSource",
    "VehicleAttitudeReader",
]
