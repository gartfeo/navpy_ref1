"""Exact aircraft-attitude input used by simulated gimbals."""

from __future__ import annotations

from typing import Protocol

from navpy.modules.common.models.attitude import Attitude


class AircraftAttitudeReader(Protocol):
    def read(self) -> Attitude | None: ...


class AircraftAttitudeSource(Protocol):
    @property
    def attitude(self) -> Attitude | None: ...


class VehicleAttitudeReader:
    """One-field adapter that exposes attitude without the vehicle host."""

    def __init__(self, source: AircraftAttitudeSource) -> None:
        self._source = source

    def read(self) -> Attitude | None:
        return self._source.attitude


__all__ = [
    "AircraftAttitudeReader",
    "AircraftAttitudeSource",
    "VehicleAttitudeReader",
]
