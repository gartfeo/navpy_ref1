"""Simulation-only target control and fleet classification."""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol, Sequence

if TYPE_CHECKING:
    from navpy.modules.common.models.location import Location


class SimulationMember(Protocol):
    @property
    def is_simulation(self) -> bool: ...

    def set_sim_target(
        self,
        command_index: int,
        location: "Location",
        location_type: str | None = None,
    ) -> None: ...


class SimulationControlFleet:
    def __init__(self, members: Sequence[SimulationMember]) -> None:
        self._members = tuple(members)

    @property
    def is_simulation(self) -> bool:
        return bool(self._members) and all(
            member.is_simulation for member in self._members
        )

    def set_sim_target(
        self,
        command_index: int,
        location: "Location",
        location_type: str | None = None,
    ) -> None:
        for member in self._members:
            member.set_sim_target(command_index, location, location_type=location_type)


__all__ = ["SimulationControlFleet"]
