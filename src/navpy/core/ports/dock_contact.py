"""Dock contact: the one success signal of a final approach.

In simulation a contact sensor stands in for the net (sim/world). A real
aircraft is caught by the net, so a real build wires no sensor. The event is
lifecycle evidence only: its distances are written to the DOCKED evidence row
and never reach frames, the navigation law or the command path.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional, Protocol


@dataclass(frozen=True)
class DockContactIdentity:
    """One armed approach: this vehicle, its task, its own dock, one episode."""

    vehicle: int
    task_id: int
    dock_uid: int
    episode: int


@dataclass(frozen=True)
class DockContactEvent:
    identity: DockContactIdentity
    aircraft_time_s: float
    closest_3d_m: float
    closest_h_m: float
    closest_v_m: float


DockContactCallback = Callable[[DockContactEvent], None]


@dataclass(frozen=True)
class DockEpisodeEnd:
    """How NAV left the armed episode, as the lifecycle saw it.

    Display only (the sim closest-approach record); never a command input.
    ``abort_reason`` names an episode that NAV ended early: a NAV_FAIL's
    reason or ``cancelled``. All unset: the reason is unknown.
    """

    docked: bool = False
    missed: bool = False
    abort_reason: Optional[str] = None


class DockContactSensor(Protocol):
    def arm(
        self,
        identity: DockContactIdentity,
        on_docked: DockContactCallback,
    ) -> None: ...

    def disarm(self, end: DockEpisodeEnd = DockEpisodeEnd()) -> None:
        """End the episode; ``end`` is the lifecycle's accepted outcome."""
        ...


__all__ = [
    "DockContactCallback",
    "DockContactEvent",
    "DockContactIdentity",
    "DockContactSensor",
    "DockEpisodeEnd",
]
