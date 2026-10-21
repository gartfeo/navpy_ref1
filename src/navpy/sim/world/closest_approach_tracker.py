"""The episode's closest pass to the own dock, for the display record.

Sim only (gcs_sim_closest_approach.md). The contact sensor feeds every joined
segment here under its own lock; nothing here reaches the lifecycle, frames,
the law or the command path.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional, Protocol

from navpy.core.approach.logs.navigation_snap_geometry import (
    calc_distance,
    closest_point_components_on_segment,
)
from navpy.core.approach.logs.navigation_snap_types import ClosestPointComponents
from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.location import Location
from navpy.core.ports.dock_contact import DockContactIdentity
from navpy.sim.world.closest_approach_record import (
    OUTCOME_DOCKED,
    OUTCOME_MISSED,
    ApproachPoint,
    AttitudeDeg,
    ClosestApproach,
    MissDistance,
)
from navpy.sim.world.simulation_object import SimulationObject

ClosestApproachSink = Callable[[ClosestApproach], None]


class TrackedSample(Protocol):
    """One telemetry position sample, as the contact sensor joins them."""

    @property
    def aircraft_time_s(self) -> float: ...

    @property
    def location(self) -> Location: ...

    @property
    def relative_alt_m(self) -> Optional[float]: ...

    @property
    def attitude(self) -> Optional[Attitude]: ...


@dataclass(frozen=True)
class ClosestPass:
    """One segment's exact closest point to the dock."""

    components: ClosestPointComponents
    start: TrackedSample
    end: TrackedSample
    dock: SimulationObject


def exact_pass(start: TrackedSample, end: TrackedSample, dock: SimulationObject) -> ClosestPass:
    components = closest_point_components_on_segment(start.location, end.location, dock.g_loc)
    return ClosestPass(components, start, end, dock)


class ClosestApproachTracker:
    """The episode's exact minimum and its contact pass; the caller locks."""

    def __init__(self) -> None:
        self._best: Optional[ClosestPass] = None
        self._contact: Optional[ClosestPass] = None

    def take(self) -> tuple[Optional[ClosestPass], Optional[ClosestPass]]:
        """The episode's (best, contact) passes; starts a new episode."""
        passes = self._best, self._contact
        self._best = self._contact = None
        return passes

    def keep(
        self,
        start: TrackedSample,
        end: TrackedSample,
        dock: SimulationObject,
        flat_m: float,
        margin_m: float,
    ) -> Optional[ClosestPass]:
        """Keep the episode's exact minimum; return the exact pass if computed.

        The exact geometry runs only for a segment whose flat distance, less
        the flat-earth bound, could beat the kept exact distance, so no
        segment that could be closer is skipped.
        """
        best = self._best
        if best is not None and not flat_m - margin_m < best.components.slant:
            return None
        segment = exact_pass(start, end, dock)
        if best is None or segment.components.slant < best.components.slant:
            self._best = segment
        return segment

    def contact(self, segment: ClosestPass) -> None:
        self._contact = segment


def closest_approach(
    identity: DockContactIdentity,
    docked: bool,
    best: Optional[ClosestPass],
    contact: Optional[ClosestPass],
) -> Optional[ClosestApproach]:
    """The record: the contact pass if the lifecycle accepted DOCKED, else
    the episode minimum; None without a joined segment."""
    closest = contact if docked and contact is not None else best
    if closest is None:
        return None
    point, dock = closest.components.c_star, closest.dock.g_loc
    start, end = closest.start, closest.end
    # Heights over home: AMSL less the start sample's home AMSL height.
    home_amsl_m = None if start.relative_alt_m is None else start.location.alt - start.relative_alt_m
    nearer = (
        start
        if calc_distance(point, start.location) <= calc_distance(point, end.location)
        else end
    )
    attitude = nearer.attitude
    return ClosestApproach(
        vehicle=identity.vehicle,
        task_id=identity.task_id,
        dock_uid=identity.dock_uid,
        episode=identity.episode,
        outcome=OUTCOME_DOCKED if docked else OUTCOME_MISSED,
        aircraft_time_s=nearer.aircraft_time_s,
        point=_approach_point(point, home_amsl_m),
        dock=_approach_point(dock, home_amsl_m),
        distance=MissDistance(
            closest.components.slant, closest.components.h_dist, closest.components.v_dist,
        ),
        attitude=None if attitude is None else AttitudeDeg(attitude.roll, attitude.pitch, attitude.yaw),
    )


def _approach_point(location: Location, home_amsl_m: Optional[float]) -> ApproachPoint:
    return ApproachPoint(
        location.lat,
        location.lng,
        location.alt,
        alt_rel_m=None if home_amsl_m is None else location.alt - home_amsl_m,
    )


__all__ = [
    "ClosestApproachSink",
    "ClosestApproachTracker",
    "ClosestPass",
    "TrackedSample",
    "closest_approach",
    "exact_pass",
]
