"""The episode's closest pass to the own dock, for the display record.

Sim only (gcs_sim_closest_approach.md). The contact sensor feeds every joined
segment here under its own lock, on the telemetry reader thread, so a segment
costs only flat-frame bounds here; the exact geometry waits for the record
(``EpisodeEvidence.best``). Nothing here reaches the lifecycle, frames, the
law or the command path.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional, Protocol

from navpy.core.approach.logs.navigation_snap_geometry import (
    calc_distance,
    closest_point_components_on_segment,
)
from navpy.core.approach.logs.navigation_snap_types import ClosestPointComponents
from navpy.core.approach.vision_nav.visual_pass import PASS_ARM_MIN_BODY_X
from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.location import Location
from navpy.core.ports.dock_contact import DockContactIdentity, DockEpisodeEnd
from navpy.sim.world.closest_approach_record import (
    OUTCOME_ABORTED,
    OUTCOME_DOCKED,
    OUTCOME_MISSED,
    ApproachPoint,
    AttitudeDeg,
    ClosestApproach,
    MissDistance,
    valid_reason,
)
from navpy.sim.world.simulation_object import SimulationObject

ClosestApproachSink = Callable[[ClosestApproach], None]
# Dock-local metres (east, north, up), the dock at the origin.
LocalPoint = tuple[float, float, float]

# Metres: a shorter segment has no direction, and a range rate per metre of
# travel at or below it is not opening (a repeated or closest-point sample).
_STILL_CLOSING_TOLERANCE_M = 1e-6

# Segments kept unresolved as candidates for the exact minimum. Only segments
# within the flat-earth bounds of the nearest one stay candidates: one to three
# on a pass. A path that holds its range to the dock within those bounds (an
# orbit) keeps adding them; past this many the oldest is resolved at once, so
# the reader thread never does more than one exact computation per sample,
# and the record resolves at most this many.
CLOSEST_APPROACH_MAX_CANDIDATES = 16


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


@dataclass(frozen=True)
class _Candidate:
    """A joined segment whose exact distance lies in [low_m, high_m]: its
    flat distance less and plus the flat-earth bound."""

    start: TrackedSample
    end: TrackedSample
    dock: SimulationObject
    low_m: float
    high_m: float


def exact_pass(start: TrackedSample, end: TrackedSample, dock: SimulationObject) -> ClosestPass:
    components = closest_point_components_on_segment(start.location, end.location, dock.g_loc)
    return ClosestPass(components, start, end, dock)


def _closer(kept: Optional[ClosestPass], later: ClosestPass) -> ClosestPass:
    """The nearer pass; the earlier one on a tie."""
    if kept is None or later.components.slant < kept.components.slant:
        return later
    return kept


@dataclass(frozen=True)
class EpisodeEvidence:
    """What the sensor kept of one episode, up to its cut."""

    # The minimum of the candidates already resolved; each is older than
    # every one still pending.
    resolved: Optional[ClosestPass] = None
    # Segments that may still hold the exact minimum, oldest first.
    candidates: tuple[_Candidate, ...] = ()
    contact: Optional[ClosestPass] = None
    # The UAV had the dock clearly ahead, then the range opened.
    passed: bool = False

    def best(self) -> Optional[ClosestPass]:
        """The episode's exact minimum, the earliest on a tie. Runs the exact
        geometry of the pending candidates: never on the reader thread."""
        best = self.resolved
        for candidate in self.candidates:
            best = _closer(best, exact_pass(candidate.start, candidate.end, candidate.dock))
        return best


class ClosestApproachTracker:
    """The episode's exact-minimum candidates, its contact pass and its pass
    evidence; the caller locks."""

    def __init__(self) -> None:
        self._resolved: Optional[ClosestPass] = None
        self._candidates: list[_Candidate] = []
        self._contact: Optional[ClosestPass] = None
        self._ahead = False
        self._passed = False
        self._cut = False

    def take(self) -> EpisodeEvidence:
        """The episode's evidence; starts a new one."""
        evidence = EpisodeEvidence(
            self._resolved, tuple(self._candidates), self._contact, self._passed,
        )
        self._resolved = self._contact = None
        self._candidates = []
        self._ahead = self._passed = self._cut = False
        return evidence

    def cut(self) -> None:
        """NAV ended the approach: later segments change no evidence."""
        self._cut = True

    def keep(
        self,
        start: TrackedSample,
        end: TrackedSample,
        dock: SimulationObject,
        flat_m: float,
        margin_m: float,
    ) -> None:
        """Keep every segment that could hold the episode's exact minimum.

        A segment whose lower bound is above a kept segment's upper bound
        (or the resolved minimum) is never the minimum, so it is dropped
        without exact geometry; no segment that could be closer is skipped.
        Nothing after the cut.
        """
        if self._cut:
            return
        segment = _Candidate(start, end, dock, flat_m - margin_m, flat_m + margin_m)
        bound_m = min(self._bound_m(), segment.high_m)
        if segment.low_m > bound_m:
            return
        candidates = [kept for kept in self._candidates if not kept.low_m > bound_m]
        candidates.append(segment)
        if len(candidates) > CLOSEST_APPROACH_MAX_CANDIDATES:
            oldest = candidates.pop(0)
            self._resolved = _closer(
                self._resolved, exact_pass(oldest.start, oldest.end, oldest.dock),
            )
        self._candidates = candidates

    def _bound_m(self) -> float:
        """The smallest upper bound on the exact minimum kept so far."""
        resolved = self._resolved
        bound_m = math.inf if resolved is None else resolved.components.slant
        return min([bound_m, *(kept.high_m for kept in self._candidates)])

    def observe(self, start: LocalPoint, end: LocalPoint) -> None:
        """Pass evidence of one joined segment, in the dock-local frame:
        the dock clearly ahead, then the range opening at a segment's end."""
        if self._cut:
            return
        direction = tuple(b - a for a, b in zip(start, end))
        length = math.hypot(*direction)
        if length <= _STILL_CLOSING_TOLERANCE_M:
            return
        if not self._ahead:
            # Within the vision pass gate's cone of the line to the dock.
            towards_dock = -sum(d * s for d, s in zip(direction, start))
            cone = PASS_ARM_MIN_BODY_X * length * math.hypot(*start)
            self._ahead = cone > 0.0 and towards_dock >= cone
        opening = sum(d * e for d, e in zip(direction, end)) / length
        if self._ahead and opening > _STILL_CLOSING_TOLERANCE_M:
            self._passed = True

    def break_run(self) -> None:
        """The sensor did not join: a pending *ahead* is not carried over."""
        if not self._cut:
            self._ahead = False

    def contact(self, segment: ClosestPass) -> None:
        self._contact = segment


def closest_approach(
    identity: DockContactIdentity,
    end: DockEpisodeEnd,
    evidence: EpisodeEvidence,
) -> Optional[ClosestApproach]:
    """The record: the contact pass if the lifecycle accepted DOCKED, else
    the episode minimum; None without a joined segment."""
    contact = evidence.contact
    closest = contact if end.docked and contact is not None else evidence.best()
    if closest is None:
        return None
    outcome, reason = _outcome(end, evidence.passed)
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
        outcome=outcome,
        aircraft_time_s=nearer.aircraft_time_s,
        point=_approach_point(point, home_amsl_m),
        dock=_approach_point(dock, home_amsl_m),
        distance=MissDistance(
            closest.components.slant, closest.components.h_dist, closest.components.v_dist,
        ),
        attitude=None if attitude is None else AttitudeDeg(attitude.roll, attitude.pitch, attitude.yaw),
        reason=reason,
    )


def _outcome(end: DockEpisodeEnd, passed: bool) -> tuple[str, Optional[str]]:
    """Docked, a latched miss, or NAV's own failure after the UAV passed the
    dock (missed_dock.md); every other end is aborted, with the reason the
    lifecycle named. Without NAV's latch there is no cut, so a cancel or an
    unknown end is never missed."""
    if end.docked:
        return OUTCOME_DOCKED, None
    if end.missed or (end.failed and passed):
        return OUTCOME_MISSED, None
    reason = end.abort_reason
    return OUTCOME_ABORTED, reason if valid_reason(reason) else None


def _approach_point(location: Location, home_amsl_m: Optional[float]) -> ApproachPoint:
    return ApproachPoint(
        location.lat,
        location.lng,
        location.alt,
        alt_rel_m=None if home_amsl_m is None else location.alt - home_amsl_m,
    )


__all__ = [
    "CLOSEST_APPROACH_MAX_CANDIDATES",
    "ClosestApproachSink",
    "ClosestApproachTracker",
    "ClosestPass",
    "EpisodeEvidence",
    "LocalPoint",
    "TrackedSample",
    "closest_approach",
    "exact_pass",
]
