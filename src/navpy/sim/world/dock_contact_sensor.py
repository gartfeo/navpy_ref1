"""Simulator contact sensor: the stand-in for the dock's net.

The only success source of a simulated final approach. It compares the
aircraft's telemetry position with the true position of the vehicle's OWN
armed dock and publishes one DOCKED event when the flown path passes within
the net radius. Nothing it computes reaches frames, the law or the command
path; it only hands one correlated event to the approach lifecycle.

It also keeps the episode's closest pass and, when the lifecycle ends the
episode, reports it once as a display record (gcs_sim_closest_approach.md).
"""

from __future__ import annotations

import logging
import math
import os
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Optional

from navpy.core.approach.logs.navigation_snap_geometry import (
    calc_distance,
    is_valid_location,
)
from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.location import Location
from navpy.core.ports.dock_contact import (
    DockContactCallback,
    DockContactEvent,
    DockContactIdentity,
)
from navpy.core.ports.pose_stream_rate import POSE_STREAM_RATE_HZ
from navpy.sim.world.dock_contact_geometry import (
    DockFrame,
    chord_bow_bound_m,
    flat_earth_error_bound_m,
    flat_segment_distance,
)
from navpy.sim.world.closest_approach_tracker import (
    ClosestApproachSink,
    ClosestApproachTracker,
    ClosestPass,
    closest_approach,
    exact_pass,
)
from navpy.sim.world.simulation_object import SimulationObject

_LOG = logging.getLogger(__name__)

# Owner-set net radius (2026-10-13). Contact is strictly inside it, matching
# the eval's strict `< 0.5 m` SNAP gate.
SIM_DOCK_NET_RADIUS_M = 0.5
# Sim-only override that shrinks the net for a forced-miss check
# (missed_dock.md): 0 makes every pass a miss. Never wider than the default,
# which the eval's DOCKED check assumes.
SIM_DOCK_NET_RADIUS_ENV = "NAVPY_SIM_DOCK_NET_RADIUS_M"
# Two consecutive position samples are joined only across at most this many
# nominal periods of the position stream (one missed sample). A longer gap
# could straight-line a curved path into a crossing that never happened, so
# it starts a new segment instead.
SIM_DOCK_CONTACT_MAX_SEGMENT_PERIODS = 2.0


def sim_dock_net_radius_m(env: Mapping[str, str] | None = None) -> float:
    """The net radius: the override when set, else the owner-set default."""
    raw = (os.environ if env is None else env).get(SIM_DOCK_NET_RADIUS_ENV)
    if raw is None or not raw.strip():
        return SIM_DOCK_NET_RADIUS_M
    radius_m = _valid_radius_m(float(raw))
    if radius_m > SIM_DOCK_NET_RADIUS_M:
        raise ValueError(f"{SIM_DOCK_NET_RADIUS_ENV} may only shrink the net, got {raw!r}")
    return radius_m


def _valid_radius_m(radius_m: float) -> float:
    if not math.isfinite(radius_m) or radius_m < 0.0:
        raise ValueError(f"dock net radius must be finite and >= 0, got {radius_m!r}")
    return radius_m


def max_segment_s(position_rate_hz: float) -> float:
    """Longest joinable sample interval for this position stream rate."""
    if not math.isfinite(position_rate_hz) or position_rate_hz <= 0.0:
        raise ValueError("position stream rate must be positive and finite")
    return SIM_DOCK_CONTACT_MAX_SEGMENT_PERIODS / position_rate_hz


@dataclass(frozen=True)
class PositionSample:
    aircraft_time_s: float
    location: Location
    ground_speed_mps: float
    # Display only (the closest-approach record), never contact geometry.
    relative_alt_m: Optional[float] = None
    attitude: Optional[Attitude] = None


@dataclass(frozen=True)
class _Joined:
    sample: PositionSample
    frame: DockFrame
    local: tuple[float, float, float]


class SimDockContactSensor:
    """Arm per (vehicle, task, dock uid, episode); publish one DOCKED event."""

    def __init__(
        self,
        docks: Callable[[], Sequence[SimulationObject]],
        position_rate_hz: float = POSE_STREAM_RATE_HZ,
        net_radius_m: float = SIM_DOCK_NET_RADIUS_M,
        report: Optional[ClosestApproachSink] = None,
    ) -> None:
        self._docks = docks
        self._report = report
        self._net_radius_m = _valid_radius_m(net_radius_m)
        self._max_segment_s = max_segment_s(position_rate_hz)
        self._lock = rt.Lock()
        self._identity: Optional[DockContactIdentity] = None
        self._on_docked: Optional[DockContactCallback] = None
        self._previous: Optional[_Joined] = None
        self._frame: Optional[DockFrame] = None
        self._published = False
        self._closest = ClosestApproachTracker()

    def arm(
        self,
        identity: DockContactIdentity,
        on_docked: DockContactCallback,
    ) -> None:
        with self._lock:
            self._identity = identity
            self._on_docked = on_docked
            self._previous = None
            self._published = False
            self._closest.take()

    def disarm(self, docked: bool = False) -> None:
        """End the episode; report its closest pass once.

        ``docked`` is the lifecycle's accepted outcome, never the sensor's own
        contact: a contact the lifecycle rejected reports as missed.
        """
        with self._lock:
            identity = self._identity
            best, contact = self._closest.take()
            self._identity = None
            self._on_docked = None
            self._previous = None
            self._published = False
        if identity is None or self._report is None:
            return
        try:
            record = closest_approach(identity, docked, best, contact)
            if record is not None:
                self._report(record)
        except Exception:
            # Display evidence must never break NAV cleanup.
            _LOG.exception("closest-approach report failed")

    def sample(self, sample: PositionSample) -> None:
        """One telemetry position; never joined across a gap or a reset."""
        with self._lock:
            event, callback = self._sample_locked(sample)
        if event is not None and callback is not None:
            callback(event)

    def _sample_locked(
        self,
        sample: PositionSample,
    ) -> tuple[Optional[DockContactEvent], Optional[DockContactCallback]]:
        """Runs on the telemetry reader thread: cheap unless near the dock.

        Every sample costs a flat-frame segment distance. The exact
        geodetic closest point and the chord-bow bound run only when that
        distance, widened by the flat-earth error bound, is within the net.
        After the one contact the episode minimum is still kept: the
        lifecycle may reject that contact.
        """
        identity = self._identity
        if identity is None:
            return None, None
        if not _valid(sample):
            self._previous = None
            return None, None
        frame = self._own_frame(identity.dock_uid)
        if frame is None:
            self._previous = None
            return None, None
        previous = self._previous
        if previous is not None and (
            sample.aircraft_time_s == previous.sample.aircraft_time_s
        ):
            return None, None
        current = _Joined(sample, frame, frame.local(sample.location))
        self._previous = current
        if previous is None or not self._joinable(previous, current):
            return None, None
        flat_m, margin_m = _flat_distance(previous, current)
        segment = self._closest.keep(previous.sample, sample, frame.dock, flat_m, margin_m)
        if self._published:
            return None, None
        # Coarse filter; never false for a segment the exact test docks: the
        # exact contact needs closest < radius (the bow is >= 0), and the
        # flat closest distance is within the flat-earth bound of the exact
        # one, so flat closest < radius + bound covers every contact.
        if not flat_m < self._net_radius_m + margin_m:
            return None, None
        if segment is None:
            segment = exact_pass(previous.sample, sample, frame.dock)
        return self._exact_contact(identity, segment)

    def _exact_contact(
        self,
        identity: DockContactIdentity,
        segment: ClosestPass,
    ) -> tuple[Optional[DockContactEvent], Optional[DockContactCallback]]:
        start, end, components = segment.start, segment.end, segment.components
        elapsed_s = end.aircraft_time_s - start.aircraft_time_s
        chord_m = calc_distance(start.location, end.location)
        speed_mps = max(
            start.ground_speed_mps,
            end.ground_speed_mps,
            chord_m / elapsed_s,
        )
        bow_m = chord_bow_bound_m(chord_m, speed_mps)
        if not components.slant + bow_m < self._net_radius_m:
            return None, None
        self._published = True
        self._closest.contact(segment)
        return (
            DockContactEvent(
                identity=identity,
                aircraft_time_s=end.aircraft_time_s,
                closest_3d_m=components.slant,
                closest_h_m=components.h_dist,
                closest_v_m=components.v_dist,
            ),
            self._on_docked,
        )

    def _joinable(self, previous: _Joined, current: _Joined) -> bool:
        elapsed_s = current.sample.aircraft_time_s - previous.sample.aircraft_time_s
        return (
            # A world reload places a new dock object: never join across it.
            previous.frame.dock is current.frame.dock
            # The aircraft clock went back (reboot/replay): a new segment.
            and elapsed_s > 0.0
            and elapsed_s <= self._max_segment_s
        )

    def _own_frame(self, dock_uid: int) -> Optional[DockFrame]:
        """The armed dock's frame, recomputed only when its object changes."""
        dock = self._own_dock(dock_uid)
        if dock is None or not is_valid_location(dock.g_loc):
            return None
        frame = self._frame
        if frame is None or frame.dock is not dock:
            frame = self._frame = DockFrame.at(dock)
        return frame

    def _own_dock(self, dock_uid: int) -> Optional[SimulationObject]:
        return next(
            (dock for dock in self._docks() if dock.uid == dock_uid),
            None,
        )


def _flat_distance(previous: _Joined, current: _Joined) -> tuple[float, float]:
    """The flat closest distance and its flat-earth error bound."""
    flat_m = flat_segment_distance(previous.local, current.local)
    reach_m = math.dist((0.0, 0.0, 0.0), previous.local) + math.dist(
        (0.0, 0.0, 0.0), current.local,
    )
    return flat_m, flat_earth_error_bound_m(reach_m, current.frame.abs_tan_lat)


def _valid(sample: PositionSample) -> bool:
    return (
        math.isfinite(sample.aircraft_time_s)
        and math.isfinite(sample.ground_speed_mps)
        and sample.ground_speed_mps >= 0.0
        and is_valid_location(sample.location)
    )


__all__ = [
    "ClosestApproachSink",
    "PositionSample",
    "SIM_DOCK_CONTACT_MAX_SEGMENT_PERIODS",
    "SIM_DOCK_NET_RADIUS_ENV",
    "SIM_DOCK_NET_RADIUS_M",
    "SimDockContactSensor",
    "sim_dock_net_radius_m",
    "max_segment_s",
]
