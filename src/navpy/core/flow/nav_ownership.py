"""Whether NavPy owns the flight mode, so recovery may change it (docs/design/navpy_recovery.md, step 3).

Pilot and autopilot failsafe keep authority. NavPy owns the mode only after it asked for GUIDED and a heartbeat newer
than that request shows GUIDED, armed. Ownership ends:
- revoked: a fresh heartbeat shows a mode NavPy did not ask for (pilot, GCS, failsafe), or the vehicle is disarmed;
- handed over: a heartbeat newer than NavPy's AUTO request shows AUTO (the mission has the UAV again).
While NavPy owns it, GUIDED and any mode shown after NavPy asked for it (the hold's LOITER) keep it owned.
Before it is owned, NavPy's GUIDED request lapses when a newer heartbeat shows disarmed or a mode other than GUIDED
and the one shown when NavPy asked (someone else changed it), or when NavPy withdraws it (stopped waiting): a later
GUIDED is then the operator's, not NavPy's.

Heartbeats are ordered by their count (newer = larger), not by a clock. A stale link (the caller's heartbeat
freshness) makes ownership unknown: no automatic mode request then, and the next fresh heartbeat decides again.
One record per approach episode; it is owned by the NAV loop thread and is not thread-safe.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from navpy.core.ports.flight_mode import FlightMode


class Ownership(Enum):
    NOT_OWNED = "not_owned"
    OWNED = "owned"
    UNKNOWN = "unknown"       # owned when last seen, but the heartbeat is stale


class OwnershipEnd(Enum):
    REVOKED_MODE = "revoked_mode"
    REVOKED_DISARM = "revoked_disarm"
    HANDED_OVER = "handed_over"


@dataclass(frozen=True)
class ModeObservation:
    """The vehicle's latest heartbeat as the caller read it: count first, then mode and armed."""
    heartbeat_count: int
    mode: FlightMode | None
    armed: bool
    fresh: bool


class NavOwnership:
    def __init__(self) -> None:
        self._owned = False
        self._fresh = True
        self._seen_count = -1
        self._requests: dict[FlightMode, int] = {}   # NavPy's mode requests: mode -> heartbeat count at request
        self._guided_from: FlightMode | None = None   # the mode shown when NavPy asked for GUIDED, not yet owned
        self.ended: OwnershipEnd | None = None

    @property
    def state(self) -> Ownership:
        if not self._owned:
            return Ownership.NOT_OWNED
        return Ownership.OWNED if self._fresh else Ownership.UNKNOWN

    @property
    def guided_pending(self) -> bool:
        """NavPy's GUIDED request is neither acquired nor lapsed nor withdrawn."""
        return not self._owned and FlightMode.GUIDED in self._requests

    def may_request_mode(self) -> bool:
        """An automatic mode request (hold, handover) only while owned and the heartbeat is fresh."""
        return self.state is Ownership.OWNED

    def requested(self, mode: FlightMode, heartbeat_count: int, shown: FlightMode | None = None) -> None:
        """NavPy sent a request for ``mode``; ``heartbeat_count`` and ``shown`` were read just before sending."""
        if mode is FlightMode.GUIDED and not self._owned:
            self.ended = None   # a new acquisition for this episode
            self._guided_from = shown
        self._requests[mode] = int(heartbeat_count)

    def withdraw(self, mode: FlightMode) -> None:
        """NavPy no longer waits for ``mode``; an owned mode stays owned."""
        if not self._owned:
            self._requests.pop(mode, None)

    def observe(self, observation: ModeObservation) -> None:
        self._fresh = bool(observation.fresh)
        if not self._fresh or observation.heartbeat_count <= self._seen_count:
            return   # stale link or no newer heartbeat: nothing to decide on
        self._seen_count = observation.heartbeat_count
        if self._owned:
            self._observe_owned(observation)
        elif self._newer_than_request(FlightMode.GUIDED, observation):
            self._observe_guided_request(observation)

    def _observe_guided_request(self, observation: ModeObservation) -> None:
        if observation.mode is FlightMode.GUIDED and observation.armed:
            self._owned = True
            self.ended = None
        elif not observation.armed or observation.mode is not self._guided_from:
            self._requests.pop(FlightMode.GUIDED, None)   # another mode or a disarm: the request lapsed

    def _observe_owned(self, observation: ModeObservation) -> None:
        mode = observation.mode
        if not observation.armed:
            self._end(OwnershipEnd.REVOKED_DISARM)
        elif mode is FlightMode.GUIDED:
            return   # acquired in GUIDED
        elif mode is None or not self._newer_than_request(mode, observation):
            # A mode NavPy did not ask for, or one shown before NavPy asked: someone else set it.
            self._end(OwnershipEnd.REVOKED_MODE)
        elif mode is FlightMode.AUTO:
            self._end(OwnershipEnd.HANDED_OVER)

    def copy(self) -> NavOwnership:
        """An independent copy: what the NAV loop publishes for the watchdog to go on with while it is hung."""
        twin = NavOwnership()
        twin.adopt(self)
        return twin

    def adopt(self, other: NavOwnership) -> None:
        """Take ``other``'s record: the watchdog's, which went on observing while the NAV loop was hung."""
        self._owned = other._owned
        self._fresh = other._fresh
        self._seen_count = other._seen_count
        self._requests = dict(other._requests)
        self._guided_from = other._guided_from
        self.ended = other.ended

    def _newer_than_request(self, mode: FlightMode, observation: ModeObservation) -> bool:
        sent_at = self._requests.get(mode)
        return sent_at is not None and observation.heartbeat_count > sent_at

    def _end(self, reason: OwnershipEnd) -> None:
        self._owned = False
        self._requests.clear()
        self.ended = reason


__all__ = ["ModeObservation", "NavOwnership", "Ownership", "OwnershipEnd"]
