"""NAV recovery's data: the cycle's view (RecoveryHold), the main thread's escalation and the vehicle
side (core/flow/nav_recovery.py has the stages)."""

from __future__ import annotations

from collections.abc import Callable
from contextlib import AbstractContextManager, nullcontext
from dataclasses import dataclass, field

from navpy.core.flow.nav_ownership import ModeObservation, NavOwnership
from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.kernel.progress_beacon import ProgressBeacon
from navpy.core.ports.flight_mode import FlightMode


class NavEscalation:
    """Between the main thread, which sees a latched component failure, and the NAV loop, which owns the mode."""

    def __init__(self) -> None:
        self._requested = rt.Event()
        self._released = rt.Event()

    def request(self) -> None:
        self._requested.set()

    @property
    def requested(self) -> bool:
        return self._requested.is_set()

    def release(self) -> None:
        """Set by the NAV loop: the UAV is the mission's, the pilot's or in RTL, so NavPy may exit."""
        self._released.set()

    @property
    def released(self) -> bool:
        return self._released.is_set()


@dataclass(frozen=True)
class WatchdogHold:
    """What the in-process watchdog did while a loop was hung (core/flow/nav_watchdog.py), told to the NAV loop
    once it runs again. The fence was closed. ``ownership`` is the watchdog's continuation of the NAV loop's record,
    with its own LOITER/RTL requests, when the NAV loop itself was hung; None when only the command worker was."""
    ownership: NavOwnership | None
    critical: bool


class WatchdogSignal:
    """Between the watchdog thread and the NAV loop; the watchdog never touches NAV state itself."""

    def __init__(self) -> None:
        self._lock = rt.Lock()
        self._hung = False
        self._fenced = False
        self._ownership: NavOwnership | None = None
        self._hold: WatchdogHold | None = None

    @property
    def fenced(self) -> bool:
        """The watchdog closed the fence and NAV has not taken the hold over yet: NAV's decision must treat it as
        recovery's hold already (no approach ended for its LOITER), though recovery's step comes after the cycle."""
        with self._lock:
            return self._fenced

    def mark_fenced(self) -> None:
        with self._lock:
            self._fenced = True

    @property
    def hung(self) -> bool:
        """A watched loop is stuck now: the NAV loop counts its periods as failed (the hold comes from that)."""
        with self._lock:
            return self._hung

    def set_hung(self, hung: bool) -> None:
        with self._lock:
            self._hung = bool(hung)

    def publish_ownership(self, ownership: NavOwnership) -> None:
        """The NAV loop's record after each step, as an independent copy the watchdog may go on with."""
        copy = ownership.copy()
        with self._lock:
            self._ownership = copy

    def ownership(self) -> NavOwnership | None:
        """A copy of the NAV loop's last published record, for the watchdog to observe and request with."""
        with self._lock:
            return None if self._ownership is None else self._ownership.copy()

    def report_hold(self, hold: WatchdogHold) -> None:
        with self._lock:
            self._hold = hold

    def take_hold(self) -> WatchdogHold | None:
        """NAV takes the hold over; from then its own stage says it is holding."""
        with self._lock:
            hold, self._hold = self._hold, None
            if hold is not None:
                self._fenced = False
            return hold


@dataclass(frozen=True)
class NavRecoveryLink:
    """The vehicle side, built where the bus and vehicle are concrete (app/runtime_composition.py)."""
    close_fence: Callable[[], object]
    reopen_fence: Callable[[], object]
    observe_mode: Callable[[], ModeObservation]
    #: Closes the fence for good: the main thread's escalation, so NAV's reopen racing it cannot open it again.
    latch_fence: Callable[[], object]
    escalation: NavEscalation = field(default_factory=NavEscalation)
    #: Wraps a command's forming and sending on a worker thread, so a close before its send drops it.
    formed_now: Callable[[], AbstractContextManager[None]] = nullcontext
    #: The command worker's passes, watched for a hang (step 6); the NAV loop's own beacon is the loop's.
    command_progress: ProgressBeacon = field(default_factory=ProgressBeacon)
    watchdog: WatchdogSignal = field(default_factory=WatchdogSignal)


@dataclass(frozen=True)
class NavRecoveryPorts:
    close_fence: Callable[[], object]
    reopen_fence: Callable[[], object]
    observe_mode: Callable[[], ModeObservation]
    request_mode: Callable[[FlightMode], object]
    logger: ILogger
    escalation: NavEscalation = field(default_factory=NavEscalation)
    watchdog: WatchdogSignal = field(default_factory=WatchdogSignal)


class RecoveryHold:
    """What the cycle reads and tells: NAV state's view of recovery (owned by the NAV loop thread).

    ``observe_mode`` is the link's heartbeat reading (NavRecoveryLink.observe_mode); None without a link, where there
    is no recovery to tell."""

    def __init__(
        self,
        observe_mode: Callable[[], ModeObservation] | None = None,
        watchdog: WatchdogSignal | None = None,
    ) -> None:
        self._observe_mode = observe_mode
        self._watchdog = watchdog
        self._holding = False
        #: Set when the hold ended in NavPy's own AUTO; kept until the decision takes it (``take_resumed``), so a
        #: cycle that fails before deciding does not lose it. Cleared by a new hold.
        self.resumed = False
        # NavPy's last GUIDED event since the last step: asked (the heartbeat read just before sending), or withdrawn
        self._guided: ModeObservation | bool | None = None

    @property
    def holding(self) -> bool:
        """Recovery holds: its stage, or the watchdog's fence not yet taken over (step 6)."""
        return self._holding or (self._watchdog is not None and self._watchdog.fenced)

    @holding.setter
    def holding(self, value: bool) -> None:
        self._holding = bool(value)

    def guided_requested(self) -> None:
        """NavPy is about to send GUIDED: the heartbeat count and mode are read now, at the request boundary, so
        ownership is judged from exactly what NavPy asked from (nav_ownership.py)."""
        if self._observe_mode is not None:
            self._guided = self._observe_mode()

    def guided_withdrawn(self) -> None:
        """NavPy stopped waiting for GUIDED (its task was cleared): a later GUIDED is not NavPy's."""
        self._guided = False

    def take_resumed(self) -> bool:
        resumed, self.resumed = self.resumed, False
        return resumed

    def take_guided_request(self) -> ModeObservation | bool | None:
        guided, self._guided = self._guided, None
        return guided


__all__ = [
    "NavEscalation",
    "NavRecoveryLink",
    "NavRecoveryPorts",
    "RecoveryHold",
    "WatchdogHold",
    "WatchdogSignal",
]
