"""NAV recovery's data: the cycle's view (RecoveryHold), the main thread's escalation and the vehicle
side (core/flow/nav_recovery.py has the stages)."""

from __future__ import annotations

from collections.abc import Callable
from contextlib import AbstractContextManager, nullcontext
from dataclasses import dataclass, field

from navpy.core.flow.nav_ownership import ModeObservation
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


class WatchdogSignal:
    """Set by the watchdog thread (core/flow/nav_watchdog.py): a watched loop is stuck now. NAV recovery reads it
    in its step, which the NAV loop runs each period and the watchdog runs while the NAV loop itself is stuck."""

    def __init__(self) -> None:
        self._hung = rt.Event()

    @property
    def hung(self) -> bool:
        return self._hung.is_set()

    def set_hung(self, hung: bool) -> None:
        if hung:
            self._hung.set()
        else:
            self._hung.clear()


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
    #: Recovery's mode requests, with a bounded wait for the send lock (nav_mode_request.MODE_SEND_LOCK_TIMEOUT_S);
    #: None: the vehicle's plain set_mode.
    request_mode: Callable[[FlightMode], object] | None = None


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

    def __init__(self, observe_mode: Callable[[], ModeObservation] | None = None) -> None:
        self._observe_mode = observe_mode
        self._guided_lock = rt.Lock()   # the cycle tells, recovery's step takes: on the watchdog thread too
        self.holding = False
        #: Set when the hold ended in NavPy's own AUTO; kept until the decision takes it (``take_resumed``), so a
        #: cycle that fails before deciding does not lose it. Cleared by a new hold.
        self.resumed = False
        # NavPy's last GUIDED event since the last step: asked (the heartbeat read just before sending), or withdrawn
        self._guided: ModeObservation | bool | None = None

    def guided_requested(self) -> None:
        """NavPy is about to send GUIDED: the heartbeat count and mode are read now, at the request boundary, so
        ownership is judged from exactly what NavPy asked from (nav_ownership.py)."""
        if self._observe_mode is not None:
            observation = self._observe_mode()
            with self._guided_lock:
                self._guided = observation

    def guided_withdrawn(self) -> None:
        """NavPy stopped waiting for GUIDED (its task was cleared): a later GUIDED is not NavPy's."""
        with self._guided_lock:
            self._guided = False

    def take_resumed(self) -> bool:
        resumed, self.resumed = self.resumed, False
        return resumed

    def take_guided_request(self) -> ModeObservation | bool | None:
        with self._guided_lock:
            guided, self._guided = self._guided, None
            return guided


__all__ = [
    "NavEscalation",
    "NavRecoveryLink",
    "NavRecoveryPorts",
    "RecoveryHold",
    "WatchdogSignal",
]
