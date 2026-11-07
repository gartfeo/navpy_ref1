"""NAV recovery's data: the cycle's view (RecoveryHold), the main thread's escalation and the vehicle
side (core/flow/nav_recovery.py has the stages)."""

from __future__ import annotations

from collections.abc import Callable
from contextlib import AbstractContextManager, nullcontext
from dataclasses import dataclass, field

from navpy.core.flow.nav_ownership import ModeObservation
from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.logging.cache_logger import ILogger
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


@dataclass(frozen=True)
class NavRecoveryPorts:
    close_fence: Callable[[], object]
    reopen_fence: Callable[[], object]
    observe_mode: Callable[[], ModeObservation]
    request_mode: Callable[[FlightMode], object]
    logger: ILogger
    escalation: NavEscalation = field(default_factory=NavEscalation)


class RecoveryHold:
    """What the cycle reads and tells: NAV state's view of recovery (no callbacks, owned by the NAV loop thread)."""

    def __init__(self) -> None:
        self.holding = False
        #: Set when the hold ended in NavPy's own AUTO; kept until the decision takes it (``take_resumed``), so a
        #: cycle that fails before deciding does not lose it. Cleared by a new hold.
        self.resumed = False
        self._guided: bool | None = None   # NavPy's last GUIDED event since the last step: asked, or withdrawn

    def guided_requested(self) -> None:
        """NavPy asked for GUIDED: recovery may acquire ownership from the next heartbeat."""
        self._guided = True

    def guided_withdrawn(self) -> None:
        """NavPy stopped waiting for GUIDED (its task was cleared): a later GUIDED is not NavPy's."""
        self._guided = False

    def take_resumed(self) -> bool:
        resumed, self.resumed = self.resumed, False
        return resumed

    def take_guided_request(self) -> bool | None:
        guided, self._guided = self._guided, None
        return guided


__all__ = [
    "NavEscalation",
    "NavRecoveryLink",
    "NavRecoveryPorts",
    "RecoveryHold",
]
