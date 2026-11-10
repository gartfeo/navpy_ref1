"""NAV recovery's data: the cycle's view (RecoveryHold), the main thread's escalation and the vehicle
side (core/flow/nav_recovery.py has the stages)."""

from __future__ import annotations

from collections.abc import Callable
from contextlib import AbstractContextManager, nullcontext
from dataclasses import dataclass, field

from navpy.core.flow.nav_mode_sender import ModeSender
from navpy.core.flow.nav_ownership import ModeObservation
from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.kernel.progress_beacon import ProgressBeacon
from navpy.core.ports.flight_mode import FlightMode


@dataclass(frozen=True)
class HoldClaim:
    """NavPy's claim on the LOITER of its recovery hold, for the autopilot's watchdog (nav_watchdog.lua; step 10,
    item 9 and Codex round 2): ``token`` names the LOITER request (each request a new one, so the watchdog takes a
    claim once, at the switch into that LOITER); ``budget_s`` is the incident's hold budget left, so the watchdog's RTL
    deadline keeps the summed-hold bound (nav_recovery_incident.py)."""
    token: int
    budget_s: float


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
    #: Recovery's guarded mode write (nav_mode_sender.GuardedSend): a bounded wait for the send lock, and the guard
    #: called under it right before the write; None: no link to write through.
    request_mode: Callable[[FlightMode, Callable[[], bool]], bool] | None = None
    #: The watchdog's veto on reopening (MavCommandFence.hold_closed), while a loop is stuck.
    hold_fence: Callable[[bool], object] | None = None
    #: NavPy's claim on its own recovery LOITER, sent to the autopilot's watchdog in the companion heartbeat
    #: (companion_hold_claim.py, step 10 item 9); None: no claim.
    claim_hold: Callable[[HoldClaim | None], object] = lambda _claim: None


@dataclass(frozen=True)
class NavRecoveryPorts:
    close_fence: Callable[[], object]
    reopen_fence: Callable[[], object]
    observe_mode: Callable[[], ModeObservation]
    request_mode: Callable[[FlightMode], object]
    logger: ILogger
    escalation: NavEscalation = field(default_factory=NavEscalation)
    watchdog: WatchdogSignal = field(default_factory=WatchdogSignal)
    #: Writes recovery's modes on its own thread (nav_mode_sender.py); None: written inline through
    #: ``request_mode`` (tests, no link).
    mode_sender: ModeSender | None = None
    #: NavPy holds in its own LOITER (NavRecoveryLink.claim_hold): the autopilot's watchdog then owns its deadline.
    claim_hold: Callable[[HoldClaim | None], object] = lambda _claim: None


class RecoveryHold:
    """What the cycle reads and tells: NAV state's view of recovery (owned by the NAV loop thread).

    ``observe_mode`` is the link's heartbeat reading (NavRecoveryLink.observe_mode); None without a link, where there
    is no recovery to tell."""

    def __init__(self, observe_mode: Callable[[], ModeObservation] | None = None) -> None:
        self._observe_mode = observe_mode
        self._guided_lock = rt.Lock()   # the cycle tells, recovery's step takes: on the watchdog thread too
        self.holding = False
        # The resume grant: recovery's authorization for one fresh approach to the kept dock, issued when the hold
        # ended in NavPy's own AUTO. The decision consumes it once that approach's NAV entry succeeded, so an entry
        # that raises is retried under the same grant; recovery voids it in any new hold or when a fresh heartbeat
        # shows a mode other than AUTO or GUIDED (the operator's). Recovery's step and the decision may run on
        # different threads (the watchdog's), so under the lock.
        self._resumed = False
        # NavPy's last GUIDED event since the last step: asked (the heartbeat read just before sending), or withdrawn
        self._guided: ModeObservation | bool | None = None
        # A failed one-shot dock's RTL, until a heartbeat shows it or a mode not NavPy's stands (nav_sortie_return.py)
        self._sortie_rtl = False

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

    @property
    def resumed(self) -> bool:
        with self._guided_lock:
            return self._resumed

    def stepped(self, normal: bool, observation: ModeObservation) -> None:
        """After each recovery step. Not NORMAL: the fence is closed by recovery, so the approach must not end for
        the hold's LOITER, nor search restart (``holding``). The resume grant is void in a new hold, or when a fresh
        heartbeat shows the operator's mode (no fresh approach over it)."""
        self.holding = not normal
        if not normal or (observation.fresh and observation.mode not in (FlightMode.AUTO, FlightMode.GUIDED)):
            self.void_resume()

    def grant_resume(self) -> None:
        with self._guided_lock:
            self._resumed = True

    def void_resume(self) -> None:
        with self._guided_lock:
            self._resumed = False

    def consume_resume(self) -> bool:
        """The decision used the grant (its fresh entry succeeded): True if it was still granted."""
        with self._guided_lock:
            resumed, self._resumed = self._resumed, False
            return resumed

    def request_sortie_rtl(self) -> None:
        """The one-shot sortie ended in a failed dock: RTL, through recovery's ownership (step 10, item 3)."""
        with self._guided_lock:
            self._sortie_rtl = True

    @property
    def sortie_rtl(self) -> bool:
        with self._guided_lock:
            return self._sortie_rtl

    def end_sortie_rtl(self) -> None:
        with self._guided_lock:
            self._sortie_rtl = False

    def take_guided_request(self) -> ModeObservation | bool | None:
        with self._guided_lock:
            guided, self._guided = self._guided, None
            return guided


__all__ = [
    "HoldClaim",
    "NavEscalation",
    "NavRecoveryLink",
    "NavRecoveryPorts",
    "RecoveryHold",
    "WatchdogSignal",
]
