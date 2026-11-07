"""NAV loop recovery stages, after ArduPlane's failsafe pattern (docs/design/navpy_recovery.md, step 4).

The loop reports each period whether its cycle ran without an error. Then:
- Short failure (bad for less than NAV_HOLD_TRIGGER_S): nothing; NavPy keeps its state, task and dock.
- Hold (bad for NAV_HOLD_TRIGGER_S): close the command fence, so nothing the cycle forms reaches the UAV, and,
  only if NavPy owns the mode (nav_ownership.py), ask for LOITER. The cycle keeps running behind the fence; that is
  how health is seen (owner ruling 2026-11-07: "keep running, blocked"). While holding, the approach must not end
  for seeing LOITER, and the search mission is not restarted (``holding``).
- Resume (good for NAV_HOLD_TRIGGER_S while holding): if the mode is still NavPy's LOITER, ask for AUTO; once a
  newer heartbeat shows it, reopen the fence. The mission goes on from its current item (MIS_RESTART 0) and the
  normal flow approaches the kept dock again. Without ownership the fence just reopens; no mode is changed.
- Critical (NAV_HOLD_RECOVERY_S in the hold, or LOITER/AUTO not confirmed in MODE_CONFIRM_S): RTL if owned. Not
  undone; the fence stays closed until NavPy is restarted (the operator's stop/start).
Pilot, GCS and failsafe modes are never touched: every mode request needs ownership and a fresh heartbeat.

Times are aircraft time (the autopilot's boot time) when it advances, else wall time (nav_recovery_clock.py): a
stalled link stops aircraft time, and the hold must still come.
"""

from __future__ import annotations

from collections.abc import Callable
from contextlib import AbstractContextManager, nullcontext
from dataclasses import dataclass
from enum import Enum

from navpy.core.flow.nav_ownership import ModeObservation, NavOwnership, Ownership, OwnershipEnd
from navpy.core.flow.nav_recovery_clock import RecoveryClock, TimeSample
from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.ports.flight_mode import FlightMode

#: Bad, then good, time that starts the hold and the resume: ArduPlane's FS_SHORT_TIMEOUT default.
NAV_HOLD_TRIGGER_S = 1.5
#: Time in the hold before RTL (owner ruling 2026-11-05: "RTL after 30 s").
NAV_HOLD_RECOVERY_S = 30.0
#: A mode request NavPy owns must show in a newer heartbeat by then (ArduPlane heartbeats at 1 Hz).
MODE_CONFIRM_S = 3.0
#: Repeat an unconfirmed mode request this often, not every 50 Hz period.
MODE_RETRY_S = 1.0


class RecoveryStage(Enum):
    NORMAL = "normal"
    HOLD = "hold"
    RESUMING = "resuming"
    CRITICAL = "critical"


@dataclass(frozen=True)
class NavRecoveryLink:
    """The vehicle side, built where the bus and vehicle are concrete (app/runtime_composition.py)."""
    close_fence: Callable[[], object]
    reopen_fence: Callable[[], object]
    observe_mode: Callable[[], ModeObservation]
    #: Wraps a command's forming and sending on a worker thread, so a close before its send drops it.
    formed_now: Callable[[], AbstractContextManager[None]] = nullcontext


@dataclass(frozen=True)
class NavRecoveryPorts:
    close_fence: Callable[[], object]
    reopen_fence: Callable[[], object]
    observe_mode: Callable[[], ModeObservation]
    request_mode: Callable[[FlightMode], object]
    logger: ILogger


class RecoveryHold:
    """What the cycle reads and tells: NAV state's view of recovery (no callbacks, owned by the NAV loop thread)."""

    def __init__(self) -> None:
        self.holding = False
        self.resumed = False   # set for one period after the hold ends
        self._guided: bool | None = None   # NavPy's last GUIDED event since the last step: asked, or withdrawn

    def guided_requested(self) -> None:
        """NavPy asked for GUIDED: recovery may acquire ownership from the next heartbeat."""
        self._guided = True

    def guided_withdrawn(self) -> None:
        """NavPy stopped waiting for GUIDED (its task was cleared): a later GUIDED is not NavPy's."""
        self._guided = False

    def take_guided_request(self) -> bool | None:
        guided, self._guided = self._guided, None
        return guided


class NavRecovery:
    """Owned by the NAV loop thread: ``step`` once per period, after the cycle.

    Critical RTL needs ownership; without it (search in the mission's AUTO, a pilot mode) NavPy stays fenced in the
    hold and resumes when healthy, since the autopilot flies on safely.
    """

    def __init__(
        self,
        ports: NavRecoveryPorts,
        hold: RecoveryHold | None = None,
        ownership: NavOwnership | None = None,
    ) -> None:
        self._ports = ports
        self.hold = RecoveryHold() if hold is None else hold
        self.ownership = NavOwnership() if ownership is None else ownership
        self.stage = RecoveryStage.NORMAL
        self._clock = RecoveryClock()
        self._bad_since: float | None = None
        self._good_since: float | None = None
        self._hold_since: float | None = None
        self._pending: FlightMode | None = None   # NavPy's mode request awaiting a newer heartbeat
        self._pending_at = (0.0, -1)   # the pending request's clock reading and heartbeat count
        self._long_hold_reported = False
        self._last_request: float | None = None

    def step(self, cycle_ok: bool, sample: TimeSample) -> None:
        now = self._clock.advance(sample)
        observation = self._ports.observe_mode()
        guided = self.hold.take_guided_request()
        if guided:
            # Recorded at the count read now, after the request: acquiring then needs the next GUIDED
            # heartbeat, one heartbeat later at most.
            self.ownership.requested(FlightMode.GUIDED, observation.heartbeat_count, observation.mode)
        elif guided is False:
            self.ownership.withdraw(FlightMode.GUIDED)
        held = self.hold.holding
        self.ownership.observe(observation)
        self._track(cycle_ok, now)
        self._step_stage(observation, now)
        # The fence is closed by recovery: the approach must not end for the hold's LOITER, nor search restart.
        self.hold.holding = self.stage is not RecoveryStage.NORMAL
        # One period's pulse: the next cycle's decision approaches the kept dock again instead of ending the approach.
        self.hold.resumed = held and not self.hold.holding

    def _step_stage(self, observation: ModeObservation, now: float) -> None:
        if self.stage is RecoveryStage.NORMAL:
            if self._bad_since is not None and now - self._bad_since >= NAV_HOLD_TRIGGER_S:
                self._enter_hold(observation, now)
        elif self.stage is RecoveryStage.HOLD:
            self._hold(observation, now)
        elif self.stage is RecoveryStage.RESUMING:
            self._resuming(observation, now)
        else:
            self._critical(observation, now)

    def _track(self, cycle_ok: bool, now: float) -> None:
        if cycle_ok:
            self._bad_since = None
            self._good_since = now if self._good_since is None else self._good_since
        else:
            self._good_since = None
            self._bad_since = now if self._bad_since is None else self._bad_since

    def _enter_hold(self, observation: ModeObservation, now: float) -> None:
        self._ports.close_fence()
        self.stage = RecoveryStage.HOLD
        self._hold_since = now
        self._good_since = None
        owned = self.ownership.may_request_mode()
        self._report(f"NavPy hold: commands fenced{', LOITER' if owned else ''}")
        if owned and observation.mode is not FlightMode.LOITER:
            self._request(FlightMode.LOITER, observation, now)

    def _hold(self, observation: ModeObservation, now: float) -> None:
        if self._mode_not_confirmed(observation, now):
            self._enter_critical(observation, now, "LOITER not confirmed")
            return
        if self._hold_since is not None and now - self._hold_since >= NAV_HOLD_RECOVERY_S:
            if self.ownership.may_request_mode():
                self._enter_critical(observation, now, f"no recovery in {NAV_HOLD_RECOVERY_S:g} s")
                return
            if not self._long_hold_reported:
                # Not NavPy's mode (the mission's AUTO, the pilot's): it flies on safely; NavPy waits fenced.
                self._long_hold_reported = True
                self._report(f"NavPy not recovered in {NAV_HOLD_RECOVERY_S:g} s; mode not NavPy's, commands stay fenced")
        if self._good_since is None or now - self._good_since < NAV_HOLD_TRIGGER_S:
            return
        if self.ownership.may_request_mode() and observation.mode is FlightMode.LOITER:
            self.stage = RecoveryStage.RESUMING
            self._request(FlightMode.AUTO, observation, now)
            self._report("NavPy healthy again: AUTO")
        elif self.ownership.state is not Ownership.UNKNOWN:
            self._resume_normal("NavPy healthy again: commands reopened")

    def _resuming(self, observation: ModeObservation, now: float) -> None:
        if self.ownership.ended is not None:
            # Handed over (AUTO shown), or the pilot took the mode meanwhile: either way NavPy goes on.
            self._resume_normal("NavPy resumed in AUTO" if self.ownership.ended is OwnershipEnd.HANDED_OVER
                                else "NavPy resumed: the mode was taken over")
        elif self._mode_not_confirmed(observation, now):
            self._enter_critical(observation, now, "AUTO not confirmed")

    def _critical(self, observation: ModeObservation, now: float) -> None:
        # Also after a critical entered unowned-looking (stale link): once a fresh heartbeat shows NavPy owns the
        # mode, RTL is sent then; a pending one is retried.
        if observation.mode is FlightMode.RTL or not self.ownership.may_request_mode():
            return
        if self._pending is FlightMode.RTL:
            self._retry(observation, now)
        else:
            self._report("NavPy critical: mode owned again, RTL")
            self._request(FlightMode.RTL, observation, now)

    def _enter_critical(self, observation: ModeObservation, now: float, why: str) -> None:
        self.stage = RecoveryStage.CRITICAL
        self._pending = None
        owned = self.ownership.may_request_mode()
        self._report(f"NavPy critical ({why}): {'RTL' if owned else 'commands stay fenced'}")
        if owned:
            self._request(FlightMode.RTL, observation, now)

    def _mode_not_confirmed(self, observation: ModeObservation, now: float) -> bool:
        """True once a pending request has waited MODE_CONFIRM_S; retries it until then."""
        if self._pending is None:
            return False
        if observation.mode is self._pending and observation.heartbeat_count > self._pending_at[1]:
            self._pending = None
            return False
        if now - self._pending_at[0] >= MODE_CONFIRM_S:
            return True
        self._retry(observation, now)
        return False

    def _retry(self, observation: ModeObservation, now: float) -> None:
        if self._pending is None or not self.ownership.may_request_mode():
            return
        if self._last_request is None or now - self._last_request >= MODE_RETRY_S:
            self._send(self._pending, observation, now)

    def _request(self, mode: FlightMode, observation: ModeObservation, now: float) -> None:
        self._pending = mode
        self._pending_at = (now, observation.heartbeat_count)
        self._send(mode, observation, now)

    def _send(self, mode: FlightMode, observation: ModeObservation, now: float) -> None:
        self.ownership.requested(mode, observation.heartbeat_count)
        self._last_request = now
        try:
            self._ports.request_mode(mode)
        except Exception as error:  # noqa: BLE001 - retried; a send error must not end the loop
            self._report(f"NavPy {mode.name} request failed: {error}")

    def _resume_normal(self, message: str) -> None:
        self._ports.reopen_fence()
        self.stage = RecoveryStage.NORMAL
        self._pending = None
        self._hold_since = None
        self._bad_since = None
        self._long_hold_reported = False
        self._report(message)

    def _report(self, message: str) -> None:
        try:
            self._ports.logger.warning(message, key="nav", dest=LogStatusDest.DRONE)
        except Exception:  # noqa: BLE001 - reporting must not end recovery
            pass


__all__ = [
    "MODE_CONFIRM_S",
    "NavRecoveryLink",
    "RecoveryHold",
    "MODE_RETRY_S",
    "NAV_HOLD_RECOVERY_S",
    "NAV_HOLD_TRIGGER_S",
    "NavRecovery",
    "NavRecoveryPorts",
    "RecoveryStage",
    "TimeSample",
]
