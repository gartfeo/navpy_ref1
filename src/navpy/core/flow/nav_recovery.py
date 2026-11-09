"""NAV loop recovery stages, after ArduPlane's failsafe pattern (docs/design/navpy_recovery.md, step 4).

The loop reports each period whether its cycle ran without an error. Then:
- Short failure (bad for less than NAV_HOLD_TRIGGER_S): nothing; NavPy keeps its state, task and dock.
- Hold (bad for NAV_HOLD_TRIGGER_S): close the command fence, so nothing the cycle forms reaches the UAV, and,
  only if NavPy owns the mode (nav_ownership.py), ask for LOITER. The cycle keeps running behind the fence; that is
  how health is seen (owner ruling 2026-11-07: "keep running, blocked"). While holding, the approach must not end
  for seeing LOITER, and the search mission is not restarted (``holding``).
- Resume (good for NAV_HOLD_TRIGGER_S while holding): if the mode is still NavPy's LOITER, ask for AUTO; once a
  newer heartbeat shows it, reopen the fence. The mission goes on from its current item (MIS_RESTART 0) and the
  normal flow approaches the kept dock again (the resume grant ``hold.resumed``, consumed once by the decision).
  The RTL budget is hold time summed across one incident's holds (nav_recovery_incident.py). Without ownership
  (the operator took the mode in the hold) the fence just reopens, no mode is changed and the approach is not
  resumed: the operator's mode stands.
- Critical (NAV_HOLD_RECOVERY_S in the hold, or LOITER/AUTO not confirmed in MODE_CONFIRM_S): RTL if owned. Not
  undone; the fence stays closed until NavPy is restarted (the operator's stop/start).
- Escalated (a failure NavPy cannot recover in place, told by the main thread through NavEscalation, step 5):
  close the fence and, if owned, hand the UAV to the mission's AUTO; AUTO not confirmed gives RTL. Released once
  a fresh heartbeat shows the mode is no longer NavPy's (and no GUIDED request of NavPy's may still take effect) or
  shows RTL; only then may a supervised NavPy exit for a fresh start.
Pilot, GCS and failsafe modes are never touched: every mode request needs ownership and a fresh heartbeat.

Times are aircraft time (the autopilot's boot time) when it advances, else wall time (nav_recovery_clock.py): a
stalled link stops aircraft time, and the hold must still come.
"""

from __future__ import annotations

from collections.abc import Callable
from enum import Enum

from navpy.core.flow.nav_mode_request import MODE_CONFIRM_S, ModeRequest
from navpy.core.flow.nav_mode_sender import ModeSender
from navpy.core.flow.nav_ownership import ModeObservation, NavOwnership, Ownership, OwnershipEnd
from navpy.core.flow.nav_recovery_incident import RecoveryIncident
from navpy.core.flow.nav_recovery_clock import RecoveryClock, TimeSample
from navpy.core.flow.nav_recovery_ports import NavRecoveryPorts, RecoveryHold
from navpy.core.flow.nav_sortie_return import return_after_failed_dock
from navpy.core.ports.flight_mode import FlightMode

#: Bad, then good, time that starts the hold and the resume: ArduPlane's FS_SHORT_TIMEOUT default.
NAV_HOLD_TRIGGER_S = 1.5
#: Time in the hold before RTL (owner ruling 2026-11-05: "RTL after 30 s").
NAV_HOLD_RECOVERY_S = 30.0


class RecoveryStage(Enum):
    NORMAL = "normal"
    HOLD = "hold"
    RESUMING = "resuming"
    ESCALATED = "escalated"
    CRITICAL = "critical"


def _inline_sender(ports: NavRecoveryPorts, report: Callable[[str], None]) -> ModeSender:
    """Without a mode sender thread (tests, no link): written inline through ``request_mode``, guard first."""
    return ModeSender(lambda mode, guard: guard() and ports.request_mode(mode) is not False,
                      ports.observe_mode, report, threaded=False)


def _owned_loiter(observation: ModeObservation, ownership: NavOwnership) -> bool:
    """Step 10, item 9: a LOITER NavPy owns (or did when last seen) is claimed in a hold, so the autopilot's watchdog
    guards it if NavPy dies (companion_hold_claim.py). Never a mode NavPy does not own: a pilot's LOITER stays the
    pilot's."""
    return observation.mode is FlightMode.LOITER and ownership.state is not Ownership.NOT_OWNED


class NavRecovery:
    """Owned by the NAV loop thread: ``step`` once per period, after the cycle. Critical RTL needs ownership;
    without it (search in the mission's AUTO, a pilot mode) NavPy stays fenced in the hold and resumes when healthy,
    since the autopilot flies on safely."""

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
        self._incident = RecoveryIncident(NAV_HOLD_RECOVERY_S)   # hold time summed per incident
        self._mode = ModeRequest(ports.mode_sender or _inline_sender(ports, self._report), self.ownership)
        self._long_hold_reported = False
        self._guided_at: float | None = None             # when recovery learned of NavPy's last GUIDED request
        self._handed_over_unhealthy = False   # AUTO was handed over while NavPy was not healthy: resume later

    def step(self, cycle_ok: bool, sample: TimeSample) -> None:
        """One thread at a time (nav_recovery_step.py); never blocks: modes are written by nav_mode_sender.py."""
        now = self._clock.advance(sample)
        self._incident.advance(self.stage in (RecoveryStage.HOLD, RecoveryStage.RESUMING), now)
        observation = self._ports.observe_mode()
        self._take_signals(observation, now)
        # A stuck loop (the watchdog's signal) fails the period like a failed cycle, and holds at once: the hang
        # has lasted longer than the hold trigger already (step 6).
        hung = self._ports.watchdog.hung
        self._track(cycle_ok and not hung, now)
        if self._ports.escalation.requested and self.stage not in (RecoveryStage.ESCALATED, RecoveryStage.CRITICAL):
            self._enter_escalated()
        self._step_stage(observation, now, hung)
        normal = self.stage is RecoveryStage.NORMAL
        self.hold.stepped(normal, observation)
        self._ports.claim_hold(not normal and _owned_loiter(observation, self.ownership))
        if normal:
            self._incident.healthy(self._good_since, now)
            return_after_failed_dock(self.hold, self._mode, self.ownership, observation, now, self._ports.logger)

    def _take_signals(self, observation: ModeObservation, now: float) -> None:
        """What the mode sender and the cycle told recovery since its last step, then the heartbeat."""
        self._mode.take_sent()
        guided = self.hold.take_guided_request()
        if isinstance(guided, ModeObservation) and self.stage is RecoveryStage.NORMAL:
            # At the count and mode read just before the send (RecoveryHold.guided_requested): a mode change after
            # it lapses the request. Behind the fence (not NORMAL) the request was dropped: not NavPy's.
            self.ownership.requested(FlightMode.GUIDED, guided.heartbeat_count, guided.mode)
            self._guided_at = now
        elif guided is False:
            self.ownership.withdraw(FlightMode.GUIDED)
        self.ownership.observe(observation)

    def _step_stage(self, observation: ModeObservation, now: float, hung: bool) -> None:
        if self.stage is RecoveryStage.NORMAL:
            if hung or (self._bad_since is not None and now - self._bad_since >= NAV_HOLD_TRIGGER_S):
                self._enter_hold(observation, now)
        elif self.stage is RecoveryStage.HOLD:
            self._hold(observation, now)
        elif self.stage is RecoveryStage.RESUMING:
            self._resuming(observation, now)
        elif self.stage is RecoveryStage.ESCALATED:
            self._escalated(observation, now)
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
        self.hold.holding = True   # before any mode request: the cycle must see the hold before its LOITER
        self._ports.close_fence()
        self._handed_over_unhealthy = False
        self.stage = RecoveryStage.HOLD
        self._incident.hold()
        self._good_since = None
        owned = self.ownership.may_request_mode()
        self._report(f"NavPy hold: commands fenced{', LOITER' if owned else ''}")
        if owned and observation.mode not in (FlightMode.LOITER, FlightMode.RTL):   # an RTL is never undone
            self._mode.send(FlightMode.LOITER, observation, now)

    def _hold(self, observation: ModeObservation, now: float) -> None:
        if self._mode.not_confirmed(observation, now):
            self._enter_critical(observation, now, "LOITER not confirmed")
            return
        if self.ownership.may_request_mode() and observation.mode is FlightMode.GUIDED and self._mode.pending is None:
            # Owned only now (NavPy's GUIDED showed after the hold began, e.g. sent just before a hang): hold.
            self._mode.send(FlightMode.LOITER, observation, now)
            self._report("NavPy hold: GUIDED now NavPy's, LOITER")
        if self._incident.expired():
            if self.ownership.may_request_mode():
                self._enter_critical(observation, now, f"no recovery in {NAV_HOLD_RECOVERY_S:g} s")
                return
            if not self._long_hold_reported:
                # Not NavPy's mode (the mission's AUTO, the pilot's): it flies on safely; NavPy waits fenced.
                self._long_hold_reported = True
                self._report(f"NavPy not recovered in {NAV_HOLD_RECOVERY_S:g} s; mode not NavPy's, commands stay fenced")
        if self._good_since is None or now - self._good_since < NAV_HOLD_TRIGGER_S:
            return
        if self._mode.pending is not None and self.ownership.may_request_mode():
            return   # the hold's LOITER must show, or its deadline gives RTL, before NavPy resumes
        if self.ownership.may_request_mode() and observation.mode is FlightMode.LOITER:
            self.stage = RecoveryStage.RESUMING
            self._mode.send(FlightMode.AUTO, observation, now)
            self._report("NavPy healthy again: AUTO")
        elif self.ownership.state is not Ownership.UNKNOWN:
            # The AUTO handed over in an unhealthy resume was NavPy's: the approach resumes now.
            self._resume_normal("NavPy healthy again: commands reopened", self._handed_over_unhealthy)

    def _resuming(self, observation: ModeObservation, now: float) -> None:
        if self.ownership.ended is not None:
            # Handed over (AUTO shown), or the pilot took the mode meanwhile: either way NavPy goes on.
            handed_over = self.ownership.ended is OwnershipEnd.HANDED_OVER
            # Only NavPy's own AUTO resumes the approach; a mode the operator took stands.
            self._resume_normal("NavPy resumed in AUTO" if handed_over else "NavPy resumed: the mode was taken over",
                                handed_over, healthy=self._bad_since is None)
        elif self._mode.not_confirmed(observation, now):
            self._enter_critical(observation, now, "AUTO not confirmed")

    def _enter_escalated(self) -> None:
        self.hold.holding = True   # before the hand-over's AUTO request
        self._ports.close_fence()
        self.stage = RecoveryStage.ESCALATED
        self._mode.clear()
        self._report("NavPy failed: commands fenced, hand-over to the mission")

    def _escalated(self, observation: ModeObservation, now: float) -> None:
        # NavPy asked for GUIDED and no heartbeat settled it (acquired, lapsed or withdrawn): the UAV may still switch
        # to GUIDED. Like any mode request it must show within MODE_CONFIRM_S.
        guided_unresolved = (self.ownership.guided_pending and self._guided_at is not None
                             and now - self._guided_at < MODE_CONFIRM_S)
        if self.ownership.may_request_mode():
            if self._mode.pending is None:
                self._mode.send(FlightMode.AUTO, observation, now)
            elif self._mode.not_confirmed(observation, now):
                self._enter_critical(observation, now, "AUTO not confirmed")
        elif (self.ownership.state is Ownership.NOT_OWNED and observation.fresh
              and not guided_unresolved):
            self._release("NavPy failed: the mode is not NavPy's")   # handed over, or the pilot's
        # Unknown or stale link, or NavPy's GUIDED request may still show: wait; a fresh heartbeat decides.

    def _critical(self, observation: ModeObservation, now: float) -> None:
        # Also after a critical entered unowned-looking (stale link): once a fresh heartbeat shows NavPy owns the
        # mode, RTL is sent then; a pending one is retried.
        if observation.mode is not FlightMode.RTL and self.ownership.may_request_mode():
            if self._mode.pending is FlightMode.RTL:
                self._mode.retry(observation, now)
            else:
                self._report("NavPy critical: mode owned again, RTL")
                self._mode.send(FlightMode.RTL, observation, now)
        if self._ports.escalation.requested and observation.fresh and (
                observation.mode is FlightMode.RTL or self.ownership.state is Ownership.NOT_OWNED):
            self._release(f"NavPy failed: {observation.mode.name if observation.mode else 'mode'} stands")

    def _release(self, message: str) -> None:
        if not self._ports.escalation.released:
            self._ports.escalation.release()
            self._report(message)

    def _enter_critical(self, observation: ModeObservation, now: float, why: str) -> None:
        self.hold.holding = True   # before the RTL request
        self.stage = RecoveryStage.CRITICAL
        self._mode.clear()
        owned = self.ownership.may_request_mode()
        self._report(f"NavPy critical ({why}): {'RTL' if owned else 'commands stay fenced'}")
        if owned:
            self._mode.send(FlightMode.RTL, observation, now)

    def _resume_normal(self, message: str, resumed: bool = False, *, healthy: bool = True) -> None:
        """Reopen; ``resumed``: approach the kept dock again. Unhealthy, or the fence held by the watchdog
        (MavCommandFence.hold_closed): stay in the hold, and resume from it once healthy."""
        self._mode.clear()
        if not healthy or self._ports.reopen_fence() is False:
            self.stage = RecoveryStage.HOLD
            self._good_since = None
            self._handed_over_unhealthy = resumed
            self._report("NavPy not healthy, or a loop stuck: commands stay fenced")
            return
        self.stage = RecoveryStage.NORMAL
        self._bad_since = None
        self._long_hold_reported = False
        self._handed_over_unhealthy = False
        if resumed:
            self.hold.grant_resume()
        self._report(message)

    def _report(self, message: str) -> None:
        try:
            self._ports.logger.recovery(message, "nav")
        except Exception:  # noqa: BLE001 - reporting must not end recovery
            pass


__all__ = [
    "NAV_HOLD_RECOVERY_S",
    "NAV_HOLD_TRIGGER_S",
    "NavRecovery",
    "RecoveryStage",
    "TimeSample",
]
