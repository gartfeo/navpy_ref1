"""NAV recovery stages (docs/design/navpy_recovery.md, step 4; owner rulings 2026-11-05 and 2026-11-07).

Short failures change nothing; 1.5 s of failed periods holds (fence, LOITER once if NavPy owns the mode); 1.5 s of
good periods resumes (AUTO, fence reopened, the task kept and the dock approached again); 30 s in the hold or an
unconfirmed mode gives RTL; a mode NavPy does not own is never touched.
"""
from __future__ import annotations

import ast
from pathlib import Path
from unittest.mock import Mock

import pytest

from navpy.app.flow.nav_runtime_composition import _return_to_search
from navpy.core.flow.nav_ownership import ModeObservation, Ownership
from navpy.core.flow.nav_mode_request import MODE_CONFIRM_S
from navpy.core.flow.nav_recovery import (
    NAV_HOLD_RECOVERY_S,
    NAV_HOLD_TRIGGER_S,
    NavRecovery,
    RecoveryStage,
)
from navpy.core.flow.nav_recovery_clock import RecoveryClock, TimeSample
from navpy.core.flow.nav_recovery_ports import NavEscalation, NavRecoveryPorts, RecoveryHold
from navpy.core.flow.nav_state import NavigationTaskState, NavPhaseState, NavState
from navpy.core.flow.navigation_decision import ApproachOutcome, NavDecisionPorts, NavStateDecision
from navpy.core.ports.flight_mode import FlightMode

PERIOD_S = 0.125   # exact in binary; eight periods per 1 Hz heartbeat
HEARTBEAT_PERIODS = 8


class FakeAutopilot:
    """Takes a requested mode and shows it in its next heartbeat (1 Hz), as ArduPlane does."""

    def __init__(self, mode: FlightMode = FlightMode.GUIDED, *, takes: bool = True) -> None:
        self.mode = mode
        self.count = 0
        self.armed = True
        self.fresh = True
        self.takes = takes
        self.requests: list[FlightMode] = []
        self._wanted: FlightMode | None = None

    def request(self, mode: FlightMode) -> None:
        self.requests.append(mode)
        if self.takes:
            self._wanted = mode

    def heartbeat(self) -> None:
        if self._wanted is not None:
            self.mode, self._wanted = self._wanted, None
        self.count += 1

    def observe(self) -> ModeObservation:
        return ModeObservation(self.count, self.mode, self.armed, self.fresh)


class Rig:
    def __init__(self, autopilot: FakeAutopilot, *, owned: bool = True) -> None:
        self.ap = autopilot
        self.fence = Mock()
        self.logger = Mock()
        self.hold = RecoveryHold(autopilot.observe)   # GUIDED requests read the heartbeat at the request
        self.escalation = NavEscalation()
        self.claims: list[bool] = []                  # the claim on NavPy's own LOITER, after each step
        self.recovery = NavRecovery(
            NavRecoveryPorts(
                close_fence=self.fence.close,
                reopen_fence=self.fence.reopen,
                observe_mode=autopilot.observe,
                request_mode=autopilot.request,
                logger=self.logger,
                escalation=self.escalation,
                claim_hold=self.claims.append,
            ),
            self.hold,
        )
        self.t = 100.0
        self.periods = 0
        if owned:   # NavPy asked for GUIDED; the next heartbeat shows it
            self.hold.guided_requested()
            self.run(True, 1.0)
            assert self.recovery.ownership.state is Ownership.OWNED
            autopilot.requests.clear()

    def run(self, cycle_ok: bool, seconds: float) -> None:
        for _ in range(round(seconds / PERIOD_S)):
            self.t += PERIOD_S
            self.periods += 1
            if self.periods % HEARTBEAT_PERIODS == 0:
                self.ap.heartbeat()
            self.recovery.step(cycle_ok, TimeSample(aircraft_s=self.t, wall_s=self.t))

    @property
    def stage(self) -> RecoveryStage:
        return self.recovery.stage


def test_a_short_failure_changes_nothing():
    rig = Rig(FakeAutopilot())
    rig.run(False, NAV_HOLD_TRIGGER_S - 0.3)
    rig.run(True, 0.5)
    rig.run(False, NAV_HOLD_TRIGGER_S - 0.3)
    assert rig.stage is RecoveryStage.NORMAL
    rig.fence.close.assert_not_called()
    assert rig.ap.requests == []
    assert not rig.hold.holding


def test_failures_for_the_trigger_time_hold_with_one_loiter_request():
    rig = Rig(FakeAutopilot())
    rig.run(False, NAV_HOLD_TRIGGER_S + 0.1)
    assert rig.stage is RecoveryStage.HOLD and rig.hold.holding
    rig.fence.close.assert_called_once()
    rig.run(False, 2.0)   # LOITER shows; failures go on
    assert rig.ap.requests == [FlightMode.LOITER]   # once, not per period
    assert rig.ap.mode is FlightMode.LOITER


def test_healthy_again_resumes_in_auto_and_reopens_the_fence():
    rig = Rig(FakeAutopilot())
    rig.run(False, NAV_HOLD_TRIGGER_S + 0.1)
    rig.run(False, 1.0)
    rig.run(True, NAV_HOLD_TRIGGER_S + 0.1)
    assert rig.stage is RecoveryStage.RESUMING and rig.ap.requests[-1] is FlightMode.AUTO
    rig.fence.reopen.assert_not_called()   # not before a newer heartbeat shows AUTO
    rig.run(True, 1.1)
    assert rig.ap.mode is FlightMode.AUTO
    assert rig.stage is RecoveryStage.NORMAL and not rig.hold.holding
    rig.fence.reopen.assert_called_once()
    assert rig.ap.requests == [FlightMode.LOITER, FlightMode.AUTO]


def test_no_recovery_in_the_hold_time_gives_rtl():
    rig = Rig(FakeAutopilot())
    rig.run(False, NAV_HOLD_TRIGGER_S + 0.1)
    rig.run(False, NAV_HOLD_RECOVERY_S + 0.2)
    assert rig.stage is RecoveryStage.CRITICAL
    rig.run(False, 1.1)
    assert rig.ap.mode is FlightMode.RTL
    assert rig.ap.requests == [FlightMode.LOITER, FlightMode.RTL]
    rig.run(True, 5.0)   # not undone automatically
    assert rig.stage is RecoveryStage.CRITICAL and rig.ap.mode is FlightMode.RTL
    rig.fence.reopen.assert_not_called()


def test_loiter_not_confirmed_gives_rtl():
    ap = FakeAutopilot()
    rig = Rig(ap)
    ap.takes = False
    rig.run(False, NAV_HOLD_TRIGGER_S + 0.1)
    rig.run(False, MODE_CONFIRM_S + 0.2)
    assert rig.stage is RecoveryStage.CRITICAL
    assert ap.requests[0] is FlightMode.LOITER and ap.requests[-1] is FlightMode.RTL
    assert ap.requests.count(FlightMode.LOITER) <= MODE_CONFIRM_S + 1   # retried about once a second


def test_a_pilot_mode_is_never_touched():
    rig = Rig(FakeAutopilot())
    rig.run(False, NAV_HOLD_TRIGGER_S + 0.1)
    rig.run(False, 1.0)
    rig.ap.mode = FlightMode.FBWA   # the pilot takes over in the hold
    rig.run(False, NAV_HOLD_RECOVERY_S + 1.0)
    rig.run(True, NAV_HOLD_TRIGGER_S + 0.2)
    assert rig.ap.requests == [FlightMode.LOITER]
    assert rig.ap.mode is FlightMode.FBWA
    assert rig.stage is RecoveryStage.NORMAL   # healthy: commands reopened, no mode change
    rig.fence.reopen.assert_called_once()


def test_search_in_the_missions_auto_is_only_fenced():
    rig = Rig(FakeAutopilot(FlightMode.AUTO), owned=False)
    rig.run(False, NAV_HOLD_TRIGGER_S + 0.1)
    assert rig.stage is RecoveryStage.HOLD
    rig.run(False, NAV_HOLD_RECOVERY_S + 1.0)   # no RTL without ownership: the mission flies on
    assert rig.stage is RecoveryStage.HOLD
    rig.run(True, NAV_HOLD_TRIGGER_S + 0.2)
    assert rig.stage is RecoveryStage.NORMAL
    assert rig.ap.requests == []


def test_stale_link_requests_no_mode():
    rig = Rig(FakeAutopilot())
    rig.ap.fresh = False
    rig.run(False, NAV_HOLD_TRIGGER_S + 0.1)
    rig.run(False, 2.0)
    assert rig.recovery.ownership.state is Ownership.UNKNOWN
    assert rig.ap.requests == []
    rig.run(True, NAV_HOLD_TRIGGER_S + 0.2)
    assert rig.stage is RecoveryStage.HOLD   # owned when last seen: stays fenced until the link decides
    rig.fence.reopen.assert_not_called()


def _clock_readings(*samples: tuple[float | None, float]) -> list[float]:
    clock = RecoveryClock()
    return [clock.advance(TimeSample(aircraft_s=a, wall_s=w)) for a, w in samples]


def test_the_clock_uses_aircraft_time_and_falls_back_to_wall_time():
    assert _clock_readings((10.0, 1000.0), (12.0, 1000.2)) == pytest.approx([0.0, 2.0])   # 10x sim
    assert _clock_readings((10.0, 1000.0), (10.0, 1003.0)) == pytest.approx([0.0, 3.0])   # link stalled
    assert _clock_readings((None, 5.0), (None, 6.5)) == pytest.approx([0.0, 1.5])


def test_the_clock_keeps_moving_when_aircraft_time_freezes_after_advancing():
    """Codex finding 1 on steps 1-4: aircraft time 100 -> 101, then frozen; wall time goes on."""
    readings = _clock_readings((100.0, 0.0), (101.0, 1.0), (101.0, 2.0), (101.0, 3.0))
    assert readings == pytest.approx([0.0, 1.0, 2.0, 3.0])


def test_the_clock_counts_a_stall_once_and_never_goes_back():
    readings = _clock_readings(
        (100.0, 0.0), (100.0, 2.0),   # stalled for 2 s of wall time
        (102.5, 2.1),                 # the link is back: aircraft time moved 2.5 s in all
        (5.0, 3.1),                   # autopilot reboot: aircraft time restarts
        (6.0, 3.2),
    )
    assert readings == pytest.approx([0.0, 2.0, 2.5, 3.5, 4.5])


def test_a_link_that_freezes_after_advancing_still_holds_and_gives_rtl():
    """Codex finding 1: with aircraft time frozen after one step, the hold and its deadline still come."""
    ap = FakeAutopilot()
    rig = Rig(ap)
    frozen = rig.t + 2 * PERIOD_S   # the timer starts; aircraft time moves once more, then stops
    for _ in range(round((NAV_HOLD_TRIGGER_S + NAV_HOLD_RECOVERY_S + 1.0) / PERIOD_S)):
        rig.t += PERIOD_S
        rig.periods += 1
        if rig.periods % HEARTBEAT_PERIODS == 0:
            ap.heartbeat()
        rig.recovery.step(False, TimeSample(aircraft_s=min(rig.t, frozen), wall_s=rig.t))
    assert rig.stage is RecoveryStage.CRITICAL
    assert ap.requests == [FlightMode.LOITER, FlightMode.RTL]


def test_a_stalled_link_still_holds_by_wall_time():
    ap = FakeAutopilot()
    rig = Rig(ap)
    frozen = rig.t
    for _ in range(round((NAV_HOLD_TRIGGER_S + 0.2) / PERIOD_S)):
        rig.t += PERIOD_S
        rig.recovery.step(False, TimeSample(aircraft_s=frozen, wall_s=rig.t))
    assert rig.stage is RecoveryStage.HOLD


def test_critical_sends_rtl_once_a_stale_link_shows_the_mode_owned_again():
    """Codex finding 3 on steps 1-4: LOITER unconfirmed while the link is stale gives critical with no RTL; the
    next fresh heartbeat shows NavPy still owns GUIDED, and RTL goes then."""
    ap = FakeAutopilot()
    rig = Rig(ap)
    ap.takes = False
    rig.run(False, NAV_HOLD_TRIGGER_S + 0.1)
    assert ap.requests == [FlightMode.LOITER]
    ap.fresh = False
    rig.run(False, MODE_CONFIRM_S + 0.2)
    assert rig.stage is RecoveryStage.CRITICAL and FlightMode.RTL not in ap.requests
    ap.fresh, ap.takes = True, True
    rig.run(False, 2.1)
    assert ap.requests[-1] is FlightMode.RTL and ap.mode is FlightMode.RTL
    rig.run(False, 3.0)
    assert ap.requests.count(FlightMode.RTL) == 1   # confirmed: not repeated
    rig.fence.reopen.assert_not_called()


def _until_normal(rig: Rig) -> None:
    while rig.stage is not RecoveryStage.NORMAL:
        assert not rig.hold.resumed
        rig.run(True, PERIOD_S)


def test_the_resume_grant_is_kept_until_the_decision_uses_it():
    rig = Rig(FakeAutopilot())
    rig.run(False, NAV_HOLD_TRIGGER_S + 0.1)
    rig.run(True, NAV_HOLD_TRIGGER_S + 0.1)
    _until_normal(rig)
    assert rig.hold.resumed and not rig.hold.holding
    rig.run(True, 1.0)
    assert rig.hold.resumed
    assert rig.hold.consume_resume() and not rig.hold.consume_resume()   # used once


def test_a_failed_cycle_right_after_the_resume_does_not_lose_it():
    """Codex finding 2 on step 5a: the cycle after the resume raises before deciding; the next decision must still
    see the resume, or it reads NavPy's own AUTO as GUIDED lost and clears the task."""
    rig = Rig(FakeAutopilot())
    rig.run(False, NAV_HOLD_TRIGGER_S + 0.1)
    rig.run(True, NAV_HOLD_TRIGGER_S + 0.1)
    _until_normal(rig)
    rig.run(False, PERIOD_S)   # the next cycle fails in sensing, before its decision
    assert rig.hold.resumed
    decision, ports, task = _decision(holding=False, hold=rig.hold, held=True)
    assert decision._await_guided(FlightMode.AUTO) is ApproachOutcome.WAITING
    ports.clear_navigation_task.assert_not_called()
    ports.start_fresh_episode.assert_called_once()     # a fresh episode asks for GUIDED (owner ruling 2026-11-09)
    assert not rig.hold.resumed                         # the grant is used


def test_an_operators_mode_after_the_resume_is_not_resumed_over():
    """Codex round 2, finding 1: NavPy's AUTO is confirmed; sensing fails before the resume is taken; the operator
    selects LOITER. The next decision must not ask for GUIDED over the operator's LOITER."""
    rig = Rig(FakeAutopilot())
    rig.run(False, NAV_HOLD_TRIGGER_S + 0.1)
    rig.run(True, NAV_HOLD_TRIGGER_S + 0.1)
    _until_normal(rig)
    rig.run(False, PERIOD_S)            # sensing fails before the decision
    rig.ap.mode = FlightMode.LOITER     # the operator's LOITER
    decision, ports, _task = _decision(holding=False, hold=rig.hold, held=True)
    assert decision._await_guided(FlightMode.LOITER) is ApproachOutcome.ABANDONED
    ports.request_guided.assert_not_called()
    ports.start_fresh_episode.assert_not_called()
    ports.clear_navigation_task.assert_called_once()
    rig.ap.heartbeat()
    rig.run(True, PERIOD_S)
    assert not rig.hold.resumed         # recovery voids the grant for the operator's mode


def test_a_new_hold_voids_a_resume_not_yet_taken():
    rig = Rig(FakeAutopilot())
    rig.run(False, NAV_HOLD_TRIGGER_S + 0.1)
    rig.run(True, NAV_HOLD_TRIGGER_S + 0.1)
    _until_normal(rig)
    rig.run(False, NAV_HOLD_TRIGGER_S + 0.1)
    assert rig.stage is RecoveryStage.HOLD and not rig.hold.resumed


def test_an_operators_auto_in_the_hold_is_not_resumed_over():
    """Codex finding 1 on step 5a: the operator selects AUTO in the hold, before recovery asks for it; when healthy
    the fence reopens, but the approach must not take GUIDED back from the operator."""
    rig = Rig(FakeAutopilot())
    rig.run(False, NAV_HOLD_TRIGGER_S + 0.1)
    rig.run(False, 1.0)                 # NavPy's LOITER shows
    rig.ap.mode = FlightMode.AUTO       # the operator's AUTO
    rig.run(False, 1.0)
    assert rig.recovery.ownership.state is Ownership.NOT_OWNED
    rig.run(True, NAV_HOLD_TRIGGER_S)
    _until_normal(rig)
    assert not rig.hold.resumed
    assert rig.ap.requests == [FlightMode.LOITER]
    decision, ports, _task = _decision(holding=False, hold=rig.hold, held=True)
    assert decision._await_guided(FlightMode.AUTO) is ApproachOutcome.ABANDONED
    ports.request_guided.assert_not_called()
    ports.start_fresh_episode.assert_not_called()


def test_an_operators_mode_in_the_resume_is_not_resumed_over():
    rig = Rig(FakeAutopilot())
    rig.run(False, NAV_HOLD_TRIGGER_S + 0.1)
    rig.run(False, 1.0)
    rig.ap.takes = False
    rig.run(True, NAV_HOLD_TRIGGER_S + 0.1)
    assert rig.stage is RecoveryStage.RESUMING
    rig.ap.mode = FlightMode.FBWA       # the operator takes over before NavPy's AUTO shows
    rig.run(True, 1.0)
    assert rig.stage is RecoveryStage.NORMAL and not rig.hold.resumed


def test_a_guided_request_is_recorded_from_the_heartbeat_before_it_was_sent():
    """Codex finding 3 on step 5a: NavPy asks for GUIDED from AUTO; the operator's FBWA heartbeat arrives before
    recovery's step. The request lapses, so a later operator GUIDED is not NavPy's and the hold changes no mode."""
    ap = FakeAutopilot(FlightMode.AUTO)
    rig = Rig(ap, owned=False)
    rig.run(True, 1.0)
    rig.hold.guided_requested()         # sent in this period's cycle, from AUTO
    ap.mode = FlightMode.FBWA
    ap.heartbeat()                      # the operator's FBWA, before recovery's step
    rig.run(True, PERIOD_S)
    ap.mode = FlightMode.GUIDED         # the operator's GUIDED
    rig.run(True, 1.1)
    assert rig.recovery.ownership.state is Ownership.NOT_OWNED
    rig.run(False, NAV_HOLD_TRIGGER_S + 1.5)
    assert rig.stage is RecoveryStage.HOLD and ap.requests == []


def test_a_guided_request_is_acquired_on_the_next_guided_heartbeat():
    ap = FakeAutopilot(FlightMode.AUTO)
    rig = Rig(ap, owned=False)
    rig.run(True, 1.0)
    rig.hold.guided_requested()
    ap.mode = FlightMode.GUIDED
    ap.heartbeat()                      # NavPy's GUIDED shows before recovery's step
    rig.run(True, PERIOD_S)
    assert rig.recovery.ownership.state is Ownership.OWNED


def test_a_mode_change_before_the_guided_request_does_not_lapse_it():
    """Codex round 2, finding 2: the last step saw AUTO; the operator's LOITER heartbeat arrives before NavPy sends
    GUIDED (from LOITER). The request is NavPy's: GUIDED shown later is owned, even while cycles fail, so a hold
    requests LOITER under ownership instead of only fencing."""
    ap = FakeAutopilot(FlightMode.AUTO)
    rig = Rig(ap, owned=False)
    rig.run(True, 1.0)                  # recovery's last step saw AUTO
    ap.mode = FlightMode.LOITER
    ap.heartbeat()                      # the operator's LOITER, before NavPy's send
    rig.hold.guided_requested()         # NavPy sends GUIDED from LOITER
    ap.request(FlightMode.GUIDED)
    ap.requests.clear()
    rig.run(False, PERIOD_S)
    assert rig.recovery.ownership.guided_pending
    rig.run(False, 1.1)                 # sensing fails; GUIDED shows
    assert ap.mode is FlightMode.GUIDED
    assert rig.recovery.ownership.state is Ownership.OWNED
    rig.run(False, NAV_HOLD_TRIGGER_S)
    assert rig.stage is RecoveryStage.HOLD and ap.requests == [FlightMode.LOITER]


def test_a_guided_request_behind_the_fence_is_not_navpys():
    ap = FakeAutopilot(FlightMode.AUTO)
    rig = Rig(ap, owned=False)
    rig.run(False, NAV_HOLD_TRIGGER_S + 0.1)
    assert rig.stage is RecoveryStage.HOLD
    rig.hold.guided_requested()         # dropped by the closed fence
    rig.run(False, PERIOD_S)
    assert not rig.recovery.ownership.guided_pending


def _guided_sent(ap: FakeAutopilot, rig: Rig) -> None:
    rig.run(True, PERIOD_S)
    rig.hold.guided_requested()
    ap.request(FlightMode.GUIDED)       # NavPy's GUIDED, sent before the escalation
    ap.requests.clear()
    rig.run(True, PERIOD_S)


def test_escalation_does_not_release_on_a_stale_link_with_guided_pending():
    """Codex finding 4 on step 5a: GUIDED sent, not yet shown; the link goes stale and a failure escalates. NavPy
    must not exit while the UAV may already be in GUIDED: a fresh heartbeat decides, then AUTO is handed over."""
    ap = FakeAutopilot(FlightMode.AUTO)
    rig = Rig(ap, owned=False)
    _guided_sent(ap, rig)
    ap.fresh = False
    rig.escalation.request()
    rig.run(True, MODE_CONFIRM_S + 1.0)
    assert not rig.escalation.released
    ap.fresh = True
    rig.run(True, 2.5)
    assert ap.requests == [FlightMode.AUTO] and ap.mode is FlightMode.AUTO and rig.escalation.released


def test_escalation_waits_for_a_pending_guided_to_show():
    ap = FakeAutopilot(FlightMode.AUTO)
    rig = Rig(ap, owned=False)
    _guided_sent(ap, rig)
    rig.escalation.request()
    rig.run(True, PERIOD_S)
    assert not rig.escalation.released
    rig.run(True, 2.5)
    assert ap.requests == [FlightMode.AUTO] and rig.escalation.released


def test_escalation_releases_once_a_pending_guided_did_not_show_in_time():
    ap = FakeAutopilot(FlightMode.AUTO, takes=False)
    rig = Rig(ap, owned=False)
    _guided_sent(ap, rig)
    rig.escalation.request()
    rig.run(True, MODE_CONFIRM_S - 0.5)
    assert not rig.escalation.released
    rig.run(True, 1.0)
    assert rig.escalation.released and ap.requests == [] and ap.mode is FlightMode.AUTO


def test_escalation_hands_an_owned_uav_to_auto_then_releases():
    rig = Rig(FakeAutopilot())
    rig.escalation.request()
    rig.run(True, PERIOD_S)
    assert rig.stage is RecoveryStage.ESCALATED and rig.hold.holding
    rig.fence.close.assert_called_once()
    assert rig.ap.requests == [FlightMode.AUTO] and not rig.escalation.released
    rig.run(True, 1.0)
    assert rig.ap.mode is FlightMode.AUTO and rig.escalation.released
    rig.run(True, 5.0)   # never back to normal: the failure is latched
    assert rig.stage is RecoveryStage.ESCALATED
    rig.fence.reopen.assert_not_called()
    assert rig.ap.requests == [FlightMode.AUTO]


def test_escalation_in_the_missions_auto_releases_at_once():
    rig = Rig(FakeAutopilot(FlightMode.AUTO), owned=False)
    rig.escalation.request()
    rig.run(True, PERIOD_S)
    assert rig.escalation.released and rig.ap.requests == []
    rig.fence.close.assert_called_once()


def test_escalation_without_auto_confirmed_releases_in_rtl():
    ap = FakeAutopilot()
    rig = Rig(ap)
    ap.takes = False
    rig.escalation.request()
    rig.run(True, MODE_CONFIRM_S + 0.25)
    assert rig.stage is RecoveryStage.CRITICAL and ap.requests[-1] is FlightMode.RTL
    assert not rig.escalation.released
    ap.takes = True
    ap.request(FlightMode.RTL)
    rig.run(True, 1.0)
    assert ap.mode is FlightMode.RTL and rig.escalation.released


def test_escalation_waits_for_a_fresh_link():
    rig = Rig(FakeAutopilot())
    rig.ap.fresh = False
    rig.escalation.request()
    rig.run(True, 3.0)
    assert rig.ap.requests == [] and not rig.escalation.released
    rig.ap.fresh = True
    rig.run(True, 2.0)
    assert rig.ap.requests == [FlightMode.AUTO] and rig.escalation.released


def test_escalation_in_the_hold_replaces_the_resume():
    rig = Rig(FakeAutopilot())
    rig.run(False, NAV_HOLD_TRIGGER_S + 0.25)
    rig.run(False, 1.0)   # LOITER
    rig.escalation.request()
    rig.run(True, NAV_HOLD_TRIGGER_S + 1.5)
    assert rig.stage is RecoveryStage.ESCALATED and rig.escalation.released
    assert rig.ap.requests == [FlightMode.LOITER, FlightMode.AUTO]
    rig.fence.reopen.assert_not_called()


def _decision(
    holding: bool, granted: bool = False, *, hold: RecoveryHold | None = None, held: bool = False,
) -> tuple[NavStateDecision, NavDecisionPorts, NavigationTaskState]:
    """``hold``: recovery's resume grant from a real RecoveryHold; else ``granted``, used up by the decision."""
    task = NavigationTaskState(nav_mode_observed=True)
    grant = [granted]

    def consume() -> bool:
        value, grant[0] = grant[0], False
        return value

    ports = NavDecisionPorts(
        clock_s=lambda: 0.0,
        clear_navigation_task=Mock(),
        request_guided=Mock(),
        final_approach_active=lambda: True,
        holding=lambda: holding,
        resume_granted=(lambda: hold.resumed) if hold is not None else (lambda: grant[0]),
        consume_resume=hold.consume_resume if hold is not None else consume,
        end_held_episode=Mock(),
        held_episode_ended=lambda: held,
        start_fresh_episode=Mock(),
    )
    phase = NavPhaseState(current=NavState.NAV, previous=NavState.NAV)
    return NavStateDecision(ports, phase, task, Mock(), Mock(), Mock()), ports, task


def test_the_holds_loiter_does_not_end_the_approach():
    decision, ports, task = _decision(holding=True)
    assert decision._await_guided(FlightMode.LOITER) is ApproachOutcome.WAITING
    ports.clear_navigation_task.assert_not_called()
    assert task.nav_mode_observed


def test_the_resumes_auto_approaches_the_kept_dock_again():
    """Codex finding 4 on steps 1-4: AUTO right after the hold keeps the task and approaches the same dock; since
    the owner ruling of 2026-11-09 with a fresh episode, whose NAV entry asks for GUIDED again."""
    decision, ports, task = _decision(holding=False, granted=True, held=True)
    assert decision._await_guided(FlightMode.AUTO) is ApproachOutcome.WAITING
    ports.clear_navigation_task.assert_not_called()
    ports.start_fresh_episode.assert_called_once()
    assert not ports.resume_granted()                  # used up


def test_a_resume_grant_with_nothing_held_authorizes_nothing():
    """A hold that was not mid-approach: the grant is discarded, never restarting a later approach."""
    decision, ports, task = _decision(holding=False, granted=True, held=False)
    assert decision._await_guided(FlightMode.GUIDED) is ApproachOutcome.NONE
    ports.start_fresh_episode.assert_not_called()
    assert not ports.resume_granted()


def test_outside_a_resume_a_non_guided_mode_ends_the_approach_as_before():
    decision, ports, _task = _decision(holding=False)
    assert decision._await_guided(FlightMode.AUTO) is ApproachOutcome.ABANDONED
    ports.clear_navigation_task.assert_called_once()


@pytest.mark.parametrize("holding, restarted", [(True, False), (False, True)])
def test_search_is_not_restarted_in_the_hold(holding, restarted):
    vehicle = Mock()
    state = Mock()
    state.recovery_hold.holding = holding
    _return_to_search(vehicle, state)
    assert vehicle.restart_mission.called is restarted


def test_the_approach_command_path_gains_no_recovery_input():
    """Pure Vision Approach Constraint: recovery reads mode and heartbeats; the approach law and command path
    under core/approach import none of it."""
    root = Path(__file__).resolve().parents[3] / "src" / "navpy" / "core" / "approach"
    offenders = []
    for path in root.rglob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.module in (
                "navpy.core.flow.nav_recovery", "navpy.core.flow.nav_recovery_ports", "navpy.core.flow.nav_mode_request", "navpy.core.flow.nav_ownership",
            ):
                offenders.append(str(path))
    assert offenders == []


def test_the_state_reads_the_links_heartbeat_at_navpys_guided_request():
    """The composed hold reads the link at the request boundary; without a link it records nothing."""
    from navpy.app.flow.nav_foundation_composition import compose_state_ownership

    seen = ModeObservation(7, FlightMode.LOITER, True, True)
    hold = compose_state_ownership(None, None, lambda: seen).recovery_hold
    hold.guided_requested()
    assert hold.take_guided_request() == seen
    unlinked = compose_state_ownership(None).recovery_hold
    unlinked.guided_requested()
    assert unlinked.take_guided_request() is None


def test_recovery_reports_reach_the_operator_as_status_texts():
    """Codex review of steps 5b-5c, finding 6: a destination without ``status`` writes only the log. Step 8: each
    report is also journaled, as source "nav"."""
    from dataclasses import replace

    from tests.logger.recording_logger import RecordingLogger

    rig = Rig(FakeAutopilot())
    logger = RecordingLogger()
    rig.recovery._ports = replace(rig.recovery._ports, logger=logger)
    rig.run(False, NAV_HOLD_TRIGGER_S + 0.1)
    assert rig.stage is RecoveryStage.HOLD
    assert logger.warnings and all(w["status"] == w["msg"] for w in logger.warnings)
    assert logger.journal.records == [("nav", w["msg"]) for w in logger.warnings]


# Step 10, item 3 (owner ruling 2026-11-09): after a failed one-shot dock recovery asks for RTL, only while NavPy owns
# the mode (nav_sortie_return.py).


def test_item3_a_failed_dock_gives_rtl_through_ownership_and_it_is_kept():
    rig = Rig(FakeAutopilot())
    rig.hold.request_sortie_rtl()
    rig.run(True, 2.0)
    assert rig.ap.requests == [FlightMode.RTL] and rig.ap.mode is FlightMode.RTL
    assert not rig.hold.sortie_rtl and rig.stage is RecoveryStage.NORMAL
    rig.run(False, NAV_HOLD_TRIGGER_S + 1.0)                   # a later NavPy hold never undoes the RTL
    assert rig.stage is RecoveryStage.HOLD
    assert rig.ap.requests == [FlightMode.RTL] and rig.ap.mode is FlightMode.RTL


def test_item3_the_rtl_is_repeated_until_a_heartbeat_shows_it():
    ap = FakeAutopilot(takes=False)
    rig = Rig(ap)
    rig.hold.request_sortie_rtl()
    rig.run(True, 3.5)
    assert len(ap.requests) >= 3 and set(ap.requests) == {FlightMode.RTL}   # every MODE_RETRY_S, never given up
    assert rig.hold.sortie_rtl
    ap.takes = True
    rig.run(True, 3.0)
    assert ap.mode is FlightMode.RTL and not rig.hold.sortie_rtl


def test_item3_an_operators_mode_stands_no_rtl():
    ap = FakeAutopilot()
    rig = Rig(ap)
    ap.mode = FlightMode.FBWA                                  # the operator took the mode
    rig.run(True, 2.0)
    rig.hold.request_sortie_rtl()
    rig.run(True, 2.0)
    assert FlightMode.RTL not in ap.requests and ap.mode is FlightMode.FBWA
    assert not rig.hold.sortie_rtl


def test_item3_a_stale_link_waits_for_a_fresh_heartbeat():
    ap = FakeAutopilot()
    rig = Rig(ap)
    ap.fresh = False
    rig.hold.request_sortie_rtl()
    rig.run(True, 2.0)
    assert ap.requests == [] and rig.hold.sortie_rtl
    ap.fresh = True
    rig.run(True, 2.0)
    assert ap.requests == [FlightMode.RTL] and ap.mode is FlightMode.RTL


# Step 10, item 9 (owner ruling 2026-11-09, "Yes, guard it"): while recovery holds in a LOITER NavPy owns, it claims
# it, so the autopilot's watchdog (nav_watchdog.lua) guards it if NavPy dies. Never a LOITER that is not NavPy's.


def test_item9_navpys_own_hold_loiter_is_claimed_once_it_shows():
    rig = Rig(FakeAutopilot())
    rig.run(False, NAV_HOLD_TRIGGER_S + 0.1)
    assert rig.stage is RecoveryStage.HOLD and rig.ap.mode is FlightMode.GUIDED
    assert not any(rig.claims)                                 # not before the LOITER shows
    rig.run(False, 1.0)
    assert rig.ap.mode is FlightMode.LOITER and rig.claims[-1] is True


def test_item9_the_claim_ends_when_navpy_resumes_in_auto():
    rig = Rig(FakeAutopilot())
    rig.run(False, NAV_HOLD_TRIGGER_S + 1.1)
    assert rig.claims[-1] is True
    rig.run(True, NAV_HOLD_TRIGGER_S + 1.2)
    assert rig.stage is RecoveryStage.NORMAL and rig.ap.mode is FlightMode.AUTO
    assert rig.claims[-1] is False


def test_item9_a_pilot_mode_in_the_hold_ends_the_claim_and_a_pilot_loiter_is_not_claimed():
    ap = FakeAutopilot()
    rig = Rig(ap)
    rig.run(False, NAV_HOLD_TRIGGER_S + 1.1)
    assert rig.claims[-1] is True
    ap.mode = FlightMode.FBWA                                  # the pilot takes the mode
    rig.run(False, 1.0)
    assert rig.claims[-1] is False
    ap.mode = FlightMode.LOITER                                # and goes to LOITER: the pilot's
    rig.run(False, 2.0)
    assert rig.stage is RecoveryStage.HOLD and not any(rig.claims[-16:])


def test_item9_a_hold_in_a_loiter_navpy_never_owned_is_not_claimed():
    ap = FakeAutopilot(FlightMode.LOITER)
    rig = Rig(ap, owned=False)
    rig.run(False, NAV_HOLD_TRIGGER_S + 2.0)
    assert rig.stage is RecoveryStage.HOLD and ap.requests == [] and not any(rig.claims)


def test_item9_a_stale_link_keeps_the_claim():
    """Owned when last seen (UNKNOWN): NavPy still holds its LOITER, so the guard stays claimed while it is alive."""
    ap = FakeAutopilot()
    rig = Rig(ap)
    rig.run(False, NAV_HOLD_TRIGGER_S + 1.1)
    ap.fresh = False
    rig.run(False, 1.0)
    assert rig.recovery.ownership.state is Ownership.UNKNOWN and rig.claims[-1] is True
