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
from navpy.core.flow.nav_recovery import (
    MODE_CONFIRM_S,
    NAV_HOLD_RECOVERY_S,
    NAV_HOLD_TRIGGER_S,
    NavRecovery,
    NavRecoveryPorts,
    RecoveryHold,
    RecoveryStage,
    TimeSample,
)
from navpy.core.flow.nav_recovery_clock import RecoveryClock
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
        self.hold = RecoveryHold()
        self.recovery = NavRecovery(
            NavRecoveryPorts(
                close_fence=self.fence.close,
                reopen_fence=self.fence.reopen,
                observe_mode=autopilot.observe,
                request_mode=autopilot.request,
                logger=self.logger,
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


def test_the_resume_is_told_for_one_period():
    rig = Rig(FakeAutopilot())
    rig.run(False, NAV_HOLD_TRIGGER_S + 0.1)
    rig.run(True, NAV_HOLD_TRIGGER_S + 0.1)
    while rig.stage is not RecoveryStage.NORMAL:
        assert not rig.hold.resumed
        rig.run(True, PERIOD_S)
    assert rig.hold.resumed and not rig.hold.holding
    rig.run(True, PERIOD_S)
    assert not rig.hold.resumed


def _decision(holding: bool, resumed: bool = False) -> tuple[NavStateDecision, NavDecisionPorts, NavigationTaskState]:
    task = NavigationTaskState(nav_mode_observed=True)
    ports = NavDecisionPorts(
        clock_s=lambda: 0.0,
        clear_navigation_task=Mock(),
        request_guided=Mock(),
        final_approach_active=lambda: True,
        holding=lambda: holding,
        resumed=lambda: resumed,
    )
    phase = NavPhaseState(current=NavState.NAV, previous=NavState.NAV)
    return NavStateDecision(ports, phase, task, Mock(), Mock(), Mock()), ports, task


def test_the_holds_loiter_does_not_end_the_approach():
    decision, ports, task = _decision(holding=True)
    assert decision._await_guided(FlightMode.LOITER) is ApproachOutcome.WAITING
    ports.clear_navigation_task.assert_not_called()
    assert task.nav_mode_observed


def test_the_resumes_auto_approaches_the_kept_dock_again():
    """Codex finding 4 on steps 1-4: AUTO right after the hold keeps the task and asks for GUIDED again."""
    decision, ports, task = _decision(holding=False, resumed=True)
    assert decision._await_guided(FlightMode.AUTO) is ApproachOutcome.WAITING
    ports.clear_navigation_task.assert_not_called()
    ports.request_guided.assert_called_once()
    assert not task.nav_mode_observed   # waiting for GUIDED again, under the usual accept timeout


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
                "navpy.core.flow.nav_recovery", "navpy.core.flow.nav_ownership",
            ):
                offenders.append(str(path))
    assert offenders == []
