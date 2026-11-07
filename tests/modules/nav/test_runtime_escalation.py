"""A latched component failure fences and hands over instead of ending NavPy (docs/design/navpy_recovery.md, step 5):
NavPy exits only once NAV released the UAV and a supervisor restarts it."""
from __future__ import annotations

from argparse import Namespace
from unittest.mock import Mock

import pytest

from navpy.app.nav_recovery_link import runtime_escalation
from navpy.app.runtime_escalation import RuntimeEscalation
from navpy.app.runtime_resources import RuntimeResourceStack, RuntimeSession
from navpy.core.flow.nav_recovery_ports import NavEscalation, NavRecoveryLink


def _escalation(*, supervised: bool) -> tuple[RuntimeEscalation, NavEscalation, Mock]:
    nav = NavEscalation()
    fence = Mock()
    return RuntimeEscalation(close_fence=fence, escalation=nav, supervised=supervised, logger=Mock()), nav, fence


LATCHED = RuntimeError("detector worker died")


def _failing() -> None:
    raise LATCHED   # a latch raises its first failure on every check


def test_a_failure_fences_and_asks_nav_without_ending_navpy():
    escalation, nav, fence = _escalation(supervised=True)
    escalation.check(_failing)
    fence.assert_called_once()
    assert nav.requested
    escalation.check(_failing)   # not released yet: NavPy stays up
    fence.assert_called_once()


def test_a_supervised_navpy_exits_once_nav_released_the_uav():
    escalation, nav, _fence = _escalation(supervised=True)
    escalation.check(_failing)
    nav.release()
    with pytest.raises(RuntimeError, match="detector worker died"):
        escalation.check(Mock())


def test_without_a_supervisor_navpy_stays_up_fenced():
    escalation, nav, _fence = _escalation(supervised=False)
    escalation.check(_failing)
    nav.release()
    for _ in range(3):
        escalation.check(Mock())


def test_the_health_check_is_not_rerun_after_the_first_failure():
    escalation, _nav, _fence = _escalation(supervised=True)
    escalation.check(_failing)
    later = Mock()
    escalation.check(later)
    later.assert_not_called()


def test_an_interrupt_still_ends_the_run_at_once():
    escalation, nav, fence = _escalation(supervised=True)

    def interrupted() -> None:
        raise KeyboardInterrupt

    with pytest.raises(KeyboardInterrupt):
        escalation.check(interrupted)
    fence.assert_not_called()
    assert not nav.requested


def test_without_a_link_there_is_no_escalation():
    assert runtime_escalation(Namespace(supervised=True), None, Mock()) is None
    link = NavRecoveryLink(close_fence=Mock(), reopen_fence=Mock(), observe_mode=Mock())
    assert runtime_escalation(Namespace(supervised=True), link, Mock()) is not None


def test_the_session_keeps_pumping_after_a_failure_until_released_then_reports_it():
    nav = NavEscalation()
    fence = Mock()
    pumps = 0
    stack = RuntimeResourceStack()
    stack.own("cleanup", lambda: None)

    def ui_step() -> bool:
        nonlocal pumps
        pumps += 1
        if pumps == 5:
            nav.release()   # NAV saw AUTO confirmed
        return True

    session = RuntimeSession(
        resources=stack, start_vision=lambda: None, start_controller=lambda: None, health_check=_failing,
        ui_step=ui_step, sleep=lambda _s: None,
        escalation=RuntimeEscalation(close_fence=fence, escalation=nav, supervised=True, logger=Mock()),
    )
    with pytest.raises(RuntimeError, match="detector worker died"):
        session.run()
    assert pumps == 5
    fence.assert_called_once()
