"""Nav publishes whether it flies a final approach, the swarm's BUSY input."""

from unittest.mock import Mock, call

import pytest

from navpy.modules.nav.nav_network import NavNetworkRuntime
from navpy.modules.nav.nav_state import NavPhaseState, NavState
from navpy.modules.nav.nav_task_availability import is_approaching
from navpy.modules.nav.navigation_loop import StateActionDispatcher
from navpy.modules.nav.navigation_transitions import TransitionOutcome


@pytest.mark.parametrize("phase", list(NavState))
def test_only_a_committed_nav_phase_is_a_final_approach(phase):
    assert is_approaching(phase) is (phase is NavState.NAV)


@pytest.fixture
def actions():
    return {state: Mock(name=state.name) for state in NavState}


def test_committed_phase_is_published_every_cycle_before_its_action(actions):
    order = []
    publish = Mock(side_effect=lambda phase: order.append(("publish", phase)))
    actions[NavState.NAV].side_effect = lambda: order.append(("act", NavState.NAV))
    phase = NavPhaseState(current=NavState.NAV, previous=NavState.CONFIRM)
    transitions = Mock()
    transitions.on_change.return_value = TransitionOutcome(NavState.NAV)
    dispatcher = StateActionDispatcher(phase, transitions, actions, publish)

    dispatcher.act()
    dispatcher.act()

    assert order == [
        ("publish", NavState.NAV),
        ("act", NavState.NAV),
        ("publish", NavState.NAV),
        ("act", NavState.NAV),
    ]


def test_a_redirected_transition_publishes_the_effective_phase(actions):
    publish = Mock()
    phase = NavPhaseState(current=NavState.RESET, previous=NavState.NAV)
    transitions = Mock()
    transitions.on_change.return_value = TransitionOutcome(NavState.ONHOLD)

    StateActionDispatcher(phase, transitions, actions, publish).act()

    publish.assert_called_once_with(NavState.ONHOLD)


def test_a_raising_transition_publishes_nothing(actions):
    publish = Mock()
    phase = NavPhaseState(current=NavState.NAV, previous=NavState.CONFIRM)
    transitions = Mock()
    transitions.on_change.side_effect = OSError("transition failed")

    with pytest.raises(OSError):
        StateActionDispatcher(phase, transitions, actions, publish).act()

    publish.assert_not_called()


def _runtime() -> NavNetworkRuntime:
    return NavNetworkRuntime(Mock(), Mock(), Mock(), Mock(), Mock(), Mock())


def test_runtime_forwards_only_changes_to_the_task_actor():
    runtime = _runtime()
    actor = Mock()
    runtime.task_actor = actor

    for approaching in (False, True, True, False, False):
        runtime.publish_approaching(approaching)

    assert actor.set_approaching.call_args_list == [
        call(False), call(True), call(False),
    ]


def test_a_replacement_actor_learns_the_current_value():
    runtime = _runtime()
    runtime.task_actor = Mock()
    runtime.publish_approaching(True)
    replacement = Mock()
    runtime.task_actor = replacement

    runtime.publish_approaching(True)

    replacement.set_approaching.assert_called_once_with(True)


def test_without_a_task_actor_nothing_is_published():
    runtime = _runtime()

    runtime.publish_approaching(True)

    actor = Mock()
    runtime.task_actor = actor
    runtime.publish_approaching(True)
    actor.set_approaching.assert_called_once_with(True)
