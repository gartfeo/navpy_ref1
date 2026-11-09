"""Focused ordering and failure semantics for NAV entry/exit owners."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from navpy.core.approach.nav_exit_cleanup import (
    NavCleanupResult,
    NavExitCleanup,
    NavExitCleanupPorts,
)
from navpy.products.delivery.nav_oneshot_completion import (
    OneShotCompletion,
    OneShotCompletionPorts,
)
from navpy.core.approach.vision_nav.episode_latch import FinalApproachEpisodeTerminal, FinalApproachTerminal
from navpy.core.flow.nav_transition import NavExitOutcome, NavTransition
from navpy.core.ports.dock_contact import DockContactIdentity
from navpy.core.flow.nav_state import NavigationTaskState, NavState


def test_exit_cleanup_fences_navigation_before_every_other_cleanup():
    events = []
    snap = object()
    logger = Mock()
    logger.defer_status_texts.side_effect = (
        lambda enable, *, flush: events.append(("status", enable, flush))
    )
    speedup = Mock()
    speedup.restore.side_effect = (
        lambda *, log: events.append(("speedup", log)) or True
    )
    cleanup = NavExitCleanup(
        NavExitCleanupPorts(
            disarm_dock_contact=lambda: events.append("disarm"),
            close_final_approach_source=lambda: events.append("source"),
            navigation_reset=lambda: events.append("fence") or snap,
            detector_stop=lambda: events.append("detector"),
        ),
        speedup,
        logger,
    )

    result = cleanup.run()

    assert result.snap is snap
    assert result.errors == ()
    assert events == [
        "disarm",
        "source",
        "fence",
        "detector",
        ("status", False, True),
        ("speedup", True),
    ]


def test_exit_cleanup_source_failure_skips_command_reset_but_cleans_independent():
    calls = []

    def fail(label):
        calls.append(label)
        raise RuntimeError(label)

    logger = Mock()
    logger.defer_status_texts.side_effect = lambda *args, **kwargs: fail("status")
    speedup = Mock()
    speedup.restore.side_effect = lambda **kwargs: fail("speedup")
    cleanup = NavExitCleanup(
        NavExitCleanupPorts(
            disarm_dock_contact=lambda: fail("disarm"),
            close_final_approach_source=lambda: fail("source"),
            navigation_reset=lambda: fail("fence"),
            detector_stop=lambda: fail("detector"),
        ),
        speedup,
        logger,
    )

    result = cleanup.run()

    assert calls == ["disarm", "source", "detector", "status", "speedup"]
    assert [str(error) for error in result.errors] == calls


def _transition(
    *,
    navigation_active=True,
    completed=True,
    oneshot_enabled=True,
    events=None,
):
    events = [] if events is None else events
    navigation_task = NavigationTaskState(
        final_approach_navigation_active=navigation_active,
        final_approach_nav_completed=completed,
    )
    snap = object()
    cleanup = Mock()
    cleanup.run.side_effect = (
        lambda: events.append("cleanup")
        or NavCleanupResult(snap=snap, errors=())
    )
    reporter = Mock()
    reporter.report.side_effect = lambda value: events.append(("snap", value))
    oneshot = Mock()
    oneshot.run.side_effect = lambda: events.append("oneshot") or ()
    transition = NavTransition(
        Mock(),
        cleanup,
        reporter,
        oneshot.run,
        navigation_task,
        lambda: oneshot_enabled,
    )
    return transition, reporter, oneshot, snap, events


def test_snap_is_reported_before_oneshot_completion_side_effects():
    transition, _, _, snap, events = _transition()

    outcome = transition.exit()

    assert events == ["cleanup", ("snap", snap), "oneshot"]
    assert outcome.redirect is NavState.ONHOLD
    assert outcome.oneshot_completed


@pytest.mark.parametrize("completed", [False])
def test_oneshot_runs_only_after_final_approach_nav_completion(completed):
    transition, _, oneshot, _, _ = _transition(completed=completed)

    outcome = transition.exit()

    oneshot.run.assert_not_called()
    assert outcome.redirect is None
    assert not outcome.oneshot_completed


def test_inactive_navigation_exit_does_not_publish_meaningless_snap():
    transition, reporter, _, _, _ = _transition(
        navigation_active=False,
        completed=False,
        oneshot_enabled=False,
    )

    transition.exit()

    reporter.report.assert_not_called()


def test_real_autopilot_one_shot_never_disarms():
    disarm = Mock()
    reset = Mock()
    logger = Mock()
    completion = OneShotCompletion(
        OneShotCompletionPorts(
            is_simulated_autopilot=lambda: False,
            is_stopping=lambda: False,
            disarm=disarm,
            return_to_hub=Mock(return_value=True),
            end_return=Mock(),
        ),
        reset,
        logger,
    )

    errors = completion.run()

    assert errors == ()
    disarm.assert_not_called()
    reset.clear.assert_called_once_with()
    assert "real autopilot" in logger.warning.call_args.args[0]


def test_simulated_one_shot_disarms_before_reset():
    events = []
    completion = OneShotCompletion(
        OneShotCompletionPorts(
            is_simulated_autopilot=lambda: True,
            is_stopping=lambda: False,
            disarm=lambda: events.append("disarm"),
            return_to_hub=lambda: events.append("hub") is None,
            end_return=lambda: events.append("end"),
        ),
        SimpleNamespace(clear=lambda: events.append("reset")),
        Mock(),
    )

    assert completion.run() == ()
    assert events == ["end", "disarm", "reset"]


# Step 10, item 3 (owner rulings 2026-11-09): a failed one-shot dock is never disarmed; NavPy flies to the DDH and
# parks there on the operator's confirmation, in simulation and on real hardware. ArduPilot's RTL would take the UAV
# from NavPy. A dock keeps the simulation disarm and ends any return.


def _completion(simulated: bool, hub: bool = True):
    events = []
    completion = OneShotCompletion(
        OneShotCompletionPorts(
            is_simulated_autopilot=lambda: simulated,
            is_stopping=lambda: False,
            disarm=lambda: events.append("disarm"),
            return_to_hub=lambda: events.append("hub") is None and hub,
            end_return=lambda: events.append("end"),
        ),
        SimpleNamespace(clear=lambda: events.append("reset")),
        Mock(),
    )
    return completion, events


@pytest.mark.parametrize("simulated", [True, False])
def test_item3_a_failed_one_shot_dock_returns_to_the_ddh_and_never_disarms(simulated):
    completion, events = _completion(simulated)
    assert completion.run(failed=True) == ()
    assert events == ["hub", "reset"]


def test_item3_without_a_ddh_a_failed_one_shot_dock_is_reported_and_held():
    completion, events = _completion(simulated=True, hub=False)
    assert completion.run(failed=True) == ()
    assert events == ["hub", "reset"]
    assert "no DDH" in completion._logger.warning.call_args.args[0]


def test_item3_a_dock_ends_the_return_and_keeps_the_simulation_disarm():
    completion, events = _completion(simulated=True)
    assert completion.run() == ()
    assert events == ["end", "disarm", "reset"]
    completion, events = _completion(simulated=False)
    assert completion.run() == ()
    assert events == ["end", "reset"]


def test_item3_a_failed_return_start_is_reported_and_the_task_still_clears():
    reset = Mock()
    completion = OneShotCompletion(
        OneShotCompletionPorts(lambda: True, lambda: False, Mock(), Mock(side_effect=RuntimeError("hub")), Mock()),
        reset, Mock(),
    )
    assert [str(error) for error in completion.run(failed=True)] == ["hub"]
    reset.clear.assert_called_once_with()


def _failed_terminal() -> FinalApproachEpisodeTerminal:
    identity = DockContactIdentity(vehicle=2, task_id=11, dock_uid=0, episode=3)
    return FinalApproachEpisodeTerminal(identity, FinalApproachTerminal.FAILED, "no_detection")


@pytest.mark.parametrize("returning, completed", [(True, False), (False, True)])
def test_item3_a_failed_one_shot_dock_with_a_ddh_is_not_the_end_of_the_sortie(returning, completed):
    """NavTransition's outcome: a failed dock returning to the DDH is not complete, so DOCKED is not final."""
    task = NavigationTaskState(final_approach_navigation_active=True)
    cleanup = Mock()
    cleanup.run.return_value = NavCleanupResult(
        snap=None, errors=(), own_dock_contact=None, dock_terminal=_failed_terminal(),
    )
    finish = Mock(return_value=())
    transition = NavTransition(
        Mock(), cleanup, Mock(), finish, task, lambda: True, logger=Mock(), sortie_returning=lambda: returning,
    )
    outcome = transition.exit(NavState.RESET)
    finish.assert_called_once_with(failed=True)
    assert outcome == NavExitOutcome(redirect=NavState.DOCKED, oneshot_completed=completed)
