"""Focused ordering and failure semantics for NAV entry/exit owners."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from navpy.modules.nav.nav_exit_cleanup import (
    NavCleanupResult,
    NavExitCleanup,
    NavExitCleanupPorts,
)
from navpy.modules.nav.nav_oneshot_completion import (
    OneShotCompletion,
    OneShotCompletionPorts,
)
from navpy.modules.nav.nav_transition import NavTransition
from navpy.modules.nav.nav_state import NavigationTaskState, NavState


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
            close_final_approach_source=lambda: fail("source"),
            navigation_reset=lambda: fail("fence"),
            detector_stop=lambda: fail("detector"),
        ),
        speedup,
        logger,
    )

    result = cleanup.run()

    assert calls == ["source", "detector", "status", "speedup"]
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
        oneshot,
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
        ),
        SimpleNamespace(clear=lambda: events.append("reset")),
        Mock(),
    )

    assert completion.run() == ()
    assert events == ["disarm", "reset"]
