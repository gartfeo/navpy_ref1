"""A missed guidance ends as "dock failed" (docs/design/missed_dock.md)."""

from __future__ import annotations

from unittest.mock import Mock

import pytest

from navpy.core.approach.logs.navigation_snap_types import ClosestSnap
from navpy.core.approach.nav_exit_cleanup import (
    NavCleanupResult,
    NavExitCleanup,
    NavExitCleanupPorts,
)
from navpy.core.approach.vision_nav.episode_latch import (
    MISSED_DOCK_REASON,
    GuidanceEpisodeLatch,
    GuidanceEpisodeResult,
    GuidanceResult,
)
from navpy.core.approach.vision_nav.dock_outcome import GuidanceDockOutcome
from navpy.core.approach.vision_nav.runtime import GuidanceSessionRuntime
from navpy.core.flow.nav_state import NavigationTaskState, NavState
from navpy.core.flow.nav_transition import NavExitOutcome, NavTransition
from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.dock_contact import DockContactEvent, DockContactIdentity

IDENTITY = DockContactIdentity(vehicle=2, task_id=11, dock_uid=0, episode=3)
REASONS = ("no_detection", "missed_dock", "navigation_rejected", "source_command_stale")


def _contact() -> DockContactEvent:
    return DockContactEvent(
        identity=IDENTITY,
        aircraft_time_s=5.0,
        closest_3d_m=0.1,
        closest_h_m=0.1,
        closest_v_m=0.0,
    )


def _failed(reason="no_detection", result=GuidanceResult.FAILED):
    return GuidanceEpisodeResult(IDENTITY, result, reason)


# Episode latch


def test_a_failure_ends_commands_and_keeps_its_reason():
    latch = GuidanceEpisodeLatch()
    latch.begin(IDENTITY)

    assert latch.latch_failure("no_detection")

    assert latch.commands_ended
    assert latch.result_snapshot() == _failed("no_detection")


def test_a_miss_stays_the_result_when_its_nav_fail_follows():
    latch = GuidanceEpisodeLatch()
    latch.begin(IDENTITY)
    latch.latch_missed()

    assert latch.latch_failure("missed_dock")

    assert latch.result_snapshot() == _failed(
        MISSED_DOCK_REASON, GuidanceResult.MISSED,
    )


def test_the_first_named_reason_wins_over_a_repeated_nav_fail():
    latch = GuidanceEpisodeLatch()
    latch.begin(IDENTITY)
    latch.latch_failure()  # an unnamed command failure latched first
    latch.latch_failure("source_command_stale")
    latch.latch_failure("no_detection")

    assert latch.result_snapshot() == _failed("source_command_stale")


def test_a_failure_after_contact_is_refused_and_a_contact_after_failure_too():
    docked = GuidanceEpisodeLatch()
    docked.begin(IDENTITY)
    docked.latch_docked(_contact())
    assert docked.latch_failure("no_detection") is False
    assert docked.result_snapshot().outcome is GuidanceResult.DOCKED

    failed = GuidanceEpisodeLatch()
    failed.begin(IDENTITY)
    failed.latch_failure("no_detection")
    assert failed.latch_docked(_contact()) is False  # late contact not counted
    assert failed.result_snapshot() == _failed("no_detection")


def test_no_snapshot_without_an_armed_episode_or_after_reset():
    unarmed = GuidanceEpisodeLatch()  # real build: no sensor, no episode
    unarmed.latch_failure("no_detection")
    assert not unarmed.commands_ended  # today's flow: RESET retires commands
    assert unarmed.result_snapshot() is None

    armed = GuidanceEpisodeLatch()
    armed.begin(IDENTITY)
    armed.latch_failure("no_detection")
    armed.reset()
    assert armed.result_snapshot() is None
    assert not armed.commands_ended


# Session runtime: a failure retires work under the runtime lock


def _session(episode: GuidanceEpisodeLatch):
    command_reset = Mock()
    evidence = Mock()
    session = GuidanceSessionRuntime(
        rt.RLock(), Mock(), command_reset, episode, Mock(), Mock(), evidence,
    )
    return session, command_reset, evidence


def test_a_failure_retires_held_and_queued_work_at_once():
    episode = GuidanceEpisodeLatch()
    episode.begin(IDENTITY)
    session, command_reset, _ = _session(episode)

    assert session.latch_failure("navigation_rejected")

    command_reset.retire_after_result.assert_called_once_with()
    assert episode.result_snapshot() == _failed("navigation_rejected")


def test_an_unarmed_failure_retires_nothing_before_reset():
    """The real build arms no episode: RESET retires its work, as before."""
    session, command_reset, _ = _session(GuidanceEpisodeLatch())

    assert session.latch_failure("no_detection") is True
    command_reset.retire_after_result.assert_not_called()


def test_a_refused_failure_retires_nothing_more():
    episode = GuidanceEpisodeLatch()
    episode.begin(IDENTITY)
    episode.latch_docked(_contact())
    session, command_reset, _ = _session(episode)

    assert session.latch_failure("no_detection") is False
    command_reset.retire_after_result.assert_not_called()


def test_the_dock_outcome_reads_the_result_and_records_the_failed_row():
    episode = GuidanceEpisodeLatch()
    record = Mock()
    outcome = GuidanceDockOutcome(rt.RLock(), episode, record)
    assert outcome.result() is None
    episode.begin(IDENTITY)
    episode.latch_failure("no_detection")

    assert outcome.result() == _failed("no_detection")
    outcome.record_failed(_failed("no_detection"), "no_detection")
    record.assert_called_once_with(IDENTITY, "no_detection")


# NAV exit cleanup: the result is read before the reset clears it


def test_cleanup_reads_the_result_before_the_navigation_reset():
    latch = GuidanceEpisodeLatch()
    latch.begin(IDENTITY)
    latch.latch_failure("no_detection")
    order = []
    ports = NavExitCleanupPorts(
        disarm_dock_contact=lambda: order.append("disarm"),
        close_guidance_source=lambda: order.append("close"),
        navigation_reset=lambda: order.append("reset") or latch.reset(),
        detector_stop=lambda: None,
        dock_outcome=Mock(
            result=lambda: order.append("result") or latch.result_snapshot(),
        ),
    )
    speedup = Mock()
    speedup.restore.return_value = True

    result = NavExitCleanup(ports, speedup, Mock()).run()

    assert order == ["disarm", "result", "close", "reset"]
    assert result.dock_result == _failed("no_detection")
    assert result.errors == ()


def test_a_result_read_error_is_a_cleanup_error():
    ports = NavExitCleanupPorts(
        disarm_dock_contact=lambda: None,
        close_guidance_source=lambda: None,
        navigation_reset=lambda: None,
        detector_stop=lambda: None,
        dock_outcome=Mock(result=Mock(side_effect=RuntimeError("result"))),
    )
    speedup = Mock()
    speedup.restore.return_value = True

    result = NavExitCleanup(ports, speedup, Mock()).run()

    assert [str(error) for error in result.errors] == ["result"]


# NAV exit: a failed dock ends like DOCKED, never back to search


def _exit(
    result=_failed(),
    *,
    contact=None,
    requested=NavState.RESET,
    active=True,
    oneshot=False,
    errors=(),
):
    navigation_task = NavigationTaskState(guidance_navigation_active=active)
    cleanup = Mock()
    cleanup.run.return_value = NavCleanupResult(
        snap=ClosestSnap(dist=0.7),
        errors=tuple(errors),
        own_dock_contact=contact,
        dock_result=result,
    )
    completion = Mock()
    completion.run.return_value = ()
    logger = Mock()
    transition = NavTransition(
        Mock(), cleanup, Mock(), completion.run, navigation_task,
        lambda: oneshot, logger=logger,
    )
    return transition.exit(requested), logger, cleanup.record_dock_failed, completion


@pytest.mark.parametrize("reason", REASONS)
def test_each_nav_fail_reason_ends_in_docked_with_one_status_line(reason):
    outcome, logger, record, _ = _exit(_failed(reason))

    assert outcome == NavExitOutcome(redirect=NavState.DOCKED)
    record.assert_called_once_with(_failed(reason), reason)
    logger.info.assert_called_once()
    assert logger.info.call_args.args[0] == (
        f"STATE DOCK_FAILED (sim): reason={reason} dock=0 task=11"
    )


def test_an_aft_dock_miss_reports_missed_dock():
    result = _failed(MISSED_DOCK_REASON, GuidanceResult.MISSED)
    outcome, logger, _, _ = _exit(result)

    assert outcome.redirect is NavState.DOCKED
    assert "reason=missed_dock" in logger.info.call_args.args[0]


def test_an_unnamed_failure_reports_nav_fail():
    _, logger, record, _ = _exit(_failed(None))

    record.assert_called_once_with(_failed(None), "nav_fail")
    assert "reason=nav_fail" in logger.info.call_args.args[0]


def test_a_kept_contact_is_docked_not_failed():
    outcome, logger, record, _ = _exit(None, contact=_contact())

    assert outcome.redirect is NavState.DOCKED
    record.assert_not_called()
    assert logger.info.call_args.args[0].startswith("STATE DOCKED (sim):")


def test_one_shot_failed_dock_finishes_as_failed_and_stays_final():
    # Owner ruling 2026-11-09 (navpy_recovery.md step 10, item 3): the sortie finish is told the dock failed, so it
    # asks for RTL instead of disarming.
    outcome, _, _, completion = _exit(oneshot=True)

    assert outcome == NavExitOutcome(redirect=NavState.DOCKED, oneshot_completed=True)
    completion.run.assert_called_once_with(failed=True)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"result": None},  # real build: no armed episode, today's RESET
        {"active": False},  # GUIDED never accepted
        {"requested": NavState.ONHOLD},  # inactive guard: the operator wins
        {"requested": NavState.DETECT},  # GUIDED departure
        {"requested": None},
        {"result": _failed(result=GuidanceResult.DOCKED)},
    ],
)
def test_out_of_scope_endings_keep_todays_flow(kwargs):
    outcome, logger, record, _ = _exit(**kwargs)

    assert outcome == NavExitOutcome()
    record.assert_not_called()
    logger.info.assert_not_called()


def test_a_cleanup_error_raises_and_reports_nothing():
    with pytest.raises(Exception, match="NAV exit failed"):
        _exit(errors=(RuntimeError("reset"),))


def test_cleanup_records_the_failed_row_through_its_port():
    outcome = Mock()
    ports = NavExitCleanupPorts(
        disarm_dock_contact=lambda: None,
        close_guidance_source=lambda: None,
        navigation_reset=lambda: None,
        detector_stop=lambda: None,
        dock_outcome=outcome,
    )
    NavExitCleanup(ports, Mock(), Mock()).record_dock_failed(_failed(), "no_detection")
    outcome.record_failed.assert_called_once_with(_failed(), "no_detection")


# The NAV_FAIL source names its reason


@pytest.mark.parametrize(
    ("mark", "reason"),
    [
        ("mark_no_detection", "no_detection"),
        ("mark_missed_dock", "missed_dock"),
        ("mark_navigation_rejected", "navigation_rejected"),
        ("mark_source_liveness_expired", "source_command_stale"),
    ],
)
def test_each_nav_fail_passes_its_reason_to_the_latch(mark, reason):
    from navpy.core.approach.guidance_command_dispatch import (
        GuidanceCommandDispatch,
        GuidanceCommandPorts,
    )

    mark_failed = Mock(return_value=True)
    dispatch = GuidanceCommandDispatch(
        GuidanceCommandPorts(nav=Mock(), mark_failed=mark_failed, logger=Mock()),
        Mock(),
    )

    getattr(dispatch, mark)(None)

    mark_failed.assert_called_once_with(reason)


def test_a_missing_active_poi_names_its_reason():
    from navpy.core.approach.guidance_source_event_handler import (
        GuidanceSourceEventHandler,
        GuidanceSourceEventHandlerPorts,
    )

    mark_failed = Mock(return_value=True)
    handler = GuidanceSourceEventHandler(GuidanceSourceEventHandlerPorts(
        active_poi=lambda: None, session_active=lambda: True, reset_source=Mock(),
        admission=Mock(), mark_failed=mark_failed, logger=Mock(), commands=Mock(),
    ))

    assert handler.handle(Mock(source_discontinuity=False)) is False
    mark_failed.assert_called_once_with("source_event_active_poi_missing")


def test_the_record_deadline_names_its_reason():
    from navpy.core.approach.guidance_record_deadline import (
        GuidanceRecordDeadline,
    )
    from navpy.core.flow.nav_constants import GUIDANCE_RECORD_TIMEOUT_S
    from navpy.core.flow.nav_state import GuidanceNavState

    now_s = [0.0]
    mark_failed = Mock()
    deadline = GuidanceRecordDeadline(
        GuidanceNavState(), mark_failed, lambda: now_s[0], Mock(), Mock(),
    )
    poi = Mock()
    deadline.defer_or_fail(poi)
    now_s[0] = GUIDANCE_RECORD_TIMEOUT_S + 0.1
    deadline.defer_or_fail(poi)

    mark_failed.assert_called_once_with("guidance_record_unavailable")


def test_a_source_pump_failure_names_its_reason():
    from navpy.app.flow.nav_guidance_composition import _report_pump_failure

    state = Mock()
    _report_pump_failure(Mock(reason="crashed", error=None), state, Mock())

    state.navigation_failures.mark_failed.assert_called_once_with("source_event_pump")


# The whole NAV exit, real owners end to end: latch → cleanup → exit → transition


def _chain(*, oneshot: bool, contact_first: bool = False, reset=None):
    from navpy.products.delivery.nav_oneshot_completion import (
        OneShotCompletion,
        OneShotCompletionPorts,
    )
    from navpy.core.flow.nav_state import ConfirmGateState, PostDockClimb
    from navpy.core.flow.navigation_transition_handler import (
        NavigationTransitionHandler,
        TransitionPorts,
    )

    events: list = []
    latch = GuidanceEpisodeLatch()
    latch.begin(IDENTITY)
    if contact_first:
        latch.latch_docked(_contact())
    latch.latch_failure("no_detection")
    latch.latch_docked(_contact())  # a contact after the NAV_FAIL is not counted
    outcome = GuidanceDockOutcome(
        rt.RLock(), latch, lambda identity, reason: events.append(("row", identity, reason)),
    )
    speedup = Mock()
    speedup.restore.return_value = True
    cleanup = NavExitCleanup(
        NavExitCleanupPorts(
            disarm_dock_contact=lambda: _contact() if contact_first else None,
            close_guidance_source=lambda: None,
            navigation_reset=lambda: events.append("reset") or latch.reset(),
            detector_stop=lambda: None,
            dock_outcome=outcome,
        ),
        speedup,
        Mock(),
    )
    reset = Mock() if reset is None else reset
    completion = OneShotCompletion(
        OneShotCompletionPorts(
            is_simulated_autopilot=lambda: True,
            is_stopping=lambda: False,
            disarm=lambda: events.append("disarm"),
            request_rtl=lambda: events.append("rtl"),
        ),
        reset,
        Mock(),
    )
    logger = Mock()
    nav = NavTransition(
        Mock(), cleanup, Mock(), completion.run,
        NavigationTaskState(guidance_navigation_active=True),
        lambda: oneshot, logger=logger,
    )
    ports = TransitionPorts(
        decision_clock_s=lambda: 0.0,
        return_to_search=Mock(),
        enter_recovery=Mock(),
        start_task_actor=Mock(),
        is_oneshot=lambda: oneshot,
    )
    climb = PostDockClimb()
    handler = NavigationTransitionHandler(
        ports, ConfirmGateState(), nav, reset, Mock(), climb,
    )
    return handler, ports, climb, events, logger


@pytest.mark.parametrize("oneshot", [True, False])
def test_a_failed_dock_enters_docked_after_the_drain_and_never_restarts_auto(oneshot):
    handler, ports, _, events, logger = _chain(oneshot=oneshot)

    result = handler.on_change(NavState.NAV, NavState.RESET)

    assert result.effective_state is NavState.DOCKED
    ports.return_to_search.assert_not_called()
    row = ("row", IDENTITY, "no_detection")
    # One-shot: RTL through recovery, no disarm (owner ruling 2026-11-09, step 10 item 3).
    expected = ["reset", "rtl", row] if oneshot else ["reset", row]
    assert events == expected  # the row follows the reset that drained commands
    assert [call.args[0] for call in logger.info.call_args_list] == [
        "STATE DOCK_FAILED (sim): reason=no_detection dock=0 task=11",
    ]


@pytest.mark.parametrize("oneshot", [True, False])
def test_a_failed_dock_clears_the_task_exactly_once(oneshot):
    # One-shot: the sortie finish clears at NAV exit and the DOCKED entry
    # skips it; cyclic: only the DOCKED entry clears.
    reset = Mock()
    handler, _, _, _, _ = _chain(oneshot=oneshot, reset=reset)

    handler.on_change(NavState.NAV, NavState.RESET)

    reset.clear.assert_called_once_with()


def test_cyclic_failed_dock_climbs_before_the_auto_restart():
    handler, ports, climb, _, _ = _chain(oneshot=False)
    handler.on_change(NavState.NAV, NavState.RESET)

    handler.on_change(NavState.DOCKED, NavState.RECOVERY)
    assert climb.active
    ports.enter_recovery.assert_called_once_with()
    ports.return_to_search.assert_not_called()


def test_cyclic_failed_dock_restarts_auto_only_on_leaving_docked_high():
    handler, ports, _, _, _ = _chain(oneshot=False)
    handler.on_change(NavState.NAV, NavState.RESET)
    ports.return_to_search.assert_not_called()

    handler.on_change(NavState.DOCKED, NavState.DETECT)
    ports.return_to_search.assert_called_once_with()


def test_a_contact_before_the_nav_fail_is_docked_end_to_end():
    handler, _, _, events, logger = _chain(oneshot=False, contact_first=True)

    result = handler.on_change(NavState.NAV, NavState.RESET)

    assert result.effective_state is NavState.DOCKED
    assert events == ["reset"]  # no DOCK_FAILED row
    assert logger.info.call_args.args[0].startswith("STATE DOCKED (sim):")


def test_real_build_one_shot_nav_fail_keeps_todays_reset_and_asks_for_no_rtl():
    """Codex round 1 on step 10, finding 1 (rebutted): the real build arms no episode (no contact sensor), so its
    NAV_FAIL is no dock outcome (rule 1 above, owner-approved 2026-10-18): the one-shot goes back to search as today,
    no sortie finish, so neither RTL nor disarm. Step 10 item 3's RTL is for DOCK_FAILED; on hardware it applies once
    a real capture signal arms episodes, and OneShotCompletion asks for it there without a simulation gate."""
    latch = GuidanceEpisodeLatch()                      # never begun: the real build
    assert latch.latch_failure("no_detection")
    result = GuidanceDockOutcome(rt.RLock(), latch, Mock()).result()
    assert result is None

    outcome, logger, record, completion = _exit(result, oneshot=True)

    assert outcome == NavExitOutcome()                       # the decision's RESET: back to search
    completion.run.assert_not_called()
    record.assert_not_called()
    logger.info.assert_not_called()
