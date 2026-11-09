"""Parking at the home dock waits for the operator: no timeout, no cancel (owner rulings 2026-11-09;
core/review/standing_review.py; docs/design/navpy_recovery.md step 10, item 3).

With a network the hub's review is never auto-confirmed, has no round deadline, no CONFIRM dwell or re-acquire abort,
and ignores a negative answer. With no network it is confirmed as before. Delivery docks keep today's behaviour.
"""
from __future__ import annotations

import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from navpy.core.flow.nav_state import NavPhaseState, NavState
from navpy.core.review.confirmation_coordinator import ConfirmationCoordinator
from navpy.core.review.confirmation_inbound import ConfirmationInboundHandler
from navpy.core.review.confirmation_manager import _active_standing
from navpy.core.review.confirmation_manager_state import ConfirmationManagerState
from navpy.core.review.confirmation_registry_state import ConfirmationStatus
from navpy.core.review.confirmation_round_runner import ConfirmationRoundRunner
from navpy.core.review.standing_review import StandingReview
from navpy.core.skills.confirm.confirmation_action import (
    FinalApproachConfirmationAdmission,
    FinalApproachConfirmationPorts,
)
from navpy.core.skills.confirm.poi_rejection_exit import PoiRejectionExit, PoiRejectionPorts
from tests.detection_factory import make_detected_poi


def _poi(task_id: int = 7):
    return make_detected_poi(obj_id=task_id, task_id=task_id, class_id=0)


def _synchronous_thread(target, args, daemon):
    return SimpleNamespace(start=lambda: target(*args))


def _coordinator(*, standing: bool, network):
    state = ConfirmationManagerState()
    runner = Mock()
    coordinator = ConfirmationCoordinator(
        sys_id=1, state=state, network=lambda: network, auto_confirm=lambda: True, assignment=Mock(),
        round_runner=runner, logger=Mock(), event_factory=threading.Event, thread_factory=_synchronous_thread,
        standing=lambda _poi: standing,
    )
    return coordinator, state, runner


def test_with_a_network_a_standing_review_is_asked_even_with_auto_confirm():
    coordinator, state, runner = _coordinator(standing=True, network=Mock())
    poi = _poi()
    coordinator.review([poi])
    runner.run.assert_called_once()
    assert state.registry.get_status(poi) is not ConfirmationStatus.CONFIRMED


def test_a_delivery_dock_keeps_auto_confirm():
    coordinator, state, runner = _coordinator(standing=False, network=Mock())
    poi = _poi()
    coordinator.review([poi])
    runner.run.assert_not_called()
    assert state.registry.get_status(poi) is ConfirmationStatus.CONFIRMED


def test_with_no_network_a_standing_review_is_confirmed_as_before():
    coordinator, state, runner = _coordinator(standing=True, network=None)
    poi = _poi()
    coordinator.review([poi])
    runner.run.assert_not_called()
    assert state.registry.get_status(poi) is ConfirmationStatus.CONFIRMED


class _Clock:
    """Each read is 100 s later: any deadline (CWT 30 s) has passed by the second read."""

    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        self.now += 100.0
        return self.now


def _runner(state, network_slot, *, standing: bool):
    failure_policy = Mock()
    failure_policy.status.return_value = ConfirmationStatus.TIMEOUT_REJECTED
    media = Mock()
    media.send.return_value = True
    return ConfirmationRoundRunner(
        sys_id=1, is_simulation=False, state=state, network=lambda: network_slot[0], wait_time_s=lambda: 30.0,
        resend_interval_s=lambda: 0.005, failure_policy=failure_policy, media=media, logger=Mock(),
        monotonic=_Clock(), event_factory=threading.Event, standing=lambda _poi: standing,
    )


def test_a_delivery_dock_round_times_out_at_its_deadline():
    state = ConfirmationManagerState()
    poi = _poi()
    worker = state.start_review(poi, threading.Event)
    _runner(state, [Mock()], standing=False).run(poi, worker)
    assert state.registry.get_status(poi) is ConfirmationStatus.TIMEOUT_REJECTED


def test_a_standing_round_has_no_deadline_and_ends_on_the_operators_confirmation():
    state = ConfirmationManagerState()
    poi = _poi()
    worker = state.start_review(poi, threading.Event)
    network = Mock()
    resent = threading.Event()
    network.broadcast.side_effect = lambda _msg: resent.set() if network.broadcast.call_count >= 6 else None
    runner = _runner(state, [network], standing=True)
    thread = threading.Thread(target=runner.run, args=(poi, worker), daemon=True)
    thread.start()
    assert resent.wait(5.0)                                  # still asking long after any deadline
    assert state.registry.get_status(poi) is ConfirmationStatus.CONFIRMING
    state.resolve_response(7, True, None)                    # the operator's Approve
    thread.join(5.0)
    assert not thread.is_alive()
    assert state.registry.get_status(poi) is ConfirmationStatus.CONFIRMED


def test_a_standing_round_whose_network_goes_is_confirmed_as_with_none():
    state = ConfirmationManagerState()
    poi = _poi()
    worker = state.start_review(poi, threading.Event)
    slot = [Mock()]
    slot[0].broadcast.side_effect = lambda _msg: slot.__setitem__(0, None) if slot[0].broadcast.call_count >= 3 \
        else None
    _runner(state, slot, standing=True).run(poi, worker)
    assert state.registry.get_status(poi) is ConfirmationStatus.CONFIRMED


@pytest.mark.parametrize("standing, resolved", [(True, False), (False, True)])
def test_a_negative_answer_is_ignored_only_for_a_standing_review(standing, resolved):
    state = Mock()
    inbound = ConfirmationInboundHandler(
        sys_id=1, state=state, media=Mock(), logger=Mock(), standing_id=lambda _poi_id: standing,
    )
    inbound.handle_response(SimpleNamespace(task_id=7, is_confirmed=False, meta=None))
    assert state.resolve_response.called is resolved
    inbound.handle_response(SimpleNamespace(task_id=7, is_confirmed=True, meta=None))
    assert state.resolve_response.call_args.args[:2] == (7, True)


def test_an_answer_is_standing_only_for_the_active_standing_poi():
    standing = StandingReview()
    poi = _poi(7)
    assert not _active_standing(poi, 7, standing)            # no check set: no POI stands
    standing.set_check(lambda candidate: candidate is poi)
    assert _active_standing(poi, 7, standing)
    assert not _active_standing(poi, 8, standing) and not _active_standing(None, 7, standing)


def _exit(*, standing: bool):
    phase = NavPhaseState(NavState.DETECT, NavState.DETECT)
    timing = Mock()
    timing.review_dwell_s.return_value = 1000.0
    timing.max_dwell_sec.return_value = 45.0
    timing.reacquire_timed_out.return_value = True
    timing.loss_elapsed_s.return_value = 3.0
    timing.reacquire_bound_s.return_value = 2.5
    timing.reacquire_bound_label.return_value = "pre-request"
    ports = PoiRejectionPorts(
        wall_s=lambda: 0.0, clear_active_poi=Mock(), stop_tracking=Mock(), logger=Mock(),
        standing=lambda _poi: standing,
    )
    retry = Mock()
    return PoiRejectionExit(ports, phase, timing, retry), ports, phase, retry


@pytest.mark.parametrize("status", [ConfirmationStatus.CONFIRMING, None])
def test_a_standing_review_has_no_dwell_limit_and_no_reacquire_abort(status):
    exit_, ports, phase, retry = _exit(standing=True)
    assert exit_.apply(_poi(), status) is False
    ports.clear_active_poi.assert_not_called()
    retry.register_poi_cooldown.assert_not_called()
    assert phase.is_current(NavState.CONFIRM)


@pytest.mark.parametrize("status", [ConfirmationStatus.CONFIRMING, None])
def test_a_delivery_dock_keeps_its_dwell_limit_and_reacquire_abort(status):
    exit_, ports, phase, retry = _exit(standing=False)
    assert exit_.apply(_poi(), status) is True
    ports.clear_active_poi.assert_called_once()
    retry.register_poi_cooldown.assert_called_once()


@pytest.mark.parametrize("standing, confirmed_locally", [(True, False), (False, True)])
def test_the_final_approach_path_never_confirms_a_standing_review_locally(standing, confirmed_locally):
    review = Mock()
    admission = FinalApproachConfirmationAdmission(
        FinalApproachConfirmationPorts(
            final_approach_active=lambda: True, can_confirm=lambda _poi: True, record_confirmed=lambda _poi: True,
            auto_confirm=lambda: True, standing=lambda _poi: standing,
        ),
        SimpleNamespace(confirmed_recorded=False), review, Mock(),
    )
    assert admission.handle(_poi()) is confirmed_locally
    assert review.confirm_local.called is confirmed_locally


@pytest.mark.parametrize("standing, task_type", [(True, "HOME_DOCK"), (False, "DOCK")])
def test_the_request_names_a_standing_review_home_dock_for_the_gcs(standing, task_type):
    """The GCS shows an approve-only card for HOME_DOCK (step 10, item 3)."""
    from navpy.core.perception.profile.vision_class_profile import DOCK_DETECT_CLASS_ID

    state = ConfirmationManagerState()
    runner = _runner(state, [Mock()], standing=standing)
    poi = make_detected_poi(obj_id=7, task_id=7, class_id=DOCK_DETECT_CLASS_ID)
    assert runner._request_message(poi, 7).task.task_type.name == task_type
