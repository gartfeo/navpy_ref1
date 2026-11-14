"""The round runner and the response resolver share one fixed deadline.

ConfirmationRoundRunner.run is driven on the test thread with a hand-moved
clock and an event whose wait advances that clock, so every case is exact
and uses no real time. Approvals are delivered from inside the runner's own
steps (broadcast, image send, wait, failure policy), where a late worker
used to leave the round open.
"""

from __future__ import annotations

import math
import threading
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Optional
from unittest.mock import Mock, call

import pytest

from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.review.confirmation_manager_state import ConfirmationManagerState
from navpy.core.review.confirmation_registry_state import ConfirmationStatus
from navpy.core.review.confirmation_round_runner import ConfirmationRoundRunner
from navpy.core.review.confirmation_round_transaction import ConfirmationResponseKind
from tests.confirmation_doubles import SettableClock
from tests.detection_factory import make_detected_poi

Kind = ConfirmationResponseKind
POI_ID = 61
START_S = 1.0
WAIT_S = 30.0
DEADLINE_S = START_S + WAIT_S
MAX_WAITS = 200


class SteppingEvent:
    """The round's event: each wait moves the clock on by the time asked."""

    def __init__(self, clock: SettableClock, on_wait: Optional[Callable[[float], None]]) -> None:
        self._clock = clock
        self._on_wait = on_wait
        self._flag = False
        self.waits: list[float] = []

    def is_set(self) -> bool:
        return self._flag

    def set(self) -> None:
        self._flag = True

    def wait(self, timeout: Optional[float] = None) -> bool:
        if self._flag:
            return True
        assert timeout is not None
        self.waits.append(timeout)
        assert len(self.waits) <= MAX_WAITS, "the wait loop did not end"
        if self._on_wait is not None:
            self._on_wait(timeout)
        else:
            # A real clock always moves; a rounding-sized wait must not freeze it.
            self._clock.now_s += max(timeout, 1e-6)
        return self._flag


@dataclass
class Rig:
    """One runner on real state, with every outside step scripted."""

    policy_status: ConfirmationStatus = ConfirmationStatus.TIMEOUT_REJECTED
    wait_values: list[float] = field(default_factory=lambda: [WAIT_S])
    resend_interval_s: float = 100.0
    on_wait: Optional[Callable[[float], None]] = None

    def __post_init__(self) -> None:
        self.clock = SettableClock(START_S)
        self.state = ConfirmationManagerState()
        self.poi: DetectedObject = make_detected_poi(obj_id=POI_ID, task_id=POI_ID, class_id=0)
        self.network = Mock()
        self.media = Mock()
        self.media.send.return_value = True
        self.failure_policy = Mock()
        self.failure_policy.status.return_value = self.policy_status
        self.logger = Mock()
        self.wait_reads = 0
        self.events: list[SteppingEvent] = []
        self.answers: list[ConfirmationResponseKind] = []
        self.runner = ConfirmationRoundRunner(
            sys_id=1,
            is_simulation=False,
            state=self.state,
            network=lambda: self.network,
            wait_time_s=self._read_wait,
            resend_interval_s=lambda: self.resend_interval_s,
            failure_policy=self.failure_policy,
            media=self.media,
            logger=self.logger,
            monotonic=self.clock,
            event_factory=self._event,
        )

    def _read_wait(self) -> float:
        value = self.wait_values[min(self.wait_reads, len(self.wait_values) - 1)]
        self.wait_reads += 1
        return value

    def _event(self) -> SteppingEvent:
        event = SteppingEvent(self.clock, self.on_wait)
        self.events.append(event)
        return event

    def answer(self, is_confirmed: bool, at_s: Optional[float] = None) -> ConfirmationResponseKind:
        """Deliver an operator answer, first moving the clock when asked."""
        if at_s is not None:
            self.clock.now_s = at_s
        result = self.state.resolve_response(POI_ID, is_confirmed)
        self.answers.append(result)
        return result

    def run(self) -> None:
        worker = self.state.start_review(self.poi, threading.Event)
        self.runner.run(self.poi, worker)

    @property
    def status(self) -> Optional[ConfirmationStatus]:
        return self.state.registry.get_status(self.poi)

    @property
    def waits(self) -> list[float]:
        return [wait for event in self.events for wait in event.waits]

    @property
    def request_sends(self) -> int:
        return self.network.broadcast.call_count

    def timeout_log(self, label: str, wait_s: float = WAIT_S) -> object:
        return call(
            f"POI {POI_ID} {label} after waiting {wait_s}s.",
            key="nav_state",
            status=f"P{POI_ID} {label} after {wait_s}s.",
        )


def _image_send(rig: Rig, action: Callable[[], object]) -> None:
    def send(*_args: object, **_kwargs: object) -> bool:
        action()
        return True

    rig.media.send.side_effect = send


@pytest.mark.parametrize(
    ("policy_status", "label"),
    [
        (ConfirmationStatus.TIMEOUT_REJECTED, "auto-rejected (timeout)"),
        (ConfirmationStatus.CONFIRMED, "auto-confirmed"),
    ],
)
def test_approval_during_a_delayed_image_send_ends_with_the_timeout_outcome(
    policy_status: ConfirmationStatus,
    label: str,
) -> None:
    """The reproduced defect: the image send outlasts the window (1 s to 1001 s)
    and an approval lands before the worker gets to its timeout."""
    rig = Rig(policy_status=policy_status)
    _image_send(rig, lambda: rig.answer(True, at_s=1001.0))

    rig.run()

    assert rig.answers == [Kind.DEADLINE_PASSED]
    assert rig.status is policy_status
    assert rig.timeout_log(label) in rig.logger.info.call_args_list
    assert rig.waits == [], "the window had already ended: nothing left to wait for"
    assert rig.state.pending_poi(POI_ID) is None


def test_a_refused_approval_does_not_stop_the_image_send_in_flight() -> None:
    """The refusal leaves the round as it was: the rest of the image still goes
    out and the worker still reaches its timeout. A refusal that woke the round
    would cut the send and leave the POI in CONFIRMING with no outcome."""
    rig = Rig()
    units_sent: list[bool] = []

    def send(_poi: object, _poi_id: int, run_unit: Callable[[Callable[[], None]], bool]) -> bool:
        if not run_unit(lambda: None):
            return False
        rig.answer(True, at_s=1001.0)
        units_sent.append(run_unit(lambda: None))
        return units_sent[-1]

    rig.media.send.side_effect = send

    rig.run()

    assert rig.answers == [Kind.DEADLINE_PASSED]
    assert units_sent == [True]
    assert rig.status is ConfirmationStatus.TIMEOUT_REJECTED
    assert rig.timeout_log("auto-rejected (timeout)") in rig.logger.info.call_args_list


def test_approval_before_the_deadline_still_confirms_after_slow_sends() -> None:
    rig = Rig()
    _image_send(rig, lambda: rig.answer(True, at_s=math.nextafter(DEADLINE_S, 0.0)))

    rig.run()

    assert rig.answers == [Kind.CONFIRMED]
    assert rig.status is ConfirmationStatus.CONFIRMED
    assert rig.waits == []
    rig.failure_policy.status.assert_called_once()  # asked, but its outcome is refused
    assert all("after waiting" not in str(entry) for entry in rig.logger.info.call_args_list)


def test_approval_in_the_gap_before_the_timeout_is_written_is_refused() -> None:
    """The wait has ended and the worker is computing its timeout outcome."""
    rig = Rig()

    def policy_status(_poi_id: int) -> ConfirmationStatus:
        rig.answer(True)
        return ConfirmationStatus.TIMEOUT_REJECTED

    rig.failure_policy.status.side_effect = policy_status

    rig.run()

    assert rig.waits == [WAIT_S]
    assert rig.clock.now_s == DEADLINE_S
    assert rig.answers == [Kind.DEADLINE_PASSED]
    assert rig.status is ConfirmationStatus.TIMEOUT_REJECTED
    assert rig.timeout_log("auto-rejected (timeout)") in rig.logger.info.call_args_list


def test_approval_inside_a_wait_that_overshoots_the_deadline_is_refused() -> None:
    rig = Rig()

    def overshoot(_timeout: float) -> None:
        rig.answer(True, at_s=DEADLINE_S + 7.0)

    rig.on_wait = overshoot

    rig.run()

    assert rig.waits == [WAIT_S]
    assert rig.answers == [Kind.DEADLINE_PASSED]
    assert rig.status is ConfirmationStatus.TIMEOUT_REJECTED


def test_resends_do_not_move_the_deadline_and_the_wait_is_read_once() -> None:
    rig = Rig(wait_values=[10.0], resend_interval_s=3.0)
    sent_at: list[float] = []
    rig.network.broadcast.side_effect = lambda _request: sent_at.append(rig.clock.now_s)

    def policy_status(_poi_id: int) -> ConfirmationStatus:
        rig.answer(True)  # at the original deadline, after three resends
        return ConfirmationStatus.TIMEOUT_REJECTED

    rig.failure_policy.status.side_effect = policy_status

    rig.run()

    assert sent_at == [START_S, START_S + 3.0, START_S + 6.0, START_S + 9.0]
    assert rig.waits == [3.0, 3.0, 3.0, 1.0]
    assert rig.clock.now_s == START_S + 10.0
    assert rig.wait_reads == 1
    assert rig.answers == [Kind.DEADLINE_PASSED]
    assert rig.status is ConfirmationStatus.TIMEOUT_REJECTED
    assert rig.timeout_log("auto-rejected (timeout)", 10.0) in rig.logger.info.call_args_list


def test_a_wait_parameter_raised_mid_round_does_not_extend_it() -> None:
    rig = Rig(wait_values=[WAIT_S, 9999.0])
    _image_send(rig, lambda: rig.answer(True, at_s=DEADLINE_S + 1.0))

    rig.run()

    assert rig.wait_reads == 1
    assert rig.answers == [Kind.DEADLINE_PASSED]
    assert rig.status is ConfirmationStatus.TIMEOUT_REJECTED
    assert rig.timeout_log("auto-rejected (timeout)", WAIT_S) in rig.logger.info.call_args_list


@pytest.mark.parametrize(
    ("answer_at_s", "expected", "status"),
    [
        (START_S + 29.0, Kind.CONFIRMED, ConfirmationStatus.CONFIRMED),
        (START_S + 31.0, Kind.DEADLINE_PASSED, ConfirmationStatus.TIMEOUT_REJECTED),
    ],
)
def test_the_first_send_counts_against_the_window(
    answer_at_s: float,
    expected: ConfirmationResponseKind,
    status: ConfirmationStatus,
) -> None:
    """The window starts when the round is built, not when the first send returns."""
    rig = Rig()

    def slow_first_send(_request: object) -> None:
        rig.clock.now_s += 20.0

    rig.network.broadcast.side_effect = slow_first_send
    _image_send(rig, lambda: rig.answer(True, at_s=answer_at_s))

    rig.run()

    assert rig.answers == [expected]
    assert rig.status is status


def test_zero_wait_stays_an_automatic_confirm() -> None:
    rig = Rig(wait_values=[0.0])
    rig.network.broadcast.side_effect = lambda _request: rig.answer(True)
    _image_send(rig, lambda: rig.answer(True))

    rig.run()

    assert rig.answers == [Kind.DEADLINE_PASSED, Kind.DEADLINE_PASSED]
    assert rig.status is ConfirmationStatus.CONFIRMED
    rig.logger.warning.assert_called_once_with(
        f"P{POI_ID} Confirmed. No wait time.",
        key="nav_state",
        dest=LogStatusDest.DRONE,
    )
    assert rig.waits == []
    rig.failure_policy.status.assert_not_called()


def test_zero_wait_still_takes_an_operator_reject() -> None:
    rig = Rig(wait_values=[0.0])
    _image_send(rig, lambda: rig.answer(False))

    rig.run()

    assert rig.answers == [Kind.REJECTED]
    assert rig.status is ConfirmationStatus.REJECTED
    rig.logger.warning.assert_not_called()


@pytest.mark.parametrize(
    ("policy_status", "label"),
    [
        (ConfirmationStatus.TIMEOUT_REJECTED, "auto-rejected (timeout)"),
        (ConfirmationStatus.CONFIRMED, "auto-confirmed"),
    ],
)
def test_a_wait_that_is_not_a_number_ends_with_the_policy_outcome(
    policy_status: ConfirmationStatus,
    label: str,
) -> None:
    """Before, such a round never ended and took an approval at any time."""
    rig = Rig(policy_status=policy_status, wait_values=[math.nan])
    _image_send(rig, lambda: rig.answer(True))

    rig.run()

    assert rig.waits == []
    assert rig.answers == [Kind.DEADLINE_PASSED]
    assert rig.status is policy_status
    assert rig.timeout_log(label, math.nan) in rig.logger.info.call_args_list


def test_a_frozen_clock_parks_on_the_event_every_pass() -> None:
    """With time standing still below the deadline the loop must not spin: each
    pass waits on the round's event, and the round ends only when it is retired."""
    rig = Rig(resend_interval_s=3.0)
    passes = 0

    def frozen(_timeout: float) -> None:
        nonlocal passes
        passes += 1
        if passes == 5:
            rig.state.reset()  # the review is withdrawn

    rig.on_wait = frozen

    rig.run()

    assert rig.waits == [3.0] * 5
    assert rig.request_sends == 5  # the first send and one resend per earlier pass
    assert rig.clock.now_s == START_S
    assert rig.status is None
    rig.failure_policy.status.assert_called_once()
    assert all("after waiting" not in str(entry) for entry in rig.logger.info.call_args_list)


def test_a_failed_first_send_takes_the_policy_outcome() -> None:
    rig = Rig()
    rig.network.broadcast.side_effect = OSError("link down")

    rig.run()

    assert rig.status is ConfirmationStatus.TIMEOUT_REJECTED
    assert rig.wait_reads == 1
    assert rig.waits == []
    rig.media.send.assert_not_called()
    assert rig.state.pending_poi(POI_ID) is None


@pytest.mark.parametrize(
    ("policy_status", "label"),
    [
        (ConfirmationStatus.TIMEOUT_REJECTED, "auto-rejected (timeout)"),
        (ConfirmationStatus.CONFIRMED, "auto-confirmed"),
    ],
)
def test_an_unanswered_round_times_out_with_the_same_outcome_and_text(
    policy_status: ConfirmationStatus,
    label: str,
) -> None:
    rig = Rig(policy_status=policy_status)

    rig.run()

    assert rig.waits == [WAIT_S]
    assert rig.request_sends == 1
    assert rig.status is policy_status
    assert rig.timeout_log(label) in rig.logger.info.call_args_list
    rig.logger.warning.assert_not_called()


def test_a_late_reject_beats_the_automatic_confirm() -> None:
    rig = Rig(policy_status=ConfirmationStatus.CONFIRMED)
    _image_send(rig, lambda: rig.answer(False, at_s=1001.0))

    rig.run()

    assert rig.answers == [Kind.REJECTED]
    assert rig.status is ConfirmationStatus.REJECTED
    assert all("auto-confirmed" not in str(entry) for entry in rig.logger.info.call_args_list)
