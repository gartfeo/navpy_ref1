"""The assignment-handshake timers keep their TTL-derived relationships."""

import math

import pytest

from navpy.modules.comm.messages import ttl_defaults
from navpy.modules.comm.messages.ttl_defaults import (
    SWARM_RESEND_INTERVAL_MS,
    get_ttl_ms,
)
from navpy.modules.comm.messages.types import MsgType
from navpy.modules.swarm.task_ack_timing import (
    ASSIGN_ACK_TIMING,
    AssignAckTiming,
    RepeatSchedule,
    ack_ttl_ms,
    assign_ack_timing,
)

REQUEST = MsgType.TASK_ASSIGN_REQUEST
RESPONSE = MsgType.TASK_ASSIGN_RESPONSE


def _assert_relationships(timing: AssignAckTiming, interval_ms: int) -> None:
    interval_s = interval_ms / 1000.0
    request, response = timing.request, timing.response
    assert request.interval_s == response.interval_s == interval_s
    # Owner: copies spread over one request TTL, release once the last copy
    # can no longer be admitted.
    assert request.copies == math.ceil(get_ttl_ms(REQUEST) / interval_ms)
    last_request_s = (request.copies - 1) * interval_s
    assert request.deadline_s == pytest.approx(
        last_request_s + get_ttl_ms(REQUEST) / 1000.0
    )
    # Helper: every step-4 copy goes out before the owner's release, and the
    # next slot would not.
    last_response_s = (response.copies - 1) * interval_s
    assert last_response_s < request.deadline_s
    assert last_response_s + interval_s >= request.deadline_s
    # WAITING outlives the last copy and the owner's ack of it.
    assert response.deadline_s == pytest.approx(
        last_response_s
        + (get_ttl_ms(RESPONSE) + ack_ttl_ms(RESPONSE)) / 1000.0
    )


def test_current_ttls_give_the_documented_timers():
    request, response = ASSIGN_ACK_TIMING.request, ASSIGN_ACK_TIMING.response

    assert (request.interval_s, request.copies, request.deadline_s) == (
        2.0, 3, 9.0,
    )
    assert (response.interval_s, response.copies, response.deadline_s) == (
        2.0, 5, 18.0,
    )
    _assert_relationships(ASSIGN_ACK_TIMING, SWARM_RESEND_INTERVAL_MS)


def test_ack_lives_as_long_as_the_acked_message():
    for acked in (REQUEST, RESPONSE):
        assert ack_ttl_ms(acked) == get_ttl_ms(acked)


@pytest.mark.parametrize(
    ("request_ttl_ms", "response_ttl_ms"),
    [(3000, 4000), (8000, 2000), (4000, 4000)],
)
def test_timers_follow_a_changed_ttl_table(
    monkeypatch, request_ttl_ms, response_ttl_ms,
):
    monkeypatch.setitem(ttl_defaults.TTL_DEFAULTS, REQUEST, request_ttl_ms)
    monkeypatch.setitem(ttl_defaults.TTL_DEFAULTS, RESPONSE, response_ttl_ms)

    timing = assign_ack_timing()

    assert timing != ASSIGN_ACK_TIMING
    _assert_relationships(timing, SWARM_RESEND_INTERVAL_MS)


def test_timers_follow_a_changed_resend_interval():
    _assert_relationships(assign_ack_timing(1500), 1500)


def test_schedule_ticks_after_each_copy_then_at_the_deadline():
    request, response = ASSIGN_ACK_TIMING.request, ASSIGN_ACK_TIMING.response

    request_ticks = [request.delay_after(sent) for sent in (1, 2, 3)]
    response_ticks = [response.delay_after(sent) for sent in range(1, 6)]

    assert request_ticks == [2.0, 2.0, 5.0]
    assert sum(request_ticks) == request.deadline_s
    assert response_ticks == [2.0, 2.0, 2.0, 2.0, 10.0]
    assert sum(response_ticks) == response.deadline_s


def test_single_copy_schedule_waits_for_the_whole_deadline():
    assert RepeatSchedule(2.0, 1, 5.0).delay_after(1) == 5.0
