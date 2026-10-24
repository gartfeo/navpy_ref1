"""The assignment-handshake timers keep their TTL-derived relationships."""

import pytest

from navpy.core.link import ttl_defaults
from navpy.core.link.ttl_defaults import (
    SWARM_RESEND_INTERVAL_MS,
    get_ttl_ms,
)
from navpy.core.link.types import MsgType
from navpy.core.swarm.task_ack_timing import (
    ASSIGN_ACK_TIMING,
    AssignAckTiming,
    ack_ttl_ms,
    assign_ack_timing,
)
from navpy.core.swarm.task_msg_refs import MsgRef, RecentRefs
from navpy.core.swarm.task_peer_roster import PEER_SILENCE_EXPIRY_S

REQUEST = MsgType.TASK_ASSIGN_REQUEST
RESPONSE = MsgType.TASK_ASSIGN_RESPONSE


def _assert_relationships(timing: AssignAckTiming, interval_ms: int) -> None:
    request, response = timing.request, timing.response
    assert request.interval_s == response.interval_s == interval_ms / 1000.0
    # Silence means one heartbeat TTL, as it does for a peer.
    assert request.silence_s == response.silence_s == (
        get_ttl_ms(MsgType.SWARM_HEARTBEAT) / 1000.0
    )
    # A sent UID is kept while it and its ack can still be admitted.
    assert request.retention_s == (
        get_ttl_ms(REQUEST) + ack_ttl_ms(REQUEST)
    ) / 1000.0
    assert response.retention_s == (
        get_ttl_ms(RESPONSE) + ack_ttl_ms(RESPONSE)
    ) / 1000.0


def test_current_ttls_give_the_documented_timers():
    request, response = ASSIGN_ACK_TIMING.request, ASSIGN_ACK_TIMING.response

    assert (request.interval_s, request.silence_s, request.retention_s) == (
        0.5, 5.0, 10.0,
    )
    assert (response.interval_s, response.silence_s, response.retention_s) == (
        0.5, 5.0, 10.0,
    )
    assert request.silence_s == PEER_SILENCE_EXPIRY_S
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


def test_recent_refs_forget_a_copy_after_its_retention():
    now = [100.0]
    refs = RecentRefs(10.0, lambda: now[0])
    old, new = MsgRef(2, 7, 1), MsgRef(2, 7, 2)
    refs.add(old)
    now[0] = 105.0
    refs.add(new)
    now[0] = 109.9
    assert old in refs and new in refs
    now[0] = 110.0
    assert old not in refs and new in refs
    assert len(refs) == 1
