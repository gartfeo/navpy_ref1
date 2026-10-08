"""PeerRoster keeps the newest availability evidence per admitted peer."""

import threading

import pytest

from navpy.modules.comm.messages.swarm_heartbeat_msg import SwarmNodeState
from navpy.modules.comm.messages.ttl_defaults import get_ttl_ms
from navpy.modules.comm.messages.types import MsgType
from navpy.modules.swarm.task_msg_refs import MsgRef
from navpy.modules.swarm.task_peer_roster import (
    MAX_REMOTE_PEERS,
    PEER_SILENCE_EXPIRY_S,
    PeerRoster,
    sent_after,
)

FREE, BUSY = SwarmNodeState.FREE, SwarmNodeState.BUSY


@pytest.fixture
def roster() -> PeerRoster:
    roster = PeerRoster(threading.RLock())
    assert roster.add(2)
    return roster


def _ref(seq: int, boot_id: int = 7, peer_id: int = 2) -> MsgRef:
    return MsgRef(peer_id, boot_id, seq)


def test_admits_up_to_the_demo_peer_limit_and_starts_free():
    roster = PeerRoster(threading.RLock())

    admitted = [roster.add(peer_id) for peer_id in range(2, 3 + MAX_REMOTE_PEERS)]

    assert admitted.count(True) == MAX_REMOTE_PEERS
    assert roster.busy() == set()
    assert roster.status(2).state is FREE


def test_newer_evidence_applies_and_older_is_ignored(roster):
    assert roster.report(2, BUSY, _ref(10))
    assert not roster.report(2, FREE, _ref(9))
    assert not roster.report(2, FREE, _ref(10))
    assert roster.busy() == {2}

    assert roster.report(2, FREE, _ref(11))
    assert roster.busy() == set()


def test_another_boot_replaces_the_record_and_its_busy_report(roster):
    roster.report(2, BUSY, _ref(10))

    assert roster.report(2, FREE, _ref(1, boot_id=8))

    status = roster.status(2)
    assert (status.state, status.order, status.busy_order) == (
        FREE, _ref(1, boot_id=8), None,
    )
    assert roster.bid_is_current(2, _ref(2, boot_id=8))


def test_evidence_without_uid_never_overrides_ordered_evidence(roster):
    assert roster.report(2, BUSY, None)  # nothing ordered yet
    assert roster.report(2, FREE, _ref(5))

    assert not roster.report(2, BUSY, None)
    assert roster.busy() == set()


def test_unadmitted_peer_reports_nothing(roster):
    assert not roster.report(3, BUSY, _ref(1, peer_id=3))
    assert roster.busy() == set()


def test_bids_older_than_the_busy_report_are_stale(roster):
    roster.report(2, BUSY, _ref(10))

    assert not roster.bid_is_current(2, _ref(9))
    assert not roster.bid_is_current(2, None)
    assert not roster.bid_is_current(2, _ref(99, boot_id=8))
    assert roster.bid_is_current(2, _ref(11))


@pytest.mark.parametrize(
    "message, report, expected",
    [
        (_ref(11), _ref(10), True),
        (_ref(10), _ref(10), False),
        (None, _ref(10), False),
        (None, None, True),
        (_ref(1, boot_id=8), _ref(10), False),
    ],
    ids=["later", "same", "no-uid", "no-report", "other-boot"],
)
def test_sent_after_needs_proof(message, report, expected):
    assert sent_after(message, report) is expected


def test_clear_forgets_peers_and_status(roster):
    roster.report(2, BUSY, _ref(10))

    roster.clear()

    assert roster.snapshot() == set()
    assert roster.busy() == set()
    assert roster.status(2) is None


# --- Silence (no heartbeat or check-in for one heartbeat TTL) --------------


class FakeClock:
    def __init__(self) -> None:
        self.now_s = 100.0

    def __call__(self) -> float:
        return self.now_s


def test_silence_expiry_is_one_heartbeat_ttl():
    assert PEER_SILENCE_EXPIRY_S == get_ttl_ms(MsgType.SWARM_HEARTBEAT) / 1000.0


def test_unheard_peer_falls_silent_and_its_next_beat_revives_it():
    clock = FakeClock()
    roster = PeerRoster(threading.RLock(), clock)
    roster.add(2)
    roster.add(3)

    clock.now_s += PEER_SILENCE_EXPIRY_S
    roster.heard(3)
    roster.expire_silent()
    assert roster.busy() == set()  # exactly one TTL is not yet silent

    clock.now_s += 0.1
    roster.expire_silent()
    assert roster.busy() == {2}
    assert roster.status(2).silent

    roster.heard(2)
    assert roster.busy() == set()
    assert roster.status(2).last_heard_s == clock.now_s


def test_check_out_silences_at_once():
    roster = PeerRoster(threading.RLock(), FakeClock())
    roster.add(2)

    roster.silence(2)

    assert roster.busy() == {2}
    assert roster.contains(2)  # still routes its acks and responses
