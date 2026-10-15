"""The auction under the baton (``lockstep_fleet.md``, Auction: timing and ids).

Three real swarm nodes run on one baton (:mod:`swarm_baton`): every timer
is a baton timer and every message a tick input after the measured hop.
The cases are the plan's: an incomplete matrix never selects, a final bid
or ACK on its deadline tick is handled before the timer, stale BUSY
evidence, a retry and a release, task registration order, and the
offset-adjusted TTL before and after the first heartbeat.
"""

from __future__ import annotations

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.core.link.check_msg import CheckInMsg
from navpy.core.link.listener_abc import ListenerAbc
from navpy.core.link.msg_meta import MsgMeta
from navpy.core.link.swarm_ack_msg import SwarmAckMsg
from navpy.core.link.swarm_heartbeat_msg import SwarmHeartbeatMsg, SwarmNodeState
from navpy.core.link.task_assignment_msg import TaskAssignRequestMsg, TaskAssignResponseMsg
from navpy.core.link.task_availability_msg import AvailableTaskRequestMsg, AvailableTaskResponseMsg
from navpy.core.link.ttl_defaults import get_ttl_ms
from navpy.core.link.types import MsgType, TaskDispatchStatus
from navpy.core.swarm.task_actor_slots import SlotState
from navpy.core.swarm.task_auction_coordinator import ASSIGN_REQUEST_SCHEDULE, PEER_SELECTION_DELAY_S
from navpy.sim.lockstep.mavlink_wire import encoder
from tests.modules.lockstep.swarm_baton import (
    DETECT_TICK,
    DROP,
    EPOCH_US,
    FAR_PEER,
    HOP_TICKS,
    NEAR_PEER,
    OWNER,
    BatonSwarm,
    Sent,
    baton_swarm,
    tick_of,
)

TASK = 2
def _detect(swarm: BatonSwarm, *task_ids: int):
    return lambda: swarm.detect(*(task_ids or (TASK,)))


def _confirmed_to(swarm: BatonSwarm, peer: int, task: int = TASK) -> bool:
    return swarm.query(lambda: _is_confirmed_to(swarm, peer, task))


def _is_confirmed_to(swarm: BatonSwarm, peer: int, task: int) -> bool:
    dispatch = swarm.owner._auction_state.lookup(task)
    assigned = {n for n, a in swarm.actors.items() if a._selection.state() is SlotState.ASSIGNED}
    return (dispatch.status is TaskDispatchStatus.CONFIRMED and dispatch.assigned_peer == peer
            and assigned == {peer})


def _owner_requests(swarm: BatonSwarm) -> list[tuple[int, int]]:
    """(tick, receiver) of every assignment request copy the owner sent."""
    return [(s.k, s.message.receiver_id) for s in swarm.requests(TaskAssignRequestMsg, OWNER)]


def _is(sent: Sent, kind: type, sender: int) -> bool:
    return isinstance(sent.message, kind) and sent.sender == sender


def _ticks(swarm: BatonSwarm, node: int, text: str) -> list[int]:
    return [k for k, line in swarm.lines(node) if text in line]


# -- the matrix and the 2 s selection timer -----------------------------------

def _far_bids_dropped(jitter_seed: int | None = None):
    with baton_swarm(jitter_seed) as swarm:
        swarm.rule = lambda s, _r: DROP if _is(s, AvailableTaskResponseMsg, FAR_PEER) else None
        swarm.run(DETECT_TICK + 400, {DETECT_TICK: _detect(swarm)})
        incomplete = list(swarm.trace)
        requests_while_incomplete = _owner_requests(swarm)
        swarm.rule = lambda _s, _r: None
        swarm.run(DETECT_TICK + 600)
        confirmed = _confirmed_to(swarm, NEAR_PEER)
    return swarm, incomplete, requests_while_incomplete, confirmed


def test_an_incomplete_matrix_never_selects_even_after_the_selection_timer():
    swarm, incomplete, requests, confirmed = _far_bids_dropped()
    offered = [k for k, node, line in incomplete if node == OWNER and "can be handled by 2" in line]
    fired = [k for k, node, line in incomplete if node == OWNER and line.startswith("selection timer")]
    assert offered and fired, "the near bid was recorded and the 2 s timer fired"
    assert fired[0] == offered[0] + tick_of(PEER_SELECTION_DELAY_S)
    assert requests == []  # four seconds with one bid of two: nobody selected
    # Once the far peer's bid is heard on a re-advert, the input completes the matrix.
    assert confirmed
    first = _owner_requests(swarm)[0]
    far_bid = _ticks(swarm, OWNER, "delivered AVAILABLE_TASK_RESPONSE from 3")[0]
    assert first == (far_bid, NEAR_PEER)
    assert _far_bids_dropped(5)[0].trace == swarm.trace


def _final_bid_at(deadline_offset: int):
    """The far peer's first bid lands ``deadline_offset`` ticks after the timer's tick."""
    _swarm, incomplete, _r, _c = _far_bids_dropped()
    timer_k = next(k for k, node, line in incomplete if node == OWNER and line.startswith("selection timer"))
    held = []

    def rule(s: Sent, _receiver: int):
        if not _is(s, AvailableTaskResponseMsg, FAR_PEER):
            return None
        held.append(s)
        return timer_k + deadline_offset if len(held) == 1 else DROP

    with baton_swarm() as swarm:
        swarm.rule = rule
        swarm.run(timer_k + 50, {DETECT_TICK: _detect(swarm)})
        assert _confirmed_to(swarm, NEAR_PEER)
    return swarm, timer_k


def test_a_final_bid_on_the_timer_deadline_tick_is_handled_before_the_timer():
    swarm, timer_k = _final_bid_at(0)
    at_deadline = [line for k, line in swarm.lines(OWNER) if k == timer_k]
    bid = next(i for i, line in enumerate(at_deadline) if line.startswith("info: Task 2 can be handled by 3"))
    assert at_deadline.index("sent TaskAssignRequestMsg") == bid + 1
    # The input completed the matrix and its assignment cancelled the due timer.
    assert _ticks(swarm, OWNER, "selection timer for task 2") == []
    assert _owner_requests(swarm) == [(timer_k, NEAR_PEER)]

    late, _ = _final_bid_at(1)  # one tick late: the timer fires first and selects nothing
    timer_first = [line for k, line in late.lines(OWNER) if k == timer_k]
    assert "selection timer for task 2" in timer_first and "sent TaskAssignRequestMsg" not in timer_first
    assert _owner_requests(late) == [(timer_k + 1, NEAR_PEER)]


def _near_replies_to_owner(s: Sent, receiver: int) -> bool:
    return receiver == OWNER and s.sender == NEAR_PEER and isinstance(
        s.message, (SwarmAckMsg, TaskAssignResponseMsg))


def _ack_run(rule):
    with baton_swarm() as swarm:
        swarm.rule = rule
        swarm.run(DETECT_TICK + 400, {DETECT_TICK: _detect(swarm)})
        confirmed = _confirmed_to(swarm, NEAR_PEER)
    return swarm, confirmed


def test_a_request_ack_on_the_resend_deadline_tick_is_handled_before_the_resend():
    lost, _ = _ack_run(lambda s, r: DROP if _near_replies_to_owner(s, r) else None)
    first_k = _owner_requests(lost)[0][0]
    resend_k = _ticks(lost, OWNER, "request not acked by 2; resent it (2/3)")[0]
    assert resend_k == first_k + tick_of(ASSIGN_REQUEST_SCHEDULE.interval_s)

    # The acceptance would cancel the resend timer itself, so it lands after the
    # deadline: on the deadline tick the task is still confirming, with only the ACK.
    accepted_k = resend_k + 50

    def held_to(ack_k: int):
        def rule(s: Sent, receiver: int) -> int | None:
            if not _near_replies_to_owner(s, receiver):
                return None
            due = accepted_k if isinstance(s.message, TaskAssignResponseMsg) else ack_k
            return due if s.k + HOP_TICKS < due else None
        return rule

    on_time, confirmed = _ack_run(held_to(resend_k))
    assert confirmed and _ticks(on_time, OWNER, "resent it") == []
    assert _owner_requests(on_time) == [(first_k, NEAR_PEER)]
    assert on_time.lines(OWNER, "delivered TASK_ASSIGN_RESPONSE from 2")[0][0] == accepted_k
    late, confirmed = _ack_run(held_to(resend_k + 1))
    assert confirmed and _ticks(late, OWNER, "resent it (2/3)") == [resend_k]


# -- stale BUSY evidence (MsgRef.follows) -------------------------------------

def _set_approaching(swarm: BatonSwarm, node: int, approaching: bool):
    return lambda: swarm.actors[node].set_approaching(approaching)


def _heartbeat(s: Sent, sender: int, state: SwarmNodeState) -> bool:
    return _is(s, SwarmHeartbeatMsg, sender) and s.message.state == state


def test_a_busy_report_older_than_a_bid_never_marks_the_peer_busy():
    """The near peer's BUSY heartbeat arrives after its newer FREE bid: ignored."""
    stale_k = 260

    def rule(s: Sent, receiver: int):
        if receiver == OWNER and _heartbeat(s, NEAR_PEER, SwarmNodeState.BUSY) and s.k < 220:
            return stale_k
        if _is(s, AvailableTaskResponseMsg, FAR_PEER):
            return 270  # the matrix stays open past the stale report
        return None

    with baton_swarm() as swarm:
        swarm.rule = rule
        swarm.run(400, {110: _set_approaching(swarm, NEAR_PEER, True),
                        210: _set_approaching(swarm, NEAR_PEER, False),
                        220: _detect(swarm)})
        busy = [s.k for s in swarm.log if _heartbeat(s, NEAR_PEER, SwarmNodeState.BUSY)]
        assert busy[0] == 200  # the BUSY report precedes the bid (later ones: its own task)
        bid = [s.k for s in swarm.requests(AvailableTaskResponseMsg, NEAR_PEER)]
        assert bid and 200 < bid[0] < stale_k - HOP_TICKS
        assert stale_k in [k for k, line in swarm.lines(OWNER)
                           if line == "delivered SWARM_HEARTBEAT from 2"]
        assert _confirmed_to(swarm, NEAR_PEER)
        assert _owner_requests(swarm)[0] == (270, NEAR_PEER)


def test_a_bid_sent_before_a_busy_report_is_stale_even_once_the_peer_is_free_again():
    """The near peer bids, reports BUSY, then FREE; the old bid lands last.

    Its FREE report does not revive the bid: only a bid that follows the
    BUSY report counts (``bid_is_current``). Later bids are lost, so the old
    one is the only near bid the owner can hear."""
    def rule(s: Sent, receiver: int):
        if receiver != OWNER or not isinstance(s.message, AvailableTaskResponseMsg):
            return None
        if s.sender == NEAR_PEER and s.k < 160:
            return 330
        return DROP  # the far peer's bids, and the near peer's newer ones

    with baton_swarm() as swarm:
        swarm.rule = rule
        swarm.run(400, {DETECT_TICK: _detect(swarm), 160: _set_approaching(swarm, NEAR_PEER, True),
                        210: _set_approaching(swarm, NEAR_PEER, False)})
        reports = [(s.k, s.message.state) for s in swarm.log if _is(s, SwarmHeartbeatMsg, NEAR_PEER) and s.k >= 200]
        assert reports[:2] == [(200, SwarmNodeState.BUSY), (300, SwarmNodeState.FREE)]
        at_330 = [line for k, line in swarm.lines(OWNER) if k == 330]
        assert "warning: Task 2: offer from 2 not recorded (no available task, invalid ETA, "                "or sent before it was busy)." in at_330
        assert not any("can be handled by 2" in line for line in at_330)
        assert _owner_requests(swarm) == []


# -- a retry and a release ------------------------------------------------------

def test_every_request_copy_lost_releases_on_schedule_and_the_retry_assigns():
    release_s = ASSIGN_REQUEST_SCHEDULE.deadline_s
    with baton_swarm() as swarm:
        end = DETECT_TICK + tick_of(release_s) + 50
        swarm.rule = lambda s, _r: DROP if _is(s, TaskAssignRequestMsg, OWNER) and s.k < end else None
        swarm.run(end + 200, {DETECT_TICK: _detect(swarm)})
        copies = _owner_requests(swarm)
        first = copies[0][0]
        step = tick_of(ASSIGN_REQUEST_SCHEDULE.interval_s)
        assert copies[:ASSIGN_REQUEST_SCHEDULE.copies] == [
            (first + i * step, NEAR_PEER) for i in range(ASSIGN_REQUEST_SCHEDULE.copies)]
        assert _ticks(swarm, OWNER, "released and re-advertising (retry 1)") == [first + tick_of(release_s)]
        assert _confirmed_to(swarm, NEAR_PEER)


def test_a_rejected_offer_is_retried_with_the_next_peer():
    """The near peer starts its own approach while WAITING and rejects."""
    def rule(s: Sent, _receiver: int):
        if _is(s, TaskAssignResponseMsg, NEAR_PEER) and s.message.is_accepted:
            return DROP  # the owner never confirms it
        return None

    with baton_swarm() as swarm:
        swarm.rule = rule
        swarm.run(400, {DETECT_TICK: _detect(swarm), DETECT_TICK + 20: _set_approaching(swarm, NEAR_PEER, True)})
        assert _ticks(swarm, NEAR_PEER, "Own final approach started; rejecting task 2 from owner 1")
        assert [peer for _k, peer in _owner_requests(swarm)][:1] == [NEAR_PEER]
        assert _ticks(swarm, OWNER, "Task 2 rejected by 2")
        assert _confirmed_to(swarm, FAR_PEER)


# -- task registration order ------------------------------------------------------

def test_offer_rows_follow_task_registration_order():
    with baton_swarm() as swarm:
        swarm.run(DETECT_TICK + 100, {DETECT_TICK: _detect(swarm, 7, 3),
                                      DETECT_TICK + 1: _detect(swarm, 5)})
        adverts = [[t.task_id for t in s.message.tasks]
                   for s in swarm.requests(AvailableTaskRequestMsg, OWNER)]
        assert adverts[:3] == [[7], [3], [5]]  # one advert per task, as registered
        for peer in (NEAR_PEER, FAR_PEER):
            bids = [h.task_id for s in swarm.requests(AvailableTaskResponseMsg, peer) for h in s.message.tasks]
            assert bids[:3] == [7, 3, 5]
        order = swarm.query(lambda: list(swarm.owner._auction_state._store.dispatches))
        assert order == [7, 3, 5]
        # One plan over the offer rows in that order; its requests follow it.
        assert [s.message.task.task_id for s in swarm.requests(TaskAssignRequestMsg, OWNER)][:2] == [7, 3]


# -- TTL against the offset estimate ---------------------------------------------

SKEW_MS = 8_000  # the sender's clock runs this far behind
TTL_MS = get_ttl_ms(MsgType.CHECK_IN)


class _Admitted(ListenerAbc):
    def __init__(self, swarm: BatonSwarm) -> None:
        self._swarm = swarm

    def on_message(self, message) -> None:
        self._swarm.note(OWNER, f"admitted {type(message).__name__} seq {message.meta.msg_seq}")


def test_ttl_is_bypassed_before_the_first_heartbeat_and_offset_adjusted_after():
    link, sink = encoder(FAR_PEER, mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
    epoch_ms = EPOCH_US // 1000

    def stamped(message, seq: int, sent_k: int, extra_age_ms: int = 0) -> bytes:
        message.meta = MsgMeta(boot_id=77, msg_seq=seq, time_ms=epoch_ms + sent_k * 10 - SKEW_MS - extra_age_ms,
                               ttl_ms=TTL_MS)
        link.send(message.to_mavlink())
        return sink.drain()

    with baton_swarm(start=False) as swarm:
        swarm.query(lambda: swarm.networks[OWNER].set_listener(_Admitted(swarm)))
        # Before any heartbeat: 8 s behind, past its 5 s TTL, still admitted.
        swarm.inject(10, OWNER, stamped(CheckInMsg(FAR_PEER), 1, 6))
        # The first heartbeat: the offset is the skew plus the 4-tick hop.
        swarm.inject(20, OWNER, stamped(SwarmHeartbeatMsg(FAR_PEER), 2, 16))
        swarm.inject(30, OWNER, stamped(CheckInMsg(FAR_PEER), 3, 26))  # adjusted age 0
        swarm.inject(40, OWNER, stamped(CheckInMsg(FAR_PEER), 4, 36, TTL_MS))  # exactly the TTL
        swarm.inject(50, OWNER, stamped(CheckInMsg(FAR_PEER), 5, 46, TTL_MS + 10))  # one tick past it
        swarm.run(60)
        admitted = [(k, line) for k, line in swarm.lines(OWNER, "admitted")]
        offset = swarm.query(lambda: swarm.networks[OWNER].message_filter._offset.get_offset(FAR_PEER))
        expired = swarm.query(lambda: swarm.networks[OWNER].message_filter.metrics.rx_dropped_expired)
    assert offset == SKEW_MS + HOP_TICKS * 10
    assert admitted == [(10, "admitted CheckInMsg seq 1"), (20, "admitted SwarmHeartbeatMsg seq 2"),
                        (30, "admitted CheckInMsg seq 3"), (40, "admitted CheckInMsg seq 4")]
    assert expired == 1
