"""SelectedTaskSlot follows the helper slot table, one case per row.

See docs/design/swarm-task-assignment-ack.md, "Helper slot".
"""

import threading

import pytest

from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.swarm_ack_msg import (
    ACK_STATUS_APPLIED,
    ACK_STATUS_PROCESSING,
    ACK_STATUS_RECEIVED,
)
from navpy.modules.comm.messages.swarm_heartbeat_msg import SwarmNodeState
from navpy.modules.comm.messages.task_message_data import TaskAssignMsgData
from navpy.modules.comm.messages.types import TaskTypeMsgData
from navpy.modules.swarm.task_actor_slots import (
    AckKind,
    RequestKind,
    SelectedTaskSlot,
    SlotState,
)
from navpy.modules.swarm.task_msg_refs import MsgRef

OWNER, BOOT = 1, 7
REQUEST = MsgRef(OWNER, BOOT, 50)


def _task(task_id: int = 10) -> TaskAssignMsgData:
    return TaskAssignMsgData(
        task_id=task_id,
        task_type=TaskTypeMsgData.DOCK,
        location=LocationMsgData(1.0, 2.0, 3.0),
    )


@pytest.fixture
def slot() -> SelectedTaskSlot:
    return SelectedTaskSlot(threading.RLock())


def _wait(slot, task_id=10, request=REQUEST):
    decision = slot.on_request(_task(task_id), request, flyable=True)
    assert decision.kind is RequestKind.WAIT
    return decision.held


def _reply(slot, held, seq=5):
    reply = MsgRef(2, 9, seq)
    assert slot.record_reply(held.token, reply)
    return reply


def _assigned(slot):
    held = _wait(slot)
    reply = _reply(slot, held)
    assert slot.on_ack(reply, ACK_STATUS_APPLIED, MsgRef(OWNER, BOOT, 60))[0] is (
        AckKind.ASSIGNED
    )
    return held


# EMPTY


def test_empty_flyable_offer_waits_invisible_to_nav(slot):
    held = _wait(slot)

    assert slot.state() is SlotState.WAITING
    assert held.task.task_id == 10 and held.owner.owner_id == OWNER
    assert slot.selected() is None
    assert slot.node_state() is SwarmNodeState.BUSY


@pytest.mark.parametrize("approaching, flyable", [(True, True), (False, False)])
def test_empty_offer_while_approaching_or_unflyable_is_rejected(
    slot, approaching, flyable,
):
    slot.set_approaching(approaching)

    decision = slot.on_request(_task(), REQUEST, flyable=flyable)

    assert decision.kind is RequestKind.REJECT
    assert slot.state() is SlotState.EMPTY


# WAITING


def test_waiting_equal_copy_answers_again_and_tracks_the_newest_request(slot):
    held = _wait(slot)

    newer = slot.on_request(_task(), MsgRef(OWNER, BOOT, 52), flyable=True)
    older = slot.on_request(_task(), MsgRef(OWNER, BOOT, 51), flyable=True)

    assert newer.kind is older.kind is RequestKind.REPEAT
    assert slot.held().request == MsgRef(OWNER, BOOT, 52)
    assert slot.held().token == held.token


def test_waiting_offer_from_restarted_owner_replaces_it(slot):
    old = _wait(slot)

    decision = slot.on_request(_task(), MsgRef(OWNER, BOOT + 1, 3), flyable=True)

    assert decision.kind is RequestKind.WAIT
    assert decision.held.owner.boot_id == BOOT + 1
    assert decision.held.token != old.token


@pytest.mark.parametrize(
    "task_id, offer",
    [(11, MsgRef(OWNER, BOOT, 51)), (10, MsgRef(3, 8, 51))],
    ids=["other-task", "other-owner"],
)
def test_waiting_rejects_any_other_offer(slot, task_id, offer):
    held = _wait(slot)

    decision = slot.on_request(_task(task_id), offer, flyable=True)

    assert decision.kind is RequestKind.REJECT
    assert slot.held() == held


def test_waiting_applied_by_owner_boot_for_its_copy_assigns(slot):
    held = _wait(slot)
    reply = _reply(slot, held)

    kind, assigned = slot.on_ack(reply, ACK_STATUS_APPLIED, MsgRef(OWNER, BOOT, 60))

    assert kind is AckKind.ASSIGNED and assigned == held
    assert slot.selected() == _task()


@pytest.mark.parametrize(
    "acked, ack",
    [
        (MsgRef(2, 9, 99), MsgRef(OWNER, BOOT, 60)),
        (MsgRef(2, 9, 5), MsgRef(OWNER, BOOT + 1, 60)),
        (MsgRef(2, 9, 5), MsgRef(3, BOOT, 60)),
    ],
    ids=["unknown-copy", "other-owner-boot", "other-sender"],
)
def test_waiting_ignores_applied_not_from_owner_for_its_copy(slot, acked, ack):
    _reply(slot, _wait(slot))

    assert slot.on_ack(acked, ACK_STATUS_APPLIED, ack) == (AckKind.NONE, None)
    assert slot.state() is SlotState.WAITING


def test_waiting_received_for_its_copy_drops_it(slot):
    held = _wait(slot)
    reply = _reply(slot, held)

    kind, dropped = slot.on_ack(reply, ACK_STATUS_RECEIVED, MsgRef(OWNER, BOOT, 60))

    assert kind is AckKind.DROPPED and dropped == held
    assert slot.state() is SlotState.EMPTY


def test_waiting_processing_keeps_it(slot):
    reply = _reply(slot, _wait(slot))

    assert slot.on_ack(reply, ACK_STATUS_PROCESSING, MsgRef(OWNER, BOOT, 60))[0] is (
        AckKind.NONE
    )
    assert slot.state() is SlotState.WAITING


def test_waiting_newer_advert_of_the_task_by_its_owner_boot_drops_it(slot):
    held = _wait(slot)

    assert slot.on_advert(MsgRef(OWNER, BOOT, 51), {10}) == held
    assert slot.state() is SlotState.EMPTY
    assert slot.node_state() is SwarmNodeState.FREE


@pytest.mark.parametrize(
    "advert, task_ids",
    [
        (MsgRef(OWNER, BOOT, 49), {10}),
        (MsgRef(OWNER, BOOT + 1, 99), {10}),
        (MsgRef(3, BOOT, 99), {10}),
        (MsgRef(OWNER, BOOT, 51), {11}),
        (None, {10}),
    ],
    ids=["older", "other-boot", "other-owner", "other-task", "no-uid"],
)
def test_waiting_keeps_other_adverts(slot, advert, task_ids):
    _wait(slot)

    assert slot.on_advert(advert, task_ids) is None
    assert slot.state() is SlotState.WAITING


def test_waiting_drops_when_own_final_approach_starts(slot):
    held = _wait(slot)

    assert slot.set_approaching(True) == held
    assert slot.state() is SlotState.EMPTY
    assert slot.node_state() is SwarmNodeState.BUSY


def test_waiting_expires_only_for_its_own_token(slot):
    held = _wait(slot)

    assert slot.expire(held.token + 1) is None
    assert slot.expire(held.token) == held
    assert slot.state() is SlotState.EMPTY


def test_waiting_survives_reset_and_clear(slot):
    held = _wait(slot)

    assert slot.release_assigned() is None
    assert slot.held() == held


# ASSIGNED


def test_assigned_ignores_adverts_acks_and_approach(slot):
    held = _assigned(slot)

    assert slot.on_advert(MsgRef(OWNER, BOOT, 99), {10}) is None
    assert slot.on_ack(MsgRef(2, 9, 5), ACK_STATUS_RECEIVED, MsgRef(OWNER, BOOT, 61))[0] is (
        AckKind.NONE
    )
    assert slot.set_approaching(True) is None
    assert slot.state() is SlotState.ASSIGNED
    assert slot.held() == held


def test_assigned_rejects_other_offers_and_answers_copies(slot):
    _assigned(slot)

    other = slot.on_request(_task(11), MsgRef(OWNER, BOOT, 70), flyable=True)
    copy = slot.on_request(_task(10), MsgRef(OWNER, BOOT, 71), flyable=True)

    assert other.kind is RequestKind.REJECT
    assert copy.kind is RequestKind.REPEAT
    assert slot.state() is SlotState.ASSIGNED


def test_assigned_is_released_when_nav_ends_it(slot):
    held = _assigned(slot)

    assert slot.release_assigned() == held
    assert slot.state() is SlotState.EMPTY
    assert slot.selected() is None
    assert slot.node_state() is SwarmNodeState.FREE


def test_reply_is_recorded_only_for_the_waiting_token(slot):
    held = _wait(slot)

    assert not slot.record_reply(held.token + 1, MsgRef(2, 9, 1))
    assert slot.record_reply(held.token, MsgRef(2, 9, 1))
