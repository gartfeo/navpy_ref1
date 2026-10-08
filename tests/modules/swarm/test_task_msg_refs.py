"""Message references order only messages from one sender boot."""

import pytest

from navpy.modules.comm.messages.check_msg import CheckInMsg
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.swarm.task_msg_refs import MsgRef, OwnerRef, msg_ref


def _meta(boot_id: int, msg_seq: int) -> MsgMeta:
    return MsgMeta(boot_id=boot_id, msg_seq=msg_seq, time_ms=0, ttl_ms=5000)


def test_ref_is_the_dedup_uid_of_the_message():
    message = CheckInMsg(sender_id=2, meta=_meta(7, 41))

    ref = msg_ref(message)

    assert ref == MsgRef(sender_id=2, boot_id=7, msg_seq=41)
    assert (ref.sender_id, ref.boot_id, ref.msg_seq) == message.get_msg_uid()


@pytest.mark.parametrize(
    "meta",
    [None, _meta(0, 0)],
    ids=["no-meta", "legacy-zero-meta"],
)
def test_message_without_valid_meta_has_no_ref(meta):
    assert msg_ref(CheckInMsg(sender_id=2, meta=meta)) is None


def test_refs_are_values():
    assert MsgRef(2, 7, 41) == MsgRef(2, 7, 41)
    assert len({MsgRef(2, 7, 41), MsgRef(2, 7, 41), MsgRef(2, 7, 42)}) == 2
    assert MsgRef(2, 7, 41).owner == OwnerRef(owner_id=2, boot_id=7)


@pytest.mark.parametrize(
    ("ref", "floor", "follows"),
    [
        (MsgRef(2, 7, 42), MsgRef(2, 7, 41), True),
        (MsgRef(2, 7, 41), MsgRef(2, 7, 41), False),
        (MsgRef(2, 7, 40), MsgRef(2, 7, 41), False),
        (MsgRef(2, 8, 99), MsgRef(2, 7, 41), False),
        (MsgRef(3, 7, 99), MsgRef(2, 7, 41), False),
        (MsgRef(2, 7, 42), None, False),
    ],
    ids=["later", "same", "earlier", "other-boot", "other-sender", "no-floor"],
)
def test_follows_orders_only_one_sender_boot(ref, floor, follows):
    assert ref.follows(floor) is follows
