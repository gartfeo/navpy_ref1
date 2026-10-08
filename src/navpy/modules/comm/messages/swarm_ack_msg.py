"""
SwarmAckMsg - navpy wrapper for the SWARM_ACK navlink message (id 25110).

Redefines the reserved 25110 slot (formerly TASK_CONFIRM_ACK) as a generic,
message-agnostic acknowledgement: it acks any message by that message's dedup
UID (sender_id, boot_id, msg_seq), not just confirm-requests.

The swarm task-assignment handshake sends and consumes it (TaskMessageSender
.ack, TaskAckRouter; docs/design/swarm-task-assignment-ack.md): an ack lives
as long as the message it acknowledges.

Mirrors the TaskConfirmResponseMsg pattern (available_task_msg.py) exactly:
receiver_id maps to the native target_system field, meta carries the
boot_id/msg_seq/time_ms/ttl_ms dedup/TTL fields.

Import this module where SwarmAckMsg is produced or consumed so @register_msg
runs (navpy has no central message loader; each consumer imports its own
message module).
"""
from typing import Any, Dict, Optional

from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink_swarm_ack_message,
    MAVLINK_MSG_ID_SWARM_ACK,
)

from navpy.modules.comm.messages.msg_abc import register_msg, MsgABC
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.types import MsgType

# SWARM_ACK.status values (0=received, 1=processing, 2=applied).
ACK_STATUS_RECEIVED = 0
ACK_STATUS_PROCESSING = 1
ACK_STATUS_APPLIED = 2


def _get_meta_fields(meta: Optional[MsgMeta]) -> tuple:
    """Helper to extract meta fields with defaults."""
    if meta:
        return meta.boot_id, meta.msg_seq, meta.time_ms, meta.ttl_ms
    return 0, 0, 0, 0


def _create_meta_from_mavlink(mav_msg) -> MsgMeta:
    """Helper to create MsgMeta from MAVLink message with meta fields."""
    return MsgMeta(
        boot_id=mav_msg.boot_id,
        msg_seq=mav_msg.msg_seq,
        time_ms=mav_msg.time_ms,
        ttl_ms=mav_msg.ttl_ms,
    )


@register_msg()
class SwarmAckMsg(MsgABC):
    """
    Generic acknowledgement of any message by its dedup UID.
    Includes dedup/TTL metadata for swarm messaging.
    """

    def __init__(self, sender_id: int, receiver_id: int, ref_boot_id: int, ref_msg_seq: int,
                 ref_msg_type: int, status: int, meta: Optional[MsgMeta] = None):
        super().__init__(sender_id, receiver_id, meta=meta)
        self.ref_boot_id = ref_boot_id
        self.ref_msg_seq = ref_msg_seq
        self.ref_msg_type = ref_msg_type
        self.status = status

    def to_dict(self) -> Dict[str, Any]:
        data = super().to_dict()
        data.update({
            'rbid': self.ref_boot_id,
            'rseq': self.ref_msg_seq,
            'rmt': self.ref_msg_type,
            'st': self.status,
        })
        return data

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'SwarmAckMsg':
        return cls(
            sender_id=data['sid'],
            receiver_id=data['rid'],
            ref_boot_id=data['rbid'],
            ref_msg_seq=data['rseq'],
            ref_msg_type=data['rmt'],
            status=data['st'],
        )

    def to_mavlink(self) -> MAVLink_swarm_ack_message:
        boot_id, msg_seq, time_ms, ttl_ms = _get_meta_fields(self.meta)
        return MAVLink_swarm_ack_message(
            boot_id, msg_seq, time_ms, ttl_ms,
            self.receiver_id if self.receiver_id is not None else 0,
            self.ref_boot_id, self.ref_msg_seq, self.ref_msg_type, self.status,
        )

    @classmethod
    def from_mavlink(cls, mav_msg: MAVLink_swarm_ack_message) -> 'SwarmAckMsg':
        meta = _create_meta_from_mavlink(mav_msg)
        return cls(
            sender_id=mav_msg.get_srcSystem(),
            receiver_id=mav_msg.target_system,
            ref_boot_id=mav_msg.ref_boot_id,
            ref_msg_seq=mav_msg.ref_msg_seq,
            ref_msg_type=mav_msg.ref_msg_type,
            status=mav_msg.status,
            meta=meta,
        )

    @classmethod
    def msg_type(cls) -> MsgType:
        return MsgType.SWARM_ACK

    @classmethod
    def mav_id(cls) -> int:
        return MAVLINK_MSG_ID_SWARM_ACK
