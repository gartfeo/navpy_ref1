"""
SwarmRequestMsg - navpy wrapper for the SWARM_REQUEST navlink message (id 25111).

Generic on-demand request: retransmit / resource (e.g. thumbnail) / force-confirm.
Subsumes the old ThumbnailRequestMsg + ForceConfirmMsg TUNNEL-subtype designs
(D-25). Mirrors the TaskConfirmResponseMsg pattern (available_task_msg.py)
exactly: receiver_id maps to the native target_system field, meta carries the
boot_id/msg_seq/time_ms/ttl_ms dedup/TTL fields.

Import this module where SwarmRequestMsg is produced or consumed so
@register_msg runs (navpy has no central message loader; each consumer
imports its own message module).
"""
from typing import Any, Dict, Optional

from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink_swarm_request_message,
    MAVLINK_MSG_ID_SWARM_REQUEST,
)

from navpy.modules.comm.messages.msg_abc import register_msg, MsgABC
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.types import MsgType

# SWARM_REQUEST.request_type values.
REQUEST_TYPE_RETRANSMIT = 0
REQUEST_TYPE_RESOURCE = 1
REQUEST_TYPE_FORCE_CONFIRM = 2

# SWARM_REQUEST.subject_type values, valid when request_type == REQUEST_TYPE_RESOURCE.
SUBJECT_TYPE_THUMBNAIL = 1


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
class SwarmRequestMsg(MsgABC):
    """
    Generic on-demand request: retransmit / resource / force-confirm.
    Includes dedup/TTL metadata for swarm messaging.
    """

    def __init__(self, sender_id: int, receiver_id: int, request_type: int, subject_type: int,
                 subject_id: int, meta: Optional[MsgMeta] = None):
        super().__init__(sender_id, receiver_id, meta=meta)
        self.request_type = request_type
        self.subject_type = subject_type
        self.subject_id = subject_id

    def to_dict(self) -> Dict[str, Any]:
        data = super().to_dict()
        data.update({
            'rqt': self.request_type,
            'sjt': self.subject_type,
            'sji': self.subject_id,
        })
        return data

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'SwarmRequestMsg':
        return cls(
            sender_id=data['sid'],
            receiver_id=data['rid'],
            request_type=data['rqt'],
            subject_type=data['sjt'],
            subject_id=data['sji'],
        )

    def to_mavlink(self) -> MAVLink_swarm_request_message:
        boot_id, msg_seq, time_ms, ttl_ms = _get_meta_fields(self.meta)
        return MAVLink_swarm_request_message(
            boot_id, msg_seq, time_ms, ttl_ms,
            self.receiver_id if self.receiver_id is not None else 0,
            self.request_type, self.subject_type, self.subject_id,
        )

    @classmethod
    def from_mavlink(cls, mav_msg: MAVLink_swarm_request_message) -> 'SwarmRequestMsg':
        meta = _create_meta_from_mavlink(mav_msg)
        return cls(
            sender_id=mav_msg.get_srcSystem(),
            receiver_id=mav_msg.target_system,
            request_type=mav_msg.request_type,
            subject_type=mav_msg.subject_type,
            subject_id=mav_msg.subject_id,
            meta=meta,
        )

    @classmethod
    def msg_type(cls) -> MsgType:
        return MsgType.SWARM_REQUEST

    @classmethod
    def mav_id(cls) -> int:
        return MAVLINK_MSG_ID_SWARM_REQUEST
