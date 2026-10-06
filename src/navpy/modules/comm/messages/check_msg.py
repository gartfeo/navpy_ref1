from typing import Dict, Any, Optional

from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink_check_in_message,
    MAVLink_check_out_message,
    MAVLINK_MSG_ID_CHECK_OUT,
    MAVLINK_MSG_ID_CHECK_IN,
)

from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.msg_abc import register_msg, MsgABC
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.types import MsgType


@register_msg()
class CheckInMsg(MsgABC):
    """
    Message class for vehicle check-in.
    Includes dedup/TTL metadata for swarm messaging.
    """

    def __init__(self, sender_id: int, meta: Optional[MsgMeta] = None):
        super().__init__(sender_id, meta=meta)

    def to_dict(self) -> Dict[str, Any]:
        return super().to_dict()

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'CheckInMsg':
        return cls(data['sid'])

    def to_mavlink(self) -> MAVLink_check_in_message:
        # Include meta fields if present, otherwise use defaults
        boot_id = self.meta.boot_id if self.meta else 0
        msg_seq = self.meta.msg_seq if self.meta else 0
        time_ms = self.meta.time_ms if self.meta else 0
        ttl_ms = self.meta.ttl_ms if self.meta else 0
        return MAVLink_check_in_message(boot_id, msg_seq, time_ms, ttl_ms)

    @classmethod
    def from_mavlink(cls, mav_msg: MAVLink_check_in_message) -> 'CheckInMsg':
        meta = MsgMeta(
            boot_id=mav_msg.boot_id,
            msg_seq=mav_msg.msg_seq,
            time_ms=mav_msg.time_ms,
            ttl_ms=mav_msg.ttl_ms,
        )
        return cls(sender_id=mav_msg.get_srcSystem(), meta=meta)

    @classmethod
    def msg_type(cls) -> MsgType:
        return MsgType.CHECK_IN

    @classmethod
    def mav_id(cls) -> int:
        return MAVLINK_MSG_ID_CHECK_IN


@register_msg()
class CheckOutMsg(MsgABC):
    """
    Message class for vehicle check-out.
    Includes dedup/TTL metadata for swarm messaging.
    """

    def __init__(self, sender_id, location: LocationMsgData, meta: Optional[MsgMeta] = None):
        super().__init__(sender_id, meta=meta)
        self.location = location

    def to_dict(self) -> Dict[str, Any]:
        data = super().to_dict()
        data.update({
            'location': self.location.to_dict(),
        })
        return data

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'CheckOutMsg':
        location = LocationMsgData.from_dict(data['location'])
        return cls(sender_id=data['sid'], location=location)

    def to_mavlink(self) -> MAVLink_check_out_message:
        loc = self.location
        # Include meta fields if present, otherwise use defaults
        boot_id = self.meta.boot_id if self.meta else 0
        msg_seq = self.meta.msg_seq if self.meta else 0
        time_ms = self.meta.time_ms if self.meta else 0
        ttl_ms = self.meta.ttl_ms if self.meta else 0
        return MAVLink_check_out_message(boot_id, msg_seq, time_ms, ttl_ms, loc.lat, loc.lng, loc.alt)

    @classmethod
    def from_mavlink(cls, mav_msg: MAVLink_check_out_message) -> 'CheckOutMsg':
        loc = LocationMsgData(lat=mav_msg.lat, lng=mav_msg.lng, alt=mav_msg.alt)
        meta = MsgMeta(
            boot_id=mav_msg.boot_id,
            msg_seq=mav_msg.msg_seq,
            time_ms=mav_msg.time_ms,
            ttl_ms=mav_msg.ttl_ms,
        )
        return cls(sender_id=mav_msg.get_srcSystem(), location=loc, meta=meta)

    @classmethod
    def msg_type(cls) -> MsgType:
        return MsgType.CHECK_OUT

    @classmethod
    def mav_id(cls) -> int:
        return MAVLINK_MSG_ID_CHECK_OUT
