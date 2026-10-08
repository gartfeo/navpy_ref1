"""
Swarm Heartbeat Message for peer presence and clock synchronization.

The SwarmHeartbeatMsg is critical for:
1. Peer liveness detection
2. Clock offset estimation between peers (for TTL calculations)
3. Deduplication (contains boot_id, msg_seq like all swarm messages)

Expected rate: 1Hz
TTL: 5000ms

The ``state`` byte carries SwarmNodeState: whether the node can take part in
a task auction (docs/design/swarm-task-assignment-ack.md, "Busy UAVs").
"""

from enum import IntEnum
from typing import Dict, Any, Optional

from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink_swarm_heartbeat_message,
    MAVLINK_MSG_ID_SWARM_HEARTBEAT,
)

from navpy.modules.comm.messages.msg_abc import register_msg, MsgABC
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.types import MsgType


class SwarmNodeState(IntEnum):
    """Heartbeat state codes; receivers treat any non-zero code as BUSY.

    BUSY: the node holds another UAV's task or flies a final approach.
    """

    FREE = 0
    BUSY = 1

    @classmethod
    def from_code(cls, code: int) -> "SwarmNodeState":
        return cls.FREE if code == cls.FREE else cls.BUSY


@register_msg()
class SwarmHeartbeatMsg(MsgABC):
    """
    Swarm heartbeat message for peer presence and clock synchronization.

    Contains:
    - sender_id: The sending node's system ID
    - state: SwarmNodeState code (FREE=0, BUSY=1)
    - boot_id, msg_seq, time_ms, ttl_ms: Via meta field (required for this message)

    The time_ms field in meta is used by receivers to compute clock offset.
    """

    def __init__(self, sender_id: int, state: int = 0,
                 meta: Optional[MsgMeta] = None):
        """
        Initialize swarm heartbeat message.

        Args:
            sender_id: The sending node's system ID
            state: SwarmNodeState code (default 0 = FREE)
            meta: Message metadata (required for clock sync)
        """
        super().__init__(sender_id, receiver_id=None, meta=meta)
        self.state = state

    def to_dict(self) -> Dict[str, Any]:
        """Serialize to dictionary."""
        data = super().to_dict()
        data.update({
            "st": self.state,
        })
        return data

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SwarmHeartbeatMsg":
        """Deserialize from dictionary."""
        return cls(
            sender_id=data["sid"],
            state=data.get("st", 0),
        )

    def to_mavlink(self) -> MAVLink_swarm_heartbeat_message:
        """Serialize to MAVLink message."""
        boot_id = self.meta.boot_id if self.meta else 0
        msg_seq = self.meta.msg_seq if self.meta else 0
        time_ms = self.meta.time_ms if self.meta else 0
        ttl_ms = self.meta.ttl_ms if self.meta else 0
        return MAVLink_swarm_heartbeat_message(
            boot_id, msg_seq, time_ms, ttl_ms, self.state
        )

    @classmethod
    def from_mavlink(cls, mav_msg: MAVLink_swarm_heartbeat_message) -> "SwarmHeartbeatMsg":
        """Deserialize from MAVLink message."""
        meta = MsgMeta(
            boot_id=mav_msg.boot_id,
            msg_seq=mav_msg.msg_seq,
            time_ms=mav_msg.time_ms,
            ttl_ms=mav_msg.ttl_ms,
        )
        return cls(
            sender_id=mav_msg.get_srcSystem(),
            state=mav_msg.state,
            meta=meta,
        )

    @classmethod
    def msg_type(cls) -> MsgType:
        return MsgType.SWARM_HEARTBEAT

    @classmethod
    def mav_id(cls) -> int:
        return MAVLINK_MSG_ID_SWARM_HEARTBEAT

    @classmethod
    def create_with_meta(cls, sender_id: int, state: int = 0) -> "SwarmHeartbeatMsg":
        """
        Factory method to create heartbeat with metadata auto-populated.

        This is the preferred way to create heartbeat messages.

        Args:
            sender_id: The sending node's system ID
            state: Current node state code

        Returns:
            SwarmHeartbeatMsg with metadata attached
        """
        msg = cls(sender_id=sender_id, state=state)
        msg.set_meta_from_provider()
        return msg
