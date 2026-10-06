"""Task assignment request and response wire messages."""

from __future__ import annotations

from typing import Any, Optional

from pymavlink.dialects.v20.ardupilotmega import (
    MAVLINK_MSG_ID_TASK_ASSIGN_REQUEST,
    MAVLINK_MSG_ID_TASK_ASSIGN_RESPONSE,
    MAVLink_task_assign_request_message,
    MAVLink_task_assign_response_message,
)

from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.msg_abc import MsgABC, register_msg
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.task_message_data import TaskAssignMsgData
from navpy.modules.comm.messages.task_message_meta import (
    get_meta_fields,
    meta_from_mavlink,
)
from navpy.modules.comm.messages.types import MsgType, TaskTypeMsgData


@register_msg()
class TaskAssignRequestMsg(MsgABC):
    def __init__(
        self,
        sender_id: int,
        receiver_id: int,
        task: TaskAssignMsgData,
        meta: Optional[MsgMeta] = None,
    ) -> None:
        super().__init__(sender_id, receiver_id, meta=meta)
        self.task = task

    def to_dict(self) -> dict[str, Any]:
        data = super().to_dict()
        data["task"] = self.task.to_dict()
        return data

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TaskAssignRequestMsg":
        return cls(
            sender_id=data["sid"],
            receiver_id=data["rid"],
            task=TaskAssignMsgData.from_dict(data["task"]),
        )

    def to_mavlink(self) -> MAVLink_task_assign_request_message:
        task = self.task
        boot_id, msg_seq, time_ms, ttl_ms = get_meta_fields(self.meta)
        return MAVLink_task_assign_request_message(
            boot_id,
            msg_seq,
            time_ms,
            ttl_ms,
            self.receiver_id if self.receiver_id is not None else 0,
            task.task_id,
            task.task_type.value,
            task.class_id,
            task.location.lat,
            task.location.lng,
            task.location.alt,
        )

    @classmethod
    def from_mavlink(
        cls,
        message: MAVLink_task_assign_request_message,
    ) -> "TaskAssignRequestMsg":
        task = TaskAssignMsgData(
            task_id=message.task_id,
            task_type=TaskTypeMsgData(message.task_type),
            location=LocationMsgData(message.lat, message.lng, message.alt),
            class_id=message.class_id,
        )
        return cls(
            sender_id=message.get_srcSystem(),
            receiver_id=message.target_system,
            task=task,
            meta=meta_from_mavlink(message),
        )

    @classmethod
    def msg_type(cls) -> MsgType:
        return MsgType.TASK_ASSIGN_REQUEST

    @classmethod
    def mav_id(cls) -> int:
        return MAVLINK_MSG_ID_TASK_ASSIGN_REQUEST


@register_msg()
class TaskAssignResponseMsg(MsgABC):
    def __init__(
        self,
        sender_id: int,
        receiver_id: int,
        task_id: int,
        is_accepted: bool,
        meta: Optional[MsgMeta] = None,
    ) -> None:
        super().__init__(sender_id, receiver_id, meta=meta)
        self.task_id = task_id
        self.is_accepted = is_accepted

    def to_dict(self) -> dict[str, Any]:
        data = super().to_dict()
        data.update({"task_id": self.task_id, "is_accepted": self.is_accepted})
        return data

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TaskAssignResponseMsg":
        return cls(
            sender_id=data["sid"],
            receiver_id=data["rid"],
            task_id=data["task_id"],
            is_accepted=data["is_accepted"],
        )

    def to_mavlink(self) -> MAVLink_task_assign_response_message:
        boot_id, msg_seq, time_ms, ttl_ms = get_meta_fields(self.meta)
        return MAVLink_task_assign_response_message(
            boot_id,
            msg_seq,
            time_ms,
            ttl_ms,
            self.receiver_id if self.receiver_id is not None else 0,
            self.task_id,
            1 if self.is_accepted else 0,
        )

    @classmethod
    def from_mavlink(
        cls,
        message: MAVLink_task_assign_response_message,
    ) -> "TaskAssignResponseMsg":
        return cls(
            sender_id=message.get_srcSystem(),
            receiver_id=message.target_system,
            task_id=message.task_id,
            is_accepted=bool(message.accepted),
            meta=meta_from_mavlink(message),
        )

    @classmethod
    def msg_type(cls) -> MsgType:
        return MsgType.TASK_ASSIGN_RESPONSE

    @classmethod
    def mav_id(cls) -> int:
        return MAVLINK_MSG_ID_TASK_ASSIGN_RESPONSE


__all__ = ["TaskAssignRequestMsg", "TaskAssignResponseMsg"]
