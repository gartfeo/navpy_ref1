"""Task confirmation request and response wire messages."""

from __future__ import annotations

from typing import Any, Optional

from pymavlink.dialects.v20.ardupilotmega import (
    MAVLINK_MSG_ID_TASK_CONFIRM_REQUEST,
    MAVLINK_MSG_ID_TASK_CONFIRM_RESPONSE,
    MAVLink_task_confirm_request_message,
    MAVLink_task_confirm_response_message,
)

from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.msg_abc import MsgABC, register_msg
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.task_message_data import TaskMsgData
from navpy.modules.comm.messages.task_message_meta import (
    get_meta_fields,
    meta_from_mavlink,
)
from navpy.modules.comm.messages.types import MsgType, TaskTypeMsgData


@register_msg()
class TaskConfirmRequestMsg(MsgABC):
    def __init__(
        self,
        sender_id: int,
        task: TaskMsgData,
        meta: Optional[MsgMeta] = None,
    ) -> None:
        super().__init__(sender_id, 0, meta=meta)
        self.task = task

    def to_dict(self) -> dict[str, Any]:
        data = super().to_dict()
        data["t"] = self.task.to_dict()
        return data

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TaskConfirmRequestMsg":
        return cls(
            sender_id=data["sid"],
            task=TaskMsgData.from_dict(data["t"]),
            meta=MsgMeta.from_dict(data),
        )

    def to_mavlink(self) -> MAVLink_task_confirm_request_message:
        task = self.task
        boot_id, msg_seq, time_ms, ttl_ms = get_meta_fields(self.meta)
        return MAVLink_task_confirm_request_message(
            boot_id,
            msg_seq,
            time_ms,
            ttl_ms,
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
        message: MAVLink_task_confirm_request_message,
    ) -> "TaskConfirmRequestMsg":
        task = TaskMsgData(
            task_id=message.task_id,
            task_type=TaskTypeMsgData(message.task_type),
            location=LocationMsgData(message.lat, message.lng, message.alt),
            class_id=message.class_id,
        )
        return cls(
            sender_id=message.get_srcSystem(),
            task=task,
            meta=meta_from_mavlink(message),
        )

    @classmethod
    def msg_type(cls) -> MsgType:
        return MsgType.TASK_CONFIRM_REQUEST

    @classmethod
    def mav_id(cls) -> int:
        return MAVLINK_MSG_ID_TASK_CONFIRM_REQUEST


@register_msg()
class TaskConfirmResponseMsg(MsgABC):
    def __init__(
        self,
        receiver_id: int,
        task_id: int,
        is_confirmed: bool,
        meta: Optional[MsgMeta] = None,
    ) -> None:
        super().__init__(0, receiver_id, meta=meta)
        self.task_id = task_id
        self.is_confirmed = is_confirmed

    def to_dict(self) -> dict[str, Any]:
        data = super().to_dict()
        data.update({"tid": self.task_id, "c": self.is_confirmed})
        return data

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TaskConfirmResponseMsg":
        return cls(
            receiver_id=data["rid"],
            task_id=data["tid"],
            is_confirmed=data["c"],
            meta=MsgMeta.from_dict(data),
        )

    def get_msg_uid(self) -> Optional[tuple]:
        uid = super().get_msg_uid()
        if uid is None:
            return None
        return (*uid, self.is_confirmed)

    def to_mavlink(self) -> MAVLink_task_confirm_response_message:
        boot_id, msg_seq, time_ms, ttl_ms = get_meta_fields(self.meta)
        return MAVLink_task_confirm_response_message(
            boot_id,
            msg_seq,
            time_ms,
            ttl_ms,
            self.receiver_id,
            self.task_id,
            1 if self.is_confirmed else 0,
        )

    @classmethod
    def from_mavlink(
        cls,
        message: MAVLink_task_confirm_response_message,
    ) -> "TaskConfirmResponseMsg":
        return cls(
            receiver_id=message.target_system,
            task_id=message.task_id,
            is_confirmed=bool(message.confirmed),
            meta=meta_from_mavlink(message),
        )

    @classmethod
    def msg_type(cls) -> MsgType:
        return MsgType.TASK_CONFIRM_RESPONSE

    @classmethod
    def mav_id(cls) -> int:
        return MAVLINK_MSG_ID_TASK_CONFIRM_RESPONSE


__all__ = ["TaskConfirmRequestMsg", "TaskConfirmResponseMsg"]
