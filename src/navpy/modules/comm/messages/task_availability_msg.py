"""Task availability request and response wire messages."""

from __future__ import annotations

from typing import Any, Optional

from pymavlink.dialects.v20.ardupilotmega import (
    MAVLINK_MSG_ID_AVAILABLE_TASK_REQUEST,
    MAVLINK_MSG_ID_AVAILABLE_TASK_RESPONSE,
    MAVLink_available_task_request_message,
    MAVLink_available_task_response_message,
)

from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.msg_abc import MsgABC, register_msg
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.task_message_data import (
    TaskHandleMsgData,
    TaskMsgData,
)
from navpy.modules.comm.messages.task_message_meta import (
    get_meta_fields,
    meta_from_mavlink,
)
from navpy.modules.comm.messages.types import MsgType, TaskTypeMsgData


@register_msg()
class AvailableTaskRequestMsg(MsgABC):
    def __init__(
        self,
        sender_id: int,
        tasks: list[TaskMsgData],
        meta: Optional[MsgMeta] = None,
    ) -> None:
        super().__init__(sender_id, meta=meta)
        self.tasks = tasks

    def to_dict(self) -> dict[str, Any]:
        data = super().to_dict()
        data["tasks"] = [task.to_dict() for task in self.tasks]
        return data

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "AvailableTaskRequestMsg":
        return cls(
            sender_id=data["sid"],
            tasks=[TaskMsgData.from_dict(task) for task in data.get("tasks", [])],
        )

    def to_mavlink(self) -> MAVLink_available_task_request_message:
        count = min(len(self.tasks), 5)
        ids = [0] * 5
        types = [0] * 5
        class_ids = [0] * 5
        lats = [0.0] * 5
        lngs = [0.0] * 5
        alts = [0.0] * 5
        for index, task in enumerate(self.tasks[:5]):
            ids[index] = task.task_id
            types[index] = task.task_type.value
            class_ids[index] = task.class_id
            lats[index] = task.location.lat
            lngs[index] = task.location.lng
            alts[index] = task.location.alt
        boot_id, msg_seq, time_ms, ttl_ms = get_meta_fields(self.meta)
        return MAVLink_available_task_request_message(
            boot_id,
            msg_seq,
            time_ms,
            ttl_ms,
            count,
            ids,
            types,
            class_ids,
            lats,
            lngs,
            alts,
        )

    @classmethod
    def from_mavlink(
        cls,
        message: MAVLink_available_task_request_message,
    ) -> "AvailableTaskRequestMsg":
        tasks = []
        for index in range(message.count):
            tasks.append(TaskMsgData(
                task_id=message.task_id[index],
                task_type=TaskTypeMsgData(message.task_type[index]),
                location=LocationMsgData(
                    message.lat[index],
                    message.lng[index],
                    message.alt[index],
                ),
                class_id=message.class_id[index],
            ))
        return cls(
            sender_id=message.get_srcSystem(),
            tasks=tasks,
            meta=meta_from_mavlink(message),
        )

    @classmethod
    def msg_type(cls) -> MsgType:
        return MsgType.AVAILABLE_TASK_REQUEST

    @classmethod
    def mav_id(cls) -> int:
        return MAVLINK_MSG_ID_AVAILABLE_TASK_REQUEST


@register_msg()
class AvailableTaskResponseMsg(MsgABC):
    def __init__(
        self,
        sender_id: int,
        receiver_id: int,
        tasks: list[TaskHandleMsgData],
        meta: Optional[MsgMeta] = None,
    ) -> None:
        super().__init__(sender_id, receiver_id, meta=meta)
        self.tasks = tasks

    def to_dict(self) -> dict[str, Any]:
        data = super().to_dict()
        data["tasks"] = [task.to_dict() for task in self.tasks]
        return data

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "AvailableTaskResponseMsg":
        return cls(
            sender_id=data["sid"],
            receiver_id=data["rid"],
            tasks=[
                TaskHandleMsgData.from_dict(task)
                for task in data.get("tasks", [])
            ],
        )

    def to_mavlink(self) -> MAVLink_available_task_response_message:
        count = min(len(self.tasks), 5)
        ids = [0] * 5
        times = [0.0] * 5
        for index, task in enumerate(self.tasks[:5]):
            ids[index] = task.task_id
            times[index] = task.time_in_min
        boot_id, msg_seq, time_ms, ttl_ms = get_meta_fields(self.meta)
        return MAVLink_available_task_response_message(
            boot_id,
            msg_seq,
            time_ms,
            ttl_ms,
            self.receiver_id if self.receiver_id is not None else 0,
            count,
            ids,
            times,
        )

    @classmethod
    def from_mavlink(
        cls,
        message: MAVLink_available_task_response_message,
    ) -> "AvailableTaskResponseMsg":
        tasks = [
            TaskHandleMsgData(message.task_id[index], message.time[index])
            for index in range(message.count)
        ]
        return cls(
            sender_id=message.get_srcSystem(),
            receiver_id=message.target_system,
            tasks=tasks,
            meta=meta_from_mavlink(message),
        )

    @classmethod
    def msg_type(cls) -> MsgType:
        return MsgType.AVAILABLE_TASK_RESPONSE

    @classmethod
    def mav_id(cls) -> int:
        return MAVLINK_MSG_ID_AVAILABLE_TASK_RESPONSE


__all__ = ["AvailableTaskRequestMsg", "AvailableTaskResponseMsg"]
