"""Compatible fleet task records for availability, assignment and confirmation.

Task/session correlation fields are not authenticated recipient identities.
Keep serialized keys and enums stable across network peers.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.msg_abc import MsgDataAbc
from navpy.modules.comm.messages.types import TaskTypeMsgData


@dataclass
class TaskMsgData(MsgDataAbc):
    task_id: int
    task_type: TaskTypeMsgData
    location: LocationMsgData
    class_id: int = 0

    def to_dict(self) -> dict[str, Any]:
        return {
            "tid": self.task_id,
            "tt": self.task_type.name,
            "l": self.location.to_dict(),
            "cid": self.class_id,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TaskMsgData":
        return cls(
            data["tid"],
            TaskTypeMsgData[data.get("tt", "UNKNOWN")],
            LocationMsgData.from_dict(data["l"]),
            class_id=data.get("cid", 0),
        )


@dataclass
class TaskHandleMsgData(MsgDataAbc):
    task_id: int
    time_in_min: float

    def to_dict(self) -> dict[str, Any]:
        return {"task_id": self.task_id, "time_in_min": self.time_in_min}

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TaskHandleMsgData":
        return cls(task_id=data["task_id"], time_in_min=data["time_in_min"])


@dataclass
class TaskAssignMsgData(MsgDataAbc):
    task_id: int
    task_type: TaskTypeMsgData
    location: LocationMsgData
    class_id: int = 0

    def to_dict(self) -> dict[str, Any]:
        return {
            "task_id": self.task_id,
            "task_type": self.task_type.name,
            "location": self.location.to_dict(),
            "class_id": self.class_id,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TaskAssignMsgData":
        return cls(
            task_id=data["task_id"],
            task_type=TaskTypeMsgData[data.get("task_type", "UNKNOWN")],
            location=LocationMsgData.from_dict(data["location"]),
            class_id=data.get("class_id", 0),
        )


__all__ = ["TaskAssignMsgData", "TaskHandleMsgData", "TaskMsgData"]
