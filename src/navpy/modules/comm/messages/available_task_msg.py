"""Compatibility facade for task protocol message classes.

Concrete owners are split by protocol responsibility.  Re-exporting the
objects here preserves every historical import path and class identity.
"""

from navpy.modules.comm.messages.task_assignment_msg import (
    TaskAssignRequestMsg,
    TaskAssignResponseMsg,
)
from pymavlink.dialects.v20.ardupilotmega import (
    MAVLINK_MSG_ID_AVAILABLE_TASK_REQUEST,
    MAVLINK_MSG_ID_AVAILABLE_TASK_RESPONSE,
    MAVLINK_MSG_ID_TASK_ASSIGN_REQUEST,
    MAVLINK_MSG_ID_TASK_ASSIGN_RESPONSE,
    MAVLINK_MSG_ID_TASK_CONFIRM_REQUEST,
    MAVLINK_MSG_ID_TASK_CONFIRM_RESPONSE,
)
from navpy.modules.comm.messages.task_availability_msg import (
    AvailableTaskRequestMsg,
    AvailableTaskResponseMsg,
)
from navpy.modules.comm.messages.task_confirmation_msg import (
    TaskConfirmRequestMsg,
    TaskConfirmResponseMsg,
)
from navpy.modules.comm.messages.task_message_data import (
    TaskAssignMsgData,
    TaskHandleMsgData,
    TaskMsgData,
)

__all__ = [
    "AvailableTaskRequestMsg",
    "AvailableTaskResponseMsg",
    "MAVLINK_MSG_ID_AVAILABLE_TASK_REQUEST",
    "MAVLINK_MSG_ID_AVAILABLE_TASK_RESPONSE",
    "MAVLINK_MSG_ID_TASK_ASSIGN_REQUEST",
    "MAVLINK_MSG_ID_TASK_ASSIGN_RESPONSE",
    "MAVLINK_MSG_ID_TASK_CONFIRM_REQUEST",
    "MAVLINK_MSG_ID_TASK_CONFIRM_RESPONSE",
    "TaskAssignMsgData",
    "TaskAssignRequestMsg",
    "TaskAssignResponseMsg",
    "TaskConfirmRequestMsg",
    "TaskConfirmResponseMsg",
    "TaskHandleMsgData",
    "TaskMsgData",
]
