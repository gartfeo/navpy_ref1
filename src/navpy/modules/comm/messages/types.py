from enum import Enum, unique


@unique
class MsgType(Enum):
    """
    Enumeration of message types used in the messaging system.
    """
    CHECK_IN = 10
    CHECK_OUT = 20
    SWARM_HEARTBEAT = 30
    AVAILABLE_TASK_REQUEST = 100
    AVAILABLE_TASK_RESPONSE = 110
    TASK_ASSIGN_REQUEST = 130
    TASK_ASSIGN_RESPONSE = 140
    TASK_CONFIRM_REQUEST = 150
    TASK_CONFIRM_RESPONSE = 160
    SLOT_HEARTBEAT = 200
    SLOT_CLAIM = 210
    VOTE_PHASE = 220
    SEARCH_STATUS = 300
    LOG_STATUS = 1000
    SWARM_ACK = 170          # Generic ack of any message by dedup UID (navlink 25110; no sender today)
    SWARM_REQUEST = 180      # Generic retransmit/resource/force-confirm request (navlink 25111)
    UNKNOWN = 0  # Unrecognized message types


@unique
class TaskTypeMsgData(Enum):
    """
    Enumeration of task types.
    """
    SMALL = 1
    MEDIUM = 2
    BIG = 3
    HEAVY = 4
    UNKNOWN = 5  # Undefined types


_CLASS_TO_TASK_TYPE = {
    0: TaskTypeMsgData.HEAVY,    # Detection class 0
    1: TaskTypeMsgData.BIG,      # Detection class 1
    2: TaskTypeMsgData.BIG,      # Detection class 2
    3: TaskTypeMsgData.MEDIUM,   # Detection class 3
    4: TaskTypeMsgData.SMALL,    # Detection class 4
}


def class_to_task_type(class_id: int) -> TaskTypeMsgData:
    """Map detection class_id to TaskTypeMsgData."""
    return _CLASS_TO_TASK_TYPE.get(class_id, TaskTypeMsgData.BIG)


class TaskDispatchStatus(Enum):
    """
    Enumeration of task dispatch statuses.
    """
    AVAILABLE = 0
    CONFIRMING = 20
    CONFIRMED = 40
