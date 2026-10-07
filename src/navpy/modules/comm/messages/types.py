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
    Enumeration of task types carried in the navlink ``task_type`` field.

    The detector publishes a single class (``dock``), so a detected POI maps to
    one task type. Wire values are stable: ``DOCK`` reuses 1 and ``UNKNOWN``
    keeps 5.
    """
    DOCK = 1
    UNKNOWN = 5  # Undefined types


def class_to_task_type(class_id: int) -> TaskTypeMsgData:
    """Map a detection class_id to TaskTypeMsgData (``dock`` -> DOCK, else UNKNOWN)."""
    # Lazy import: the vision package imports comm, so a module-level import cycles.
    from navpy.modules.vision.vision_class_profile import DOCK_DETECT_CLASS_ID

    if class_id == DOCK_DETECT_CLASS_ID:
        return TaskTypeMsgData.DOCK
    return TaskTypeMsgData.UNKNOWN


class TaskDispatchStatus(Enum):
    """
    Enumeration of task dispatch statuses.
    """
    AVAILABLE = 0
    CONFIRMING = 20
    CONFIRMED = 40
