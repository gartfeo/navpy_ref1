"""
TTL (Time-To-Live) defaults for swarm message types.

Defines how long each message type remains valid after creation.
Used by MessageFilter to drop stale messages.

Navlines:
- Heartbeats: Short TTL (1-5s), high frequency
- State updates: Medium TTL (5-10s)
- Commands/assignments: Medium TTL (2-5s) - must be acted on promptly
- Status/info: Longer TTL (10-30s) - informational, less time-critical
"""

from navpy.modules.comm.messages.types import MsgType

# Default TTL for message types not explicitly listed (5 seconds)
DEFAULT_TTL_MS = 5000

# TTL defaults per message type in milliseconds
TTL_DEFAULTS = {
    # Heartbeats - short validity, high frequency
    MsgType.SWARM_HEARTBEAT: 5000,    # 5s TTL, expected rate 1Hz
    MsgType.SLOT_HEARTBEAT: 2000,     # 2s TTL, expected rate 3Hz

    # Peer presence
    MsgType.CHECK_IN: 5000,           # 5s - announcement of joining
    MsgType.CHECK_OUT: 10000,         # 10s - departure notification

    # Task coordination - need timely processing
    MsgType.AVAILABLE_TASK_REQUEST: 5000,    # 5s
    MsgType.AVAILABLE_TASK_RESPONSE: 5000,   # 5s
    MsgType.TASK_ASSIGN_REQUEST: 5000,       # 5s
    MsgType.TASK_ASSIGN_RESPONSE: 5000,      # 5s
    MsgType.TASK_CONFIRM_REQUEST: 5000,      # 5s
    MsgType.TASK_CONFIRM_RESPONSE: 5000,     # 5s

    # Slot management
    MsgType.SLOT_CLAIM: 2000,         # 2s - claims must be timely
    MsgType.VOTE_PHASE: 6000,         # 6s - voting rounds

    # Status/info - less time critical
    MsgType.SEARCH_STATUS: 15000,     # 15s
    MsgType.LOG_STATUS: 30000,        # 30s - logs can be older

    # Reliability toolkit (D-07/D-25/D-26) - short TTL matching the 2.0s resend cadence
    MsgType.SWARM_ACK: 3000,          # 3s
    MsgType.SWARM_REQUEST: 3000,      # 3s

    # Unknown/fallback
    MsgType.UNKNOWN: DEFAULT_TTL_MS,
}


def get_ttl_ms(msg_type: MsgType) -> int:
    """
    Get TTL in milliseconds for a message type.

    Args:
        msg_type: The message type

    Returns:
        TTL in milliseconds from defaults table, or DEFAULT_TTL_MS if not found
    """
    return TTL_DEFAULTS.get(msg_type, DEFAULT_TTL_MS)
