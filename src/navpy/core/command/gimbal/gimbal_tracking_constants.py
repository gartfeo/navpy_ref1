"""Shared constants for gimbal tracking session capabilities."""

MODE_LOCK = 0
MODE_FOLLOW = 1
ARMED_ANY_OBJ_ID = -1
GIMBAL_COMMAND_ERRORS = (OSError,)

__all__ = [
    "ARMED_ANY_OBJ_ID",
    "GIMBAL_COMMAND_ERRORS",
    "MODE_FOLLOW",
    "MODE_LOCK",
]
