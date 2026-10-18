"""The camera-protocol requests a companion admits from any system.

A ground station asks NavPy's camera components what they stream with
MAV_CMD_REQUEST_MESSAGE (docs/design/gcs_video_streams.md, section 1). Only
these requests pass the inbound router's own-system rule; every other message
from a foreign system stays dropped.
"""

from __future__ import annotations

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_REQUEST_MESSAGE,
    MAVLINK_MSG_ID_CAMERA_INFORMATION,
    MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION,
    MAVLink_message,
)

from navpy.plugins.vehicle.ardupilot.mavlink_camera_components import (
    GIMBAL_DEVICE_ID_BY_CAMERA_COMPONENT,
)

CAMERA_REQUESTABLE_MESSAGE_IDS = frozenset({
    MAVLINK_MSG_ID_CAMERA_INFORMATION,
    MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION,
})


def requested_message_id(message: MAVLink_message) -> int | None:
    """The message id a REQUEST_MESSAGE command asks for; None otherwise."""
    if message.get_type() != "COMMAND_LONG":
        return None
    if getattr(message, "command", None) != MAV_CMD_REQUEST_MESSAGE:
        return None
    param1 = getattr(message, "param1", None)
    if not isinstance(param1, (int, float)) or not float(param1).is_integer():
        return None
    return int(param1)


def is_camera_stream_request(message: MAVLink_message, own_system: int) -> bool:
    """True for REQUEST_MESSAGE 259/269 to one of this system's cameras."""
    return (
        requested_message_id(message) in CAMERA_REQUESTABLE_MESSAGE_IDS
        and getattr(message, "target_system", None) == own_system
        and getattr(message, "target_component", None)
        in GIMBAL_DEVICE_ID_BY_CAMERA_COMPONENT
    )


__all__ = [
    "CAMERA_REQUESTABLE_MESSAGE_IDS",
    "is_camera_stream_request",
    "requested_message_id",
]
