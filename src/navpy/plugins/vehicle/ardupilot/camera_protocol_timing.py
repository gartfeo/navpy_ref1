"""Timing shared by NavPy's camera components and the GCS camera discovery.

MAVLink leaves heartbeat timing to the channel, so these are our radio-link
defaults, not protocol values (docs/design/gcs_video_streams.md, section 1).
"""

# Camera HEARTBEAT period; the rate QGC and PX4 use.
CAMERA_HEARTBEAT_PERIOD_S = 1.0

# A camera is lost after five missed heartbeats; its streams are removed.
CAMERA_LOST_AFTER_S = 5.0 * CAMERA_HEARTBEAT_PERIOD_S

# One request send fails when its ACK or its payload has not arrived in this
# time: above the radio round trip plus the companion's loop. Check against
# the measured link RTT on hardware.
CAMERA_REQUEST_TIMEOUT_S = 2.0

# Sends per request: the camera protocol's own retry rule.
CAMERA_REQUEST_MAX_SENDS = 3

__all__ = [
    "CAMERA_HEARTBEAT_PERIOD_S",
    "CAMERA_LOST_AFTER_S",
    "CAMERA_REQUEST_MAX_SENDS",
    "CAMERA_REQUEST_TIMEOUT_S",
]
