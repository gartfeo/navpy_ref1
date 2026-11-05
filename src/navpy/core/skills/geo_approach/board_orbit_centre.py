"""The waiting orbit's centre from the own board pose (HITL step 22).

Before NAV, acquisition may place the orbit with the aircraft's GPS position
(owner ruling, 2026-10-07). The board centre in the camera frame (solvePnP)
goes through the gimbal and aircraft attitude at the capture instant to NED,
and from the camera location at that instant to a geodetic point. Like the
height-zero ground ray it replaces (which needs no board but takes the dock
to sit on the terrain), the result is home-relative, so the orbit planner
compares it with the aircraft's home-relative position on every path.
Nothing here is stored where NAV's law can read it.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
import pymap3d

from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.location import Location
from navpy.core.ports.board_pose import BoardPoseRejection
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.ports.gimbal_abc import GimbalData

CameraPointToNed = Callable[[np.ndarray, GimbalData, Attitude], np.ndarray]


@dataclass(frozen=True)
class BoardOrbitCentre:
    """The board centre, or why this frame gives none (``location`` None)."""

    location: Location | None
    reason: str | None = None


def board_orbit_centre(
    poi: DetectedObject,
    camera_point_to_ned: CameraPointToNed,
    home: Location | None,
) -> BoardOrbitCentre | None:
    """None when the detector gave no board pose (a model detector)."""
    pose = poi.geo.board_pose
    if pose is None:
        return None
    if isinstance(pose, BoardPoseRejection):
        return BoardOrbitCentre(None, pose.reason)
    if poi.pose.is_frame_atomic is not True:
        return BoardOrbitCentre(None, "pose_not_at_capture")
    camera = poi.geo.capture_camera_location
    if camera is None or not camera.is_absolute:
        return BoardOrbitCentre(None, "no_capture_location")
    if home is None or not home.is_absolute or not math.isfinite(home.alt):
        return BoardOrbitCentre(None, "no_home")
    attitude = poi.pose.aircraft_attitude
    gimbal = poi.pose.gimbal_data
    if attitude is None or gimbal is None:
        return BoardOrbitCentre(None, "no_capture_pose")
    ned = np.asarray(
        camera_point_to_ned(np.asarray(pose.centre_camera_m), gimbal, attitude),
        dtype=float,
    ).reshape(-1)
    if ned.shape != (3,) or not np.all(np.isfinite(ned)):
        return BoardOrbitCentre(None, "non_finite_centre")
    latitude, longitude, altitude = pymap3d.ned2geodetic(
        float(ned[0]), float(ned[1]), float(ned[2]),
        camera.lat, camera.lng, camera.alt,
    )
    values = (float(latitude), float(longitude), float(altitude))
    if not all(math.isfinite(value) for value in values):
        return BoardOrbitCentre(None, "non_finite_centre")
    return BoardOrbitCentre(
        Location(values[0], values[1], values[2] - home.alt, is_absolute=False)
    )


__all__ = ["BoardOrbitCentre", "CameraPointToNed", "board_orbit_centre"]
