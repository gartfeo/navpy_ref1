"""Immutable input for one accepted-or-throttled navigation log sample."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from navpy.modules.common.models.location import Location


@dataclass(frozen=True)
class NavigationSample:
    """Data carried from the public logger API to its sample recorder."""

    current_location: Optional[Location]
    target_location: Optional[Location]
    distance: float
    command_roll: Optional[float]
    command_pitch: Optional[float]
    yaw_error: float
    pitch_error: float
    actual_roll: Optional[float]
    actual_pitch: Optional[float]
    x_error: Optional[int]
    y_error: Optional[int]
    terminal_angle: Optional[float]
    detected_current_location: Optional[Location]
    detected_target_location: Optional[Location]
    camera_matrix: Optional[np.ndarray]
    gimbal_attitude: object
