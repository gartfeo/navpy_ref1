"""Legacy Roll-L1 controller composition boundary."""

from __future__ import annotations

import math
from collections.abc import Callable
from time import time

import numpy as np

from navpy.modules.navigation.nav.roll_l1_state import (
    RollL1Parameters,
    RollL1State,
)
from navpy.modules.navigation.nav.roll_l1_inputs import RollL1MotionSnapshot
from navpy.modules.navigation.nav.roll_l1_waypoint_solver import RollL1WaypointSolver
from navpy.utils.math_utils import GRAVITY_MSS


class RollL1Control:
    def __init__(
        self,
        parameter_reader: Callable[[], RollL1Parameters],
        motion_reader: Callable[[], RollL1MotionSnapshot],
    ) -> None:
        self._parameter_reader = parameter_reader
        self._motion_reader = motion_reader
        self.reset()

    def reset(self) -> None:
        self._parameters = self._parameter_reader()
        self._state = RollL1State(last_update_s=time())
        self._waypoint_solver = RollL1WaypointSolver(
            self._motion_reader,
            self._parameters,
            self._state,
        )

    def get_yaw(self) -> float:
        return self._waypoint_solver.get_yaw()

    def get_yaw_sensor(self) -> float:
        return self._waypoint_solver.get_yaw_sensor()

    def nav_roll(self, pitch: float | None) -> float:
        pitch = 0 if pitch is None else pitch
        pitch = np.clip(
            pitch,
            self._parameters.pitch_min,
            self._parameters.pitch_max,
        )
        pitch_limit = math.radians(89.9)
        pitch_rad = np.clip(
            math.radians(pitch),
            -pitch_limit,
            pitch_limit,
        )
        roll = math.degrees(math.atan(
            self._state.lat_accel_demand
            / (GRAVITY_MSS * math.cos(pitch_rad))
        ))
        return np.clip(roll, -90, 90)

    def update(
        self,
        previous_waypoint,
        next_waypoint,
        current,
        bearing_cd: float,
    ) -> None:
        self._waypoint_solver.update(
            previous_waypoint,
            next_waypoint,
            current,
            bearing_cd,
        )
