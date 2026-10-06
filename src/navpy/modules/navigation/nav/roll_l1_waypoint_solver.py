"""Waypoint tracking calculation for legacy Roll-L1 navigation."""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from time import time

import numpy as np

from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo import normalize
from navpy.modules.navigation.nav.roll_l1_state import (
    RollL1Parameters,
    RollL1State,
)
from navpy.modules.navigation.nav.roll_l1_inputs import RollL1MotionSnapshot
from navpy.utils.math_utils import (
    get_distance_NE,
    wrap_180_cd,
    wrap_360_cd,
    wrap_PI,
)


def _cross2d(a: Sequence[float], b: Sequence[float]) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


class RollL1WaypointSolver:
    """Mutate L1 state from a vehicle and a waypoint pair."""

    def __init__(
        self,
        motion_reader: Callable[[], RollL1MotionSnapshot],
        parameters: RollL1Parameters,
        state: RollL1State,
    ) -> None:
        self._motion_reader = motion_reader
        self._parameters = parameters
        self._state = state

    def get_yaw(self) -> float:
        return self._yaw(self._motion_reader())

    def get_yaw_sensor(self) -> float:
        return self._yaw_sensor(self._motion_reader())

    def update(
        self,
        previous_waypoint: Location,
        next_waypoint: Location,
        current: Location | None,
        bearing_cd: float,
    ) -> None:
        now = time()
        dt = now - self._state.last_update_s
        self._state.last_update_s = now
        if dt > 1.0:
            self._state.xtrack_integral = 0.0
        dt = min(dt, 0.1)
        self._state.last_update_us = now
        gain = 4.0 * self._parameters.damping**2

        if current is None:
            self._state.data_is_stale = True
            return
        self._state.target_bearing_cd = bearing_cd
        motion = self._motion_reader()

        ground_vector = list(motion.velocity_ne_mps)
        ground_speed = np.linalg.norm(ground_vector)
        vector_angle = np.pi / 2 + np.arctan2(
            -ground_vector[0],
            ground_vector[1],
        )
        yaw = self._yaw(motion)
        moving_forwards = abs(wrap_PI(vector_angle - yaw)) < np.pi / 2
        if ground_speed < 0.1 or not moving_forwards:
            ground_speed = 0.1
            ground_vector = [
                np.cos(yaw) * ground_speed,
                np.sin(yaw) * ground_speed,
            ]

        self._state.l1_distance = max(
            (1.0 / math.pi)
            * self._parameters.damping
            * self._parameters.period
            * ground_speed,
            0,
        )
        path = get_distance_NE(previous_waypoint, next_waypoint)
        path_length = np.linalg.norm(path)
        if path_length < 1.0e-6:
            path = get_distance_NE(current, next_waypoint)
            if np.linalg.norm(path) < 1.0e-6:
                path = [np.cos(yaw), np.sin(yaw)]
        path = normalize(path)

        from_start = get_distance_NE(previous_waypoint, current)
        self._state.crosstrack_error = _cross2d(from_start, path)
        start_distance = np.linalg.norm(from_start)
        along_track = np.dot(from_start, path)
        if (
            start_distance > self._state.l1_distance
            and along_track / max(start_distance, 1.0) < -0.7071
        ):
            nu = self._toward_waypoint(
                from_start,
                ground_vector,
                start=True,
            )
        elif along_track > path_length + ground_speed * 3:
            from_finish = get_distance_NE(next_waypoint, current)
            nu = self._toward_waypoint(
                from_finish,
                ground_vector,
                start=False,
            )
        else:
            nu = self._along_path(path, ground_vector, dt)

        nu = self._prevent_indecision(nu, motion)
        self._state.last_nu = nu
        limited_nu = np.clip(nu, -1.5708, 1.5708)
        self._state.lat_accel_demand = (
            gain
            * ground_speed
            * ground_speed
            / max(self._state.l1_distance, 0.1)
            * np.sin(limited_nu)
        )
        self._state.bearing_error = limited_nu
        self._state.data_is_stale = False

    def _toward_waypoint(
        self,
        from_waypoint: Sequence[float],
        ground_vector: Sequence[float],
        *,
        start: bool,
    ) -> float:
        unit = normalize(from_waypoint)
        toward = [-unit[0], -unit[1]]
        nu = np.arctan2(
            _cross2d(ground_vector, toward),
            np.dot(ground_vector, toward),
        )
        self._state.nav_bearing = np.arctan2(toward[1], toward[0])
        return nu

    def _along_path(
        self,
        path: Sequence[float],
        ground_vector: Sequence[float],
        dt: float,
    ) -> float:
        nu2 = np.arctan2(
            _cross2d(ground_vector, path),
            np.dot(ground_vector, path),
        )
        sine_nu1 = self._state.crosstrack_error / max(
            self._state.l1_distance,
            0.1,
        )
        nu1 = np.arcsin(np.clip(sine_nu1, -0.7071, 0.7071))
        gain = self._parameters.xtrack_i_gain
        if gain <= 0 or gain != self._state.previous_integral_gain:
            self._state.xtrack_integral = 0
            self._state.previous_integral_gain = gain
        elif np.abs(nu1) < np.radians(5):
            self._state.xtrack_integral = np.clip(
                self._state.xtrack_integral + nu1 * gain * dt,
                -0.1,
                0.1,
            )
        nu1 += self._state.xtrack_integral
        self._state.nav_bearing = wrap_PI(
            np.arctan2(path[1], path[0]) + nu1
        )
        return nu1 + nu2

    def _prevent_indecision(
        self,
        nu: float,
        motion: RollL1MotionSnapshot,
    ) -> float:
        limit = 0.9 * np.pi
        if (
            abs(nu) > limit
            and abs(self._state.last_nu) > limit
            and abs(wrap_180_cd(
                self._state.target_bearing_cd - self._yaw_sensor(motion)
            )) > 12000
            and nu * self._state.last_nu < 0.0
        ):
            return self._state.last_nu
        return nu

    @staticmethod
    def _yaw(motion: RollL1MotionSnapshot) -> float:
        return float(np.radians(motion.yaw_deg))

    @staticmethod
    def _yaw_sensor(motion: RollL1MotionSnapshot) -> float:
        return float(wrap_360_cd(motion.heading_deg * 100))
