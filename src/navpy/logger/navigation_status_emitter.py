"""Verbose operator/status output for one navigation sample."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.logger.navigation_sample import NavigationSample
from navpy.logger.navigation_snap_geometry import calc_distance


class NavigationStatusEmitter:
    """Formats and emits the verbose navigation and gimbal diagnostic rows."""

    def __init__(self, logger: ILogger) -> None:
        self._logger = logger

    def emit(
        self,
        sample: NavigationSample,
    ) -> tuple[Optional[float], Optional[float]]:
        cx, cy = self._principal_point(sample.camera_matrix)
        x_err_center = (
            cx - sample.x_error if sample.x_error is not None else None
        )
        y_err_center = (
            cy - sample.y_error if sample.y_error is not None else None
        )

        cmd_roll_text = (
            f"{sample.command_roll:.1f}"
            if sample.command_roll is not None
            else "N/A"
        )
        cmd_pitch_text = (
            f"{sample.command_pitch:.1f}"
            if sample.command_pitch is not None
            else "00.0"
        )
        x_text = str(x_err_center) if x_err_center is not None else "N/A"
        y_text = str(y_err_center) if y_err_center is not None else "N/A"
        term_text = (
            f"{sample.terminal_angle:.1f}"
            if sample.terminal_angle is not None
            else "N/A"
        )
        actual_roll_text = _optional_angle_text(sample.actual_roll)
        actual_pitch_text = _optional_angle_text(sample.actual_pitch)

        current_delta = calc_distance(
            sample.current_location,
            sample.detected_current_location,
        )
        target_delta = calc_distance(
            sample.target_location,
            sample.detected_target_location,
        )
        debug_text = (
            f"c_l ({current_delta:.1f}m): "
            f"{sample.detected_current_location} [{sample.current_location}], "
            f"t_l ({target_delta:.1f}m): "
            f"{sample.detected_target_location} [{sample.target_location}], "
            f"term: {term_text}"
        )
        self._logger.info(
            f"({x_text}, {y_text}): {sample.distance:.1f}m - "
            f"nr: {cmd_roll_text} (dy: {sample.yaw_error:.1f}, "
            f"ar: {actual_roll_text}), "
            f"np: {cmd_pitch_text} (dp: {sample.pitch_error:.1f}, "
            f"ap: {actual_pitch_text}), {debug_text}",
            key="navigation",
            status=(
                f"({cmd_roll_text},{cmd_pitch_text}) "
                f"d:{sample.distance:.0f}"
            ),
            dest=LogStatusDest.DRONE,
        )

        if sample.gimbal_attitude is not None:
            gimbal = sample.gimbal_attitude
            self._logger.info(
                f"GIMBAL_DIVE: gmb=(y{gimbal.yaw:.0f},"
                f"p{gimbal.pitch:.0f}) "
                f"los_body=(y{sample.yaw_error:.0f},"
                f"p{sample.pitch_error:.0f}) "
                f"px=({x_text},{y_text}) d={sample.distance:.0f}m",
                key="navigation",
                dest=LogStatusDest.DRONE,
            )
        return x_err_center, y_err_center

    @staticmethod
    def _principal_point(k: Optional[np.ndarray]) -> tuple[float, float]:
        if k is not None:
            try:
                return float(k[0][2]), float(k[1][2])
            except (IndexError, TypeError, ValueError):
                pass
        return 960.0, 540.0


def _optional_angle_text(value: object) -> str:
    if value is None or isinstance(value, bool):
        return "N/A"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "N/A"
    return f"{number:.1f}" if math.isfinite(number) else "N/A"
