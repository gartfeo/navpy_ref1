"""Post-command diagnostics for legacy geo-assisted final approach."""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

from navpy.logger.navigation_logger import NavigationLogger
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.navigation.legacy_terminal_calculation import (
    LegacyTerminalCalculation,
)
from navpy.modules.vision.models.detect_data import DetectedObject


class LegacyTerminalDiagnostics:
    """Log command diagnostics after vehicle actuation has succeeded."""

    def __init__(
            self,
            navigation_logger: NavigationLogger,
            get_locked_target: Callable[[], Optional[Location]],
    ) -> None:
        self._navigation_logger = navigation_logger
        self._get_locked_target = get_locked_target

    def log(
            self,
            calculation: LegacyTerminalCalculation,
            current_location: Location,
            current_attitude: Attitude,
            detect_data: Optional[DetectedObject],
    ) -> None:
        x_error = detect_data.pixel.u_px if detect_data else None
        y_error = detect_data.pixel.v_px if detect_data else None
        detected_current_location = (
            detect_data.geo.camera_location if detect_data else None
        )
        is_sim_detection = bool(
            detect_data and detect_data.geo.is_simulation
        )
        detected_target_debug = (
            detect_data.geo.truth_target_location if is_sim_detection else None
        )
        distance_target = (
            detected_target_debug or self._get_locked_target()
        )
        distance_debug = GeoRefCalc.calculate_distance(
            current_loc=current_location,
            target_loc=distance_target,
        )
        self._navigation_logger.log(
            c_loc=current_location,
            t_loc=calculation.nav_target_location,
            distance=distance_debug,
            cmd_roll=calculation.cmd_roll_deg,
            cmd_pitch=calculation.cmd_pitch_deg,
            yaw_error=calculation.yaw_error,
            pitch_error=calculation.pitch_error,
            actual_roll=current_attitude.roll,
            actual_pitch=current_attitude.pitch,
            x_error=x_error,
            y_error=y_error,
            detect_c_loc=detected_current_location,
            detect_t_loc_debug=detected_target_debug,
            k=detect_data.optics.camera_matrix() if detect_data else None,
            gimbal_att=(
                detect_data.pose.gimbal_data.att
                if detect_data
                and detect_data.pose.gimbal_data is not None
                else None
            ),
        )


__all__ = ["LegacyTerminalDiagnostics"]
