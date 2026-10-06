"""Primary navigation-sample recording and wall-clock rate gating."""

from __future__ import annotations

from typing import Any, Callable, Optional

from navpy.logger.navigation_clocks import (
    current_source_time_s,
    minimum_wall_interval_s,
    primary_log_gate,
)
from navpy.logger.navigation_log_streams import NavigationLogStreams
from navpy.logger.navigation_sample import NavigationSample
from navpy.logger.navigation_snap_geometry import calc_h_v_dist
from navpy.logger.navigation_snap_session import NavigationSnapSession
from navpy.logger.navigation_status_emitter import NavigationStatusEmitter
from navpy.logger.log_schema import PRIMARY_SCHEMA


class NavigationSampleRecorder:
    """Throttle, format, and publish primary navigation rows."""

    def __init__(
        self,
        streams: NavigationLogStreams,
        snap_session: NavigationSnapSession,
        status_emitter: NavigationStatusEmitter,
        wall_time: Callable[[], float],
        timestamp: Callable[[], str],
        source_time: Optional[Callable[[], float]],
        cadence_interval: Optional[Callable[[float], float]],
        nominal_interval_s: float,
    ) -> None:
        self._streams = streams
        self._snap_session = snap_session
        self._status_emitter = status_emitter
        self._wall_time = wall_time
        self._timestamp = timestamp
        self._source_time = source_time
        self._cadence_interval = cadence_interval
        self._nominal_interval_s = nominal_interval_s
        self._last_log_time = 0.0

    def record(
        self,
        sample: NavigationSample,
        rate_gate: Optional[bool],
    ) -> bool:
        h_dist, v_dist = calc_h_v_dist(
            sample.current_location,
            sample.detected_target_location,
        )
        self._snap_session.sample(
            sample.current_location,
            sample.detected_target_location,
        )
        should_write, next_log_time = primary_log_gate(
            rate_gate=rate_gate,
            last_log_time=self._last_log_time,
            current_time=self._wall_time,
            minimum_interval=self._minimum_interval,
        )
        if not should_write:
            return False
        self._last_log_time = next_log_time
        x_error, y_error = self._status_emitter.emit(sample)
        self._streams.write_primary(
            self._format_row(sample, h_dist, v_dist, x_error, y_error)
        )
        return True

    def reset_gate(self) -> None:
        self._last_log_time = 0.0

    def _minimum_interval(self) -> float:
        return minimum_wall_interval_s(
            self._cadence_interval,
            self._nominal_interval_s,
        )

    def _format_row(
        self,
        sample: NavigationSample,
        h_dist: float,
        v_dist: float,
        x_error: Optional[float],
        y_error: Optional[float],
    ) -> str:
        values: dict[str, Any] = {
            "ts": self._timestamp(),
            "dist": sample.distance,
            "h_dist": h_dist,
            "v_dist": v_dist,
            "cmd_r": sample.command_roll,
            "cmd_p": sample.command_pitch,
            "yaw_err": sample.yaw_error,
            "pitch_err": sample.pitch_error,
            "act_r": sample.actual_roll,
            "act_p": sample.actual_pitch,
            "x_err": x_error,
            "y_err": y_error,
            "src_t": current_source_time_s(self._source_time),
        }
        return PRIMARY_SCHEMA.format_row(values) + "\n"
