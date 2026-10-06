"""Public composition boundary for navigation logging."""

from __future__ import annotations

import math
import time
from typing import Any, Callable, Mapping, Optional

import numpy as np

from navpy.logger.cache_logger import ILogger
from navpy.logger.navigation_clocks import current_wall_timestamp
from navpy.logger.navigation_event_recorder import NavigationEventRecorder
from navpy.logger.navigation_log_streams import (
    NavigationLogStreams,
    open_navigation_log_streams,
)
from navpy.logger.navigation_sample import NavigationSample
from navpy.logger.navigation_sample_recorder import NavigationSampleRecorder
from navpy.logger.navigation_snap_types import ClosestPointComponents, ClosestSnap
from navpy.logger.navigation_snap_session import NavigationSnapSession
from navpy.logger.navigation_status_emitter import NavigationStatusEmitter
from navpy.logger.log_events import LogEvent
from navpy.logger.log_schema import PRIMARY_SCHEMA
from navpy.modules.common.models.location import Location


NAVIGATION_EVIDENCE_FAILURE_MARKER = "NAVIGATION_EVIDENCE_FAILURE"


__all__ = (
    "ClosestPointComponents",
    "ClosestSnap",
    "NAVIGATION_EVIDENCE_FAILURE_MARKER",
    "NavigationLogger",
)


class NavigationLogger:
    """Stable navigation-log API backed by focused recorders and state owners."""

    MIN_LOG_INTERVAL = 0.1
    DEBUG_EVENT_FLUSH_INTERVAL = 64

    def __init__(
        self,
        sys_id: int,
        logger: ILogger,
        time_source: Optional[Callable[[], float]] = None,
        cadence_interval: Optional[Callable[[float], float]] = None,
        *,
        wall_time: Optional[Callable[[], float]] = None,
        timestamp: Optional[Callable[[], str]] = None,
        streams: Optional[NavigationLogStreams] = None,
    ) -> None:
        owned_streams = streams or open_navigation_log_streams(sys_id, logger)
        row_timestamp = timestamp or current_wall_timestamp
        self._logger = logger
        self._streams = owned_streams
        self._events = NavigationEventRecorder(
            owned_streams,
            row_timestamp,
            self.DEBUG_EVENT_FLUSH_INTERVAL,
        )
        self._snap_session = NavigationSnapSession(self._events)
        self._samples = NavigationSampleRecorder(
            owned_streams,
            self._snap_session,
            NavigationStatusEmitter(logger),
            wall_time or time.time,
            row_timestamp,
            time_source,
            cadence_interval,
            self.MIN_LOG_INTERVAL,
        )
        try:
            self.log_event(
                LogEvent.CONFIG,
                {
                    "min_log_interval_s": f"{self.MIN_LOG_INTERVAL}",
                    "schema": PRIMARY_SCHEMA.name,
                },
            )
        except BaseException as init_error:
            try:
                owned_streams.close()
            except BaseException as close_error:
                if close_error is not init_error:
                    raise close_error from init_error
            raise init_error

    def log(
        self,
        c_loc: Optional[Location],
        t_loc: Optional[Location],
        distance: float,
        cmd_roll: Optional[float],
        cmd_pitch: Optional[float],
        yaw_error: float,
        pitch_error: float,
        actual_roll: Optional[float],
        actual_pitch: Optional[float],
        x_error: Optional[int],
        y_error: Optional[int],
        term_angle: Optional[float] = None,
        detect_c_loc: Optional[Location] = None,
        detect_t_loc_debug: Optional[Location] = None,
        k: Optional[np.ndarray] = None,
        final_approach_los_down_deg: Optional[float] = None,
        gimbal_att=None,
        rate_gate: Optional[bool] = None,
    ) -> bool:
        """Record one sample; only wall cadence controls primary-row gating."""
        del final_approach_los_down_deg
        return self._samples.record(
            NavigationSample(
                current_location=c_loc,
                poi_location=t_loc,
                distance=distance,
                command_roll=cmd_roll,
                command_pitch=cmd_pitch,
                yaw_error=yaw_error,
                pitch_error=pitch_error,
                actual_roll=_finite_or_none(actual_roll),
                actual_pitch=_finite_or_none(actual_pitch),
                x_error=x_error,
                y_error=y_error,
                final_approach_angle=term_angle,
                detected_current_location=detect_c_loc,
                detected_poi_location=detect_t_loc_debug,
                camera_matrix=k,
                gimbal_attitude=gimbal_att,
            ),
            rate_gate,
        )

    def capture_event_timestamp(self) -> str:
        """Capture wall time at the producing command boundary."""
        return self._events.capture_timestamp()

    def log_event(
        self,
        event: LogEvent,
        payload: Mapping[str, Any],
        *,
        timestamp: str | None = None,
    ) -> None:
        self._events.record(event, payload, timestamp=timestamp)

    def sample_snap(
        self,
        c_loc: Optional[Location],
        t_loc: Optional[Location],
        *,
        source: Optional[str] = None,
        final_approach_gap_payload: Optional[Mapping[str, str]] = None,
    ) -> None:
        del source, final_approach_gap_payload
        self._snap_session.sample(c_loc, t_loc)

    def get_snap(self) -> ClosestSnap:
        return self._snap_session.snapshot()

    def write_summary_and_reset(
        self,
        algorithm: Optional[str] = None,
        kp: Optional[float] = None,
    ) -> ClosestSnap:
        snap = self._snap_session.write_summary_and_reset(algorithm, kp)
        self._samples.reset_gate()
        return snap

    def drain(self) -> None:
        self._streams.drain()

    def close(self) -> None:
        self._streams.close()


def _finite_or_none(value: object) -> Optional[float]:
    if value is None or isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None
