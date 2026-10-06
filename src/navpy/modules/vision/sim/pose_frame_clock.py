"""Raw ArduPilot pose-clock acceptance for simulator vision frames."""

from __future__ import annotations

import math
from typing import Any, Optional, Union

from navpy.modules.vision.sim.sim_runtime_ports import FrameOutcomeSink, ResetAction


POSE_CLOCK_REBOOT_JUMP_S = 60.0
POSE_CLOCK_FRESH_BOOT_WINDOW_S = 3.0
TimestampValue = Union[int, float, str, None]


class PoseFrameClock:
    """Own raw source time and one-frame-per-ATTITUDE admission state."""

    def __init__(
            self,
            *,
            reorder_tolerance_s: float,
            record_outcome: FrameOutcomeSink,
    ) -> None:
        self._reorder_tolerance_s = max(0.0, float(reorder_tolerance_s))
        self._record_outcome = record_outcome
        self._source_now_s: Optional[float] = None
        self._last_frame_s: Optional[float] = None

    @property
    def source_now_s(self) -> Optional[float]:
        return self._source_now_s

    @property
    def last_frame_s(self) -> Optional[float]:
        return self._last_frame_s

    @staticmethod
    def message_boot_time_s(message: Any) -> Optional[float]:
        value = getattr(message, "time_boot_ms", None)
        if isinstance(value, bool) or not isinstance(value, int):
            return None
        return float(value & 0xFFFFFFFF) * 1e-3

    @staticmethod
    def restarted(
            candidate_s: float,
            current_s: float,
            *,
            reorder_tolerance_s: float = 0.0,
    ) -> bool:
        backward_s = current_s - candidate_s
        return (
            backward_s > POSE_CLOCK_REBOOT_JUMP_S
            or (
                backward_s > max(0.0, reorder_tolerance_s)
                and candidate_s < POSE_CLOCK_FRESH_BOOT_WINDOW_S <= current_s
            )
        )

    def advance(
            self,
            value: TimestampValue,
            *,
            on_restart: ResetAction,
    ) -> Optional[float]:
        """Advance raw AP time without scaling, remapping, or extrapolation."""
        try:
            timestamp_s = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(timestamp_s):
            return None
        if self._source_now_s is None or timestamp_s >= self._source_now_s:
            self._source_now_s = timestamp_s
            return timestamp_s
        if self.restarted(
                timestamp_s,
                self._source_now_s,
                reorder_tolerance_s=self._reorder_tolerance_s,
        ):
            on_restart()
            return None
        return self._source_now_s

    def accept_attitude_timestamp(
            self,
            value: TimestampValue,
            *,
            on_restart: ResetAction,
            record_emitted: bool = True,
    ) -> Optional[float]:
        try:
            timestamp_s = float(value)
        except (TypeError, ValueError):
            timestamp_s = math.nan
        if not math.isfinite(timestamp_s):
            self._record_outcome(None, "missing_attitude_dropped", None)
            return None
        if self.advance(timestamp_s, on_restart=on_restart) is None:
            return None
        if self._last_frame_s is not None:
            if timestamp_s == self._last_frame_s:
                self._record_outcome(timestamp_s, "dup_attitude", None)
                return None
            if timestamp_s < self._last_frame_s:
                self._record_outcome(timestamp_s, "reordered_attitude", None)
                return None
        self._last_frame_s = timestamp_s
        if record_emitted:
            self._record_outcome(timestamp_s, "emitted", timestamp_s)
        return timestamp_s

    def reset_frame_history(self) -> None:
        self._last_frame_s = None

    def reset(self) -> None:
        self._source_now_s = None
        self._last_frame_s = None
