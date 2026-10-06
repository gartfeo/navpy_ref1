"""Optional source-frame cadence telemetry for the simulator detector."""

from __future__ import annotations

import time
from typing import Optional

import navpy.modules.vehicle.pose_cadence_debug as pose_cadence_debug


class FrameOutcomeRecorder:
    """Record detector source hops without retaining the vehicle facade."""

    def __init__(self, target_system: int) -> None:
        self._target_system = int(target_system or 0)

    def __call__(
            self,
            boot_s: Optional[float],
            outcome: str,
            frame_ts_s: Optional[float],
    ) -> None:
        if not pose_cadence_debug.ENABLED:
            return
        pose_cadence_debug.record_detector_pose(
            self._target_system,
            time.time(),
            boot_s,
            outcome,
            frame_ts_s,
        )
