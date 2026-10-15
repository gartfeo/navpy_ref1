"""Make sustained attitude/position pairing loss visible, not silent."""

from __future__ import annotations

from typing import Optional

from navpy.core.approach.vision_nav.command_freshness import (
    FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S,
)
from navpy.sim.detection.sim_runtime_ports import WarningSink


class UnpairedPoseMonitor:
    """Warn once when polls stay unpaired for the observation freshness limit.

    Source time is the latest polled ATTITUDE boot time. Past the limit no
    command can use a frame anyway (``command_freshness``), so that is the
    point where fail-closed frame dropping is reported. One more warning is
    logged when pairing resumes after such a report.
    """

    def __init__(self, warning: WarningSink) -> None:
        self._warning = warning
        self._unpaired_since_s: Optional[float] = None
        self._warned = False

    def observe(self, source_time_s: Optional[float], *, paired: bool) -> None:
        if paired:
            self._resume(source_time_s)
            return
        if source_time_s is None:
            return
        if self._unpaired_since_s is None or source_time_s < self._unpaired_since_s:
            # First unpaired poll, or the source clock restarted.
            self._unpaired_since_s = source_time_s
        unpaired_s = source_time_s - self._unpaired_since_s
        if self._warned or unpaired_s < FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S:
            return
        self._warned = True
        self._warning(
            "DetectorSim: no exact ATTITUDE/GLOBAL_POSITION_INT pair for "
            f"{unpaired_s:.3f}s of source time (since AP "
            f"{self._unpaired_since_s:.3f}s); frames are being dropped"
        )

    def _resume(self, source_time_s: Optional[float]) -> None:
        if self._warned:
            self._warning(
                "DetectorSim: ATTITUDE/GLOBAL_POSITION_INT pairing resumed at "
                f"AP {source_time_s:.3f}s"
                if source_time_s is not None
                else "DetectorSim: ATTITUDE/GLOBAL_POSITION_INT pairing resumed"
            )
        self._unpaired_since_s = None
        self._warned = False


__all__ = ["UnpairedPoseMonitor"]
