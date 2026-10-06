"""Closest-approach session lifecycle for navigation logging."""

from __future__ import annotations

from typing import Mapping, Optional

from navpy.logger.navigation_event_recorder import NavigationEventRecorder
from navpy.logger.navigation_event_rows import snap_components_payload
from navpy.logger.navigation_snap_tracker import ClosestApproachTracker
from navpy.logger.navigation_snap_types import ClosestSnap
from navpy.logger.log_events import LogEvent
from navpy.modules.common.models.location import Location


class NavigationSnapSession:
    """Own closest-approach state, scoring, summary, and reset."""

    def __init__(self, event_recorder: NavigationEventRecorder) -> None:
        self._event_recorder = event_recorder
        self._snap = ClosestSnap()
        self._previous: Optional[Location] = None
        self._tracker = ClosestApproachTracker()

    def sample(
        self,
        current: Optional[Location],
        poi: Optional[Location],
    ) -> None:
        self._previous = self._tracker.update(
            self._snap,
            self._previous,
            current,
            poi,
        )

    def snapshot(self) -> ClosestSnap:
        return self._snap.clone()

    def write_summary_and_reset(
        self,
        algorithm: Optional[str],
        kp: Optional[float],
    ) -> ClosestSnap:
        snap = self._snap.clone()
        if self._should_write(snap):
            kp_str = f"kp={kp:.2f}" if kp is not None else ""
            self._event_recorder.record(
                LogEvent.SNAP,
                {
                    "algorithm": algorithm,
                    "snap": str(snap),
                    "kp_str": kp_str,
                },
            )
            if snap.has_components():
                self._event_recorder.record(
                    LogEvent.SNAP_COMPONENTS,
                    self._component_payload(snap, algorithm, kp_str),
                )
        self._snap = ClosestSnap()
        self._previous = None
        return snap

    def _should_write(self, snap: ClosestSnap) -> bool:
        return (
            snap.dist != float("inf")
            and self._event_recorder.has_compact_stream()
        )

    @staticmethod
    def _component_payload(
        snap: ClosestSnap,
        algorithm: Optional[str],
        kp_str: str,
    ) -> Mapping[str, str]:
        return snap_components_payload(
            snap,
            algorithm=algorithm,
            kp_str=kp_str,
        )
