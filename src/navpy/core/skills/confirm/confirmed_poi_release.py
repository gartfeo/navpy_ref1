"""Confirmed-POI admission into NAV."""

from __future__ import annotations

from collections.abc import Callable

from navpy.core.skills.confirm.confirmation_policy import PoiRetryPolicy
from navpy.core.skills.confirm.final_approach_release_gate import FinalApproachReleaseGate
from navpy.core.flow.nav_state import GeoHoldState, NavPhaseState, NavState
from navpy.core.skills.confirm.track_recovery import TrackRecovery
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.perception.detection.poi_identity import get_poi_task_id


class ConfirmedPoiRelease:
    """Apply release readiness and optional geo reacquisition before NAV."""

    def __init__(
        self,
        phase: NavPhaseState,
        geo_hold: GeoHoldState,
        retry: PoiRetryPolicy,
        recovery: TrackRecovery,
        release: FinalApproachReleaseGate,
        final_approach_active: Callable[[], bool],
    ) -> None:
        self._phase = phase
        self._geo_hold = geo_hold
        self._retry = retry
        self._recovery = recovery
        self._release = release
        self._final_approach_active = final_approach_active

    def apply(self, active: DetectedObject) -> None:
        self._retry.clear_reask(get_poi_task_id(active))
        if (
            self._phase.current != NavState.NAV
            and self._final_approach_active()
            and not self._release.is_ready(active)
        ):
            self._phase.request(NavState.CONFIRM)
            return
        if self._geo_hold.active and not self._recovery.reacquire_geo_hold(active):
            return
        self._phase.request(NavState.NAV)


__all__ = ["ConfirmedPoiRelease"]
