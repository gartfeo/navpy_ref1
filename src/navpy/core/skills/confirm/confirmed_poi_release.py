"""Confirmed-POI release readiness: keep confirming, aborted, or ready to fly."""

from __future__ import annotations

from collections.abc import Callable
from enum import Enum, auto

from navpy.core.skills.confirm.confirmation_policy import PoiRetryPolicy
from navpy.core.skills.confirm.guidance_release_gate import GuidanceReleaseGate
from navpy.core.flow.nav_state import GeoHoldState, NavPhaseState, NavState
from navpy.core.skills.confirm.track_recovery import TrackRecovery
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.perception.detection.poi_identity import get_poi_task_id


class ReleaseOutcome(Enum):
    WAIT = auto()  # keep confirming
    ABORTED = auto()  # the geo hold was torn down
    READY = auto()  # ready to fly the approach


class ConfirmedPoiRelease:
    """Report release readiness after optional geo reacquisition."""

    def __init__(
        self,
        phase: NavPhaseState,
        geo_hold: GeoHoldState,
        retry: PoiRetryPolicy,
        recovery: TrackRecovery,
        release: GuidanceReleaseGate,
        guidance_active: Callable[[], bool],
        acquisition_ready: Callable[[], bool] = lambda: True,
    ) -> None:
        self._phase = phase
        self._geo_hold = geo_hold
        self._retry = retry
        self._recovery = recovery
        self._release = release
        self._guidance_active = guidance_active
        # HITL step 25: the fresh lock and established orbit, on this tick's detection.
        self._acquisition_ready = acquisition_ready

    def apply(self, active: DetectedObject) -> ReleaseOutcome:
        self._retry.clear_reask(get_poi_task_id(active))
        if self._phase.current != NavState.NAV and not self._acquisition_ready():
            self._phase.request(NavState.CONFIRM)
            return ReleaseOutcome.WAIT
        if (
            self._phase.current != NavState.NAV
            and self._guidance_active()
            and not self._release.is_ready(active)
        ):
            self._phase.request(NavState.CONFIRM)
            return ReleaseOutcome.WAIT
        if self._geo_hold.active and not self._recovery.reacquire_geo_hold(active):
            return ReleaseOutcome.ABORTED
        return ReleaseOutcome.READY


__all__ = ["ConfirmedPoiRelease", "ReleaseOutcome"]
