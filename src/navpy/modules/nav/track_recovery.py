"""Delivery-reference tracking loss, geo hold and stable-track reacquisition.

Track identity continuity does not establish an authorized recipient identity.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.nav.confirmation_policy import PoiRetryPolicy
from navpy.modules.nav.detection_snapshot import DetectionSnapshot
from navpy.modules.nav.navigation_task_reset import AutoMissionResume
from navpy.modules.nav.nav_constants import (
    TRACK_LOSS_GEO_HOLD_TIMEOUT_SEC,
    TRACK_REACQUIRE_GATE_RADIUS_M,
)
from navpy.modules.nav.nav_state import (
    ConfirmGateState,
    GeoHoldState,
    NavPhaseState,
    NavState,
)
from navpy.modules.nav.confirmation_manager import ConfirmationManager, ConfirmationStatus
from navpy.modules.vision.detector_ports import (
    GeoPointingPort,
    PoiIdentityPort,
    TrackingCommandPort,
    TrackingStatusPort,
)
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.poi_identity import get_poi_task_id


class TrackRecovery:
    """Re-arm detection or fail closed to the normal navigation task teardown."""

    def __init__(
        self,
        phase: NavPhaseState,
        geo_hold: GeoHoldState,
        confirm: ConfirmGateState,
        tracking: TrackingCommandPort,
        confirmation_manager: ConfirmationManager,
        retry: PoiRetryPolicy,
        resume_auto: AutoMissionResume,
        logger: ILogger,
    ) -> None:
        self._phase = phase
        self._geo_hold = geo_hold
        self._confirm = confirm
        self._tracking = tracking
        self._confirmation_manager = confirmation_manager
        self._retry = retry
        self._resume_auto = resume_auto
        self._logger = logger

    def held_poi_geo(self, assigned: Optional[Location]) -> Optional[Location]:
        return self._geo_hold.last_own_poi_geo or assigned

    def abort_untracked_navigation_task(self, active: DetectedObject) -> None:
        self._geo_hold.active = False
        self._geo_hold.poi_location = None
        self._confirm.loss_started_at = None
        self._retry.register_poi_cooldown(active)
        self._confirmation_manager.clear_active_poi()
        self._tracking.stop_tracking()
        self._phase.request(NavState.DETECT)
        self._logger.info(
            f"GEO-HOLD abort: P{get_poi_task_id(active)} navigation task torn "
            "down to DETECT (no tracking armed), resuming AUTO mission",
            key="nav",
            dest=LogStatusDest.DRONE,
        )
        self._resume_auto.run()

    def rearm_detection_or_abort(
        self,
        active: DetectedObject,
        context: str,
    ) -> bool:
        task_id = get_poi_task_id(active)
        try:
            self._tracking.start_tracking(task_id)
        except Exception as error:  # noqa: BLE001 - fail closed
            self._logger.warning(
                f"GEO-HOLD {context}: detection re-arm for P{task_id} also "
                f"failed ({error}) — aborting navigation task",
                key="nav",
                dest=LogStatusDest.DRONE,
            )
            self.abort_untracked_navigation_task(active)
            return True
        self._logger.warning(
            f"GEO-HOLD {context}: detection tracking restored for P{task_id} "
            "— legacy abort stays in charge",
            key="nav",
            dest=LogStatusDest.DRONE,
        )
        return False

    def reacquire_geo_hold(self, active: DetectedObject) -> bool:
        task_id = get_poi_task_id(active)
        try:
            self._tracking.start_tracking(task_id)
        except Exception as error:  # noqa: BLE001 - fail closed
            self._logger.warning(
                f"GEO-HOLD: P{task_id} reacquire re-arm failed ({error}) — "
                "aborting navigation task (rearm-or-abort, fail closed)",
                key="nav",
                dest=LogStatusDest.DRONE,
            )
            self.abort_untracked_navigation_task(active)
            return False
        self._geo_hold.active = False
        self._geo_hold.poi_location = None
        self._confirm.loss_started_at = None
        self._logger.info(
            f"GEO-HOLD: P{task_id} reacquired, resuming detection tracking",
            key="nav",
            dest=LogStatusDest.DRONE,
        )
        return True


@dataclass(frozen=True)
class IdentityReacquisitionPorts:
    ground_location: Callable[..., Optional[Location]]
    absolute_location: Callable[[Optional[Location]], Optional[Location]]


class IdentityReacquisition:
    """Rebind a same-class, nearby detection to the original task id."""

    def __init__(
        self,
        ports: IdentityReacquisitionPorts,
        geo_hold: GeoHoldState,
        detections: DetectionSnapshot,
        poi_identity: PoiIdentityPort,
        retry: PoiRetryPolicy,
        recovery: TrackRecovery,
        logger: ILogger,
    ) -> None:
        self._ports = ports
        self._geo_hold = geo_hold
        self._detections = detections
        self._poi_identity = poi_identity
        self._retry = retry
        self._recovery = recovery
        self._logger = logger

    def try_reacquire(self, active: DetectedObject) -> bool:
        held = self._geo_hold.poi_location
        if held is None:
            return False
        active_class_id = active.classification.class_id
        held_task_id = get_poi_task_id(active)
        for candidate in self._detections.pois():
            if candidate.classification.class_id != active_class_id:
                continue
            if self._retry.is_in_poi_cooldown(candidate):
                continue
            candidate_geo = candidate.geo.projected_poi_location
            if candidate_geo is None:
                candidate_geo = self._ports.ground_location(
                    candidate,
                    allow_fallback=False,
                )
            candidate_geo = self._ports.absolute_location(candidate_geo)
            if candidate_geo is None:
                continue
            distance = GeoRefCalc.calculate_distance(candidate_geo, held)
            if distance is None or distance > TRACK_REACQUIRE_GATE_RADIUS_M:
                continue
            old_local_id = candidate.identity.obj_id
            if not self._poi_identity.rebind_task_id(held_task_id, candidate):
                continue
            self._logger.info(
                f"GEO-HOLD: P{held_task_id} identity-expiry re-bind — "
                f"old local id={old_local_id} distance={distance:.1f}m",
                key="nav",
                dest=LogStatusDest.DRONE,
            )
            return not self._recovery.reacquire_geo_hold(active)
        return False


class GeoHoldCoordinator:
    """Enter or leave bounded non-final-approach geo hold."""

    def __init__(
        self,
        geo_hold: GeoHoldState,
        confirm: ConfirmGateState,
        tracking: TrackingCommandPort,
        tracking_status: TrackingStatusPort,
        geo_pointing: GeoPointingPort,
        final_approach_active: Callable[[], bool],
        clock_s: Callable[[], float],
        assigned_location: Callable[[], Optional[Location]],
        recovery: TrackRecovery,
        identity: IdentityReacquisition,
        geo_ref: GeoRefCalc,
        logger: ILogger,
    ) -> None:
        self._geo_hold = geo_hold
        self._confirm = confirm
        self._tracking = tracking
        self._tracking_status = tracking_status
        self._geo_pointing = geo_pointing
        self._final_approach_active = final_approach_active
        self._clock_s = clock_s
        self._assigned_location = assigned_location
        self._recovery = recovery
        self._identity = identity
        self._geo_ref = geo_ref
        self._logger = logger

    def update(
        self,
        active: DetectedObject,
        status: Optional[ConfirmationStatus],
        present: bool,
    ) -> bool:
        if self._geo_hold.active:
            if present:
                return not self._recovery.reacquire_geo_hold(active)
            return self._identity.try_reacquire(active)
        if present or status in {
            ConfirmationStatus.CONFIRMED,
            ConfirmationStatus.REJECTED,
            ConfirmationStatus.TIMEOUT_REJECTED,
        }:
            return False
        if self._final_approach_active():
            return False
        loss_hold_s = self._tracking_status.loss_hold_sec
        if loss_hold_s is None or self._confirm.loss_started_at is None:
            return False
        if self._clock_s() - self._confirm.loss_started_at < loss_hold_s:
            return False
        held = self._recovery.held_poi_geo(self._assigned_location())
        if held is None:
            return False
        try:
            self._tracking.stop_tracking(to_neutral=False)
            self._geo_pointing.start_geo_tracking(held, self._geo_ref)
        except Exception as error:  # noqa: BLE001 - rearm or abort
            self._logger.warning(
                f"GEO-HOLD arm failed for P{get_poi_task_id(active)} "
                f"poi={held}: {error}",
                key="nav",
                dest=LogStatusDest.DRONE,
            )
            return self._recovery.rearm_detection_or_abort(
                active,
                "entry arm failed",
            )
        if not self._geo_pointing.is_geo_armed:
            self._logger.warning(
                f"GEO-HOLD arm declined for P{get_poi_task_id(active)} "
                f"poi={held}: detector did not arm geo tracking",
                key="nav",
                dest=LogStatusDest.DRONE,
            )
            return self._recovery.rearm_detection_or_abort(
                active,
                "entry arm declined",
            )
        self._geo_hold.active = True
        self._geo_hold.poi_location = held
        self._logger.info(
            f"GEO-HOLD: P{get_poi_task_id(active)} entered geo-hold "
            f"poi={held} timeout={TRACK_LOSS_GEO_HOLD_TIMEOUT_SEC:.0f}s",
            key="nav",
            dest=LogStatusDest.DRONE,
        )
        return False


__all__ = [
    "GeoHoldCoordinator",
    "IdentityReacquisition",
    "IdentityReacquisitionPorts",
    "TrackRecovery",
]
