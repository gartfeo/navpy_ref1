"""CONFIRM tick orchestration and final-approach/local admission."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Protocol

from navpy.modules.nav.confirmation_reporting import (
    ConfirmBlockedReporter,
    ConfirmDebugReporter,
)
from navpy.modules.nav.confirmation_frame_policy import ConfirmationFramePolicy
from navpy.modules.nav.confirmation_recognition import RecognitionGate
from navpy.modules.nav.confirmation_review import ConfirmationReview
from navpy.modules.nav.detection_snapshot import DetectionSnapshot
from navpy.modules.nav.peer_poi_notification import PeerPoiNotifier
from navpy.modules.nav.poi_selection import PoiSelector
from navpy.modules.nav.nav_state import (
    GeoHoldState,
    FinalApproachNavState,
)
from navpy.modules.nav.peer_geo import PeerGeoAcquisition
from navpy.modules.nav.confirmation_manager import ConfirmationManager
from navpy.modules.vision.detector_ports import GeoPointingPort
from navpy.modules.vision.models.detect_data import DetectedObject


@dataclass(frozen=True)
class FinalApproachConfirmationPorts:
    final_approach_active: Callable[[], bool]
    can_confirm: Callable[[DetectedObject], bool]
    record_confirmed: Callable[[DetectedObject], bool]
    auto_confirm: Callable[[], bool]


class FinalApproachConfirmationAdmission:
    def __init__(
        self,
        ports: FinalApproachConfirmationPorts,
        final_approach_state: FinalApproachNavState,
        review: ConfirmationReview,
        debug: ConfirmDebugReporter,
    ) -> None:
        self._ports = ports
        self._final_approach_state = final_approach_state
        self._review = review
        self._debug = debug

    def handle(self, poi: DetectedObject) -> bool:
        if not self._ports.final_approach_active():
            return False
        if not self._ports.can_confirm(poi):
            self._debug.log(f"final_approach_los_unavailable obj={poi.identity.obj_id}")
            return True
        if not self._ports.auto_confirm():
            return False
        if not self._ports.record_confirmed(poi):
            self._debug.log(f"final_approach_record_unavailable obj={poi.identity.obj_id}")
            return True
        self._final_approach_state.confirmed_recorded = True
        self._review.confirm_local(poi)
        return True


class ActivePoiDetectionQuery(Protocol):
    """Fresh active-POI lookup required by one CONFIRM tick."""

    def find_active_poi_detection(self) -> Optional[DetectedObject]: ...


class ConfirmationGeoHold:
    """Keep an already-active loss geo hold pointed during review."""

    def __init__(
        self,
        geo_hold: GeoHoldState,
        confirmation_manager: ConfirmationManager,
        geo_pointing: GeoPointingPort,
        acquisition: PeerGeoAcquisition,
        current_location: Callable[[], object],
        current_attitude: Callable[[], object],
    ) -> None:
        self._geo_hold = geo_hold
        self._confirmation_manager = confirmation_manager
        self._geo_pointing = geo_pointing
        self._acquisition = acquisition
        self._current_location = current_location
        self._current_attitude = current_attitude

    def tick(self) -> None:
        if not self._geo_hold.active:
            return
        location = self._current_location()
        attitude = self._current_attitude()
        if location is None or attitude is None:
            return
        active = self._confirmation_manager.active_poi
        class_id = active.classification.class_id if active is not None else 0
        state = self._acquisition.snapshot(
            location,
            attitude,
            poi_location=self._geo_hold.poi_location,
            class_id=class_id,
        )
        if state is not None:
            self._geo_pointing.prepare_geo_acquisition(
                location,
                attitude,
                state.class_id,
                state.min_pixels,
            )
        self._geo_pointing.update_geo(location, attitude)


class ConfirmationAction:
    """Run one confirmation tick against explicit collaborators."""

    def __init__(
        self,
        detections: DetectionSnapshot,
        selector: PoiSelector,
        peer_notifier: PeerPoiNotifier,
        confirmation_manager: ConfirmationManager,
        source: ActivePoiDetectionQuery,
        frame_policy: ConfirmationFramePolicy,
        final_approach_admission: FinalApproachConfirmationAdmission,
        recognition_gate: RecognitionGate,
        blocked: ConfirmBlockedReporter,
        debug: ConfirmDebugReporter,
        geo_hold: ConfirmationGeoHold,
    ) -> None:
        self._detections = detections
        self._selector = selector
        self._peer_notifier = peer_notifier
        self._confirmation_manager = confirmation_manager
        self._source = source
        self._frame_policy = frame_policy
        self._final_approach_admission = final_approach_admission
        self._recognition_gate = recognition_gate
        self._blocked = blocked
        self._debug = debug
        self._geo_hold = geo_hold

    def act(self) -> None:
        self._geo_hold.tick()
        selection = self._detections.selection()
        _, peers = self._selector.select(
            list(selection.pois),
            selection.primary_poi,
        )
        if peers:
            self._peer_notifier.notify(peers)
        active = self._confirmation_manager.active_poi
        if active is None or self._confirmation_manager.get_status(active) is not None:
            self._blocked.clear()
            return
        fresh = self._source.find_active_poi_detection()
        if fresh is None:
            self._debug.log("no_detection")
            self._blocked.clear()
            return
        if self._final_approach_admission.handle(fresh):
            return
        if not self._frame_policy.is_ready(fresh):
            return
        self._recognition_gate.request(fresh)


__all__ = [
    "ActivePoiDetectionQuery",
    "ConfirmationAction",
    "ConfirmationGeoHold",
    "FinalApproachConfirmationAdmission",
    "FinalApproachConfirmationPorts",
]
