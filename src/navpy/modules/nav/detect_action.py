"""DETECT-state action and peer geo pre-acquisition."""

from __future__ import annotations

from navpy.modules.nav.navigation_task_action import NavigationTaskAction
from navpy.modules.nav.peer_poi_notification import PeerPoiNotifier
from navpy.modules.nav.poi_selection import PoiSelector
from navpy.modules.nav.detection_snapshot import DetectionSnapshot
from navpy.modules.nav.peer_geo_tick import DetectPointingPorts, PeerGeoTick
from navpy.modules.nav.confirmation_manager import ConfirmationManager


class DetectAction:
    """Select work and maintain non-final-approach peer geo pointing."""

    def __init__(
        self,
        detections: DetectionSnapshot,
        selector: PoiSelector,
        navigation_task: NavigationTaskAction,
        peer_notifier: PeerPoiNotifier,
        confirmation_manager: ConfirmationManager,
        peer_geo: PeerGeoTick,
    ) -> None:
        self._detections = detections
        self._selector = selector
        self._navigation_task = navigation_task
        self._peer_notifier = peer_notifier
        self._confirmation_manager = confirmation_manager
        self._peer_geo = peer_geo

    def act(self) -> None:
        selection = self._detections.selection()
        own, peers = self._selector.select(
            list(selection.pois),
            selection.primary_poi,
        )
        if peers:
            self._peer_notifier.notify(peers)
        if self._confirmation_manager.active_poi is None:
            self._navigation_task.handle_new_poi(own)
        self._peer_geo.update()


__all__ = ["DetectAction", "DetectPointingPorts"]
