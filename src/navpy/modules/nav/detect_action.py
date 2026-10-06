"""DETECT-state action and peer geo pre-acquisition."""

from __future__ import annotations

from navpy.modules.nav.navigation_task_action import NavigationTaskAction
from navpy.modules.nav.peer_target_notification import PeerTargetNotifier
from navpy.modules.nav.target_selection import TargetSelector
from navpy.modules.nav.detection_snapshot import DetectionSnapshot
from navpy.modules.nav.peer_geo_tick import DetectPointingPorts, PeerGeoTick
from navpy.modules.nav.confirmation_manager import ConfirmationManager


class DetectAction:
    """Select work and maintain non-terminal peer geo pointing."""

    def __init__(
        self,
        detections: DetectionSnapshot,
        selector: TargetSelector,
        navigation_task: NavigationTaskAction,
        peer_notifier: PeerTargetNotifier,
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
            list(selection.targets),
            selection.primary_target,
        )
        if peers:
            self._peer_notifier.notify(peers)
        if self._confirmation_manager.active_target is None:
            self._navigation_task.handle_new_target(own)
        self._peer_geo.update()


__all__ = ["DetectAction", "DetectPointingPorts"]
