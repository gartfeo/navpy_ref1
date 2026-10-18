"""Selection of one local POI and any peer candidates."""

from __future__ import annotations

from typing import Callable, Optional

from navpy.core.skills.confirm.confirmation_policy import PoiRetryPolicy
from navpy.core.skills.search.dock_roles import DockRoles
from navpy.core.review.confirmation_manager import ConfirmationManager
from navpy.core.review.confirmation_registry_state import ConfirmationStatus
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.perception.detection.poi_identity import get_poi_task_id
from navpy.core.perception.detection.poi_priority import (
    find_poi_by_task_id,
    prioritize_pois,
)


class PoiSelector:
    """Split detections into one local candidate and peer candidates."""

    def __init__(
        self,
        confirmation_manager: ConfirmationManager,
        retry: PoiRetryPolicy,
        peer_available: Callable[[], bool],
        roles: DockRoles,
    ) -> None:
        self._confirmation_manager = confirmation_manager
        self._retry = retry
        self._peer_available = peer_available
        self._roles = roles

    def _may_offer(self, poi: DetectedObject) -> bool:
        return self._peer_available() and self._roles.may_offer(poi)

    def select(
        self,
        available_pois: list[DetectedObject],
        primary_poi: Optional[DetectedObject] = None,
    ) -> tuple[Optional[DetectedObject], list[DetectedObject]]:
        if not available_pois:
            return None, []
        active = self._confirmation_manager.active_poi
        if active is not None:
            own = find_poi_by_task_id(
                available_pois,
                get_poi_task_id(active),
            )
            peers = [
                poi
                for poi in available_pois
                if poi is not own and self._may_offer(poi)
            ]
            return own, peers
        own = None
        peers: list[DetectedObject] = []
        permanent = {
            ConfirmationStatus.PEER_NOTIFIED,
            ConfirmationStatus.REJECTED,
            ConfirmationStatus.CONFIRMED,
        }
        ordered = prioritize_pois(available_pois, primary_poi)
        for poi in self._roles.order(ordered):
            status = self._confirmation_manager.get_status(poi)
            exhausted = (
                status is ConfirmationStatus.TIMEOUT_REJECTED
                and not self._retry.can_reask(poi)
            )
            if status in permanent or exhausted:
                if self._may_offer(poi):
                    peers.append(poi)
                continue
            if self._retry.is_in_poi_cooldown(poi):
                continue
            if own is None and self._roles.may_take(poi):
                own = poi
            elif self._may_offer(poi):
                peers.append(poi)
        return own, peers


__all__ = ["PoiSelector"]
