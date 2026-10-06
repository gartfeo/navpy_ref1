"""Peer-POI metadata resolution and notification."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from navpy.logger.cache_logger import ILogger
from navpy.modules.common.models.location import Location
from navpy.modules.nav.confirmation_manager import ConfirmationManager, ConfirmationStatus
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.poi_identity import same_poi_identity


@dataclass(frozen=True)
class PeerPoiNotifierPorts:
    detector_is_simulation: Callable[[], bool]
    ground_location: Callable[..., Optional[Location]]
    notify: Callable[[list[DetectedObject]], None]


class PeerPoiNotifier:
    """Resolve DETECT/CONFIRM peer metadata before network notification."""

    def __init__(
        self,
        ports: PeerPoiNotifierPorts,
        confirmation_manager: ConfirmationManager,
        logger: ILogger,
    ) -> None:
        self._ports = ports
        self._confirmation_manager = confirmation_manager
        self._logger = logger

    def notify(self, peer_pois: list[DetectedObject]) -> None:
        active = self._confirmation_manager.active_poi
        notify_pois: list[DetectedObject] = []
        for poi in peer_pois:
            if active and same_poi_identity(poi, active):
                continue
            if poi.geo.projected_poi_location is None:
                if (
                    self._ports.detector_is_simulation()
                    and poi.geo.truth_poi_location is not None
                ):
                    location = poi.geo.truth_poi_location
                else:
                    location = self._ports.ground_location(
                        poi,
                        allow_fallback=False,
                    )
                if location is None:
                    self._logger.warning(
                        "Peer POI missing geo fix; skipping notify"
                    )
                    continue
                poi.set_p_t_g_loc(location)
            if self._confirmation_manager.get_status(poi) is None:
                notify_pois.append(poi)
        if notify_pois:
            self._ports.notify(notify_pois)
            for poi in notify_pois:
                self._confirmation_manager.update_status(
                    poi,
                    ConfirmationStatus.PEER_NOTIFIED,
                )


__all__ = ["PeerPoiNotifier", "PeerPoiNotifierPorts"]
