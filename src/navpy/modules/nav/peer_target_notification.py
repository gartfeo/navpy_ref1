"""Peer-target metadata resolution and notification."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from navpy.logger.cache_logger import ILogger
from navpy.modules.common.models.location import Location
from navpy.modules.nav.confirmation_manager import ConfirmationManager, ConfirmationStatus
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.target_identity import same_target_identity


@dataclass(frozen=True)
class PeerTargetNotifierPorts:
    detector_is_simulation: Callable[[], bool]
    ground_location: Callable[..., Optional[Location]]
    notify: Callable[[list[DetectedObject]], None]


class PeerTargetNotifier:
    """Resolve DETECT/CONFIRM peer metadata before network notification."""

    def __init__(
        self,
        ports: PeerTargetNotifierPorts,
        confirmation_manager: ConfirmationManager,
        logger: ILogger,
    ) -> None:
        self._ports = ports
        self._confirmation_manager = confirmation_manager
        self._logger = logger

    def notify(self, peer_targets: list[DetectedObject]) -> None:
        active = self._confirmation_manager.active_target
        notify_targets: list[DetectedObject] = []
        for target in peer_targets:
            if active and same_target_identity(target, active):
                continue
            if target.geo.projected_target_location is None:
                if (
                    self._ports.detector_is_simulation()
                    and target.geo.truth_target_location is not None
                ):
                    location = target.geo.truth_target_location
                else:
                    location = self._ports.ground_location(
                        target,
                        allow_fallback=False,
                    )
                if location is None:
                    self._logger.warning(
                        "Peer target missing geo fix; skipping notify"
                    )
                    continue
                target.set_p_t_g_loc(location)
            if self._confirmation_manager.get_status(target) is None:
                notify_targets.append(target)
        if notify_targets:
            self._ports.notify(notify_targets)
            for target in notify_targets:
                self._confirmation_manager.update_status(
                    target,
                    ConfirmationStatus.PEER_NOTIFIED,
                )


__all__ = ["PeerTargetNotifier", "PeerTargetNotifierPorts"]
