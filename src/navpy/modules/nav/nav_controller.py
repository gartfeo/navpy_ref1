"""Stable thin entry point for the composed navigation application."""

from __future__ import annotations

from typing import Optional

from navpy.args.nav_args import NavArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.navigation.approach_strategy import ApproachKind
from navpy.modules.navigation.navigation import Navigation
from navpy.modules.nav.nav_composition import create_nav_application
from navpy.modules.nav.nav_constants import (
    DIST_EPS,
    DIST_INC_MAX,
    NAV_LOOP_PERIOD_S,
    NAV_LOOP_RATE_HZ,
    PASSED_POI_BEHIND_MIN_DEG,
    PEER_APPROACH_DIST,
    PEER_APPROACH_MARGIN_M,
    POI_CLOSE_DIST,
    FINAL_APPROACH_RECORD_TIMEOUT_S,
)
from navpy.modules.vehicle.vehicle_contract import IVehicle
from navpy.modules.vision.detection_coordination import DetectionCoordination


class NavController:
    """Delegate the public lifecycle surface to one composed application."""

    def __init__(
        self,
        vehicle: IVehicle,
        detection: DetectionCoordination,
        navigation: Navigation,
        args: NavArgs,
        logger: ILogger,
        approach_kind: ApproachKind = ApproachKind.OFFSET,
        vision_profile: Optional[dict] = None,
        scheduler_cadence: Optional[SchedulerCadence] = None,
    ) -> None:
        self._application = create_nav_application(
            vehicle,
            detection,
            navigation,
            args,
            logger,
            approach_kind,
            vision_profile,
            scheduler_cadence,
        )

    def set_network(self, network) -> None:
        self._application.set_network(network)

    def start(self, loop_rate_hz: float) -> None:
        self._application.start(loop_rate_hz)

    def stop(self) -> None:
        self._application.stop()

    def raise_if_failed(self) -> None:
        self._application.raise_if_failed()

    def force_confirm_override(self, task_id: int) -> None:
        self._application.force_confirm_override(task_id)


__all__ = [
    "DIST_EPS",
    "DIST_INC_MAX",
    "NAV_LOOP_PERIOD_S",
    "NAV_LOOP_RATE_HZ",
    "NavController",
    "PASSED_POI_BEHIND_MIN_DEG",
    "PEER_APPROACH_DIST",
    "PEER_APPROACH_MARGIN_M",
    "POI_CLOSE_DIST",
    "FINAL_APPROACH_RECORD_TIMEOUT_S",
]
