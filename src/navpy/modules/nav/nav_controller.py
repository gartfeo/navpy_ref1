"""Stable thin entry point for the composed navigation application."""

from __future__ import annotations

from typing import Optional

from navpy.args.nav_args import NavArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.navigation.approach_strategy import ApproachKind
from navpy.modules.navigation.navigation import Navigation
from navpy.modules.nav.confirmation_frame_policy import (
    extract_confirmation_size as _extract_confirmation_size,
)
from navpy.modules.nav.nav_composition import create_nav_application
from navpy.modules.nav.nav_constants import (
    ALT_HYST,
    CONFIRM_DWELL_MARGIN_SEC,
    CONFIRM_FRESH_DETECTION_MAX_AGE_S,
    CONFIRM_MAX_DWELL_SEC,
    CONFIRM_REACQUIRE_ABORT_SEC,
    CONFIRM_REASK_MAX_ATTEMPTS,
    DIST_EPS,
    DIST_INC_MAX,
    GUIDED_ACCEPT_TIMEOUT_S,
    MAX_CONFIRM_GATE_RESETS,
    NAV_LOOP_PERIOD_S,
    NAV_LOOP_RATE_HZ,
    PASSED_TARGET_BEHIND_MIN_DEG,
    PEER_APPROACH_DIST,
    PEER_APPROACH_MARGIN_M,
    TARGET_CLOSE_DIST,
    TARGET_REACQUIRE_COOLDOWN_SEC,
    TERMINAL_RECORD_TIMEOUT_S,
    TRACK_LOSS_GEO_HOLD_TIMEOUT_SEC,
    TRACK_REACQUIRE_GATE_RADIUS_M,
)
from navpy.modules.nav.nav_state import NavState
from navpy.modules.vehicle.vehicle_interface import IVehicle
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
    "ALT_HYST",
    "CONFIRM_DWELL_MARGIN_SEC",
    "CONFIRM_FRESH_DETECTION_MAX_AGE_S",
    "CONFIRM_MAX_DWELL_SEC",
    "CONFIRM_REACQUIRE_ABORT_SEC",
    "CONFIRM_REASK_MAX_ATTEMPTS",
    "DIST_EPS",
    "DIST_INC_MAX",
    "GUIDED_ACCEPT_TIMEOUT_S",
    "MAX_CONFIRM_GATE_RESETS",
    "NAV_LOOP_PERIOD_S",
    "NAV_LOOP_RATE_HZ",
    "NavController",
    "NavState",
    "PASSED_TARGET_BEHIND_MIN_DEG",
    "PEER_APPROACH_DIST",
    "PEER_APPROACH_MARGIN_M",
    "TARGET_CLOSE_DIST",
    "TARGET_REACQUIRE_COOLDOWN_SEC",
    "TERMINAL_RECORD_TIMEOUT_S",
    "TRACK_LOSS_GEO_HOLD_TIMEOUT_SEC",
    "TRACK_REACQUIRE_GATE_RADIUS_M",
    "_extract_confirmation_size",
]
