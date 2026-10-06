"""Rejected, timed-out, and missing-POI exit behavior."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.confirmation_policy import (
    ConfirmationTimingPolicy,
    PoiRetryPolicy,
)
from navpy.modules.nav.navigation_task_reset import AutoMissionResume
from navpy.modules.nav.nav_state import NavPhaseState, NavState
from navpy.modules.nav.confirmation_manager import ConfirmationStatus
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.poi_identity import get_poi_task_id


@dataclass(frozen=True)
class PoiRejectionPorts:
    wall_s: Callable[[], float]
    clear_active_poi: Callable[[], None]
    stop_tracking: Callable[[], None]
    logger: ILogger


class PoiRejectionExit:
    """Close review state and resume AUTO for every rejection-like exit."""

    def __init__(
        self,
        ports: PoiRejectionPorts,
        phase: NavPhaseState,
        timing: ConfirmationTimingPolicy,
        retry: PoiRetryPolicy,
        resume_auto: AutoMissionResume,
    ) -> None:
        self._ports = ports
        self._phase = phase
        self._timing = timing
        self._retry = retry
        self._resume_auto = resume_auto

    def apply(
        self,
        active: DetectedObject,
        status: ConfirmationStatus | None,
    ) -> bool:
        if status == ConfirmationStatus.REJECTED:
            self._retry.clear_reask(get_poi_task_id(active))
            self._teardown()
            return self._resume()
        if status == ConfirmationStatus.TIMEOUT_REJECTED:
            self._teardown()
            return self._resume()
        if status == ConfirmationStatus.CONFIRMING:
            if not self._review_expired(active):
                return False
            return self._resume()
        if self._timing.reacquire_timed_out():
            self._abort_missing(active)
            return self._resume()
        self._phase.request(NavState.CONFIRM)
        return False

    def _review_expired(self, active: DetectedObject) -> bool:
        dwell = self._timing.review_dwell_s(self._ports.wall_s)
        max_dwell = self._timing.max_dwell_sec()
        if dwell < max_dwell:
            self._phase.request(NavState.CONFIRM)
            return False
        self._retry.register_poi_cooldown(active)
        self._teardown()
        self._ports.logger.info(
            "CONFIRM max dwell exceeded: under review "
            f"{dwell:.1f}s >= {max_dwell:.1f}s, aborting to DETECT",
            key="nav",
            dest=LogStatusDest.DRONE,
        )
        return True

    def _abort_missing(self, active: DetectedObject) -> None:
        loss_run = self._timing.loss_elapsed_s()
        bound = self._timing.reacquire_bound_s()
        bound_label = self._timing.reacquire_bound_label()
        self._retry.register_poi_cooldown(active)
        self._teardown()
        self._ports.logger.info(
            "CONFIRM re-acquire timeout: POI absent "
            f"{loss_run:.1f}s >= {bound:.1f}s "
            f"({bound_label}), aborting to DETECT",
            key="nav",
            dest=LogStatusDest.DRONE,
        )

    def _teardown(self) -> None:
        self._ports.clear_active_poi()
        self._ports.stop_tracking()
        self._phase.request(NavState.DETECT)

    def _resume(self) -> bool:
        self._resume_auto.run()
        self._ports.logger.info(
            "REJECTED: resuming AUTO mission",
            key="nav",
            dest=LogStatusDest.DRONE,
        )
        return True


__all__ = ["PoiRejectionExit", "PoiRejectionPorts"]
