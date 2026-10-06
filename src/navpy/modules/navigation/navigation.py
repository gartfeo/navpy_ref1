"""Thin composition root for algorithm-specific navigation services."""

from __future__ import annotations

from typing import Optional

from navpy.args.navigation_args import NavigationArgs
from navpy.logger.cache_logger import ILogger
from navpy.logger.navigation_logger import ClosestSnap
from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.navigation.geo.zc_util import ZcUtil
from navpy.modules.navigation.navigation_composition import compose_navigation
from navpy.modules.navigation.navigation_command_worker import (
    CommandLoopObserver,
)
from navpy.modules.navigation.navigation_source_dispatch import SourceDispatch
from navpy.modules.navigation.mission_planner import MissionPlanner
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.models.detect_data import DetectedObject


class Navigation:
    """Compose navigation collaborators and publish explicit capability services."""

    def __init__(
        self,
        vehicle: IVehicle,
        geo_ref: GeoRefCalc,
        zc_util: Optional[ZcUtil],
        mission_planner: MissionPlanner,
        logger: ILogger,
        args: NavigationArgs,
        scheduler_cadence: Optional[SchedulerCadence] = None,
    ) -> None:
        if zc_util is None and getattr(args, "use_terrain", True):
            raise ValueError(
                "Current Implementation requires ZcUtil for terrain calculations."
            )
        composition = compose_navigation(
            vehicle,
            geo_ref,
            zc_util,
            mission_planner,
            logger,
            args,
            scheduler_cadence,
        )
        self._lifecycle = composition.lifecycle
        self._mode_state = composition.mode_state
        self.final_approach = composition.final_approach
        self.legacy_pois = composition.legacy_pois
        self.vehicle_commands = composition.vehicle_commands
        self._bind_source_dispatch = composition.bind_source_dispatch

    def start(self) -> ClosestSnap:
        return self._lifecycle.start()

    def init(self) -> None:
        self._lifecycle.init()

    def reset(self) -> ClosestSnap:
        return self._lifecycle.reset()

    def stop(self) -> None:
        self._lifecycle.stop()

    def pause_final_approach(self) -> None:
        self._lifecycle.pause()

    def raise_if_failed(self) -> None:
        self._lifecycle.raise_if_failed()

    def nav(self, detect_data: DetectedObject) -> bool:
        with self._mode_state.runtime_session() as runtime:
            return runtime.nav(detect_data)

    def bind_final_approach_source_dispatch(
        self,
        callback: SourceDispatch,
        observer: CommandLoopObserver | None = None,
    ) -> None:
        """Publish the final-approach source's dispatch, and its observer if it
        has one. The observer is record-only; nothing reads it back."""
        self._bind_source_dispatch(callback, observer)

    @property
    def algorithm_info(self) -> tuple[str, Optional[float]]:
        return self._lifecycle.algorithm_info


__all__ = ["Navigation"]
