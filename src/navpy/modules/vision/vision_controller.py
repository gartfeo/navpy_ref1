"""Stable public adapter over the composed vision system."""

from __future__ import annotations

from argparse import Namespace

from navpy.args.vision_args import VisionArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.navigation.approach_strategy import ApproachKind
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.navigation.geo.zc_util import ZcUtil
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.detection_coordination import DetectionCoordination
from navpy.modules.vision.detection_coordinator import DetectionCoordinator
from navpy.modules.vision.detector_ports import DetectorFleetMember
from navpy.modules.vision.vision_controller_composition import (
    build_vision_controller,
)
from navpy.modules.vision.vision_controller_ports import VisionControllerPorts
from navpy.modules.vision.vision_profile_types import VisionProfile


class VisionController:
    """One-field compatibility boundary over focused vision owners."""

    def __init__(
        self,
        vehicle: IVehicle,
        args: Namespace,
        vision_args: VisionArgs,
        logger: ILogger,
        zc_util: ZcUtil | None = None,
        scheduler_cadence: SchedulerCadence | None = None,
    ) -> None:
        self._ports: VisionControllerPorts = build_vision_controller(
            vehicle,
            args,
            vision_args,
            logger,
            zc_util,
            scheduler_cadence,
        )

    @property
    def coordinator(self) -> DetectionCoordinator:
        return self._ports.coordinator

    @property
    def coordination(self) -> DetectionCoordination:
        return self._ports.coordinator.coordination

    @property
    def detectors(self) -> list[DetectorFleetMember]:
        return self._ports.detectors

    @property
    def detector(self) -> DetectionCoordinator:
        return self._ports.coordinator

    @property
    def mounts(self) -> list[CameraMount]:
        return self._ports.mounts

    @property
    def geo_ref(self) -> GeoRefCalc:
        return self._ports.geo_ref

    @property
    def profile_name(self) -> str:
        return self._ports.profile_name

    @property
    def profile(self) -> VisionProfile:
        return self._ports.profile

    @property
    def approach_kind(self) -> ApproachKind:
        return self._ports.approach_kind

    def start(self) -> None:
        self._ports.lifecycle.start()

    def stop(self) -> bool:
        return self._ports.lifecycle.stop()

    @property
    def is_quiescent(self) -> bool:
        return self._ports.lifecycle.is_quiescent

    def refresh(self) -> None:
        self._ports.lifecycle.refresh()

    def ui_step(self) -> bool:
        return self._ports.ui.ui_step()

    def raise_if_failed(self) -> None:
        self._ports.lifecycle.raise_if_failed()


__all__ = ["VisionController"]
