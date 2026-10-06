"""Explicit construction of profile-backed simulated detectors."""

from __future__ import annotations

from argparse import Namespace
from pathlib import Path

import numpy as np

from navpy.logger.cache_logger import ILogger
from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.navigation.geo.zc_util import ZcUtil
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.sim.detector_sim import DetectorSim
from navpy.modules.vision.vision_detector_factory_ports import VisionDetectorNode
from navpy.modules.vision.vision_profile_types import CameraMountSpec, VisionProfile
from navpy.modules.vision.vision_tracking_composition import build_tracking_config
from navpy.modules.vision.vision_ui_ports import SimDebugSnapshot, SimDebugSource
from navpy.modules.vision.vision_zoom_profile import build_zoom_config


class SimDetectorFactory:
    """Compose one simulator detector without exposing controller state."""

    def __init__(
        self,
        vehicle: IVehicle,
        args: Namespace,
        logger: ILogger,
        geo_ref: GeoRefCalc,
        profile: VisionProfile,
        zc_util: ZcUtil | None,
        scheduler_cadence: SchedulerCadence | None,
    ) -> None:
        self._vehicle = vehicle
        self._args = args
        self._logger = logger
        self._geo_ref = geo_ref
        self._profile = profile
        self._zc_util = zc_util
        self._scheduler_cadence = scheduler_cadence
        self._assets_path = Path(__file__).resolve().parents[4] / ".sim"

    def create(
        self,
        mount_spec: CameraMountSpec,
        detector_settings: VisionProfile,
        mount_index: int,
    ) -> VisionDetectorNode:
        del mount_index
        ideal_360 = bool(detector_settings.get("ideal_360", False))
        tracking_config = (
            None
            if ideal_360
            else build_tracking_config(
                mount_spec.device,
                sim=True,
            )
        )
        zoom_config = (
            None
            if ideal_360
            else build_zoom_config(
                mount_spec.device,
                self._profile,
            )
        )
        detector = DetectorSim(
            vehicle=self._vehicle,
            mount=mount_spec.mount,
            geo_ref=self._geo_ref,
            logger=self._logger,
            args=self._args,
            zc_util=self._zc_util,
            sim_assets_path=None if ideal_360 else str(self._assets_path),
            tracking_config=tracking_config,
            zoom_config=zoom_config,
            scheduler_cadence=self._scheduler_cadence,
            ideal_360=ideal_360,
        )
        if ideal_360:
            self._logger.info(
                f"DetectorSim({mount_spec.mount.name}): "
                "static ideal 360 enabled"
            )
        self._logger.info(
            f"Created sim detector for mount '{mount_spec.mount.name}'"
        )

        def read_debug_snapshot() -> SimDebugSnapshot:
            detections = tuple(detector.get_latest_detections())
            class_id = detections[0].classification.class_id if detections else None
            raw_intrinsics = mount_spec.mount.get_k()
            intrinsics = (
                None
                if raw_intrinsics is None
                else np.array(raw_intrinsics, copy=True)
            )
            if intrinsics is not None:
                intrinsics.setflags(write=False)
            return SimDebugSnapshot(
                source_name=detector.source_name,
                gimbal=mount_spec.mount.get_gimbal_data(),
                image_width=mount_spec.mount.image_width or 1920,
                image_height=mount_spec.mount.image_height or 1080,
                intrinsics=intrinsics,
                current_zoom=mount_spec.mount.get_current_zoom(),
                detections=detections,
                rate_result=detector.rate_result,
                zoom_result=detector.get_zoom_result(),
                zoom_target_pixels=detector.get_zoom_target_pixels(class_id),
            )

        return VisionDetectorNode(
            detector=detector,
            cleanup=detector,
            real_ui=None,
            sim_debug=SimDebugSource(
                read_snapshot=read_debug_snapshot,
            ),
        )


__all__ = ["SimDetectorFactory"]
