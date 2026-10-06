"""Stable DetectorSim API assembled from focused capability facets."""

from __future__ import annotations

from argparse import Namespace

from navpy.logger.cache_logger import ILogger
from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.navigation.geo.zc_util import ZcUtil
from navpy.modules.navigation.gimbal_navigation_state import GimbalTrackingSetup
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.detector_abc import DetectorAbc
from navpy.modules.vision.sim.sim_detector_assembly import (
    SimDetectorDependencies,
    SimDetectorOptions,
    SimDetectorPorts,
    build_sim_detector,
)
from navpy.modules.vision.sim.sim_detector_public_control import (
    SimDetectorGeoFacet,
    SimDetectorIdentityFacet,
    SimDetectorTrackingFacet,
)
from navpy.modules.vision.sim.sim_detector_public_runtime import (
    SimDetectorDetectionFacet,
    SimDetectorLifecycleFacet,
    SimDetectorProjectionFacet,
    SimDetectorSimulationFacet,
)
from navpy.modules.vision.sim.sim_frame_generator import SimFrameGenerator
from navpy.modules.vision.target_zoom_types import TargetZoomTrackerConfig


class DetectorSim(
    SimDetectorIdentityFacet,
    SimDetectorTrackingFacet,
    SimDetectorGeoFacet,
    SimDetectorLifecycleFacet,
    SimDetectorDetectionFacet,
    SimDetectorSimulationFacet,
    SimDetectorProjectionFacet,
    DetectorAbc,
):
    """One-field compatibility adapter over focused simulator owners."""

    def __init__(
        self,
        vehicle: IVehicle,
        mount: CameraMount,
        geo_ref: GeoRefCalc,
        logger: ILogger,
        args: Namespace,
        zc_util: ZcUtil | None = None,
        sim_assets_path: str | None = None,
        tracking_config: GimbalTrackingSetup | None = None,
        zoom_config: TargetZoomTrackerConfig | None = None,
        scheduler_cadence: SchedulerCadence | None = None,
        ideal_360: bool = False,
    ) -> None:
        self._parts: SimDetectorPorts = build_sim_detector(
            SimDetectorDependencies(
                vehicle,
                mount,
                geo_ref,
                logger,
                args,
                zc_util,
                scheduler_cadence,
            ),
            SimDetectorOptions(
                sim_assets_path=sim_assets_path,
                tracking_config=tracking_config,
                zoom_config=zoom_config,
                ideal_360=ideal_360,
                frame_generator_factory=SimFrameGenerator,
            ),
        )


__all__ = ["DetectorSim", "SimFrameGenerator"]
