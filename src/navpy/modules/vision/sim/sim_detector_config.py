"""Typed input and output records for simulator detector composition."""

from __future__ import annotations

from argparse import Namespace
from dataclasses import dataclass
from typing import Optional

from navpy.logger.cache_logger import ILogger
from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.navigation.geo.zc_util import ZcUtil
from navpy.modules.navigation.gimbal_navigation import GimbalNavigation
from navpy.modules.navigation.gimbal_navigation_state import GimbalTrackingSetup
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.sim.detection_publication_buffer import (
    DetectionPublicationBuffer,
)
from navpy.modules.vision.sim.sim_detection_pipeline import SimDetectionPipeline
from navpy.modules.vision.sim.sim_detector_controls import (
    SimDetectorIdentity,
    SimGeoControls,
    SimPoiControls,
    SimTrackingControls,
    SimZoomControls,
)
from navpy.modules.vision.sim.sim_detector_lifecycle import SimDetectorLifecycle
from navpy.modules.vision.sim.sim_detector_loop import SimDetectorWorker
from navpy.modules.vision.sim.sim_runtime_ports import ConfirmationFrameFactory
from navpy.modules.vision.sim.sim_poi_projector import SimPoiProjector
from navpy.modules.vision.poi_zoom_types import PoiZoomTrackerConfig


@dataclass(frozen=True)
class SimDetectorDependencies:
    vehicle: IVehicle
    mount: CameraMount
    geo_ref: GeoRefCalc
    logger: ILogger
    args: Namespace
    zc_util: Optional[ZcUtil]
    scheduler_cadence: Optional[SchedulerCadence]


@dataclass(frozen=True)
class SimDetectorOptions:
    sim_assets_path: Optional[str]
    tracking_config: Optional[GimbalTrackingSetup]
    zoom_config: Optional[PoiZoomTrackerConfig]
    ideal_360: bool
    frame_generator_factory: ConfirmationFrameFactory


@dataclass(frozen=True)
class SimDetectorPorts:
    identity: SimDetectorIdentity
    detection: DetectionPublicationBuffer
    tracking: SimTrackingControls
    zoom: SimZoomControls
    geo: SimGeoControls
    simulation: SimPoiControls
    lifecycle: SimDetectorLifecycle
    cadence: SimDetectorWorker
    renderer: SimDetectionPipeline
    projector: SimPoiProjector
    navigation: Optional[GimbalNavigation]


__all__ = [
    "SimDetectorDependencies",
    "SimDetectorOptions",
    "SimDetectorPorts",
]
