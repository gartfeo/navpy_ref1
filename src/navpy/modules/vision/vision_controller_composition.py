"""Bounded composition root for the public vision controller."""

from __future__ import annotations

from navpy.exception_groups import ExceptionGroup

from argparse import Namespace

from navpy.args.vision_args import VisionArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.navigation.geo.zc_util import ZcUtil
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.detection_coordinator import DetectionCoordinator
from navpy.modules.vision.gimbal_telemetry_publisher import (
    GimbalTelemetryPublisher,
)
from navpy.modules.vision.vision_controller_ports import VisionControllerPorts
from navpy.modules.vision.vision_debug_hud import SimTrackerHudRenderer
from navpy.modules.vision.vision_debug_panel import SimPanelRenderer
from navpy.modules.vision.vision_debug_pois import SimPoiOverlayRenderer
from navpy.modules.vision.vision_debug_window import VisionDebugWindow
from navpy.modules.vision.vision_detector_factory_ports import (
    VisionDetectorFactory,
    build_detector_nodes,
    cleanup_detector_nodes,
    cleanup_mount_specs,
)
from navpy.modules.vision.vision_lifecycle import VisionLifecycle
from navpy.modules.vision.vision_profile_composition import build_vision_profile
from navpy.modules.vision.vision_sim_debug_view import (
    StackedSimulatorView,
    VisionUiDispatcher,
)


def build_vision_controller(
    vehicle: IVehicle,
    args: Namespace,
    vision_args: VisionArgs,
    logger: ILogger,
    zc_util: ZcUtil | None,
    scheduler_cadence: SchedulerCadence | None,
) -> VisionControllerPorts:
    profile = build_vision_profile(
        vision_args.profile_name,
        vehicle,
        logger,
        scheduler_cadence,
    )
    nodes = ()
    try:
        factory: VisionDetectorFactory
        if vision_args.detector_type == "real":
            if bool(profile.detector_settings.get("ideal_360", False)):
                raise ValueError(
                    "ideal_360 is a simulator-only sensor and cannot be "
                    "used with --detector-type=real"
                )
            from navpy.modules.vision.real_detector_factory import (
                RealDetectorFactory,
            )

            factory = RealDetectorFactory(
                vehicle,
                logger,
                profile.profile,
                vision_args.model_path,
                vision_args.debug_show,
                backend=vision_args.detector_backend,
            )
        else:
            from navpy.modules.vision.sim_detector_factory import (
                SimDetectorFactory,
            )

            factory = SimDetectorFactory(
                vehicle,
                args,
                logger,
                profile.geo_ref,
                profile.profile,
                zc_util,
                scheduler_cadence,
            )
        nodes = build_detector_nodes(
            factory,
            profile.mount_specs,
            profile.detector_settings,
        )
        detectors = [node.detector for node in nodes]
        coordinator = DetectionCoordinator(detectors, logger)
        publisher = GimbalTelemetryPublisher(vehicle, profile.mount_specs, logger)
        real_ui = tuple(
            node.real_ui for node in nodes if node.real_ui is not None
        )
        sim_sources = tuple(
            node.sim_debug for node in nodes if node.sim_debug is not None
        )
        simulator_view = (
            StackedSimulatorView(
                sim_sources,
                SimPanelRenderer(
                    lambda: vehicle.attitude,
                    SimTrackerHudRenderer(),
                    SimPoiOverlayRenderer(profile.geo_ref.calc_ned),
                ),
                VisionDebugWindow(f"navpy-sim-{vehicle.source_system}"),
            )
            if vision_args.debug_show and sim_sources
            else None
        )
        ui = VisionUiDispatcher(real_ui, simulator_view)
        lifecycle = VisionLifecycle(
            [spec.mount for spec in profile.mount_specs],
            publisher,
            coordinator,
            simulator_view,
            logger.error,
        )
        return VisionControllerPorts(
            coordinator=coordinator,
            detectors=detectors,
            mounts=[spec.mount for spec in profile.mount_specs],
            geo_ref=profile.geo_ref,
            profile_name=profile.name,
            profile=profile.profile,
            approach_kind=profile.approach_kind,
            lifecycle=lifecycle,
            ui=ui,
        )
    except Exception as construction_error:
        cleanup_errors = cleanup_detector_nodes(nodes)
        cleanup_errors.extend(cleanup_mount_specs(profile.mount_specs))
        if cleanup_errors:
            raise ExceptionGroup(
                "vision construction and rollback failed",
                [construction_error, *cleanup_errors],
            ) from None
        raise


__all__ = ["build_vision_controller"]
