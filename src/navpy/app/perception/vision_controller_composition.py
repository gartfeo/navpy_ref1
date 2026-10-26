"""Bounded composition root for the public vision controller."""

from __future__ import annotations

from navpy.core.kernel.exception_groups import ExceptionGroup

from argparse import Namespace

from navpy.app.vision_args import VisionArgs
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.kernel.scheduler_cadence import SchedulerCadence
from navpy.core.perception.geo.zc_util import ZcUtil
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.core.perception.detection_coordinator import DetectionCoordinator
from navpy.plugins.gimbal.siyi.hardware.command_fence import SiyiCommandFence
from navpy.app.perception.camera_stream_composition import streaming_cameras
from navpy.core.perception.vision_publisher_group import VisionPublisherGroup
from navpy.plugins.vehicle.ardupilot.camera_stream_publisher import (
    CameraStreamPublisher,
)
from navpy.plugins.vehicle.ardupilot.gimbal_telemetry_publisher import (
    GimbalTelemetryPublisher,
)
from navpy.app.perception.vision_controller_ports import VisionControllerPorts
from navpy.sim.debug_view.vision_debug_hud import SimTrackerHudRenderer
from navpy.sim.debug_view.vision_debug_panel import SimPanelRenderer
from navpy.sim.debug_view.vision_debug_pois import SimPoiOverlayRenderer
from navpy.sim.debug_view.vision_debug_window import VisionDebugWindow
from navpy.app.perception.vision_detector_factory_ports import (
    VisionDetectorFactory,
    build_detector_nodes,
    cleanup_detector_nodes,
    cleanup_mount_specs,
)
from navpy.core.perception.vision_lifecycle import VisionLifecycle
from navpy.app.perception.vision_profile_composition import build_vision_profile
from navpy.sim.debug_view.vision_sim_debug_view import (
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
    siyi_fence: SiyiCommandFence | None = None,
) -> VisionControllerPorts:
    profile = build_vision_profile(
        vision_args.profile_name,
        vehicle,
        logger,
        scheduler_cadence,
        use_sim_gimbal=vision_args.detector_type != "real",
        siyi_fence=siyi_fence,
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
            from navpy.app.perception.real_detector_factory import (
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
            from navpy.app.perception.sim_detector_factory import (
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
        publisher = VisionPublisherGroup((
            GimbalTelemetryPublisher(vehicle, profile.mount_specs, logger),
            # Operator display only: camera-protocol stream discovery.
            CameraStreamPublisher(
                vehicle,
                vehicle.target_system,
                streaming_cameras(
                    profile.mount_specs,
                    [node.camera_health for node in nodes],
                ),
                logger,
            ),
        ))
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
            dock_contact=factory.dock_contact,
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
