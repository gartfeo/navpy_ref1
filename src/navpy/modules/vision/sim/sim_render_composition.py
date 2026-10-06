"""Compose simulator target rendering and post-render side effects."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from navpy.logger.cache_log_level import CacheLogLevel

from navpy.modules.navigation.gimbal_navigation import GimbalNavigation
from navpy.modules.vision.sim.detection_publication_buffer import (
    DetectionPublicationBuffer,
)
from navpy.modules.vision.detector_ports import source_name_from_gimbal
from navpy.modules.vision.sim.finite_target_projector import FiniteTargetProjector
from navpy.modules.vision.sim.finite_projection_evidence import projection_log_sink
from navpy.modules.vision.sim.projection_run_recorder import ProjectionRunRecorder, create_projection_recorder, rollback_projection_recorder
from navpy.modules.vision.sim.ideal_camera_state import IdealCameraState
from navpy.modules.vision.sim.ideal_target_projector import (
    IdealTargetProjector,
    UasFrameConvention,
)
from navpy.modules.vision.sim.sim_camera_ports import (
    CaptureCameraPort,
    FrameSize,
    ProjectionCameraPort,
    TrackingCameraPort,
)
from navpy.modules.vision.sim.sim_confirmation_capture import ConfirmationCapture
from navpy.modules.vision.sim.sim_detection_context import SimDetectionContext
from navpy.modules.vision.sim.sim_detection_gap import ForcedDetectionGapPolicy
from navpy.modules.vision.sim.sim_detection_pipeline import SimDetectionPipeline
from navpy.modules.vision.sim.sim_detector_config import (
    SimDetectorDependencies,
    SimDetectorOptions,
)
from navpy.modules.vision.sim.sim_detector_controls import SimTrackingControls
from navpy.modules.vision.sim.sim_detector_state import (
    ForcedGapState,
    SimCaptureState,
)
from navpy.modules.vision.sim.sim_frame_timestamp import SimFrameTimestampResolver
from navpy.modules.vision.sim.sim_frame_transactions import (
    DetectionFrameTransactions,
)
from navpy.modules.vision.sim.sim_source_composition import (
    SimSourceGraph,
    SourceTimestampClock,
)
from navpy.modules.vision.sim.sim_target_catalog import SimTargetCatalog
from navpy.modules.vision.sim.sim_target_projector import SimTargetProjector
from navpy.modules.vision.sim.sim_tracking_update import SimTrackingUpdater
from navpy.modules.vision.target_provider import TargetProvider


@dataclass(frozen=True)
class SimRenderFoundation:
    target_provider: SimTargetCatalog
    navigation: Optional[GimbalNavigation]
    capture: SimCaptureState
    gap: ForcedGapState
    tracking: SimTrackingControls
    projector: SimTargetProjector
    publications: DetectionPublicationBuffer
    source_clock: SourceTimestampClock
    ideal_camera: IdealCameraState
    evidence_recorder: Optional[ProjectionRunRecorder] = None


def build_render_foundation(
    dependencies: SimDetectorDependencies,
    options: SimDetectorOptions,
    source: SimSourceGraph,
) -> SimRenderFoundation:
    target_provider = SimTargetCatalog(TargetProvider(
        dependencies.args,
        dependencies.vehicle,
        dependencies.zc_util,
        dependencies.logger,
    ))
    navigation = _build_navigation(dependencies, options)
    capture = _build_capture(dependencies, options)
    gap = ForcedGapState()
    tracking = SimTrackingControls(navigation, capture)
    source_clock = SourceTimestampClock(source.pose_source)
    frame_size = FrameSize.with_defaults(
        dependencies.mount.image_width,
        dependencies.mount.image_height,
    )
    ideal_camera = IdealCameraState(dependencies.mount.get_gimbal_data)
    evidence_recorder = create_projection_recorder(
        dependencies.vehicle.source_system, ideal_360=options.ideal_360,
        name_reader=lambda: dependencies.mount.name,
    )
    try:
        projector = _build_projector(
            dependencies,
            options,
            target_provider,
            source_clock,
            frame_size,
            ideal_camera,
            evidence_recorder,
        )
        publications = DetectionPublicationBuffer(
            source.store,
            record_outcome=source.record_outcome,
            fallback_source_name=lambda: source_name_from_gimbal(
                dependencies.mount.get_gimbal_data
            ),
        )
    except BaseException as error:
        rollback_projection_recorder(evidence_recorder, error)
        raise
    return SimRenderFoundation(
        target_provider,
        navigation,
        capture,
        gap,
        tracking,
        projector,
        publications,
        source_clock,
        ideal_camera,
        evidence_recorder,
    )


def build_pipeline(
    dependencies: SimDetectorDependencies,
    options: SimDetectorOptions,
    source: SimSourceGraph,
    render: SimRenderFoundation,
) -> SimDetectionPipeline:
    frame_size = FrameSize.with_defaults(
        dependencies.mount.image_width,
        dependencies.mount.image_height,
    )
    confirmation = ConfirmationCapture(
        CaptureCameraPort(dependencies.mount.get_k, frame_size),
        render.capture,
        dependencies.logger.debug,
    )
    gap_policy = ForcedDetectionGapPolicy(render.tracking, render.gap)
    tracking_update = SimTrackingUpdater(
        render.tracking,
        TrackingCameraPort(
            dependencies.mount.get_k,
            dependencies.mount.get_gimbal_data,
        ),
        dependencies.logger.debug,
        render.projector.diagnose_track_loss,
    )
    timestamps = SimFrameTimestampResolver(
        lambda value, *, record_emitted: source.clock.accept_attitude_timestamp(
            value,
            on_restart=source.pose_source.reset,
            record_emitted=record_emitted,
        ),
    )
    transactions = DetectionFrameTransactions(
        source.coordinator,
        render.publications.source_name,
    )
    context = SimDetectionContext(
        options.ideal_360,
        dependencies.mount.sync_zoom_from_hardware,
        render.target_provider.snapshot,
        render.projector.update,
        timestamps,
        transactions,
    )
    return SimDetectionPipeline(
        context,
        confirmation_capture=confirmation,
        forced_gap_policy=gap_policy,
        tracking_updater=tracking_update,
        evidence_recorder=render.evidence_recorder,
    )


def _build_navigation(
    dependencies: SimDetectorDependencies,
    options: SimDetectorOptions,
) -> Optional[GimbalNavigation]:
    if options.ideal_360:
        return None
    if options.tracking_config is None and options.zoom_config is None:
        return None
    return GimbalNavigation(
        dependencies.mount,
        dependencies.logger,
        tracking=options.tracking_config,
        zoom_config=options.zoom_config,
    )


def _build_capture(
    dependencies: SimDetectorDependencies,
    options: SimDetectorOptions,
) -> SimCaptureState:
    if not options.sim_assets_path or options.ideal_360:
        return SimCaptureState()
    frame_size = FrameSize.with_defaults(
        dependencies.mount.image_width,
        dependencies.mount.image_height,
    )
    renderer = options.frame_generator_factory(
        options.sim_assets_path,
        (frame_size.width_px, frame_size.height_px),
    )
    if renderer.is_available:
        dependencies.logger.info(
            f"DetectorSim: Frame generator enabled from {options.sim_assets_path}"
        )
    return SimCaptureState(renderer=renderer)


def _build_projector(
    dependencies: SimDetectorDependencies,
    options: SimDetectorOptions,
    targets: SimTargetCatalog,
    source_clock: SourceTimestampClock,
    frame_size: FrameSize,
    ideal_camera: IdealCameraState,
    evidence_recorder: Optional[ProjectionRunRecorder] = None,
) -> SimTargetProjector:
    camera = ProjectionCameraPort(
        dependencies.mount.get_k,
        dependencies.mount.get_gimbal_data,
        frame_size,
        dependencies.mount.is_valid,
    )
    def record_failure(message: str) -> None:
        if evidence_recorder is not None:
            evidence_recorder.failed_notice(message)
        dependencies.logger.single_warning(message, key='sim_projection_evidence_error')

    finite = FiniteTargetProjector(
        camera,
        dependencies.geo_ref.calc_uv,
        targets.snapshot,
        source_clock.now,
        evidence_sink=evidence_recorder if evidence_recorder is not None else projection_log_sink(
            dependencies.logger.debug, dependencies.vehicle.source_system,
            is_debug_enabled=lambda: dependencies.logger.is_enabled_for(CacheLogLevel.DEBUG),
        ) if not options.ideal_360 else None,
        evidence_error_sink=record_failure,
    )
    ideal = IdealTargetProjector(
        ideal_camera,
        frame_size,
        UasFrameConvention(
            dependencies.geo_ref.uas_seq,
            dependencies.geo_ref.degrees,
        ),
        source_clock.now,
    )
    return SimTargetProjector(options.ideal_360, finite, ideal)


__all__ = [
    "SimRenderFoundation",
    "build_pipeline",
    "build_render_foundation",
]
