"""State, frame association, and navigation runtime composition."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.core.command.gimbal.gimbal_navigation import GimbalNavigation
from navpy.core.perception.camera.board_clock_map import AttitudeBoardClock
from navpy.core.perception.camera.frame_provider import FrameProvider
from navpy.core.perception.camera.pts_read_map import IntegrationTimingSource
from navpy.core.perception.pipeline.integration_association import (
    IntegrationAssociation,
)
from navpy.core.perception.pipeline.real_detector_config import (
    DetectorDependencies,
    DetectorPipelineConfig,
    RealDetectorConfig,
)
from navpy.core.perception.pipeline.real_detector_models import RealDetectorModels
from navpy.core.perception.pipeline.real_gimbal_adapters import GimbalMeasurementAdapter
from navpy.core.perception.pipeline.real_detector_state import (
    ConfirmationFrameStore,
    DetectionResultStore,
    FreshnessPolicy,
    OverlayStore,
)
from navpy.core.perception.pipeline.real_detector_inference_state import (
    DeepSearchInbox,
    DetectionBatchInbox,
    InferenceGeneration,
)
from navpy.core.perception.pipeline.real_detector_runtime_state import (
    DetectorRunState,
    PipelineMutationGate,
    RuntimeMetrics,
)
from navpy.core.perception.pipeline.real_frame_association import FrameAssociationBuilder
from navpy.core.perception.pipeline.real_inference import DeepSearchChannel
from navpy.core.perception.pipeline.real_detection_mapper import (
    DetectedObjectMapper,
    DetectionMappingConfig,
)
from navpy.core.perception.pipeline.real_tracking import (
    TrackingNavigationSink,
    TrackingRecovery,
)


@dataclass(frozen=True)
class RealDetectorState:
    run: DetectorRunState
    detections: DetectionBatchInbox
    deep_detections: DeepSearchInbox
    results: DetectionResultStore
    overlays: OverlayStore
    confirmation_frames: ConfirmationFrameStore
    freshness: FreshnessPolicy
    metrics: RuntimeMetrics
    mutation_gate: PipelineMutationGate
    inference_generation: InferenceGeneration


@dataclass(frozen=True)
class RealDetectorRuntime:
    frame_provider: FrameProvider
    navigation: GimbalNavigation | None
    association: FrameAssociationBuilder
    mapper: DetectedObjectMapper
    navigation_sink: TrackingNavigationSink
    recovery: TrackingRecovery
    deep_channel: DeepSearchChannel


def build_state(config: RealDetectorConfig) -> RealDetectorState:
    track_period_s = 1.0 / max(1e-6, config.pipeline.track_hz)
    detections = DetectionBatchInbox()
    deep_detections = DeepSearchInbox()
    return RealDetectorState(
        run=DetectorRunState(),
        detections=detections,
        deep_detections=deep_detections,
        results=DetectionResultStore(),
        overlays=OverlayStore(),
        confirmation_frames=ConfirmationFrameStore(maximum_frames=32),
        freshness=FreshnessPolicy(track_period_s),
        metrics=RuntimeMetrics(),
        mutation_gate=PipelineMutationGate(),
        inference_generation=InferenceGeneration(detections, deep_detections),
    )


def _vehicle_air_speed(vehicle: object) -> Callable[[], float | None]:
    """Outside HITL only: the vehicle's own airspeed."""
    return lambda: vehicle.air_speed


def build_association(
    dependencies: DetectorDependencies,
    pipeline: DetectorPipelineConfig | None = None,
    integration_source: IntegrationTimingSource | None = None,
) -> FrameAssociationBuilder:
    vehicle = dependencies.vehicle
    mount = dependencies.mount
    calibration = None if pipeline is None else pipeline.capture_calibration
    source = "" if pipeline is None else str(pipeline.frame_source)
    integration = None
    air_speed_reader = _vehicle_air_speed(vehicle)
    if integration_source is not None:
        if pipeline is None or pipeline.law_airspeed is None:
            raise ValueError("integration timing needs the eligible airspeed reader")
        # HITL step 20: the eligible pitot source only, never VFR_HUD.
        air_speed_reader = pipeline.law_airspeed
        integration = IntegrationAssociation(
            integration_source,
            mount.gimbal_capture_history,
            None if pipeline is None else pipeline.integration_timing_log,
            None if pipeline is None else pipeline.fixed_zoom,
        )
    return FrameAssociationBuilder(
        attitude_sample_reader=lambda: vehicle.attitude_sample,
        mount_state_reader=mount.capture_frame_state,
        location_reader=lambda: vehicle.location(False),
        # Acquisition only (step 22): the capture-time camera position.
        position_sample_reader=lambda: getattr(vehicle, "position_sample", None),
        air_speed_reader=air_speed_reader,
        attitude_history_reader=vehicle.attitude_history,
        # From the LIVE connection (vehicle facet), never from configuration.
        link_identity_reader=lambda: vehicle.link_identity,
        calibration=calibration,
        source_id=source,
        integration=integration,
    )


def build_runtime(
    dependencies: DetectorDependencies,
    config: RealDetectorConfig,
    state: RealDetectorState,
    models: RealDetectorModels,
) -> RealDetectorRuntime:
    pipeline = config.pipeline
    integration_source = None
    if pipeline.integration_timing is not None:
        integration_source = IntegrationTimingSource(pipeline.integration_timing)
    frame_provider = FrameProvider(
        source=pipeline.frame_source,
        logger=dependencies.logger,
        # Timing only: maps a simulated frame's board time onto the ATTITUDE
        # receipt clock (npyc:// sources); it never reads a pose.
        board_clock=AttitudeBoardClock(
            lambda count: dependencies.vehicle.attitude_history(count)
        ),
        # Every published frame feeds the integration map once.
        publication_observer=(
            None if integration_source is None else integration_source.observe
        ),
        capture_wrapper=pipeline.capture_wrapper,
    )
    navigation = None
    if (
        config.gimbal.tracking is not None
        or config.gimbal.zoom is not None
        or config.gimbal.search is not None
        or pipeline.fixed_zoom is not None
    ):
        navigation = GimbalNavigation(
            dependencies.mount,
            dependencies.logger,
            tracking=config.gimbal.tracking,
            zoom_config=config.gimbal.zoom,
            fixed_zoom=pipeline.fixed_zoom,
            search=config.gimbal.search,
        )
    association = build_association(dependencies, pipeline, integration_source)
    return RealDetectorRuntime(
        frame_provider=frame_provider,
        navigation=navigation,
        association=association,
        mapper=DetectedObjectMapper(
            DetectionMappingConfig(pipeline.reference_height_m, pipeline.output_mode),
            state.confirmation_frames,
            models.board_pose,
        ),
        navigation_sink=TrackingNavigationSink(
            None if navigation is None else GimbalMeasurementAdapter(navigation),
            state.freshness,
        ),
        recovery=TrackingRecovery(
            models.bridge,
            models.appearance,
            models.identity,
            models.poi_lock,
            pipeline.use_poi_lock,
            state.metrics,
        ),
        deep_channel=DeepSearchChannel(
            models.deep_search is not None,
            models.deep_config,
            models.poi_lock,
            state.overlays,
            state.freshness,
            state.inference_generation,
        ),
    )


__all__ = [
    "RealDetectorRuntime",
    "RealDetectorState",
    "build_association",
    "build_runtime",
    "build_state",
]
