"""State, frame association, and navigation runtime composition."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.modules.navigation.gimbal_navigation import GimbalNavigation
from navpy.modules.vision.frame_provider import FrameProvider
from navpy.modules.vision.real_detector_config import (
    DetectorDependencies,
    DetectorPipelineConfig,
    RealDetectorConfig,
)
from navpy.modules.vision.real_detector_models import RealDetectorModels
from navpy.modules.vision.real_gimbal_adapters import GimbalMeasurementAdapter
from navpy.modules.vision.real_detector_state import (
    ConfirmationFrameStore,
    DeepSearchInbox,
    DetectionBatchInbox,
    DetectionResultStore,
    DetectorRunState,
    FreshnessPolicy,
    InferenceGeneration,
    OverlayStore,
    PipelineMutationGate,
    RuntimeMetrics,
)
from navpy.modules.vision.real_frame_association import FrameAssociationBuilder
from navpy.modules.vision.real_inference import DeepSearchChannel
from navpy.modules.vision.real_detection_mapper import (
    DetectedObjectMapper,
    DetectionMappingConfig,
)
from navpy.modules.vision.real_tracking import (
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


def build_association(
    dependencies: DetectorDependencies,
    pipeline: DetectorPipelineConfig | None = None,
) -> FrameAssociationBuilder:
    vehicle = dependencies.vehicle
    mount = dependencies.mount
    calibration = None if pipeline is None else pipeline.capture_calibration
    source = "" if pipeline is None else str(pipeline.frame_source)
    return FrameAssociationBuilder(
        attitude_sample_reader=lambda: vehicle.attitude_sample,
        mount_state_reader=mount.capture_frame_state,
        location_reader=lambda: vehicle.location(False),
        air_speed_reader=lambda: vehicle.air_speed,
        attitude_history_reader=vehicle.attitude_history,
        # From the LIVE connection (vehicle facet), never from configuration.
        link_identity_reader=lambda: vehicle.link_identity,
        calibration=calibration,
        source_id=source,
    )


def build_runtime(
    dependencies: DetectorDependencies,
    config: RealDetectorConfig,
    state: RealDetectorState,
    models: RealDetectorModels,
) -> RealDetectorRuntime:
    pipeline = config.pipeline
    frame_provider = FrameProvider(
        source=pipeline.frame_source,
        logger=dependencies.logger,
    )
    navigation = None
    if config.gimbal.tracking is not None or config.gimbal.zoom is not None:
        navigation = GimbalNavigation(
            dependencies.mount,
            dependencies.logger,
            tracking=config.gimbal.tracking,
            zoom_config=config.gimbal.zoom,
        )
    association = build_association(dependencies, pipeline)
    return RealDetectorRuntime(
        frame_provider=frame_provider,
        navigation=navigation,
        association=association,
        mapper=DetectedObjectMapper(
            DetectionMappingConfig(pipeline.reference_height_m, pipeline.output_mode),
            state.confirmation_frames,
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
