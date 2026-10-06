"""Worker, lifecycle, diagnostics, and reset composition."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.modules.vehicle.pose_streams import (
    request_pose_streams,
    resolve_pose_stream_rate_hz,
)
from navpy.modules.vision.real_detector_config import (
    DetectorDependencies,
    RealDetectorConfig,
)
from navpy.modules.vision.real_detector_controls import DetectorResetController
from navpy.modules.vision.real_detector_diagnostics import (
    DetectorDiagnostics,
)
from navpy.modules.vision.real_detector_lifecycle import (
    DetectorLifecycle,
    DetectorResources,
    DetectorWorkers,
    PoseStreamRequester,
)
from navpy.modules.vision.real_detector_models import RealDetectorModels
from navpy.modules.vision.real_detector_overlay import (
    MountedDetectorOverlayRenderer,
)
from navpy.modules.vision.real_detector_runtime_composition import (
    RealDetectorRuntime,
    RealDetectorState,
)
from navpy.modules.vision.real_inference import DeepSearchLoop, DetectionLoop
from navpy.modules.vision.real_tracking import TrackingLoop
from navpy.modules.vision.real_tracking_batch import (
    TrackingAssociationProcessor,
    TrackingBatchProcessor,
    TrackingModels,
    TrackingPublications,
    TrackingResultPublisher,
)


@dataclass(frozen=True)
class RealDetectorWorkers:
    detection: DetectionLoop
    tracking: TrackingLoop
    deep_search: DeepSearchLoop | None


def build_workers(
    dependencies: DetectorDependencies,
    config: RealDetectorConfig,
    state: RealDetectorState,
    models: RealDetectorModels,
    runtime: RealDetectorRuntime,
) -> RealDetectorWorkers:
    pipeline = config.pipeline
    batch_processor = TrackingBatchProcessor(
        state.mutation_gate,
        TrackingAssociationProcessor(
            TrackingModels(models.tracker, models.identity, models.poi_lock),
            runtime.recovery,
            runtime.deep_channel,
            state.metrics,
            state.inference_generation,
            pipeline.use_poi_lock,
        ),
        TrackingResultPublisher(
            runtime.mapper,
            TrackingPublications(state.results, state.overlays),
            runtime.navigation_sink,
        ),
    )
    return RealDetectorWorkers(
        detection=DetectionLoop(
            state.run,
            1.0 / max(1e-6, pipeline.detect_hz),
            runtime.frame_provider,
            runtime.association,
            models.yolo,
            state.inference_generation,
            state.metrics,
            dependencies.logger,
        ),
        tracking=TrackingLoop(
            state.run,
            1.0 / max(1e-6, pipeline.track_hz),
            state.detections,
            state.results,
            batch_processor,
            runtime.navigation_sink,
            state.metrics,
            dependencies.logger,
        ),
        deep_search=(
            DeepSearchLoop(
                state.run,
                runtime.frame_provider,
                models.deep_search,
                models.deep_config,
                runtime.deep_channel,
                state.metrics,
                dependencies.logger,
            )
            if models.deep_search is not None
            else None
        ),
    )


def build_diagnostics(
    dependencies: DetectorDependencies,
    config: RealDetectorConfig,
    state: RealDetectorState,
    models: RealDetectorModels,
    runtime: RealDetectorRuntime,
) -> DetectorDiagnostics:
    return DetectorDiagnostics(
        state.run,
        runtime.frame_provider,
        state.overlays,
        state.freshness,
        state.metrics,
        models.identity,
        MountedDetectorOverlayRenderer(
            dependencies.mount,
            lambda: dependencies.vehicle.attitude,
        ),
        config.pipeline.use_poi_lock,
        config.debug,
        dependencies.logger,
    )


def build_resources(
    state: RealDetectorState,
    models: RealDetectorModels,
) -> DetectorResources:
    return DetectorResources(
        models.tracker,
        models.appearance,
        models.yolo,
        models.deep_search,
        state.confirmation_frames,
    )


def build_lifecycle(
    dependencies: DetectorDependencies,
    state: RealDetectorState,
    runtime: RealDetectorRuntime,
    workers: RealDetectorWorkers,
    diagnostics: DetectorDiagnostics,
    resources: DetectorResources,
) -> DetectorLifecycle:
    pose_streams = PoseStreamRequester(
        lambda: resolve_pose_stream_rate_hz(dependencies.vehicle),
        lambda rate_hz: request_pose_streams(
            dependencies.vehicle,
            dependencies.logger,
            rate_hz=rate_hz,
        ),
        runtime.association,
    )
    return DetectorLifecycle(
        state.run,
        runtime.frame_provider,
        DetectorWorkers(
            workers.detection,
            workers.tracking,
            workers.deep_search,
        ),
        resources,
        pose_streams,
        diagnostics,
        dependencies.logger,
    )


def build_reset(
    state: RealDetectorState,
    models: RealDetectorModels,
) -> DetectorResetController:
    return DetectorResetController(
        state.run,
        state.results,
        state.inference_generation,
        state.confirmation_frames,
        models.tracker,
        models.identity,
        models.poi_lock,
        models.bridge,
        state.mutation_gate,
    )


__all__ = [
    "RealDetectorWorkers",
    "build_diagnostics",
    "build_lifecycle",
    "build_reset",
    "build_resources",
    "build_workers",
]
