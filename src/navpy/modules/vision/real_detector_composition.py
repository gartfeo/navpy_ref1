"""Bounded composition root for the real detector."""

from __future__ import annotations

from navpy.exception_groups import ExceptionGroup

from navpy.modules.vision.real_detector_config import (
    DetectorDependencies,
    DetectorGimbalConfig,
    DetectorModelConfig,
    DetectorPipelineConfig,
    RealDetectorConfig,
    RealDetectorPorts,
)
from navpy.modules.vision.detector_ports import (
    GimbalSourceIdentity,
    IdentitySchedulerCadence,
    NonSimulationControls,
    PollingDetectionEvents,
)
from navpy.modules.vision.real_detector_controls import (
    DetectionQuery,
    DetectorGeoControl,
    DetectorTrackingControl,
)
from navpy.modules.vision.real_detector_models import (
    build_models,
    close_model_resources,
)
from navpy.modules.vision.real_gimbal_adapters import (
    GimbalGeoAdapter,
    GimbalTrackingAdapter,
)
from navpy.modules.vision.real_detector_runtime_composition import (
    build_runtime,
    build_state,
)
from navpy.modules.vision.real_detector_worker_composition import (
    build_diagnostics,
    build_lifecycle,
    build_reset,
    build_resources,
    build_workers,
)


def build_real_detector(
    dependencies: DetectorDependencies,
    config: RealDetectorConfig,
) -> RealDetectorPorts:
    state = build_state(config)
    models = build_models(dependencies.logger, config, state.metrics)
    resources = None
    try:
        resources = build_resources(state, models)
        runtime = build_runtime(dependencies, config, state, models)
        workers = build_workers(dependencies, config, state, models, runtime)
        diagnostics = build_diagnostics(
            dependencies,
            config,
            state,
            models,
            runtime,
        )
        lifecycle = build_lifecycle(
            dependencies,
            state,
            runtime,
            workers,
            diagnostics,
            resources,
        )
        return RealDetectorPorts(
            mount=dependencies.mount,
            identity=GimbalSourceIdentity(dependencies.mount.get_gimbal_data),
            events=PollingDetectionEvents(),
            simulation=NonSimulationControls(),
            cadence=IdentitySchedulerCadence(),
            compatibility_navigation=runtime.navigation,
            tracking=DetectorTrackingControl(
                None
                if runtime.navigation is None
                else GimbalTrackingAdapter(runtime.navigation),
                models.target_lock,
            ),
            geo=DetectorGeoControl(
                None
                if runtime.navigation is None
                else GimbalGeoAdapter(runtime.navigation)
            ),
            lifecycle=lifecycle,
            reset=build_reset(state, models),
            query=DetectionQuery(
                state.results,
                state.freshness,
                models.target_lock,
                config.pipeline.use_target_lock,
            ),
            diagnostics=diagnostics,
        )
    except Exception as creation_error:
        if resources is None:
            cleanup_errors = close_model_resources(
                dependencies.logger,
                tracker=models.tracker,
                appearance=models.appearance,
                yolo=models.yolo,
                deep_search=models.deep_search,
            )
        else:
            try:
                resources.close()
            except Exception as cleanup_error:
                cleanup_errors = (cleanup_error,)
            else:
                cleanup_errors = ()
        if cleanup_errors:
            raise ExceptionGroup(
                "detector construction and rollback failed",
                [creation_error, *cleanup_errors],
            ) from None
        raise


__all__ = [
    "DetectorDependencies",
    "DetectorGimbalConfig",
    "DetectorModelConfig",
    "DetectorPipelineConfig",
    "RealDetectorConfig",
    "RealDetectorPorts",
    "build_real_detector",
]
