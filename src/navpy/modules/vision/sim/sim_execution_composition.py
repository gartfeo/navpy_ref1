"""Compose simulator worker cadence, reset, and lifecycle."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.modules.vision.sim.sim_detection_pipeline import SimDetectionPipeline
from navpy.modules.vision.sim.sim_detector_config import (
    SimDetectorDependencies,
    SimDetectorOptions,
)
from navpy.modules.vision.sim.sim_detector_lifecycle import SimDetectorLifecycle
from navpy.modules.vision.sim.sim_detector_loop import SimDetectorWorker
from navpy.modules.vision.sim.sim_detector_reset import SimDetectorReset
from navpy.modules.vision.sim.sim_render_composition import SimRenderFoundation
from navpy.modules.vision.sim.sim_source_composition import (
    SimSourceGraph,
    build_polling_reader,
)
from navpy.modules.vision.sim.scheduler_cadence_lease import (
    SchedulerCadenceLease,
)

SIM_DETECTOR_POLL_RATE_HZ = 30.0


@dataclass(frozen=True)
class SimExecutionGraph:
    worker: SimDetectorWorker
    lifecycle: SimDetectorLifecycle


def build_execution_graph(
    dependencies: SimDetectorDependencies,
    options: SimDetectorOptions,
    source: SimSourceGraph,
    render: SimRenderFoundation,
    pipeline: SimDetectionPipeline,
) -> SimExecutionGraph:
    cadence_lease = SchedulerCadenceLease.create(
        dependencies.vehicle.sim_speedup,
        dependencies.scheduler_cadence,
    )
    worker = SimDetectorWorker(
        coordinator=source.coordinator,
        polling_pose_reader=build_polling_reader(dependencies.vehicle),
        error=dependencies.logger.error,
        cadence_lease=cadence_lease,
        scheduler_period_s=1.0 / SIM_DETECTOR_POLL_RATE_HZ,
        ideal_360=options.ideal_360,
        detect_targets=pipeline.detect_targets,
        record_outcome=source.record_outcome,
    )
    resetter = SimDetectorReset(
        pose_source=source.pose_source,
        target_catalog=render.target_provider,
        capture_state=render.capture,
        gap_state=render.gap,
        tracking=render.tracking,
        camera_refresh=dependencies.mount.refresh,
        ideal_camera=render.ideal_camera if options.ideal_360 else None,
    )
    evidence_recorder = render.evidence_recorder
    lifecycle = SimDetectorLifecycle(
        coordinator=source.coordinator,
        activation=source.activation,
        pose_source=source.pose_source,
        worker=worker,
        resetter=resetter,
        info=dependencies.logger.info,
        diagnostic_close=evidence_recorder.close if evidence_recorder else None,
    )
    return SimExecutionGraph(worker, lifecycle)


__all__ = [
    "SIM_DETECTOR_POLL_RATE_HZ",
    "SimExecutionGraph",
    "build_execution_graph",
]
