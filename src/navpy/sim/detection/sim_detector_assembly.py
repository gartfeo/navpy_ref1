"""Thin composition root for one simulated detector."""

from __future__ import annotations

from navpy.sim.detection.sim_detector_config import (
    SimDetectorDependencies,
    SimDetectorOptions,
    SimDetectorPorts,
)
from navpy.sim.detection.sim_detector_controls import (
    SimDetectorIdentity,
    SimGeoControls,
    SimPoiControls,
    SimZoomControls,
)
from navpy.sim.camera.projection_run_recorder import rollback_projection_recorder
from navpy.sim.detection.sim_execution_composition import (
    build_execution_graph,
)
from navpy.sim.detection.sim_render_composition import (
    build_pipeline,
    build_render_foundation,
)
from navpy.sim.detection.sim_source_composition import build_source_graph


def build_sim_detector(
    dependencies: SimDetectorDependencies,
    options: SimDetectorOptions,
) -> SimDetectorPorts:
    source = build_source_graph(dependencies, options)
    render = build_render_foundation(dependencies, options, source)
    try:
        pipeline = build_pipeline(dependencies, options, source, render)
        execution = build_execution_graph(
            dependencies,
            options,
            source,
            render,
            pipeline,
        )
    except BaseException as error:
        rollback_projection_recorder(render.evidence_recorder, error)
        raise

    return SimDetectorPorts(
        SimDetectorIdentity(dependencies.mount),
        render.publications,
        render.tracking,
        SimZoomControls(
            render.navigation,
            render.tracking,
            render.capture,
        ),
        SimGeoControls(render.navigation),
        SimPoiControls(render.poi_provider),
        execution.lifecycle,
        execution.worker,
        pipeline,
        render.projector,
        render.navigation,
    )


__all__ = [
    "SimDetectorDependencies",
    "SimDetectorOptions",
    "SimDetectorPorts",
    "build_sim_detector",
]
