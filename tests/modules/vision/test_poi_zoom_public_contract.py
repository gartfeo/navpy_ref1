"""Stable public identities around the split POI-zoom implementation."""

from __future__ import annotations

import inspect

from navpy.modules.vision import continuous_zoom_policy
from navpy.modules.vision import poi_zoom_orchestrator
from navpy.modules.vision import poi_zoom_types


def test_facades_reexport_exact_owner_symbols() -> None:
    assert (
        continuous_zoom_policy.ContinuousZoomDecision
        is poi_zoom_types.ContinuousZoomDecision
    )


def test_tracker_call_shape_remains_compatible() -> None:
    constructor = inspect.signature(poi_zoom_orchestrator.PoiZoomTracker)
    assert tuple(constructor.parameters) == ("mount", "logger", "config", "clock")
    assert constructor.parameters["config"].default is None
    assert constructor.parameters["clock"].default is None

    update = inspect.signature(poi_zoom_orchestrator.PoiZoomTracker.update)
    assert tuple(update.parameters) == ("self", "poi", "now", "pointing")
    assert update.parameters["now"].default is None
    assert update.parameters["pointing"].default is None
    assert isinstance(
        poi_zoom_orchestrator.PoiZoomTracker._absolute_target,
        property,
    )
    assert isinstance(
        inspect.getattr_static(
            poi_zoom_orchestrator.PoiZoomTracker,
            "_extract_bbox_cxcywh",
        ),
        staticmethod,
    )


def test_config_and_result_field_contracts_are_explicit() -> None:
    assert tuple(poi_zoom_types.PoiZoomTrackerConfig.__dataclass_fields__) == (
        "target_pixels",
    )
    assert tuple(poi_zoom_types.ZoomTrackResult.__dataclass_fields__) == (
        "state",
        "has_poi",
        "size_px",
        "target_pixels",
        "reason",
        "current_zoom",
        "desired_zoom",
        "command_zoom",
        "at_max_zoom",
        "transition_pending",
    )
