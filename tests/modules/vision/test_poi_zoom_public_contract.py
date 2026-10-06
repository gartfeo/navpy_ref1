"""Stable public identities around the split POI-zoom implementation."""

from __future__ import annotations

import inspect

import navpy.modules.vision as vision_package
from navpy.modules.vision import continuous_zoom_policy
from navpy.modules.vision import poi_zoom_orchestrator
from navpy.modules.vision import poi_zoom_tracker
from navpy.modules.vision import poi_zoom_types
from navpy.modules.vision import vision_profiles
from navpy.modules.vision import vision_zoom_profile
from navpy.modules.vision import zoom_tracking_types


def test_facades_reexport_exact_owner_symbols() -> None:
    assert (
        vision_package.PoiZoomTracker
        is poi_zoom_tracker.PoiZoomTracker
        is poi_zoom_orchestrator.PoiZoomTracker
    )
    assert (
        vision_package.PoiZoomTrackerConfig
        is poi_zoom_tracker.PoiZoomTrackerConfig
        is poi_zoom_types.PoiZoomTrackerConfig
    )
    assert (
        poi_zoom_tracker.ZoomTrackResult
        is poi_zoom_types.ZoomTrackResult
    )
    assert (
        poi_zoom_tracker.ZoomTrackingState
        is zoom_tracking_types.ZoomTrackingState
    )
    assert (
        continuous_zoom_policy.ContinuousZoomDecision
        is poi_zoom_types.ContinuousZoomDecision
    )
    assert vision_profiles.build_zoom_config is vision_zoom_profile.build_zoom_config


def test_tracker_call_shape_remains_compatible() -> None:
    constructor = inspect.signature(poi_zoom_tracker.PoiZoomTracker)
    assert tuple(constructor.parameters) == ("mount", "logger", "config", "clock")
    assert constructor.parameters["config"].default is None
    assert constructor.parameters["clock"].default is None

    update = inspect.signature(poi_zoom_tracker.PoiZoomTracker.update)
    assert tuple(update.parameters) == ("self", "poi", "now", "pointing")
    assert update.parameters["now"].default is None
    assert update.parameters["pointing"].default is None
    assert isinstance(
        poi_zoom_tracker.PoiZoomTracker._absolute_target,
        property,
    )
    assert isinstance(
        inspect.getattr_static(
            poi_zoom_tracker.PoiZoomTracker,
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
