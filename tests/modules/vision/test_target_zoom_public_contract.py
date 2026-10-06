"""Stable public identities around the split target-zoom implementation."""

from __future__ import annotations

import inspect

import navpy.modules.vision as vision_package
from navpy.modules.vision import continuous_zoom_policy
from navpy.modules.vision import target_zoom_orchestrator
from navpy.modules.vision import target_zoom_tracker
from navpy.modules.vision import target_zoom_types
from navpy.modules.vision import vision_profiles
from navpy.modules.vision import vision_zoom_profile
from navpy.modules.vision import zoom_tracking_types


def test_facades_reexport_exact_owner_symbols() -> None:
    assert (
        vision_package.TargetZoomTracker
        is target_zoom_tracker.TargetZoomTracker
        is target_zoom_orchestrator.TargetZoomTracker
    )
    assert (
        vision_package.TargetZoomTrackerConfig
        is target_zoom_tracker.TargetZoomTrackerConfig
        is target_zoom_types.TargetZoomTrackerConfig
    )
    assert (
        target_zoom_tracker.ZoomTrackResult
        is target_zoom_types.ZoomTrackResult
    )
    assert (
        target_zoom_tracker.ZoomTrackingState
        is zoom_tracking_types.ZoomTrackingState
    )
    assert (
        continuous_zoom_policy.ContinuousZoomDecision
        is target_zoom_types.ContinuousZoomDecision
    )
    assert vision_profiles.build_zoom_config is vision_zoom_profile.build_zoom_config


def test_tracker_call_shape_remains_compatible() -> None:
    constructor = inspect.signature(target_zoom_tracker.TargetZoomTracker)
    assert tuple(constructor.parameters) == ("mount", "logger", "config", "clock")
    assert constructor.parameters["config"].default is None
    assert constructor.parameters["clock"].default is None

    update = inspect.signature(target_zoom_tracker.TargetZoomTracker.update)
    assert tuple(update.parameters) == ("self", "target", "now", "pointing")
    assert update.parameters["now"].default is None
    assert update.parameters["pointing"].default is None
    assert isinstance(
        target_zoom_tracker.TargetZoomTracker._absolute_target,
        property,
    )
    assert isinstance(
        inspect.getattr_static(
            target_zoom_tracker.TargetZoomTracker,
            "_extract_bbox_cxcywh",
        ),
        staticmethod,
    )


def test_config_and_result_field_contracts_are_explicit() -> None:
    assert tuple(target_zoom_types.TargetZoomTrackerConfig.__dataclass_fields__) == (
        "target_pixels",
    )
    assert tuple(target_zoom_types.ZoomTrackResult.__dataclass_fields__) == (
        "state",
        "has_target",
        "size_px",
        "target_pixels",
        "reason",
        "current_zoom",
        "desired_zoom",
        "command_zoom",
        "at_max_zoom",
        "transition_pending",
    )
