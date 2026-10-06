"""Regression contract for the source-driven POI-zoom controller."""

from __future__ import annotations

import ast
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from navpy.modules.vision.gimbal_rate_types import (
    GimbalTrackResult,
    TrackingState,
)
from navpy.modules.vision.poi_zoom_tracker import (
    PoiZoomTracker,
    PoiZoomTrackerConfig,
    ZoomTrackingState,
)


def _poi(size_px: float = 20.0, *, cx: float = 960.0) -> SimpleNamespace:
    side = size_px / math.sqrt(2.0)
    return SimpleNamespace(
        class_id=0,
        tracking_bbox_cxcywh=(cx, 540.0, side, side),
        bbox_cxcywh=None,
    )


class _ZoomGimbal:
    def __init__(self) -> None:
        self.set_zoom = Mock(return_value=True)
        self.zoom_in = Mock(return_value=True)
        self.zoom_out = Mock(return_value=True)
        self.zoom_hold = Mock(return_value=True)


class _Mount:
    def __init__(self, *, mode: str = "hybrid") -> None:
        self._mode = mode
        self.gimbal = _ZoomGimbal()
        self.image_width = 1920
        self.image_height = 1080
        self.zoom_calibration = None
        self.get_k = Mock(return_value=((1000.0, 0.0, 960.0), (0.0, 1000.0, 540.0), (0.0, 0.0, 1.0)))
        self.get_zoom_levels = Mock(return_value=["1", "2", "3", "10"])
        self.get_current_zoom = Mock(return_value="2")
        self.get_current_zoom_command = Mock(return_value="2")
        self.get_fresh_zoom_command = Mock(return_value="2")
        self.get_fresh_zoom_sample_id = Mock(return_value=None)
        self.sync_zoom_from_hardware = Mock(return_value=True)
        self.command_zoom = Mock(
            side_effect=lambda command: self.gimbal.set_zoom(command)
        )
        self.set_zoom = Mock(return_value=True)
        if mode == "absolute":
            del self.gimbal.zoom_in
            del self.gimbal.zoom_out
        elif mode == "continuous":
            del self.gimbal.set_zoom
            self.command_zoom = None
            self.get_current_zoom_command = None
        elif mode == "discrete":
            del self.gimbal.set_zoom
            del self.gimbal.zoom_in
            del self.gimbal.zoom_out
            self.command_zoom = None
            self.get_current_zoom_command = None

    def supports_absolute_zoom(self) -> bool:
        return self._mode in {"absolute", "hybrid"}

    def supports_continuous_zoom(self) -> bool:
        return self._mode in {"continuous", "hybrid"}

    def start_continuous_zoom(self, direction: ZoomTrackingState) -> bool:
        if direction is ZoomTrackingState.ZOOMING_IN:
            return self.gimbal.zoom_in()
        return self.gimbal.zoom_out()

    def hold_zoom(self) -> bool:
        return self.gimbal.zoom_hold()


def _tracker(mount: _Mount, pixels: float = 100.0) -> PoiZoomTracker:
    return PoiZoomTracker(
        mount=mount,
        logger=Mock(),
        config=PoiZoomTrackerConfig(target_pixels={"default": pixels}),
    )


def _commit_continuous_in(
    tracker: PoiZoomTracker,
    mount: _Mount,
) -> None:
    tracker.update(_poi(20.0))
    tracker.update(_poi(20.0))
    assert mount.gimbal.zoom_in.call_count == 1


def test_failed_nav_widen_is_not_consumed_and_retries() -> None:
    mount = _Mount(mode="hybrid")
    mount.command_zoom.side_effect = [False, True]
    tracker = _tracker(mount)

    tracker.set_size_demand(False)
    tracker.update(_poi(20.0))
    tracker.update(_poi(20.0))

    assert mount.command_zoom.call_count == 2
    assert mount.command_zoom.call_args_list[0] == mount.command_zoom.call_args_list[1]


def test_failed_continuous_start_reports_hold_and_remains_retryable() -> None:
    mount = _Mount(mode="continuous")
    mount.gimbal.zoom_in.side_effect = [False, True]
    tracker = _tracker(mount)

    tracker.update(_poi())
    failed = tracker.update(_poi())
    retried = tracker.update(_poi())

    assert failed.state is ZoomTrackingState.HOLDING
    assert failed.reason == "actuator-error"
    assert retried.state is ZoomTrackingState.ZOOMING_IN
    assert mount.gimbal.zoom_in.call_count == 2


def test_failed_hold_preserves_active_direction_and_retries() -> None:
    mount = _Mount(mode="continuous")
    tracker = _tracker(mount)
    _commit_continuous_in(tracker, mount)
    mount.gimbal.zoom_hold.side_effect = [False, True]

    tracker.update(_poi(120.0))
    failed = tracker.update(_poi(120.0))
    stopped = tracker.update(_poi(120.0))

    assert failed.state is ZoomTrackingState.ZOOMING_IN
    assert failed.reason == "actuator-error"
    assert stopped.state is ZoomTrackingState.HOLDING
    assert mount.gimbal.zoom_hold.call_count == 2


@pytest.mark.parametrize(
    "boundary,arm",
    [
        ("sync", lambda mount: mount.sync_zoom_from_hardware),
        ("absolute", lambda mount: mount.command_zoom),
        ("continuous", lambda mount: mount.gimbal.zoom_in),
        ("hold", lambda mount: mount.gimbal.zoom_hold),
        ("readback", lambda mount: mount.get_current_zoom_command),
        ("geometry", lambda mount: mount.get_k),
    ],
)
def test_programmer_type_errors_propagate(boundary: str, arm) -> None:
    mode = "absolute" if boundary in {"absolute", "readback"} else "continuous"
    mount = _Mount(mode=mode)
    callback = arm(mount)
    callback.side_effect = TypeError(f"{boundary} contract defect")
    tracker = _tracker(mount)

    if boundary == "continuous":
        tracker.update(_poi())
    elif boundary == "hold":
        _commit_continuous_in(tracker, mount)
        tracker.update(_poi(120.0))
    pointing = None
    if boundary == "geometry":
        pointing = GimbalTrackResult(
            state=TrackingState.TRACKING,
            has_poi=True,
            yaw_error=0.0,
            pitch_error=0.0,
            yaw_rate_estimate=0.0,
            pitch_rate_estimate=0.0,
            mature=True,
        )

    with pytest.raises(TypeError, match="contract defect"):
        tracker.update(_poi() if boundary != "hold" else _poi(120.0), pointing=pointing)


@pytest.mark.parametrize("mode", ["absolute", "continuous", "discrete"])
@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_nonfinite_readback_fails_closed_with_finite_result(
    mode: str,
    value: float,
) -> None:
    mount = _Mount(mode=mode)
    mount.get_current_zoom.return_value = value
    if callable(mount.get_current_zoom_command):
        mount.get_current_zoom_command.return_value = value
    mount.get_fresh_zoom_command.return_value = value
    tracker = _tracker(mount)

    result = tracker.update(_poi())

    assert result.state is ZoomTrackingState.HOLDING
    assert result.reason == "readback"
    for field in (result.size_px, result.target_pixels, result.current_zoom,
                  result.desired_zoom, result.command_zoom):
        assert field is None or math.isfinite(field)
    mount.command_zoom.assert_not_called() if callable(mount.command_zoom) else None
    mount.set_zoom.assert_not_called()
    mount.gimbal.zoom_in.assert_not_called() if hasattr(mount.gimbal, "zoom_in") else None


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_nonfinite_pointing_never_opens_zoom_in_gate(bad: float) -> None:
    mount = _Mount(mode="continuous")
    tracker = _tracker(mount)
    pointing = GimbalTrackResult(
        state=TrackingState.TRACKING,
        has_poi=True,
        yaw_error=bad,
        pitch_error=0.0,
        yaw_rate_estimate=0.0,
        pitch_rate_estimate=0.0,
        mature=True,
    )

    tracker.update(_poi(), pointing=pointing)
    result = tracker.update(_poi(), pointing=pointing)

    assert result.state is ZoomTrackingState.HOLDING
    assert result.reason == "centering"
    mount.gimbal.zoom_in.assert_not_called()


def test_config_copies_and_freezes_derived_target_thresholds() -> None:
    source = {"default": 20.0, "4": 35.0}
    config = PoiZoomTrackerConfig(target_pixels=source)
    source["4"] = 999.0

    assert config.target_pixels["4"] == 35.0
    with pytest.raises(TypeError):
        config.target_pixels["4"] = 40.0  # type: ignore[index]


@pytest.mark.parametrize(
    "pixels",
    [
        {},
        {"default": 0.0},
        {"default": math.nan},
        {"default": math.inf},
        {"default": True},
        {"default": 20.0, "4": -1.0},
    ],
)
def test_config_rejects_invalid_derived_thresholds(pixels: dict) -> None:
    with pytest.raises(ValueError):
        PoiZoomTrackerConfig(target_pixels=pixels)


def test_new_poi_session_resets_zoom_policy_and_recognition_demand() -> None:
    mount = _Mount(mode="continuous")
    tracker = _tracker(mount)
    tracker.set_size_demand(False)
    tracker.update(_poi(120.0))

    tracker.start_session()

    assert tracker.size_demand is True
    first = tracker.update(_poi())
    assert first.reason == "confirming"
    mount.gimbal.zoom_in.assert_not_called()


def test_active_continuous_drive_rebinds_to_replaced_gimbal() -> None:
    mount = _Mount(mode="continuous")
    tracker = _tracker(mount)
    _commit_continuous_in(tracker, mount)
    replacement = _ZoomGimbal()
    mount.gimbal = replacement

    first = tracker.update(_poi())
    second = tracker.update(_poi())

    assert first.reason == "confirming"
    assert second.state is ZoomTrackingState.ZOOMING_IN
    replacement.zoom_in.assert_called_once_with()


def test_replacement_forgets_old_sample_telemetry_capability() -> None:
    mount = _Mount(mode="continuous")
    mount.get_fresh_zoom_sample_id.return_value = "old-A"
    tracker = _tracker(mount)
    _commit_continuous_in(tracker, mount)
    replacement = _ZoomGimbal()
    mount.gimbal = replacement
    mount.get_fresh_zoom_sample_id.return_value = None

    first = tracker.update(_poi())
    second = tracker.update(_poi())

    assert first.reason == "confirming"
    assert second.state is ZoomTrackingState.ZOOMING_IN
    replacement.zoom_in.assert_called_once_with()


def test_active_absolute_target_rebinds_to_replaced_gimbal() -> None:
    mount = _Mount(mode="absolute")
    tracker = _tracker(mount)
    tracker.update(_poi())
    replacement = _ZoomGimbal()
    mount.gimbal = replacement

    result = tracker.update(_poi())

    assert result.state is ZoomTrackingState.ZOOMING_IN
    replacement.set_zoom.assert_called_once_with("10")


def test_zoom_leaves_have_no_mount_or_navigation_reach_through() -> None:
    root = Path(__file__).parents[3] / "src" / "navpy" / "modules" / "vision"
    files = sorted(root.glob("poi_zoom_*.py")) + [
        root / "continuous_zoom_policy.py",
        root / "continuous_zoom_rules.py",
    ]
    forbidden_names = {"_mount", "vehicle", "geo_ref", "location", "altitude"}
    for path in files:
        if path.name == "poi_zoom_mount_adapters.py":
            continue
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        names = {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)}
        attrs = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
        assert not (forbidden_names & (names | attrs)), path
