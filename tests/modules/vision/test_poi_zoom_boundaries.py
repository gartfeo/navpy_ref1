"""Hardware/session boundary contracts for source-driven POI zoom."""

from __future__ import annotations

import math
from types import SimpleNamespace
from unittest.mock import Mock

from navpy.modules.vision.poi_zoom_mount_adapters import MountZoomAdapter
from navpy.modules.vision.poi_zoom_tracker import (
    PoiZoomTracker,
    PoiZoomTrackerConfig,
    ZoomTrackingState,
)


def _poi(size_px: float) -> SimpleNamespace:
    side = size_px / math.sqrt(2.0)
    return SimpleNamespace(
        class_id=0,
        tracking_bbox_cxcywh=(960.0, 540.0, side, side),
        bbox_cxcywh=None,
    )


def _mount(
    mode: str,
    *,
    levels: tuple[object, ...] = ("1", "2", "3", "10"),
    current: str = "2",
) -> SimpleNamespace:
    gimbal = SimpleNamespace()
    if mode in {"absolute", "hybrid"}:
        gimbal.set_zoom = Mock(return_value=True)
    if mode in {"continuous", "hybrid"}:
        gimbal.zoom_in = Mock(return_value=True)
        gimbal.zoom_out = Mock(return_value=True)
        gimbal.zoom_hold = Mock(return_value=True)
    def start_continuous(direction: ZoomTrackingState) -> bool:
        if direction is ZoomTrackingState.ZOOMING_IN:
            return gimbal.zoom_in()
        return gimbal.zoom_out()

    return SimpleNamespace(
        gimbal=gimbal,
        image_width=1920,
        image_height=1080,
        zoom_calibration=None,
        get_k=Mock(
            return_value=(
                (1000.0, 0.0, 960.0),
                (0.0, 1000.0, 540.0),
                (0.0, 0.0, 1.0),
            )
        ),
        get_zoom_levels=Mock(return_value=list(levels)),
        get_current_zoom=Mock(return_value=current),
        get_current_zoom_command=(
            Mock(return_value=current)
            if mode in {"absolute", "hybrid"}
            else None
        ),
        get_fresh_zoom_command=Mock(return_value=current),
        get_fresh_zoom_sample_id=Mock(return_value=None),
        sync_zoom_from_hardware=Mock(return_value=True),
        command_zoom=(
            Mock(return_value=True)
            if mode in {"absolute", "hybrid"}
            else None
        ),
        set_zoom=Mock(return_value=True),
        supports_absolute_zoom=lambda: mode in {"absolute", "hybrid"},
        supports_continuous_zoom=lambda: mode in {"continuous", "hybrid"},
        start_continuous_zoom=start_continuous,
        hold_zoom=lambda: gimbal.zoom_hold(),
    )


def _tracker(mount: object) -> PoiZoomTracker:
    return PoiZoomTracker(
        mount,
        Mock(),
        PoiZoomTrackerConfig(target_pixels={"default": 100.0}),
    )


def test_failed_active_session_start_preserves_policy_and_demand() -> None:
    mount = _mount("continuous")
    tracker = _tracker(mount)
    tracker.update(_poi(20.0))
    tracker.update(_poi(20.0))
    tracker.set_size_demand(False)
    before = tracker.last_result
    mount.gimbal.zoom_hold.side_effect = OSError("link")

    assert tracker.start_session() is False

    assert tracker.size_demand is False
    assert tracker.last_result is before
    assert tracker.last_result.state is ZoomTrackingState.ZOOMING_IN
    mount.gimbal.zoom_hold.assert_called_once()

    mount.gimbal.zoom_hold.side_effect = None
    assert tracker.start_session() is True
    assert tracker.size_demand is True
    first = tracker.update(_poi(20.0))
    assert first.reason == "confirming"
    assert mount.gimbal.zoom_hold.call_count == 2


def test_invalid_level_is_removed_with_its_label_not_by_parallel_index() -> None:
    mount = _mount(
        "discrete",
        levels=("1", "bad", math.nan, True, -2, "03", "3"),
        current="1",
    )
    adapter = MountZoomAdapter(mount, Mock())

    assert adapter.capabilities.levels == ("1", "03")
    assert adapter.capabilities.minimum == 1.0
    assert adapter.capabilities.maximum == 3.0

    result = _tracker(mount).update(_poi(20.0))
    assert result.state is ZoomTrackingState.ZOOMING_IN
    mount.set_zoom.assert_called_once_with("03")


def test_fewer_than_two_valid_discrete_levels_is_unsupported() -> None:
    mount = _mount("discrete", levels=("bad", False, "2"), current="2")
    tracker = _tracker(mount)

    result = tracker.update(_poi(20.0))

    assert result.state is ZoomTrackingState.UNSUPPORTED
    mount.set_zoom.assert_not_called()


def test_absolute_command_waits_for_a_fresh_hardware_sample() -> None:
    mount = _mount("absolute")
    mount.get_fresh_zoom_sample_id.return_value = "sample-1"
    tracker = _tracker(mount)

    tracker.update(_poi(20.0))
    assert mount.command_zoom.call_args.args == ("10",)

    waiting = tracker.update(_poi(40.0))
    assert waiting.reason == "settling"
    assert mount.command_zoom.call_count == 1

    mount.get_fresh_zoom_sample_id.return_value = "sample-2"
    tracker.update(_poi(40.0))
    assert mount.command_zoom.call_count == 2
    assert mount.command_zoom.call_args.args == ("5",)


def test_discrete_command_waits_for_a_fresh_hardware_sample() -> None:
    mount = _mount("discrete", levels=("1", "3", "10"), current="1")
    mount.get_fresh_zoom_sample_id.return_value = "sample-1"
    tracker = _tracker(mount)

    tracker.update(_poi(20.0))
    mount.set_zoom.assert_called_once_with("3")

    mount.get_current_zoom.return_value = "3"
    waiting = tracker.update(_poi(20.0))
    assert waiting.reason == "settling"
    assert mount.set_zoom.call_count == 1

    mount.get_fresh_zoom_sample_id.return_value = "sample-2"
    tracker.update(_poi(20.0))
    assert mount.set_zoom.call_count == 2
    assert mount.set_zoom.call_args.args == ("10",)


def test_sync_oserror_fails_closed_without_actuation() -> None:
    mount = _mount("hybrid")
    mount.sync_zoom_from_hardware.side_effect = OSError("telemetry")
    tracker = _tracker(mount)

    result = tracker.update(_poi(20.0))

    assert result.state is ZoomTrackingState.HOLDING
    assert result.reason == "optics"
    mount.command_zoom.assert_not_called()
    mount.gimbal.zoom_in.assert_not_called()


def test_capability_oserror_is_unavailable_and_retries_without_actuation() -> None:
    mount = _mount("absolute")
    mount.get_zoom_levels.side_effect = [OSError("telemetry"), ["1", "2", "10"]]
    tracker = _tracker(mount)

    failed = tracker.update(_poi(20.0))

    assert failed.state is ZoomTrackingState.HOLDING
    assert failed.reason == "optics"
    mount.command_zoom.assert_not_called()

    recovered = tracker.update(_poi(20.0))
    assert recovered.state is ZoomTrackingState.ZOOMING_IN
    mount.command_zoom.assert_called_once()


def test_readback_oserrors_fail_closed_without_actuation() -> None:
    for provider_name in (
        "get_current_zoom_command",
        "get_fresh_zoom_command",
        "get_fresh_zoom_sample_id",
    ):
        mount = _mount("absolute")
        getattr(mount, provider_name).side_effect = OSError("telemetry")
        tracker = _tracker(mount)

        result = tracker.update(_poi(20.0))

        assert result.state is ZoomTrackingState.HOLDING
        assert result.reason == "readback"
        mount.command_zoom.assert_not_called()


def test_geometry_oserror_fails_closed_without_actuation() -> None:
    mount = _mount("hybrid")
    mount.get_k.side_effect = OSError("telemetry")
    tracker = _tracker(mount)

    result = tracker.update(_poi(20.0))

    assert result.state is ZoomTrackingState.HOLDING
    assert result.reason == "optics"
    mount.command_zoom.assert_not_called()
    mount.gimbal.zoom_in.assert_not_called()


def test_absolute_reset_never_requires_a_continuous_hold_capability() -> None:
    mount = _mount("absolute")
    tracker = _tracker(mount)
    tracker.update(_poi(20.0))

    assert tracker.reset() is True
    assert tracker.last_result.state is ZoomTrackingState.IDLE


def test_failed_reset_to_min_preserves_the_existing_session() -> None:
    mount = _mount("absolute")
    tracker = _tracker(mount)
    tracker.update(_poi(120.0))
    tracker.set_size_demand(False)
    prior = tracker.last_result
    mount.command_zoom.side_effect = OSError("link")

    assert tracker.reset_to_min() is False

    assert tracker.size_demand is False
    assert tracker.last_result is prior
    mount.command_zoom.assert_called_once_with("1")

    mount.command_zoom.side_effect = None
    assert tracker.reset_to_min() is True
    assert tracker.last_result.state is ZoomTrackingState.IDLE


def test_reset_to_min_command_waits_for_fresh_readback_before_followup() -> None:
    mount = _mount("absolute")
    mount.get_fresh_zoom_sample_id.return_value = "sample-1"
    tracker = _tracker(mount)

    assert tracker.reset_to_min() is True
    mount.command_zoom.assert_called_once_with("1")

    waiting = tracker.update(_poi(20.0))
    assert waiting.reason == "settling"
    assert mount.command_zoom.call_count == 1

    mount.get_fresh_zoom_sample_id.return_value = "sample-2"
    tracker.update(_poi(20.0))
    assert mount.command_zoom.call_count == 2
    assert mount.command_zoom.call_args.args == ("10",)


def test_reset_to_min_keeps_the_newest_hardware_sample_gate() -> None:
    mount = _mount("continuous")
    tracker = _tracker(mount)
    tracker.update(_poi(20.0))
    tracker.update(_poi(20.0))
    mount.gimbal.zoom_in.reset_mock()
    mount.get_fresh_zoom_sample_id.side_effect = [
        "hold-B",
        "minimum-C",
        "minimum-C",
        "minimum-C",
    ]

    assert tracker.reset_to_min() is True
    first = tracker.update(_poi(20.0))
    second = tracker.update(_poi(20.0))

    assert first.reason in {"confirming", "settling"}
    assert second.reason == "settling"
    mount.gimbal.zoom_in.assert_not_called()
