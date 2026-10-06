import math
import unittest
from unittest.mock import Mock, call

from navpy.modules.vision.continuous_zoom_policy import ContinuousZoomPolicy
from navpy.modules.vision.gimbal_rate_tracker import GimbalTrackResult, TrackingState
from navpy.modules.vision.peripheral.gimbal_abc import GimbalAbc
from navpy.modules.vision.vision_profiles import MIN_TRACK_PIXELS
from navpy.modules.vision.zoom_calibration import (
    ZoomCalibrationEntry,
    ZoomCalibrationTable,
)
from navpy.modules.vision.poi_zoom_tracker import (
    PoiZoomTracker,
    PoiZoomTrackerConfig,
    ZoomTrackingState,
)

_SQRT2 = math.sqrt(2.0)


def _create_poi(
    size_px=None,
    class_id=0,
    confirmation_size_px=None,
    *,
    cx=960.0,
    cy=540.0,
    width=None,
):
    """Build a mock POI whose live-tracking bbox has a specified
    DIAGONAL equal to ``size_px``.

    When ``width`` is omitted (None) the bbox is made square with
    side = ``size_px / sqrt(2)``, so ``diagonal = size_px``.
    This preserves all zoom-math (target_scale = target_pixels / diagonal)
    unchanged from the old height-only measure.

    Tests that exercise frame-containment pass an explicit ``width`` and
    ``cx`` to place the box at a specific position; those tests must use
    dimensions whose ``sqrt(w^2+h^2) < target_pixels`` (i.e., the box must
    still be "below recognition threshold").
    """
    poi = Mock()
    poi.class_id = class_id
    if size_px is None:
        poi.tracking_bbox_cxcywh = None
    else:
        if width is None:
            side = float(size_px) / _SQRT2
            w, h = side, side
        else:
            w, h = float(width), float(size_px)
        poi.tracking_bbox_cxcywh = (float(cx), float(cy), w, h)
    poi.bbox_cxcywh = None if confirmation_size_px is None else (
        float(cx), float(cy),
        float(confirmation_size_px) / _SQRT2,
        float(confirmation_size_px) / _SQRT2,
    )
    return poi


def _absolute_gimbal():
    gimbal = Mock(spec=["set_zoom", "zoom_hold"])
    gimbal.set_zoom.return_value = True
    gimbal.zoom_hold.return_value = True
    return gimbal


def _continuous_gimbal():
    gimbal = Mock(spec=["zoom_in", "zoom_out", "zoom_hold"])
    gimbal.zoom_in.return_value = True
    gimbal.zoom_out.return_value = True
    gimbal.zoom_hold.return_value = True
    return gimbal


def _hybrid_gimbal():
    gimbal = Mock(spec=["set_zoom", "zoom_in", "zoom_out", "zoom_hold"])
    gimbal.set_zoom.return_value = True
    gimbal.zoom_in.return_value = True
    gimbal.zoom_out.return_value = True
    gimbal.zoom_hold.return_value = True
    return gimbal


def _no_zoom_gimbal():
    return Mock(spec=[])


def _make_mount(
    *,
    levels=("1", "2", "3"),
    current_zoom="1",
    mode="absolute",
    min_zoom=1.0,
    max_zoom=10.0,
    zoom_step=0.1,
):
    mount = Mock()
    mount.get_zoom_levels.return_value = list(levels)
    mount.get_current_zoom.return_value = current_zoom
    mount.get_current_zoom_command.return_value = current_zoom
    mount.get_fresh_zoom_command.return_value = None
    mount.get_fresh_zoom_sample_id.return_value = None
    mount.command_zoom.return_value = True
    mount.set_zoom.return_value = True
    mount.sync_zoom_from_hardware.return_value = True
    mount.image_width = 1920
    mount.image_height = 1080
    mount.zoom_calibration = None
    if mode == "absolute":
        mount.gimbal = _absolute_gimbal()
    elif mode == "hybrid":
        mount.gimbal = _hybrid_gimbal()
    elif mode == "continuous":
        mount.gimbal = _continuous_gimbal()
    else:
        mount.gimbal = _no_zoom_gimbal()
    mount.supports_absolute_zoom.return_value = mode in {"absolute", "hybrid"}
    mount.supports_continuous_zoom.return_value = mode in {"continuous", "hybrid"}
    mount.start_continuous_zoom.side_effect = lambda direction: (
        mount.gimbal.zoom_in()
        if direction is ZoomTrackingState.ZOOMING_IN
        else mount.gimbal.zoom_out()
    )
    mount.hold_zoom.side_effect = lambda: mount.gimbal.zoom_hold()
    if mode in {"absolute", "continuous", "hybrid"}:
        count = int(round((max_zoom - min_zoom) / zoom_step))
        values = (min_zoom + index * zoom_step for index in range(count + 1))
        mount.zoom_calibration = ZoomCalibrationTable(
            entries=tuple(
                ZoomCalibrationEntry(f"{value:.6f}", value)
                for value in values
            )
        )
    return mount


def _cfg(**overrides) -> PoiZoomTrackerConfig:
    defaults = dict(target_pixels={"default": 100.0})
    defaults.update(overrides)
    return PoiZoomTrackerConfig(**defaults)


class _InheritedNoopZoomGimbal(GimbalAbc):
    def get_data(self):
        return Mock()

    def set_att(self, att):
        pass


class TestAbsoluteSetpoint(unittest.TestCase):
    def setUp(self):
        self.logger = Mock()

    def test_bbox_small_commands_computed_absolute_zoom_in(self):
        mount = _make_mount(current_zoom="2", mode="absolute")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        result = tracker.update(_create_poi(50.0), now=1.0)

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(result.target_pixels, 100.0)
        self.assertEqual(result.reason, "minimum")
        self.assertEqual(result.current_zoom, 2.0)
        self.assertAlmostEqual(result.desired_zoom, 4.0, places=5)
        self.assertEqual(result.command_zoom, 4.0)
        self.assertFalse(result.at_max_zoom)
        mount.command_zoom.assert_called_once_with("4")
        mount.set_zoom.assert_not_called()
        mount.gimbal.zoom_hold.assert_not_called()

    def test_bbox_large_without_frame_risk_holds_zoom(self):
        mount = _make_mount(current_zoom="4", mode="absolute")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        result = tracker.update(_create_poi(200.0), now=1.0)

        self.assertEqual(result.state, ZoomTrackingState.HOLDING)
        mount.command_zoom.assert_not_called()
        mount.set_zoom.assert_not_called()

    def test_bbox_near_frame_edge_commands_absolute_zoom_out(self):
        mount = _make_mount(current_zoom="4", mode="absolute")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        result = tracker.update(
            _create_poi(100.0, cx=80.0, cy=540.0, width=120.0),
            now=1.0,
        )

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_OUT)
        self.assertEqual(result.reason, "frame")
        mount.command_zoom.assert_called_once_with("3.7")
        mount.set_zoom.assert_not_called()

    def test_same_absolute_target_is_not_recommanded(self):
        mount = _make_mount(current_zoom="2", mode="absolute")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        tracker.update(_create_poi(50.0), now=1.1)

        mount.command_zoom.assert_called_once_with("4")

    def test_in_band_after_absolute_target_needs_no_continuous_hold(self):
        mount = _make_mount(current_zoom="2", mode="absolute")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        result = tracker.update(_create_poi(100.0), now=1.1)

        self.assertEqual(result.state, ZoomTrackingState.HOLDING)
        mount.gimbal.zoom_hold.assert_not_called()

    def test_poi_loss_clears_absolute_target_without_continuous_hold(self):
        mount = _make_mount(current_zoom="2", mode="absolute")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        result = tracker.update(None, now=1.1)

        self.assertEqual(result.state, ZoomTrackingState.IDLE)
        mount.gimbal.zoom_hold.assert_not_called()

    def test_log_bbox_sequence_does_not_zoom_out_above_minimum(self):
        mount = _make_mount(current_zoom="1", mode="absolute")
        tracker = PoiZoomTracker(
            mount=mount,
            logger=self.logger,
            config=_cfg(target_pixels={"default": 15.0}),
        )

        for index, size_px in enumerate((18.0, 9.0, 17.0, 33.0, 10.0, 17.0)):
            tracker.update(_create_poi(size_px), now=1.0 + index * 0.1)

        self.assertEqual(
            mount.command_zoom.call_args_list,
            [call("1.7"), call("1.5")],
        )
        mount.gimbal.zoom_hold.assert_not_called()

    def test_small_bbox_near_edge_commands_largest_safe_partial_zoom_in(self):
        mount = _make_mount(current_zoom="2", mode="absolute")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        # bbox: w=60, h=50 → diagonal=sqrt(6100)≈78.1 < target=100.
        # cx=200: left_ray=200-960-30=-790 → frame limit=873.6/790≈1.106.
        # target_scale=100/78.1≈1.280; 1.106 < 1.280 → frame-limited.
        # command_scale=1.106, command_zoom=quantize(2*1.106)=2.2.
        # full_target_zoom=2*1.280≈2.561.
        result = tracker.update(
            _create_poi(50.0, cx=200.0, cy=540.0, width=60.0),
            now=1.0,
        )

        _diag = math.sqrt(60.0**2 + 50.0**2)  # ≈78.102
        _target_scale = 100.0 / _diag
        _full_target_zoom = 2.0 * _target_scale

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(result.reason, "frame-limited")
        self.assertEqual(result.target_pixels, 100.0)
        self.assertEqual(result.current_zoom, 2.0)
        self.assertAlmostEqual(result.desired_zoom, _full_target_zoom, places=5)
        self.assertAlmostEqual(result.command_zoom, 2.2, places=5)
        self.assertFalse(result.at_max_zoom)
        mount.command_zoom.assert_called_once_with("2.2")

    def test_below_target_frame_limited_quantization_hold_is_not_stable(self):
        mount = _make_mount(current_zoom="2", mode="absolute")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        # bbox: w=60, h=50 → diagonal≈78.1 < target=100.
        # cx=120: left_ray=120-960-30=-870 → frame limit=873.6/870≈1.004.
        # target_scale≈1.280; 1.004 < 1.280 → frame-limited.
        # command_zoom=quantize(2*1.004)=quantize(2.008)=2.0 (no new command,
        # since it equals current_zoom="2") → HOLDING "frame-limited".
        result = tracker.update(
            _create_poi(50.0, cx=120.0, cy=540.0, width=60.0),
            now=1.0,
        )

        _diag = math.sqrt(60.0**2 + 50.0**2)  # ≈78.102
        _target_scale = 100.0 / _diag
        _full_target_zoom = 2.0 * _target_scale

        self.assertEqual(result.state, ZoomTrackingState.HOLDING)
        self.assertEqual(result.reason, "frame-limited")
        self.assertEqual(result.target_pixels, 100.0)
        self.assertEqual(result.current_zoom, 2.0)
        self.assertAlmostEqual(result.desired_zoom, _full_target_zoom, places=5)
        self.assertEqual(result.command_zoom, 2.0)
        self.assertFalse(result.at_max_zoom)
        self.assertFalse(tracker.is_zoom_stable)
        mount.command_zoom.assert_not_called()

    def test_at_max_zoom_below_target_is_stable(self):
        mount = _make_mount(current_zoom="3", mode="absolute", max_zoom=3.0)
        mount.get_fresh_zoom_command.return_value = "3"
        tracker = PoiZoomTracker(
            mount=mount,
            logger=self.logger,
            config=_cfg(),
        )

        result = tracker.update(_create_poi(50.0), now=1.0)

        self.assertEqual(result.state, ZoomTrackingState.HOLDING)
        self.assertEqual(result.reason, "max")
        self.assertEqual(result.target_pixels, 100.0)
        self.assertEqual(result.current_zoom, 3.0)
        self.assertAlmostEqual(result.desired_zoom, 6.0, places=5)
        self.assertEqual(result.command_zoom, 3.0)
        self.assertTrue(result.at_max_zoom)
        self.assertTrue(tracker.is_zoom_stable)
        mount.command_zoom.assert_not_called()

    def test_absolute_max_zoom_stability_requires_fresh_readback(self):
        mount = _make_mount(current_zoom="3", mode="absolute", max_zoom=3.0)
        mount.get_fresh_zoom_command.return_value = None
        tracker = PoiZoomTracker(
            mount=mount,
            logger=self.logger,
            config=_cfg(),
        )

        result = tracker.update(_create_poi(50.0), now=1.0)

        self.assertEqual(result.state, ZoomTrackingState.HOLDING)
        self.assertEqual(result.reason, "minimum")
        self.assertFalse(result.at_max_zoom)
        self.assertFalse(tracker.is_zoom_stable)
        mount.command_zoom.assert_not_called()

    def test_frame_violation_widens_even_when_full_target_zoom_exceeds_max(self):
        mount = _make_mount(current_zoom="3", mode="absolute", max_zoom=3.0)
        tracker = PoiZoomTracker(
            mount=mount,
            logger=self.logger,
            config=_cfg(),
        )

        result = tracker.update(
            _create_poi(50.0, cx=80.0, cy=540.0, width=120.0),
            now=1.0,
        )

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_OUT)
        self.assertEqual(result.reason, "frame")
        self.assertFalse(result.at_max_zoom)
        mount.command_zoom.assert_called_once_with("2.8")

    def test_quantized_noop_below_max_is_not_stable_or_at_max(self):
        mount = _make_mount(current_zoom="2", mode="absolute")
        tracker = PoiZoomTracker(
            mount=mount,
            logger=self.logger,
            config=_cfg(),
        )

        result = tracker.update(_create_poi(99.0), now=1.0)

        self.assertEqual(result.state, ZoomTrackingState.HOLDING)
        self.assertEqual(result.reason, "minimum")
        self.assertEqual(result.command_zoom, 2.0)
        self.assertFalse(result.at_max_zoom)
        self.assertFalse(tracker.is_zoom_stable)
        mount.command_zoom.assert_not_called()

    def test_sync_zoom_from_hardware_called_every_tick(self):
        mount = _make_mount(mode="absolute")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(100.0), now=1.0)
        tracker.update(_create_poi(50.0), now=1.1)
        tracker.update(_create_poi(200.0), now=1.2)

        self.assertEqual(mount.sync_zoom_from_hardware.call_count, 3)

    def test_sync_not_called_before_is_supported_passes(self):
        mount = _make_mount(levels=(), current_zoom=None, mode="none")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger)

        tracker.update(_create_poi(50.0), now=1.0)

        mount.sync_zoom_from_hardware.assert_not_called()

    def test_prefers_live_tracking_bbox_over_confirmation_bbox(self):
        mount = _make_mount(current_zoom="2", mode="absolute")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        result = tracker.update(
            _create_poi(50.0, confirmation_size_px=200.0), now=1.0,
        )

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        self.assertAlmostEqual(result.size_px, 50.0, places=5)
        mount.command_zoom.assert_called_once_with("4")

    def test_missing_current_zoom_falls_back_to_continuous_when_available(self):
        mount = _make_mount(current_zoom=None, mode="absolute")
        mount.get_current_zoom_command.return_value = None
        mount.gimbal.zoom_in = Mock(return_value=True)
        mount.gimbal.zoom_out = Mock(return_value=True)
        mount.supports_continuous_zoom.return_value = True
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        result = tracker.update(_create_poi(50.0), now=1.1)

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        mount.gimbal.zoom_in.assert_called_once()

    def test_reset_clears_absolute_state(self):
        mount = _make_mount(current_zoom="2", mode="absolute")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())
        tracker.update(_create_poi(50.0), now=1.0)
        self.assertEqual(tracker._absolute_target, "4")

        tracker.reset()

        self.assertIsNone(tracker._absolute_target)
        self.assertEqual(tracker.last_result.state, ZoomTrackingState.IDLE)
        self.assertFalse(tracker.last_result.at_max_zoom)

    def test_poi_loss_clears_at_max_state(self):
        mount = _make_mount(current_zoom="3", mode="absolute", max_zoom=3.0)
        mount.get_fresh_zoom_command.return_value = "3"
        tracker = PoiZoomTracker(
            mount=mount,
            logger=self.logger,
            config=_cfg(),
        )
        tracker.update(_create_poi(50.0), now=1.0)
        self.assertTrue(tracker.last_result.at_max_zoom)

        result = tracker.update(None, now=1.1)

        self.assertEqual(result.state, ZoomTrackingState.IDLE)
        self.assertFalse(result.at_max_zoom)


class TestContinuousFallback(unittest.TestCase):
    def setUp(self):
        self.logger = Mock()

    def test_hybrid_mount_confirm_phase_uses_stepped_drive(self):
        # CONFIRM/recognition phase (size_demand=True, the default): a hybrid
        # mount (set_zoom AND zoom_in/out) now drives the recognition zoom on
        # the CONTINUOUS stepped drive (zoom_in/out), NOT an absolute set_zoom
        # seek. An absolute magnifying jump triggers the real SIYI autofocus
        # hunt while the gimbal is still slewing the new POI; stepped zoom
        # keeps focus continuous and re-centers between steps. (This reverses
        # the 2026-06-28 "Fix 1" absolute-confirm routing; the stepped overshoot
        # it avoided is bounded by the center-between-steps gate — verified at
        # realistic speedup.) POI 50px / current_zoom 2x, demand 100px.
        mount = _make_mount(current_zoom="2", mode="hybrid")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        result = tracker.update(_create_poi(50.0), now=1.1)

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(result.reason, "minimum")
        mount.gimbal.zoom_in.assert_called_once()
        mount.command_zoom.assert_not_called()

    def test_hybrid_mount_nav_phase_uses_continuous_rate_drive(self):
        # NAV/track-retention phase (size_demand=False): the demand is the
        # 12px tracking floor, where there is nothing to overshoot, so the
        # hybrid mount keeps the continuous rate drive (guards uav_3's NAV
        # re-acquire). An 8px bbox is below the floor, so it must zoom in via
        # the continuous path, NOT the absolute setpoint.
        mount = _make_mount(current_zoom="2", mode="hybrid")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())
        tracker.set_size_demand(False)

        # First NAV tick is the one-shot FOV widen (absolute); subsequent
        # ticks use the continuous rate drive for track-retention.
        tracker.update(_create_poi(8.0), now=1.0)  # widen edge
        tracker.update(_create_poi(8.0), now=1.1)  # continuous confirming
        result = tracker.update(_create_poi(8.0), now=1.2)  # continuous commit

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        mount.gimbal.zoom_in.assert_called_once()

    def test_confirm_stepped_then_nav_widen_then_continuous_sequencing(self):
        # Switching branches mid-session (stepped confirm -> NAV widen ->
        # continuous track-retention) must not leave stale state. Confirm now
        # runs on the stepped drive; NAV entry snaps the FOV wide in one
        # absolute zoom-OUT (which supersedes the in-flight confirm drive and
        # clears _continuous_direction), then the continuous rate drive engages
        # cleanly on a sub-floor bbox.
        mount = _make_mount(current_zoom="5", mode="hybrid")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        # CONFIRM phase: stepped zoom-in (was absolute under the reverted Fix 1).
        tracker.update(_create_poi(50.0), now=1.0)
        confirm = tracker.update(_create_poi(50.0), now=1.1)
        self.assertEqual(confirm.state, ZoomTrackingState.ZOOMING_IN)
        mount.gimbal.zoom_in.assert_called_once()
        mount.command_zoom.assert_not_called()
        mount.gimbal.zoom_in.reset_mock()

        # NAV entry: first tick widens the FOV to min in one absolute seek,
        # superseding the in-flight stepped confirm drive.
        tracker.set_size_demand(False)
        widen = tracker.update(_create_poi(8.0), now=1.2)
        self.assertEqual(widen.state, ZoomTrackingState.ZOOMING_OUT)
        self.assertEqual(widen.reason, "nav-widen")
        mount.command_zoom.assert_called_once_with("1")

        # Subsequent NAV ticks: continuous rate drive on a sub-floor bbox.
        tracker.update(_create_poi(8.0), now=1.3)  # continuous confirming
        nav = tracker.update(_create_poi(8.0), now=1.4)  # continuous commit
        self.assertEqual(nav.state, ZoomTrackingState.ZOOMING_IN)
        mount.gimbal.zoom_in.assert_called_once()

    def test_nav_entry_widens_fov_to_min_in_one_seek(self):
        # On the recognition->retention edge, the FOV snaps to min_zoom (1x) in
        # a single absolute command, instead of a slow reactive continuous
        # zoom-out — so the dive starts wide (max gimbal-lag tolerance).
        mount = _make_mount(current_zoom="5", mode="hybrid")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())
        tracker.set_size_demand(False)

        result = tracker.update(_create_poi(200.0), now=1.0)

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_OUT)
        self.assertEqual(result.reason, "nav-widen")
        mount.command_zoom.assert_called_once_with("1")
        # One-shot: the widen does not re-fire on the next tick (a 200px bbox
        # is far above the 12px floor, so the retention path just holds).
        mount.command_zoom.reset_mock()
        tracker.update(_create_poi(200.0), now=1.1)
        mount.command_zoom.assert_not_called()

    def test_bbox_small_fires_zoom_in_without_absolute_support(self):
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        result = tracker.update(_create_poi(50.0), now=1.1)

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        mount.gimbal.zoom_in.assert_called_once()
        mount.command_zoom.assert_not_called()

    def test_below_target_within_old_band_still_zooms_in(self):
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(90.0), now=1.0)
        result = tracker.update(_create_poi(90.0), now=1.1)

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        self.assertFalse(tracker.is_zoom_stable)
        mount.gimbal.zoom_in.assert_called_once()

    def test_frame_violation_zooms_out(self):
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(
            _create_poi(100.0, cx=80.0, cy=540.0, width=120.0), now=1.0,
        )
        result = tracker.update(
            _create_poi(100.0, cx=80.0, cy=540.0, width=120.0),
            now=1.1,
        )

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_OUT)
        self.assertEqual(result.reason, "frame")
        mount.gimbal.zoom_out.assert_called_once()
        mount.gimbal.zoom_in.assert_not_called()

    def test_frame_violation_at_fresh_min_zoom_holds_if_poi_large_enough(self):
        mount = _make_mount(current_zoom="1", mode="continuous")
        mount.get_fresh_zoom_command.return_value = "1"
        tracker = PoiZoomTracker(
            mount=mount,
            logger=self.logger,
            config=_cfg(),
        )

        result = tracker.update(
            _create_poi(100.0, cx=80.0, cy=540.0, width=120.0),
            now=1.0,
        )

        self.assertEqual(result.state, ZoomTrackingState.HOLDING)
        self.assertEqual(result.reason, "frame")
        self.assertEqual(result.current_zoom, 1.0)
        self.assertTrue(tracker.is_zoom_stable)
        mount.gimbal.zoom_out.assert_not_called()

    def test_frame_violation_at_fresh_min_zoom_below_target_holds_unstable(self):
        mount = _make_mount(current_zoom="1", mode="continuous")
        mount.get_fresh_zoom_command.return_value = "1"
        tracker = PoiZoomTracker(
            mount=mount,
            logger=self.logger,
            config=_cfg(),
        )

        # bbox: w=60, h=50 → diagonal≈78.1 < target=100.
        # cx=80: left_ray=80-960-30=-910 → frame limit=873.6/910≈0.960 < 1 → violation.
        # At min zoom (fresh="1") → HOLDING "frame"; size<target → not stable.
        result = tracker.update(
            _create_poi(50.0, cx=80.0, cy=540.0, width=60.0),
            now=1.0,
        )

        self.assertEqual(result.state, ZoomTrackingState.HOLDING)
        self.assertEqual(result.reason, "frame")
        self.assertFalse(tracker.is_zoom_stable)
        mount.gimbal.zoom_out.assert_not_called()

    def test_below_target_frame_limited_holds_unstable(self):
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        # bbox: w=60, h=50 → diagonal≈78.1 < target=100.
        # cx=116.4: left_ray=116.4-960-30=-873.6 → frame limit=873.6/873.6=1.0 ≤ 1.0.
        # Continuous path: _max_contained_zoom_in_scale ≤ 1.0 → HOLDING "frame-limited".
        # size_px<target → not stable.
        result = tracker.update(
            _create_poi(50.0, cx=116.4, cy=540.0, width=60.0),
            now=1.0,
        )

        self.assertEqual(result.state, ZoomTrackingState.HOLDING)
        self.assertEqual(result.reason, "frame-limited")
        self.assertFalse(result.at_max_zoom)
        self.assertFalse(tracker.is_zoom_stable)
        mount.gimbal.zoom_in.assert_not_called()
        mount.gimbal.zoom_out.assert_not_called()

    def test_fresh_max_zoom_below_target_holds_stable(self):
        mount = _make_mount(current_zoom="3", mode="continuous", max_zoom=3.0)
        mount.get_fresh_zoom_command.return_value = "3"
        tracker = PoiZoomTracker(
            mount=mount,
            logger=self.logger,
            config=_cfg(),
        )

        result = tracker.update(_create_poi(50.0), now=1.0)

        self.assertEqual(result.state, ZoomTrackingState.HOLDING)
        self.assertEqual(result.reason, "max")
        self.assertEqual(result.current_zoom, 3.0)
        self.assertTrue(result.at_max_zoom)
        self.assertTrue(tracker.is_zoom_stable)
        mount.gimbal.zoom_in.assert_not_called()

    def test_camera_zoom_fallback_does_not_prove_continuous_max(self):
        mount = _make_mount(current_zoom="3", mode="continuous", max_zoom=3.0)
        mount.get_fresh_zoom_command.return_value = None
        tracker = PoiZoomTracker(
            mount=mount,
            logger=self.logger,
            config=_cfg(),
        )

        tracker.update(_create_poi(50.0), now=1.0)
        result = tracker.update(_create_poi(50.0), now=1.1)

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        self.assertFalse(result.at_max_zoom)
        self.assertFalse(tracker.is_zoom_stable)
        mount.gimbal.zoom_in.assert_called_once()

    def test_same_direction_continues_without_new_command(self):
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        tracker.update(_create_poi(60.0), now=1.1)
        tracker.update(_create_poi(80.0), now=1.2)

        mount.gimbal.zoom_in.assert_called_once()
        mount.gimbal.zoom_hold.assert_not_called()

    def test_above_minimum_holds_active_continuous_drive(self):
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        tracker.update(_create_poi(50.0), now=1.1)
        tracker.update(_create_poi(200.0), now=1.2)
        result = tracker.update(_create_poi(200.0), now=1.3)

        self.assertEqual(result.state, ZoomTrackingState.HOLDING)
        self.assertEqual(result.reason, "above-minimum")
        mount.gimbal.zoom_in.assert_called_once()
        mount.gimbal.zoom_hold.assert_called_once()
        mount.gimbal.zoom_out.assert_not_called()

    def test_poi_loss_stops_continuous_drive(self):
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        tracker.update(_create_poi(50.0), now=1.1)
        result = tracker.update(None, now=1.2)

        self.assertEqual(result.state, ZoomTrackingState.IDLE)
        mount.gimbal.zoom_hold.assert_called_once()

    def test_reset_stops_active_continuous_drive(self):
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())
        tracker.update(_create_poi(50.0), now=1.0)
        tracker.update(_create_poi(50.0), now=1.1)

        tracker.reset()

        mount.gimbal.zoom_hold.assert_called_once()
        self.assertEqual(tracker.last_result.state, ZoomTrackingState.IDLE)


class TestDiscreteFallback(unittest.TestCase):
    """When mount lacks absolute and continuous zoom, fall back to labels."""

    def setUp(self):
        self.logger = Mock()

    def test_no_absolute_or_continuous_uses_discrete(self):
        mount = _make_mount(
            levels=("1", "2", "3"), current_zoom="1", mode="none",
        )
        tracker = PoiZoomTracker(
            mount=mount, logger=self.logger,
            config=_cfg(),
        )

        result = tracker.update(_create_poi(50.0), now=1.0)

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        mount.set_zoom.assert_called_once_with("2")
        self.assertTrue(
            any("discrete" in str(c).lower()
                for c in self.logger.warning.call_args_list)
        )

    def test_discrete_below_target_within_old_band_steps_in(self):
        mount = _make_mount(
            levels=("1", "2", "3"), current_zoom="1", mode="none",
        )
        tracker = PoiZoomTracker(
            mount=mount, logger=self.logger,
            config=_cfg(),
        )

        result = tracker.update(_create_poi(90.0), now=1.0)

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        self.assertFalse(tracker.is_zoom_stable)
        mount.set_zoom.assert_called_once_with("2")

    def test_discrete_max_below_target_within_old_band_is_at_max_stable(self):
        mount = _make_mount(
            levels=("1", "2", "3"), current_zoom="3", mode="none",
        )
        mount.get_fresh_zoom_command.return_value = "3"
        tracker = PoiZoomTracker(
            mount=mount, logger=self.logger,
            config=_cfg(),
        )

        result = tracker.update(_create_poi(90.0), now=1.0)

        self.assertEqual(result.state, ZoomTrackingState.HOLDING)
        self.assertEqual(result.reason, "max")
        self.assertTrue(result.at_max_zoom)
        self.assertTrue(tracker.is_zoom_stable)
        mount.set_zoom.assert_not_called()

    def test_discrete_max_below_target_requires_fresh_readback_for_stability(self):
        mount = _make_mount(
            levels=("1", "2", "3"), current_zoom="3", mode="none",
        )
        mount.get_fresh_zoom_command.return_value = None
        tracker = PoiZoomTracker(
            mount=mount, logger=self.logger,
            config=_cfg(),
        )

        result = tracker.update(_create_poi(90.0), now=1.0)

        self.assertEqual(result.state, ZoomTrackingState.HOLDING)
        self.assertEqual(result.reason, "limit")
        self.assertFalse(result.at_max_zoom)
        self.assertFalse(tracker.is_zoom_stable)
        mount.set_zoom.assert_not_called()

    def test_discrete_uses_each_new_hardware_readback_without_wall_cooldown(self):
        mount = _make_mount(
            levels=("1", "2", "3"), current_zoom="1", mode="none",
        )
        mount.get_current_zoom.side_effect = ["1", "2"]
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        tracker.update(_create_poi(50.0), now=1.5)

        self.assertEqual(mount.set_zoom.call_args_list, [call("2"), call("3")])

    def test_unsupported_without_any_zoom(self):
        mount = _make_mount(levels=(), current_zoom=None, mode="none")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger)

        result = tracker.update(_create_poi(50.0), now=1.0)

        self.assertEqual(result.state, ZoomTrackingState.UNSUPPORTED)

    def test_inherited_gimbal_noop_zoom_methods_are_not_supported(self):
        mount = _make_mount(levels=(), current_zoom=None, mode="none")
        mount.gimbal = _InheritedNoopZoomGimbal()
        tracker = PoiZoomTracker(mount=mount, logger=self.logger)

        result = tracker.update(_create_poi(50.0), now=1.0)

        self.assertEqual(result.state, ZoomTrackingState.UNSUPPORTED)
        mount.sync_zoom_from_hardware.assert_not_called()


class TestConfigValidation(unittest.TestCase):
    def test_removed_tuning_knobs_are_not_configurable(self):
        for name in (
            "tracking_pixels",
            "target_band_ratio",
            "centering_divergence_deadband_frac",
            "min_zoom_interval",
            "absolute_command_step",
            "frame_margin_ratio",
            "min_zoom",
            "max_zoom",
        ):
            with self.subTest(name=name), self.assertRaises(TypeError):
                PoiZoomTrackerConfig(**{name: 1.0})

    def test_accepts_defaults(self):
        PoiZoomTrackerConfig()

    def test_threshold_mapping_is_immutable(self):
        cfg = PoiZoomTrackerConfig(target_pixels={"default": 20.0})
        with self.assertRaises(TypeError):
            cfg.target_pixels["default"] = 30.0


class TestIsZoomStable(unittest.TestCase):
    def setUp(self):
        self.logger = Mock()

    def test_idle_before_any_update_is_not_stable(self):
        mount = _make_mount(mode="absolute")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())
        self.assertFalse(tracker.is_zoom_stable)

    def test_zooming_is_not_stable(self):
        mount = _make_mount(current_zoom="2", mode="absolute")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())
        tracker.update(_create_poi(50.0), now=1.0)
        self.assertEqual(tracker.last_result.state, ZoomTrackingState.ZOOMING_IN)
        self.assertFalse(tracker.is_zoom_stable)

    def test_holding_with_poi_is_stable(self):
        mount = _make_mount(mode="absolute")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())
        # Use 200.0 (clearly above target=100) so floating-point rounding in
        # the diagonal can never make size_px < target_pixels.
        tracker.update(_create_poi(200.0), now=1.0)
        self.assertEqual(tracker.last_result.state, ZoomTrackingState.HOLDING)
        self.assertTrue(tracker.is_zoom_stable)

    def test_poi_loss_is_not_stable(self):
        mount = _make_mount(mode="absolute")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())
        tracker.update(None, now=1.0)
        self.assertEqual(tracker.last_result.state, ZoomTrackingState.IDLE)
        self.assertFalse(tracker.is_zoom_stable)

    def test_unsupported_is_stable(self):
        mount = _make_mount(levels=(), current_zoom=None, mode="none")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger)
        tracker.update(_create_poi(50.0), now=1.0)
        self.assertEqual(tracker.last_result.state, ZoomTrackingState.UNSUPPORTED)
        self.assertTrue(tracker.is_zoom_stable)

    def _settling_idle_tracker(self):
        # Commit an IN drive, lose the POI (hold records the sample),
        # then reacquire on the SAME sample so the tracker reports settling.
        mount = _make_mount(mode="continuous")
        mount.get_fresh_zoom_sample_id.return_value = "1"
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())
        tracker.update(_create_poi(50.0), now=1.0)
        tracker.update(_create_poi(50.0), now=1.1)
        tracker.update(None, now=1.2)
        return tracker

    def test_settling_hold_below_target_is_not_stable(self):
        tracker = self._settling_idle_tracker()
        result = tracker.update(_create_poi(90.0), now=1.3)
        self.assertEqual(result.reason, "settling")
        self.assertEqual(result.state, ZoomTrackingState.HOLDING)
        self.assertFalse(tracker.is_zoom_stable)

    def test_steady_hold_above_target_is_stable_on_stale_sample(self):
        # A steady state needs no transition, so even on a stale sample
        # it reports its genuine reason and keeps confirmation viable.
        tracker = self._settling_idle_tracker()
        result = tracker.update(_create_poi(160.0), now=1.3)
        self.assertEqual(result.reason, "above-minimum")
        self.assertEqual(result.state, ZoomTrackingState.HOLDING)
        self.assertTrue(tracker.is_zoom_stable)

    def test_settling_active_drive_is_not_stable(self):
        mount = _make_mount(mode="continuous")
        mount.get_fresh_zoom_sample_id.return_value = "1"
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())
        tracker.update(_create_poi(50.0), now=1.0)
        tracker.update(_create_poi(50.0), now=1.1)
        result = tracker.update(_create_poi(160.0), now=1.2)
        self.assertEqual(result.reason, "settling")
        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        self.assertFalse(tracker.is_zoom_stable)


class TestContinuousZoomPolicy(unittest.TestCase):
    """Decision table for the pure continuous policy (no hardware)."""

    def _decide(self, policy=None, **overrides):
        defaults = dict(
            size_px=50.0,
            target_pixels=100.0,
            containment_violation_scale=1.0,
            zoom_in_reserve_scale=4.0,
            at_min_zoom=False,
            at_max_zoom=False,
            active_direction=None,
            sample_advanced=True,
            optical_fresh=True,
            centered=True,
        )
        defaults.update(overrides)
        policy = policy or ContinuousZoomPolicy()
        policy.decide(**defaults)
        return policy.decide(**defaults)

    def test_containment_violation_zooms_out(self):
        decision = self._decide(containment_violation_scale=0.8)
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_OUT)
        self.assertEqual(decision.reason, "frame")

    def test_containment_violation_at_min_zoom_holds(self):
        decision = self._decide(
            containment_violation_scale=0.8, at_min_zoom=True,
        )
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "frame")
        self.assertFalse(decision.at_max_zoom)

    def test_containment_violation_wins_over_above_minimum(self):
        decision = self._decide(
            size_px=200.0, containment_violation_scale=0.8,
        )
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_OUT)
        self.assertEqual(decision.reason, "frame")

    def test_above_minimum_holds(self):
        decision = self._decide(size_px=100.0)
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "above-minimum")
        self.assertFalse(decision.at_max_zoom)

    def test_above_minimum_wins_over_max_zoom(self):
        decision = self._decide(size_px=150.0, at_max_zoom=True)
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "above-minimum")
        self.assertFalse(decision.at_max_zoom)

    def test_below_minimum_at_max_zoom_holds_at_max(self):
        decision = self._decide(at_max_zoom=True)
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "max")
        self.assertTrue(decision.at_max_zoom)

    def test_below_minimum_without_reserve_holds_frame_limited(self):
        decision = self._decide(zoom_in_reserve_scale=1.0)
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "frame-limited")
        self.assertFalse(decision.at_max_zoom)

    def test_below_minimum_with_full_reserve_zooms_in(self):
        # bbox 50 -> target 100 needs reserve >= 2.0 to reach the minimum
        # while staying inside the frame margin.
        decision = self._decide(zoom_in_reserve_scale=2.5)
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(decision.reason, "minimum")

    def test_unknown_geometry_falls_through_to_zoom_in(self):
        decision = self._decide(
            containment_violation_scale=None, zoom_in_reserve_scale=None,
        )
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(decision.reason, "minimum")

    def test_policy_exposes_no_tunable_band(self):
        with self.assertRaises(TypeError):
            ContinuousZoomPolicy(target_band_ratio=0.2)


class TestContinuousZoomPolicyHysteresis(unittest.TestCase):
    """Size-side hysteresis latch lifecycle (band ratio 0.15)."""

    def _decide(self, policy, **overrides):
        defaults = dict(
            size_px=50.0,
            target_pixels=100.0,
            containment_violation_scale=1.0,
            zoom_in_reserve_scale=4.0,
            at_min_zoom=False,
            at_max_zoom=False,
            active_direction=None,
            sample_advanced=True,
            optical_fresh=True,
            centered=True,
        )
        defaults.update(overrides)
        policy.decide(**defaults)
        return policy.decide(**defaults)

    def _arm(self, policy):
        # An active IN drive reaching target_pixels arms the latch.
        decision = self._decide(
            policy, size_px=110.0,
            active_direction=ZoomTrackingState.ZOOMING_IN,
        )
        self.assertEqual(decision.reason, "above-minimum")
        return policy

    def test_dip_inside_band_after_in_hold_is_suppressed(self):
        policy = self._arm(ContinuousZoomPolicy())
        decision = self._decide(policy, size_px=90.0)
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "in-band")

    def test_dip_below_band_edge_rearms_zoom_in(self):
        policy = self._arm(ContinuousZoomPolicy())
        decision = self._decide(policy, size_px=84.0)
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(decision.reason, "minimum")

    def test_rearm_clears_latch_for_next_cycle(self):
        policy = self._arm(ContinuousZoomPolicy())
        self._decide(policy, size_px=84.0)  # re-arms IN, clears latch
        decision = self._decide(policy, size_px=99.0)
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)

    def test_above_minimum_without_active_in_does_not_arm_latch(self):
        policy = ContinuousZoomPolicy()
        decision = self._decide(policy, size_px=110.0)
        self.assertEqual(decision.reason, "above-minimum")
        decision = self._decide(policy, size_px=99.0)
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)

    def test_in_band_hold_at_max_zoom_reports_max(self):
        policy = self._arm(ContinuousZoomPolicy())
        decision = self._decide(policy, size_px=90.0, at_max_zoom=True)
        self.assertEqual(decision.reason, "max")
        self.assertTrue(decision.at_max_zoom)

    def test_containment_violation_wins_over_in_band(self):
        policy = self._arm(ContinuousZoomPolicy())
        decision = self._decide(
            policy, size_px=90.0, containment_violation_scale=0.8,
        )
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_OUT)
        self.assertEqual(decision.reason, "frame")

    def test_frame_limited_dip_below_band_keeps_latch(self):
        policy = self._arm(ContinuousZoomPolicy())
        decision = self._decide(
            policy, size_px=84.0, zoom_in_reserve_scale=1.0,
        )
        self.assertEqual(decision.reason, "frame-limited")
        decision = self._decide(policy, size_px=90.0)
        self.assertEqual(decision.reason, "in-band")

    def test_frame_limited_wins_over_in_band(self):
        policy = self._arm(ContinuousZoomPolicy())
        decision = self._decide(
            policy, size_px=90.0, zoom_in_reserve_scale=1.0,
        )
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "frame-limited")

    def test_reset_clears_latch(self):
        policy = self._arm(ContinuousZoomPolicy())
        policy.reset()
        decision = self._decide(policy, size_px=90.0)
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)


class TestContinuousZoomPolicyOutLatch(unittest.TestCase):
    """Latched containment controller + IN-entry reserve rule."""

    def _decide(self, policy, **overrides):
        defaults = dict(
            size_px=50.0,
            target_pixels=100.0,
            containment_violation_scale=1.0,
            zoom_in_reserve_scale=4.0,
            at_min_zoom=False,
            at_max_zoom=False,
            active_direction=None,
            sample_advanced=True,
            optical_fresh=True,
            centered=True,
        )
        defaults.update(overrides)
        policy.decide(**defaults)
        return policy.decide(**defaults)

    def _latch_out(self, policy):
        decision = self._decide(policy, containment_violation_scale=0.8)
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_OUT)
        return policy

    def test_resolved_violation_with_low_reserve_keeps_zooming_out(self):
        policy = self._latch_out(ContinuousZoomPolicy())
        decision = self._decide(policy, zoom_in_reserve_scale=1.05)
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_OUT)
        self.assertEqual(decision.reason, "frame")

    def test_restored_reserve_above_target_exits_to_hold(self):
        policy = self._latch_out(ContinuousZoomPolicy())
        decision = self._decide(
            policy, size_px=110.0, zoom_in_reserve_scale=1.2,
        )
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "above-minimum")

    def test_restored_reserve_below_target_with_full_reserve_exits_to_in(self):
        policy = self._latch_out(ContinuousZoomPolicy())
        decision = self._decide(
            policy, size_px=80.0, zoom_in_reserve_scale=2.0,
        )
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(decision.reason, "minimum")

    def test_restored_reserve_below_target_without_full_reserve_is_frame_limited(self):
        policy = self._latch_out(ContinuousZoomPolicy())
        decision = self._decide(
            policy, size_px=50.0, zoom_in_reserve_scale=1.2,
        )
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "frame-limited")

    def test_exit_clears_latch(self):
        policy = self._latch_out(ContinuousZoomPolicy())
        self._decide(policy, size_px=110.0, zoom_in_reserve_scale=1.2)
        decision = self._decide(
            policy, size_px=110.0, zoom_in_reserve_scale=1.05,
        )
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "above-minimum")

    def test_at_min_zoom_during_latched_out_holds_and_unlatches(self):
        policy = self._latch_out(ContinuousZoomPolicy())
        decision = self._decide(
            policy, zoom_in_reserve_scale=1.05, at_min_zoom=True,
        )
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "frame")
        decision = self._decide(policy, size_px=50.0)
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)

    def test_unknown_geometry_during_latched_out_falls_through(self):
        policy = self._latch_out(ContinuousZoomPolicy())
        decision = self._decide(
            policy,
            size_px=50.0,
            containment_violation_scale=None,
            zoom_in_reserve_scale=None,
        )
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)

    def test_reset_clears_out_latch(self):
        # With the latch set, reserve 1.05 would continue ZOOMING_OUT;
        # after reset the same input is an ordinary frame-limited hold.
        policy = self._latch_out(ContinuousZoomPolicy())
        policy.reset()
        decision = self._decide(policy, zoom_in_reserve_scale=1.05)
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "frame-limited")

    def test_in_entry_blocked_by_band_floor(self):
        # bbox close to target: target/bbox arm is small, 1 + band governs.
        decision = self._decide(
            ContinuousZoomPolicy(), size_px=95.0,
            zoom_in_reserve_scale=1.10,
        )
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "frame-limited")

    def test_in_entry_blocked_by_journey_reserve(self):
        # bbox far below target: target/bbox arm governs even with margin.
        decision = self._decide(
            ContinuousZoomPolicy(), size_px=50.0,
            zoom_in_reserve_scale=1.5,
        )
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "frame-limited")

    def test_in_entry_requires_journey_with_band_slack(self):
        # Reserve covers the bare journey (2.0) but not journey * (1+band)
        # (2.3): entering would land the bbox on the frame margin.
        decision = self._decide(
            ContinuousZoomPolicy(), size_px=50.0,
            zoom_in_reserve_scale=2.1,
        )
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "frame-limited")

    def test_active_in_continues_between_exhaustion_and_entry_threshold(self):
        decision = self._decide(
            ContinuousZoomPolicy(), size_px=50.0,
            zoom_in_reserve_scale=1.5,
            active_direction=ZoomTrackingState.ZOOMING_IN,
        )
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(decision.reason, "minimum")

    def test_active_in_stops_on_reserve_exhaustion(self):
        decision = self._decide(
            ContinuousZoomPolicy(), size_px=50.0,
            zoom_in_reserve_scale=1.0,
            active_direction=ZoomTrackingState.ZOOMING_IN,
        )
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "frame-limited")

    def test_violation_wins_over_active_in_continuation(self):
        decision = self._decide(
            ContinuousZoomPolicy(), size_px=50.0,
            containment_violation_scale=0.8,
            active_direction=ZoomTrackingState.ZOOMING_IN,
        )
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_OUT)
        self.assertEqual(decision.reason, "frame")


class TestContinuousOutLatchIntegration(unittest.TestCase):
    """Tracker-level containment episode: 2 hardware commands, no flip."""

    def setUp(self):
        self.logger = Mock()

    def test_centering_reserve_swings_do_not_pair_chatter(self):
        # SITL 10x acquisition pattern (2026-06-12 115741 run): a small
        # below-target bbox whose containment reserve swings across the
        # entry threshold as the gimbal centers. Entry fires once; the
        # active drive tolerates reserve dips down to exhaustion instead
        # of stopping at the entry threshold.
        mount = _make_mount(mode="continuous")
        # The 120-wide box has characteristic diagonal ~124 px; raise the
        # recognition target above it so the box stays "below-target" (wants
        # zoom-in) under the diagonal measure, preserving the reserve geometry.
        tracker = PoiZoomTracker(
            mount=mount, logger=self.logger,
            config=_cfg(target_pixels={"default": 200.0}),
        )

        centered = dict(cx=960.0, cy=540.0, width=120.0)   # reserve ~14.6
        off_center = dict(cx=200.0, cy=540.0, width=120.0)  # reserve ~1.07
        frames = (centered, off_center, centered, off_center, centered,
                  off_center)
        for index, position in enumerate(frames):
            tracker.update(
                _create_poi(30.0, **position), now=1.0 + index * 0.1,
            )

        # Per-frame swings never persist two samples: nothing commits.
        mount.gimbal.zoom_in.assert_not_called()
        mount.gimbal.zoom_hold.assert_not_called()
        mount.gimbal.zoom_out.assert_not_called()

        # Once the pointing genuinely settles, the entry commits once.
        tracker.update(_create_poi(30.0, **centered), now=1.6)
        tracker.update(_create_poi(30.0, **centered), now=1.7)
        mount.gimbal.zoom_in.assert_called_once()

    def test_containment_episode_costs_two_commands(self):
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        # A persistent violation near the left margin commits the OUT
        # drive on its second sample.
        tracker.update(
            _create_poi(100.0, cx=80.0, cy=540.0, width=120.0), now=1.0,
        )
        tracker.update(
            _create_poi(100.0, cx=80.0, cy=540.0, width=120.0), now=1.1,
        )
        # Violation cleared but reserve still < 1 + band: drive continues.
        tracker.update(
            _create_poi(100.0, cx=150.0, cy=540.0, width=120.0), now=1.2,
        )
        # Centered with reserve restored and bbox above target: the exit
        # commits on its second sample and the episode ends.
        tracker.update(_create_poi(110.0), now=1.3)
        result = tracker.update(_create_poi(110.0), now=1.4)

        self.assertEqual(result.state, ZoomTrackingState.HOLDING)
        self.assertEqual(result.reason, "above-minimum")
        mount.gimbal.zoom_out.assert_called_once()
        mount.gimbal.zoom_hold.assert_called_once()
        mount.gimbal.zoom_in.assert_not_called()


class TestContinuousZoomPolicySampleGating(unittest.TestCase):
    """Stale-sample freezes for non-safety transitions."""

    def _decide(self, policy, **overrides):
        defaults = dict(
            size_px=50.0,
            target_pixels=100.0,
            containment_violation_scale=1.0,
            zoom_in_reserve_scale=4.0,
            at_min_zoom=False,
            at_max_zoom=False,
            active_direction=None,
            sample_advanced=True,
            optical_fresh=True,
            centered=True,
        )
        defaults.update(overrides)
        policy.decide(**defaults)
        return policy.decide(**defaults)

    def test_stale_idle_stays_idle(self):
        decision = self._decide(ContinuousZoomPolicy(), sample_advanced=False)
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "settling")

    def test_stale_active_in_continues_and_does_not_arm_latch(self):
        policy = ContinuousZoomPolicy()
        decision = self._decide(
            policy, size_px=110.0, sample_advanced=False,
            active_direction=ZoomTrackingState.ZOOMING_IN,
        )
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(decision.reason, "settling")
        # The stale above-minimum frame must not have armed the size
        # latch: a fresh below-target frame still re-enters IN.
        decision = self._decide(policy, size_px=90.0)
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)

    def test_fresh_above_minimum_arms_latch_after_stale_frame(self):
        policy = ContinuousZoomPolicy()
        self._decide(
            policy, size_px=110.0, sample_advanced=False,
            active_direction=ZoomTrackingState.ZOOMING_IN,
        )
        decision = self._decide(
            policy, size_px=110.0,
            active_direction=ZoomTrackingState.ZOOMING_IN,
        )
        self.assertEqual(decision.reason, "above-minimum")
        decision = self._decide(policy, size_px=90.0)
        self.assertEqual(decision.reason, "in-band")

    def test_stale_violation_does_not_commit(self):
        # A violation on a stale sample cannot even begin confirmation:
        # the drive continues until the readback advances.
        policy = ContinuousZoomPolicy()
        decision = policy.decide(
            size_px=50.0, target_pixels=100.0,
            containment_violation_scale=0.8, zoom_in_reserve_scale=4.0,
            at_min_zoom=False, at_max_zoom=False,
            active_direction=ZoomTrackingState.ZOOMING_IN,
            sample_advanced=False,
            optical_fresh=True,
            centered=True,
        )
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(decision.reason, "settling")

    def test_persistent_violation_commits_out_on_second_sample(self):
        policy = ContinuousZoomPolicy()
        kwargs = dict(
            size_px=50.0, target_pixels=100.0,
            containment_violation_scale=0.8, zoom_in_reserve_scale=4.0,
            at_min_zoom=False, at_max_zoom=False,
            active_direction=ZoomTrackingState.ZOOMING_IN,
            sample_advanced=True,
            optical_fresh=True,
            centered=True,
        )
        first = policy.decide(**kwargs)
        self.assertEqual(first.reason, "confirming")
        self.assertEqual(first.state, ZoomTrackingState.ZOOMING_IN)
        second = policy.decide(**kwargs)
        self.assertEqual(second.state, ZoomTrackingState.ZOOMING_OUT)
        self.assertEqual(second.reason, "frame")

    def test_stale_out_exit_is_deferred(self):
        policy = ContinuousZoomPolicy()
        self._decide(policy, containment_violation_scale=0.8)
        decision = self._decide(
            policy, size_px=110.0, zoom_in_reserve_scale=2.0,
            sample_advanced=False,
            active_direction=ZoomTrackingState.ZOOMING_OUT,
        )
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_OUT)
        self.assertEqual(decision.reason, "settling")
        decision = self._decide(
            policy, size_px=110.0, zoom_in_reserve_scale=2.0,
            active_direction=ZoomTrackingState.ZOOMING_OUT,
        )
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "above-minimum")

    def test_at_min_escape_commits_with_confirmation(self):
        policy = ContinuousZoomPolicy()
        self._decide(policy, containment_violation_scale=0.8)
        # Stale sample: the escape cannot progress, drive continues.
        decision = policy.decide(
            size_px=50.0, target_pixels=100.0,
            containment_violation_scale=0.8, zoom_in_reserve_scale=4.0,
            at_min_zoom=True, at_max_zoom=False,
            active_direction=ZoomTrackingState.ZOOMING_OUT,
            sample_advanced=False,
            optical_fresh=True,
            centered=True,
        )
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_OUT)
        self.assertEqual(decision.reason, "settling")
        # Two advanced samples at the hardware limit commit the escape.
        decision = self._decide(
            policy, containment_violation_scale=0.8,
            at_min_zoom=True,
            active_direction=ZoomTrackingState.ZOOMING_OUT,
        )
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "frame")


class TestOpticalFreshness(unittest.TestCase):
    """A re-fed cached frame (COASTING) is not an independent optical
    sample: it can neither arm, confirm, nor abandon a pending
    transition. Only fresh, advanced ticks mutate the pending."""

    def _decide(self, policy, **overrides):
        defaults = dict(
            size_px=50.0,
            target_pixels=100.0,
            containment_violation_scale=1.0,
            zoom_in_reserve_scale=4.0,
            at_min_zoom=False,
            at_max_zoom=False,
            active_direction=None,
            sample_advanced=True,
            optical_fresh=True,
            centered=True,
        )
        defaults.update(overrides)
        return policy.decide(**defaults)

    def test_refed_tick_cannot_arm_pending(self):
        policy = ContinuousZoomPolicy()
        decision = self._decide(policy, optical_fresh=False)
        self.assertEqual(decision.reason, "settling")
        # Arming starts on the FIRST fresh tick, not on the re-fed one:
        # a commit still needs two fresh samples after the gap.
        decision = self._decide(policy)
        self.assertEqual(decision.reason, "confirming")
        decision = self._decide(policy)
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)

    def test_refed_tick_cannot_confirm_pending(self):
        policy = ContinuousZoomPolicy()
        decision = self._decide(policy)
        self.assertEqual(decision.reason, "confirming")
        # The same optical frame re-fed during a COASTING stutter must
        # not act as the confirming second sample.
        decision = self._decide(policy, optical_fresh=False)
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "settling")
        # The next fresh sample repeating the condition commits: pending
        # survived the gap and both samples are real optical frames.
        decision = self._decide(policy)
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)

    def test_refed_steady_tick_preserves_pending(self):
        policy = ContinuousZoomPolicy()
        decision = self._decide(policy)
        self.assertEqual(decision.reason, "confirming")
        # COASTING flips the centering input, so the re-fed raw decision
        # reads steady-state ("centering" hold). That reading is gap
        # noise and must not abandon the pending armed on real optics.
        decision = self._decide(policy, optical_fresh=False, centered=False)
        self.assertEqual(decision.state, ZoomTrackingState.HOLDING)
        self.assertEqual(decision.reason, "centering")
        decision = self._decide(policy)
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)

    def test_fresh_steady_tick_abandons_pending(self):
        policy = ContinuousZoomPolicy()
        decision = self._decide(policy)
        self.assertEqual(decision.reason, "confirming")
        # A FRESH steady-state reading is proof the condition did not
        # persist: the pending is abandoned and arming must restart.
        decision = self._decide(policy, size_px=160.0)
        self.assertEqual(decision.reason, "above-minimum")
        decision = self._decide(policy)
        self.assertEqual(decision.reason, "confirming")


class TestSampleGatingIntegration(unittest.TestCase):
    """Tracker-level gating against the mount sample id."""

    def setUp(self):
        self.logger = Mock()

    def test_same_sample_jitter_produces_no_alternating_commands(self):
        mount = _make_mount(mode="continuous")
        mount.get_fresh_zoom_sample_id.return_value = "1"
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        for index, size_px in enumerate((50.0, 160.0, 90.0, 160.0, 90.0)):
            tracker.update(_create_poi(size_px), now=1.0 + index * 0.1)

        # Per-frame alternation never persists two samples: no commands.
        mount.gimbal.zoom_in.assert_not_called()
        mount.gimbal.zoom_hold.assert_not_called()
        mount.gimbal.zoom_out.assert_not_called()

    def test_sample_progression_reenables_transitions(self):
        mount = _make_mount(mode="continuous")
        mount.get_fresh_zoom_sample_id.side_effect = ["1", "2", "3", "4", "5"]
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        tracker.update(_create_poi(50.0), now=1.1)   # commit IN
        result = tracker.update(_create_poi(160.0), now=1.2)
        self.assertEqual(result.reason, "confirming")
        result = tracker.update(_create_poi(160.0), now=1.3)  # commit HOLD
        self.assertEqual(result.reason, "above-minimum")
        result = tracker.update(_create_poi(90.0), now=1.4)
        self.assertEqual(result.reason, "in-band")

        mount.gimbal.zoom_in.assert_called_once()
        mount.gimbal.zoom_hold.assert_called_once()

    def test_poi_loss_reacquire_on_same_sample_defers_one_tick(self):
        mount = _make_mount(mode="continuous")
        mount.get_fresh_zoom_sample_id.return_value = "7"
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        tracker.update(_create_poi(50.0), now=1.1)  # commit IN
        tracker.update(None, now=1.2)  # active drive: zoom_hold fires
        result = tracker.update(_create_poi(60.0), now=1.3)
        self.assertEqual(result.reason, "settling")
        mount.gimbal.zoom_in.assert_called_once()

        mount.get_fresh_zoom_sample_id.return_value = "8"
        result = tracker.update(_create_poi(60.0), now=1.4)
        self.assertEqual(result.reason, "confirming")
        result = tracker.update(_create_poi(60.0), now=1.5)
        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(mount.gimbal.zoom_in.call_count, 2)

    def test_poi_loss_hold_records_sample_current_at_hold_time(self):
        # The readback advances between the zoom_in command and the
        # POI-loss hold: the hold must gate against the NEW sample,
        # so reacquiring on that same sample defers.
        mount = _make_mount(mode="continuous")
        mount.get_fresh_zoom_sample_id.return_value = "7"
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        tracker.update(_create_poi(50.0), now=1.1)  # commit IN
        mount.get_fresh_zoom_sample_id.return_value = "8"
        tracker.update(None, now=1.2)  # hold fires, records "8"
        result = tracker.update(_create_poi(60.0), now=1.3)
        self.assertEqual(result.reason, "settling")
        mount.gimbal.zoom_in.assert_called_once()

        mount.get_fresh_zoom_sample_id.return_value = "9"
        result = tracker.update(_create_poi(60.0), now=1.4)
        self.assertEqual(result.reason, "confirming")
        result = tracker.update(_create_poi(60.0), now=1.5)
        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(mount.gimbal.zoom_in.call_count, 2)

    def test_idle_poi_loss_clears_gating_state(self):
        mount = _make_mount(mode="continuous")
        mount.get_fresh_zoom_sample_id.return_value = "7"
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        tracker.update(_create_poi(50.0), now=1.1)   # commit IN, records "7"
        tracker.update(_create_poi(160.0), now=1.2)  # settling (same sample)
        mount.get_fresh_zoom_sample_id.return_value = "8"
        tracker.update(_create_poi(160.0), now=1.3)  # confirming
        tracker.update(_create_poi(160.0), now=1.4)  # commit HOLD, records "8"
        tracker.update(None, now=1.5)  # idle loss: no hold, clears gating
        tracker.update(_create_poi(50.0), now=1.6)   # confirming (gate open)
        result = tracker.update(_create_poi(50.0), now=1.7)

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(mount.gimbal.zoom_in.call_count, 2)

    def test_mount_without_sample_telemetry_disables_gating(self):
        mount = _make_mount(mode="continuous")  # sample id defaults to None
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        tracker.update(_create_poi(50.0), now=1.1)   # commit IN
        tracker.update(_create_poi(160.0), now=1.2)  # confirming
        result = tracker.update(_create_poi(160.0), now=1.3)

        self.assertEqual(result.reason, "above-minimum")
        mount.gimbal.zoom_hold.assert_called_once()

    def test_stale_telemetry_after_seen_freezes_transitions(self):
        # Once the mount has proven sample telemetry, a stale readback
        # (None) freezes transitions instead of disabling the gate: the
        # gate fails closed, not open.
        mount = _make_mount(mode="continuous")
        mount.get_fresh_zoom_sample_id.side_effect = ["1", None, "2"]
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)   # confirming
        result = tracker.update(_create_poi(50.0), now=1.1)  # stale
        self.assertEqual(result.reason, "settling")
        mount.gimbal.zoom_in.assert_not_called()

        # Telemetry recovers: the pending survived the stale tick and
        # the fresh sample confirms it.
        result = tracker.update(_create_poi(50.0), now=1.2)
        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        mount.gimbal.zoom_in.assert_called_once()

    def test_sample_id_seen_by_lifecycle_hold_latches_telemetry(self):
        # The first valid sample id may be observed by a POI-loss
        # hold rather than a decision tick. It must still latch the
        # telemetry capability, so later None readbacks freeze instead
        # of taking the never-seen fail-open path.
        mount = _make_mount(mode="continuous")
        mount.get_fresh_zoom_sample_id.side_effect = [None, None, "8", None, None]
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)   # confirming
        tracker.update(_create_poi(50.0), now=1.1)   # commit IN
        self.assertEqual(mount.gimbal.zoom_in.call_count, 1)
        tracker.update(None, now=1.2)                   # loss: hold sees "8"

        result = tracker.update(_create_poi(50.0), now=1.3)  # stale None
        self.assertEqual(result.reason, "settling")
        result = tracker.update(_create_poi(50.0), now=1.4)  # stale None
        self.assertEqual(result.reason, "settling")
        self.assertEqual(mount.gimbal.zoom_in.call_count, 1)


class TestSizeDemandIntegration(unittest.TestCase):
    """Tracker-level lifecycle of the mission-policy demand switch.

    With the demand off the tracker applies the track-retention floor
    (tracking_pixels, default MIN_TRACK_PIXELS) instead of the per-class
    recognition size — keep the track alive, not chase recognition pixels.
    """

    def setUp(self):
        self.logger = Mock()

    def test_demand_off_quiesces_active_drive_once(self):
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        tracker.update(_create_poi(50.0), now=1.1)  # commit IN
        tracker.set_size_demand(False)
        result = tracker.update(_create_poi(60.0), now=1.2)
        self.assertEqual(result.reason, "confirming")
        result = tracker.update(_create_poi(60.0), now=1.3)  # commit HOLD
        self.assertEqual(result.reason, "above-minimum")
        self.assertEqual(result.target_pixels, MIN_TRACK_PIXELS)
        tracker.update(_create_poi(70.0), now=1.4)

        mount.gimbal.zoom_in.assert_called_once()
        mount.gimbal.zoom_hold.assert_called_once()

    def test_demand_off_violation_still_zooms_out(self):
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())
        tracker.set_size_demand(False)

        tracker.update(
            _create_poi(100.0, cx=80.0, cy=540.0, width=120.0), now=1.0,
        )
        result = tracker.update(
            _create_poi(100.0, cx=80.0, cy=540.0, width=120.0), now=1.1,
        )

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_OUT)
        mount.gimbal.zoom_out.assert_called_once()

    def test_demand_off_small_bbox_zooms_in_to_tracking_floor(self):
        # The field regression (run 165504): bbox at 9 px with the demand
        # off must zoom IN toward the tracking floor, not sit at min zoom
        # until detection drops below the 8 px floor and the track dies.
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())
        tracker.set_size_demand(False)

        tracker.update(_create_poi(9.0), now=1.0)
        result = tracker.update(_create_poi(9.0), now=1.1)

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(result.target_pixels, MIN_TRACK_PIXELS)
        mount.gimbal.zoom_in.assert_called_once()

    def test_range_growth_refires_zoom_in_before_detection_floor(self):
        # Outbound leg: bbox shrinks as range grows. The tracker must act
        # while the POI is still comfortably detectable.
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())
        tracker.set_size_demand(False)

        # Square boxes so the characteristic diagonal equals the loop value;
        # it shrinks 16 -> 10 px across the tracking floor (MIN_TRACK_PIXELS=12),
        # staying above the 8 px detection gate.
        for index, size_px in enumerate((16.0, 14.0, 11.0, 10.0)):
            tracker.update(
                _create_poi(size_px), now=1.0 + index * 0.1,
            )

        mount.gimbal.zoom_in.assert_called_once()
        mount.gimbal.zoom_out.assert_not_called()

    def test_reset_keeps_tracking_demand(self):
        # Mission phase belongs to the caller: the loss-recovery path
        # resets the zoom session mid-NAV and must NOT flip the demand
        # back to recognition size.
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())
        tracker.set_size_demand(False)
        tracker.reset()

        result = tracker.update(_create_poi(50.0), now=1.0)

        self.assertEqual(result.state, ZoomTrackingState.HOLDING)
        self.assertEqual(result.reason, "above-minimum")
        self.assertEqual(result.target_pixels, MIN_TRACK_PIXELS)
        mount.gimbal.zoom_in.assert_not_called()

    def test_loss_recovery_keeps_tracking_demand(self):
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())
        tracker.set_size_demand(False)

        tracker.update(_create_poi(9.0), now=1.0)
        tracker.update(_create_poi(9.0), now=1.1)  # commit IN
        tracker.update(None, now=1.2)  # loss: session reset
        tracker.update(_create_poi(9.0), now=1.3)  # confirming
        result = tracker.update(_create_poi(9.0), now=1.4)

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(result.target_pixels, MIN_TRACK_PIXELS)

    def test_nav_dive_replay_produces_no_zoom_in(self):
        # 1x NAV dive pattern (run 151042): bbox repeatedly grows past
        # the margin, the OUT episode drops it below target, and with the
        # demand off nothing zooms back in between episodes.
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())
        tracker.set_size_demand(False)

        edge = dict(cx=170.0, cy=540.0, width=200.0)
        frames = (
            _create_poi(230.0, **edge),     # violation: confirming
            _create_poi(120.0, **edge),     # violation persists: OUT commits
            _create_poi(60.0),              # recovered: exit confirming
            _create_poi(70.0),              # exit persists: HOLD commits
            _create_poi(240.0, **edge),     # ONE-frame violation: suppressed
            _create_poi(50.0),              # back to steady hold
            _create_poi(60.0),              # stays quiet
        )
        for index, poi in enumerate(frames):
            tracker.update(poi, now=1.0 + index * 0.1)

        mount.gimbal.zoom_in.assert_not_called()
        self.assertEqual(mount.gimbal.zoom_out.call_count, 1)
        self.assertEqual(mount.gimbal.zoom_hold.call_count, 1)


class TestContinuousHysteresisIntegration(unittest.TestCase):
    """Tracker-level behavior of the size-side hysteresis."""

    def setUp(self):
        self.logger = Mock()

    def test_jitter_around_target_does_not_chatter(self):
        # Replay of the centered-bbox jitter pattern from the
        # 2026-06-12 chatter log, scaled to target_pixels=100.
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        heights = (50.0, 110.0, 98.0, 94.0, 88.0, 109.0, 127.0,
                   79.0, 119.0, 135.0, 93.0)
        for index, size_px in enumerate(heights):
            tracker.update(_create_poi(size_px), now=1.0 + index * 0.1)

        # 98/94 confirm one IN; 109/127 confirm one HOLD (latch arms);
        # the one-frame dip to 79 never persists: no further commands.
        self.assertEqual(mount.gimbal.zoom_in.call_count, 1)
        self.assertEqual(mount.gimbal.zoom_hold.call_count, 1)
        mount.gimbal.zoom_out.assert_not_called()

    def test_in_band_hold_is_not_stable(self):
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        tracker.update(_create_poi(50.0), now=1.1)   # commit IN
        tracker.update(_create_poi(110.0), now=1.2)  # confirming
        tracker.update(_create_poi(110.0), now=1.3)  # commit HOLD + latch
        result = tracker.update(_create_poi(90.0), now=1.4)

        self.assertEqual(result.state, ZoomTrackingState.HOLDING)
        self.assertEqual(result.reason, "in-band")
        self.assertFalse(tracker.is_zoom_stable)

    def test_poi_loss_clears_hysteresis_latch(self):
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        tracker.update(_create_poi(50.0), now=1.1)
        tracker.update(_create_poi(110.0), now=1.2)
        tracker.update(_create_poi(110.0), now=1.3)
        tracker.update(None, now=1.4)
        tracker.update(_create_poi(90.0), now=1.5)  # confirming
        result = tracker.update(_create_poi(90.0), now=1.6)

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)

    def test_reset_clears_hysteresis_latch(self):
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        tracker.update(_create_poi(50.0), now=1.1)
        tracker.update(_create_poi(110.0), now=1.2)
        tracker.update(_create_poi(110.0), now=1.3)
        tracker.reset()
        tracker.update(_create_poi(90.0), now=1.4)  # confirming
        result = tracker.update(_create_poi(90.0), now=1.5)

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)


class TestTransitionConfirmation(unittest.TestCase):
    """No drive transition commits on a single optical sample."""

    def setUp(self):
        self.logger = Mock()

    def _decide_raw(self, policy, **overrides):
        defaults = dict(
            size_px=50.0,
            target_pixels=100.0,
            containment_violation_scale=1.0,
            zoom_in_reserve_scale=4.0,
            at_min_zoom=False,
            at_max_zoom=False,
            active_direction=None,
            sample_advanced=True,
            optical_fresh=True,
            centered=True,
        )
        defaults.update(overrides)
        return policy.decide(**defaults)

    def test_field_triplet_replay_produces_no_commands(self):
        # 2026-06-12 run 184615: the 10x gimbal swings the POI
        # edge<->center between consecutive frames, producing
        # OUT(x)/IN(x)/HOLD(x) triplets at the same bbox. Single-frame
        # geometry crossings must never command the lens.
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        edge = dict(cx=120.0, cy=540.0, width=120.0)      # margin violated
        centered = dict(cx=960.0, cy=540.0, width=120.0)  # full reserve
        for index in range(8):
            position = edge if index % 2 == 0 else centered
            tracker.update(
                _create_poi(92.0, **position), now=1.0 + index * 0.1,
            )

        mount.gimbal.zoom_in.assert_not_called()
        mount.gimbal.zoom_out.assert_not_called()
        mount.gimbal.zoom_hold.assert_not_called()

    def test_spike_during_active_in_does_not_stop_or_arm_latch(self):
        policy = ContinuousZoomPolicy()
        # Commit an IN drive.
        self._decide_raw(policy, size_px=50.0)
        self._decide_raw(policy, size_px=50.0)
        # One above-minimum spike: pending only, drive continues.
        decision = self._decide_raw(
            policy, size_px=110.0,
            active_direction=ZoomTrackingState.ZOOMING_IN,
        )
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(decision.reason, "confirming")
        # Back below target: the latch was never armed, so a dip below
        # the band edge is NOT in-band suppressed -- the drive continues.
        decision = self._decide_raw(
            policy, size_px=90.0,
            active_direction=ZoomTrackingState.ZOOMING_IN,
        )
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(decision.reason, "minimum")

    def test_same_state_different_reason_restarts_confirmation(self):
        policy = ContinuousZoomPolicy()
        # Pending HOLD above-minimum...
        decision = self._decide_raw(
            policy, size_px=110.0,
            active_direction=ZoomTrackingState.ZOOMING_IN,
        )
        self.assertEqual(decision.reason, "confirming")
        # ...followed by HOLD frame-limited: must NOT commit above-minimum.
        decision = self._decide_raw(
            policy, size_px=50.0, zoom_in_reserve_scale=0.9,
            active_direction=ZoomTrackingState.ZOOMING_IN,
        )
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(decision.reason, "confirming")

    def test_stale_tick_preserves_pending_confirmation(self):
        policy = ContinuousZoomPolicy()
        decision = self._decide_raw(policy, size_px=50.0)
        self.assertEqual(decision.reason, "confirming")
        # Stale tick: neither consumes nor clears the pending transition.
        decision = self._decide_raw(
            policy, size_px=50.0, sample_advanced=False,
        )
        self.assertEqual(decision.reason, "settling")
        # Next advanced tick with the same raw decision commits.
        decision = self._decide_raw(policy, size_px=50.0)
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)

    def test_reset_clears_pending_confirmation(self):
        policy = ContinuousZoomPolicy()
        self._decide_raw(policy, size_px=50.0)
        policy.reset()
        decision = self._decide_raw(policy, size_px=50.0)
        self.assertEqual(decision.reason, "confirming")

    def test_confirming_hold_is_not_stable(self):
        # A pending transition (e.g. a suspected violation) must block
        # confirmation snapshots even when the bbox meets the target.
        mount = _make_mount(mode="continuous")
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        result = tracker.update(
            _create_poi(160.0, cx=120.0, cy=540.0, width=120.0), now=1.0,
        )

        self.assertEqual(result.reason, "confirming")
        self.assertFalse(tracker.is_zoom_stable)


class TestCenteringGate(unittest.TestCase):
    """Zoom-in entry requires the gimbal pointing to be settled:
    TRACKING + mature estimate + lead-projected error inside the zoom
    cone + not diverging. 'Center first, then magnify.'"""

    def setUp(self):
        self.logger = Mock()

    @staticmethod
    def _pointing(state=TrackingState.TRACKING, mature=True,
                  yaw=0.0, pitch=0.0, yaw_rate=0.0, pitch_rate=0.0):
        return GimbalTrackResult(
            state=state, has_poi=state == TrackingState.TRACKING,
            yaw_error=yaw, pitch_error=pitch,
            yaw_rate_estimate=yaw_rate, pitch_rate_estimate=pitch_rate,
            mature=mature,
        )

    def _tracker(self, with_k=False):
        mount = _make_mount(mode="continuous")
        if with_k:
            import numpy as np
            mount.get_k.return_value = np.array(
                [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]],
            )
        return mount, PoiZoomTracker(
            mount=mount, logger=self.logger, config=_cfg(),
        )

    def test_no_pointing_disables_gate(self):
        mount, tracker = self._tracker()
        tracker.update(_create_poi(50.0), now=1.0)
        tracker.update(_create_poi(50.0), now=1.1)
        mount.gimbal.zoom_in.assert_called_once()

    def test_coasting_blocks_zoom_in_entry(self):
        mount, tracker = self._tracker()
        pointing = self._pointing(state=TrackingState.COASTING)
        tracker.update(_create_poi(50.0), now=1.0, pointing=pointing)
        result = tracker.update(_create_poi(50.0), now=1.1, pointing=pointing)

        self.assertEqual(result.reason, "centering")
        mount.gimbal.zoom_in.assert_not_called()

    def test_immature_tracking_blocks_zoom_in_entry(self):
        # The obj_2 grab: zoom fired 57 ms after idle->TRACKING, before
        # the estimator had any rate information.
        mount, tracker = self._tracker()
        pointing = self._pointing(mature=False)
        tracker.update(_create_poi(50.0), now=1.0, pointing=pointing)
        result = tracker.update(_create_poi(50.0), now=1.1, pointing=pointing)

        self.assertEqual(result.reason, "centering")
        mount.gimbal.zoom_in.assert_not_called()

    def test_rate_sign_does_not_double_gate_mature_projected_error(self):
        # The rate tracker already publishes the mature lead-projected error.
        # A second error*rate divergence gate would count the same motion twice.
        mount, tracker = self._tracker()
        pointing = self._pointing(yaw=0.0716, yaw_rate=0.17)
        tracker.update(_create_poi(74.0), now=1.0, pointing=pointing)
        result = tracker.update(_create_poi(74.0), now=1.1, pointing=pointing)

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        mount.gimbal.zoom_in.assert_called_once()

    def test_out_of_cone_pointing_blocks_zoom_in_entry(self):
        # fy=1000, usable half-extent 453.6 px -> half-angle 0.426 rad;
        # bbox 50 / target 100 -> cone 0.213 rad. Error 0.3 rad is
        # outside even though it is converging.
        mount, tracker = self._tracker(with_k=True)
        pointing = self._pointing(yaw=0.30, yaw_rate=-0.05)
        tracker.update(_create_poi(50.0), now=1.0, pointing=pointing)
        result = tracker.update(_create_poi(50.0), now=1.1, pointing=pointing)

        self.assertEqual(result.reason, "centering")
        mount.gimbal.zoom_in.assert_not_called()

    def test_centered_converging_acquisition_commits_zoom_in(self):
        # The good 10x entry: bbox 9 px (R~11 -> cone ~0.038 rad), error
        # ~0.7 deg (0.012 rad) and converging.
        mount, tracker = self._tracker(with_k=True)
        pointing = self._pointing(yaw=0.012, yaw_rate=-0.01, pitch=0.005)
        tracker.update(_create_poi(9.0, width=14.0), now=1.0,
                       pointing=pointing)
        result = tracker.update(_create_poi(9.0, width=14.0), now=1.1,
                                pointing=pointing)

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        mount.gimbal.zoom_in.assert_called_once()

    def test_subdeadband_jitter_does_not_block_zoom_in(self):
        # The dominant zoom blocker in SITL 093101 (uav_2/3 stuck below
        # recognition size, reserve ~6.3): the gimbal is settled but the
        # rate estimate carries sub-degree noise whose sign matches the
        # tiny residual error. Undamped, error*rate>0 flips `centered`
        # every frame so the two-sample confirm never commits a drive.
        # Here the 9x14 box gives cone ~0.071 rad, deadband 0.8*cone ~0.057;
        # an error of 0.012 rad (~0.7 deg, well inside the deadband) with
        # a matching-sign (outward) rate must NOT count as divergence.
        mount, tracker = self._tracker(with_k=True)
        pointing = self._pointing(yaw=0.012, yaw_rate=+0.01, pitch=0.004)
        tracker.update(_create_poi(9.0, width=14.0), now=1.0,
                       pointing=pointing)
        result = tracker.update(_create_poi(9.0, width=14.0), now=1.1,
                                pointing=pointing)

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        mount.gimbal.zoom_in.assert_called_once()

    def test_finite_projected_error_inside_cone_opens_entry(self):
        # Containment is the sole entry rule: this mature projected error is
        # inside the cone, regardless of the separate rate-estimate sign.
        mount, tracker = self._tracker(with_k=True)
        pointing = self._pointing(yaw=0.065, yaw_rate=+0.01)
        tracker.update(_create_poi(9.0, width=14.0), now=1.0,
                       pointing=pointing)
        result = tracker.update(_create_poi(9.0, width=14.0), now=1.1,
                                pointing=pointing)

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        mount.gimbal.zoom_in.assert_called_once()

    def test_projected_error_does_not_require_a_second_rate_signal(self):
        # A mature projected error is already the controller input. Requiring
        # a parallel rate field would add an unrelated readiness gate.
        mount, tracker = self._tracker()
        pointing = GimbalTrackResult(
            state=TrackingState.TRACKING, has_poi=True,
            yaw_error=0.01, pitch_error=0.0,
            yaw_rate_estimate=None, pitch_rate_estimate=None,
            mature=True,
        )
        tracker.update(_create_poi(50.0), now=1.0, pointing=pointing)
        result = tracker.update(_create_poi(50.0), now=1.1, pointing=pointing)

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        mount.gimbal.zoom_in.assert_called_once()

    def test_centering_flip_restarts_confirmation(self):
        mount, tracker = self._tracker()
        centered = self._pointing()
        uncentered = self._pointing(mature=False)

        tracker.update(_create_poi(50.0), now=1.0, pointing=centered)
        tracker.update(_create_poi(50.0), now=1.1, pointing=uncentered)
        tracker.update(_create_poi(50.0), now=1.2, pointing=centered)
        mount.gimbal.zoom_in.assert_not_called()
        tracker.update(_create_poi(50.0), now=1.3, pointing=centered)
        mount.gimbal.zoom_in.assert_called_once()

    def test_uncentered_does_not_stop_active_drive(self):
        # Entry-only gate: de-settling mid-drive must not add a new
        # flicker-sensitive stop condition.
        policy = ContinuousZoomPolicy()
        kwargs = dict(
            size_px=50.0, target_pixels=100.0,
            containment_violation_scale=1.0, zoom_in_reserve_scale=4.0,
            at_min_zoom=False, at_max_zoom=False,
            active_direction=ZoomTrackingState.ZOOMING_IN,
            sample_advanced=True, optical_fresh=True, centered=False,
        )
        decision = policy.decide(**kwargs)
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(decision.reason, "minimum")

    def test_hold_on_decenter_pauses_active_zoom_in(self):
        # The size-demand confirm gate: with hold_on_decenter=True, an active
        # zoom-in that de-centers is PAUSED (HOLD "centering") so the gimbal
        # re-centers between steps and the narrowing FOV never crops an
        # off-center POI out of frame (uav_1 110558). Contrast
        # test_uncentered_does_not_stop_active_drive (entry-only, the default).
        policy = ContinuousZoomPolicy()
        kwargs = dict(
            size_px=50.0, target_pixels=100.0,
            containment_violation_scale=1.0, zoom_in_reserve_scale=4.0,
            at_min_zoom=False, at_max_zoom=False,
            active_direction=ZoomTrackingState.ZOOMING_IN,
            sample_advanced=True, optical_fresh=True, centered=False,
            hold_on_decenter=True,
        )
        # First sample arms the transition (drive continues); a second
        # independent sample commits the centering HOLD.
        first = policy.decide(**kwargs)
        self.assertEqual(first.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(first.reason, "confirming")
        second = policy.decide(**kwargs)
        self.assertEqual(second.state, ZoomTrackingState.HOLDING)
        self.assertEqual(second.reason, "centering")

    def test_hold_on_decenter_keeps_active_zoom_in_when_centered(self):
        # hold_on_decenter only bites when off-center: a centered active drive
        # zooms on, so the stepped zoom resumes once the gimbal re-centers.
        policy = ContinuousZoomPolicy()
        decision = policy.decide(
            size_px=50.0, target_pixels=100.0,
            containment_violation_scale=1.0, zoom_in_reserve_scale=4.0,
            at_min_zoom=False, at_max_zoom=False,
            active_direction=ZoomTrackingState.ZOOMING_IN,
            sample_advanced=True, optical_fresh=True, centered=True,
            hold_on_decenter=True,
        )
        self.assertEqual(decision.state, ZoomTrackingState.ZOOMING_IN)
        self.assertEqual(decision.reason, "minimum")

    def test_size_demand_confirm_pauses_zoom_on_decenter(self):
        # Tracker wiring: the confirm phase (size_demand True, the default)
        # passes hold_on_decenter, so a mid-zoom de-center pauses the stepped
        # drive. Start an active zoom-in (centered, below the 100px demand),
        # then de-center: the drive must HOLD "centering" and stop the lens.
        mount, tracker = self._tracker()
        centered = self._pointing()
        tracker.update(_create_poi(50.0), now=1.0, pointing=centered)
        tracker.update(_create_poi(50.0), now=1.1, pointing=centered)
        mount.gimbal.zoom_in.assert_called_once()

        uncentered = self._pointing(mature=False)
        tracker.update(_create_poi(50.0), now=1.2, pointing=uncentered)
        result = tracker.update(_create_poi(50.0), now=1.3, pointing=uncentered)
        self.assertEqual(result.state, ZoomTrackingState.HOLDING)
        self.assertEqual(result.reason, "centering")
        mount.gimbal.zoom_hold.assert_called_once()

    def test_containment_out_wins_over_centering(self):
        mount, tracker = self._tracker()
        pointing = self._pointing(mature=False)
        edge = dict(cx=80.0, cy=540.0, width=120.0)
        tracker.update(_create_poi(100.0, **edge), now=1.0,
                       pointing=pointing)
        result = tracker.update(_create_poi(100.0, **edge), now=1.1,
                                pointing=pointing)

        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_OUT)
        mount.gimbal.zoom_out.assert_called_once()


class TestCoastingRefeedIntegration(unittest.TestCase):
    """During a COASTING stutter the navigation layer re-feeds the cached
    last frame so the zoom session is not flushed. That cached frame is
    not an independent optical sample: it must not confirm a pending
    transition (one real frame + one re-feed is still ONE sample)."""

    def setUp(self):
        self.logger = Mock()

    @staticmethod
    def _pointing(state=TrackingState.TRACKING):
        return GimbalTrackResult(
            state=state, has_poi=state == TrackingState.TRACKING,
            yaw_error=0.0, pitch_error=0.0,
            yaw_rate_estimate=0.0, pitch_rate_estimate=0.0,
            mature=True,
        )

    def _tracker(self):
        mount = _make_mount(mode="continuous")
        return mount, PoiZoomTracker(
            mount=mount, logger=self.logger, config=_cfg(),
        )

    def test_refed_frame_cannot_confirm_containment_out(self):
        # The final-review blocking scenario: a one-frame margin
        # violation followed by a one-frame detector dropout. The
        # re-fed frame repeats the violation reading but is the SAME
        # optical sample — it must not commit the OUT.
        mount, tracker = self._tracker()
        edge = dict(cx=80.0, cy=540.0, width=120.0)
        frame = _create_poi(100.0, **edge)

        tracker.update(frame, now=1.0, pointing=self._pointing())
        result = tracker.update(
            frame, now=1.1, pointing=self._pointing(TrackingState.COASTING),
        )
        self.assertEqual(result.reason, "settling")
        mount.gimbal.zoom_out.assert_not_called()
        # The preserved pending must surface on the result: the OUT can
        # still commit on the next fresh frame, so confirmation capture
        # must not treat this HOLDING tick as stable (bbox >= target
        # would otherwise read stable here).
        self.assertTrue(result.transition_pending)
        self.assertFalse(result.is_stable)

        # A second real frame repeating the violation commits.
        result = tracker.update(
            _create_poi(100.0, **edge), now=1.2,
            pointing=self._pointing(),
        )
        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_OUT)
        mount.gimbal.zoom_out.assert_called_once()

    def test_zoom_in_pending_survives_coasting_gap(self):
        # The gap's "centering" steady reading is gap noise: it must
        # not abandon a pending armed on real optics, and the first
        # fresh frame after the gap may commit (two real samples).
        mount, tracker = self._tracker()

        tracker.update(_create_poi(50.0), now=1.0,
                       pointing=self._pointing())
        result = tracker.update(
            _create_poi(50.0), now=1.1,
            pointing=self._pointing(TrackingState.COASTING),
        )
        self.assertEqual(result.reason, "centering")
        mount.gimbal.zoom_in.assert_not_called()

        result = tracker.update(_create_poi(50.0), now=1.2,
                                pointing=self._pointing())
        self.assertEqual(result.state, ZoomTrackingState.ZOOMING_IN)
        mount.gimbal.zoom_in.assert_called_once()


class TestZoomLevelLogging(unittest.TestCase):
    """Continuous IN / OUT / HOLD log lines include the lens zoom level."""

    def setUp(self):
        self.logger = Mock()

    def _log_lines(self):
        return [
            str(call.args[0]) for call in self.logger.info.call_args_list
            if call.args and "Zoom " in str(call.args[0])
        ]

    def test_in_and_hold_lines_include_zoom_level(self):
        mount = _make_mount(mode="continuous")
        mount.get_fresh_zoom_command.return_value = "2.5"
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        tracker.update(_create_poi(50.0), now=1.1)   # commit IN
        tracker.update(_create_poi(200.0), now=1.2)  # confirming
        tracker.update(_create_poi(200.0), now=1.3)  # commit HOLD

        lines = self._log_lines()
        self.assertTrue(any("Zoom IN" in ln and "zoom=2.5" in ln for ln in lines))
        self.assertTrue(any("Zoom HOLD" in ln and "zoom=2.5" in ln for ln in lines))

    def test_out_line_includes_zoom_level(self):
        mount = _make_mount(mode="continuous")
        mount.get_fresh_zoom_command.return_value = "4.0"
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        edge = dict(cx=80.0, cy=540.0, width=120.0)
        tracker.update(_create_poi(100.0, **edge), now=1.0)
        tracker.update(_create_poi(100.0, **edge), now=1.1)  # commit OUT

        lines = self._log_lines()
        self.assertTrue(any("Zoom OUT" in ln and "zoom=4.0" in ln for ln in lines))

    def test_missing_readback_logs_question_mark(self):
        mount = _make_mount(mode="continuous")  # fresh zoom command -> None
        tracker = PoiZoomTracker(mount=mount, logger=self.logger, config=_cfg())

        tracker.update(_create_poi(50.0), now=1.0)
        tracker.update(_create_poi(50.0), now=1.1)  # commit IN

        lines = self._log_lines()
        self.assertTrue(any("Zoom IN" in ln and "zoom=?" in ln for ln in lines))


class TestGetPoiPixelsForClass(unittest.TestCase):
    def _make_tracker(self, target_pixels):
        mount = _make_mount(
            levels=("1", "2"), current_zoom="1", mode="none",
        )
        config = PoiZoomTrackerConfig(target_pixels=target_pixels)
        return PoiZoomTracker(mount=mount, logger=Mock(), config=config)

    def test_returns_class_specific_value(self):
        tracker = self._make_tracker({"0": 15.0, "4": 18.0, "default": 20.0})
        self.assertAlmostEqual(tracker.get_target_pixels_for_class(0), 15.0)
        self.assertAlmostEqual(tracker.get_target_pixels_for_class(4), 18.0)

    def test_falls_back_to_default_for_unknown_class(self):
        tracker = self._make_tracker({"0": 15.0, "default": 20.0})
        self.assertAlmostEqual(tracker.get_target_pixels_for_class(99), 20.0)

    def test_none_class_returns_default(self):
        tracker = self._make_tracker({"0": 15.0, "default": 20.0})
        self.assertAlmostEqual(tracker.get_target_pixels_for_class(None), 20.0)


class TestExtractBboxNonFinite(unittest.TestCase):
    """_extract_bbox_cxcywh must reject non-finite bbox components so a NaN/inf
    dimension is treated as "no usable bbox" rather than propagating into
    poi_size.characteristic_pixels (which raises on non-finite)."""

    @staticmethod
    def _poi(bbox):
        poi = Mock()
        poi.tracking_bbox_cxcywh = bbox
        poi.bbox_cxcywh = None
        return poi

    def test_non_finite_dims_rejected(self):
        for bad in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(bad=bad):
                self.assertIsNone(PoiZoomTracker._extract_bbox_cxcywh(
                    self._poi((960.0, 540.0, bad, 50.0))))
                self.assertIsNone(PoiZoomTracker._extract_bbox_cxcywh(
                    self._poi((960.0, 540.0, 50.0, bad))))

    def test_non_finite_center_rejected(self):
        self.assertIsNone(PoiZoomTracker._extract_bbox_cxcywh(
            self._poi((float("nan"), 540.0, 50.0, 50.0))))

    def test_finite_bbox_accepted(self):
        self.assertEqual(
            PoiZoomTracker._extract_bbox_cxcywh(
                self._poi((960.0, 540.0, 40.0, 30.0))),
            (960.0, 540.0, 40.0, 30.0),
        )


if __name__ == "__main__":
    unittest.main()
