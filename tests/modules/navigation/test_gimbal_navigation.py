"""Tests for GimbalNavigation — extracted from the tracking logic that used
to live inside DetectorSim.
"""

import unittest
from threading import Event, Thread
from unittest.mock import MagicMock, Mock, patch

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.navigation.gimbal_navigation import (
    GimbalNavigation,
    MODE_FOLLOW,
    MODE_LOCK,
)
from navpy.modules.navigation.gimbal_navigation_state import (
    GimbalLossPolicy,
    GimbalTrackingSetup,
)
from navpy.modules.vision.gimbal_rate_tracker import (
    GimbalRateTrackerConfig,
    TrackingState,
)
from navpy.modules.vision.gimbal_rate_types import (
    GimbalObservationDisposition,
    GimbalRateUpdate,
    GimbalTrackResult,
)
from navpy.modules.vision.models.detect_data import DetectedObject
from tests.detection_factory import make_detected_poi
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.poi_zoom_tracker import (
    PoiZoomTrackerConfig,
    ZoomTrackResult,
    ZoomTrackingState,
)


def _make_poi(x_error=960, y_error=540):
    k = np.array([[1000, 0, 960], [0, 1000, 540], [0, 0, 1]], dtype=np.float32)
    return make_detected_poi(
        obj_id=7,
        x_error=x_error,
        y_error=y_error,
        reference_height_m=2.0,
        k=k,
        g_data=GimbalData(att=Attitude(0, 0, 0)),
        uas_att=Attitude(0, 0, 0),
    )


def _make_mount():
    gimbal = MagicMock()
    # attribute that GimbalRateTracker.return_to_neutral() will poke via
    # gimbal.set_att(...)
    gimbal.set_att = MagicMock()
    gimbal.set_rate = MagicMock()
    gimbal.set_motion_mode = MagicMock()
    gimbal.zoom_in.return_value = True
    gimbal.zoom_out.return_value = True
    gimbal.zoom_hold.return_value = True
    mount = MagicMock()
    mount.name = "test_mount"
    mount.gimbal = gimbal
    mount.image_width = 1920
    mount.image_height = 1080
    mount.get_k.return_value = np.array(
        [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )
    mount.get_zoom_levels.return_value = ["1", "2", "10"]
    mount.get_current_zoom.return_value = "1"
    mount.get_current_zoom_command.return_value = "1"
    mount.get_fresh_zoom_command.return_value = "1"
    mount.get_fresh_zoom_sample_id.return_value = None
    mount.sync_zoom_from_hardware.return_value = True
    mount.command_zoom.return_value = True
    mount.zoom_calibration = None
    mount.supports_absolute_zoom.return_value = True
    mount.supports_continuous_zoom.return_value = True
    mount.start_continuous_zoom.side_effect = lambda direction: (
        gimbal.zoom_in()
        if direction is ZoomTrackingState.ZOOMING_IN
        else gimbal.zoom_out()
    )
    mount.hold_zoom.side_effect = lambda: gimbal.zoom_hold()
    # Distinctive gimbal bearing readback so tests can prove a Tier-2 last-LOS
    # hold re-points at the CAMERA bearing (pitch=-15, yaw=42) and NOT at
    # body-forward neutral (pitch=neutral, yaw=0).
    mount.get_gimbal_data.return_value = GimbalData(att=Attitude(-15.0, 42.0, 0.0))
    return mount, gimbal


def _rate_tracker(navigation):
    return navigation._parts.status._trackers.rate


def _zoom_tracker(navigation):
    return navigation._parts.status._trackers.zoom


class TestGimbalNavigation(unittest.TestCase):
    def setUp(self):
        self.mount, self.gimbal = _make_mount()
        self.logger = Mock()
        self.logger.is_enabled_for.return_value = False
        self.rate_cfg = GimbalRateTrackerConfig(
            correction_bw=1.0,
            max_rate=100.0,
            command_lead_time=0.15,
        )
        self.loss_policy = GimbalLossPolicy(
            hold_sec=0.5,
            repoint_sec=2.0,
        )
        self.tracking = GimbalTrackingSetup(self.rate_cfg, self.loss_policy)
        self.zoom_cfg = PoiZoomTrackerConfig(
            target_pixels={"default": 80.0},
        )

    def _make(
        self,
        *,
        with_rate=True,
        with_zoom=True,
        rate_tracker=None,
        zoom_tracker=None,
    ):
        return GimbalNavigation(
            self.mount,
            self.logger,
            tracking=self.tracking if with_rate else None,
            zoom_config=self.zoom_cfg if with_zoom else None,
            rate_tracker=rate_tracker,
            zoom_tracker=zoom_tracker,
            neutral_pitch_deg=-20.0,
        )

    # --- lifecycle ---------------------------------------------------------

    def test_start_tracking_switches_to_lock_and_stores_obj_id(self):
        g = self._make()
        g.start_tracking(42)

        self.assertEqual(g.tracking_obj_id, 42)
        self.gimbal.set_motion_mode.assert_called_once_with(MODE_LOCK)

    def test_stop_tracking_returns_to_follow_and_neutral(self):
        g = self._make()
        g.start_tracking(42)
        self.gimbal.reset_mock()

        g.stop_tracking()

        self.assertIsNone(g.tracking_obj_id)
        # set_motion_mode(FOLLOW) happens before return_to_neutral's set_att
        self.gimbal.set_motion_mode.assert_called_with(MODE_FOLLOW)
        # return_to_neutral issues set_att(neutral_pitch, 0, 0)
        self.gimbal.set_att.assert_called()
        called_attitude = self.gimbal.set_att.call_args.args[0]
        self.assertAlmostEqual(called_attitude.pitch, -20.0)
        self.assertAlmostEqual(called_attitude.yaw, 0.0)

    def test_stop_tracking_without_prior_start_is_noop(self):
        g = self._make()
        g.stop_tracking()
        # No mode switch, no set_att, and no set_zoom — a never-armed
        # navigation must not touch the gimbal on stop.
        self.gimbal.set_motion_mode.assert_not_called()
        self.gimbal.set_att.assert_not_called()
        self.mount.command_zoom.assert_not_called()

    def test_stop_tracking_keep_attitude_skips_follow_neutral(self):
        """to_neutral=False disarms + resets zoom but skips the FOLLOW mode
        switch and return_to_neutral() hardware calls (the geo-hold handoff
        shape: NavController re-arms geo-pointing at the SAME attitude right
        after this call, so recentring first would be a wasted swing)."""
        g = self._make()
        g.start_tracking(42)
        self.gimbal.reset_mock()
        self.mount.command_zoom.reset_mock()

        g.stop_tracking(to_neutral=False)

        self.assertIsNone(g.tracking_obj_id)
        self.gimbal.set_motion_mode.assert_not_called()
        self.gimbal.set_att.assert_not_called()
        self.gimbal.set_rate.assert_called_with(0.0, 0.0)
        # Zoom still resets — only the FOLLOW/neutral hardware calls are skipped.
        self.mount.command_zoom.assert_called_with("1")

    def test_loss_hold_sec_property_reads_rate_config(self):
        g = self._make()
        self.assertEqual(g.loss_hold_sec, self.loss_policy.hold_sec)

    def test_set_zoom_size_demand_forwards_to_tracker(self):
        g = self._make()
        g.set_zoom_size_demand(False)
        self.assertFalse(_zoom_tracker(g).size_demand)

    def test_set_zoom_size_demand_without_zoom_is_noop(self):
        g = self._make(with_zoom=False)
        g.set_zoom_size_demand(False)  # must not raise

    def test_final_approach_zoom_freeze_commands_min_and_never_reads_bbox(self):
        class PoisonBboxPoi:
            @property
            def tracking_bbox_cxcywh(self):
                raise AssertionError("bbox must not be read during final-approach NAV")

            @property
            def bbox_cxcywh(self):
                raise AssertionError("bbox must not be read during final-approach NAV")

        g = self._make(with_rate=False, with_zoom=True)
        g.start_tracking(7)
        g.freeze_final_approach_zoom_at_min()
        _zoom_tracker(g).update = Mock(
            side_effect=AssertionError("bbox-driven zoom tracker must stay frozen")
        )

        g.update(PoisonBboxPoi(), now=10.0)

        self.mount.command_zoom.assert_called_with("1")
        _zoom_tracker(g).update.assert_not_called()

    def test_final_approach_zoom_freeze_serializes_with_visual_update(self):
        g = self._make(with_rate=False, with_zoom=True)
        g.start_tracking(7)
        tracker = _zoom_tracker(g)
        freeze_entered = Event()
        release_freeze = Event()
        update_finished = Event()

        def blocking_reset():
            freeze_entered.set()
            self.assertTrue(release_freeze.wait(timeout=2.0))
            return True

        tracker.reset_to_min = Mock(side_effect=blocking_reset)
        tracker.update = Mock()
        freeze_result = []
        freezer = Thread(
            target=lambda: freeze_result.append(
                g.freeze_final_approach_zoom_at_min()
            )
        )
        updater = Thread(
            target=lambda: (
                g.update(_make_poi(), now=10.0),
                update_finished.set(),
            )
        )

        freezer.start()
        self.assertTrue(freeze_entered.wait(timeout=1.0))
        updater.start()
        self.assertFalse(update_finished.wait(timeout=0.05))
        release_freeze.set()
        freezer.join(timeout=2.0)
        updater.join(timeout=2.0)

        self.assertEqual(freeze_result, [True])
        self.assertFalse(freezer.is_alive())
        self.assertFalse(updater.is_alive())
        tracker.update.assert_not_called()

    def test_final_approach_zoom_freeze_waits_for_inflight_visual_update(self):
        g = self._make(with_rate=False, with_zoom=True)
        g.start_tracking(7)
        tracker = _zoom_tracker(g)
        update_entered = Event()
        release_update = Event()
        freeze_finished = Event()

        def blocking_update(*_args, **_kwargs):
            update_entered.set()
            self.assertTrue(release_update.wait(timeout=2.0))
            return tracker.last_result

        tracker.update = Mock(side_effect=blocking_update)
        updater = Thread(target=lambda: g.update(_make_poi(), now=10.0))
        freezer = Thread(
            target=lambda: (
                g.freeze_final_approach_zoom_at_min(),
                freeze_finished.set(),
            )
        )
        updater.start()
        self.assertTrue(update_entered.wait(timeout=1.0))
        freezer.start()
        self.assertFalse(freeze_finished.wait(timeout=0.05))
        release_update.set()
        updater.join(timeout=2.0)
        freezer.join(timeout=2.0)

        self.assertFalse(updater.is_alive())
        self.assertFalse(freezer.is_alive())
        tracker.update.assert_called_once()
        g.update(_make_poi(), now=10.1)
        tracker.update.assert_called_once()

    def test_failed_final_approach_zoom_freeze_latches_and_retries_hardware(self):
        g = self._make(with_rate=False, with_zoom=True)
        g.start_tracking(7)
        prior_result = _zoom_tracker(g).last_result
        self.mount.command_zoom.side_effect = OSError("link")

        self.assertFalse(g.freeze_final_approach_zoom_at_min())

        self.assertTrue(
            g.status.detection.final_approach_zoom_frozen_at_min
        )
        self.assertFalse(_zoom_tracker(g).size_demand)
        self.assertIs(_zoom_tracker(g).last_result, prior_result)

        self.mount.command_zoom.side_effect = None
        self.assertTrue(g.freeze_final_approach_zoom_at_min())
        self.assertTrue(
            g.status.detection.final_approach_zoom_frozen_at_min
        )
        self.assertFalse(_zoom_tracker(g).size_demand)

    def test_new_tracking_session_releases_final_approach_zoom_freeze(self):
        g = self._make(with_rate=False, with_zoom=True)
        g.start_tracking(7)
        g.freeze_final_approach_zoom_at_min()
        _zoom_tracker(g).update = Mock()

        g.start_tracking(8)
        poi = _make_poi()
        g.update(poi, now=10.0)

        _zoom_tracker(g).update.assert_called_once_with(poi, pointing=None)

    def test_start_tracking_restores_zoom_size_demand(self):
        g = self._make()
        g.set_zoom_size_demand(False)

        g.start_tracking(42)

        self.assertTrue(_zoom_tracker(g).size_demand)

    def test_start_tracking_rollback_restores_prior_zoom_size_demand(self):
        """A failed hardware arm must not leak the demand reset into the
        surviving (rolled-back) session."""
        g = self._make()
        g.set_zoom_size_demand(False)
        self.gimbal.set_motion_mode.side_effect = RuntimeError("UDP drop")

        with self.assertRaises(RuntimeError):
            g.start_tracking(42)

        self.assertFalse(_zoom_tracker(g).size_demand)

    def test_failed_zoom_hold_does_not_rebind_tracking_session(self):
        rate_tracker = Mock()
        g = self._make(rate_tracker=rate_tracker)
        g.start_tracking(7)
        rate_tracker.reset.reset_mock()
        poi = Mock(
            class_id=0,
            tracking_bbox_cxcywh=(960.0, 540.0, 10.0, 10.0),
            bbox_cxcywh=None,
        )
        _zoom_tracker(g).update(poi)
        _zoom_tracker(g).update(poi)
        g.set_zoom_size_demand(False)
        prior_result = _zoom_tracker(g).last_result
        prior_generation = g.status.generation
        self.gimbal.zoom_hold.side_effect = OSError("link")

        with self.assertRaisesRegex(OSError, "prior zoom session"):
            g.start_tracking(8)

        self.assertEqual(g.tracking_obj_id, 7)
        self.assertEqual(g.status.generation, prior_generation)
        self.assertFalse(_zoom_tracker(g).size_demand)
        self.assertIs(_zoom_tracker(g).last_result, prior_result)
        rate_tracker.reset.assert_not_called()

        self.gimbal.zoom_hold.side_effect = None
        g.start_tracking(8)
        self.assertEqual(g.tracking_obj_id, 8)
        self.assertTrue(_zoom_tracker(g).size_demand)

    def test_start_tracking_rolls_back_on_hardware_raise(self):
        """If set_motion_mode raises during arm, bookkeeping must revert to
        the pre-start state and the caller sees the exception so it can
        abort its own arm (e.g. Detector.start_tracking rolls back the
        YOLO lock)."""
        g = self._make(with_zoom=False)
        prev_gen = g.status.generation
        self.gimbal.set_motion_mode.side_effect = RuntimeError("UDP drop")

        with self.assertRaises(RuntimeError):
            g.start_tracking(42)

        # Armament rolled back
        self.assertIsNone(g.tracking_obj_id)
        # Generation bumped on both the arm AND the rollback so any in-flight
        # update() captures a stale entry_generation and bails.
        self.assertEqual(g.status.generation, prev_gen)

    def test_stop_tracking_failure_preserves_session_for_retry(self):
        """stop_tracking is best-effort teardown — a flaky UDP link must
        not propagate through the caller."""
        g = self._make(with_zoom=False)
        g.start_tracking(7)
        self.gimbal.set_motion_mode.side_effect = RuntimeError("UDP drop")

        with self.assertRaisesRegex(RuntimeError, "UDP drop"):
            g.stop_tracking()
        self.assertEqual(g.tracking_obj_id, 7)

        self.gimbal.set_motion_mode.side_effect = None
        g.stop_tracking()
        self.assertIsNone(g.tracking_obj_id)

    def test_arm_with_no_configured_trackers_is_noop(self):
        g = self._make(with_rate=False, with_zoom=False)
        g.arm()
        self.assertIsNone(g.tracking_obj_id)
        self.gimbal.set_motion_mode.assert_not_called()

    def test_zoom_only_start_tracking_arms_navigation_without_mode_switch(self):
        g = self._make(with_rate=False, with_zoom=True)
        g.start_tracking(5)
        self.assertEqual(g.tracking_obj_id, 5)
        # No rate tracker → no LOCK/FOLLOW mode switch should be issued.
        self.gimbal.set_motion_mode.assert_not_called()

    def test_zoom_only_stop_tracking_resets_zoom(self):
        g = self._make(with_rate=False, with_zoom=True)
        g.start_tracking(5)
        self.mount.command_zoom.reset_mock()
        g.stop_tracking()
        self.mount.command_zoom.assert_called_with("1")
        self.assertIsNone(g.tracking_obj_id)

    def test_rearm_clears_prior_state(self):
        """start_tracking after stop_tracking must leave _last_track_time /
        _gimbal_recentered / _last_tracked_for_zoom clean so loss-timeout
        timing doesn't carry over from the previous session."""
        g = self._make(with_zoom=False)
        g.start_tracking(7)
        g.update(_make_poi(), now=10.0)        # prime _last_track_time
        g.update(None, now=12.5)                   # force recentre
        g.stop_tracking()

        # Re-arm
        g.start_tracking(9)
        snapshot = g.status.detection
        self.assertIsNone(snapshot.last_track_time)
        self.assertFalse(snapshot.recentered)
        self.assertIsNone(snapshot.last_tracked_for_zoom)

    def test_update_skips_commit_when_stop_races_mid_call(self):
        """If stop_tracking fires while update() is between snapshot and
        commit, the commit must not write recentre / cached-zoom state
        into the disarmed navigation."""
        g = self._make(with_zoom=False)
        g.start_tracking(7)
        # Prime the recentre state machine so a commit would try to
        # write _last_track_time + _gimbal_recentered.
        g.update(_make_poi(), now=10.0)

        racing_stop_calls = []

        def stop_mid_update(_sample, **_kwargs):
            # Called from inside the (mocked) rate tracker's update.
            g.stop_tracking()
            racing_stop_calls.append(True)
            return GimbalRateUpdate(
                GimbalTrackResult(TrackingState.TRACKING, True),
                GimbalObservationDisposition.ACCEPTED,
            )

        _rate_tracker(g).update = Mock(side_effect=stop_mid_update)
        g.update(_make_poi(), now=10.1)

        self.assertTrue(racing_stop_calls, "race hook did not fire")
        # Navigation is disarmed; bookkeeping must remain clean.
        self.assertIsNone(g.tracking_obj_id)
        snapshot = g.status.detection
        self.assertIsNone(snapshot.last_track_time)
        self.assertFalse(snapshot.recentered)
        self.assertIsNone(snapshot.last_tracked_for_zoom)

    def test_update_skips_commit_when_stop_start_sandwich_races(self):
        """A stop_tracking + start_tracking sandwich with a new obj_id
        mid-update must not let the prior session's commit clobber the
        fresh session's bookkeeping."""
        g = self._make(with_zoom=False)
        g.start_tracking(7)
        g.update(_make_poi(), now=10.0)

        def restart_mid_update(_sample, **_kwargs):
            g.stop_tracking()
            g.start_tracking(99)
            return GimbalRateUpdate(
                GimbalTrackResult(TrackingState.TRACKING, True),
                GimbalObservationDisposition.ACCEPTED,
            )

        _rate_tracker(g).update = Mock(side_effect=restart_mid_update)
        g.update(_make_poi(), now=10.1)

        # New session is armed; prior recentre / cache state must NOT bleed.
        self.assertEqual(g.tracking_obj_id, 99)
        snapshot = g.status.detection
        self.assertIsNone(snapshot.last_track_time)
        self.assertFalse(snapshot.recentered)
        self.assertIsNone(snapshot.last_tracked_for_zoom)

    def test_loss_recentre_latch_committed_only_under_session_gate(self):
        """Loss recovery must not commit after the session generation changes."""
        g = self._make(with_zoom=False)
        g.start_tracking(7)
        g.update(_make_poi(), now=10.0)
        g.update(None, now=12.5)
        self.assertTrue(g.status.detection.recentered)

        g2 = self._make(with_zoom=False)
        g2.start_tracking(7)
        g2.update(_make_poi(), now=10.0)
        fence = g2._parts.status._fence
        raced = []

        def advance_session(_attitude):
            # Model the generation change from a session restart during the
            # loss hardware call. The rate-tracker race tests miss this path:
            # loss does not call rate_tracker.update().
            with fence.lock:
                fence.generation += 1
            raced.append(True)

        self.gimbal.set_att.side_effect = advance_session
        g2.update(None, now=12.5)

        self.assertEqual(raced, [True], "loss hardware race hook did not fire")
        self.assertFalse(g2.status.detection.recentered)

    def test_loss_is_not_forwarded_to_rate_tracker(self):
        g = self._make(with_zoom=False)
        g.start_tracking(7)
        g.update(_make_poi(), now=10.0)
        _rate_tracker(g).update = Mock(
            side_effect=AssertionError("loss reached rate tracker")
        )
        self.gimbal.set_motion_mode.reset_mock()
        self.gimbal.set_att.reset_mock()

        g.update(None, now=13.0)

        _rate_tracker(g).update.assert_not_called()
        self.gimbal.set_motion_mode.assert_called_once_with(MODE_FOLLOW)

    def test_zoom_only_update_feeds_zoom_tracker_directly(self):
        """Zoom-only mount must still exercise the zoom tracker through
        update() — covers the cached_for_zoom pass-through branch that
        only runs when _rate_tracker is None."""
        zoom_mock = MagicMock()
        zoom_mock.size_demand = True
        g = self._make(
            with_rate=False,
            with_zoom=True,
            zoom_tracker=zoom_mock,
        )
        g.start_tracking(5)

        t = _make_poi()
        g.update(t, now=10.0)

        # Zoom-only mounts have no pointing data: the centering gate is
        # disabled via pointing=None.
        zoom_mock.update.assert_called_with(t, pointing=None)
        # No rate-tracker hardware commands should be attempted.
        self.gimbal.set_motion_mode.assert_not_called()
        self.gimbal.set_rate.assert_not_called()

    def test_update_is_noop_when_disarmed(self):
        """update() must not drive the gimbal when the navigation hasn't been
        armed — covers the race where a track thread reads tracking_obj_id
        before stop_tracking but calls update afterwards."""
        g = self._make(with_zoom=False)
        # No start_tracking / arm.
        g.update(_make_poi(), now=10.0)
        self.gimbal.set_motion_mode.assert_not_called()
        # Rate tracker is not ticked when disarmed — set_rate only fires
        # if the tracker sees a detection.
        self.gimbal.set_rate.assert_not_called()

    # --- per-tick: rate tracking ------------------------------------------

    def test_update_with_poi_feeds_rate_tracker(self):
        g = self._make(with_zoom=False)
        g.start_tracking(7)
        g.update(_make_poi(x_error=960, y_error=540), now=10.0)

        # Rate tracker issues set_rate on the gimbal for every tick
        self.gimbal.set_rate.assert_called()

    def test_update_tier1_hold_no_mode_change(self):
        # Tier 1 HOLD [0, loss_hold_sec=0.5): the rate tracker coasts then holds
        # rate=0 in LOCK; gimbal navigation issues NO mode change / set_att and
        # does not latch holding or recentre.
        g = self._make(with_zoom=False)
        g.start_tracking(7)
        g.update(_make_poi(), now=10.0)
        self.gimbal.set_motion_mode.reset_mock()
        self.gimbal.set_att.reset_mock()

        # 0.3 s after last detection — inside loss_hold_sec (0.5)
        g.update(None, now=10.3)

        self.gimbal.set_motion_mode.assert_not_called()
        self.gimbal.set_att.assert_not_called()
        snapshot = g.status.detection
        self.assertTrue(snapshot.holding)
        self.assertFalse(snapshot.recentered)

    def test_update_tier2_zero_rate_never_replays_body_readback(self):
        # Tier 2 POINT [loss_hold_sec=0.5, loss_repoint_sec=2.0): re-assert LOCK
        # and the LAST CAMERA BEARING (readback -15/42), NOT body-forward
        # neutral. Pure camera-state hold; no zoom reset.
        g = self._make(with_zoom=False)
        g.start_tracking(7)
        g.update(_make_poi(), now=10.0)
        self.gimbal.set_motion_mode.reset_mock()
        self.gimbal.set_att.reset_mock()
        self.gimbal.set_rate.reset_mock()
        self.mount.get_gimbal_data.reset_mock()

        # 1.0 s after last detection — Tier 2
        g.update(None, now=11.0)

        self.gimbal.set_motion_mode.assert_not_called()
        self.gimbal.set_att.assert_not_called()
        self.gimbal.set_rate.assert_called_once_with(0.0, 0.0)
        self.mount.get_gimbal_data.assert_not_called()
        snapshot = g.status.detection
        self.assertTrue(snapshot.holding)
        self.assertFalse(snapshot.recentered)

    def test_update_tier2_does_not_repeat_zero_rate_or_attitude(self):
        # The point-at-last-LOS command must re-fire each tick it stays in
        # Tier 2 (SIYI holds the last commanded attitude, so a single dropped
        # datagram must not leave the camera drifting), while the log/flag
        # transition fires once.
        g = self._make(with_zoom=False)
        g.start_tracking(7)
        g.update(_make_poi(), now=10.0)
        g.update(None, now=11.0)  # Tier 2 (first)
        self.gimbal.set_att.reset_mock()
        self.gimbal.set_rate.reset_mock()
        self.gimbal.set_motion_mode.reset_mock()

        g.update(None, now=11.5)  # Tier 2 (still)

        self.gimbal.set_att.assert_not_called()
        self.gimbal.set_rate.assert_not_called()
        self.gimbal.set_motion_mode.assert_not_called()

    def test_update_tier3_returns_to_search(self):
        g = self._make(with_zoom=False)
        g.start_tracking(7)
        g.update(_make_poi(), now=10.0)
        self.gimbal.set_motion_mode.reset_mock()
        self.gimbal.set_att.reset_mock()

        # 2.5 s after last detection — past loss_repoint_sec (2.0)
        g.update(None, now=12.5)

        self.gimbal.set_motion_mode.assert_called_with(MODE_FOLLOW)
        self.gimbal.set_att.assert_called()
        called_attitude = self.gimbal.set_att.call_args.args[0]
        self.assertAlmostEqual(called_attitude.pitch, -20.0)
        self.assertAlmostEqual(called_attitude.yaw, 0.0)
        self.assertTrue(g.status.detection.recentered)

    def test_return_to_search_fires_only_once_until_reacquired(self):
        g = self._make(with_zoom=False)
        g.start_tracking(7)
        g.update(_make_poi(), now=10.0)

        g.update(None, now=12.5)  # first return-to-search (past 2.0 repoint)
        self.gimbal.set_att.reset_mock()
        self.gimbal.set_motion_mode.reset_mock()

        g.update(None, now=13.0)  # still lost — should NOT re-send recentre
        self.gimbal.set_att.assert_not_called()
        self.gimbal.set_motion_mode.assert_not_called()

    def test_reacquire_after_return_to_search_switches_back_to_lock(self):
        g = self._make(with_zoom=False)
        g.start_tracking(7)
        g.update(_make_poi(), now=10.0)
        g.update(None, now=12.5)  # return to FOLLOW (past 2.0 repoint)
        self.gimbal.set_motion_mode.reset_mock()

        g.update(_make_poi(), now=13.0)  # re-acquired

        self.gimbal.set_motion_mode.assert_called_with(MODE_LOCK)

    def test_reacquire_after_tier2_hold_clears_holding_no_mode_flip(self):
        # After a Tier-2 last-LOS hold, re-acquiring must clear the holding flag
        # and NOT need a FOLLOW->LOCK mode flip (the gimbal never left LOCK).
        g = self._make(with_zoom=False)
        g.start_tracking(7)
        g.update(_make_poi(), now=10.0)
        g.update(None, now=11.0)  # Tier 2 -> holding
        self.assertTrue(g.status.detection.holding)
        self.gimbal.set_motion_mode.reset_mock()

        g.update(_make_poi(), now=11.5)  # re-acquired from hold

        self.assertFalse(g.status.detection.holding)
        self.gimbal.set_motion_mode.assert_not_called()

    # --- per-tick: zoom tracking ------------------------------------------

    def _make_with_tracker_mocks(self, *, rate_state=TrackingState.TRACKING):
        rate_mock = MagicMock()
        rate_mock.state = rate_state
        rate_result = GimbalTrackResult(
            state=rate_state,
            has_poi=rate_state is TrackingState.TRACKING,
        )
        rate_mock.last_result = rate_result
        rate_mock.update.return_value = GimbalRateUpdate(
            rate_result,
            GimbalObservationDisposition.ACCEPTED,
        )
        zoom_mock = MagicMock()
        zoom_mock.size_demand = True
        navigation = self._make(
            rate_tracker=rate_mock,
            zoom_tracker=zoom_mock,
        )
        navigation.start_tracking(7)
        return navigation, rate_mock, zoom_mock

    def test_zoom_tracker_receives_poi_in_tracking_state(self):
        g, rate_mock, zoom_mock = self._make_with_tracker_mocks(
            rate_state=TrackingState.TRACKING,
        )

        poi = _make_poi()
        g.update(poi, now=10.0)

        # The zoom tracker receives the rate tracker's result of THIS
        # tick as the pointing snapshot for the centering gate.
        zoom_mock.update.assert_called_with(
            poi, pointing=rate_mock.update.return_value.result,
        )

    def test_zoom_tracker_receives_no_stale_poi_during_loss(self):
        g, rate_mock, zoom_mock = self._make_with_tracker_mocks(
            rate_state=TrackingState.TRACKING,
        )

        # First tick: TRACKING with a real POI — navigation caches it
        t0 = _make_poi(x_error=960)
        g.update(t0, now=10.0)

        # Next tick: COASTING without a POI — navigation feeds the cache
        rate_mock.state = TrackingState.COASTING
        zoom_mock.reset_mock()
        g.update(None, now=10.1)

        poi_arg = zoom_mock.update.call_args.args[0]
        pointing = zoom_mock.update.call_args.kwargs["pointing"]
        self.assertIsNone(poi_arg)
        self.assertEqual(pointing.state, TrackingState.COASTING)
        self.assertFalse(pointing.has_poi)

    def test_zoom_tracker_receives_none_in_holding_state(self):
        g, rate_mock, zoom_mock = self._make_with_tracker_mocks(
            rate_state=TrackingState.HOLDING,
        )

        g.update(None, now=15.0)

        poi_arg = zoom_mock.update.call_args.args[0]
        pointing = zoom_mock.update.call_args.kwargs["pointing"]
        self.assertIsNone(poi_arg)
        self.assertEqual(pointing.state, TrackingState.IDLE)

    def test_is_zoom_stable_true_when_no_zoom_configured(self):
        g = self._make(with_zoom=False)
        self.assertTrue(g.is_zoom_stable)

    def test_zoom_result_none_when_no_zoom_configured(self):
        g = self._make(with_zoom=False)
        self.assertIsNone(g.zoom_result)

    def test_zoom_result_returns_tracker_snapshot(self):
        expected = ZoomTrackResult(
            state=ZoomTrackingState.HOLDING,
            has_poi=True,
            size_px=80.0,
            target_pixels=80.0,
            reason="above-minimum",
        )
        zoom_tracker = MagicMock()
        zoom_tracker.last_result = expected
        g = self._make(zoom_tracker=zoom_tracker)

        self.assertIs(g.zoom_result, expected)

    def test_stop_tracking_resets_zoom_to_min(self):
        g = self._make()
        g.start_tracking(7)
        self.mount.command_zoom.reset_mock()

        g.stop_tracking()

        # min_zoom=1.0 stringified. No AF — SIYI continuous-AF owns focus.
        self.mount.command_zoom.assert_called_with("1")

    def test_tier3_return_to_search_resets_zoom_to_min(self):
        g = self._make()
        g.start_tracking(7)
        g.update(_make_poi(), now=10.0)
        self.mount.command_zoom.reset_mock()

        g.update(None, now=12.5)  # past 2.0 repoint boundary -> Tier 3

        self.mount.command_zoom.assert_called_with("1")
        self.assertTrue(g.status.detection.recentered)
        self.assertEqual(_zoom_tracker(g).last_result.state.name, "IDLE")

    def test_zoom_preserved_through_tiers_1_2_reset_only_in_tier3(self):
        # The whole point of the fix: a brief/medium loss must NOT reset zoom,
        # so an orbiting UAV keeps the pixels it built toward the 48px confirm
        # gate through detector stalls. Zoom is only reset once we give up and
        # return to search (Tier 3).
        g = self._make()
        g.start_tracking(7)
        g.update(_make_poi(), now=10.0)
        self.mount.command_zoom.reset_mock()

        g.update(None, now=10.3)   # Tier 1 HOLD
        self.mount.command_zoom.assert_not_called()

        g.update(None, now=11.0)   # Tier 2 POINT
        self.mount.command_zoom.assert_not_called()
        snapshot = g.status.detection
        self.assertTrue(snapshot.holding)
        self.assertFalse(snapshot.recentered)

        g.update(None, now=12.5)   # Tier 3 RETURN
        self.mount.command_zoom.assert_called_with("1")
        self.assertTrue(g.status.detection.recentered)

    def test_zoom_reset_during_loss_when_preservation_disabled(self):
        # preserve_zoom_during_loss=False restores the legacy behavior: zoom is
        # reset as soon as the point tier (Tier 2) engages.
        self.rate_cfg = GimbalRateTrackerConfig(
            correction_bw=1.0,
            max_rate=100.0,
            command_lead_time=0.15,
        )
        self.loss_policy = GimbalLossPolicy(
            hold_sec=0.5,
            repoint_sec=2.0,
            preserve_zoom_during_loss=False,
        )
        self.tracking = GimbalTrackingSetup(self.rate_cfg, self.loss_policy)
        g = self._make()
        g.start_tracking(7)
        g.update(_make_poi(), now=10.0)
        self.mount.command_zoom.reset_mock()

        g.update(None, now=11.0)  # Tier 2 with preservation off -> zoom reset

        self.mount.command_zoom.assert_called_with("1")

    def test_return_to_search_does_not_reset_zoom_when_no_zoom_tracker(self):
        g = self._make(with_zoom=False)
        g.start_tracking(7)
        g.update(_make_poi(), now=10.0)
        self.mount.command_zoom.reset_mock()
        self.gimbal.set_motion_mode.reset_mock()
        self.gimbal.set_att.reset_mock()

        g.update(None, now=12.5)  # past 2.0 repoint boundary -> Tier 3

        self.mount.command_zoom.assert_not_called()
        self.gimbal.set_motion_mode.assert_called_with(MODE_FOLLOW)
        self.gimbal.set_att.assert_called()

    def test_return_to_search_tolerates_zoom_reset_failure(self):
        g = self._make()
        g.start_tracking(7)
        g.update(_make_poi(), now=10.0)
        self.mount.command_zoom.side_effect = OSError("UDP drop")
        self.gimbal.set_motion_mode.reset_mock()
        self.gimbal.set_att.reset_mock()

        g.update(None, now=12.5)  # past 2.0 repoint boundary -> Tier 3

        self.assertTrue(g.status.detection.recentered)
        self.gimbal.set_motion_mode.assert_called_with(MODE_FOLLOW)
        self.gimbal.set_att.assert_called()


class _FakeLocation:
    """Minimal stand-in for navpy.modules.common.models.location.Location.

    GimbalNavigation only forwards Locations to ``geo_ref.calc_gimbal_lock_att_loc``
    as opaque objects, so the tests don't need the real Location class.
    """

    def __init__(self, lat=0.0, lng=0.0, alt=0.0):
        self.lat = lat
        self.lng = lng
        self.alt = alt

    def __repr__(self):
        return f"_FakeLocation(lat={self.lat}, lng={self.lng}, alt={self.alt})"


class _FakeGeoRef:
    """Recording fake GeoRefCalc that returns a fixed LOCK command."""

    def __init__(self, pitch=-15.0, yaw=45.0, uv=(50.0, 50.0)):
        self.pitch = pitch
        self.yaw = yaw
        self.uv = uv
        self.calls = []
        self.uv_calls = []

    def calc_gimbal_lock_att_loc(self, uav_loc, poi_loc, uas_att, g_data):
        self.calls.append((uav_loc, poi_loc, uas_att, g_data))
        return Attitude(self.pitch, self.yaw, 0.0)

    def calc_uv(self, p_ned, k, g_data, uas_att):
        self.uv_calls.append((p_ned, k, g_data, uas_att))
        return self.uv


class TestGimbalNavigationGeo(unittest.TestCase):
    """Pre-acquisition geo-pointing API on GimbalNavigation."""

    def setUp(self):
        self.mount, self.gimbal = _make_mount()
        self.logger = Mock()
        self.logger.is_enabled_for.return_value = False
        self.rate_cfg = GimbalRateTrackerConfig(
            correction_bw=1.0,
            max_rate=100.0,
            command_lead_time=0.15,
        )
        self.loss_policy = GimbalLossPolicy(
            hold_sec=0.5,
            repoint_sec=2.0,
        )
        self.tracking = GimbalTrackingSetup(self.rate_cfg, self.loss_policy)
        self.poi_loc = _FakeLocation(lat=40.3, lng=44.4, alt=1500.0)
        self.uav_loc = _FakeLocation(lat=40.31, lng=44.41, alt=1700.0)
        self.uav_att = Attitude(0, 0, 0)
        self.geo_ref = _FakeGeoRef(pitch=-25.0, yaw=120.0)
        self.g_data = GimbalData(att=Attitude(0.0, 0.0, 0.0))
        self.mount.get_gimbal_data.return_value = self.g_data
        self.zoom_cfg = PoiZoomTrackerConfig(
            target_pixels={"default": 80.0},
        )

    def _make(self, *, with_rate=True, with_zoom=False):
        return GimbalNavigation(
            self.mount,
            self.logger,
            tracking=self.tracking if with_rate else None,
            zoom_config=self.zoom_cfg if with_zoom else None,
            neutral_pitch_deg=-20.0,
        )

    def _configure_acquisition(
        self,
        *,
        zoom_entry=None,
        current_k=None,
    ):
        self.mount.camera = Mock()
        self.mount.camera._zoom_map = {
            "2": zoom_entry or {
                "fx": 500.0,
                "fy": 500.0,
                "cx": 50.0,
                "cy": 50.0,
            },
        }
        self.mount.get_zoom_levels.return_value = ["2"]
        self.mount.get_k.return_value = (
            np.array([
                [100.0, 0.0, 50.0],
                [0.0, 100.0, 50.0],
                [0.0, 0.0, 1.0],
            ])
            if current_k is None
            else current_k
        )
        self.mount.get_gimbal_data.return_value = GimbalData(
            att=Attitude(0, 0, 0)
        )
        self.mount.is_valid.return_value = True
        self.mount.set_zoom.return_value = True

    def _prepare_acquisition(
        self,
        navigation,
        *,
        poi_ned=None,
        min_pixels=10.0,
    ):
        navigation.start_geo_tracking(self.poi_loc, self.geo_ref)
        with patch(
            "navpy.modules.navigation.gimbal_navigation_composition.pymap3d.geodetic2ned",
            return_value=(
                np.array([100.0, 0.0, 0.0])
                if poi_ned is None
                else poi_ned
            ),
        ):
            return navigation.prepare_geo_acquisition(
                self.uav_loc,
                self.uav_att,
                class_id=0,
                min_pixels=min_pixels,
            )

    # --- visual-only detection-loss boundary ------------------------------

    def _armed_after_geo_handoff(self, *, with_zoom=False):
        """Enter visual tracking after a legacy pre-acquisition geo phase."""
        navigation = self._make(with_zoom=with_zoom)
        navigation.start_geo_tracking(self.poi_loc, self.geo_ref)
        navigation.start_tracking(7)
        return navigation

    def test_visual_handoff_discards_geo_reacquisition_source(self):
        navigation = self._armed_after_geo_handoff()

        self.assertFalse(navigation.is_geo_armed)
        self.assertTrue(navigation.is_detection_armed)
        self.assertIsNone(navigation.status.geo.poi)
        self.assertIsNone(navigation.status.geo.geo_ref)

    def test_visual_loss_recentres_without_reading_poi_geo(self):
        navigation = self._armed_after_geo_handoff()
        navigation.update(_make_poi(), now=10.0)
        self.gimbal.set_motion_mode.reset_mock()
        self.gimbal.set_att.reset_mock()
        self.geo_ref.calls.clear()

        navigation.update(None, now=12.5)  # past 2.0 repoint boundary -> Tier 3

        self.assertEqual(self.geo_ref.calls, [])
        self.gimbal.set_motion_mode.assert_called_with(MODE_FOLLOW)
        attitude = self.gimbal.set_att.call_args.args[0]
        self.assertAlmostEqual(attitude.pitch, -20.0)
        self.assertAlmostEqual(attitude.yaw, 0.0)

    def test_tier2_zero_rate_uses_neither_camera_readback_nor_geo(self):
        # Pure-vision: the Tier-2 last-LOS hold must re-point using ONLY the
        # gimbal roll-yaw-pitch readback (camera-state). It must NEVER consult
        # the geo POI / geo_ref pose math during a CONFIRM loss. This fake
        # geo_ref FAILS (records a call) if the loss path touches it.
        self.mount.get_gimbal_data.return_value = GimbalData(
            att=Attitude(-12.0, 33.0, 0.0)
        )
        navigation = self._armed_after_geo_handoff()
        navigation.update(_make_poi(), now=10.0)
        self.gimbal.set_att.reset_mock()
        self.gimbal.set_motion_mode.reset_mock()
        self.gimbal.set_rate.reset_mock()
        self.mount.get_gimbal_data.reset_mock()
        self.geo_ref.calls.clear()

        navigation.update(None, now=11.0)  # Tier 2 POINT

        # No geo pose math was invoked during the loss re-point.
        self.assertEqual(self.geo_ref.calls, [])
        self.gimbal.set_att.assert_not_called()
        self.gimbal.set_motion_mode.assert_not_called()
        self.gimbal.set_rate.assert_called_once_with(0.0, 0.0)
        self.mount.get_gimbal_data.assert_not_called()

    def test_visual_tracking_update_rejects_aircraft_pose_inputs(self):
        navigation = self._armed_after_geo_handoff()

        with self.assertRaises(TypeError):
            navigation.update(
                None,
                now=12.5,
                uav_loc=self.uav_loc,
                uav_att=self.uav_att,
            )

    def test_visual_loss_resets_zoom_instead_of_geo_holding(self):
        navigation = self._armed_after_geo_handoff(with_zoom=True)
        navigation.update(_make_poi(), now=10.0)
        self.mount.command_zoom.reset_mock()

        navigation.update(None, now=12.5)  # past 2.0 repoint boundary -> Tier 3

        self.mount.command_zoom.assert_called_with("1")

    # --- arm / disarm -----------------------------------------------------

    def test_start_geo_tracking_switches_to_lock_and_records_poi(self):
        g = self._make()
        g.start_geo_tracking(self.poi_loc, self.geo_ref)

        self.assertTrue(g.is_geo_armed)
        self.assertFalse(g.is_detection_armed)
        self.gimbal.set_motion_mode.assert_called_once_with(MODE_LOCK)
        # No set_att on arm — first command is on the first update_geo tick.
        self.gimbal.set_att.assert_not_called()

    def test_start_geo_tracking_no_op_without_rate_tracker(self):
        g = self._make(with_rate=False)
        g.start_geo_tracking(self.poi_loc, self.geo_ref)

        self.assertFalse(g.is_geo_armed)
        self.gimbal.set_motion_mode.assert_not_called()

    def test_start_geo_tracking_raises_when_detection_armed(self):
        g = self._make()
        g.start_tracking(7)
        self.gimbal.reset_mock()

        with self.assertRaises(RuntimeError):
            g.start_geo_tracking(self.poi_loc, self.geo_ref)

        self.assertFalse(g.is_geo_armed)
        self.gimbal.set_motion_mode.assert_not_called()
        self.gimbal.set_att.assert_not_called()

    def test_start_geo_tracking_rolls_back_on_lock_failure(self):
        g = self._make()
        self.gimbal.set_motion_mode.side_effect = RuntimeError("UDP drop")

        with self.assertRaises(RuntimeError):
            g.start_geo_tracking(self.poi_loc, self.geo_ref)

        self.assertFalse(g.is_geo_armed)

    def test_start_geo_tracking_swap_does_not_reassert_lock(self):
        g = self._make()
        g.start_geo_tracking(self.poi_loc, self.geo_ref)
        self.gimbal.reset_mock()

        new_poi = _FakeLocation(lat=41.0, lng=45.0, alt=1600.0)
        g.start_geo_tracking(new_poi, self.geo_ref)

        self.assertTrue(g.is_geo_armed)
        self.gimbal.set_motion_mode.assert_not_called()

    def test_stop_geo_tracking_returns_to_follow_and_neutral(self):
        g = self._make()
        g.start_geo_tracking(self.poi_loc, self.geo_ref)
        self.gimbal.reset_mock()

        g.stop_geo_tracking()

        self.assertFalse(g.is_geo_armed)
        self.gimbal.set_motion_mode.assert_called_with(MODE_FOLLOW)
        # return_to_neutral issues set_att(neutral_pitch, 0, 0)
        self.gimbal.set_att.assert_called()
        called_attitude = self.gimbal.set_att.call_args.args[0]
        self.assertAlmostEqual(called_attitude.pitch, -20.0)

    def test_stop_geo_tracking_when_never_armed_does_not_touch_hardware(self):
        g = self._make()

        g.stop_geo_tracking()

        self.gimbal.set_motion_mode.assert_not_called()
        self.gimbal.set_att.assert_not_called()

    # --- per-tick update_geo ---------------------------------------------

    def test_update_geo_emits_set_att_with_pose_math_result(self):
        g = self._make()
        g.start_geo_tracking(self.poi_loc, self.geo_ref)
        self.gimbal.reset_mock()

        g.update_geo(self.uav_loc, self.uav_att)

        self.assertEqual(len(self.geo_ref.calls), 1)
        recorded_uav, recorded_poi, recorded_att, recorded_g_data = self.geo_ref.calls[0]
        self.assertIs(recorded_uav, self.uav_loc)
        self.assertIs(recorded_poi, self.poi_loc)
        self.assertIs(recorded_att, self.uav_att)
        self.assertIs(recorded_g_data, self.g_data)

        self.gimbal.set_att.assert_called_once()
        att = self.gimbal.set_att.call_args.args[0]
        self.assertAlmostEqual(att.pitch, -25.0)
        self.assertAlmostEqual(att.yaw, 120.0)
        self.assertAlmostEqual(att.roll, 0.0)

    def test_update_geo_skips_ray_diagnostic_when_debug_disabled(self):
        g = self._make()
        g.start_geo_tracking(self.poi_loc, self.geo_ref)
        self.gimbal.reset_mock()

        with patch(
            "navpy.modules.navigation.gimbal_navigation_composition.compute_geo_ray_diagnostic",
        ) as compute_diag:
            g.update_geo(self.uav_loc, self.uav_att)

        compute_diag.assert_not_called()
        self.logger.debug.assert_not_called()

    def test_update_geo_debug_logs_cached_ray_diagnostic(self):
        g = self._make()
        g.start_geo_tracking(self.poi_loc, self.geo_ref)
        self.gimbal.reset_mock()
        self.logger.is_enabled_for.return_value = True
        self.mount.get_k.return_value = np.array([
            [100.0, 0.0, 50.0],
            [0.0, 100.0, 50.0],
            [0.0, 0.0, 1.0],
        ])
        g_data = GimbalData(att=Attitude(-20.0, 30.0, 0.0))
        self.mount.get_gimbal_data.return_value = g_data
        self.mount.is_valid.return_value = True
        diagnostic = Mock()
        diagnostic.cache_key.return_value = ("same",)
        diagnostic.format_log.return_value = "GEO_RAY test"
        self.logger.debug.reset_mock()

        with patch(
            "navpy.modules.navigation.gimbal_navigation_composition.compute_geo_ray_diagnostic",
            return_value=diagnostic,
        ) as compute_diag:
            g.update_geo(self.uav_loc, self.uav_att)
            g.update_geo(self.uav_loc, self.uav_att)

        self.assertEqual(compute_diag.call_count, 2)
        kwargs = compute_diag.call_args.kwargs
        self.assertIs(kwargs["poi_loc"], self.poi_loc)
        self.assertIs(kwargs["uav_loc"], self.uav_loc)
        self.assertIs(kwargs["uav_att"], self.uav_att)
        self.assertIs(kwargs["k"], self.mount.get_k.return_value)
        self.assertIs(kwargs["g_data"], g_data)
        self.assertIs(kwargs["geo_ref"], self.geo_ref)
        self.assertIs(kwargs["is_valid_pixel"], self.mount.is_valid)
        diagnostic.cache_key.assert_called_with(-25.0, 120.0)
        diagnostic.format_log.assert_called_with(
            self.mount.name, -25.0, 120.0, g_data.att,
        )
        self.logger.debug.assert_called_once_with("GEO_RAY test")

    def test_update_geo_no_op_when_disarmed(self):
        g = self._make()

        g.update_geo(self.uav_loc, self.uav_att)

        self.gimbal.set_att.assert_not_called()
        self.assertEqual(self.geo_ref.calls, [])

    def test_prepare_geo_acquisition_sets_smallest_valid_zoom(self):
        g = self._make(with_zoom=True)
        self.mount.camera = Mock()
        self.mount.camera._zoom_map = {
            "1": {"fx": 100.0, "fy": 100.0, "cx": 50.0, "cy": 50.0},
            "2": {"fx": 500.0, "fy": 500.0, "cx": 50.0, "cy": 50.0},
            "3": {"fx": 1000.0, "fy": 1000.0, "cx": 50.0, "cy": 50.0},
        }
        self.mount.get_zoom_levels.return_value = ["1", "2", "3"]
        self.mount.get_k.return_value = np.array([
            [100.0, 0.0, 50.0],
            [0.0, 100.0, 50.0],
            [0.0, 0.0, 1.0],
        ])
        self.mount.get_gimbal_data.return_value = GimbalData(att=Attitude(0, 0, 0))
        self.mount.is_valid.return_value = True
        self.mount.set_zoom.return_value = True
        g.start_geo_tracking(self.poi_loc, self.geo_ref)

        with patch(
            "navpy.modules.navigation.gimbal_navigation_composition.pymap3d.geodetic2ned",
            return_value=np.array([100.0, 0.0, 0.0]),
        ):
            prepared = g.prepare_geo_acquisition(
                self.uav_loc, self.uav_att, class_id=0, min_pixels=10.0,
            )

        self.assertTrue(prepared)
        self.mount.set_zoom.assert_called_once_with("2")

    def test_prepare_geo_acquisition_reduces_excessive_existing_zoom(self):
        g = self._make(with_zoom=True)
        self.mount.camera = Mock()
        self.mount.camera._zoom_map = {
            "1": {"fx": 100.0, "fy": 100.0, "cx": 50.0, "cy": 50.0},
            "2": {"fx": 500.0, "fy": 500.0, "cx": 50.0, "cy": 50.0},
            "5": {"fx": 1250.0, "fy": 1250.0, "cx": 50.0, "cy": 50.0},
        }
        self.mount.get_zoom_levels.return_value = ["1", "2", "5"]
        self.mount.get_k.return_value = np.array([
            [1250.0, 0.0, 50.0],
            [0.0, 1250.0, 50.0],
            [0.0, 0.0, 1.0],
        ])
        self.mount.get_gimbal_data.return_value = GimbalData(
            att=Attitude(0, 0, 0)
        )
        self.mount.is_valid.return_value = True
        self.mount.set_zoom.return_value = True
        g.start_geo_tracking(self.poi_loc, self.geo_ref)

        with patch(
            "navpy.modules.navigation.gimbal_navigation_composition.pymap3d.geodetic2ned",
            return_value=np.array([100.0, 0.0, 0.0]),
        ):
            prepared = g.prepare_geo_acquisition(
                self.uav_loc,
                self.uav_att,
                class_id=0,
                min_pixels=10.0,
            )
            repeated = g.prepare_geo_acquisition(
                self.uav_loc,
                self.uav_att,
                class_id=0,
                min_pixels=10.0,
            )

        self.assertTrue(prepared)
        self.assertFalse(repeated)
        self.mount.set_zoom.assert_called_once_with("2")

    def test_prepare_geo_acquisition_rejects_zoom_that_loses_frame(self):
        g = self._make(with_zoom=True)
        self.mount.camera = Mock()
        self.mount.camera._zoom_map = {
            "1": {"fx": 100.0, "fy": 100.0, "cx": 50.0, "cy": 50.0},
            "2": {"fx": 500.0, "fy": 500.0, "cx": 50.0, "cy": 50.0},
        }
        self.mount.get_zoom_levels.return_value = ["1", "2"]
        self.mount.get_k.return_value = np.array([
            [100.0, 0.0, 50.0],
            [0.0, 100.0, 50.0],
            [0.0, 0.0, 1.0],
        ])
        self.mount.get_gimbal_data.return_value = GimbalData(att=Attitude(0, 0, 0))
        self.mount.is_valid.return_value = False
        self.mount.set_zoom.return_value = True
        g.start_geo_tracking(self.poi_loc, self.geo_ref)

        with patch(
            "navpy.modules.navigation.gimbal_navigation_composition.pymap3d.geodetic2ned",
            return_value=np.array([100.0, 0.0, 0.0]),
        ):
            prepared = g.prepare_geo_acquisition(
                self.uav_loc, self.uav_att, class_id=0, min_pixels=10.0,
            )

        self.assertFalse(prepared)
        self.mount.set_zoom.assert_not_called()

    def test_prepare_geo_acquisition_rejects_nonfinite_class_size(self):
        with patch(
            "navpy.modules.navigation.gimbal_navigation_composition.get_class_detect_size",
            return_value=float("nan"),
        ):
            g = self._make(with_zoom=True)
        self._configure_acquisition()

        prepared = self._prepare_acquisition(g)

        self.assertFalse(prepared)
        self.mount.set_zoom.assert_not_called()

    def test_prepare_geo_acquisition_rejects_nonfinite_min_pixels(self):
        g = self._make(with_zoom=True)
        self._configure_acquisition()

        prepared = self._prepare_acquisition(g, min_pixels=float("nan"))

        self.assertFalse(prepared)
        self.mount.set_zoom.assert_not_called()

    def test_prepare_geo_acquisition_rejects_nonfinite_slant(self):
        g = self._make(with_zoom=True)
        self._configure_acquisition()

        prepared = self._prepare_acquisition(
            g,
            poi_ned=np.array([float("nan"), 0.0, 0.0]),
        )

        self.assertFalse(prepared)
        self.mount.set_zoom.assert_not_called()

    def test_prepare_geo_acquisition_uses_candidate_when_current_k_is_invalid(self):
        g = self._make(with_zoom=True)
        self._configure_acquisition(
            current_k=np.array([
                [100.0, 0.0, 50.0],
                [0.0, float("nan"), 50.0],
                [0.0, 0.0, 1.0],
            ]),
        )

        prepared = self._prepare_acquisition(g)

        self.assertTrue(prepared)
        self.mount.set_zoom.assert_called_once_with("2")

    def test_prepare_geo_acquisition_rejects_nonfinite_candidate_k(self):
        g = self._make(with_zoom=True)
        self._configure_acquisition(
            zoom_entry={
                "fx": float("nan"),
                "fy": 500.0,
                "cx": 50.0,
                "cy": 50.0,
            },
        )

        prepared = self._prepare_acquisition(g)

        self.assertFalse(prepared)
        self.assertEqual(self.geo_ref.uv_calls, [])
        self.mount.set_zoom.assert_not_called()

    def test_prepare_geo_acquisition_rejects_nonfinite_projected_size(self):
        with patch(
            "navpy.modules.navigation.gimbal_navigation_composition.get_class_detect_size",
            return_value=1e308,
        ):
            g = self._make(with_zoom=True)
        self._configure_acquisition(
            zoom_entry={
                "fx": 1e38,
                "fy": 1e38,
                "cx": 50.0,
                "cy": 50.0,
            },
        )

        prepared = self._prepare_acquisition(
            g,
            poi_ned=np.array([1.0, 0.0, 0.0]),
        )

        self.assertFalse(prepared)
        self.mount.set_zoom.assert_not_called()

    def test_prepare_geo_acquisition_rejects_nonfinite_projected_pixel(self):
        g = self._make(with_zoom=True)
        self._configure_acquisition()
        self.geo_ref.uv = (float("nan"), 50.0)

        prepared = self._prepare_acquisition(g)

        self.assertFalse(prepared)
        self.mount.is_valid.assert_not_called()
        self.mount.set_zoom.assert_not_called()

    def test_prepare_geo_acquisition_contains_operational_zoom_oserror(self):
        g = self._make(with_zoom=True)
        self._configure_acquisition()
        self.mount.set_zoom.side_effect = OSError("camera unavailable")

        prepared = self._prepare_acquisition(g)

        self.assertFalse(prepared)
        self.logger.warning.assert_called()

    def test_prepare_geo_acquisition_propagates_programmer_zoom_typeerror(self):
        g = self._make(with_zoom=True)
        self._configure_acquisition()
        self.mount.set_zoom.side_effect = TypeError("bad zoom command")

        with self.assertRaisesRegex(TypeError, "bad zoom command"):
            self._prepare_acquisition(g)

    def test_stop_geo_tracking_resets_prepared_zoom(self):
        g = self._make(with_zoom=True)
        self.mount.camera = Mock()
        self.mount.camera._zoom_map = {
            "1": {"fx": 100.0, "fy": 100.0, "cx": 50.0, "cy": 50.0},
            "2": {"fx": 500.0, "fy": 500.0, "cx": 50.0, "cy": 50.0},
        }
        self.mount.get_zoom_levels.return_value = ["1", "2"]
        self.mount.get_k.return_value = np.array([
            [100.0, 0.0, 50.0],
            [0.0, 100.0, 50.0],
            [0.0, 0.0, 1.0],
        ])
        self.mount.get_gimbal_data.return_value = GimbalData(att=Attitude(0, 0, 0))
        self.mount.is_valid.return_value = True
        self.mount.set_zoom.return_value = True
        g.start_geo_tracking(self.poi_loc, self.geo_ref)
        with patch(
            "navpy.modules.navigation.gimbal_navigation_composition.pymap3d.geodetic2ned",
            return_value=np.array([100.0, 0.0, 0.0]),
        ):
            g.prepare_geo_acquisition(
                self.uav_loc, self.uav_att, class_id=0, min_pixels=10.0,
            )
        self.mount.command_zoom.reset_mock()

        g.stop_geo_tracking()

        self.mount.command_zoom.assert_called_with("1")

    def test_update_geo_no_op_on_missing_inputs(self):
        g = self._make()
        g.start_geo_tracking(self.poi_loc, self.geo_ref)
        self.gimbal.reset_mock()

        g.update_geo(None, self.uav_att)
        g.update_geo(self.uav_loc, None)

        self.gimbal.set_att.assert_not_called()
        self.assertEqual(self.geo_ref.calls, [])

    def test_update_geo_contains_operational_pose_math_oserror(self):
        g = self._make()
        g.start_geo_tracking(self.poi_loc, self.geo_ref)
        self.gimbal.reset_mock()

        def _raise(*args, **kwargs):
            raise OSError("sensor unavailable")

        self.geo_ref.calc_gimbal_lock_att_loc = _raise

        g.update_geo(self.uav_loc, self.uav_att)

        self.gimbal.set_att.assert_not_called()
        self.logger.warning.assert_called()

    def test_update_geo_propagates_programmer_pose_math_errors(self):
        for error in (
            TypeError("bad pose input"),
            AttributeError("missing pose field"),
            ValueError("invalid pose value"),
        ):
            with self.subTest(error=type(error).__name__):
                g = self._make()
                bad_ref = _FakeGeoRef()
                bad_ref.calc_gimbal_lock_att_loc = Mock(side_effect=error)
                g.start_geo_tracking(self.poi_loc, bad_ref)
                self.gimbal.reset_mock()

                with self.assertRaisesRegex(type(error), str(error)):
                    g.update_geo(self.uav_loc, self.uav_att)

                self.gimbal.set_att.assert_not_called()

    def test_update_geo_rejects_nonfinite_commands_before_set_att(self):
        for pitch, yaw in (
            (float("nan"), 120.0),
            (-25.0, float("inf")),
        ):
            with self.subTest(pitch=pitch, yaw=yaw):
                g = self._make()
                geo_ref = _FakeGeoRef(pitch=pitch, yaw=yaw)
                g.start_geo_tracking(self.poi_loc, geo_ref)
                self.gimbal.reset_mock()

                g.update_geo(self.uav_loc, self.uav_att)

                self.gimbal.set_att.assert_not_called()

    def test_update_geo_contains_operational_set_att_oserror(self):
        g = self._make()
        g.start_geo_tracking(self.poi_loc, self.geo_ref)
        self.gimbal.reset_mock()
        self.gimbal.set_att.side_effect = OSError("gimbal unavailable")

        g.update_geo(self.uav_loc, self.uav_att)

        self.logger.warning.assert_called()

    def test_update_geo_propagates_programmer_set_att_typeerror(self):
        g = self._make()
        g.start_geo_tracking(self.poi_loc, self.geo_ref)
        self.gimbal.reset_mock()
        self.gimbal.set_att.side_effect = TypeError("bad attitude command")

        with self.assertRaisesRegex(TypeError, "bad attitude command"):
            g.update_geo(self.uav_loc, self.uav_att)

    def test_poi_swap_invalidates_in_flight_update(self):
        """Generation gate: a stop+start sandwich between snapshot and
        commit must prevent the stale set_att from landing on the new
        session.
        """
        g = self._make()
        g.start_geo_tracking(self.poi_loc, self.geo_ref)
        self.gimbal.reset_mock()

        # Inject the swap inside the pose-math call so it fires after
        # the generation snapshot but before the commit.
        new_poi = _FakeLocation(lat=41.0, lng=45.0, alt=1600.0)
        new_ref = _FakeGeoRef(pitch=10.0, yaw=10.0)

        original_calc = self.geo_ref.calc_gimbal_lock_att_loc

        def _swap_during_calc(uav_loc, poi_loc, uas_att, g_data):
            result = original_calc(uav_loc, poi_loc, uas_att, g_data)
            g.start_geo_tracking(new_poi, new_ref)
            return result

        self.geo_ref.calc_gimbal_lock_att_loc = _swap_during_calc

        g.update_geo(self.uav_loc, self.uav_att)

        # The stale set_att for the old session must NOT have landed.
        # The swap itself does not call set_att (no LOCK reassertion on
        # swap), so set_att should not have been called at all.
        self.gimbal.set_att.assert_not_called()

    # --- handoff to detection --------------------------------------------

    def test_start_tracking_while_geo_armed_clears_geo_without_neutralizing(self):
        g = self._make()
        g.start_geo_tracking(self.poi_loc, self.geo_ref)
        # set_motion_mode(LOCK) was called by start_geo_tracking. Reset
        # so we can observe the handoff cleanly.
        self.gimbal.reset_mock()

        g.start_tracking(7)

        self.assertFalse(g.is_geo_armed)
        self.assertTrue(g.is_detection_armed)
        # Detection mode reasserts LOCK (its existing behavior). FOLLOW +
        # neutral must NOT be issued during handoff — that is the whole
        # point of _clear_geo_state_locked vs stop_geo_tracking.
        for call in self.gimbal.set_motion_mode.call_args_list:
            self.assertEqual(call.args[0], MODE_LOCK)
        # set_att is the neutral-attitude command that return_to_neutral()
        # would issue. It must NOT be called during handoff; the gimbal
        # stays at its current attitude until the rate tracker takes over.
        self.gimbal.set_att.assert_not_called()

    def test_start_tracking_failure_restores_prior_geo_session(self):
        """A failed detection handoff must not destroy the prior geo
        session — the gimbal hardware never moved, so the geo session is
        recoverable.
        """
        g = self._make()
        g.start_geo_tracking(self.poi_loc, self.geo_ref)
        # Set up a hardware failure on the LOCK reassertion that
        # start_tracking does.
        self.gimbal.set_motion_mode.side_effect = RuntimeError("UDP drop")

        with self.assertRaises(RuntimeError):
            g.start_tracking(7)

        self.assertFalse(g.is_detection_armed)
        # Geo session must be restored.
        self.assertTrue(g.is_geo_armed)

    def test_start_geo_tracking_no_op_on_none_arguments(self):
        g = self._make()

        g.start_geo_tracking(None, self.geo_ref)
        g.start_geo_tracking(self.poi_loc, None)

        self.assertFalse(g.is_geo_armed)
        self.gimbal.set_motion_mode.assert_not_called()

    def test_geo_update_after_detection_armed_is_a_no_op(self):
        g = self._make()
        g.start_geo_tracking(self.poi_loc, self.geo_ref)
        g.start_tracking(7)
        self.gimbal.reset_mock()

        g.update_geo(self.uav_loc, self.uav_att)

        self.gimbal.set_att.assert_not_called()

    def test_geo_tracking_facade_reexports_exact_owner_symbols(self):
        from navpy.modules.navigation.gimbal_geo_acquisition import (
            GeoAcquisitionZoom as OwnedGeoAcquisitionZoom,
        )
        from navpy.modules.navigation.gimbal_geo_session import (
            GimbalGeoTracking as OwnedGimbalGeoTracking,
        )
        from navpy.modules.navigation.gimbal_geo_tracking import (
            GeoAcquisitionZoom,
            GeoZoomSelector,
            GimbalGeoTracking,
        )
        from navpy.modules.navigation.gimbal_geo_zoom_selector import (
            GeoZoomSelector as OwnedGeoZoomSelector,
        )

        self.assertIs(GeoAcquisitionZoom, OwnedGeoAcquisitionZoom)
        self.assertIs(GeoZoomSelector, OwnedGeoZoomSelector)
        self.assertIs(GimbalGeoTracking, OwnedGimbalGeoTracking)


if __name__ == "__main__":
    unittest.main()
