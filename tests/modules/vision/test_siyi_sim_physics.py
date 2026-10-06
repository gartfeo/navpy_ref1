"""Deterministic tests for GimbalPhysics — no threads, no sleeps."""

import unittest

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.peripheral.siyi.sim.gimbal_physics import (
    GimbalPhysics,
    MAX_SLEW_RATE,
    MODE_FOLLOW,
    MODE_FPV,
    MODE_LOCK,
    ZOOM_INCREMENTAL_SPEED,
    ZR10,
    _wrap180,
)


class TestAngleLimits(unittest.TestCase):
    """Angles must clamp to ZR10 hardware limits."""

    def test_yaw_clamps_at_max(self):
        p = GimbalPhysics()
        p.set_motion_mode(MODE_FOLLOW)  # yaw limits apply in FOLLOW
        p.set_speed(100, 0)
        # 2 seconds at max rate → 180°, but limit is 135°
        p.update(2.0)
        self.assertAlmostEqual(p.yaw, ZR10.MAX_YAW_DEG)

    def test_yaw_clamps_at_min(self):
        p = GimbalPhysics()
        p.set_motion_mode(MODE_FOLLOW)  # yaw limits apply in FOLLOW
        p.set_speed(-100, 0)
        p.update(2.0)
        self.assertAlmostEqual(p.yaw, ZR10.MIN_YAW_DEG)

    def test_pitch_clamps_at_max(self):
        p = GimbalPhysics()
        p.set_speed(0, 100)
        p.update(1.0)
        self.assertAlmostEqual(p.pitch, ZR10.MAX_PITCH_DEG)

    def test_pitch_clamps_at_min(self):
        p = GimbalPhysics()
        p.set_speed(0, -100)
        p.update(2.0)
        self.assertAlmostEqual(p.pitch, ZR10.MIN_PITCH_DEG)


class TestSlewRate(unittest.TestCase):
    """Command units [-100, 100] map linearly to physical slew rate."""

    def test_full_speed_matches_max_slew_rate(self):
        p = GimbalPhysics()
        p.set_speed(100, 0)
        p.update(1.0)
        self.assertAlmostEqual(p.yaw, MAX_SLEW_RATE)

    def test_half_speed_is_half_rate(self):
        p = GimbalPhysics()
        p.set_speed(50, 0)
        p.update(1.0)
        self.assertAlmostEqual(p.yaw, MAX_SLEW_RATE / 2)

    def test_negative_speed_reverses_direction(self):
        p = GimbalPhysics()
        p.set_speed(-50, 0)
        p.update(1.0)
        self.assertAlmostEqual(p.yaw, -MAX_SLEW_RATE / 2)

    def test_pitch_rate(self):
        p = GimbalPhysics()
        p.set_speed(0, -50)
        p.update(0.5)
        expected = -(MAX_SLEW_RATE / 2) * 0.5
        self.assertAlmostEqual(p.pitch, expected)

    def test_speed_records_angular_rate(self):
        p = GimbalPhysics()
        p.set_speed(60, -40)
        p.update(0.1)
        self.assertAlmostEqual(p.yaw_speed, 0.6 * MAX_SLEW_RATE)
        self.assertAlmostEqual(p.pitch_speed, -0.4 * MAX_SLEW_RATE)

    def test_speed_is_zero_when_clamped_at_limit(self):
        """At a physical stop, achieved rate must be 0 even if commanded."""
        p = GimbalPhysics(initial_yaw=ZR10.MAX_YAW_DEG)
        p.set_motion_mode(MODE_FOLLOW)  # yaw limits apply in FOLLOW
        p.set_speed(100, 0)
        p.update(0.1)
        self.assertAlmostEqual(p.yaw, ZR10.MAX_YAW_DEG)
        self.assertAlmostEqual(p.yaw_speed, 0.0)

    def test_command_clamped_above_100(self):
        p = GimbalPhysics()
        p.set_speed(200, 0)
        p.update(1.0)
        self.assertAlmostEqual(p.yaw, MAX_SLEW_RATE)

    def test_command_clamped_below_minus_100(self):
        p = GimbalPhysics()
        p.set_speed(-200, 0)
        p.update(1.0)
        self.assertAlmostEqual(p.yaw, -MAX_SLEW_RATE)


class TestAngleSeek(unittest.TestCase):
    """Absolute angle commands slew toward the target at the physical bound."""

    def test_target_angle_uses_full_bounded_slew_without_proportional_lag(self):
        p = GimbalPhysics()
        p.set_target_angles(45.0, -30.0)

        p.update(0.02)

        self.assertAlmostEqual(p.yaw, 1.8)
        self.assertAlmostEqual(p.pitch, -1.8)
        self.assertAlmostEqual(p.yaw_speed, MAX_SLEW_RATE)
        self.assertAlmostEqual(p.pitch_speed, -MAX_SLEW_RATE)

    def test_target_angle_does_not_overshoot_within_one_physics_step(self):
        p = GimbalPhysics()
        p.set_target_angles(1.0, -0.5)

        p.update(0.02)

        self.assertAlmostEqual(p.yaw, 1.0)
        self.assertAlmostEqual(p.pitch, -0.5)
        self.assertAlmostEqual(p.yaw_speed, 50.0)
        self.assertAlmostEqual(p.pitch_speed, -25.0)

    def test_center_converges(self):
        p = GimbalPhysics(initial_yaw=50.0, initial_pitch=-20.0)
        p.center()
        # Step many small increments to let the P-controller converge
        for _ in range(200):
            p.update(0.02)
        self.assertAlmostEqual(p.yaw, 0.0, places=1)
        self.assertAlmostEqual(p.pitch, 0.0, places=1)

    def test_set_target_angles_converges(self):
        p = GimbalPhysics()
        p.set_target_angles(45.0, -30.0)
        for _ in range(200):
            p.update(0.02)
        self.assertAlmostEqual(p.yaw, 45.0, places=1)
        self.assertAlmostEqual(p.pitch, -30.0, places=1)

    def test_target_beyond_body_stop_clamps(self):
        """A world target past the ±135° body stop (vehicle level) clamps to
        the stop. 200° wraps to -160° (the short-path side), so yaw settles at
        the -135° stop; pitch -100° clamps to the -90° stop."""
        p = GimbalPhysics()  # LOCK, vehicle level
        p.set_target_angles(200.0, -100.0)
        for _ in range(200):
            p.update(0.02)
        self.assertAlmostEqual(p.yaw, ZR10.MIN_YAW_DEG, places=1)
        self.assertAlmostEqual(p.pitch, ZR10.MIN_PITCH_DEG, places=1)

    def test_target_clears_on_arrival(self):
        p = GimbalPhysics()
        p.set_target_angles(1.0, 1.0)
        for _ in range(200):
            p.update(0.02)
        # After arrival, target should be cleared
        angular = p.snapshot().angular
        self.assertIsNone(angular.target_yaw)
        self.assertIsNone(angular.target_pitch)


class TestCommandArbitration(unittest.TestCase):
    """set_speed cancels angle targets; set_target_angles cancels rate mode."""

    def test_set_speed_cancels_angle_target(self):
        p = GimbalPhysics()
        p.set_target_angles(90.0, -45.0)
        self.assertIsNotNone(p.snapshot().angular.target_yaw)

        p.set_speed(50, 0)
        self.assertIsNone(p.snapshot().angular.target_yaw)
        self.assertIsNone(p.snapshot().angular.target_pitch)

    def test_set_target_angles_overrides_rate(self):
        p = GimbalPhysics()
        p.set_speed(100, -100)
        p.update(0.5)  # accumulate some yaw
        prev_yaw = p.yaw

        p.set_target_angles(0.0, 0.0)
        angular = p.snapshot().angular
        self.assertEqual(angular.yaw_command, 0.0)
        self.assertEqual(angular.pitch_command, 0.0)
        self.assertIsNotNone(angular.target_yaw)

    def test_mode_change_cancels_pending_angle_seek(self):
        """A motion-mode change cancels a pending angle-seek target (it was
        expressed in the previous frame) instead of chasing a stale value."""
        p = GimbalPhysics()  # LOCK
        p.set_target_angles(120.0, -20.0)
        self.assertIsNotNone(p.snapshot().angular.target_yaw)
        p.set_motion_mode(MODE_FOLLOW)
        angular = p.snapshot().angular
        self.assertIsNone(angular.target_yaw)
        self.assertIsNone(angular.target_pitch)
        self.assertEqual(angular.yaw_command, 0.0)


class TestZoomAbsolute(unittest.TestCase):
    """Absolute zoom seeks to target level."""

    def test_zoom_reaches_target(self):
        p = GimbalPhysics()
        p.set_target_zoom(5.0)
        for _ in range(100):
            p.update(0.02)
        self.assertAlmostEqual(p.zoom_level, 5.0, places=1)

    def test_zoom_clamps_at_max(self):
        p = GimbalPhysics()
        p.set_target_zoom(50.0)  # above max → clamped to 30.0
        # 30 - 1 = 29 levels at 5/sec → need ~6s. 350 × 0.02 = 7s.
        for _ in range(350):
            p.update(0.02)
        self.assertAlmostEqual(p.zoom_level, ZR10.MAX_ZOOM, places=1)

    def test_zoom_clamps_at_min(self):
        p = GimbalPhysics(initial_zoom=5.0)
        p.set_target_zoom(0.5)  # below min
        for _ in range(200):
            p.update(0.02)
        self.assertAlmostEqual(p.zoom_level, 1.0, places=1)


class TestZoomIncremental(unittest.TestCase):
    """zoom_in/zoom_out/stop_zoom incremental lifecycle."""

    def test_zoom_in_increases_level(self):
        p = GimbalPhysics()
        p.start_zoom_in()
        p.update(1.0)
        self.assertAlmostEqual(p.zoom_level, 1.0 + ZOOM_INCREMENTAL_SPEED)

    def test_zoom_out_decreases_level(self):
        p = GimbalPhysics(initial_zoom=10.0)
        p.start_zoom_out()
        p.update(1.0)
        self.assertAlmostEqual(p.zoom_level, 10.0 - ZOOM_INCREMENTAL_SPEED)

    def test_zoom_out_clamps_at_min(self):
        p = GimbalPhysics()
        p.start_zoom_out()
        p.update(1.0)
        self.assertAlmostEqual(p.zoom_level, 1.0)

    def test_stop_zoom_halts_motion(self):
        p = GimbalPhysics()
        p.start_zoom_in()
        p.update(0.5)
        level_before = p.zoom_level
        p.stop_zoom()
        p.update(0.5)
        self.assertAlmostEqual(p.zoom_level, level_before)

    def test_get_zoom_during_ramp(self):
        p = GimbalPhysics()
        p.start_zoom_in()
        p.update(0.2)
        self.assertGreater(p.zoom_level, 1.0)
        self.assertLess(p.zoom_level, 1.0 + ZOOM_INCREMENTAL_SPEED)


class TestZoomArbitration(unittest.TestCase):
    """Absolute zoom cancels incremental and vice versa."""

    def test_absolute_cancels_incremental(self):
        p = GimbalPhysics()
        p.start_zoom_in()
        self.assertEqual(p.snapshot().zoom.direction, 1)
        p.set_target_zoom(5.0)
        zoom = p.snapshot().zoom
        self.assertEqual(zoom.direction, 0)
        self.assertIsNotNone(zoom.target)

    def test_incremental_cancels_absolute(self):
        p = GimbalPhysics()
        p.set_target_zoom(5.0)
        self.assertIsNotNone(p.snapshot().zoom.target)
        p.start_zoom_out()
        zoom = p.snapshot().zoom
        self.assertIsNone(zoom.target)
        self.assertEqual(zoom.direction, -1)


class TestFixedPhysicalTime(unittest.TestCase):
    """Physics consumes only explicit dt; speedup belongs to cadence."""

    def test_equal_explicit_dt_produces_equal_motion(self):
        p1 = GimbalPhysics()
        p2 = GimbalPhysics()
        p1.set_speed(100, 0)
        p2.set_speed(100, 0)
        p1.update(1.0)
        p2.update(1.0)
        self.assertAlmostEqual(p1.yaw, p2.yaw)

    def test_repeated_explicit_dt_accumulates_linearly(self):
        p = GimbalPhysics()
        p.set_speed(50, 0)
        p.update(1.0)
        yaw_after_1s = p.yaw
        p.update(1.0)
        self.assertAlmostEqual(p.yaw, yaw_after_1s * 2, places=1)


class TestStabilizationLockMode(unittest.TestCase):
    """LOCK mode: world-frame angles stay constant unless body stops hit."""

    def test_world_angles_unchanged_by_small_vehicle_motion(self):
        p = GimbalPhysics(initial_yaw=30.0, initial_pitch=-15.0)
        p.set_motion_mode(MODE_LOCK)
        # Small vehicle rotation — body frame well within limits
        p.set_vehicle_attitude(Attitude(10.0, 20.0, 5.0))
        p.update(0.1)
        self.assertAlmostEqual(p.yaw, 30.0)
        self.assertAlmostEqual(p.pitch, -15.0)

    def test_lock_mode_yaw_body_stop_drifts_world(self):
        """LOCK: when vehicle heading pushes the body yaw past the ±135° stop,
        the world yaw drifts to hold the mechanical limit."""
        p = GimbalPhysics(initial_yaw=100.0)
        p.set_motion_mode(MODE_LOCK)

        # Vehicle yaws -50°: body = 100 - (-50) = 150° > 135° → clamp to 135°
        p.set_vehicle_attitude(Attitude(0.0, -50.0, 0.0))
        p.update(0.01)

        # body held at +135° stop → world = 135 + (-50) = 85°
        self.assertAlmostEqual(p.yaw, 85.0, places=1)

    def test_body_frame_pitch_stop_clamps(self):
        p = GimbalPhysics(initial_pitch=-80.0)
        p.set_motion_mode(MODE_LOCK)

        # Vehicle pitches up 20°: body = -80 - 20 = -100° < -90° limit
        p.set_vehicle_attitude(Attitude(20.0, 0.0, 0.0))
        p.update(0.01)

        # Body clamped to -90°, so world = -90 + 20 = -70°
        self.assertAlmostEqual(p.pitch, -70.0, places=1)

    def test_lock_mode_no_yaw_drift_within_body_limits(self):
        """LOCK: vehicle rotation does not shift world yaw while the body yaw
        stays within the ±135° stop."""
        p = GimbalPhysics(initial_yaw=130.0)
        p.set_motion_mode(MODE_LOCK)
        # Vehicle yaws +10°: body = 130 - 10 = 120° < 135° → no clamp
        p.set_vehicle_attitude(Attitude(0.0, 10.0, 0.0))
        p.update(0.02)
        self.assertAlmostEqual(p.yaw, 130.0, places=1)
        self.assertAlmostEqual(p.yaw_speed, 0.0)

    def test_lock_mode_yaw_body_clamped_to_135(self):
        """LOCK: body-frame yaw cannot exceed the ±135° mechanical stop; the
        world yaw drifts so the body holds at the stop."""
        p = GimbalPhysics(initial_yaw=135.0)
        p.set_motion_mode(MODE_LOCK)

        # Vehicle yaws -15°: body = 135 - (-15) = 150° > 135° → clamp to 135°
        p.set_vehicle_attitude(Attitude(0.0, -15.0, 0.0))
        p.update(0.01)

        # body held at the +135° stop; world drifts to 135 + (-15) = 120°
        self.assertAlmostEqual(p.yaw - (-15.0), ZR10.MAX_YAW_DEG, places=1)
        self.assertAlmostEqual(p.yaw, 120.0, places=1)

    def test_lock_world_target_reachable_via_vehicle_heading(self):
        """LOCK: a world bearing past ±135° is reachable when the vehicle
        heading keeps body yaw within the stop — the orbit case. The old
        world-frame clamp wrongly pinned this near 135°-of-north."""
        p = GimbalPhysics()
        p.set_motion_mode(MODE_LOCK)
        # Heading 90°; world target 170° → body = 80° (well within ±135)
        p.set_vehicle_attitude(Attitude(0.0, 90.0, 0.0))
        p.set_target_angles(170.0, 0.0)
        for _ in range(300):
            p.update(0.02)
        self.assertAlmostEqual(p.yaw, 170.0, places=0)

    def test_lock_yaw_short_path_across_180(self):
        """LOCK: seeking a target across the ±180° seam slews the short way and
        converges, instead of swinging the long way around."""
        p = GimbalPhysics(initial_yaw=170.0)
        p.set_motion_mode(MODE_LOCK)
        # Heading 90° keeps both 170° and -170° within the body stop.
        p.set_vehicle_attitude(Attitude(0.0, 90.0, 0.0))
        p.set_target_angles(-170.0, 0.0)  # short path is +20° through 180°
        p.update(0.05)
        # Moved in the +yaw direction (toward the seam), not back toward 0.
        self.assertGreater(p.yaw, 170.0)
        self.assertLessEqual(abs(p.yaw_speed), MAX_SLEW_RATE + 0.5)
        for _ in range(300):
            p.update(0.02)
        self.assertAlmostEqual(p.yaw, -170.0, places=0)

    def test_lock_yaw_speed_no_spike_at_180_crossing(self):
        """LOCK: yaw_speed uses a wrapped delta, so crossing the ±180° seam in
        one step does not report a false ~360°/s spike."""
        p = GimbalPhysics(initial_yaw=178.0)
        p.set_motion_mode(MODE_LOCK)
        p.set_vehicle_attitude(Attitude(0.0, 90.0, 0.0))
        p.set_target_angles(-178.0, 0.0)  # short path +4° through the seam
        p.update(0.2)  # large enough to cross the seam in one step
        self.assertLess(abs(p.yaw_speed), MAX_SLEW_RATE + 0.5)

    def test_lock_tracks_through_full_orbit_without_clamp(self):
        """LOCK: as the vehicle heading sweeps 360° (orbit), the gimbal holds a
        side target (body ~±90°) without ever saturating at the yaw stop —
        directly reproduces the peer-orbit scenario."""
        p = GimbalPhysics()
        p.set_motion_mode(MODE_LOCK)
        max_body = 0.0
        for deg in range(0, 360, 5):
            heading = float(deg)
            world_target = _wrap180(heading + 90.0)  # side target on the orbit
            p.set_vehicle_attitude(Attitude(0.0, heading, 0.0))
            p.set_target_angles(world_target, 0.0)
            for _ in range(20):
                p.update(0.05)
            max_body = max(max_body, abs(_wrap180(p.yaw - heading)))
        # Body yaw stays near 90° (the side target) — never pinned at 135°.
        self.assertLess(max_body, 135.0)
        self.assertAlmostEqual(max_body, 90.0, delta=5.0)


class TestStabilizationFollowMode(unittest.TestCase):
    """FOLLOW: yaw is body-relative, pitch is stabilized with body stops."""

    def test_follow_yaw_clamped_directly(self):
        p = GimbalPhysics()
        p.set_motion_mode(MODE_FOLLOW)
        p.set_speed(100, 0)
        p.update(2.0)
        # Yaw is body-relative in FOLLOW, clamped directly
        self.assertAlmostEqual(p.yaw, ZR10.MAX_YAW_DEG)

    def test_follow_pitch_body_stop(self):
        p = GimbalPhysics(initial_pitch=-80.0)
        p.set_motion_mode(MODE_FOLLOW)
        # Vehicle pitches up 20°: body_pitch = -80 - 20 = -100° < -90°
        p.set_vehicle_attitude(Attitude(20.0, 0.0, 0.0))
        p.update(0.01)
        # Body clamped to -90°, world = -90 + 20 = -70°
        self.assertAlmostEqual(p.pitch, -70.0, places=1)

    def test_follow_target_clamps_raw_not_wrapped(self):
        """FOLLOW yaw is body-frame: an out-of-range target clamps to the
        mechanical stop directly (+200° → +135°), not wrapped to -160° then
        -135°."""
        p = GimbalPhysics()
        p.set_motion_mode(MODE_FOLLOW)
        p.set_target_angles(200.0, 0.0)
        self.assertAlmostEqual(
            p.snapshot().angular.target_yaw,
            ZR10.MAX_YAW_DEG,
            places=1,
        )
        p.set_target_angles(-200.0, 0.0)
        self.assertAlmostEqual(
            p.snapshot().angular.target_yaw,
            ZR10.MIN_YAW_DEG,
            places=1,
        )

    def test_follow_yaw_speed_sign_for_large_body_move(self):
        """FOLLOW yaw_speed uses the raw (body-frame) delta, so a large move
        to the stop reports the correct (negative) sign — no ±180° wrap, which
        would flip it positive."""
        p = GimbalPhysics(initial_yaw=130.0)
        p.set_motion_mode(MODE_FOLLOW)
        p.set_speed(-100, 0)  # drive toward the -135° stop
        p.update(4.0)
        self.assertAlmostEqual(p.yaw, ZR10.MIN_YAW_DEG, places=1)
        self.assertLess(p.yaw_speed, 0.0)

    def test_follow_seek_uses_raw_error_not_wrapped(self):
        """FOLLOW yaw is body-frame (no ±180° seam): seeking +135° → -135°
        drives the long way through 0 to the correct stop, instead of taking
        the wrapped short path into the wrong (+135°) stop and stalling."""
        p = GimbalPhysics(initial_yaw=135.0)
        p.set_motion_mode(MODE_FOLLOW)
        p.set_target_angles(-135.0, 0.0)
        for _ in range(100):
            p.update(0.1)
        self.assertAlmostEqual(p.yaw, -135.0, places=1)

    def test_follow_seek_reverse_direction(self):
        """FOLLOW: the symmetric case -135° → +135° also converges."""
        p = GimbalPhysics(initial_yaw=-135.0)
        p.set_motion_mode(MODE_FOLLOW)
        p.set_target_angles(135.0, 0.0)
        for _ in range(100):
            p.update(0.1)
        self.assertAlmostEqual(p.yaw, 135.0, places=1)


class TestStabilizationFPVMode(unittest.TestCase):
    """FPV: yaw follows aircraft, pitch stabilized, roll follows aircraft."""

    def test_fpv_yaw_clamped_directly(self):
        """Yaw follows aircraft → body-frame, clamped directly."""
        p = GimbalPhysics()
        p.set_motion_mode(MODE_FPV)
        p.set_speed(100, 0)
        p.update(2.0)
        self.assertAlmostEqual(p.yaw, ZR10.MAX_YAW_DEG)

    def test_fpv_pitch_stabilized_with_body_stop(self):
        """Pitch is stabilized in FPV — body-frame stop applies."""
        p = GimbalPhysics(initial_pitch=-80.0)
        p.set_motion_mode(MODE_FPV)
        # Vehicle pitches up 20°: body = -80 - 20 = -100° < -90° stop
        p.set_vehicle_attitude(Attitude(20.0, 0.0, 0.0))
        p.update(0.01)
        # Body clamped to -90°, world = -90 + 20 = -70°
        self.assertAlmostEqual(p.pitch, -70.0, places=1)

    def test_fpv_yaw_unaffected_by_vehicle_yaw(self):
        """Yaw is body-frame in FPV, vehicle yaw doesn't shift limits."""
        p = GimbalPhysics(initial_yaw=30.0)
        p.set_motion_mode(MODE_FPV)
        p.set_vehicle_attitude(Attitude(0.0, 50.0, 0.0))
        p.update(0.01)
        # Yaw stays at 30 — clamped directly, not offset by vehicle
        self.assertAlmostEqual(p.yaw, 30.0)


class TestZeroDt(unittest.TestCase):
    """update(0) and update(negative) should be no-ops."""

    def test_zero_dt_no_change(self):
        p = GimbalPhysics()
        p.set_speed(100, 100)
        p.update(0.0)
        self.assertEqual(p.yaw, 0.0)
        self.assertEqual(p.pitch, 0.0)

    def test_negative_dt_no_change(self):
        p = GimbalPhysics()
        p.set_speed(100, 100)
        p.update(-1.0)
        self.assertEqual(p.yaw, 0.0)
        self.assertEqual(p.pitch, 0.0)


if __name__ == "__main__":
    unittest.main()
