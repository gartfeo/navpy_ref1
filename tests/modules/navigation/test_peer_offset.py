"""Tests for peer_offset module — camera-based approach offset."""
import math
import unittest
from unittest.mock import Mock

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.approach_strategy import ApproachKind
from navpy.modules.navigation.peer_offset import (
    calc_peer_approach_offset,
    select_approach_mount,
)


import numpy as np


def _make_mount(pitch: float, max_detect_distance: float = 3000.0,
                fy: float = 2000.0, cy: float = 540.0, image_height: int = 1080,
                has_zoom: bool = True, zoom_ratio: float = None,
                base_fy: float = None) -> Mock:
    """Create a mock CameraMount with gimbal pitch and camera intrinsics.

    ``has_zoom`` defaults to True to preserve historical behavior of
    existing tests that assumed the detect-distance orbit branch.
    ``base_fy`` is the 1x focal length (defaults to ``fy``) and
    ``zoom_ratio`` the max/1x focal-length span (defaults to 10x for a zoom
    mount, 1x for a fixed one) — both consumed by the furthest-orbit sizing.
    Pass ``base_fy=None`` explicitly via ``Mock`` only to force the
    no-usable-focal-length fallback.
    """
    mount = Mock()
    gimbal_data = Mock()
    gimbal_data.att = Attitude(pitch=pitch, yaw=0, roll=0)
    gimbal_data.max_detect_distance = max_detect_distance
    mount.get_gimbal_data.return_value = gimbal_data
    mount.name = "test_mount"
    k = np.array([[fy, 0, image_height / 2], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    mount.get_k.return_value = k
    mount.image_height = image_height
    mount.has_zoom = has_zoom
    mount.zoom_ratio = zoom_ratio if zoom_ratio is not None else (10.0 if has_zoom else 1.0)
    mount.base_fy = base_fy if base_fy is not None else fy
    return mount


class TestSelectApproachMount(unittest.TestCase):

    def test_picks_least_negative_pitch(self):
        """Among -14° and -28°, picks -14° (most forward-looking)."""
        m14 = _make_mount(-14)
        m28 = _make_mount(-28)
        result = select_approach_mount([m14, m28])
        self.assertIs(result, m14)

    def test_picks_least_negative_reversed_order(self):
        """Order should not matter."""
        m14 = _make_mount(-14)
        m28 = _make_mount(-28)
        result = select_approach_mount([m28, m14])
        self.assertIs(result, m14)

    def test_skips_positive_pitch(self):
        """Mount at +4.8° is skipped; only -14° is selected."""
        m_pos = _make_mount(4.8)
        m_neg = _make_mount(-14)
        result = select_approach_mount([m_pos, m_neg])
        self.assertIs(result, m_neg)

    def test_all_upward_returns_none(self):
        """No suitable mount when all pitch >= 0."""
        m1 = _make_mount(0)
        m2 = _make_mount(10)
        result = select_approach_mount([m1, m2])
        self.assertIsNone(result)

    def test_empty_list_returns_none(self):
        result = select_approach_mount([])
        self.assertIsNone(result)

    def test_single_downward_mount(self):
        m = _make_mount(-45)
        result = select_approach_mount([m])
        self.assertIs(result, m)


class TestCalcPeerApproachOffset(unittest.TestCase):

    def test_valid_interval_returns_approach(self):
        """Steep camera with high fy returns a valid approach distance."""
        target = Location(lat=32.0, lng=34.0, alt=200)
        drone = Location(lat=32.1, lng=34.0, alt=200)
        mount = _make_mount(-30, fy=3000)

        plan = calc_peer_approach_offset(target, drone, [mount])

        self.assertGreater(plan.offset_distance, 0)
        self.assertGreater(target.distance_to(plan.approach_location), 50)

    def test_offset_direction_along_approach(self):
        """Offset point should be between drone and target."""
        target = Location(lat=32.0, lng=34.0, alt=200)
        drone = Location(lat=32.1, lng=34.0, alt=200)
        mount = _make_mount(-30, fy=3000)

        plan = calc_peer_approach_offset(target, drone, [mount])

        self.assertGreater(plan.approach_location.lat, target.lat)
        self.assertLess(plan.approach_location.lat, drone.lat)

    def test_no_interval_uses_half_max_slant(self):
        """When no valid interval, falls back to half max detection slant range."""
        target = Location(lat=32.0, lng=34.0, alt=5000)
        drone = Location(lat=32.1, lng=34.0, alt=5000)
        mount = _make_mount(-5)

        plan = calc_peer_approach_offset(target, drone, [mount])

        self.assertGreater(plan.offset_distance, 0)
        self.assertNotEqual(plan.approach_location.lat, target.lat)

    def test_no_mount_returns_target(self):
        """Fallback when no downward camera — returns original target."""
        target = Location(lat=32.0, lng=34.0, alt=250)
        drone = Location(lat=32.1, lng=34.0, alt=250)

        plan = calc_peer_approach_offset(target, drone, [])

        self.assertIs(plan.approach_location, target)
        self.assertEqual(plan.offset_distance, 0.0)

    def test_steeper_pitch_allows_closer_approach(self):
        """Steep pitch gives wider interval and closer approach."""
        target = Location(lat=32.0, lng=34.0, alt=200)
        drone = Location(lat=32.1, lng=34.0, alt=200)

        plan_moderate = calc_peer_approach_offset(target, drone, [_make_mount(-20, fy=3000)])
        plan_steep = calc_peer_approach_offset(target, drone, [_make_mount(-45, fy=3000)])

        if plan_moderate.offset_distance > 0 and plan_steep.offset_distance > 0:
            self.assertGreater(plan_moderate.offset_distance, plan_steep.offset_distance)

    def test_zero_altitude_returns_target(self):
        """Zero altitude → no offset possible."""
        target = Location(lat=32.0, lng=34.0, alt=0)
        drone = Location(lat=32.1, lng=34.0, alt=0)
        mount = _make_mount(-14)

        plan = calc_peer_approach_offset(target, drone, [mount])

        self.assertIs(plan.approach_location, target)
        self.assertEqual(plan.offset_distance, 0.0)

    def test_all_upward_mounts_returns_target(self):
        """All mounts looking up — fallback to target."""
        target = Location(lat=32.0, lng=34.0, alt=250)
        drone = Location(lat=32.1, lng=34.0, alt=250)
        m1 = _make_mount(0)
        m2 = _make_mount(10)

        plan = calc_peer_approach_offset(target, drone, [m1, m2])

        self.assertIs(plan.approach_location, target)
        self.assertEqual(plan.offset_distance, 0.0)

    def test_offset_preserves_target_alt(self):
        """Offset location keeps the same altitude as target."""
        target = Location(lat=32.0, lng=34.0, alt=300)
        drone = Location(lat=32.1, lng=34.0, alt=250)
        mount = _make_mount(-14)

        plan = calc_peer_approach_offset(target, drone, [mount])

        self.assertEqual(plan.approach_location.alt, target.alt)

    def test_orbit_returns_target_as_approach(self):
        """ORBIT places the approach point at the target itself."""
        target = Location(lat=32.0, lng=34.0, alt=200)
        drone = Location(lat=32.1, lng=34.0, alt=300)
        mount = _make_mount(-24, fy=2262)

        plan = calc_peer_approach_offset(
            target, drone, [mount], kind=ApproachKind.ORBIT,
        )

        self.assertEqual(plan.kind, ApproachKind.ORBIT)
        self.assertEqual(plan.approach_location.lat, target.lat)
        self.assertEqual(plan.approach_location.lng, target.lng)
        self.assertEqual(plan.offset_distance, 0.0)
        self.assertIsNotNone(plan.orbit_radius)
        self.assertGreater(plan.orbit_radius, 50)

    def test_orbit_radius_from_detect_pixels(self):
        """Orbit radius derived from fy, class_size, MIN_DETECT_PIXELS with 10% margin."""
        from navpy.modules.vision.vision_profiles import MIN_DETECT_PIXELS, get_class_detect_size

        target = Location(lat=32.0, lng=34.0, alt=200)
        drone = Location(lat=32.1, lng=34.0, alt=300)
        fy = 2262.0
        mount = _make_mount(-24, fy=fy)

        plan = calc_peer_approach_offset(
            target, drone, [mount], kind=ApproachKind.ORBIT,
        )

        alt = 100.0  # drone 300 - target 200
        class_size = get_class_detect_size(0)  # Class 0 diagonal = sqrt(3.5^2+2.5^2)
        detect_slant = fy * class_size / MIN_DETECT_PIXELS * 0.9
        expected_radius = math.sqrt(detect_slant ** 2 - alt ** 2)
        self.assertAlmostEqual(plan.orbit_radius, expected_radius, delta=1.0)

    def test_orbit_radius_independent_of_gimbal_pitch(self):
        """Different gimbal pitches produce the same orbit radius."""
        target = Location(lat=32.0, lng=34.0, alt=200)
        drone = Location(lat=32.1, lng=34.0, alt=300)

        plan_15 = calc_peer_approach_offset(
            target, drone, [_make_mount(-15, fy=2000)],
            kind=ApproachKind.ORBIT,
        )
        plan_45 = calc_peer_approach_offset(
            target, drone, [_make_mount(-45, fy=2000)],
            kind=ApproachKind.ORBIT,
        )

        self.assertAlmostEqual(plan_15.orbit_radius, plan_45.orbit_radius, places=1)

    def test_orbit_detection_possible_at_orbit_distance(self):
        """Target must have >= MIN_DETECT_PIXELS at the orbit slant range."""
        from navpy.modules.vision.vision_profiles import MIN_DETECT_PIXELS, get_class_detect_size

        target = Location(lat=32.0, lng=34.0, alt=200)
        drone = Location(lat=32.1, lng=34.0, alt=300)
        fy = 2262.0
        mount = _make_mount(-24, fy=fy)

        plan = calc_peer_approach_offset(
            target, drone, [mount], kind=ApproachKind.ORBIT,
        )

        alt = 100.0
        orbit_slant = math.sqrt(plan.orbit_radius ** 2 + alt ** 2)
        class_size = get_class_detect_size(0)  # Detection class 0
        pixels_at_orbit = fy * class_size / orbit_slant
        self.assertGreaterEqual(pixels_at_orbit, MIN_DETECT_PIXELS,
                                f"Target has {pixels_at_orbit:.1f}px at orbit, need {MIN_DETECT_PIXELS}")

    def test_orbit_large_fy_produces_far_orbit(self):
        """High focal length camera produces far orbit radius."""
        target = Location(lat=32.0, lng=34.0, alt=200)
        drone = Location(lat=32.1, lng=34.0, alt=400)  # alt diff = 200m

        plan = calc_peer_approach_offset(
            target, drone, [_make_mount(-15, fy=5000)],
            kind=ApproachKind.ORBIT,
        )

        self.assertGreater(plan.orbit_radius, 1000)


class TestOrbitBranchesByZoomAvailability(unittest.TestCase):
    """Orbit slant range must depend on whether the mount can zoom."""

    def test_zoom_mount_orbits_at_detect_slant(self):
        from navpy.modules.vision.vision_profiles import MIN_DETECT_PIXELS, get_class_detect_size

        target = Location(lat=32.0, lng=34.0, alt=200)
        drone = Location(lat=32.1, lng=34.0, alt=300)
        fy = 2262.0
        mount = _make_mount(-24, fy=fy, has_zoom=True)

        plan = calc_peer_approach_offset(
            target, drone, [mount], kind=ApproachKind.ORBIT,
        )

        alt = 100.0
        class_size = get_class_detect_size(0)  # Class 0 diagonal = sqrt(3.5^2+2.5^2)
        slant = fy * class_size / MIN_DETECT_PIXELS * 0.9
        expected = math.sqrt(slant ** 2 - alt ** 2)
        self.assertAlmostEqual(plan.orbit_radius, expected, delta=1.0)

    def test_fixed_mount_orbits_at_confirm_slant(self):
        """Without zoom, the drone must orbit closer — at the confirm slant."""
        from navpy.modules.vision.vision_profiles import MIN_CONFIRM_PIXELS, get_class_detect_size

        target = Location(lat=32.0, lng=34.0, alt=200)
        drone = Location(lat=32.1, lng=34.0, alt=300)
        fy = 2262.0
        mount = _make_mount(-24, fy=fy, has_zoom=False)

        plan = calc_peer_approach_offset(
            target, drone, [mount], kind=ApproachKind.ORBIT,
        )

        alt = 100.0
        # Fixed mount uses same get_class_detect_size(0) as zoom mount,
        # but with MIN_CONFIRM_PIXELS instead of MIN_DETECT_PIXELS.
        class_size = get_class_detect_size(0)  # Class 0 diagonal = sqrt(3.5^2+2.5^2)
        slant = fy * class_size / MIN_CONFIRM_PIXELS * 0.9
        expected = math.sqrt(slant ** 2 - alt ** 2)
        self.assertAlmostEqual(plan.orbit_radius, expected, delta=1.0)

    def test_fixed_mount_orbit_radius_smaller_than_zoom_mount(self):
        """Sanity: no-zoom orbit is strictly smaller than with-zoom orbit."""
        target = Location(lat=32.0, lng=34.0, alt=200)
        drone = Location(lat=32.1, lng=34.0, alt=300)
        fy = 2262.0
        zoom_mount = _make_mount(-24, fy=fy, has_zoom=True)
        fixed_mount = _make_mount(-24, fy=fy, has_zoom=False)

        zoom_plan = calc_peer_approach_offset(
            target, drone, [zoom_mount], kind=ApproachKind.ORBIT,
        )
        fixed_plan = calc_peer_approach_offset(
            target, drone, [fixed_mount], kind=ApproachKind.ORBIT,
        )

        self.assertLess(fixed_plan.orbit_radius, zoom_plan.orbit_radius)

    def test_zoom_orbit_furthest_recognition_binds(self):
        """Zoom mount + limits: orbit at the FURTHEST standoff bounded by
        max-zoom recognition (the binding constraint for a high-recognition
        target), well above the dive-feasibility floor."""
        from navpy.modules.navigation.orbit_geometry import (
            OrbitNavigationLimits, r_nav_min,
        )
        from navpy.modules.navigation.peer_offset import (
            MIN_APPROACH_STANDOFF_M, MIN_TRACK_PIXELS, _ORBIT_PIXEL_MARGIN,
        )
        from navpy.modules.vision.vision_profiles import get_class_detect_size

        target = Location(lat=32.0, lng=34.0, alt=200)
        drone = Location(lat=32.1, lng=34.0, alt=350)  # alt above target = 150
        fy, zoom_ratio, recog_px = 2262.0, 10.0, 150.0
        mount = _make_mount(-24, fy=fy, has_zoom=True, zoom_ratio=zoom_ratio)
        limits = OrbitNavigationLimits(25.0, 45.0, -40.0)

        plan = calc_peer_approach_offset(
            target, drone, [mount], kind=ApproachKind.ORBIT,
            orbit_limits=limits, recognition_px=recog_px,
        )

        size = get_class_detect_size(0)
        track = fy * size / MIN_TRACK_PIXELS
        recog = fy * zoom_ratio * size / recog_px
        self.assertLess(recog, track)  # recognition is the binding bound
        orbit_slant = min(track, recog) * _ORBIT_PIXEL_MARGIN
        floor = r_nav_min(limits, 150.0, MIN_APPROACH_STANDOFF_M)
        expected = max(math.sqrt(orbit_slant ** 2 - 150.0 ** 2), floor)
        self.assertAlmostEqual(plan.orbit_radius, expected, delta=1.0)
        self.assertGreater(plan.orbit_radius, floor)  # camera bound wins → far orbit

    def test_zoom_orbit_track_binds_for_low_recognition_demand(self):
        """With a small recognition demand the 1x tracking bound (12 px) is the
        binding (closer) constraint and the orbit sits there."""
        from navpy.modules.navigation.orbit_geometry import OrbitNavigationLimits
        from navpy.modules.navigation.peer_offset import (
            MIN_TRACK_PIXELS, _ORBIT_PIXEL_MARGIN,
        )
        from navpy.modules.vision.vision_profiles import get_class_detect_size

        target = Location(lat=32.0, lng=34.0, alt=200)
        drone = Location(lat=32.1, lng=34.0, alt=350)  # alt = 150
        fy, zoom_ratio, recog_px = 2262.0, 10.0, 20.0
        mount = _make_mount(-24, fy=fy, has_zoom=True, zoom_ratio=zoom_ratio)

        plan = calc_peer_approach_offset(
            target, drone, [mount], kind=ApproachKind.ORBIT,
            orbit_limits=OrbitNavigationLimits(25.0, 45.0, -40.0),
            recognition_px=recog_px,
        )

        size = get_class_detect_size(0)
        track = fy * size / MIN_TRACK_PIXELS
        recog = fy * zoom_ratio * size / recog_px
        self.assertLess(track, recog)  # tracking is the binding bound
        orbit_slant = track * _ORBIT_PIXEL_MARGIN
        expected = math.sqrt(orbit_slant ** 2 - 150.0 ** 2)
        self.assertAlmostEqual(plan.orbit_radius, expected, delta=1.0)

    def test_zoom_orbit_floored_by_dive_feasibility(self):
        """A very high recognition demand drives the camera bound below the
        dive-feasibility floor; the orbit is floored there (recognition no
        longer reachable at max zoom — logged)."""
        from navpy.modules.navigation.orbit_geometry import (
            OrbitNavigationLimits, r_nav_min,
        )
        from navpy.modules.navigation.peer_offset import MIN_APPROACH_STANDOFF_M

        target = Location(lat=32.0, lng=34.0, alt=200)
        drone = Location(lat=32.1, lng=34.0, alt=350)  # alt = 150
        mount = _make_mount(-24, fy=2262.0, has_zoom=True, zoom_ratio=10.0)
        limits = OrbitNavigationLimits(25.0, 45.0, -40.0)

        plan = calc_peer_approach_offset(
            target, drone, [mount], kind=ApproachKind.ORBIT,
            orbit_limits=limits, recognition_px=500.0,
        )

        floor = r_nav_min(limits, 150.0, MIN_APPROACH_STANDOFF_M)
        self.assertAlmostEqual(plan.orbit_radius, floor, delta=1.0)

    def test_zoom_orbit_falls_back_to_floor_when_focal_unreadable(self):
        """No usable focal length (no base_fy, get_k raises) → the orbit uses
        the dive-feasibility floor rather than crashing."""
        from navpy.modules.navigation.orbit_geometry import (
            OrbitNavigationLimits, r_nav_min,
        )
        from navpy.modules.navigation.peer_offset import MIN_APPROACH_STANDOFF_M

        target = Location(lat=32.0, lng=34.0, alt=200)
        drone = Location(lat=32.1, lng=34.0, alt=350)  # alt = 150
        mount = _make_mount(-24, has_zoom=True)
        mount.base_fy = None
        mount.get_k.side_effect = RuntimeError("camera readback failed")
        limits = OrbitNavigationLimits(25.0, 45.0, -40.0)

        plan = calc_peer_approach_offset(
            target, drone, [mount], kind=ApproachKind.ORBIT,
            orbit_limits=limits,
        )

        floor = r_nav_min(limits, 150.0, MIN_APPROACH_STANDOFF_M)
        self.assertAlmostEqual(plan.orbit_radius, floor, delta=1.0)

    def test_zoom_orbit_warns_when_floor_breaks_recognition(self):
        """When the dive floor pushes the orbit past the max-zoom recognition
        range, a floor-violation warning names the recognition bound."""
        from navpy.modules.navigation.orbit_geometry import OrbitNavigationLimits

        target = Location(lat=32.0, lng=34.0, alt=200)
        drone = Location(lat=32.1, lng=34.0, alt=350)  # alt = 150
        mount = _make_mount(-24, fy=2262.0, has_zoom=True, zoom_ratio=10.0)

        with self.assertLogs(
                "navpy.modules.navigation.peer_offset", level="WARNING") as cm:
            calc_peer_approach_offset(
                target, drone, [mount], kind=ApproachKind.ORBIT,
                orbit_limits=OrbitNavigationLimits(25.0, 45.0, -40.0),
                recognition_px=500.0,
            )

        self.assertTrue(
            any("recognition NOT reachable" in m for m in cm.output),
            f"expected a recognition floor-violation warning, got: {cm.output}",
        )

    def test_no_zoom_mount_ignores_orbit_limits(self):
        """A fixed-FOV mount keeps camera-confirm-range sizing even when
        orbit_limits is supplied (it can't zoom to recover recognition)."""
        from navpy.modules.navigation.orbit_geometry import OrbitNavigationLimits
        from navpy.modules.vision.vision_profiles import MIN_CONFIRM_PIXELS, get_class_detect_size

        target = Location(lat=32.0, lng=34.0, alt=200)
        drone = Location(lat=32.1, lng=34.0, alt=300)  # alt = 100
        fy = 2262.0
        mount = _make_mount(-24, fy=fy, has_zoom=False)

        plan = calc_peer_approach_offset(
            target, drone, [mount], kind=ApproachKind.ORBIT,
            orbit_limits=OrbitNavigationLimits(25.0, 45.0, -40.0),
        )

        slant = fy * get_class_detect_size(0) / MIN_CONFIRM_PIXELS * 0.9
        expected = math.sqrt(slant ** 2 - 100.0 ** 2)
        self.assertAlmostEqual(plan.orbit_radius, expected, delta=1.0)


if __name__ == '__main__':
    unittest.main()
