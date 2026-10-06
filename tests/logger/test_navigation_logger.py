import math
import unittest

import pymap3d

from navpy.logger.navigation_event_recorder import NavigationEventRecorder
from navpy.logger.navigation_log_streams import NavigationLogStreams
from navpy.logger.navigation_snap import (
    ClosestSnap,
    closest_horizontal_on_segment,
    closest_on_segment,
    closest_point_components_on_segment,
)
from navpy.logger.navigation_snap_session import NavigationSnapSession
from navpy.modules.common.models.location import Location


class _MemoryStream:
    def write(self, text: str) -> None:
        pass

    def flush(self) -> None:
        pass

    def close(self) -> None:
        pass


def _snap_session() -> NavigationSnapSession:
    events = NavigationEventRecorder(
        NavigationLogStreams(_MemoryStream(), _MemoryStream()),
        lambda: "00:00:00.000",
        64,
    )
    return NavigationSnapSession(events)


def _loc_from_ned(
    reference: Location,
    north: float,
    east: float,
    down: float,
) -> Location:
    lat, lng, alt = pymap3d.ned2geodetic(
        north,
        east,
        down,
        reference.lat,
        reference.lng,
        reference.alt,
    )
    return Location(lat, lng, alt, is_absolute=True)


class TestClosestHorizontalProjection(unittest.TestCase):
    def test_horizontal_projection_differs_from_3d(self):
        poi = Location(40.0, 44.0, 100.0, is_absolute=True)
        start = Location(40.0001, 44.0, 150.0, is_absolute=True)
        end = Location(39.9999, 44.0, 100.0, is_absolute=True)

        _, horizontal_3d, _, _, _ = closest_on_segment(start, end, poi)
        horizontal_min, _ = closest_horizontal_on_segment(start, end, poi)

        self.assertLessEqual(horizontal_min, horizontal_3d + 0.1)

    def test_horizontal_min_at_segment_midpoint(self):
        poi = Location(40.0, 44.0, 100.0, is_absolute=True)
        start = Location(40.001, 43.999, 150.0, is_absolute=True)
        end = Location(40.001, 44.001, 120.0, is_absolute=True)

        horizontal_min, _ = closest_horizontal_on_segment(
            start,
            end,
            poi,
        )

        self.assertGreater(horizontal_min, 50)
        self.assertLess(horizontal_min, 200)

    def test_snap_str_shows_h_min_only_when_different(self):
        same = ClosestSnap(
            dist=5.0,
            h_dist=3.0,
            v_dist=4.0,
            h_min=3.0,
            v_at_h_min=4.0,
        )
        different = ClosestSnap(
            dist=5.0,
            h_dist=3.0,
            v_dist=4.0,
            h_min=1.0,
            v_at_h_min=10.0,
        )

        self.assertNotIn("h_min", str(same))
        self.assertIn("h_min=1.0", str(different))

    def test_snap_status_shows_h_min_only_when_different(self):
        same = ClosestSnap(
            dist=5.0,
            h_dist=3.0,
            v_dist=4.0,
            h_min=3.0,
            v_at_h_min=4.0,
        )
        different = ClosestSnap(
            dist=5.0,
            h_dist=3.0,
            v_dist=4.0,
            h_min=1.0,
            v_at_h_min=10.0,
        )

        self.assertEqual(same.status().count("["), 1)
        self.assertIn("1.0m", different.status())

    def test_3d_closest_is_uav_to_poi(self):
        poi = Location(40.0, 44.0, 100.0, is_absolute=True)
        start = Location(40.001, 44.0, 100.0, is_absolute=True)
        end = Location(39.999, 44.0, 100.0, is_absolute=True)

        closest, h_dist, v_dist, slant, _ = closest_on_segment(
            start,
            end,
            poi,
        )

        self.assertLess(slant, 5.0)
        self.assertLess(h_dist, 5.0)
        self.assertLess(v_dist, 1.0)
        self.assertAlmostEqual(closest.lat, poi.lat, places=4)
        self.assertAlmostEqual(closest.lng, poi.lng, places=4)

    def test_snap_session_updates_without_primary_log(self):
        session = _snap_session()
        current = Location(40.0001, 44.0, 100.0, is_absolute=True)
        poi = Location(40.0, 44.0, 100.0, is_absolute=True)

        session.sample(current, poi)
        snap = session.snapshot()

        self.assertLess(snap.dist, float("inf"))
        self.assertLess(snap.h_dist, 20.0)


class TestClosestPointComponents(unittest.TestCase):
    def test_components_are_track_frame_errors_at_3d_closest_point(self):
        poi = Location(40.0, 44.0, 100.0, is_absolute=True)
        start = _loc_from_ned(poi, -10.0, 2.0, -3.0)
        end = _loc_from_ned(poi, 10.0, 2.0, -3.0)

        components = closest_point_components_on_segment(start, end, poi)

        self.assertAlmostEqual(components.lateral, 2.0, delta=0.02)
        self.assertAlmostEqual(components.longitudinal, 0.0, delta=0.02)
        self.assertAlmostEqual(components.vertical, 3.0, delta=0.02)
        self.assertAlmostEqual(components.h_dist, 2.0, delta=0.02)
        self.assertAlmostEqual(
            components.slant,
            math.sqrt(13.0),
            delta=0.02,
        )

    def test_snap_session_records_component_scores(self):
        poi = Location(40.0, 44.0, 100.0, is_absolute=True)
        start = _loc_from_ned(poi, -10.0, 2.0, -3.0)
        end = _loc_from_ned(poi, 10.0, 2.0, -3.0)
        session = _snap_session()

        session.sample(start, poi)
        session.sample(end, poi)
        snap = session.snapshot()

        self.assertTrue(snap.has_components())
        self.assertAlmostEqual(snap.component_lateral, 2.0, delta=0.02)
        self.assertAlmostEqual(snap.component_longitudinal, 0.0, delta=0.02)
        self.assertAlmostEqual(snap.component_vertical, 3.0, delta=0.02)


if __name__ == "__main__":
    unittest.main()
