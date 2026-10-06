"""Tests for mission_metadata.py — read_fallback_delivery_location and read_mission_metadata."""
import unittest
from unittest.mock import MagicMock

from gcs.backend.planner.waypoint_builder import build_mission, META_FALLBACK_DELIVERY_LOCATION
from navpy.modules.nav.mission_metadata import read_fallback_delivery_location, read_mission_metadata


def _make_track(n=3, base_lat=32.0, base_lon=34.0, step=0.001):
    return [{"lat": base_lat + i * step, "lon": base_lon} for i in range(n)]


def _mock_vehicle_from_loader(wp_loader):
    """Create a mock IVehicle that returns items from a MAVWPLoader."""
    vehicle = MagicMock()
    vehicle.mission_items_count = wp_loader.count()

    def get_item(seq):
        if seq < 0 or seq >= wp_loader.count():
            return None
        return wp_loader.wp(seq)

    vehicle.get_mission_item = get_item
    return vehicle


class TestReadFallbackDeliveryLocation(unittest.TestCase):
    """read_fallback_delivery_location from mission waypoints built by waypoint_builder."""

    def test_with_fallback_delivery_location(self):
        """Fallback delivery location present in mission."""
        track = _make_track(3)
        dt = {"lat": 40.5, "lon": 44.5}
        wp_loader = build_mission(track, 100, fallback_delivery_location=dt)
        vehicle = _mock_vehicle_from_loader(wp_loader)

        loc = read_fallback_delivery_location(vehicle)
        self.assertIsNotNone(loc)
        self.assertAlmostEqual(loc.lat, 40.5, places=6)
        self.assertAlmostEqual(loc.lng, 44.5, places=6)

    def test_without_fallback_delivery_location(self):
        """No fallback delivery location in mission → returns None."""
        track = _make_track(3)
        wp_loader = build_mission(track, 100)
        vehicle = _mock_vehicle_from_loader(wp_loader)

        loc = read_fallback_delivery_location(vehicle)
        self.assertIsNone(loc)

    def test_with_other_metadata(self):
        """Fallback delivery location found even alongside polygon and launch metadata."""
        track = _make_track(3)
        polygon = [{"lat": 32.0, "lon": 34.0}]
        launch = {"lat": 31.5, "lon": 33.5}
        dt = {"lat": 40.5, "lon": 44.5}
        wp_loader = build_mission(
            track, 100,
            polygon=polygon, launch_point=launch, fallback_delivery_location=dt,
        )
        vehicle = _mock_vehicle_from_loader(wp_loader)

        loc = read_fallback_delivery_location(vehicle)
        self.assertIsNotNone(loc)
        self.assertAlmostEqual(loc.lat, 40.5, places=6)
        self.assertAlmostEqual(loc.lng, 44.5, places=6)

    def test_empty_mission(self):
        """Empty mission → returns None."""
        vehicle = MagicMock()
        vehicle.mission_items_count = 0
        vehicle.get_mission_item = MagicMock(return_value=None)

        loc = read_fallback_delivery_location(vehicle)
        self.assertIsNone(loc)


class TestReadMissionMetadata(unittest.TestCase):
    """read_mission_metadata from mission waypoints built by waypoint_builder."""

    def test_search_pattern_and_location_type_share_last_item(self):
        """The last metadata item packs search pattern and location type together."""
        track = _make_track(3)
        dt = {"lat": 40.5, "lon": 44.5, "type": "fuel"}
        wp_loader = build_mission(
            track, 120, search_pattern="corridor",
            polygon=[{"lat": 32.0, "lon": 34.0}],
            fallback_delivery_location=dt,
        )
        vehicle = _mock_vehicle_from_loader(wp_loader)

        meta = read_mission_metadata(vehicle)
        self.assertEqual(meta.search_pattern, "corridor")
        self.assertEqual(meta.fallback_delivery_location_type, "fuel")

    def test_decodes_search_pattern(self):
        """Mission search_pattern is correctly decoded."""
        track = _make_track(3)
        wp_loader = build_mission(track, 100, search_pattern="corridor",
                                  corridor_backbone=[{"lat": 32.0, "lon": 34.0},
                                                     {"lat": 32.01, "lon": 34.0}],
                                  corridor_count=1)
        vehicle = _mock_vehicle_from_loader(wp_loader)

        meta = read_mission_metadata(vehicle)
        self.assertEqual(meta.search_pattern, "corridor")

    def test_collects_waypoint_altitudes(self):
        """Collects distinct altitudes from NAV_WAYPOINT items."""
        track = _make_track(5)
        wp_loader = build_mission(track, 150)
        vehicle = _mock_vehicle_from_loader(wp_loader)

        meta = read_mission_metadata(vehicle)
        # Should have at least one altitude (the track altitude)
        self.assertGreater(len(meta.waypoint_altitudes), 0)
        self.assertIn(150.0, meta.waypoint_altitudes)

    def test_scan_altitude_uses_scan_band_not_corridor(self):
        """scan_altitude_rel is the post-marker scan band, NOT the higher
        corridor band that max(waypoint_altitudes) would pick."""
        track = _make_track(5)
        wp_loader = build_mission(
            track, 149,  # altitude_m = scan/zone altitude
            corridor_count=2, corridor_altitude_m=438,
            polygon=[{"lat": 32.0, "lon": 34.0}],  # forces a metadata marker
            fallback_delivery_location={"lat": 40.5, "lon": 44.5},
        )
        vehicle = _mock_vehicle_from_loader(wp_loader)

        meta = read_mission_metadata(vehicle)
        # Both bands present; old max() would have returned the 438 corridor.
        self.assertIn(438.0, meta.waypoint_altitudes)
        self.assertIn(149.0, meta.waypoint_altitudes)
        self.assertEqual(meta.scan_altitude_rel, 149.0)

    def test_scan_altitude_when_scan_above_corridor(self):
        """Scan band higher than corridor still resolves to the scan band."""
        track = _make_track(5)
        wp_loader = build_mission(
            track, 149, corridor_count=2, corridor_altitude_m=100,
            polygon=[{"lat": 32.0, "lon": 34.0}],
            fallback_delivery_location={"lat": 40.5, "lon": 44.5},
        )
        vehicle = _mock_vehicle_from_loader(wp_loader)

        meta = read_mission_metadata(vehicle)
        self.assertEqual(meta.scan_altitude_rel, 149.0)

    def test_scan_altitude_no_corridor(self):
        """corridor_count==0: markers precede all track waypoints, so the
        whole track is the scan band."""
        track = _make_track(4)
        wp_loader = build_mission(
            track, 120, corridor_count=0,
            polygon=[{"lat": 32.0, "lon": 34.0}],
        )
        vehicle = _mock_vehicle_from_loader(wp_loader)

        meta = read_mission_metadata(vehicle)
        self.assertEqual(meta.scan_altitude_rel, 120.0)

    def test_scan_altitude_no_markers_multi_band_is_ambiguous(self):
        """No metadata marker AND two distinct bands -> no authoritative
        corridor/scan boundary -> scan_altitude_rel is None (callers fall
        back to legacy sizing rather than guess the wrong band)."""
        track = _make_track(4)
        wp_loader = build_mission(
            track, 149, corridor_count=2, corridor_altitude_m=438,
        )  # no polygon/backbone/launch/fallback_delivery_location -> no markers
        vehicle = _mock_vehicle_from_loader(wp_loader)

        meta = read_mission_metadata(vehicle)
        self.assertIn(438.0, meta.waypoint_altitudes)
        self.assertIn(149.0, meta.waypoint_altitudes)
        self.assertIsNone(meta.scan_altitude_rel)

    def test_scan_altitude_no_markers_single_band(self):
        """No metadata marker but a single uniform altitude -> unambiguous
        -> that altitude is returned."""
        track = _make_track(4)
        wp_loader = build_mission(track, 120)  # no corridor, no markers
        vehicle = _mock_vehicle_from_loader(wp_loader)

        meta = read_mission_metadata(vehicle)
        self.assertEqual(meta.scan_altitude_rel, 120.0)

    def test_fallback_delivery_location_included(self):
        """Fallback delivery location is included in metadata."""
        track = _make_track(3)
        dt = {"lat": 40.5, "lon": 44.5, "type": "operations_site"}
        wp_loader = build_mission(track, 100, fallback_delivery_location=dt)
        vehicle = _mock_vehicle_from_loader(wp_loader)

        meta = read_mission_metadata(vehicle)
        self.assertIsNotNone(meta.fallback_delivery_location)
        self.assertAlmostEqual(meta.fallback_delivery_location.lat, 40.5, places=6)
        self.assertEqual(meta.fallback_delivery_location_type, "operations_site")

    def test_empty_mission(self):
        """Empty mission returns defaults."""
        vehicle = MagicMock()
        vehicle.mission_items_count = 0
        vehicle.get_mission_item = MagicMock(return_value=None)

        meta = read_mission_metadata(vehicle)
        self.assertEqual(meta.search_pattern, "distributed")
        self.assertEqual(meta.waypoint_altitudes, [])
        self.assertIsNone(meta.scan_altitude_rel)

    def test_metadata_carries_no_dock_class_selection(self):
        """Every zone targets the single dock class, so none is decoded."""
        track = _make_track(3)
        wp_loader = build_mission(track, 100, polygon=[{"lat": 32.0, "lon": 34.0}])
        vehicle = _mock_vehicle_from_loader(wp_loader)

        meta = read_mission_metadata(vehicle)
        self.assertFalse(hasattr(meta, "dock_classes"))
        self.assertFalse(hasattr(meta, "detect_class_ids"))


if __name__ == "__main__":
    unittest.main()
