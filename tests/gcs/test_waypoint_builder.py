"""Tests for waypoint_builder — metadata encoding, corridor markers, and round-trip."""
import unittest

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_NAV_WAYPOINT,
    MAV_CMD_NAV_TAKEOFF,
)

from gcs.backend.planner.waypoint_builder import (
    build_mission,
    CORRIDOR_END_MARKER,
    SEARCH_PATTERN_IDS,
    SEARCH_PATTERN_NAMES,
    encode_meta_z,
    decode_meta_z,
    encode_location_type_into_z,
    decode_location_type_from_z,
    META_POLYGON_VERTEX,
    META_CORRIDOR_VERTEX,
    META_LAUNCH_POINT,
    META_DEFAULT_DELIVERY_HUB,
)
from navpy.modules.nav.mission_encoding import LOCATION_TYPE_IDS


def _make_track(n=5, base_lat=32.0, base_lon=34.0, step=0.001):
    """Generate n waypoints in a line."""
    return [{"lat": base_lat + i * step, "lon": base_lon} for i in range(n)]


class TestBuildMissionBasic(unittest.TestCase):
    """Basic mission structure without corridor or metadata."""

    def test_empty_track(self):
        wp = build_mission([], 100)
        self.assertEqual(wp.count(), 0)

    def test_structure_home_takeoff_waypoints(self):
        """No metadata → no ROI items at all."""
        track = _make_track(3)
        wp = build_mission(track, 100)
        # home + takeoff + 3 waypoints = 5 (no marker without metadata)
        self.assertEqual(wp.count(), 5)
        self.assertEqual(wp.wp(0).command, MAV_CMD_NAV_WAYPOINT)  # home
        self.assertEqual(wp.wp(1).command, MAV_CMD_NAV_TAKEOFF)
        for i in range(2, 5):
            self.assertEqual(wp.wp(i).command, MAV_CMD_NAV_WAYPOINT)

    def test_altitude_set_on_track_waypoints(self):
        track = _make_track(2)
        wp = build_mission(track, 150)
        # home alt = 0, takeoff = 150, track = 150
        self.assertEqual(wp.wp(0).z, 0)
        self.assertEqual(wp.wp(1).z, 150)
        self.assertEqual(wp.wp(2).z, 150)
        self.assertEqual(wp.wp(3).z, 150)

    def test_coordinates_int_format(self):
        track = [{"lat": 32.123456, "lon": 34.654321}]
        wp = build_mission(track, 100)
        # seq 2 is first track waypoint (no marker)
        self.assertEqual(wp.wp(2).x, int(32.123456 * 1e7))
        self.assertEqual(wp.wp(2).y, int(34.654321 * 1e7))

    def test_no_marker_without_metadata(self):
        """No metadata → no ROI items in mission."""
        track = _make_track(4)
        wp = build_mission(track, 100, corridor_count=0)
        commands = [wp.wp(i).command for i in range(wp.count())]
        self.assertNotIn(CORRIDOR_END_MARKER, commands)


class TestCorridorWithoutMetadata(unittest.TestCase):
    """Corridor waypoints without metadata — no ROI markers."""

    def test_no_marker_without_metadata(self):
        """corridor_count=2 but no metadata → no ROI items."""
        track = _make_track(5)
        wp = build_mission(track, 100, corridor_count=2)
        # home + takeoff + 5 waypoints = 7 (no marker)
        self.assertEqual(wp.count(), 7)
        commands = [wp.wp(i).command for i in range(wp.count())]
        self.assertNotIn(CORRIDOR_END_MARKER, commands)

    def test_all_corridor_no_track(self):
        """corridor_count == len(track), no metadata → no markers."""
        track = _make_track(3)
        wp = build_mission(track, 100, corridor_count=3)
        commands = [wp.wp(i).command for i in range(wp.count())]
        self.assertNotIn(CORRIDOR_END_MARKER, commands)

    def test_corridor_count_exceeds_track(self):
        """corridor_count > len(track) should not crash."""
        track = _make_track(2)
        wp = build_mission(track, 100, corridor_count=5)
        self.assertEqual(wp.count(), 4)  # home + takeoff + 2 wps

    def test_waypoint_order_preserved(self):
        """Corridor + track waypoints preserve original order (no metadata)."""
        track = [
            {"lat": 32.0, "lon": 34.0},
            {"lat": 32.1, "lon": 34.1},
            {"lat": 32.2, "lon": 34.2},
            {"lat": 32.3, "lon": 34.3},
        ]
        wp = build_mission(track, 100, corridor_count=2)
        # seq 0=home, 1=takeoff, 2-5=track (no marker)
        self.assertAlmostEqual(wp.wp(2).x / 1e7, 32.0, places=5)
        self.assertAlmostEqual(wp.wp(3).x / 1e7, 32.1, places=5)
        self.assertAlmostEqual(wp.wp(4).x / 1e7, 32.2, places=5)
        self.assertAlmostEqual(wp.wp(5).x / 1e7, 32.3, places=5)


class TestCorridorWithMetadata(unittest.TestCase):
    """Corridor waypoints WITH metadata — ROI items mark the boundary."""

    def test_marker_position_with_polygon(self):
        """With corridor_count=2, metadata at boundary between corridor and track."""
        track = _make_track(5)
        polygon = [{"lat": 32.0, "lon": 34.0}]
        wp = build_mission(track, 100, corridor_count=2, polygon=polygon)
        # home + takeoff + 5 track wps + 1 meta = 8
        self.assertEqual(wp.count(), 8)
        self.assertEqual(wp.wp(2).command, MAV_CMD_NAV_WAYPOINT)  # corridor 0
        self.assertEqual(wp.wp(3).command, MAV_CMD_NAV_WAYPOINT)  # corridor 1
        self.assertEqual(wp.wp(4).command, CORRIDOR_END_MARKER)   # metadata
        self.assertEqual(wp.wp(5).command, MAV_CMD_NAV_WAYPOINT)  # track 0

    def test_single_corridor_with_launch_point(self):
        """corridor_count=1 with launch_point metadata."""
        track = _make_track(4)
        launch = {"lat": 31.5, "lon": 33.5}
        wp = build_mission(track, 100, corridor_count=1, launch_point=launch)
        # home + takeoff + 4 track wps + 1 meta = 7
        self.assertEqual(wp.count(), 7)
        self.assertEqual(wp.wp(2).command, MAV_CMD_NAV_WAYPOINT)  # corridor
        self.assertEqual(wp.wp(3).command, CORRIDOR_END_MARKER)   # launch_point meta
        self.assertEqual(wp.wp(4).command, MAV_CMD_NAV_WAYPOINT)  # track start

    def test_waypoint_order_with_metadata(self):
        """Corridor + metadata + track preserve correct order."""
        track = [
            {"lat": 32.0, "lon": 34.0},
            {"lat": 32.1, "lon": 34.1},
            {"lat": 32.2, "lon": 34.2},
            {"lat": 32.3, "lon": 34.3},
        ]
        polygon = [{"lat": 31.0, "lon": 33.0}]
        wp = build_mission(track, 100, corridor_count=2, polygon=polygon)
        # seq 2,3 = corridor; seq 4 = meta; seq 5,6 = track
        self.assertAlmostEqual(wp.wp(2).x / 1e7, 32.0, places=5)
        self.assertAlmostEqual(wp.wp(3).x / 1e7, 32.1, places=5)
        self.assertEqual(wp.wp(4).command, CORRIDOR_END_MARKER)
        self.assertAlmostEqual(wp.wp(5).x / 1e7, 32.2, places=5)
        self.assertAlmostEqual(wp.wp(6).x / 1e7, 32.3, places=5)


class TestSearchPatternEncoding(unittest.TestCase):
    """Search pattern bitmask stored in z (alt) of the LAST metadata item only."""

    def test_search_pattern_bitmask_values(self):
        """Search pattern IDs use bitmask encoding."""
        self.assertEqual(SEARCH_PATTERN_IDS["distributed"], 1)
        self.assertEqual(SEARCH_PATTERN_IDS["corridor"], 4)

    def test_search_pattern_ids_reverse_map(self):
        """SEARCH_PATTERN_NAMES should reverse SEARCH_PATTERN_IDS."""
        for name, tid in SEARCH_PATTERN_IDS.items():
            self.assertEqual(SEARCH_PATTERN_NAMES[tid], name)

    def test_distributed_search_pattern_only_on_last_z(self):
        """Distributed search_pattern → z on last metadata item only, others z=0."""
        track = _make_track(3)
        polygon = [{"lat": 32.0, "lon": 34.0}, {"lat": 32.1, "lon": 34.0}]
        wp = build_mission(track, 100, search_pattern="distributed", polygon=polygon)
        # First meta item (seq 2): z=0
        self.assertEqual(wp.wp(2).z, 0)
        # Last meta item (seq 3): z=encode_meta_z("distributed")
        self.assertEqual(wp.wp(3).z, encode_meta_z("distributed"))

    def test_corridor_search_pattern_only_on_last_z(self):
        """Corridor search_pattern → z on last metadata item only."""
        track = _make_track(3)
        polygon = [{"lat": 32.0, "lon": 34.0}]
        wp = build_mission(track, 100, search_pattern="corridor", polygon=polygon)
        self.assertEqual(wp.wp(2).z, encode_meta_z("corridor"))

    def test_unknown_search_pattern_defaults_to_distributed(self):
        """Unknown search_pattern name → defaults to distributed bitmask (1)."""
        track = _make_track(3)
        polygon = [{"lat": 32.0, "lon": 34.0}]
        wp = build_mission(track, 100, search_pattern="unknown", polygon=polygon)
        self.assertEqual(wp.wp(2).z, encode_meta_z("unknown"))

    def test_no_metadata_no_marker(self):
        """Without metadata, no ROI items at all (search_pattern defaults on download)."""
        track = _make_track(3)
        wp = build_mission(track, 100, search_pattern="corridor")
        commands = [wp.wp(i).command for i in range(wp.count())]
        self.assertNotIn(CORRIDOR_END_MARKER, commands)


class TestRoundTrip(unittest.TestCase):
    """Simulate upload → download round-trip parsing."""

    def _simulate_download(self, wp_loader):
        """Mimic the download endpoint logic.

        Every DO_SET_ROI_LOCATION is a metadata item:
        - param1 = meta_type, x/y = lat/lon
        - META_DEFAULT_DELIVERY_HUB z bits 8-10 = compact location type id
        - First one marks the corridor boundary
        - Only the LAST metadata item's z carries search_pattern
        """
        waypoints = []
        corridor_end_index = None
        polygon_vertices = []
        corridor_backbone = []
        launch_point = None
        default_delivery_hub = None
        last_meta_z = 0.0
        nav_index = 0
        in_metadata = False
        for i in range(wp_loader.count()):
            wp = wp_loader.wp(i)
            if i == 0 or wp.command == MAV_CMD_NAV_TAKEOFF:
                continue
            if wp.command == CORRIDOR_END_MARKER:
                if not in_metadata:
                    corridor_end_index = nav_index
                    in_metadata = True
                # Track z — the last metadata item carries the encoded value
                if wp.z != 0:
                    last_meta_z = wp.z
                # Metadata: param1 = meta_type, x/y = lat/lon
                meta_type = int(wp.param1)
                lat = wp.x / 1e7
                lon = wp.y / 1e7
                if meta_type == META_POLYGON_VERTEX:
                    polygon_vertices.append({"lat": lat, "lon": lon})
                elif meta_type == META_CORRIDOR_VERTEX:
                    corridor_backbone.append({"lat": lat, "lon": lon})
                elif meta_type == META_LAUNCH_POINT:
                    launch_point = {"lat": lat, "lon": lon}
                elif meta_type == META_DEFAULT_DELIVERY_HUB:
                    default_delivery_hub = {"lat": lat, "lon": lon}
                    location_type = decode_location_type_from_z(wp.z)
                    if location_type:
                        default_delivery_hub["type"] = location_type
                continue
            in_metadata = False
            waypoints.append({"lat": wp.x / 1e7, "lon": wp.y / 1e7})
            nav_index += 1
        search_pattern = decode_meta_z(last_meta_z)
        # Trim the trailing default-delivery-hub NAV_WAYPOINT (matches real download)
        if default_delivery_hub and waypoints:
            last = waypoints[-1]
            if (abs(last["lat"] - default_delivery_hub["lat"]) < 1e-5 and
                    abs(last["lon"] - default_delivery_hub["lon"]) < 1e-5):
                waypoints.pop()
        return waypoints, corridor_end_index, search_pattern, polygon_vertices, corridor_backbone, launch_point, default_delivery_hub

    def test_round_trip_no_metadata(self):
        """No metadata → all waypoints returned, no corridor boundary detected."""
        track = _make_track(4)
        wp = build_mission(track, 100, corridor_count=0)
        waypoints, corridor_end_index, search_pattern, *_ = self._simulate_download(wp)
        self.assertEqual(len(waypoints), 4)
        self.assertIsNone(corridor_end_index)  # no ROI items
        self.assertEqual(search_pattern, "distributed")  # default

    def test_round_trip_with_metadata(self):
        """Metadata present → corridor boundary detected."""
        track = _make_track(5)
        polygon = [{"lat": 32.0, "lon": 34.0}]
        wp = build_mission(track, 100, corridor_count=2, polygon=polygon)
        waypoints, corridor_end_index, *_ = self._simulate_download(wp)
        self.assertEqual(len(waypoints), 5)
        self.assertEqual(corridor_end_index, 2)

    def test_round_trip_without_corridor(self):
        """corridor_count=0 with metadata → corridor_end_index=0."""
        track = _make_track(4)
        polygon = [{"lat": 32.0, "lon": 34.0}]
        wp = build_mission(track, 100, corridor_count=0, polygon=polygon)
        waypoints, corridor_end_index, *_ = self._simulate_download(wp)
        self.assertEqual(len(waypoints), 4)
        self.assertEqual(corridor_end_index, 0)

    def test_round_trip_coordinates_preserved(self):
        """Coordinates survive the round-trip (within INT precision)."""
        track = [
            {"lat": 32.123456, "lon": 34.654321},
            {"lat": 32.234567, "lon": 34.765432},
            {"lat": 32.345678, "lon": 34.876543},
        ]
        polygon = [{"lat": 32.0, "lon": 34.0}]
        wp = build_mission(track, 100, corridor_count=1, polygon=polygon)
        waypoints, *_ = self._simulate_download(wp)
        for orig, downloaded in zip(track, waypoints):
            self.assertAlmostEqual(orig["lat"], downloaded["lat"], places=6)
            self.assertAlmostEqual(orig["lon"], downloaded["lon"], places=6)

    def test_round_trip_search_pattern_distributed(self):
        track = _make_track(3)
        polygon = [{"lat": 32.0, "lon": 34.0}]
        wp = build_mission(track, 100, search_pattern="distributed", polygon=polygon)
        _, _, search_pattern, *_ = self._simulate_download(wp)
        self.assertEqual(search_pattern, "distributed")

    def test_round_trip_search_pattern_corridor(self):
        track = _make_track(5)
        polygon = [{"lat": 32.0, "lon": 34.0}]
        wp = build_mission(track, 100, corridor_count=2, search_pattern="corridor", polygon=polygon)
        _, _, search_pattern, *_ = self._simulate_download(wp)
        self.assertEqual(search_pattern, "corridor")


class TestMetadataEncoding(unittest.TestCase):
    """Polygon and corridor backbone metadata encoded as DO items."""

    def test_polygon_vertices_encoded(self):
        """Polygon vertices inserted as metadata DO items."""
        track = _make_track(3)
        polygon = [
            {"lat": 32.0, "lon": 34.0},
            {"lat": 32.1, "lon": 34.0},
            {"lat": 32.1, "lon": 34.1},
            {"lat": 32.0, "lon": 34.1},
        ]
        wp = build_mission(track, 100, polygon=polygon)
        # home + takeoff + 4 meta + 3 track = 9
        self.assertEqual(wp.count(), 9)
        for j in range(4):
            meta = wp.wp(2 + j)
            self.assertEqual(meta.command, CORRIDOR_END_MARKER)
            self.assertEqual(int(meta.param1), META_POLYGON_VERTEX)
            self.assertAlmostEqual(meta.x / 1e7, polygon[j]["lat"], places=6)
            self.assertAlmostEqual(meta.y / 1e7, polygon[j]["lon"], places=6)
        # Only last meta item has z set, others are 0
        for j in range(3):
            self.assertEqual(wp.wp(2 + j).z, 0)
        self.assertEqual(wp.wp(5).z, encode_meta_z("distributed"))
        # track waypoints follow at seq 6-8
        for j in range(3):
            self.assertEqual(wp.wp(6 + j).command, MAV_CMD_NAV_WAYPOINT)

    def test_corridor_backbone_encoded(self):
        """Corridor backbone vertices encoded as metadata DO items."""
        track = _make_track(3)
        corridor_backbone = [
            {"lat": 31.9, "lon": 33.9},
            {"lat": 31.95, "lon": 33.95},
        ]
        wp = build_mission(track, 100, corridor_backbone=corridor_backbone)
        # home + takeoff + 2 meta + 3 track = 7
        self.assertEqual(wp.count(), 7)
        for j in range(2):
            meta = wp.wp(2 + j)
            self.assertEqual(int(meta.param1), META_CORRIDOR_VERTEX)
            self.assertAlmostEqual(meta.x / 1e7, corridor_backbone[j]["lat"], places=6)
            self.assertAlmostEqual(meta.y / 1e7, corridor_backbone[j]["lon"], places=6)

    def test_polygon_and_corridor_backbone_combined(self):
        """Both polygon and corridor backbone encoded together."""
        track = _make_track(3)
        polygon = [
            {"lat": 32.0, "lon": 34.0},
            {"lat": 32.1, "lon": 34.0},
            {"lat": 32.1, "lon": 34.1},
        ]
        corridor_backbone = [{"lat": 31.9, "lon": 33.9}]
        wp = build_mission(track, 100, polygon=polygon, corridor_backbone=corridor_backbone)
        # home + takeoff + 3 poly + 1 corr + 3 track = 9
        self.assertEqual(wp.count(), 9)
        # polygon first (seq 2-4)
        for j in range(3):
            self.assertEqual(int(wp.wp(2 + j).param1), META_POLYGON_VERTEX)
        # corridor after polygon (seq 5)
        self.assertEqual(int(wp.wp(5).param1), META_CORRIDOR_VERTEX)

    def test_no_metadata_no_roi(self):
        """No metadata → no ROI items in mission."""
        track = _make_track(3)
        wp = build_mission(track, 100)
        # home + takeoff + 3 track = 5
        self.assertEqual(wp.count(), 5)
        commands = [wp.wp(i).command for i in range(wp.count())]
        self.assertNotIn(CORRIDOR_END_MARKER, commands)


class TestMetadataRoundTrip(TestRoundTrip):
    """Round-trip tests for polygon and corridor backbone metadata."""

    def test_round_trip_polygon_preserved(self):
        """Polygon vertices survive upload → download round-trip."""
        track = _make_track(3)
        polygon = [
            {"lat": 32.0, "lon": 34.0},
            {"lat": 32.1, "lon": 34.0},
            {"lat": 32.1, "lon": 34.1},
            {"lat": 32.0, "lon": 34.1},
        ]
        wp = build_mission(track, 100, polygon=polygon)
        _, _, _, dl_polygon, dl_backbone, _, _ = self._simulate_download(wp)
        self.assertEqual(len(dl_polygon), 4)
        for orig, dl in zip(polygon, dl_polygon):
            self.assertAlmostEqual(orig["lat"], dl["lat"], places=6)
            self.assertAlmostEqual(orig["lon"], dl["lon"], places=6)
        self.assertEqual(len(dl_backbone), 0)

    def test_round_trip_corridor_backbone_preserved(self):
        """Corridor backbone vertices survive upload → download round-trip."""
        track = _make_track(3)
        backbone = [
            {"lat": 31.9, "lon": 33.9},
            {"lat": 31.95, "lon": 33.95},
        ]
        wp = build_mission(track, 100, corridor_backbone=backbone)
        _, _, _, dl_polygon, dl_backbone, _, _ = self._simulate_download(wp)
        self.assertEqual(len(dl_polygon), 0)
        self.assertEqual(len(dl_backbone), 2)
        for orig, dl in zip(backbone, dl_backbone):
            self.assertAlmostEqual(orig["lat"], dl["lat"], places=6)
            self.assertAlmostEqual(orig["lon"], dl["lon"], places=6)

    def test_round_trip_both_preserved(self):
        """Both polygon and corridor backbone survive round-trip together."""
        track = _make_track(3)
        polygon = [
            {"lat": 32.0, "lon": 34.0},
            {"lat": 32.1, "lon": 34.0},
            {"lat": 32.1, "lon": 34.1},
        ]
        backbone = [{"lat": 31.9, "lon": 33.9}]
        wp = build_mission(track, 100, polygon=polygon, corridor_backbone=backbone)
        waypoints, corridor_end_index, _, dl_polygon, dl_backbone, _, _ = self._simulate_download(wp)
        self.assertEqual(len(waypoints), 3)
        self.assertEqual(corridor_end_index, 0)
        self.assertEqual(len(dl_polygon), 3)
        self.assertEqual(len(dl_backbone), 1)

    def test_round_trip_no_metadata_backward_compat(self):
        """No metadata → empty polygon/backbone/launch_point, no corridor boundary."""
        track = _make_track(3)
        wp = build_mission(track, 100)
        _, corridor_end_index, _, dl_polygon, dl_backbone, dl_lp, _ = self._simulate_download(wp)
        self.assertIsNone(corridor_end_index)
        self.assertEqual(len(dl_polygon), 0)
        self.assertEqual(len(dl_backbone), 0)
        self.assertIsNone(dl_lp)

    def test_round_trip_metadata_with_corridor(self):
        """Metadata works correctly with corridor waypoints."""
        track = _make_track(5)
        polygon = [
            {"lat": 32.0, "lon": 34.0},
            {"lat": 32.1, "lon": 34.0},
            {"lat": 32.1, "lon": 34.1},
        ]
        backbone = [{"lat": 31.9, "lon": 33.9}]
        wp = build_mission(track, 100, corridor_count=2, search_pattern="corridor",
                           polygon=polygon, corridor_backbone=backbone)
        waypoints, corridor_end_index, search_pattern, dl_polygon, dl_backbone, _, _ = self._simulate_download(wp)
        self.assertEqual(len(waypoints), 5)
        self.assertEqual(corridor_end_index, 2)
        self.assertEqual(search_pattern, "corridor")
        self.assertEqual(len(dl_polygon), 3)
        self.assertEqual(len(dl_backbone), 1)


class TestLaunchPoint(unittest.TestCase):
    """Launch point as separate home/takeoff position."""

    def test_launch_point_as_home_takeoff(self):
        """Home and Takeoff use launch_point coords, not track[0]."""
        launch = {"lat": 31.5, "lon": 33.5}
        track = _make_track(3, base_lat=32.0, base_lon=34.0)
        wp = build_mission(track, 100, launch_point=launch)
        # Home (seq 0) uses launch coords
        self.assertEqual(wp.wp(0).x, int(31.5 * 1e7))
        self.assertEqual(wp.wp(0).y, int(33.5 * 1e7))
        self.assertEqual(wp.wp(0).z, 0)
        # Takeoff (seq 1) uses launch coords
        self.assertEqual(wp.wp(1).x, int(31.5 * 1e7))
        self.assertEqual(wp.wp(1).y, int(33.5 * 1e7))
        self.assertEqual(wp.wp(1).z, 100)
        self.assertEqual(wp.wp(1).command, MAV_CMD_NAV_TAKEOFF)
        # Launch_point meta at seq 2, track at seq 3
        self.assertAlmostEqual(wp.wp(3).x / 1e7, 32.0, places=5)

    def test_launch_point_not_in_track(self):
        """Launch point is not added as a NAV_WAYPOINT in the track."""
        launch = {"lat": 31.5, "lon": 33.5}
        track = _make_track(3)
        wp = build_mission(track, 100, launch_point=launch)
        # home + takeoff + 1 launch_point meta + 3 track = 6
        self.assertEqual(wp.count(), 6)
        self.assertEqual(wp.wp(2).command, CORRIDOR_END_MARKER)
        self.assertEqual(int(wp.wp(2).param1), META_LAUNCH_POINT)
        self.assertEqual(wp.wp(3).command, MAV_CMD_NAV_WAYPOINT)

    def test_launch_point_metadata_encoded(self):
        """Launch point encoded as META_LAUNCH_POINT metadata item with search_pattern in z (last item)."""
        launch = {"lat": 31.5, "lon": 33.5}
        track = _make_track(3)
        wp = build_mission(track, 100, launch_point=launch)
        meta = wp.wp(2)
        self.assertEqual(meta.command, CORRIDOR_END_MARKER)
        self.assertEqual(int(meta.param1), META_LAUNCH_POINT)
        self.assertAlmostEqual(meta.x / 1e7, 31.5, places=6)
        self.assertAlmostEqual(meta.y / 1e7, 33.5, places=6)
        # Single meta item is the last → carries encoded z
        self.assertEqual(meta.z, encode_meta_z("distributed"))

    def test_no_launch_point_uses_track_first(self):
        """Without launch_point, Home/Takeoff use track[0] (backward compat)."""
        track = _make_track(3, base_lat=32.0, base_lon=34.0)
        wp = build_mission(track, 100)
        self.assertEqual(wp.wp(0).x, int(32.0 * 1e7))
        self.assertEqual(wp.wp(0).y, int(34.0 * 1e7))
        self.assertEqual(wp.wp(1).x, int(32.0 * 1e7))
        self.assertEqual(wp.wp(1).y, int(34.0 * 1e7))


class TestLaunchPointRoundTrip(TestRoundTrip):
    """Round-trip tests for launch_point metadata."""

    def test_launch_point_metadata_round_trip(self):
        """Launch point survives upload → download round-trip."""
        launch = {"lat": 31.5, "lon": 33.5}
        track = _make_track(3)
        wp = build_mission(track, 100, launch_point=launch)
        _, _, _, _, _, dl_lp, _ = self._simulate_download(wp)
        self.assertIsNotNone(dl_lp)
        self.assertAlmostEqual(dl_lp["lat"], 31.5, places=6)
        self.assertAlmostEqual(dl_lp["lon"], 33.5, places=6)

    def test_launch_point_with_polygon_and_corridor(self):
        """Launch point + polygon + corridor backbone all survive round-trip."""
        launch = {"lat": 31.5, "lon": 33.5}
        track = _make_track(5)
        polygon = [
            {"lat": 32.0, "lon": 34.0},
            {"lat": 32.1, "lon": 34.0},
            {"lat": 32.1, "lon": 34.1},
        ]
        backbone = [{"lat": 31.9, "lon": 33.9}]
        wp = build_mission(track, 100, corridor_count=2, search_pattern="corridor",
                           polygon=polygon, corridor_backbone=backbone,
                           launch_point=launch)
        waypoints, corridor_end_index, search_pattern, dl_poly, dl_corr, dl_lp, _ = self._simulate_download(wp)
        self.assertEqual(len(waypoints), 5)
        self.assertEqual(corridor_end_index, 2)
        self.assertEqual(search_pattern, "corridor")
        self.assertEqual(len(dl_poly), 3)
        self.assertEqual(len(dl_corr), 1)
        self.assertIsNotNone(dl_lp)
        self.assertAlmostEqual(dl_lp["lat"], 31.5, places=6)

    def test_no_launch_point_returns_none(self):
        """Without launch_point, download returns None for launch_point."""
        track = _make_track(3)
        wp = build_mission(track, 100)
        _, _, _, _, _, dl_lp, _ = self._simulate_download(wp)
        self.assertIsNone(dl_lp)


class TestMetaZEncoding(unittest.TestCase):
    """Metadata z layout: [location_type:3 | search_pattern:8]; no dock class field."""

    def test_encode_search_pattern_only(self):
        self.assertEqual(encode_meta_z("distributed"), 1.0)
        self.assertEqual(encode_meta_z("corridor"), 4.0)

    def test_encode_unknown_search_pattern_defaults_to_distributed(self):
        self.assertEqual(encode_meta_z("unknown"), encode_meta_z("distributed"))

    def test_decode_zero_defaults_to_distributed(self):
        self.assertEqual(decode_meta_z(0), "distributed")

    def test_encode_decode_roundtrip(self):
        for search_pattern in SEARCH_PATTERN_IDS:
            with self.subTest(search_pattern=search_pattern):
                self.assertEqual(decode_meta_z(encode_meta_z(search_pattern)), search_pattern)

    def test_location_type_occupies_bits_8_to_10(self):
        z = encode_location_type_into_z(encode_meta_z("corridor"), "other")
        self.assertEqual(z, float((7 << 8) | 4))
        self.assertLess(z, float(1 << 11))  # 11-bit field fits a DO item param

    def test_location_type_and_search_pattern_roundtrip_together(self):
        for search_pattern in SEARCH_PATTERN_IDS:
            for type_name in LOCATION_TYPE_IDS:
                with self.subTest(search_pattern=search_pattern, type_name=type_name):
                    z = encode_location_type_into_z(encode_meta_z(search_pattern), type_name)
                    self.assertEqual(decode_meta_z(z), search_pattern)
                    self.assertEqual(decode_location_type_from_z(z), type_name)

    def test_unknown_location_type_encodes_as_unset(self):
        z = encode_location_type_into_z(encode_meta_z("distributed"), "nope")
        self.assertEqual(z, encode_meta_z("distributed"))
        self.assertIsNone(decode_location_type_from_z(z))

    def test_reencoding_location_type_replaces_previous_value(self):
        z = encode_location_type_into_z(encode_meta_z("corridor"), "fuel")
        z = encode_location_type_into_z(z, "bridge")
        self.assertEqual(decode_location_type_from_z(z), "bridge")
        self.assertEqual(decode_meta_z(z), "corridor")


class TestMetaZOnMission(TestRoundTrip):
    """Search pattern lives only on the last metadata item."""

    def test_z_only_on_last_metadata_item(self):
        """Only the last metadata item has non-zero z; all others are z=0."""
        track = _make_track(3)
        polygon = [
            {"lat": 32.0, "lon": 34.0},
            {"lat": 32.1, "lon": 34.0},
            {"lat": 32.1, "lon": 34.1},
        ]
        launch = {"lat": 31.5, "lon": 33.5}
        wp = build_mission(track, 100, polygon=polygon, launch_point=launch,
                           search_pattern="corridor")
        # 3 polygon + 1 launch = 4 metadata items at seq 2-5
        for j in range(3):
            self.assertEqual(wp.wp(2 + j).z, 0, f"metadata item {j} should have z=0")
        self.assertEqual(wp.wp(5).z, encode_meta_z("corridor"))

    def test_build_mission_rejects_dock_classes_argument(self):
        """The dock-class selection is gone from the builder API."""
        with self.assertRaises(TypeError):
            build_mission(_make_track(3), 100, dock_classes=["dock"])


class TestMissionDeliveryHub(unittest.TestCase):
    """Default delivery hub metadata encoding and round-trip."""

    def test_default_delivery_hub_encoded_as_metadata(self):
        """Default delivery hub inserted as META_DEFAULT_DELIVERY_HUB metadata item."""
        track = _make_track(3)
        dt = {"lat": 40.5, "lon": 44.5, "type": "bridge"}
        wp = build_mission(track, 100, default_delivery_hub=dt)
        # home + takeoff + 1 meta (default_delivery_hub) + 3 track + 1 dt NAV_WP = 7
        self.assertEqual(wp.count(), 7)
        meta = wp.wp(2)
        self.assertEqual(meta.command, CORRIDOR_END_MARKER)
        self.assertEqual(int(meta.param1), META_DEFAULT_DELIVERY_HUB)
        self.assertEqual(decode_location_type_from_z(meta.z), "bridge")
        self.assertAlmostEqual(meta.x / 1e7, 40.5, places=6)
        self.assertAlmostEqual(meta.y / 1e7, 44.5, places=6)

    def test_default_delivery_hub_with_other_metadata(self):
        """Default delivery hub combined with polygon and launch point."""
        track = _make_track(3)
        polygon = [{"lat": 32.0, "lon": 34.0}]
        launch = {"lat": 31.5, "lon": 33.5}
        dt = {"lat": 40.5, "lon": 44.5}
        wp = build_mission(track, 100, polygon=polygon, launch_point=launch, default_delivery_hub=dt)
        # home + takeoff + 3 meta (poly + launch + dt) + 3 track + 1 dt NAV_WP = 9
        self.assertEqual(wp.count(), 9)
        # Check meta types in order
        self.assertEqual(int(wp.wp(2).param1), META_POLYGON_VERTEX)
        self.assertEqual(int(wp.wp(3).param1), META_LAUNCH_POINT)
        self.assertEqual(int(wp.wp(4).param1), META_DEFAULT_DELIVERY_HUB)

    def test_default_delivery_hub_last_wp_is_nav_waypoint(self):
        """Last mission item is a NAV_WAYPOINT at delivery hub coordinates with mission altitude."""
        track = _make_track(3)
        dt = {"lat": 40.5, "lon": 44.5}
        wp = build_mission(track, 100, default_delivery_hub=dt)
        last = wp.wp(wp.count() - 1)
        self.assertEqual(last.command, MAV_CMD_NAV_WAYPOINT)
        self.assertAlmostEqual(last.x / 1e7, 40.5, places=6)
        self.assertAlmostEqual(last.y / 1e7, 44.5, places=6)
        self.assertEqual(last.z, 100)

    def test_no_default_delivery_hub(self):
        """No default_delivery_hub → same as before."""
        track = _make_track(3)
        wp = build_mission(track, 100)
        self.assertEqual(wp.count(), 5)  # home + takeoff + 3 wps


class TestMetadataEmptyTrack(TestRoundTrip):
    """P-5: metadata must survive even when corridor_count >= len(track).

    A distributed set whose scan track is empty uploads waypoints == corridor
    points, so corridor_count == len(track) and the in-loop `i == corridor_count`
    guard never fires. The metadata block must be emitted anyway so the companion
    can still recover search_pattern / polygon.
    """

    def test_metadata_emitted_when_corridor_count_equals_track_len(self):
        track = _make_track(2)  # e.g. 2 corridor points, empty scan track
        polygon = [{"lat": 32.0, "lon": 34.0}, {"lat": 32.1, "lon": 34.0}]
        wp = build_mission(track, 100, corridor_count=2, search_pattern="corridor",
                           polygon=polygon)
        commands = [wp.wp(i).command for i in range(wp.count())]
        self.assertIn(CORRIDOR_END_MARKER, commands)   # metadata NOT dropped
        # home + takeoff + 2 track + 2 polygon meta = 6 (meta appended after track)
        self.assertEqual(wp.count(), 6)
        self.assertEqual(wp.wp(2).command, MAV_CMD_NAV_WAYPOINT)
        self.assertEqual(wp.wp(3).command, MAV_CMD_NAV_WAYPOINT)
        self.assertEqual(wp.wp(4).command, CORRIDOR_END_MARKER)
        self.assertEqual(wp.wp(5).command, CORRIDOR_END_MARKER)

    def test_corridor_count_exceeds_track_still_emits_metadata(self):
        track = _make_track(2)
        polygon = [{"lat": 32.0, "lon": 34.0}]
        wp = build_mission(track, 100, corridor_count=5, polygon=polygon)
        commands = [wp.wp(i).command for i in range(wp.count())]
        self.assertIn(CORRIDOR_END_MARKER, commands)

    def test_round_trip_recovers_search_pattern_with_empty_scan_track(self):
        track = _make_track(2)
        polygon = [{"lat": 32.0, "lon": 34.0}]
        wp = build_mission(track, 100, corridor_count=2, search_pattern="corridor",
                           polygon=polygon)
        _, _, search_pattern, dl_poly, _, _, _ = self._simulate_download(wp)
        self.assertEqual(search_pattern, "corridor")
        self.assertEqual(len(dl_poly), 1)

    def test_no_metadata_empty_track_still_no_marker(self):
        # Without metadata the post-loop insert is a no-op (backward compatible).
        track = _make_track(2)
        wp = build_mission(track, 100, corridor_count=2)
        commands = [wp.wp(i).command for i in range(wp.count())]
        self.assertNotIn(CORRIDOR_END_MARKER, commands)


class TestMissionDeliveryHubRoundTrip(TestRoundTrip):
    """Default delivery hub survives build → simulate_download round-trip."""

    def test_default_delivery_hub_round_trip(self):
        """Default delivery hub coordinates survive round-trip."""
        track = _make_track(3)
        dt = {"lat": 40.5, "lon": 44.5, "type": "fuel"}
        wp = build_mission(track, 100, default_delivery_hub=dt)
        *_, dl_dt = self._simulate_download(wp)
        self.assertIsNotNone(dl_dt)
        self.assertAlmostEqual(dl_dt["lat"], 40.5, places=6)
        self.assertAlmostEqual(dl_dt["lon"], 44.5, places=6)
        self.assertEqual(dl_dt["type"], "fuel")

    def test_default_delivery_hub_with_all_metadata(self):
        """Default delivery hub + polygon + launch + corridor all survive round-trip."""
        track = _make_track(5)
        polygon = [{"lat": 32.0, "lon": 34.0}, {"lat": 32.1, "lon": 34.0}]
        backbone = [{"lat": 31.9, "lon": 33.9}]
        launch = {"lat": 31.5, "lon": 33.5}
        dt = {"lat": 40.5, "lon": 44.5}
        wp = build_mission(track, 100, corridor_count=2, search_pattern="corridor",
                           polygon=polygon, corridor_backbone=backbone,
                           launch_point=launch, default_delivery_hub=dt)
        wps, ci, search_pattern, dl_poly, dl_corr, dl_lp, dl_dt = self._simulate_download(wp)
        self.assertEqual(len(wps), 5)
        self.assertEqual(ci, 2)
        self.assertEqual(search_pattern, "corridor")
        self.assertEqual(len(dl_poly), 2)
        self.assertEqual(len(dl_corr), 1)
        self.assertIsNotNone(dl_lp)
        self.assertIsNotNone(dl_dt)
        self.assertAlmostEqual(dl_dt["lat"], 40.5, places=6)

    def test_no_default_delivery_hub_returns_none(self):
        """Without default_delivery_hub, download returns None."""
        track = _make_track(3)
        wp = build_mission(track, 100)
        *_, dl_dt = self._simulate_download(wp)
        self.assertIsNone(dl_dt)


class TestTakeoffAltitude(unittest.TestCase):
    """Safe takeoff-height sets the NAV_TAKEOFF climb target (seq 1)."""

    def test_takeoff_altitude_used_when_below_leg(self):
        """takeoff_altitude_m below the first-leg altitude → used verbatim."""
        track = _make_track(2)
        wp = build_mission(track, 150, takeoff_altitude_m=40)
        self.assertEqual(wp.wp(1).command, MAV_CMD_NAV_TAKEOFF)
        self.assertEqual(wp.wp(1).z, 40)     # takeoff climbs to safe height
        self.assertEqual(wp.wp(2).z, 150)    # track still at zone altitude
        self.assertEqual(wp.wp(3).z, 150)

    def test_takeoff_altitude_clamped_to_zone_altitude(self):
        """takeoff_altitude_m above the mission altitude → clamped down.

        Never command a climb above the first leg followed by a descent.
        """
        track = _make_track(2)
        wp = build_mission(track, 30, takeoff_altitude_m=40)
        self.assertEqual(wp.wp(1).z, 30)     # clamped to altitude_m

    def test_takeoff_altitude_uses_corridor_alt_ceiling(self):
        """With a corridor, the first-leg (corridor) altitude is the ceiling."""
        track = _make_track(5)
        # safe height below corridor altitude → used verbatim
        wp = build_mission(track, 200, corridor_count=2,
                           corridor_altitude_m=100, takeoff_altitude_m=40)
        self.assertEqual(wp.wp(1).z, 40)
        # safe height above corridor altitude → clamped to corridor altitude
        wp2 = build_mission(track, 200, corridor_count=2,
                            corridor_altitude_m=30, takeoff_altitude_m=40)
        self.assertEqual(wp2.wp(1).z, 30)

    def test_takeoff_altitude_none_legacy_zone(self):
        """None → legacy behavior: takeoff climbs to the full zone altitude."""
        track = _make_track(2)
        wp = build_mission(track, 150)
        self.assertEqual(wp.wp(1).z, 150)

    def test_takeoff_altitude_none_legacy_corridor(self):
        """None + corridor → legacy behavior: takeoff climbs to corridor alt."""
        track = _make_track(5)
        wp = build_mission(track, 200, corridor_count=2, corridor_altitude_m=120)
        self.assertEqual(wp.wp(1).z, 120)


if __name__ == "__main__":
    unittest.main()
