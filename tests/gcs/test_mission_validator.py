"""Tests for mission_validator: vehicle probe and fleet cross-validation."""
from __future__ import annotations

from unittest.mock import MagicMock

import pytest
from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_DO_JUMP,
    MAV_CMD_NAV_TAKEOFF,
    MAV_CMD_NAV_WAYPOINT,
)

from gcs.backend.mission_validator import (
    MissionProbe,
    FleetValidation,
    validate_vehicle_mission,
    validate_fleet_missions,
    parse_mission_items,
)
from gcs.backend.planner.waypoint_builder import (
    CORRIDOR_END_MARKER,
    META_POLYGON_VERTEX,
    encode_meta_z,
)


def _make_wp(command, param1=0.0, param2=0.0, x=0, y=0, z=0.0):
    """Create a mock mission item."""
    wp = MagicMock()
    wp.command = command
    wp.param1 = param1
    wp.param2 = param2
    wp.x = x
    wp.y = y
    wp.z = z
    return wp


def _make_loc(lat, lon, alt):
    """Create a mock Location."""
    loc = MagicMock()
    loc.lat = lat
    loc.lng = lon
    loc.alt = alt
    return loc


def _make_vehicle(items: list, locations: dict | None = None):
    """Create a mock vehicle that returns the given mission items.

    locations: optional dict {seq_index: Location} for get_mission_item_location.
    """
    vehicle = MagicMock()
    vehicle.download_mission.return_value = len(items)
    vehicle.get_mission_item.side_effect = lambda i: items[i] if i < len(items) else None
    vehicle.get_mission_item_location.side_effect = lambda i: (locations or {}).get(i)
    vehicle.target_system = 1
    return vehicle


class TestValidateVehicleMission:
    def test_valid_mission_with_track_waypoints(self):
        """Mission with home + takeoff + metadata + track → valid with cached dict."""
        meta_z = encode_meta_z("distributed")
        items = [
            _make_wp(MAV_CMD_NAV_WAYPOINT),                     # home (seq 0)
            _make_wp(MAV_CMD_NAV_TAKEOFF, z=100),               # takeoff
            _make_wp(CORRIDOR_END_MARKER, param1=META_POLYGON_VERTEX,
                     x=int(32.0 * 1e7), y=int(34.0 * 1e7), z=0),
            _make_wp(CORRIDOR_END_MARKER, param1=META_POLYGON_VERTEX,
                     x=int(32.1 * 1e7), y=int(34.1 * 1e7), z=meta_z),
            _make_wp(MAV_CMD_NAV_WAYPOINT, x=int(32.0 * 1e7), y=int(34.0 * 1e7), z=100),
            _make_wp(MAV_CMD_NAV_WAYPOINT, x=int(32.05 * 1e7), y=int(34.05 * 1e7), z=100),
        ]
        locations = {
            4: _make_loc(32.0, 34.0, 100),
            5: _make_loc(32.05, 34.05, 100),
        }
        vehicle = _make_vehicle(items, locations)
        probe, cached = validate_vehicle_mission(vehicle, 1)

        assert probe.valid is True
        assert probe.item_count == 6
        assert probe.track_count == 2
        assert probe.search_pattern == "distributed"
        assert len(probe.polygon) == 2
        assert abs(probe.polygon[0]["lat"] - 32.0) < 1e-5
        # Cached dict has full response
        assert cached is not None
        assert cached["sys_id"] == 1
        assert len(cached["waypoints"]) == 2
        assert cached["search_pattern"] == "distributed"

    def test_empty_mission_invalid(self):
        """Download returning 0 items is invalid."""
        vehicle = MagicMock()
        vehicle.download_mission.return_value = 0
        probe, cached = validate_vehicle_mission(vehicle, 1)

        assert probe.valid is False
        assert probe.item_count == 0
        assert probe.track_count == 0
        assert cached is None

    def test_home_only_invalid(self):
        """Mission with only a home item has no track waypoints."""
        items = [_make_wp(MAV_CMD_NAV_WAYPOINT)]  # home (seq 0)
        vehicle = _make_vehicle(items)
        probe, cached = validate_vehicle_mission(vehicle, 1)

        assert probe.valid is False
        assert probe.item_count == 1
        assert probe.track_count == 0
        assert cached is None

    def test_corridor_search_pattern_extracted(self):
        """Corridor search_pattern is correctly decoded from metadata z."""
        meta_z = encode_meta_z("corridor")
        items = [
            _make_wp(MAV_CMD_NAV_WAYPOINT),                     # home
            _make_wp(MAV_CMD_NAV_TAKEOFF, z=100),               # takeoff
            _make_wp(CORRIDOR_END_MARKER, param1=META_POLYGON_VERTEX,
                     x=int(32.0 * 1e7), y=int(34.0 * 1e7), z=meta_z),
            _make_wp(MAV_CMD_NAV_WAYPOINT, z=100),              # track wp
        ]
        locations = {3: _make_loc(32.0, 34.0, 100)}
        vehicle = _make_vehicle(items, locations)
        probe, cached = validate_vehicle_mission(vehicle, 1)

        assert probe.valid is True
        assert probe.search_pattern == "corridor"
        assert cached is not None
        assert cached["search_pattern"] == "corridor"

    def test_home_and_takeoff_only_invalid(self):
        """Mission with only home + takeoff but no track waypoints is invalid."""
        items = [
            _make_wp(MAV_CMD_NAV_WAYPOINT),         # home
            _make_wp(MAV_CMD_NAV_TAKEOFF, z=100),   # takeoff
        ]
        vehicle = _make_vehicle(items)
        probe, cached = validate_vehicle_mission(vehicle, 1)

        assert probe.valid is False
        assert probe.item_count == 2
        assert probe.track_count == 0
        assert cached is None


class TestParseMissionItemsSkipJump:
    def test_skip_jump_before_metadata_is_not_a_route_point(self):
        """The planner's DO_JUMP over the metadata block carries no route point.

        A real vehicle reports a (0, 0) location for it; it must not appear
        as a waypoint nor shift the corridor/scan boundary.
        """
        meta_z = encode_meta_z("distributed")
        items = [
            _make_wp(MAV_CMD_NAV_WAYPOINT),                                  # home
            _make_wp(MAV_CMD_NAV_TAKEOFF, z=100),                            # takeoff
            _make_wp(MAV_CMD_NAV_WAYPOINT, x=int(31.9 * 1e7), y=int(33.9 * 1e7), z=120),  # corridor
            _make_wp(MAV_CMD_DO_JUMP, param1=5.0, param2=-1.0),              # skip-jump
            _make_wp(CORRIDOR_END_MARKER, param1=META_POLYGON_VERTEX,
                     x=int(32.0 * 1e7), y=int(34.0 * 1e7), z=meta_z),
            _make_wp(MAV_CMD_NAV_WAYPOINT, x=int(32.0 * 1e7), y=int(34.0 * 1e7), z=100),  # track
        ]
        locations = {
            2: _make_loc(31.9, 33.9, 120),
            3: _make_loc(0.0, 0.0, 0.0),
            5: _make_loc(32.0, 34.0, 100),
        }
        parsed = parse_mission_items(_make_vehicle(items, locations), len(items), 1)

        assert [wp["mission_sequence"] for wp in parsed["waypoints"]] == [2, 5]
        assert parsed["corridor_end_index"] == 1
        assert len(parsed["polygon"]) == 1


class TestValidateFleetMissions:
    def test_fleet_all_valid(self):
        """All vehicles valid with same search_pattern → fleet valid."""
        probes = {
            1: MissionProbe(valid=True, item_count=6, track_count=3, search_pattern="distributed"),
            2: MissionProbe(valid=True, item_count=8, track_count=5, search_pattern="distributed"),
        }
        fleet = validate_fleet_missions(probes)

        assert fleet.valid is True
        assert fleet.zone_count == 2
        assert fleet.invalid_vehicles == []
        assert fleet.search_pattern_mismatch is False

    def test_fleet_missing_vehicle(self):
        """One vehicle invalid → fleet invalid with invalid_vehicles populated."""
        probes = {
            1: MissionProbe(valid=True, item_count=6, track_count=3, search_pattern="distributed"),
            2: MissionProbe(valid=False, item_count=0),
        }
        fleet = validate_fleet_missions(probes)

        assert fleet.valid is False
        assert fleet.invalid_vehicles == [2]
        assert fleet.zone_count == 1

    def test_fleet_search_pattern_mismatch(self):
        """Different search_patterns across vehicles → fleet invalid with search_pattern_mismatch."""
        probes = {
            1: MissionProbe(valid=True, item_count=6, track_count=3, search_pattern="distributed"),
            2: MissionProbe(valid=True, item_count=8, track_count=5, search_pattern="corridor"),
        }
        fleet = validate_fleet_missions(probes)

        assert fleet.valid is False
        assert fleet.search_pattern_mismatch is True
        assert fleet.invalid_vehicles == []

    def test_empty_probes(self):
        """No probes → fleet invalid."""
        fleet = validate_fleet_missions({})
        assert fleet.valid is False
