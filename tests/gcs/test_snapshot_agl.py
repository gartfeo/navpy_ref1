"""Tests for alt_rel in VehicleEntry.snapshot()."""
from unittest.mock import MagicMock

from gcs.backend.vehicle_manager import VehicleEntry


def _make_vehicle(alt_msl=None, alt_rel=None):
    """Create a mock VehicleMav with configurable altitudes."""
    v = MagicMock()
    loc_abs = MagicMock() if alt_msl is not None else None
    loc_rel = MagicMock() if alt_rel is not None else None
    if loc_abs:
        loc_abs.lat, loc_abs.lng, loc_abs.alt = 32.0, 34.0, alt_msl
    if loc_rel:
        loc_rel.alt = alt_rel
    v.location = lambda is_relative: loc_rel if is_relative else loc_abs
    v.get_mode = None
    v.attitude = None
    v.is_armed = False
    v.battery_level = 80
    v.heading = 90
    v.ground_speed = 15.0
    v.link_ok = True
    v.mission_items_next = 0
    v.mission_items_count = 10
    return v


class TestSnapshotAltRel:
    def test_alt_rel_present(self):
        v = _make_vehicle(alt_msl=250.0, alt_rel=150.0)
        snap = VehicleEntry(1, v, "uav1").snapshot()
        assert snap["alt"] == 250.0
        assert snap["alt_rel"] == 150.0

    def test_alt_rel_none_without_location(self):
        v = _make_vehicle(alt_msl=None, alt_rel=None)
        snap = VehicleEntry(1, v, "uav1").snapshot()
        assert snap["alt"] is None
        assert snap["alt_rel"] is None


class TestSnapshotClimb:
    def test_climb_present(self):
        v = _make_vehicle(alt_msl=250.0, alt_rel=150.0)
        v.climb_rate = 2.5
        snap = VehicleEntry(1, v, "uav1").snapshot()
        assert snap["climb"] == 2.5


class TestSnapshotThrottle:
    def test_throttle_present(self):
        v = _make_vehicle(alt_msl=250.0, alt_rel=150.0)
        v.throttle_pct = 75.0
        snap = VehicleEntry(1, v, "uav1").snapshot()
        assert snap["throttle"] == 75.0
