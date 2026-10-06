"""Tests for bitmask encoding/decoding and ordinal resolution in NavigationPoiArgs."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from navpy.args.navigation_poi_args import (
    NavigationPoiArgs,
    decode_wp_bitmask,
    encode_wp_bitmask,
    nav_wp_indices,
)


# ── encode / decode helpers ──────────────────────────────────────────

class TestEncodeWpBitmask:
    def test_single_wp(self):
        assert encode_wp_bitmask([5]) == 16  # WP 5 → bit 4 → 1 << 4

    def test_multiple_wps(self):
        assert encode_wp_bitmask([5, 6, 7]) == 112  # (1<<4)|(1<<5)|(1<<6)

    def test_empty_list(self):
        assert encode_wp_bitmask([]) == 0

    def test_wp_one(self):
        assert encode_wp_bitmask([1]) == 1  # WP 1 → bit 0

    def test_wp_24(self):
        assert encode_wp_bitmask([24]) == 1 << 23  # WP 24 → bit 23

    def test_out_of_range_ignored(self):
        assert encode_wp_bitmask([0, -1, 25]) == 0  # 0 and negatives invalid

    def test_mixed_valid_invalid(self):
        assert encode_wp_bitmask([0, 5, 30]) == 16  # only WP 5 valid


class TestDecodeWpBitmask:
    def test_single_bit(self):
        assert decode_wp_bitmask(16) == [5]  # bit 4 → WP 5

    def test_multiple_bits(self):
        assert decode_wp_bitmask(112) == [5, 6, 7]

    def test_zero(self):
        assert decode_wp_bitmask(0) == []

    def test_all_bits(self):
        full = (1 << 24) - 1
        assert decode_wp_bitmask(full) == list(range(1, 25))


class TestRoundTrip:
    @pytest.mark.parametrize("wp_numbers", [
        [5], [5, 6, 7], [1, 24], [1, 2, 3, 4], [],
    ])
    def test_encode_decode_roundtrip(self, wp_numbers):
        assert decode_wp_bitmask(encode_wp_bitmask(wp_numbers)) == sorted(wp_numbers)


# ── nav_wp_indices ───────────────────────────────────────────────────

def _make_vehicle(mission_count=10, metadata_seqs=None):
    """Create mock vehicle. metadata_seqs marks which seqs are DO_SET_ROI_LOCATION (195)."""
    metadata_seqs = set(metadata_seqs or [])
    v = Mock()
    v.mission_items_count = mission_count
    v.get_param_or_default = Mock(side_effect=lambda name, default: default)

    def _get_mi(seq):
        if 0 <= seq < mission_count:
            wp = Mock()
            wp.command = 195 if seq in metadata_seqs else 16  # 16 = NAV_WAYPOINT
            return wp
        return None
    v.get_mission_item = Mock(side_effect=_get_mi)
    return v


def _make_args(**overrides):
    args = SimpleNamespace(AAS_TARG_WPS='4', AAS_TARG_ALT=150)
    for k, v in overrides.items():
        setattr(args, k, v)
    return args


class TestNavWpIndices:
    def test_all_nav_waypoints(self):
        """Skips HOME at seq 0, returns remaining nav WPs."""
        vehicle = _make_vehicle(mission_count=5)
        assert nav_wp_indices(vehicle) == [1, 2, 3, 4]

    def test_skips_non_nav_items(self):
        vehicle = _make_vehicle(mission_count=6, metadata_seqs={2, 4})
        assert nav_wp_indices(vehicle) == [1, 3, 5]

    def test_empty_mission(self):
        vehicle = _make_vehicle(mission_count=0)
        assert nav_wp_indices(vehicle) == []


# ── NavigationPoiArgs.refresh() ─────────────────────────────────────

class TestRefreshRouting:
    def test_str_input_comma_separated(self):
        """CLI string '4,5,6' resolved as 1-based WP numbers."""
        vehicle = _make_vehicle()
        args = _make_args(AAS_TARG_WPS='4,5,6')
        gta = NavigationPoiArgs(args, vehicle)
        # nav_indices = [1..9], WP 4 → seq 4, WP 5 → seq 5, WP 6 → seq 6
        assert gta.poi_wp_indices == {4: 4, 5: 5, 6: 6}
        assert gta.poi_count == 3

    def test_str_input_single(self):
        """CLI string '4' resolved as 1-based WP number."""
        vehicle = _make_vehicle()
        args = _make_args(AAS_TARG_WPS='4')
        gta = NavigationPoiArgs(args, vehicle)
        assert gta.poi_wp_indices == {4: 4}

    def test_float_input_bitmask(self):
        """MAVLink float 112.0 decoded as bitmask → WPs {5, 6, 7}."""
        vehicle = _make_vehicle()
        vehicle.get_param_or_default = Mock(
            side_effect=lambda name, default: 112.0 if name == 'AAS_TARG_WPS' else default
        )
        args = _make_args()  # CLI value == default, so MAVLink value used
        gta = NavigationPoiArgs(args, vehicle)
        # bits 4,5,6 → WPs 5,6,7 → seq 5, seq 6, seq 7
        assert gta.poi_wp_indices == {5: 5, 6: 6, 7: 7}

    def test_float_input_single_wp(self):
        """MAVLink float 16.0 decoded as bitmask → WP {5}."""
        vehicle = _make_vehicle()
        vehicle.get_param_or_default = Mock(
            side_effect=lambda name, default: 16.0 if name == 'AAS_TARG_WPS' else default
        )
        args = _make_args()
        gta = NavigationPoiArgs(args, vehicle)
        # bit 4 → WP 5 → seq 5
        assert gta.poi_wp_indices == {5: 5}

    def test_out_of_range_wps_dropped(self):
        """WP numbers beyond nav WP count are dropped."""
        vehicle = _make_vehicle(mission_count=5)  # nav WPs: [1,2,3,4] → WPs 1-4
        args = _make_args(AAS_TARG_WPS='2,6,8')
        gta = NavigationPoiArgs(args, vehicle)
        # WP 2 → seq 2, WPs 6,8 out of range (only 4 nav WPs)
        assert gta.poi_wp_indices == {2: 2}
        assert gta.poi_count == 1


# ── Ordinal resolution (non-nav items skipped) ──────────────────────

class TestOrdinalResolution:
    """1-based WP numbers resolve to mission indices, skipping HOME and non-NAV_WAYPOINT items."""

    def test_wps_skip_metadata_items(self):
        """WP numbers count only nav WPs, skipping HOME and DO/ROI items."""
        # Mission: 0(HOME),1(WP),2(WP),3(ROI),4(ROI),5(WP),6(WP),7(WP),8(WP),9(WP)
        vehicle = _make_vehicle(mission_count=10, metadata_seqs={3, 4})
        # nav_indices = [1, 2, 5, 6, 7, 8, 9]
        args = _make_args(AAS_TARG_WPS='4,5,6')
        gta = NavigationPoiArgs(args, vehicle)
        # WP 4 → nav_indices[3]=6, WP 5 → nav_indices[4]=7, WP 6 → nav_indices[5]=8
        assert gta.poi_wp_indices == {4: 6, 5: 7, 6: 8}
        assert gta.poi_count == 3

    def test_bitmask_wps_skip_metadata(self):
        """Bitmask-decoded WP numbers also resolve through nav WP indices."""
        # bitmask = (1<<3)|(1<<4)|(1<<5) = 56 → WPs 4,5,6
        vehicle = _make_vehicle(mission_count=10, metadata_seqs={3, 4})
        vehicle.get_param_or_default = Mock(
            side_effect=lambda name, default: 56.0 if name == 'AAS_TARG_WPS' else default
        )
        args = _make_args()
        gta = NavigationPoiArgs(args, vehicle)
        # WP 4 → nav_indices[3]=6, WP 5 → nav_indices[4]=7, WP 6 → nav_indices[5]=8
        assert gta.poi_wp_indices == {4: 6, 5: 7, 6: 8}

    def test_no_metadata_wp_matches_seq(self):
        """When all items are NAV_WAYPOINT, WP N maps to mission seq N."""
        vehicle = _make_vehicle(mission_count=10)
        args = _make_args(AAS_TARG_WPS='4,5,6')
        gta = NavigationPoiArgs(args, vehicle)
        # nav_indices = [1..9], WP 4 → seq 4, WP 5 → seq 5, WP 6 → seq 6
        assert gta.poi_wp_indices == {4: 4, 5: 5, 6: 6}

    def test_wps_beyond_nav_count_dropped(self):
        """WP numbers exceeding nav WP count are dropped."""
        # 10 items, 2 metadata → 7 nav WPs after HOME skip (WPs 1-7)
        vehicle = _make_vehicle(mission_count=10, metadata_seqs={4, 5})
        args = _make_args(AAS_TARG_WPS='8,9')
        gta = NavigationPoiArgs(args, vehicle)
        assert gta.poi_wp_indices == {}
        assert gta.poi_count == 0
