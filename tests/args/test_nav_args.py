"""Tests for NavArgs min_wp ordinal resolution."""

from types import SimpleNamespace
from unittest.mock import Mock

from navpy.args.nav_args import NavArgs


def _make_vehicle(mission_count=10, metadata_seqs=None):
    """Create mock vehicle. metadata_seqs marks which seqs are non-NAV_WAYPOINT (195)."""
    metadata_seqs = set(metadata_seqs or [])
    v = Mock()
    v.mission_items_count = mission_count
    v.get_param_or_default = Mock(side_effect=lambda name, default: default)

    def _get_mi(seq):
        if 0 <= seq < mission_count:
            wp = Mock()
            wp.command = 195 if seq in metadata_seqs else 16
            return wp
        return None
    v.get_mission_item = Mock(side_effect=_get_mi)
    return v


def _make_args(**overrides):
    # Bare _make_args() means "no CLI override"; every kwarg is treated as an
    # explicit CLI arg (recorded in _cli_overrides, like StoreWithFlag does).
    defaults = dict(NavArgs.PARAMS, nav_sim_speedup=0.0)
    defaults.update(overrides)
    ns = SimpleNamespace(**defaults)
    ns._cli_overrides = set(overrides) & set(NavArgs.PARAMS)
    return ns


class TestMinWpOrdinalResolution:
    def test_all_nav_waypoints(self):
        """When all items are NAV_WAYPOINT, WP N maps to mission seq N."""
        vehicle = _make_vehicle(mission_count=10)
        args = _make_args(AAS_NAV_LAST_WP=4)
        na = NavArgs(args, vehicle)
        # nav_indices = [1..9], WP 4 → nav_indices[3] = 4
        assert na.min_wp == 4

    def test_skips_non_nav_items(self):
        """1-based WP resolves through nav WPs, skipping HOME and metadata."""
        # Mission: 0(HOME),1(WP),2(WP),3(ROI),4(ROI),5(WP),6(WP),7(WP),8(WP),9(WP)
        # nav_indices = [1, 2, 5, 6, 7, 8, 9]
        vehicle = _make_vehicle(mission_count=10, metadata_seqs={3, 4})
        args = _make_args(AAS_NAV_LAST_WP=4)
        na = NavArgs(args, vehicle)
        # WP 4 → nav_indices[3] = 6
        assert na.min_wp == 6

    def test_out_of_range_fallback(self):
        """Out-of-range WP number falls back to clamped value."""
        vehicle = _make_vehicle(mission_count=3)  # nav WPs: [1, 2] → WPs 1-2
        args = _make_args(AAS_NAV_LAST_WP=5)
        na = NavArgs(args, vehicle)
        assert na.min_wp == 2  # clamped to mission_count - 1

    def test_empty_mission_fallback(self):
        """Empty mission falls back to 0."""
        vehicle = _make_vehicle(mission_count=0)
        args = _make_args(AAS_NAV_LAST_WP=4)
        na = NavArgs(args, vehicle)
        assert na.min_wp == 0


class TestConfirmOnFailDefault:
    def test_default_is_reject(self):
        """MISS-04: confirm-on-fail defaults to REJECT — a confirm window that
        expires without an operator response must never auto-commit the target
        unless CONFIRM was explicitly configured (CLI -tcfc or AAS_NAV_CM_FL)."""
        na = NavArgs(_make_args(), _make_vehicle())
        assert NavArgs.PARAMS['AAS_NAV_CM_FL'] is False
        assert na.is_confirm_on_fail is False

    def test_explicit_cli_confirm_still_works(self):
        """Explicit -tcfc true keeps the old CONFIRM behavior available."""
        na = NavArgs(_make_args(AAS_NAV_CM_FL=True), _make_vehicle())
        assert na.is_confirm_on_fail is True

    def test_explicit_cli_reject_beats_mavlink_param(self):
        """-tcfc false must win over a stored AAS_NAV_CM_FL=1 MAVLink param:
        CLI presence is tracked via _cli_overrides, not value != default."""
        vehicle = _make_vehicle()
        vehicle.get_param_or_default = Mock(return_value=True)
        na = NavArgs(_make_args(AAS_NAV_CM_FL=False), vehicle)
        assert na.is_confirm_on_fail is False

    def test_mavlink_param_beats_default_without_cli(self):
        """No CLI arg: an operator-set AAS_NAV_CM_FL=1 param still wins."""
        vehicle = _make_vehicle()
        vehicle.get_param_or_default = Mock(return_value=True)
        na = NavArgs(_make_args(), vehicle)
        assert na.is_confirm_on_fail is True

    def test_argparse_records_cli_overrides(self):
        """Real parser: -tcfc false lands in _cli_overrides; untouched args
        do not, so their MAVLink params stay authoritative."""
        import argparse
        parser = argparse.ArgumentParser()
        NavArgs.add_args(parser)
        args = parser.parse_args(['-tcfc', 'false', '-nos'])
        assert args._cli_overrides == {'AAS_NAV_CM_FL', 'AAS_NAV_ONESHOT'}
        assert args.AAS_NAV_CM_FL is False
        assert args.AAS_NAV_ONESHOT is True
