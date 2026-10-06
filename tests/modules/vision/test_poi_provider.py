"""Tests for PoiProvider — mission modification behaviour."""

import unittest
from unittest.mock import Mock, patch, PropertyMock

from navpy.modules.common.models.location import Location
from navpy.modules.vision.poi_provider import PoiProvider


def _make_mocks(poi_alt=150, poi_wp_indices=None):
    """Build mock args, vehicle, zc_util, logger for PoiProvider construction."""
    if poi_wp_indices is None:
        poi_wp_indices = {4: 4}  # ordinal 4 → mission index 4

    args = Mock()
    args.AAS_TARG_WPS = '4'
    args.AAS_TARG_ALT = poi_alt

    vehicle = Mock()
    vehicle.home_location = Location(32.0, 34.0, 100.0)
    vehicle.mission_items_count = 6

    wp = Mock(command=16)  # MAV_CMD_NAV_WAYPOINT = 16
    vehicle.get_mission_item.return_value = wp

    wp_loc = Location(32.5, 34.5, 200.0, is_absolute=False)
    vehicle.get_mission_item_location.return_value = wp_loc

    vehicle.get_param_or_default.side_effect = lambda name, default: default

    zc_util = None  # no terrain correction
    logger = Mock()

    return args, vehicle, zc_util, logger


def _provider_args(use_terrain=None):
    args = Mock()
    if use_terrain is not None:
        args.AAS_USE_TRN = use_terrain
    return args


class TestRefreshDoesNotModifyMission(unittest.TestCase):
    """Normal refresh (location=None) must never call update_mission_item_location or upload_mission."""

    @patch('navpy.modules.vision.poi_provider.NavigationPoiArgs')
    def test_refresh_does_not_modify_mission(self, mock_args_cls):
        args_inst = Mock()
        args_inst.poi_alt = 150.0
        args_inst.poi_wp_indices = {4: 4}
        args_inst.refresh = Mock()
        mock_args_cls.return_value = args_inst

        _, vehicle, zc_util, logger = _make_mocks()

        tp = PoiProvider(_provider_args(), vehicle, zc_util, logger)
        vehicle.reset_mock()

        tp.refresh()

        vehicle.update_mission_item_location.assert_not_called()
        vehicle.upload_mission.assert_not_called()

    @patch('navpy.modules.vision.poi_provider.NavigationPoiArgs')
    def test_init_does_not_modify_mission(self, mock_args_cls):
        args_inst = Mock()
        args_inst.poi_alt = 150.0
        args_inst.poi_wp_indices = {4: 4}
        mock_args_cls.return_value = args_inst

        _, vehicle, zc_util, logger = _make_mocks()

        PoiProvider(Mock(), vehicle, zc_util, logger)

        vehicle.update_mission_item_location.assert_not_called()
        vehicle.upload_mission.assert_not_called()


class TestSetSimPoiDoesNotModifyMission(unittest.TestCase):
    """set_sim_poi must NOT modify or re-upload the vehicle mission."""

    @patch('navpy.modules.vision.poi_provider.NavigationPoiArgs')
    def test_set_sim_poi_does_not_modify_mission(self, mock_args_cls):
        args_inst = Mock()
        args_inst.poi_alt = 150.0
        args_inst.poi_wp_indices = {}
        mock_args_cls.return_value = args_inst

        _, vehicle, zc_util, logger = _make_mocks()

        tp = PoiProvider(_provider_args(), vehicle, zc_util, logger)
        vehicle.reset_mock()

        sim_loc = Location(33.0, 35.0, 0.0)
        tp.set_sim_poi(3, sim_loc)

        vehicle.update_mission_item_location.assert_not_called()
        vehicle.upload_mission.assert_not_called()

    @patch('navpy.modules.vision.poi_provider.NavigationPoiArgs')
    def test_set_sim_poi_adds_poi_with_correct_location(self, mock_args_cls):
        args_inst = Mock()
        args_inst.poi_alt = 150.0
        args_inst.poi_wp_indices = {}
        mock_args_cls.return_value = args_inst

        _, vehicle, zc_util, logger = _make_mocks()

        tp = PoiProvider(_provider_args(), vehicle, zc_util, logger)

        sim_loc = Location(33.0, 35.0, 0.0)
        tp.set_sim_poi(3, sim_loc)

        self.assertEqual(len(tp.pois), 1)
        poi = tp.pois[0]
        self.assertAlmostEqual(poi.g_loc.lat, 33.0)
        self.assertAlmostEqual(poi.g_loc.lng, 35.0)


class TestSimPoiAltitude(unittest.TestCase):
    """Verify that set_sim_poi uses the provided location's altitude, not configured poi_alt."""

    @patch('navpy.modules.vision.poi_provider.NavigationPoiArgs')
    def test_sim_poi_ground_altitude(self, mock_args_cls):
        """set_sim_poi with alt=0 places POI at ground level (home_alt), not home_alt + poi_alt."""
        args_inst = Mock()
        args_inst.poi_alt = 150.0
        args_inst.poi_wp_indices = {}
        mock_args_cls.return_value = args_inst

        _, vehicle, zc_util, logger = _make_mocks()

        tp = PoiProvider(_provider_args(), vehicle, zc_util, logger)

        sim_loc = Location(33.0, 35.0, 0.0)
        tp.set_sim_poi(3, sim_loc)

        poi = tp.pois[0]
        # alt=0 → g_loc.alt = home_alt(100) + 0 = 100 (ground level)
        self.assertAlmostEqual(poi.g_loc.alt, 100.0)
        self.assertEqual(poi.height, 0.0)

    @patch('navpy.modules.vision.poi_provider.NavigationPoiArgs')
    def test_sim_poi_preserves_nonzero_alt(self, mock_args_cls):
        """set_sim_poi with alt=50 places POI at home_alt + 50."""
        args_inst = Mock()
        args_inst.poi_alt = 150.0
        args_inst.poi_wp_indices = {}
        mock_args_cls.return_value = args_inst

        _, vehicle, zc_util, logger = _make_mocks()

        tp = PoiProvider(_provider_args(), vehicle, zc_util, logger)

        sim_loc = Location(33.0, 35.0, 50.0)
        tp.set_sim_poi(3, sim_loc)

        poi = tp.pois[0]
        # alt=50 → g_loc.alt = home_alt(100) + 50 = 150
        self.assertAlmostEqual(poi.g_loc.alt, 150.0)
        self.assertEqual(poi.height, 50.0)


class TestRefreshDoesNotDownload(unittest.TestCase):
    """refresh() must never trigger a mission download."""

    @patch('navpy.modules.vision.poi_provider.NavigationPoiArgs')
    def test_refresh_does_not_download(self, mock_args_cls):
        args_inst = Mock()
        args_inst.poi_alt = 150.0
        args_inst.poi_wp_indices = {4: 4}
        args_inst.refresh = Mock()
        mock_args_cls.return_value = args_inst

        _, vehicle, zc_util, logger = _make_mocks()

        tp = PoiProvider(_provider_args(), vehicle, zc_util, logger)
        vehicle.reset_mock()

        tp.refresh()

        vehicle.download_mission.assert_not_called()


class TestPoiAltitudeComputed(unittest.TestCase):
    """Verify POI objects receive correct terrain-relative altitude."""

    @patch('navpy.modules.vision.poi_provider.NavigationPoiArgs')
    def test_poi_altitude_no_terrain(self, mock_args_cls):
        """Without zc_util, POI altitude = home_alt + poi_alt."""
        args_inst = Mock()
        args_inst.poi_alt = 150.0
        args_inst.poi_wp_indices = {4: 4}
        mock_args_cls.return_value = args_inst

        _, vehicle, zc_util, logger = _make_mocks()
        # zc_util is None → no terrain correction

        tp = PoiProvider(Mock(), vehicle, zc_util, logger)

        self.assertEqual(len(tp.pois), 1)
        poi = tp.pois[0]
        # alt should be home_alt (100) + poi_alt (150) = 250
        self.assertAlmostEqual(poi.g_loc.alt, 250.0)
        self.assertEqual(poi.height, 150.0)
        self.assertTrue(poi.g_loc.is_absolute)

    @patch('navpy.modules.vision.poi_provider.NavigationPoiArgs')
    def test_poi_altitude_terrain_disabled_ignores_terrain_sources(self, mock_args_cls):
        """Disabled terrain must not let environment altitude shift certification POIs."""
        args_inst = Mock()
        args_inst.poi_alt = 150.0
        args_inst.poi_wp_indices = {4: 4}
        mock_args_cls.return_value = args_inst

        _, vehicle, _, logger = _make_mocks()
        vehicle.terrain_height_at = Mock(return_value=321.25)

        zc_util = Mock()
        zc_util.get_elevation.side_effect = lambda coords: 80.0

        tp = PoiProvider(_provider_args(use_terrain=False), vehicle, zc_util, logger)

        poi = tp.pois[0]
        vehicle.terrain_height_at.assert_not_called()
        zc_util.get_elevation.assert_not_called()
        self.assertAlmostEqual(poi.g_loc.alt, 250.0)

    @patch('navpy.modules.vision.poi_provider.NavigationPoiArgs')
    def test_poi_altitude_bare_mock_args_do_not_enable_terrain(self, mock_args_cls):
        """Mock-created attributes must not accidentally enable terrain correction."""
        args_inst = Mock()
        args_inst.poi_alt = 150.0
        args_inst.poi_wp_indices = {4: 4}
        mock_args_cls.return_value = args_inst

        _, vehicle, _, logger = _make_mocks()
        vehicle.terrain_height_at = Mock(return_value=321.25)

        zc_util = Mock()
        zc_util.get_elevation.side_effect = lambda coords: 80.0

        tp = PoiProvider(Mock(), vehicle, zc_util, logger)

        poi = tp.pois[0]
        vehicle.terrain_height_at.assert_not_called()
        zc_util.get_elevation.assert_not_called()
        self.assertAlmostEqual(poi.g_loc.alt, 250.0)

    @patch('navpy.modules.vision.poi_provider.NavigationPoiArgs')
    def test_poi_altitude_with_terrain(self, mock_args_cls):
        """With zc_util, altitude accounts for terrain delta."""
        args_inst = Mock()
        args_inst.poi_alt = 150.0
        args_inst.poi_wp_indices = {4: 4}
        mock_args_cls.return_value = args_inst

        _, vehicle, _, logger = _make_mocks()

        zc_util = Mock()
        # home elevation 50m, POI elevation 80m -> delta = +30
        zc_util.get_elevation.side_effect = lambda coords: 50.0 if coords[0] == 32.0 else 80.0

        tp = PoiProvider(_provider_args(use_terrain=True), vehicle, zc_util, logger)

        poi = tp.pois[0]
        # terrain_relative_alt = 150 + (80 - 50) = 180
        # g_loc.alt = home_alt(100) + 180 = 280
        self.assertAlmostEqual(poi.g_loc.alt, 280.0)

    @patch('navpy.modules.vision.poi_provider.NavigationPoiArgs')
    def test_poi_altitude_prefers_vehicle_terrain_height(self, mock_args_cls):
        """Sim POI altitude should match vehicle terrain when available."""
        args_inst = Mock()
        args_inst.poi_alt = 2.5
        args_inst.poi_wp_indices = {4: 4}
        mock_args_cls.return_value = args_inst

        _, vehicle, _, logger = _make_mocks()
        vehicle.terrain_height_at = Mock(return_value=321.25)

        zc_util = Mock()
        zc_util.get_elevation.side_effect = lambda coords: 50.0

        tp = PoiProvider(_provider_args(use_terrain=True), vehicle, zc_util, logger)

        poi = tp.pois[0]
        vehicle.terrain_height_at.assert_called_once_with(32.5, 34.5)
        zc_util.get_elevation.assert_not_called()
        self.assertAlmostEqual(poi.g_loc.alt, 323.75)

    @patch('navpy.modules.vision.poi_provider.NavigationPoiArgs')
    def test_poi_altitude_preserves_fractional_terrain_delta(self, mock_args_cls):
        """Terrain correction must not truncate sub-meter POI altitude."""
        args_inst = Mock()
        args_inst.poi_alt = 0.0
        args_inst.poi_wp_indices = {4: 4}
        mock_args_cls.return_value = args_inst

        _, vehicle, _, logger = _make_mocks()

        zc_util = Mock()
        zc_util.get_elevation.side_effect = (
            lambda coords: 50.25 if coords[0] == 32.0 else 51.75
        )

        tp = PoiProvider(_provider_args(use_terrain=True), vehicle, zc_util, logger)

        poi = tp.pois[0]
        self.assertAlmostEqual(poi.g_loc.alt, 101.5)

    @patch('navpy.modules.vision.poi_provider.NavigationPoiArgs')
    def test_poi_uses_waypoint_coords(self, mock_args_cls):
        """Normal refresh uses the waypoint's lat/lng, not overwritten values."""
        args_inst = Mock()
        args_inst.poi_alt = 150.0
        args_inst.poi_wp_indices = {4: 4}
        mock_args_cls.return_value = args_inst

        _, vehicle, zc_util, logger = _make_mocks()
        vehicle.get_mission_item_location.return_value = Location(31.0, 33.0, 200.0)

        tp = PoiProvider(Mock(), vehicle, zc_util, logger)

        poi = tp.pois[0]
        self.assertAlmostEqual(poi.g_loc.lat, 31.0)
        self.assertAlmostEqual(poi.g_loc.lng, 33.0)


if __name__ == '__main__':
    unittest.main()
