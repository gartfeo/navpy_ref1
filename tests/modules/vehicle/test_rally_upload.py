"""The rally table travels in its own mission namespace, apart from the flight mission."""
from __future__ import annotations

import unittest
from types import SimpleNamespace

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_NAV_RALLY_POINT,
    MAV_MISSION_TYPE_RALLY,
)

from gcs.backend.planner.rally_builder import build_rally_items
from navpy.plugins.vehicle.ardupilot.vehicle_public_mission import VehicleRallyFacet
from tests.modules.vehicle.test_fence_upload import MissionHarness, _seed_loader


def _facet(harness: MissionHarness) -> VehicleRallyFacet:
    facet = VehicleRallyFacet()
    facet._parts = SimpleNamespace(mission=harness.mission)
    return facet


class TestRallyTransfer(unittest.TestCase):
    def test_upload_rally_uses_the_rally_namespace_and_keeps_the_mission(self):
        harness = MissionHarness()
        harness.mission.load_items(_seed_loader((1.0, 2.0, 3.0), (1.1, 2.1, 3.0)))
        recorded = {"items": []}
        harness.install_upload_handshake(1, recorded)

        ok = _facet(harness).upload_rally(build_rally_items({"lat": 1.5, "lon": 2.5}, 100.0, 3))

        self.assertTrue(ok)
        self.assertEqual(recorded["count"], (1, MAV_MISSION_TYPE_RALLY))
        self.assertEqual(recorded["items"], [(0, MAV_CMD_NAV_RALLY_POINT, MAV_MISSION_TYPE_RALLY)])
        self.assertEqual(harness.mission.count, 2)

    def test_clear_rally_does_not_wipe_mission(self):
        harness = MissionHarness()
        harness.mission.load_items(_seed_loader((1.0, 2.0, 3.0)))

        _facet(harness).clear_rally()

        harness.mav.mission_clear_all_send.assert_called_once_with(3, 0, MAV_MISSION_TYPE_RALLY)
        self.assertEqual(harness.mission.count, 1)
