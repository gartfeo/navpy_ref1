import os
import time
import unittest

from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink_mission_item_int_message, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    MAV_CMD_NAV_TAKEOFF, MAV_CMD_NAV_LOITER_UNLIM, MAV_CMD_NAV_RETURN_TO_LAUNCH
)

from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vehicle.vehicle_mav import VehicleMav
from navpy.modules.vehicle.flight_mode import FlightMode
from navpy.logger.cache_logger import ConsoleLogger
from navpy.modules.common.models.location import Location


def _split_env(varname: str):
    raw = os.getenv(varname, "")
    return [s.strip() for s in raw.split(",") if s.strip()]


class VehicleMavLiveTest(unittest.TestCase):
    """
    Live-connection integration tests.

    • Set env var  DRONEMAV_CONN="conn1[,conn2]"  before running. ex. DRONEMAV_CONN=udp:0.0.0.0:14560,udp:0.0.0.0:14570
    • If absent → entire class is skipped.
    """

    CONNECTIONS = _split_env("DRONEMAV_CONN")

    @classmethod
    def setUpClass(cls):
        if not cls.CONNECTIONS:
            raise unittest.SkipTest("DRONEMAV_CONN not provided – skipping live tests")

        cls.vehicles: list[IVehicle] = []
        for idx, conn_str in enumerate(cls.CONNECTIONS, start=1):
            try:
                dm = VehicleMav(device=conn_str, target_system=idx, baud=115200, logger=ConsoleLogger(),
                                skip_mission_download=True)
                # wait until we have at least one heartbeat / mode
                t0 = time.time()
                while dm.get_mode is None and time.time() - t0 < 10:
                    time.sleep(0.2)
                cls.vehicles.append(dm)
            except Exception as exc:
                raise unittest.SkipTest(f"Cannot connect to '{conn_str}': {exc}")

    @classmethod
    def tearDownClass(cls):
        for dm in getattr(cls, "vehicles", []):
            dm.close()

    # ------------------------------------------------------------------
    # ---------- single-vehicle coverage (always executed) -------------
    # ------------------------------------------------------------------
    def test_full_api_single_vehicle(self):
        vehicle = self.vehicles[0]

        # telemetry
        _ = vehicle.battery_level
        _ = vehicle.velocity
        _ = vehicle.air_speed
        _ = vehicle.ground_speed
        _ = vehicle.heading

        # parameter access
        sys_id = int(vehicle.get_parameter("SYSID_THISMAV"))
        self.assertEqual(sys_id, vehicle.target_system)

        # angle limit helpers
        _ = vehicle.max_pitch
        _ = vehicle.min_pitch
        _ = vehicle.lim_roll

        # attitude setters
        vehicle.set_attitude(roll=0.0, pitch=0.0, yaw=0.0, thr=0.5)

        # mode round-trip
        vehicle.set_mode(FlightMode.GUIDED)
        time.sleep(1)
        self.assertEqual(vehicle.get_mode, FlightMode.GUIDED)

        # goto
        here = vehicle.location(is_relative=False)
        self.assertIsNotNone(here, "location() returned None")
        d_lat = 3 / 111_111.0  # ~3 m north
        target = Location(here.lat + d_lat, here.lng, here.alt, is_absolute=True)
        vehicle.goto(target)

        # mission upload / download basic
        wp = MAVLink_mission_item_int_message(
            vehicle.target_system,
            0,
            0,
            MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            MAV_CMD_NAV_TAKEOFF,
            0,
            1,
            0, 0, 0, 0,
            int(here.lat * 1e7),
            int(here.lng * 1e7),
            here.alt + 10,
            0,
        )
        vehicle.clear_mission()
        vehicle.update_mission_item(0, wp)
        self.assertTrue(vehicle.upload_mission(), "mission upload failed")
        self.assertEqual(vehicle.mission_items_count, 1)
        self.assertEqual(vehicle.download_mission(), 1, "mission download failed")

        # RC callback path
        captured = []
        vehicle.register_rc_channel(3, lambda v: captured.append(v))
        mock_msg = type(
            "MockMsg",
            (),
            {
                "chan3_raw": 1600,
                "get_msgId": lambda _self: 65,
                "get_srcSystem": lambda _self: vehicle.target_system,
                "get_srcComponent": lambda _self: 1,
                "get_seq": lambda _self: 0,
                "get_type": lambda _self: "RC_CHANNELS",
            },
        )()
        vehicle.feed_message(mock_msg)
        self.assertIn(1600, captured)

        # logging helper
        vehicle.send_status_text("live-test complete")

    # ------------------------------------------------------------------
    # ---------- multi-vehicle independence check ----------------------
    # ------------------------------------------------------------------
    @unittest.skipUnless(len(CONNECTIONS) >= 2, "only one connection string supplied")
    def test_two_vehicle_mode_independence(self):
        vehicle1, vehicle2 = self.vehicles[:2]

        # Force distinct starting modes
        vehicle1.set_mode(FlightMode.GUIDED)
        vehicle2.set_mode(FlightMode.MANUAL)
        time.sleep(1)

        # Change mode on vehicle1 only
        vehicle1.set_mode(FlightMode.LOITER)
        time.sleep(1)

        self.assertEqual(vehicle1.get_mode, FlightMode.LOITER)
        self.assertNotEqual(vehicle2.get_mode, FlightMode.LOITER)

    # ------------------------------------------------------------------
    # ---------- multi-vehicle mission independence -------------------
    # ------------------------------------------------------------------
    @unittest.skipUnless(len(CONNECTIONS) >= 2, "only one connection string supplied")
    def test_two_vehicle_mission_independence(self):
        vehicle1, vehicle2 = self.vehicles[:2]

        # Build two distinct waypoints
        home = vehicle1.location(is_relative=False)
        wp1 = MAVLink_mission_item_int_message(
            vehicle1.target_system, 0,
            0,
            MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            MAV_CMD_NAV_LOITER_UNLIM, 0, 1,
            0, 0, 0, 0,
            int(home.lat * 1e7), int(home.lng * 1e7), home.alt, 0,
        )
        wp2 = MAVLink_mission_item_int_message(
            vehicle2.target_system, 0,
            0,
            MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 1,
            0, 0, 0, 0,
            int(home.lat * 1e7), int(home.lng * 1e7), home.alt, 0,
        )

        # Upload on both
        vehicle1.clear_mission()
        vehicle2.clear_mission()
        vehicle1.update_mission_item(0, wp1)
        vehicle2.update_mission_item(0, wp2)

        self.assertTrue(vehicle1.upload_mission(), "vehicle1 mission upload failed")
        self.assertTrue(vehicle2.upload_mission(), "vehicle2 mission upload failed")

        self.assertEqual(vehicle1.mission_items_count, 1)
        self.assertEqual(vehicle2.mission_items_count, 1)

        # Clear only vehicle1 ⇒ vehicle2 still has its mission
        vehicle1.clear_mission()
        self.assertEqual(vehicle1.mission_items_count, 0)
        self.assertEqual(vehicle2.mission_items_count, 1)

    # ------------------------------------------------------------------
    # ---------- local _mission loader independence --------------------
    # ------------------------------------------------------------------
    @unittest.skipUnless(len(CONNECTIONS) >= 2, "only one connection string supplied")
    def test_local_loader_independence(self):
        vehicle1, vehicle2 = self.vehicles[:2]

        # wipe both local caches
        vehicle1.clear_mission()
        vehicle2.clear_mission()

        # give each a different local command
        wpA = MAVLink_mission_item_int_message(
            vehicle1.target_system, 0, 0, 0, 16, 0, 1,
            1, 2, 3, 4, 100, 200, 300, 0
        )
        wpB = MAVLink_mission_item_int_message(
            vehicle2.target_system, 0, 0, 0, 17, 0, 1,
            5, 6, 7, 8, 400, 500, 600, 0
        )

        vehicle1.update_mission_item(0, wpA)
        vehicle2.update_mission_item(0, wpB)

        mi1 = vehicle1.get_mission_item(0)
        mi2 = vehicle2.get_mission_item(0)

        self.assertEqual(mi1.command, 16)
        self.assertEqual(mi2.command, 17)

    # ------------------------------------------------------------------
    # ------- new: multi-vehicle status & parameter independence -------
    # ------------------------------------------------------------------
    @unittest.skipUnless(len(CONNECTIONS) >= 2, "only one connection string supplied")
    def test_two_vehicle_status_and_parameter_independence(self):
        vehicle1, vehicle2 = self.vehicles[:2]

        # ---- status (mode) independence ----
        # record initial
        orig1 = vehicle1.get_mode
        orig2 = vehicle2.get_mode
        # change vehicle1
        vehicle1.set_mode(FlightMode.STABILIZE)
        time.sleep(1)
        self.assertEqual(vehicle1.get_mode, FlightMode.STABILIZE)
        self.assertNotEqual(vehicle2.get_mode, FlightMode.STABILIZE)

        # revert vehicle1
        vehicle1.set_mode(FlightMode.GUIDED)
        time.sleep(1)
        self.assertEqual(vehicle1.get_mode, FlightMode.GUIDED)

        # ---- parameter independence ----
        p1 = vehicle1.get_parameter("SYSID_THISMAV")
        p2 = vehicle2.get_parameter("SYSID_THISMAV")
        # vehicle1.vehicle_sender == 1, vehicle2.vehicle_sender == 2
        self.assertEqual(p1, 1)
        self.assertEqual(p2, 2)

        # clear vehicle1 cache only
        vehicle1._param_cache.clear()
        # vehicle2 cache still has its entry
        self.assertIn("SYSID_THISMAV", vehicle2._param_cache)


if __name__ == "__main__":
    unittest.main(verbosity=2)
