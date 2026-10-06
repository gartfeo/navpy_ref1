import inspect
import math
import threading
import time
import unittest
from unittest.mock import MagicMock, patch

from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import (
    ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MAV_FRAME_GLOBAL_RELATIVE_ALT,
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MAV_CMD_DO_REPOSITION,
    MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
    MAV_TYPE_CAMERA, MAV_TYPE_FIXED_WING, MAV_TYPE_GCS,
    MAV_TYPE_ONBOARD_CONTROLLER,
    MAV_MODE_FLAG_SAFETY_ARMED, MAV_MISSION_ACCEPTED,
    ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE,
    ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE,
    ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE,
    MAV_COMP_ID_AUTOPILOT1,
)

from navpy.modules.vehicle.vehicle_mav import VehicleMav
from navpy.modules.vehicle.attitude_command import euler_to_quaternion
from navpy.modules.vehicle.flight_mode import FlightMode
from navpy.modules.vehicle.inbound_router import InboundMessageRouter
from navpy.modules.vehicle.mission_inbox import MissionInbox
from navpy.logger.cache_logger import ConsoleLogger
from navpy.modules.common.models.location import Location


class VehicleMavTest(unittest.TestCase):
    _device_counter = 0

    def setUp(self):
        # fake MAVLink connection
        self.fake_mav = MagicMock()
        self.fake_mav.file = MagicMock()
        self.fake_mav_of_poi = MagicMock()
        self.fake_conn = MagicMock()
        self.fake_conn.mav = self.fake_mav
        self.fake_conn.wait_heartbeat = MagicMock()
        self.fake_conn.recv_match = MagicMock(return_value=None)
        self.fake_conn.close = MagicMock()
        self.logger = MagicMock()

        patcher = patch(
            "navpy.modules.vehicle.mav_bus.mavutil.mavlink_connection",
            return_value=self.fake_conn,
        )
        self.addCleanup(patcher.stop)
        patcher.start()

        # patch mode_string_v10 globally once
        self.mode_patch = patch(
            "navpy.modules.vehicle.mode_control.mavutil.mode_string_v10",
            return_value="MANUAL",
        )
        self.mode_patch.start()
        self.addCleanup(self.mode_patch.stop)

        mav_of_poi_patch = patch(
            "navpy.modules.vehicle.vehicle_mav.mavutil.mavlink.MAVLink",
            return_value=self.fake_mav_of_poi,
        )
        self.addCleanup(mav_of_poi_patch.stop)
        mav_of_poi_patch.start()

        self.dm = self._new_dm(device="dev", target_system=42)

    # --------------- unit tests -------------
    def test_constructor_uses_supplied_bus(self):
        bus = MagicMock()
        bus.conn = self.fake_conn
        bus.send_lock = threading.RLock()
        lease = MagicMock()
        bus.reserve.return_value = lease

        with patch("navpy.modules.vehicle.mav_bus.MavBus.get_or_create") as get_or_create:
            dm = VehicleMav(
                device="dev-supplied-bus",
                target_system=9,
                logger=ConsoleLogger(),
                skip_mission_download=True,
                wait_heartbeat=False,
                send_heartbeat=False,
                bus=bus,
            )

        self.addCleanup(dm.close)
        get_or_create.assert_not_called()
        bus.reserve.assert_called_once_with(9)
        lease.attach.assert_called_once()
        self.assertIsInstance(lease.attach.call_args.args[0], InboundMessageRouter)

    def test_battery_level_value(self):
        dm = self._new_dm(device='dev', target_system=1)
        msg = self._make_vehicle_msg(dm, "BATTERY_STATUS", battery_remaining=50)
        dm.feed_message(msg)
        self.assertEqual(dm.battery_level, 50)

    def test_battery_level_none(self):
        dm = self._new_dm(device='dev', target_system=1)
        msg = self._make_vehicle_msg(dm, "BATTERY_STATUS", battery_remaining=None)
        dm.feed_message(msg)
        self.assertIsNone(dm.battery_level)

    def test_velocity(self):
        dm = self._new_dm(device='dev', target_system=1)
        msg = self._make_vehicle_msg(
            dm, "GLOBAL_POSITION_INT", vx=100, vy=200, vz=-50,
        )
        dm.feed_message(msg)
        self.assertEqual(dm.velocity, (1.0, 2.0, -0.5))

    def test_set_mode_and_get_mode(self):
        dm = self._new_dm(device='dev', target_system=2)
        # Patch the mode_mapping on connection
        self.fake_conn.mode_mapping.return_value = {'MANUAL': 1, 'GUIDED': 4}
        dm.set_mode(FlightMode('GUIDED'))
        self.fake_conn.mav.set_mode_send.assert_called_with(
            dm.target_system,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4
        )
        # Stub module-level mode_string_v10 to return 'GUIDED'
        with patch('navpy.modules.vehicle.mode_control.mavutil.mode_string_v10', return_value='GUIDED'):
            hb = self._make_vehicle_msg(
                dm,
                "HEARTBEAT",
                type=MAV_TYPE_FIXED_WING,
                base_mode=0,
                custom_mode=4,
            )
            dm.feed_message(hb)
            self.assertEqual(dm.get_mode, FlightMode('GUIDED'))

    def test_goto_calls_send(self):
        dm = self._new_dm(device='dev', target_system=5)
        loc = Location(10.12345, 20.54321, 100.0, is_absolute=True)
        dm.goto(loc)

        self.assertEqual(self.fake_conn.mav.mission_item_send.call_count, 1)

        args = self.fake_conn.mav.mission_item_send.call_args[0]
        # indices: 0=sysid, 1=compid, 2=seq, 3=frame, 4=cmd, 5=current, 6=autocontinue, 7..10=params, 11..13=lat/lon/alt
        self.assertEqual(args[0], dm.target_system)  # <-- was args[1]
        self.assertEqual(args[3], MAV_FRAME_GLOBAL_RELATIVE_ALT)  # <-- was GLOBAL_INT
        self.assertEqual(args[4], mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)
        self.assertEqual(args[5], 2)  # current=2 (guided reposition)
        self.assertAlmostEqual(args[11], float(loc.lat), places=6)
        self.assertAlmostEqual(args[12], float(loc.lng), places=6)
        # If goto() converts abs->relative, you could also assert z matches expected rel alt

    def test_goto_loiter_sends_unverified_param_then_reposition(self):
        """goto_loiter sets WP_LOITER_RAD fire-and-forget (never blocking on the
        echo) and carries the radius in DO_REPOSITION param3."""
        dm = self._new_dm(device='dev', target_system=7)

        def assert_param_before_command(name, value):
            # The (best-effort) WP_LOITER_RAD set must be issued before the
            # DO_REPOSITION command (matters for Plane < 4.4 compat).
            self.assertEqual(name, "WP_LOITER_RAD")
            self.assertEqual(value, 80.0)
            self.fake_conn.mav.command_int_send.assert_not_called()
            return True

        with patch.object(
                dm._parts.parameters,
                'send_unverified',
                side_effect=assert_param_before_command,
        ) as send_param, patch.object(self.logger, 'info') as fake_info:
            loc = Location(10.0, 20.0, 100.0, is_absolute=False)
            dm.goto_loiter(loc, 80.0)
            send_param.assert_called_once_with("WP_LOITER_RAD", 80.0)
            fake_info.assert_called_once()

        self.fake_conn.mav.command_int_send.assert_called_once()
        args = self.fake_conn.mav.command_int_send.call_args[0]
        self.assertEqual(args[0], dm.target_system)
        self.assertEqual(args[2], MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
        self.assertEqual(args[3], MAV_CMD_DO_REPOSITION)
        self.assertEqual(args[6], -1)
        self.assertEqual(args[7], MAV_DO_REPOSITION_FLAGS_CHANGE_MODE)
        self.assertEqual(args[8], 80.0)  # param3 = radius
        self.assertEqual(args[9], 0)
        self.assertEqual(args[10], int(round(loc.lat * 1e7)))
        self.assertEqual(args[11], int(round(loc.lng * 1e7)))
        self.assertAlmostEqual(args[12], loc.alt)

    def test_goto_loiter_param_send_failure_does_not_abort_reposition(self):
        """A WP_LOITER_RAD send failure must not abort the loiter command -
        DO_REPOSITION param3 still carries the radius."""
        dm = self._new_dm(device='dev', target_system=8)
        with patch.object(
                dm._parts.parameters,
                'send_unverified',
                side_effect=RuntimeError("link down"),
        ) as send_param, patch.object(self.logger, 'warning') as fake_warn:
            dm.goto_loiter(Location(10.0, 20.0, 100.0, is_absolute=False), 80.0)
            send_param.assert_called_once_with("WP_LOITER_RAD", 80.0)
            fake_warn.assert_called_once()
        self.fake_conn.mav.command_int_send.assert_called_once()
        args = self.fake_conn.mav.command_int_send.call_args[0]
        self.assertEqual(args[8], 80.0)  # radius still in param3

    def test_goto_loiter_rejects_invalid_radius(self):
        """Non-finite or non-positive radius commands neither the param nor the
        reposition."""
        dm = self._new_dm(device='dev', target_system=11)
        for bad in (0.0, -5.0, float('nan'), float('inf'), None, "not-a-number"):
            self.fake_conn.mav.command_int_send.reset_mock()
            with patch.object(
                    dm._parts.parameters, 'send_unverified',
            ) as send_param, patch.object(self.logger, 'warning') as fake_warn:
                dm.goto_loiter(Location(10.0, 20.0, 100.0, is_absolute=False), bad)
                send_param.assert_not_called()
                fake_warn.assert_called_once()
            self.fake_conn.mav.command_int_send.assert_not_called()

    def test_goto_loiter_converts_absolute_altitude(self):
        dm = self._new_dm(device='dev', target_system=9)
        home = self._make_vehicle_msg(
            dm,
            "HOME_POSITION",
            latitude=int(40.0 * 1e7),
            longitude=int(44.0 * 1e7),
            altitude=int(1200.0 * 1000),
        )
        dm.feed_message(home)

        with patch.object(dm._parts.parameters, 'send_unverified', return_value=True):
            dm.goto_loiter(Location(10.0, 20.0, 1300.0, is_absolute=True), 80.0)

        args = self.fake_conn.mav.command_int_send.call_args[0]
        self.assertAlmostEqual(args[12], 100.0)

    def test_send_param_set_unverified_sends_without_waiting(self):
        """The focused fire-and-forget writer invalidates stale cache state."""
        from pymavlink.dialects.v20.ardupilotmega import MAV_PARAM_TYPE_REAL32
        dm = self._new_dm(device='dev', target_system=12)
        dm.feed_message(self._make_vehicle_msg(
            dm,
            "PARAM_VALUE",
            param_id="WP_LOITER_RAD",
            param_value=999.0,
            param_type=MAV_PARAM_TYPE_REAL32,
        ))
        self.assertEqual(dm.get_parameter("WP_LOITER_RAD"), 999.0)
        result = dm._parts.parameters.send_unverified("wp_loiter_rad", 80.0)
        self.assertTrue(result)
        self.fake_conn.mav.param_set_send.assert_called_once()
        pargs = self.fake_conn.mav.param_set_send.call_args[0]
        self.assertEqual(pargs[0], dm.target_system)
        self.assertEqual(pargs[2], b"WP_LOITER_RAD")
        self.assertEqual(pargs[3], 80.0)
        self.assertEqual(pargs[4], MAV_PARAM_TYPE_REAL32)
        self.fake_conn.mav.param_request_read_send.reset_mock()
        self.assertIsNone(dm.get_parameter(
            "WP_LOITER_RAD", timeout=0.01, retries=1, quiet=True,
        ))
        self.fake_conn.mav.param_request_read_send.assert_called_once()

    def test_send_param_set_unverified_rejects_bad_value(self):
        """Non-numeric / non-finite values send nothing and return False."""
        dm = self._new_dm(device='dev', target_system=13)
        for bad in (float('nan'), float('inf'), None, "x"):
            self.fake_conn.mav.param_set_send.reset_mock()
            with patch.object(self.logger, 'warning') as fake_warn:
                result = dm._parts.parameters.send_unverified(
                    "WP_LOITER_RAD", bad,
                )
            self.assertFalse(result)
            self.fake_conn.mav.param_set_send.assert_not_called()
            fake_warn.assert_called_once()

    def test_send_param_set_unverified_skips_when_param_write_busy(self):
        """When a verified write holds the param-write lock, the unverified
        helper skips (returns False, sends nothing) rather than block the loop."""
        dm = self._new_dm(device='dev', target_system=14)
        # Plain Lock: a same-thread non-reentrant acquire models the contended
        # state the helper must not block on.
        writer = dm._parts.parameters._writer
        self.assertTrue(writer._write_lock.acquire(blocking=False))
        try:
            result = writer.send_unverified("WP_LOITER_RAD", 80.0)
        finally:
            writer._write_lock.release()
        self.assertFalse(result)
        self.fake_conn.mav.param_set_send.assert_not_called()

    def test_angle_params(self):
        dm = self._new_dm(device='dev', target_system=3)
        for name, value in (
            ('PTCH_LIM_MAX_DEG', 30.0), ('ROLL_LIMIT_DEG', 45.0),
        ):
            dm.feed_message(self._make_vehicle_msg(
                dm,
                "PARAM_VALUE",
                param_id=name,
                param_value=value,
                param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            ))
        self.assertEqual(dm.max_pitch, 30.0)
        self.assertEqual(dm.lim_roll, 45.0)

    def test_register_rc_channel_and_callback(self):
        dm = self._new_dm(device='dev', target_system=4)
        received = []

        def cb(val):
            received.append(val)

        dm.register_rc_channel(3, cb)
        msg = self._make_vehicle_msg(dm, "RC_CHANNELS", chan3_raw=1500)
        dm.feed_message(msg)
        self.assertEqual(received, [1500])

    def test_clear_and_close(self):
        dm = self._new_dm(device='dev', target_system=1)
        from pymavlink.mavwp import MAVWPLoader
        loader = MAVWPLoader(dm.target_system, 0)
        loader.add_latlonalt(32.0, 34.0, 100)
        dm.load_mission_items(loader)
        self.assertGreater(dm.mission_items_count, 0)
        dm.clear_mission()
        self.assertEqual(dm.mission_items_count, 0)
        self.fake_conn.close.reset_mock()
        dm.close()
        dm.close()
        self.fake_conn.close.assert_called_once()

    def test_wind_and_attitude_properties(self):
        self._cache("WIND", direction=90, speed=5.5, speed_z=-0.3)
        self._cache("ATTITUDE", pitch=0.1, roll=0.2, yaw=1.5)
        w = self.dm.wind
        a = self.dm.attitude
        self.assertEqual((w.direction, w.speed, w.speed_z), (90, 5.5, -0.3))
        self.assertAlmostEqual(a.pitch, math.degrees(0.1))
        self.assertAlmostEqual(a.roll, math.degrees(0.2))
        self.assertAlmostEqual(a.yaw, math.degrees(1.5))

    def test_heading_ground_and_air_speed(self):
        self._cache("VFR_HUD", heading=123, groundspeed=11.1, airspeed=12.2)
        self.assertEqual(self.dm.heading, 123)
        self.assertEqual(self.dm.ground_speed, 11.1)
        self.assertEqual(self.dm.air_speed, 12.2)

    def test_ground_speed_ned_does_not_invent_track_from_hud_heading(self):
        self._cache("VFR_HUD", heading=90, groundspeed=10, airspeed=12, climb=2)
        self.assertIsNone(self.dm.ground_speed_ned)
        self.assertEqual(self.dm.heading, 90)
        self.assertEqual(self.dm.ground_speed, 10)
        self.assertEqual(self.dm.climb_rate, 2)

    def test_ground_speed_ned_uses_measured_local_velocity_before_global(self):
        self._cache("VFR_HUD", heading=90, groundspeed=10, airspeed=12, climb=2)
        self._cache("GLOBAL_POSITION_INT", vx=700, vy=800, vz=900)
        self._cache("LOCAL_POSITION_NED", vx=1.25, vy=-2.5, vz=3.75)
        self.assertEqual(self.dm.ground_speed_ned.tolist(), [1.25, -2.5, 3.75])

    def test_ground_speed_ned_converts_measured_global_centimetres_per_second(self):
        self._cache("VFR_HUD", heading=90, groundspeed=10, airspeed=12, climb=2)
        self._cache("GLOBAL_POSITION_INT", vx=125, vy=-250, vz=375)
        self.assertEqual(self.dm.ground_speed_ned.tolist(), [1.25, -2.5, 3.75])

    def test_ground_speed_ned_is_none_without_velocity_messages(self):
        self.assertIsNone(self.dm.ground_speed_ned)

    def test_climb_rate(self):
        self._cache("VFR_HUD", heading=0, groundspeed=0, airspeed=0, climb=3.5)
        self.assertEqual(self.dm.climb_rate, 3.5)

    def test_climb_rate_none_without_hud(self):
        self.assertIsNone(self.dm.climb_rate)

    def test_throttle_pct(self):
        self._cache("VFR_HUD", heading=0, groundspeed=0, airspeed=0, throttle=72)
        self.assertEqual(self.dm.throttle_pct, 72)

    def test_throttle_pct_none_without_hud(self):
        self.assertIsNone(self.dm.throttle_pct)

    def test_home_and_location(self):
        # home
        self._cache("HOME_POSITION", latitude=404000000, longitude=441000000, altitude=1500000)
        home = self.dm.home_location
        self.assertEqual((round(home.lat, 5), round(home.lng, 5), home.alt), (40.4, 44.1, 1500))

        # absolute location
        self._cache("GLOBAL_POSITION_INT", lat=404100000, lon=441100000, alt=250000, relative_alt=50 * 1e3)
        loc_abs = self.dm.location(is_relative=False)
        loc_rel = self.dm.location(is_relative=True)
        self.assertAlmostEqual(loc_abs.alt, 250.0)
        self.assertAlmostEqual(loc_rel.alt, 50.0)

    def test_terrain_height_at_requests_matching_report(self):
        lat = 40.320295
        lng = 44.455211
        lat_int = int(lat * 1e7)
        lng_int = int(lng * 1e7)

        def reply(*_args):
            self.dm.feed_message(self._make_vehicle_msg(
                self.dm,
                "TERRAIN_REPORT",
                pending=0,
                spacing=100,
                lat=lat_int,
                lon=lng_int,
                terrain_height=1305.8,
            ))

        self.fake_conn.mav.terrain_check_send.side_effect = reply
        height = self.dm.terrain_height_at(lat, lng)

        self.fake_conn.mav.terrain_check_send.assert_called_once_with(lat_int, lng_int)
        self.assertAlmostEqual(height, 1305.8)

    def test_terrain_height_at_accepts_available_report_with_pending_blocks(self):
        def reply(lat, lon):
            self.dm.feed_message(self._make_vehicle_msg(
                self.dm, "TERRAIN_REPORT", lat=lat, lon=lon,
                spacing=100, pending=168, terrain_height=1295.1,
            ))

        self.fake_conn.mav.terrain_check_send.side_effect = reply
        # A synchronous reply must resolve without entering a condition wait.
        with patch.object(self.dm._parts.position._messages._condition, "wait", side_effect=AssertionError("unnecessary wait")):
            self.assertEqual(self.dm.terrain_height_at(40.31, 44.46), 1295.1)

    def test_terrain_height_at_unavailable_report_resolves_without_waiting(self):
        for pending in (0, 168):
            with self.subTest(pending=pending):
                def reply(lat, lon):
                    self.dm.feed_message(self._make_vehicle_msg(
                        self.dm, "TERRAIN_REPORT", lat=lat, lon=lon,
                        spacing=0, pending=pending, terrain_height=0.0,
                    ))

                self.fake_conn.mav.terrain_check_send.side_effect = reply
                with patch.object(self.dm._parts.position._messages._condition, "wait", side_effect=AssertionError("unnecessary wait")):
                    self.assertIsNone(self.dm.terrain_height_at(40.31, 44.46))

    def test_terrain_height_at_rejects_stale_wrong_location_and_nonfinite_reports(self):
        lat, lon = 40.31, 44.46
        self.dm.feed_message(self._make_vehicle_msg(
            self.dm, "TERRAIN_REPORT", lat=int(lat * 1e7), lon=int(lon * 1e7),
            spacing=100, pending=0, terrain_height=123.0,
        ))
        self.assertIsNone(self.dm.terrain_height_at(lat, lon, timeout=0))
        for offset, height in ((100, 123.0), (0, float("nan")), (0, float("inf"))):
            with self.subTest(offset=offset, height=height):
                def reply(lat_int, lon_int):
                    self.dm.feed_message(self._make_vehicle_msg(
                        self.dm, "TERRAIN_REPORT", lat=lat_int + offset, lon=lon_int,
                        spacing=100, pending=0, terrain_height=height,
                    ))

                self.fake_conn.mav.terrain_check_send.side_effect = reply
                self.assertIsNone(self.dm.terrain_height_at(lat, lon, timeout=0))

    def test_terrain_height_at_rejects_other_vehicle_report(self):
        def reply(lat, lon):
            message = self._make_vehicle_msg(
                self.dm, "TERRAIN_REPORT", lat=lat, lon=lon,
                spacing=100, pending=0, terrain_height=123.0,
            )
            message.get_srcSystem.return_value = self.dm.target_system + 1
            self.dm._parts.position._messages.publish(
                "TERRAIN_REPORT", message, receipt_time_s=time.monotonic(),
            )

        self.fake_conn.mav.terrain_check_send.side_effect = reply
        self.assertIsNone(self.dm.terrain_height_at(40.31, 44.46, timeout=0))

    def test_terrain_height_at_preserves_no_reply_deadline(self):
        messages = self.dm._parts.position._messages
        with patch("navpy.modules.vehicle.position_service.time") as clock:
            clock.monotonic.return_value = 100.0
            with patch.object(messages, "wait_after", return_value=None) as wait:
                self.assertIsNone(self.dm.terrain_height_at(40.31, 44.46))
                self.assertEqual(wait.call_args.kwargs["deadline"], 101.0)
                self.assertIsNone(self.dm.terrain_height_at(40.31, 44.46, timeout=0.2))
                self.assertEqual(wait.call_args.kwargs["deadline"], 100.2)

    def test_get_parameter_cached_and_uncached(self):
        # ── 1. cached fast-path ───────────────────────────────────────────
        self.dm.feed_message(self._make_vehicle_msg(
            self.dm,
            "PARAM_VALUE",
            param_id="FOO",
            param_value=7.0,
            param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        ))
        self.assertEqual(self.dm.get_parameter("FOO"), 7.0)

        # ── 2. uncached: simulate PARAM_VALUE reply ───────────────────────
        def fake_param_request_read_send(ts, comp, name_bytes, idx):
            pname = name_bytes.decode().rstrip("\x00")
            self.dm.feed_message(self._make_vehicle_msg(
                self.dm,
                "PARAM_VALUE",
                param_id=pname,
                param_value=3.14,
                param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                seq=1,
            ))

        with patch.object(
                self.fake_conn.mav,
                "param_request_read_send",
                side_effect=fake_param_request_read_send,
        ):
            val = self.dm.get_parameter("BAR", timeout=0.1, retries=1)

        self.assertEqual(val, 3.14)
        self.assertEqual(self.dm.get_parameter("BAR"), 3.14)

    def test_get_parameter_caches_missing_parameter(self):
        with patch.object(self.fake_conn.mav, "param_request_read_send") as request:
            self.assertIsNone(self.dm.get_parameter("MISSING", timeout=0.01, retries=1))
            self.assertIsNone(self.dm.get_parameter("MISSING", timeout=0.01, retries=1))

        request.assert_called_once()

    def test_is_simulated_autopilot_true_when_sim_speedup_present(self):
        # SITL exposes SIM_SPEEDUP; a positive read means simulated autopilot.
        self.dm.feed_message(self._make_vehicle_msg(
            self.dm,
            "PARAM_VALUE",
            param_id="SIM_SPEEDUP",
            param_value=3.0,
            param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        ))
        self.assertTrue(self.dm.is_simulated_autopilot())
        self.assertEqual(self.dm.sim_speedup(), 3.0)

    def test_is_simulated_autopilot_false_on_real_hardware(self):
        # Real ArduPilot has no SIM_SPEEDUP -> read returns None -> assume real.
        with patch.object(
                self.dm._parts.simulation._reader, "get", return_value=None,
        ):
            self.assertFalse(self.dm.is_simulated_autopilot())

    def test_is_simulated_autopilot_honors_runtime_parameter_override(self):
        with patch.object(
                self.dm._parts.simulation._reader, "get", return_value=3.0,
        ) as get_parameter:
            self.assertTrue(self.dm.is_simulated_autopilot())
        get_parameter.assert_called_once_with(
            "SIM_SPEEDUP", timeout=1.0, quiet=True,
        )

    def test_is_simulated_autopilot_cached_without_reread(self):
        # Once positively seen, no further param read is issued.
        self.dm.feed_message(self._make_vehicle_msg(
            self.dm,
            "PARAM_VALUE",
            param_id="SIM_SPEEDUP",
            param_value=3.0,
            param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        ))
        self.assertTrue(self.dm.is_simulated_autopilot())
        with patch.object(self.dm._parts.simulation._reader, "get") as get_param:
            self.assertTrue(self.dm.is_simulated_autopilot())
        get_param.assert_not_called()

    def test_sim_speedup_reads_new_cache_value_before_probe_deadline(self):
        self.dm.feed_message(self._make_vehicle_msg(
            self.dm,
            "PARAM_VALUE",
            param_id="SIM_SPEEDUP",
            param_value=10.0,
            param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        ))

        self.assertEqual(self.dm.sim_speedup(), 10.0)
        self.fake_conn.mav.param_request_read_send.assert_not_called()

    def test_angle_param_helpers(self):
        for name, value in (
            ("PTCH_LIM_MIN_DEG", -5.0), ("ROLL_LIMIT_DEG", 35.0),
        ):
            self.dm.feed_message(self._make_vehicle_msg(
                self.dm,
                "PARAM_VALUE",
                param_id=name,
                param_value=value,
                param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            ))
        self.assertEqual(self.dm.min_pitch, -5.0)
        self.assertEqual(self.dm.lim_roll, 35.0)

    def test_euler_to_quat_math(self):
        w, x, y, z = euler_to_quaternion(math.pi, 0, 0)
        # 180° roll should flip x & w
        self.assertAlmostEqual(w, 0.0, places=5)
        self.assertAlmostEqual(abs(x), 1.0, places=5)
        self.assertAlmostEqual(y, 0.0, places=5)
        self.assertAlmostEqual(z, 0.0, places=5)

    def test_set_attitude_invokes_mavlink(self):
        self.dm.set_attitude(0.1, 0.2, yaw=0.3, thr=0.55)
        self.fake_conn.mav.set_attitude_target_send.assert_called_once()
        # throttle passed through
        args = self.fake_conn.mav.set_attitude_target_send.call_args[0]
        self.assertAlmostEqual(args[-1], 0.55, places=2)
        self.assertEqual(
            args[3],
            ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
            | ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
            | ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE,
        )

    def test_set_attitude_exposes_outbound_debug_poi(self):
        self.dm.set_attitude(0.1, 0.2, yaw=None, thr=0.55)

        target = self.dm.attitude_target_debug

        self.assertIsNotNone(target)
        self.assertAlmostEqual(target.roll, math.degrees(0.1))
        self.assertAlmostEqual(target.pitch, math.degrees(0.2))
        self.assertIsNone(target.yaw)
        self.assertIsNone(target.requested_yaw)
        self.assertAlmostEqual(target.thrust, 0.55)
        self.assertEqual(
            target.type_mask,
            ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
            | ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE,
        )
        self.assertGreaterEqual(target.age_ms, 0.0)

    def test_set_attitude_yaw_none_uses_arduplane_partial_roll_pitch_mask(self):
        self.dm.set_attitude(0.1, 0.2, yaw=None, thr=0.55)

        args = self.fake_conn.mav.set_attitude_target_send.call_args[0]

        self.assertEqual(
            args[3],
            ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
            | ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE,
        )

    def test_nav_controller_output_debug_exposes_latest_cache(self):
        self._cache(
            "NAV_CONTROLLER_OUTPUT",
            nav_roll=12.5,
            nav_pitch=-23.5,
            nav_bearing=90,
            target_bearing=91,
            wp_dist=123,
        )

        nav = self.dm.nav_controller_output_debug

        self.assertIsNotNone(nav)
        self.assertEqual(nav.nav_roll, 12.5)
        self.assertEqual(nav.nav_pitch, -23.5)
        self.assertEqual(nav.nav_bearing, 90)
        self.assertEqual(nav.target_bearing, 91)
        self.assertEqual(nav.wp_dist, 123)
        self.assertGreaterEqual(nav.age_ms, 0.0)

    def test_send_helpers(self):
        dummy_msg = MagicMock()

        # send_mavlink_message => uses conn.mav.send(...)
        self.dm.send_mavlink_message(dummy_msg)
        self.fake_conn.mav.send.assert_called_once_with(dummy_msg)

        # send_status_text => uses _conn.mav.statustext_send(...)
        self.dm.send_status_text("hello")
        self.fake_conn.mav.statustext_send.assert_called_once()

        sev, payload = self.fake_conn.mav.statustext_send.call_args[0]
        self.assertEqual(sev, mavutil.mavlink.MAV_SEVERITY_INFO)
        self.assertEqual(payload, b"hello")

    def test_send_mavlink_message_can_override_source_component(self):
        dummy_msg = MagicMock()
        self.fake_conn.mav.srcComponent = 191

        self.dm.send_mavlink_message(dummy_msg, source_component=100)

        self.fake_conn.mav.send.assert_called_with(dummy_msg)
        self.assertEqual(self.fake_conn.mav.srcComponent, 191)


    def test_heartbeat_ignores_companion_computer(self):
        """Companion heartbeats must not replace autopilot armed state."""
        dm = self._new_dm(device='dev', target_system=1)
        autopilot_hb = self._make_vehicle_msg(
            dm,
            "HEARTBEAT",
            type=MAV_TYPE_FIXED_WING,
            base_mode=MAV_MODE_FLAG_SAFETY_ARMED,
            custom_mode=10,
        )
        dm.feed_message(autopilot_hb)
        cc_hb = self._make_vehicle_msg(
            dm,
            "HEARTBEAT",
            component_id=191,
            seq=1,
            type=MAV_TYPE_ONBOARD_CONTROLLER,
            base_mode=0,
            custom_mode=0,
        )
        dm.feed_message(cc_hb)
        self.assertTrue(dm.is_armed)

    def test_heartbeat_ignores_gcs(self):
        """GCS heartbeats must not replace autopilot armed state."""
        dm = self._new_dm(device='dev', target_system=1)
        autopilot_hb = self._make_vehicle_msg(
            dm,
            "HEARTBEAT",
            type=MAV_TYPE_FIXED_WING,
            base_mode=MAV_MODE_FLAG_SAFETY_ARMED,
        )
        dm.feed_message(autopilot_hb)
        gcs_hb = self._make_vehicle_msg(
            dm,
            "HEARTBEAT",
            component_id=190,
            seq=1,
            type=MAV_TYPE_GCS,
            base_mode=0,
        )
        dm.feed_message(gcs_hb)
        self.assertTrue(dm.is_armed)

    def test_heartbeat_ignores_same_system_camera_component(self):
        dm = self._new_dm(device='dev', target_system=1)
        autopilot_hb = self._make_vehicle_msg(
            dm,
            "HEARTBEAT",
            type=MAV_TYPE_FIXED_WING,
            base_mode=MAV_MODE_FLAG_SAFETY_ARMED,
        )
        dm.feed_message(autopilot_hb)
        camera_hb = self._make_vehicle_msg(
            dm,
            "HEARTBEAT",
            component_id=100,
            seq=1,
            type=MAV_TYPE_CAMERA,
            base_mode=0,
        )
        dm.feed_message(camera_hb)
        self.assertTrue(dm.is_armed)

    def test_is_armed_true(self):
        """is_armed returns True when heartbeat has SAFETY_ARMED flag."""
        hb = self._make_vehicle_msg(
            self.dm,
            "HEARTBEAT",
            type=MAV_TYPE_FIXED_WING,
            base_mode=MAV_MODE_FLAG_SAFETY_ARMED,
        )
        self.dm.feed_message(hb)
        self.assertTrue(self.dm.is_armed)

    def test_is_armed_false_no_heartbeat(self):
        """is_armed returns False when no heartbeat received yet."""
        self.assertFalse(self.dm.is_armed)

    def test_is_armed_false_disarmed(self):
        """is_armed returns False when heartbeat has no SAFETY_ARMED flag."""
        hb = self._make_vehicle_msg(
            self.dm,
            "HEARTBEAT",
            type=MAV_TYPE_FIXED_WING,
            base_mode=0,
        )
        self.dm.feed_message(hb)
        self.assertFalse(self.dm.is_armed)

    def test_send_command_long(self):
        """send_command_long sends COMMAND_LONG through the connection."""
        self.dm.send_command_long(400, p1=1, p2=21196)
        self.fake_conn.mav.command_long_send.assert_called_once_with(
            self.dm.target_system, 0,
            400, 0,
            1, 21196, 0, 0, 0, 0, 0,
        )

    def test_load_mission_items(self):
        """load_mission_items replaces internal mission from a wp_loader."""
        from pymavlink.mavwp import MAVWPLoader
        loader = MAVWPLoader(self.dm.target_system, 0)
        loader.add_latlonalt(32.0, 34.0, 100)
        loader.add_latlonalt(32.1, 34.1, 100)

        self.dm.load_mission_items(loader)
        self.assertEqual(self.dm.mission_items_count, 2)

    def test_close_stops_heartbeat(self):
        self.fake_conn.close.reset_mock()
        self.dm.close()
        self.dm.close()
        self.fake_conn.close.assert_called_once()

    # --------------- helpers ----------------
    def _cache(self, mtype, **fields):
        msg = self._make_vehicle_msg(self.dm, mtype, **fields)
        self.dm.feed_message(msg)
        return msg

    # --------------- link quality tests ----------------
    def _make_pkt(self, sys_id, comp_id, seq, mtype="DATA"):
        """Create a mock MAVLink message with the given system/component/seq."""
        m = MagicMock()
        m.get_srcSystem.return_value = sys_id
        m.get_srcComponent.return_value = comp_id
        m.get_seq.return_value = seq
        m.get_type.return_value = mtype
        m.get_msgId.return_value = 33  # GLOBAL_POSITION_INT (any valid id)
        return m

    def test_link_quality_no_loss(self):
        """100% quality when all packets arrive in order from one component."""
        dm = self._new_dm(device='dev', target_system=1)
        for seq in range(20):
            dm.feed_message(self._make_pkt(1, 1, seq))
        self.assertEqual(dm.link_quality, 100)

    def test_link_quality_with_loss(self):
        """Quality drops when packets are lost (sequence gaps)."""
        dm = self._new_dm(device='dev', target_system=1)
        # Send 10 consecutive, then skip 5
        for seq in range(10):
            dm.feed_message(self._make_pkt(1, 1, seq))
        dm.feed_message(self._make_pkt(1, 1, 15))  # skipped 10-14
        # pkts_ok = 1 (init) + 11 = 12, pkts_lost = 5
        quality = dm.link_quality
        self.assertGreater(quality, 50)
        self.assertLess(quality, 100)

    def test_link_quality_per_component(self):
        """Interleaved components must NOT cause phantom packet loss."""
        dm = self._new_dm(device='dev', target_system=1)
        # Component 1: seq 0..9, Component 191: seq 0..9
        for i in range(10):
            dm.feed_message(self._make_pkt(1, 1, i))
            dm.feed_message(self._make_pkt(1, 191, i))
        # No packets were actually lost
        self.assertEqual(dm.link_quality, 100)

    def test_link_quality_tracks_only_the_autopilot_component(self):
        """Non-autopilot components share the aircraft's system id (companion
        computer + its camera ids) and one encoder-wide sequence counter, so
        their per-component streams have holes by construction. Their gaps
        must not touch link quality; only component 1 is tracked."""
        dm = self._new_dm(device='dev', target_system=1)
        # Component 1: clean sequence 0..9
        for seq in range(10):
            dm.feed_message(self._make_pkt(1, 1, seq))
        # Component 2: seq 0, then skip to 10 -- untracked, no phantom loss
        dm.feed_message(self._make_pkt(1, 2, 0))
        dm.feed_message(self._make_pkt(1, 2, 10))
        self.assertEqual(dm.link_quality, 100)

    def test_link_quality_duplicate_seq_ignored(self):
        """Duplicate sequence numbers (3DR radio bug) must not count as loss."""
        dm = self._new_dm(device='dev', target_system=1)
        for seq in range(5):
            dm.feed_message(self._make_pkt(1, 1, seq))
        # Duplicate: send seq=4 again
        dm.feed_message(self._make_pkt(1, 1, 4))
        # Then continue normally
        dm.feed_message(self._make_pkt(1, 1, 5))
        self.assertEqual(dm.link_quality, 100)

    def test_link_quality_seq_wraparound(self):
        """Sequence wraparound at 256 must not count as loss."""
        dm = self._new_dm(device='dev', target_system=1)
        dm.feed_message(self._make_pkt(1, 1, 254))
        dm.feed_message(self._make_pkt(1, 1, 255))
        dm.feed_message(self._make_pkt(1, 1, 0))
        dm.feed_message(self._make_pkt(1, 1, 1))
        self.assertEqual(dm.link_quality, 100)

    def test_gps_hacc_from_h_acc(self):
        """gps_hacc prefers h_acc (mm) over eph."""
        self._cache("GPS_RAW_INT", h_acc=14, eph=120,
                     satellites_visible=10, fix_type=6)
        self.assertAlmostEqual(self.dm.gps_hacc, 0.014)

    def test_gps_hacc_fallback_eph(self):
        """gps_hacc falls back to eph (cm) when h_acc is 0 or absent."""
        self._cache("GPS_RAW_INT", h_acc=0, eph=150,
                     satellites_visible=10, fix_type=3)
        self.assertAlmostEqual(self.dm.gps_hacc, 1.5)

    def test_gps_hacc_none_when_no_msg(self):
        """gps_hacc returns None when no GPS_RAW_INT cached."""
        self.assertIsNone(self.dm.gps_hacc)

    def test_gps_hacc_none_when_unknown(self):
        """gps_hacc returns None when eph is 65535 (unknown) and no h_acc."""
        self._cache("GPS_RAW_INT", h_acc=0, eph=65535,
                     satellites_visible=10, fix_type=3)
        self.assertIsNone(self.dm.gps_hacc)

    def test_disarm_sends_command_long(self):
        dm = self._new_dm(device='dev', target_system=7)
        dm.disarm()

        from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_COMPONENT_ARM_DISARM
        self.fake_conn.mav.command_long_send.assert_called_once()
        args = self.fake_conn.mav.command_long_send.call_args[0]
        # args: target_system, target_component, command, confirmation, p1..p7
        self.assertEqual(args[0], 7)           # target system
        self.assertEqual(args[2], MAV_CMD_COMPONENT_ARM_DISARM)  # command
        self.assertEqual(args[4], 0)           # p1=0 means disarm
        self.assertEqual(args[5], 21196)       # p2=21196 force in-flight disarm

    def test_disarm_honors_runtime_command_override(self):
        dm = self._new_dm(device='dev', target_system=7)
        self.fake_conn.mav.command_long_send.reset_mock()

        with patch.object(dm._parts.runtime.commands, "send_command_long") as send_command:
            dm.disarm()

        from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_COMPONENT_ARM_DISARM
        send_command.assert_called_once_with(
            MAV_CMD_COMPONENT_ARM_DISARM, p1=0, p2=21196,
        )
        self.fake_conn.mav.command_long_send.assert_not_called()

    def test_facade_keeps_real_interface_signatures_and_properties(self):
        self.assertEqual(
            list(inspect.signature(VehicleMav.send_command_long).parameters),
            ["self", "command", "p1", "p2", "p3", "p4", "p5", "p6", "p7"],
        )
        self.assertIsInstance(VehicleMav.link_ok, property)

    def test_attitude_cache_rejects_non_autopilot_component(self):
        dm = self._new_dm(device='dev', target_system=1)
        msg = self._make_vehicle_msg(
            dm,
            "ATTITUDE",
            component_id=191,
            time_boot_ms=100,
            pitch=math.radians(12.0),
            yaw=0.0,
            roll=0.0,
        )

        dm.feed_message(msg)

        self.assertIsNone(dm.attitude)

    def test_attitude_cache_rejects_older_boot_time(self):
        dm = self._new_dm(device='dev', target_system=1)
        first = self._make_vehicle_msg(
            dm,
            "ATTITUDE",
            time_boot_ms=200,
            pitch=math.radians(-20.0),
            yaw=0.0,
            roll=0.0,
        )
        older = self._make_vehicle_msg(
            dm,
            "ATTITUDE",
            time_boot_ms=150,
            pitch=math.radians(-1.0),
            yaw=0.0,
            roll=0.0,
            seq=1,
        )

        dm.feed_message(first)
        dm.feed_message(older)

        self.assertAlmostEqual(dm.attitude.pitch, -20.0)

    def test_attitude_cache_accepts_newer_boot_time(self):
        dm = self._new_dm(device='dev', target_system=1)
        first = self._make_vehicle_msg(
            dm,
            "ATTITUDE",
            time_boot_ms=200,
            pitch=math.radians(-20.0),
            yaw=0.0,
            roll=0.0,
        )
        newer = self._make_vehicle_msg(
            dm,
            "ATTITUDE",
            time_boot_ms=250,
            pitch=math.radians(-5.0),
            yaw=0.0,
            roll=0.0,
            seq=1,
        )

        dm.feed_message(first)
        dm.feed_message(newer)

        self.assertAlmostEqual(dm.attitude.pitch, -5.0)

    def test_attitude_cache_accepts_reboot_after_short_uptime(self):
        """A reboot after <60 s uptime restarts the boot clock near zero. The
        first post-reboot frame must be accepted, not frozen out for the whole
        pre-reboot uptime (finding: <60 s-uptime boot-freeze)."""
        dm = self._new_dm(device='dev', target_system=1)
        pre_reboot = self._make_vehicle_msg(
            dm, "ATTITUDE", time_boot_ms=50_000,
            pitch=math.radians(-20.0), yaw=0.0, roll=0.0,
        )
        post_reboot = self._make_vehicle_msg(
            dm, "ATTITUDE", time_boot_ms=80,
            pitch=math.radians(-5.0), yaw=0.0, roll=0.0, seq=1,
        )

        dm.feed_message(pre_reboot)
        dm.feed_message(post_reboot)

        self.assertAlmostEqual(dm.attitude.pitch, -5.0)

    def test_attitude_cache_still_rejects_late_packet_at_high_uptime(self):
        """A link-reordered packet at high uptime carries a stamp near the
        current (large) uptime and must stay rejected — the fresh-boot accept
        must not weaken out-of-order rejection."""
        dm = self._new_dm(device='dev', target_system=1)
        current = self._make_vehicle_msg(
            dm, "ATTITUDE", time_boot_ms=500_000,
            pitch=math.radians(-20.0), yaw=0.0, roll=0.0,
        )
        late = self._make_vehicle_msg(
            dm, "ATTITUDE", time_boot_ms=499_500,
            pitch=math.radians(-5.0), yaw=0.0, roll=0.0, seq=1,
        )

        dm.feed_message(current)
        dm.feed_message(late)

        self.assertAlmostEqual(dm.attitude.pitch, -20.0)

    def test_attitude_sample_exposes_atomic_receipt_time(self):
        dm = self._new_dm(device='dev', target_system=1)
        attitude = self._make_vehicle_msg(
            dm,
            "ATTITUDE",
            time_boot_ms=250,
            pitch=math.radians(-5.0),
            yaw=0.0,
            roll=0.0,
            rollspeed=0.01,
            pitchspeed=-0.02,
            yawspeed=0.03,
        )

        with patch("navpy.modules.vehicle.inbound_router.time.time", return_value=456.25):
            dm.feed_message(attitude)
        sample = dm.attitude_sample

        self.assertEqual(sample.receipt_time_s, 456.25)
        self.assertEqual(sample.time_boot_s, 0.25)
        self.assertEqual(sample.body_rates_rad_s, (0.01, -0.02, 0.03))
        sample.attitude.pitch = 99.0
        self.assertAlmostEqual(dm.attitude_sample.attitude.pitch, -5.0)

    def test_simulator_truth_pose_prefers_integer_coordinates_and_copies_pose(self):
        dm = self._new_dm(device="dev", target_system=1)
        sim_state = self._make_vehicle_msg(
            dm,
            "SIM_STATE",
            lat=401234567.0,
            lon=441234567.0,
            lat_int=401234567,
            lon_int=441234567,
            alt=1234.5,
            roll=math.radians(3.0),
            pitch=math.radians(-4.0),
            yaw=math.radians(25.0),
            vn=99.0,
            ve=98.0,
            vd=97.0,
        )

        with patch("navpy.modules.vehicle.inbound_router.time.time", return_value=456.25):
            dm.feed_message(sim_state)
        pose = dm.simulator_truth_pose

        self.assertAlmostEqual(pose.location.lat, 40.1234567)
        self.assertAlmostEqual(pose.location.lng, 44.1234567)
        self.assertAlmostEqual(pose.location.alt, 1234.5)
        self.assertAlmostEqual(pose.attitude.roll, 3.0)
        self.assertAlmostEqual(pose.attitude.pitch, -4.0)
        self.assertAlmostEqual(pose.attitude.yaw, 25.0)
        self.assertEqual(pose.receipt_time_s, 456.25)
        self.assertFalse(hasattr(pose, "velocity"))
        self.assertFalse(hasattr(pose, "body_rates_rad_s"))
        pose.attitude.yaw = 99.0
        self.assertAlmostEqual(dm.simulator_truth_pose.attitude.yaw, 25.0)

    def test_simulator_truth_pose_falls_back_when_int_extensions_default_zero(self):
        dm = self._new_dm(device="dev", target_system=1)
        sim_state = self._make_vehicle_msg(
            dm,
            "SIM_STATE",
            lat=401234567.0,
            lon=441234567.0,
            lat_int=0,
            lon_int=0,
            alt=1234.5,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
        )

        dm.feed_message(sim_state)
        pose = dm.simulator_truth_pose

        self.assertAlmostEqual(pose.location.lat, 40.1234567)
        self.assertAlmostEqual(pose.location.lng, 44.1234567)

    def test_simulator_truth_pose_accepts_legacy_coordinates_in_degrees(self):
        dm = self._new_dm(device="dev", target_system=1)
        sim_state = self._make_vehicle_msg(
            dm,
            "SIM_STATE",
            lat=40.1234567,
            lon=44.1234567,
            lat_int=0,
            lon_int=0,
            alt=1234.5,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
        )

        dm.feed_message(sim_state)
        pose = dm.simulator_truth_pose

        self.assertAlmostEqual(pose.location.lat, 40.1234567)
        self.assertAlmostEqual(pose.location.lng, 44.1234567)

    def test_simulator_truth_pose_rejects_nonfinite_or_out_of_range_coordinates(self):
        dm = self._new_dm(device="dev", target_system=1)
        nonfinite = self._make_vehicle_msg(
            dm,
            "SIM_STATE",
            lat=math.nan,
            lon=441234567.0,
            lat_int=0,
            lon_int=0,
            alt=1234.5,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
        )
        out_of_range = self._make_vehicle_msg(
            dm,
            "SIM_STATE",
            lat=0.0,
            lon=0.0,
            lat_int=1_000_000_000,
            lon_int=441234567,
            alt=1234.5,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            seq=1,
        )

        dm.feed_message(nonfinite)
        self.assertIsNone(dm.simulator_truth_pose)
        dm.feed_message(out_of_range)
        self.assertIsNone(dm.simulator_truth_pose)
    def test_simulator_truth_pose_rejects_non_autopilot_component(self):
        dm = self._new_dm(device="dev", target_system=1)
        sim_state = self._make_vehicle_msg(
            dm,
            "SIM_STATE",
            component_id=191,
            lat_int=401234567,
            lon_int=441234567,
            alt=1234.5,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
        )

        dm.feed_message(sim_state)

        self.assertIsNone(dm.simulator_truth_pose)

    def _new_dm(self, *, device, target_system):
        VehicleMavTest._device_counter += 1
        unique_device = f"{device}:{VehicleMavTest._device_counter}"
        dm = VehicleMav(device=unique_device,
                        target_system=target_system,
                        logger=self.logger,
                        skip_mission_download=True,
                        wait_heartbeat=False,
                        heartbeat_timeout=999.0)
        # ensure reader thread is stopped when the test ends
        self.addCleanup(dm.close)
        return dm

    def _make_vehicle_msg(
            self,
            dm,
            mtype,
            *,
            component_id=MAV_COMP_ID_AUTOPILOT1,
            seq=0,
            **attrs,
    ):
        msg = MagicMock()
        msg.get_type.return_value = mtype
        msg.get_srcSystem.return_value = dm.target_system
        msg.get_srcComponent.return_value = component_id
        msg.get_seq.return_value = seq
        msg.get_msgId.return_value = 30
        for key, value in attrs.items():
            setattr(msg, key, value)
        return msg


class TestUploadMission(unittest.TestCase):
    """Tests for the generation-based mission upload protocol."""

    _dev_counter = 0

    def setUp(self):
        TestUploadMission._dev_counter += 1
        self._device = f"dev_upload_{TestUploadMission._dev_counter}"

        self.fake_conn = MagicMock()
        self.fake_conn.mav = MagicMock()
        self.fake_conn.recv_match = MagicMock(return_value=None)
        self.fake_conn.close = MagicMock()

        patcher = patch(
            "navpy.modules.vehicle.mav_bus.mavutil.mavlink_connection",
            return_value=self.fake_conn,
        )
        self.addCleanup(patcher.stop)
        patcher.start()

        self.mode_patch = patch(
            "navpy.modules.vehicle.vehicle_mav.mavutil.mode_string_v10",
            return_value="MANUAL",
        )
        self.mode_patch.start()
        self.addCleanup(self.mode_patch.stop)

        mav_of_poi_patch = patch(
            "navpy.modules.vehicle.vehicle_mav.mavutil.mavlink.MAVLink",
            return_value=MagicMock(),
        )
        self.addCleanup(mav_of_poi_patch.stop)
        mav_of_poi_patch.start()

        self.dm = VehicleMav(
            device=self._device, target_system=1,
            logger=ConsoleLogger(),
            skip_mission_download=True,
            wait_heartbeat=False,
        )
        self.addCleanup(self.dm.close)

    def _make_mission_msg(self, mtype, **kwargs):
        """Create a mock mission-protocol message."""
        m = MagicMock()
        m.get_type.return_value = mtype
        m.get_srcSystem.return_value = self.dm.target_system
        m.get_srcComponent.return_value = 1
        m.get_seq.return_value = 0
        m.get_msgId.return_value = 44
        m.mission_type = kwargs.pop("mission_type", 0)
        for k, v in kwargs.items():
            setattr(m, k, v)
        return m

    def _load_wp(self, count):
        """Load *count* dummy waypoints into the mission loader."""
        from pymavlink.mavwp import MAVWPLoader
        loader = MAVWPLoader(self.dm.target_system, 0)
        for i in range(count):
            loader.add_latlonalt(32.0 + i * 0.001, 34.0, 100)
        self.dm.load_mission_items(loader)

    def test_empty_mission_returns_true(self):
        self.assertTrue(self.dm.upload_mission(timeout=1.0))

    def test_mission_inbox_publishes_request_after_cursor(self):
        inbox = MissionInbox()
        msg = self._make_mission_msg("MISSION_REQUEST_INT", seq=0)
        cursor = inbox.cursor()
        inbox.publish(msg, receipt_time_s=1.0)
        envelope = inbox.wait_after(
            cursor, lambda candidate: candidate is msg,
            deadline=time.monotonic() + 0.1,
        )
        self.assertIs(envelope.message, msg)

    def test_mission_inbox_publishes_ack_after_cursor(self):
        inbox = MissionInbox()
        msg = self._make_mission_msg("MISSION_ACK", type=MAV_MISSION_ACCEPTED)
        cursor = inbox.cursor()
        inbox.publish(msg, receipt_time_s=1.0)
        envelope = inbox.wait_after(
            cursor, lambda candidate: candidate is msg,
            deadline=time.monotonic() + 0.1,
        )
        self.assertIs(envelope.message, msg)

    def test_mission_inbox_ignores_non_protocol_message(self):
        inbox = MissionInbox()
        msg = self._make_mission_msg("GLOBAL_POSITION_INT")
        cursor = inbox.cursor()
        inbox.publish(msg, receipt_time_s=1.0)
        self.assertEqual(inbox.cursor(), cursor)

    def test_mission_inbox_cursor_excludes_stale_message(self):
        inbox = MissionInbox()
        stale = self._make_mission_msg("MISSION_REQUEST_INT", seq=0)
        inbox.publish(stale, receipt_time_s=1.0)
        cursor = inbox.cursor()
        self.assertIsNone(inbox.wait_after(
            cursor, lambda _message: True,
            deadline=time.monotonic() + 0.01,
        ))

    def _simulate_autopilot(self, total):
        """Request each next item immediately after receiving its predecessor."""
        def on_count_send(*args):
            if total:
                self.dm.feed_message(self._make_mission_msg(
                    "MISSION_REQUEST_INT", seq=0,
                ))

        def on_item_send(*args):
            sequence = args[2]
            if sequence + 1 < total:
                self.dm.feed_message(self._make_mission_msg(
                    "MISSION_REQUEST_INT", seq=sequence + 1,
                ))
            else:
                self.dm.feed_message(self._make_mission_msg(
                    "MISSION_ACK", type=MAV_MISSION_ACCEPTED,
                ))

        self.fake_conn.mav.mission_count_send.side_effect = on_count_send
        self.fake_conn.mav.mission_item_int_send.side_effect = on_item_send

    def test_upload_single_wp(self):
        """Full upload of a single waypoint via simulated autopilot responses."""
        self._load_wp(1)
        self._simulate_autopilot(1)
        result = self.dm.upload_mission(timeout=5.0)
        self.assertTrue(result)
        self.fake_conn.mav.mission_count_send.assert_called()
        self.fake_conn.mav.mission_item_int_send.assert_called_once()

    def test_upload_multiple_wps(self):
        """Upload 3 waypoints — autopilot requests each sequentially."""
        self._load_wp(3)
        self._simulate_autopilot(3)
        result = self.dm.upload_mission(timeout=5.0)
        self.assertTrue(result)
        self.assertEqual(self.fake_conn.mav.mission_item_int_send.call_count, 3)

    def test_upload_fails_no_request(self):
        """Upload fails when autopilot never sends MISSION_REQUEST."""
        self._load_wp(1)
        result = self.dm.upload_mission(timeout=0.5, retries=1)
        self.assertFalse(result)

    def test_upload_fails_no_ack(self):
        """Upload fails when autopilot requests items but never ACKs."""
        self._load_wp(1)

        def on_count_send(*args):
            import threading
            def respond():
                import time
                time.sleep(0.02)
                req = self._make_mission_msg("MISSION_REQUEST_INT", seq=0)
                self.dm.feed_message(req)
            threading.Thread(target=respond, daemon=True).start()
        self.fake_conn.mav.mission_count_send.side_effect = on_count_send

        result = self.dm.upload_mission(timeout=1.0)
        self.assertFalse(result)

    def test_stale_messages_drained_before_upload(self):
        """A pre-transaction request cannot satisfy a new upload."""
        stale = self._make_mission_msg("MISSION_REQUEST_INT", seq=99)
        self.dm.feed_message(stale)

        self._load_wp(1)
        self._simulate_autopilot(1)
        result = self.dm.upload_mission(timeout=5.0)
        self.assertTrue(result)


if __name__ == '__main__':
    unittest.main()
