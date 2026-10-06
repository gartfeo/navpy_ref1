"""Tests for typed parameter writes through the explicit VehicleMav adapter.

Covers C-cast float send, int round-and-compare, REAL32 tolerance, legacy
mav_param_type=None, fresh-response boundaries, cache invalidation, range
validation, subscription cancellation, and other-param-echo filtering.
"""
from __future__ import annotations

import threading
import time
import unittest
from unittest.mock import MagicMock, patch

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_PARAM_TYPE_INT8, MAV_PARAM_TYPE_INT16, MAV_PARAM_TYPE_INT32,
    MAV_PARAM_TYPE_REAL32,
)

from navpy.modules.vehicle.vehicle_mav import VehicleMav, _value_in_int_range


def _make_vehicle(self) -> VehicleMav:
    """Build a VehicleMav backed by a fully mocked MavBus connection."""
    self.fake_mav = MagicMock()
    self.fake_mav.file = MagicMock()
    self.fake_conn = MagicMock()
    self.fake_conn.mav = self.fake_mav
    self.fake_conn.recv_match = MagicMock(return_value=None)
    self.fake_conn.close = MagicMock()
    self.fake_conn.wait_heartbeat = MagicMock()
    self.logger = MagicMock()

    patcher = patch(
        "navpy.modules.vehicle.mav_bus.mavutil.mavlink_connection",
        return_value=self.fake_conn,
    )
    self.addCleanup(patcher.stop)
    patcher.start()

    dm = VehicleMav(
        device=f"unit:{id(self)}",
        target_system=42,
        logger=self.logger,
        wait_heartbeat=False,
        send_heartbeat=False,
        skip_mission_download=True,
    )
    self.addCleanup(dm.close)
    return dm


def _param_value_msg(name: str, value: float, param_type: int) -> MagicMock:
    msg = MagicMock()
    msg.param_id = name
    msg.param_value = value
    msg.param_type = param_type
    msg.get_srcSystem = MagicMock(return_value=42)
    msg.get_srcComponent = MagicMock(return_value=1)
    msg.get_type = MagicMock(return_value="PARAM_VALUE")
    msg.get_seq = MagicMock(return_value=0)
    msg.get_msgId = MagicMock(return_value=22)  # PARAM_VALUE id, irrelevant
    return msg


class _Test(unittest.TestCase):
    def _wait_until(self, predicate, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.01)
        return False


class TestRangeValidation(_Test):
    def test_value_in_int_range_int8(self):
        self.assertTrue(_value_in_int_range(127, MAV_PARAM_TYPE_INT8))
        self.assertTrue(_value_in_int_range(-128, MAV_PARAM_TYPE_INT8))
        self.assertFalse(_value_in_int_range(128, MAV_PARAM_TYPE_INT8))
        self.assertFalse(_value_in_int_range(-129, MAV_PARAM_TYPE_INT8))

    def test_value_in_int_range_int16(self):
        self.assertTrue(_value_in_int_range(32767, MAV_PARAM_TYPE_INT16))
        self.assertFalse(_value_in_int_range(70000, MAV_PARAM_TYPE_INT16))


class TestSetParameter(_Test):
    def setUp(self):
        self.dm = _make_vehicle(self)

    def _drive_echo(self, name, value, ptype):
        """Deliver a PARAM_VALUE in response to the next `param_set_send`.

        Uses a side_effect on the mock so the echo arrives deterministically
        once the verifier callback is registered, regardless of host load.
        Returns a sentinel thread (already finished) for API compatibility
        with callers that `t.join()` afterwards.
        """
        delivered = threading.Event()

        def _deliver_echo(*_args, **_kwargs):
            self.dm.feed_message(_param_value_msg(name, value, ptype))
            delivered.set()
        self.fake_mav.param_set_send.side_effect = _deliver_echo

        class _ImmediateThread:
            def join(self_inner, timeout=None):
                delivered.wait(timeout=timeout or 1.0)

        return _ImmediateThread()

    def test_real32_value_match(self):
        t = self._drive_echo("FOO", 1.25, MAV_PARAM_TYPE_REAL32)
        ok = self.dm.set_parameter(
            "FOO", 1.25, mav_param_type=MAV_PARAM_TYPE_REAL32, timeout=1.0)
        t.join()
        self.assertTrue(ok)
        # param_set_send was called with float(value).
        args, _kwargs = self.fake_mav.param_set_send.call_args
        self.assertEqual(args[0], 42)            # target_system
        self.assertEqual(args[2], b"FOO")        # name bytes
        self.assertAlmostEqual(args[3], 1.25)    # float value
        self.assertEqual(args[4], MAV_PARAM_TYPE_REAL32)

    def test_verified_sim_speedup_updates_scheduler_cache_immediately(self):
        t = self._drive_echo(
            "SIM_SPEEDUP",
            10.0,
            MAV_PARAM_TYPE_REAL32,
        )

        ok = self.dm.set_parameter(
            "SIM_SPEEDUP",
            10.0,
            mav_param_type=MAV_PARAM_TYPE_REAL32,
            timeout=1.0,
        )
        t.join()

        self.assertTrue(ok)
        self.assertEqual(self.dm.sim_speedup(), 10.0)
        self.assertTrue(self.dm.is_simulated_autopilot())

    def test_real32_value_mismatch_returns_false(self):
        t = self._drive_echo("FOO", 9.99, MAV_PARAM_TYPE_REAL32)
        ok = self.dm.set_parameter(
            "FOO", 1.25, mav_param_type=MAV_PARAM_TYPE_REAL32, timeout=0.5)
        t.join()
        self.assertFalse(ok)

    def test_int32_round_and_compare(self):
        # ArduPilot echoes the stored int as a float; we round before compare.
        t = self._drive_echo("INT_PARAM", 42.0, MAV_PARAM_TYPE_INT32)
        ok = self.dm.set_parameter(
            "INT_PARAM", 42, mav_param_type=MAV_PARAM_TYPE_INT32, timeout=1.0)
        t.join()
        self.assertTrue(ok)
        args, _ = self.fake_mav.param_set_send.call_args
        # Always C-cast float on the wire — never bytewise reinterpretation.
        self.assertAlmostEqual(args[3], 42.0)

    def test_int32_value_mismatch(self):
        t = self._drive_echo("INT_PARAM", 41.4, MAV_PARAM_TYPE_INT32)
        ok = self.dm.set_parameter(
            "INT_PARAM", 42, mav_param_type=MAV_PARAM_TYPE_INT32, timeout=0.5)
        t.join()
        self.assertFalse(ok)

    def test_legacy_no_type_value_only_check(self):
        """When the caller omits mav_param_type, we send REAL32 and verify
        value-only — echo with a different param_type still counts."""
        t = self._drive_echo("FOO", 7.0, MAV_PARAM_TYPE_INT32)
        ok = self.dm.set_parameter("FOO", 7.0, timeout=1.0)
        t.join()
        self.assertTrue(ok)
        args, _ = self.fake_mav.param_set_send.call_args
        self.assertEqual(args[4], MAV_PARAM_TYPE_REAL32)

    def test_strict_type_mismatch_does_not_match(self):
        t = self._drive_echo("FOO", 1.0, MAV_PARAM_TYPE_REAL32)
        ok = self.dm.set_parameter(
            "FOO", 1.0, mav_param_type=MAV_PARAM_TYPE_INT16, timeout=0.3)
        t.join()
        self.assertFalse(ok)

    def test_other_param_echo_ignored(self):
        # Deliver a wrong-name echo first, then the correct one — verifier
        # must filter by pname and only match the second.
        def _deliver(*_a, **_kw):
            self.dm.feed_message(_param_value_msg(
                "OTHER", 1.0, MAV_PARAM_TYPE_REAL32))
            self.dm.feed_message(_param_value_msg(
                "FOO", 5.0, MAV_PARAM_TYPE_REAL32))
        self.fake_mav.param_set_send.side_effect = _deliver
        ok = self.dm.set_parameter(
            "FOO", 5.0, mav_param_type=MAV_PARAM_TYPE_REAL32, timeout=1.0)
        self.assertTrue(ok)

    def test_timeout_does_not_accept_stale_response(self):
        self.dm.feed_message(_param_value_msg(
            "FOO", 1.0, MAV_PARAM_TYPE_REAL32,
        ))
        ok = self.dm.set_parameter(
            "FOO", 1.0, mav_param_type=MAV_PARAM_TYPE_REAL32, timeout=0.1)
        self.assertFalse(ok)

    def test_matching_echo_is_available_from_parameter_repository(self):
        t = self._drive_echo("FOO", 1.0, MAV_PARAM_TYPE_REAL32)
        ok = self.dm.set_parameter(
            "FOO", 1.0, mav_param_type=MAV_PARAM_TYPE_REAL32, timeout=1.0)
        t.join()
        self.assertTrue(ok)
        self.assertEqual(self.dm.get_parameter("FOO"), 1.0)

    def test_stale_parameter_invalidated_before_send(self):
        self.dm.feed_message(_param_value_msg(
            "FOO", 99.0, MAV_PARAM_TYPE_REAL32,
        ))
        self.assertEqual(self.dm.get_parameter("FOO"), 99.0)
        # Don't deliver an echo; let it time out.
        ok = self.dm.set_parameter(
            "FOO", 1.0, mav_param_type=MAV_PARAM_TYPE_REAL32, timeout=0.1)
        self.assertFalse(ok)
        self.fake_mav.param_request_read_send.reset_mock()
        self.assertIsNone(self.dm.get_parameter(
            "FOO", timeout=0.01, retries=1, quiet=True,
        ))
        self.fake_mav.param_request_read_send.assert_called_once()

    def test_int16_out_of_range_rejected_before_send(self):
        with patch.object(self.logger, "warning") as warn:
            ok = self.dm.set_parameter(
                "BIG", 70000, mav_param_type=MAV_PARAM_TYPE_INT16,
                timeout=0.1,
            )
        self.assertFalse(ok)
        warn.assert_called()
        self.fake_mav.param_set_send.assert_not_called()

    def test_subscription_cancel_is_idempotent(self):
        received = []
        subscription = self.dm.on_message("PARAM_VALUE", received.append)
        subscription.cancel()
        subscription.cancel()
        self.dm.feed_message(_param_value_msg(
            "FOO", 1.0, MAV_PARAM_TYPE_REAL32,
        ))
        self.assertEqual(received, [])

    def test_fractional_int_rejected_before_send(self):
        ok = self.dm.set_parameter(
            "FOO", 1.5, mav_param_type=MAV_PARAM_TYPE_INT32, timeout=0.1)
        self.assertFalse(ok)
        self.fake_mav.param_set_send.assert_not_called()

    def test_inf_rejected(self):
        ok = self.dm.set_parameter(
            "FOO", float("inf"), mav_param_type=MAV_PARAM_TYPE_REAL32,
            timeout=0.1,
        )
        self.assertFalse(ok)
        self.fake_mav.param_set_send.assert_not_called()

    def test_nan_rejected(self):
        ok = self.dm.set_parameter(
            "FOO", float("nan"), mav_param_type=MAV_PARAM_TYPE_REAL32,
            timeout=0.1,
        )
        self.assertFalse(ok)
        self.fake_mav.param_set_send.assert_not_called()

    def test_concurrent_writes_serialised(self):
        """Two concurrent set_parameter calls must each get their own echo.

        We drive an echo synchronously in response to each `param_set_send`
        call so the lock's serialisation behaviour is exercised directly:
        the second writer can only see its echo after the first has
        released the lock.
        """
        results = {}

        active_calls = []  # list of (name, value, ptype)
        active_cv = threading.Condition()

        def _send_recorder(target_sys, comp, name_bytes, value, ptype):
            with active_cv:
                active_calls.append((name_bytes.decode().rstrip("\x00"),
                                     value, ptype))
                active_cv.notify_all()
        self.fake_mav.param_set_send.side_effect = _send_recorder

        def driver():
            seen = 0
            while seen < 2:
                with active_cv:
                    while len(active_calls) <= seen:
                        active_cv.wait(timeout=2.0)
                    name, value, ptype = active_calls[seen]
                seen += 1
                # Deliver the matching echo for whichever call just happened.
                self.dm.feed_message(_param_value_msg(name, value, ptype))
        d = threading.Thread(target=driver, daemon=True); d.start()

        def writer(name, value, ptype):
            results[name] = self.dm.set_parameter(
                name, value, mav_param_type=ptype, timeout=2.0,
            )

        t1 = threading.Thread(
            target=writer, args=("FOO", 1.0, MAV_PARAM_TYPE_REAL32), daemon=True,
        )
        t2 = threading.Thread(
            target=writer, args=("BAR", 2.0, MAV_PARAM_TYPE_REAL32), daemon=True,
        )
        t1.start(); t2.start()
        t1.join(timeout=5.0); t2.join(timeout=5.0); d.join(timeout=5.0)

        self.assertTrue(results.get("FOO"))
        self.assertTrue(results.get("BAR"))
        # Exactly two sends — one per writer, fully serialised.
        self.assertEqual(self.fake_mav.param_set_send.call_count, 2)


if __name__ == "__main__":
    unittest.main()
