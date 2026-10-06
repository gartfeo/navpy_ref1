"""Tests for VehicleMav.prearm_ok property and snapshot integration."""
import threading
import types
import unittest
from unittest.mock import MagicMock

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_COMP_ID_AUTOPILOT1,
    MAV_SYS_STATUS_PREARM_CHECK,
)

from navpy.modules.vehicle.vehicle_mav import VehicleMav


def _make_sys_status(present=0, enabled=0, health=0):
    """Create a minimal SYS_STATUS-like object with sensor bitmasks."""
    return types.SimpleNamespace(
        onboard_control_sensors_present=present,
        onboard_control_sensors_enabled=enabled,
        onboard_control_sensors_health=health,
        voltage_battery=0,
        current_battery=-1,
    )


def _make_vehicle(test_case: unittest.TestCase | None = None):
    """Construct through the public composition root on a connection-free bus."""
    connection = types.SimpleNamespace(mav=MagicMock())
    bus = MagicMock(
        conn=connection,
        send_lock=threading.RLock(),
        heartbeats={},
    )
    bus.reserve.return_value = MagicMock()
    vehicle = VehicleMav(
        "unused",
        target_system=1,
        logger=MagicMock(),
        skip_mission_download=True,
        wait_heartbeat=False,
        send_heartbeat=False,
        bus=bus,
    )
    if test_case is not None:
        test_case.addCleanup(vehicle.close)
    return vehicle


def _feed(vehicle, message_type: str, message) -> None:
    message.get_msgId = lambda: 0
    message.get_srcSystem = lambda: 1
    message.get_srcComponent = lambda: MAV_COMP_ID_AUTOPILOT1
    message.get_seq = lambda: 0
    message.get_type = lambda: message_type
    vehicle.feed_message(message)


def _make_entry(vehicle):
    """Build VehicleEntry against the normal public VehicleMav surface."""
    from gcs.backend.vehicle_manager import VehicleEntry
    return VehicleEntry(1, vehicle, "test")


class TestPrearmOk(unittest.TestCase):
    """VehicleMav.prearm_ok property."""

    def test_none_when_no_sys_status(self):
        v = _make_vehicle()
        self.assertIsNone(v.prearm_ok)

    def test_none_when_prearm_bit_not_present(self):
        v = _make_vehicle()
        _feed(v, "SYS_STATUS", _make_sys_status(present=0, enabled=0, health=0))
        self.assertIsNone(v.prearm_ok)

    def test_none_when_prearm_present_but_not_enabled(self):
        v = _make_vehicle()
        bit = MAV_SYS_STATUS_PREARM_CHECK
        _feed(v, "SYS_STATUS", _make_sys_status(present=bit, enabled=0, health=0))
        self.assertIsNone(v.prearm_ok)

    def test_true_when_healthy(self):
        v = _make_vehicle()
        bit = MAV_SYS_STATUS_PREARM_CHECK
        _feed(v, "SYS_STATUS", _make_sys_status(
            present=bit, enabled=bit, health=bit,
        ))
        self.assertIs(v.prearm_ok, True)

    def test_false_when_not_healthy(self):
        v = _make_vehicle()
        bit = MAV_SYS_STATUS_PREARM_CHECK
        _feed(v, "SYS_STATUS", _make_sys_status(
            present=bit, enabled=bit, health=0,
        ))
        self.assertIs(v.prearm_ok, False)

    def test_true_with_other_bits_set(self):
        v = _make_vehicle()
        bit = MAV_SYS_STATUS_PREARM_CHECK
        other = 0x01 | 0x02 | 0x04
        _feed(v, "SYS_STATUS", _make_sys_status(
            present=bit | other, enabled=bit | other, health=bit | other,
        ))
        self.assertIs(v.prearm_ok, True)

    def test_false_with_other_bits_healthy(self):
        v = _make_vehicle()
        bit = MAV_SYS_STATUS_PREARM_CHECK
        other = 0x01 | 0x02
        _feed(v, "SYS_STATUS", _make_sys_status(
            present=bit | other, enabled=bit | other, health=other,
        ))
        self.assertIs(v.prearm_ok, False)


class TestPrearmCheckState(unittest.TestCase):
    """VehicleMav.prearm_check_state distinguishes the None cases."""

    def test_no_sys_status(self):
        v = _make_vehicle()
        self.assertEqual(v.prearm_check_state, "no_sys_status")

    def test_not_reported_when_present_bit_absent(self):
        v = _make_vehicle()
        _feed(v, "SYS_STATUS", _make_sys_status(present=0, enabled=0, health=0))
        self.assertEqual(v.prearm_check_state, "not_reported")

    def test_checks_disabled_when_present_but_not_enabled(self):
        v = _make_vehicle()
        bit = MAV_SYS_STATUS_PREARM_CHECK
        _feed(v, "SYS_STATUS", _make_sys_status(present=bit, enabled=0, health=0))
        self.assertEqual(v.prearm_check_state, "checks_disabled")

    def test_failed_when_enabled_but_not_healthy(self):
        v = _make_vehicle()
        bit = MAV_SYS_STATUS_PREARM_CHECK
        _feed(v, "SYS_STATUS", _make_sys_status(present=bit, enabled=bit, health=0))
        self.assertEqual(v.prearm_check_state, "failed")

    def test_ok_when_present_enabled_healthy(self):
        v = _make_vehicle()
        bit = MAV_SYS_STATUS_PREARM_CHECK
        _feed(v, "SYS_STATUS", _make_sys_status(
            present=bit, enabled=bit, health=bit,
        ))
        self.assertEqual(v.prearm_check_state, "ok")

    def test_other_bits_do_not_affect_state(self):
        v = _make_vehicle()
        bit = MAV_SYS_STATUS_PREARM_CHECK
        other = 0x01 | 0x02 | 0x04
        # PREARM present+enabled but its health bit clear (other bits healthy)
        _feed(v, "SYS_STATUS", _make_sys_status(
            present=bit | other, enabled=bit | other, health=other,
        ))
        self.assertEqual(v.prearm_check_state, "failed")

    def test_prearm_ok_matches_state(self):
        """prearm_ok stays a faithful tri-state projection of the state."""
        bit = MAV_SYS_STATUS_PREARM_CHECK
        cases = [
            (None, None),  # no SYS_STATUS -> no_sys_status -> None
            (_make_sys_status(present=0), None),  # not_reported -> None
            (_make_sys_status(present=bit), None),  # checks_disabled -> None
            (_make_sys_status(present=bit, enabled=bit), False),  # failed
            (_make_sys_status(present=bit, enabled=bit, health=bit), True),  # ok
        ]
        for ss, expected in cases:
            v = _make_vehicle(self)
            if ss is not None:
                _feed(v, "SYS_STATUS", ss)
            self.assertIs(v.prearm_ok, expected)


class TestPrearmModeNotArmableReset(unittest.TestCase):
    """Stale 'mode not armable' flags must reset on ANY armable state (ok or
    checks_disabled) so they can't exempt a later real failure."""

    def _entry(self, vehicle):
        return _make_entry(vehicle)

    def test_resets_when_armable_observed_via_cycle_not_property(self):
        """The real backend path: the armable state is observed by the per-cycle
        refresh (snapshot), NOT by reading the launch-gate property. A stale
        mode-only flag must still be cleared so a later real failure blocks."""
        bit = MAV_SYS_STATUS_PREARM_CHECK
        v = _make_vehicle()
        entry = self._entry(v)
        # 1) failed with only "mode not armable" -> exempt
        _feed(v, "SYS_STATUS", _make_sys_status(present=bit, enabled=bit, health=0))
        entry._prearm_mode_only = True
        entry._refresh_prearm_flags()
        self.assertTrue(entry.prearm_mode_not_armable_only)
        # 2) becomes armable via checks_disabled — observed ONLY by the cycle refresh
        _feed(v, "SYS_STATUS", _make_sys_status(present=bit, enabled=0, health=0))
        entry._refresh_prearm_flags()
        # 3) real failure again, no fresh STATUSTEXT -> must NOT be exempt
        _feed(v, "SYS_STATUS", _make_sys_status(present=bit, enabled=bit, health=0))
        entry._refresh_prearm_flags()
        self.assertFalse(entry.prearm_mode_not_armable_only)

    def test_resets_through_ok(self):
        bit = MAV_SYS_STATUS_PREARM_CHECK
        v = _make_vehicle()
        entry = self._entry(v)
        _feed(v, "SYS_STATUS", _make_sys_status(present=bit, enabled=bit, health=0))
        entry._prearm_mode_only = True
        entry._refresh_prearm_flags()
        self.assertTrue(entry.prearm_mode_not_armable_only)
        _feed(v, "SYS_STATUS", _make_sys_status(present=bit, enabled=bit, health=bit))
        entry._refresh_prearm_flags()
        _feed(v, "SYS_STATUS", _make_sys_status(present=bit, enabled=bit, health=0))
        entry._refresh_prearm_flags()
        self.assertFalse(entry.prearm_mode_not_armable_only)

    def test_mode_only_still_exempts_while_failed(self):
        """Without an armable transition, the exemption still holds across cycles."""
        bit = MAV_SYS_STATUS_PREARM_CHECK
        v = _make_vehicle()
        entry = self._entry(v)
        _feed(v, "SYS_STATUS", _make_sys_status(present=bit, enabled=bit, health=0))
        entry._prearm_mode_only = True
        entry._refresh_prearm_flags()
        self.assertTrue(entry.prearm_mode_not_armable_only)
        entry._refresh_prearm_flags()
        self.assertTrue(entry.prearm_mode_not_armable_only)

    def test_stale_flag_set_while_already_armable_is_cleared(self):
        """A late 'mode not armable' flag set while ALREADY armable (no transition)
        must still be cleared, so it can't exempt a later real failure."""
        bit = MAV_SYS_STATUS_PREARM_CHECK
        v = _make_vehicle()
        entry = self._entry(v)
        # Already armable, flags clear.
        _feed(v, "SYS_STATUS", _make_sys_status(present=bit, enabled=bit, health=bit))
        entry._refresh_prearm_flags()
        # A stale flag gets set while still armable (no armable transition occurs).
        entry._prearm_mode_only = True
        entry._refresh_prearm_flags()  # clears every armable cycle, not just transitions
        # Real failure follows with no fresh STATUSTEXT -> must NOT be exempt.
        _feed(v, "SYS_STATUS", _make_sys_status(present=bit, enabled=bit, health=0))
        entry._refresh_prearm_flags()
        self.assertFalse(entry.prearm_mode_not_armable_only)


class TestSnapshotPrearmOk(unittest.TestCase):
    """VehicleEntry.snapshot() includes prearm_ok."""

    def test_snapshot_includes_prearm_ok(self):
        v = _make_vehicle(self)

        bit = MAV_SYS_STATUS_PREARM_CHECK
        _feed(v, "SYS_STATUS", _make_sys_status(
            present=bit, enabled=bit, health=bit,
        ))

        entry = _make_entry(v)
        snap = entry.snapshot()
        self.assertIn("prearm_ok", snap)
        self.assertIs(snap["prearm_ok"], True)
        self.assertEqual(snap["prearm_check_state"], "ok")

    def test_snapshot_reports_checks_disabled(self):
        v = _make_vehicle(self)

        bit = MAV_SYS_STATUS_PREARM_CHECK
        # present but not enabled -> arming checks disabled
        _feed(v, "SYS_STATUS", _make_sys_status(present=bit, enabled=0, health=0))

        entry = _make_entry(v)
        snap = entry.snapshot()
        # prearm_ok stays None, but the state distinguishes the case
        self.assertIsNone(snap["prearm_ok"])
        self.assertEqual(snap["prearm_check_state"], "checks_disabled")


if __name__ == "__main__":
    unittest.main()
