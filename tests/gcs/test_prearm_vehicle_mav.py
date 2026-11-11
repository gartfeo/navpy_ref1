"""Tests for VehicleMav.prearm_ok property and snapshot integration."""
import threading
import types
import unittest
from unittest.mock import MagicMock

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_COMP_ID_AUTOPILOT1,
    MAV_SYS_STATUS_PREARM_CHECK,
)

from navpy.plugins.vehicle.ardupilot.vehicle_mav import VehicleMav


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


def _statustext(vehicle, text: str) -> None:
    """Deliver an autopilot STATUSTEXT through the real subscription path."""
    _feed(vehicle, "STATUSTEXT", types.SimpleNamespace(severity=3, text=text))


def _autopilot_clock(vehicle, boot_time_ms: int) -> None:
    """Advance the autopilot clock (time_boot_ms) via GLOBAL_POSITION_INT."""
    _feed(vehicle, "GLOBAL_POSITION_INT", types.SimpleNamespace(
        time_boot_ms=boot_time_ms, lat=0, lon=0, alt=0, relative_alt=0,
        vx=0, vy=0, vz=0, hdg=0,
    ))


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
        _statustext(v, "PreArm: Mode not armable")
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
        _statustext(v, "PreArm: Mode not armable")
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
        _statustext(v, "PreArm: Mode not armable")
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
        _statustext(v, "PreArm: Mode not armable")
        entry._refresh_prearm_flags()  # clears every armable cycle, not just transitions
        # Real failure follows with no fresh STATUSTEXT -> must NOT be exempt.
        _feed(v, "SYS_STATUS", _make_sys_status(present=bit, enabled=bit, health=0))
        entry._refresh_prearm_flags()
        self.assertFalse(entry.prearm_mode_not_armable_only)


class TestPrearmReasonsAgeOut(unittest.TestCase):
    """The exemption follows ArduPilot's *current* PreArm reasons: a reason it
    stopped re-reporting (every 30 s of autopilot time) ages out, a reason it
    keeps repeating keeps blocking. Regression for a single transient
    "Accels inconsistent" blocking launch while the UAV sat in RTL."""

    def _failed_entry(self):
        bit = MAV_SYS_STATUS_PREARM_CHECK
        v = _make_vehicle(self)
        entry = _make_entry(v)
        _feed(v, "SYS_STATUS", _make_sys_status(present=bit, enabled=bit, health=0))
        entry._refresh_prearm_flags()
        return v, entry

    def _report_cycles(self, v, entry, start_ms, cycles, texts_per_cycle):
        """ArduPilot report bursts 30 s apart (autopilot clock), with a
        snapshot refresh after each, as the telemetry loop does."""
        for i in range(cycles):
            _autopilot_clock(v, start_ms + i * 30_000)
            for text in texts_per_cycle:
                _statustext(v, text)
            entry._refresh_prearm_flags()

    def test_stale_reason_then_mode_only_repeats_allows_launch(self):
        v, entry = self._failed_entry()
        _autopilot_clock(v, 100_000)
        _statustext(v, "PreArm: Accels inconsistent")
        _statustext(v, "PreArm: RTL mode not armable")
        self.assertFalse(entry.prearm_mode_not_armable_only)
        # Only the mode reason repeats afterwards.
        self._report_cycles(v, entry, 130_000, 2, ["PreArm: RTL mode not armable"])
        self.assertFalse(entry.prearm_mode_not_armable_only,
                         "one missed report must not drop a reason")
        self._report_cycles(v, entry, 190_000, 1, ["PreArm: RTL mode not armable"])
        self.assertTrue(entry.prearm_mode_not_armable_only)

    def test_still_repeating_real_reason_blocks(self):
        v, entry = self._failed_entry()
        self._report_cycles(v, entry, 100_000, 10, [
            "PreArm: Accels inconsistent", "PreArm: RTL mode not armable",
        ])
        self.assertFalse(entry.prearm_mode_not_armable_only)

    def test_real_reason_survives_one_lost_report(self):
        v, entry = self._failed_entry()
        self._report_cycles(v, entry, 100_000, 1, [
            "PreArm: Accels inconsistent", "PreArm: RTL mode not armable",
        ])
        # Accels STATUSTEXT lost on the link in the next report only.
        self._report_cycles(v, entry, 130_000, 1, ["PreArm: RTL mode not armable"])
        self._report_cycles(v, entry, 160_000, 1, [
            "PreArm: Accels inconsistent", "PreArm: RTL mode not armable",
        ])
        self.assertFalse(entry.prearm_mode_not_armable_only)

    def test_wall_time_alone_does_not_age_reasons(self):
        """Ageing is on the autopilot clock: with no newer autopilot time
        (link silent, sim paused) a reason never expires."""
        v, entry = self._failed_entry()
        _autopilot_clock(v, 100_000)
        _statustext(v, "PreArm: Accels inconsistent")
        _statustext(v, "PreArm: RTL mode not armable")
        for _ in range(5):
            _statustext(v, "PreArm: RTL mode not armable")
            entry._refresh_prearm_flags()
        self.assertFalse(entry.prearm_mode_not_armable_only)

    def test_foreign_clock_source_is_ignored(self):
        """Only this vehicle's own autopilot advances the reason clock."""
        v, entry = self._failed_entry()
        _autopilot_clock(v, 100_000)
        _statustext(v, "PreArm: Accels inconsistent")
        _statustext(v, "PreArm: RTL mode not armable")
        companion = types.SimpleNamespace(
            time_boot_ms=900_000, get_srcSystem=lambda: 1,
            get_srcComponent=lambda: 191)
        entry._on_autopilot_clock(companion)
        _statustext(v, "PreArm: RTL mode not armable")
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
