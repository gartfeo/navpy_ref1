"""Tests for companion computer heartbeat tracking in VehicleManager."""
import time
import unittest
from unittest.mock import MagicMock, patch

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_TYPE_FIXED_WING,
    MAV_TYPE_ONBOARD_CONTROLLER,
)

from navpy.modules.vehicle.mav_bus import MavBus
from gcs.backend.vehicle_manager import (
    VehicleManager,
    COMPANION_OK_WINDOW_S,
    COMPANION_DOWN_WINDOW_S,
)


class TestMavBusCompanionHeartbeat(unittest.TestCase):
    """Verify MavBus tracks companion heartbeats separately."""

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_companion_heartbeats_dict_exists(self, mock_mavutil):
        conn = MagicMock()
        conn.recv_match.return_value = None
        mock_mavutil.mavlink_connection.return_value = conn
        bus = MavBus.get_or_create("test:companion1", source_system=255)
        self.assertIsInstance(bus.companion_heartbeats, dict)
        self.assertEqual(len(bus.companion_heartbeats), 0)
        bus.close()

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_companion_heartbeat_tracked(self, mock_mavutil):
        """Simulate a MAV_TYPE_ONBOARD_CONTROLLER heartbeat being recorded."""
        conn = MagicMock()
        conn.recv_match.return_value = None
        mock_mavutil.mavlink_connection.return_value = conn
        bus = MavBus.get_or_create("test:companion2", source_system=255)
        _record_heartbeat(bus, 1, MAV_TYPE_ONBOARD_CONTROLLER)
        self.assertIn(1, bus.companion_heartbeats)
        bus.close()

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_vehicle_heartbeat_not_in_companion(self, mock_mavutil):
        """Autopilot and companion heartbeats now share one srcSystem, so the
        MAV_TYPE split is the only thing keeping them apart: MavBus files
        MAV_TYPE_ONBOARD_CONTROLLER in companion_heartbeats and every other
        non-GCS type in heartbeats, two separate dicts that never alias."""
        conn = MagicMock()
        conn.recv_match.return_value = None
        mock_mavutil.mavlink_connection.return_value = conn
        bus = MavBus.get_or_create("test:companion3", source_system=255)
        _record_heartbeat(bus, 1, MAV_TYPE_FIXED_WING)
        self.assertNotIn(1, bus.companion_heartbeats)
        bus.close()


def _heartbeat(sys_id: int, mav_type: int):
    msg = MagicMock()
    msg.get_type.return_value = "HEARTBEAT"
    msg.get_srcSystem.return_value = sys_id
    msg.type = mav_type
    return msg


def _record_heartbeat(
    bus: MavBus,
    sys_id: int,
    mav_type: int,
    age_s: float = 0.0,
) -> None:
    """Drive the heartbeat owner through the same observation path as MAVLink."""
    observed_at = time.time() - age_s
    with patch(
        "navpy.modules.vehicle.mav_bus_heartbeats.time.time",
        return_value=observed_at,
    ):
        bus._heartbeats.observe(_heartbeat(sys_id, mav_type))


class TestSharedSysIdHeartbeatSplit(unittest.TestCase):
    """The companion now shares its aircraft's system id, so both heartbeats
    arrive with the SAME srcSystem. MavBus must still keep them apart -- the
    only discriminator left is MAV_TYPE (MAV_TYPE_ONBOARD_CONTROLLER goes to
    companion_heartbeats, every other non-GCS type to heartbeats), and the two
    dicts are independent so one key can hold both without aliasing.
    """

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_same_sys_id_lands_in_both_dicts_without_aliasing(self, mock_mavutil):
        conn = MagicMock()
        conn.recv_match.side_effect = [
            _heartbeat(7, MAV_TYPE_FIXED_WING),
            _heartbeat(7, MAV_TYPE_ONBOARD_CONTROLLER),
        ] + [None] * 10_000
        mock_mavutil.mavlink_connection.return_value = conn
        bus = MavBus.get_or_create("test:shared_sysid_split", source_system=255)
        try:
            deadline = time.time() + 2.0
            while time.time() < deadline:
                if 7 in bus.heartbeats and 7 in bus.companion_heartbeats:
                    break
                time.sleep(0.01)

            # Autopilot heartbeat is NOT mistaken for the companion, and the
            # companion heartbeat is NOT mistaken for the autopilot.
            self.assertIn(7, bus.heartbeats)
            self.assertIn(7, bus.companion_heartbeats)
            self.assertIsNot(bus.heartbeats, bus.companion_heartbeats)
        finally:
            bus.close()

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_autopilot_only_never_marks_the_companion_alive(self, mock_mavutil):
        conn = MagicMock()
        conn.recv_match.side_effect = [
            _heartbeat(7, MAV_TYPE_FIXED_WING),
        ] + [None] * 10_000
        mock_mavutil.mavlink_connection.return_value = conn
        bus = MavBus.get_or_create("test:autopilot_only_split", source_system=255)
        try:
            deadline = time.time() + 2.0
            while time.time() < deadline and 7 not in bus.heartbeats:
                time.sleep(0.01)

            self.assertIn(7, bus.heartbeats)
            self.assertNotIn(7, bus.companion_heartbeats)
        finally:
            bus.close()


class TestVehicleManagerCompanion(unittest.TestCase):
    """Tests for VehicleManager.is_companion_active and snapshot companion_ok."""

    def setUp(self):
        self.mgr = VehicleManager()

    def test_is_companion_active_no_buses(self):
        """No buses -> companion not active."""
        self.assertFalse(self.mgr.is_companion_active(1))

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_is_companion_active_recent(self, mock_mavutil):
        """Recent companion heartbeat -> active."""
        conn = MagicMock()
        conn.recv_match.return_value = None
        mock_mavutil.mavlink_connection.return_value = conn
        bus = self.mgr._get_or_create_bus("test:comp_recent")
        _record_heartbeat(bus, 1, MAV_TYPE_ONBOARD_CONTROLLER)
        self.assertTrue(self.mgr.is_companion_active(1))
        bus.close()
        self.mgr._buses.pop("test:comp_recent", None)

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_is_companion_active_uses_the_shared_sys_id(self, mock_mavutil):
        """The companion heartbeat carries the aircraft sysid, so the liveness
        lookup keys straight on sys_id."""
        conn = MagicMock()
        conn.recv_match.return_value = None
        mock_mavutil.mavlink_connection.return_value = conn
        bus = self.mgr._get_or_create_bus("test:comp_mapped")
        _record_heartbeat(bus, 1, MAV_TYPE_ONBOARD_CONTROLLER)
        self.assertTrue(self.mgr.is_companion_active(1))
        self.assertEqual(self.mgr.companion_status(1), "ok")
        bus.close()
        self.mgr._buses.pop("test:comp_mapped", None)

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_is_companion_active_stale(self, mock_mavutil):
        """Stale companion heartbeat -> not active."""
        conn = MagicMock()
        conn.recv_match.return_value = None
        mock_mavutil.mavlink_connection.return_value = conn
        bus = self.mgr._get_or_create_bus("test:comp_stale")
        _record_heartbeat(bus, 1, MAV_TYPE_ONBOARD_CONTROLLER, age_s=10.0)
        self.assertFalse(self.mgr.is_companion_active(1, timeout=5.0))
        bus.close()
        self.mgr._buses.pop("test:comp_stale", None)

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_is_companion_active_different_sys_id(self, mock_mavutil):
        """Companion heartbeat for sys_id 2 does not make sys_id 1 active."""
        conn = MagicMock()
        conn.recv_match.return_value = None
        mock_mavutil.mavlink_connection.return_value = conn
        bus = self.mgr._get_or_create_bus("test:comp_diff")
        _record_heartbeat(bus, 2, MAV_TYPE_ONBOARD_CONTROLLER)
        self.assertFalse(self.mgr.is_companion_active(1))
        bus.close()
        self.mgr._buses.pop("test:comp_diff", None)

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_snapshot_includes_companion_ok(self, mock_mavutil):
        """get_all_snapshots includes companion_ok in each snapshot."""
        mock_conn = MagicMock()
        mock_conn.recv_match.return_value = None
        mock_mavutil.mavlink_connection.return_value = mock_conn

        entry = self.mgr.add_vehicle("test:snap_comp", 1, "test-uav")
        # Set a recent companion heartbeat on the bus
        bus = self.mgr._buses["test:snap_comp"]
        _record_heartbeat(bus, 1, MAV_TYPE_ONBOARD_CONTROLLER)

        snapshots = self.mgr.get_all_snapshots()
        self.assertEqual(len(snapshots), 1)
        self.assertIn("companion_ok", snapshots[0])
        self.assertTrue(snapshots[0]["companion_ok"])
        self.mgr.shutdown()

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_snapshot_companion_ok_false_when_no_heartbeat(self, mock_mavutil):
        """companion_ok is False when no companion heartbeat received."""
        mock_conn = MagicMock()
        mock_conn.recv_match.return_value = None
        mock_mavutil.mavlink_connection.return_value = mock_conn

        entry = self.mgr.add_vehicle("test:snap_nocomp", 1, "test-uav")

        snapshots = self.mgr.get_all_snapshots()
        self.assertEqual(len(snapshots), 1)
        self.assertFalse(snapshots[0]["companion_ok"])
        self.mgr.shutdown()


class TestCompanionStatusTriState(unittest.TestCase):
    """Tri-state companion liveness: ok / checking / down.

    A brief gap in the 1 Hz companion heartbeat on the shared lossy link is
    ordinary packet loss, not a dead companion. The tri-state reports it as
    "checking" (transient) so the badge and launch-readiness don't flap to
    "faulty" — the bug this fix targets.
    """

    def setUp(self):
        self.mgr = VehicleManager()

    def _bus_with_hb_age(self, device, sys_id, age_s, mock_mavutil):
        conn = MagicMock()
        conn.recv_match.return_value = None
        mock_mavutil.mavlink_connection.return_value = conn
        bus = self.mgr._get_or_create_bus(device)
        _record_heartbeat(
            bus,
            sys_id,
            MAV_TYPE_ONBOARD_CONTROLLER,
            age_s=age_s,
        )
        return bus

    def _cleanup(self, device, bus):
        bus.close()
        self.mgr._buses.pop(device, None)

    def test_status_never_seen_is_down(self):
        self.assertEqual(self.mgr.companion_status(1), "down")

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_status_fresh_is_ok(self, mock_mavutil):
        bus = self._bus_with_hb_age("test:cs_ok", 1, 0.0, mock_mavutil)
        self.assertEqual(self.mgr.companion_status(1), "ok")
        self._cleanup("test:cs_ok", bus)

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_status_transient_gap_is_checking_not_down(self, mock_mavutil):
        """A mid-band (~10 s) gap is 'checking', not 'down'. The old 5 s
        is_companion_active() called this inactive/faulty (the flap)."""
        age = (COMPANION_OK_WINDOW_S + COMPANION_DOWN_WINDOW_S) / 2.0
        bus = self._bus_with_hb_age("test:cs_check", 1, age, mock_mavutil)
        self.assertEqual(self.mgr.companion_status(1), "checking")
        # Regression: the pre-fix 5 s check reports this gap as faulty.
        self.assertFalse(
            self.mgr.is_companion_active(1, timeout=COMPANION_OK_WINDOW_S)
        )
        self._cleanup("test:cs_check", bus)

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_status_sustained_silence_is_down(self, mock_mavutil):
        bus = self._bus_with_hb_age(
            "test:cs_down", 1, COMPANION_DOWN_WINDOW_S + 5.0, mock_mavutil
        )
        self.assertEqual(self.mgr.companion_status(1), "down")
        self._cleanup("test:cs_down", bus)

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_snapshot_checking_keeps_companion_ok_true(self, mock_mavutil):
        """Snapshot exposes companion_status; during 'checking', companion_ok
        stays True so prearm launch-readiness does not trip."""
        mock_conn = MagicMock()
        mock_conn.recv_match.return_value = None
        mock_mavutil.mavlink_connection.return_value = mock_conn
        self.mgr.add_vehicle("test:snap_check", 1, "test-uav")
        bus = self.mgr._buses["test:snap_check"]
        _record_heartbeat(
            bus,
            1,
            MAV_TYPE_ONBOARD_CONTROLLER,
            age_s=COMPANION_OK_WINDOW_S + 2.0,
        )
        snaps = self.mgr.get_all_snapshots()
        self.assertEqual(snaps[0]["companion_status"], "checking")
        self.assertTrue(snaps[0]["companion_ok"])
        self.mgr.shutdown()

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_snapshot_down_sets_companion_ok_false(self, mock_mavutil):
        mock_conn = MagicMock()
        mock_conn.recv_match.return_value = None
        mock_mavutil.mavlink_connection.return_value = mock_conn
        self.mgr.add_vehicle("test:snap_down", 1, "test-uav")
        bus = self.mgr._buses["test:snap_down"]
        _record_heartbeat(
            bus,
            1,
            MAV_TYPE_ONBOARD_CONTROLLER,
            age_s=COMPANION_DOWN_WINDOW_S + 5.0,
        )
        snaps = self.mgr.get_all_snapshots()
        self.assertEqual(snaps[0]["companion_status"], "down")
        self.assertFalse(snaps[0]["companion_ok"])
        self.mgr.shutdown()


if __name__ == "__main__":
    unittest.main()
