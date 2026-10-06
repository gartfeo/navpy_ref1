"""Connect defers the mission probe off the critical path (instant cards).

add_vehicle must register the vehicle and return immediately, running the
blocking mission probe on a background thread, so a vehicle's UAV-status card
can render without waiting for its mission download to finish.
"""
import threading
import time
import unittest
from unittest.mock import MagicMock, patch

from gcs.backend.vehicle_manager import VehicleManager


class TestConnectAsyncProbe(unittest.TestCase):
    def setUp(self):
        self.mgr = VehicleManager()

    def tearDown(self):
        try:
            self.mgr.shutdown()
        except Exception:
            pass

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_add_vehicle_does_not_block_on_probe(self, mock_mavutil):
        conn = MagicMock()
        conn.recv_match.return_value = None
        mock_mavutil.mavlink_connection.return_value = conn

        probe_started = threading.Event()
        probe_may_finish = threading.Event()
        called_with = []

        def slow_probe(sys_id, expected=None):
            called_with.append(sys_id)
            probe_started.set()
            probe_may_finish.wait(timeout=5.0)  # hold the probe open

        self.mgr.probe_existing_mission = MagicMock(side_effect=slow_probe)

        t0 = time.time()
        entry = self.mgr.add_vehicle("test:async_probe", 1, "u1")
        elapsed = time.time() - t0

        # Vehicle is registered immediately (its card can render) ...
        self.assertIsNotNone(entry)
        self.assertIn(1, self.mgr._vehicles)
        # ... and add_vehicle did NOT block on the (up to 5 s) probe.
        self.assertLess(elapsed, 2.0)
        # The probe runs asynchronously, for the right sys_id.
        self.assertTrue(probe_started.wait(timeout=3.0))
        self.assertEqual(called_with, [1])
        probe_may_finish.set()  # release the probe thread

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_is_probing_true_during_probe_then_false(self, mock_mavutil):
        """is_probing drives the UI 'Downloading…' load stage in auto-connect
        mode: True from the moment the probe is kicked (synchronously, so the
        card never briefly reads 'ready') until the probe thread finishes."""
        conn = MagicMock()
        conn.recv_match.return_value = None
        mock_mavutil.mavlink_connection.return_value = conn

        probe_started = threading.Event()
        probe_may_finish = threading.Event()

        def slow_probe(sys_id, expected=None):
            probe_started.set()
            probe_may_finish.wait(timeout=5.0)

        self.mgr.probe_existing_mission = MagicMock(side_effect=slow_probe)

        entry = self.mgr.add_vehicle("test:probing_flag", 3, "u3")
        # Set synchronously at kick — true immediately, before the probe body runs.
        self.assertTrue(entry.is_probing)
        self.assertTrue(probe_started.wait(timeout=3.0))
        self.assertTrue(entry.is_probing)  # still probing while the probe holds

        probe_may_finish.set()  # let the probe thread finish
        # The thread's finally clears the flag.
        deadline = time.time() + 3.0
        while entry.is_probing and time.time() < deadline:
            time.sleep(0.01)
        self.assertFalse(entry.is_probing)

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_is_probing_exposed_in_snapshot(self, mock_mavutil):
        """The snapshot broadcast to the frontend carries is_probing so the card
        can render the load stage."""
        conn = MagicMock()
        conn.recv_match.return_value = None
        mock_mavutil.mavlink_connection.return_value = conn

        # Keep the probe from clearing the flag so we can observe True in a snapshot.
        self.mgr.probe_existing_mission = MagicMock(
            side_effect=lambda sys_id, expected=None: time.sleep(2.0)
        )
        entry = self.mgr.add_vehicle("test:probing_snap", 4, "u4")
        snap = entry.snapshot()
        self.assertIn("is_probing", snap)
        self.assertTrue(snap["is_probing"])

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_is_probing_cleared_if_probe_thread_cannot_start(self, mock_mavutil):
        """If the OS can't start the probe thread (thread exhaustion under load),
        is_probing must NOT stick True — otherwise the card is pinned on
        "Downloading…" for the vehicle's whole session. add_vehicle must also not
        raise (fail-open: the vehicle stays connected)."""
        conn = MagicMock()
        conn.recv_match.return_value = None
        mock_mavutil.mavlink_connection.return_value = conn

        self.mgr.probe_existing_mission = MagicMock()  # no-op so the initial probe is trivial
        entry = self.mgr.add_vehicle("test:thread_boom", 5, "u5")
        self.assertIsNotNone(entry)

        # Re-kick the probe with thread creation failing. The patch is scoped to
        # this single call, which creates exactly one thread (the probe) — so it
        # can't disturb the MavBus reader thread created during add_vehicle.
        with patch("gcs.backend.vehicle_manager.threading.Thread") as MockThread:
            MockThread.return_value.start.side_effect = RuntimeError("can't start new thread")
            self.mgr._probe_mission_async(entry)  # must not raise

        self.assertFalse(entry.is_probing)   # flag cleared despite start() failing
        self.assertIn("is_probing", entry.snapshot())

    @patch("navpy.modules.vehicle.mav_bus.mavutil")
    def test_async_probe_exception_does_not_escape(self, mock_mavutil):
        conn = MagicMock()
        conn.recv_match.return_value = None
        mock_mavutil.mavlink_connection.return_value = conn

        attempted = threading.Event()

        def boom(sys_id, expected=None):
            try:
                raise RuntimeError("probe boom")
            finally:
                attempted.set()

        self.mgr.probe_existing_mission = MagicMock(side_effect=boom)
        # add_vehicle must not raise even though the async probe throws.
        self.mgr.add_vehicle("test:async_boom", 2, "u2")
        self.assertTrue(attempted.wait(timeout=3.0))


if __name__ == "__main__":
    unittest.main()
