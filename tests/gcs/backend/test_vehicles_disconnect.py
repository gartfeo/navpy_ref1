"""Tests for POST /api/vehicles/disconnect companion cleanup (issue #163).

Explicit operator disconnect must stop the vehicle's GCS-managed NavPy
companion and clear its auto-managed intent, so no orphaned process keeps a
serial0 link and no stale intent resumes restarts on re-discovery.
"""
import asyncio
import threading
import unittest
from unittest.mock import AsyncMock, MagicMock, patch

from gcs.backend import navpy_sim_runtime as runtime_mod
from gcs.backend.routes import navpy_sim as navpy_mod
from gcs.backend.routes import vehicles as vehicles_mod
from gcs.backend.vehicle_manager import vehicle_mgr


def _reset_runtime_state():
    runtime_mod._navpy_mgr = None
    runtime_mod._auto_managed_sysids.clear()


class TestDisconnectStopsCompanion(unittest.TestCase):

    def setUp(self):
        _reset_runtime_state()

    def tearDown(self):
        _reset_runtime_state()

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(vehicles_mod.vehicle_mgr, "remove_vehicle", return_value=[3])
    def test_disconnect_stops_companion_and_clears_intent(
            self, mock_remove, mock_broadcast):
        mock_mgr = MagicMock()
        mock_mgr.stop.return_value = True
        runtime_mod._navpy_mgr = mock_mgr
        runtime_mod._auto_managed_sysids.add(3)

        result = asyncio.run(vehicles_mod.disconnect_vehicle(3))

        mock_remove.assert_called_once_with(3)
        mock_mgr.stop.assert_called_once_with(3)
        self.assertNotIn(3, runtime_mod._auto_managed_sysids)
        mock_broadcast.assert_awaited_once()
        self.assertEqual(
            result, {"status": "disconnected", "sys_id": 3, "removed": [3]})

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(vehicles_mod.vehicle_mgr, "remove_vehicle", return_value=[])
    def test_unknown_sysid_is_full_noop(self, mock_remove, mock_broadcast):
        """Disconnecting a never-connected sys_id must not touch companions."""
        mock_mgr = MagicMock()
        runtime_mod._navpy_mgr = mock_mgr
        runtime_mod._auto_managed_sysids.add(3)

        result = asyncio.run(vehicles_mod.disconnect_vehicle(99))

        mock_mgr.stop.assert_not_called()
        self.assertIn(3, runtime_mod._auto_managed_sysids)
        mock_broadcast.assert_not_awaited()
        self.assertEqual(
            result, {"status": "disconnected", "sys_id": 99, "removed": []})

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(vehicles_mod.vehicle_mgr, "remove_vehicle", return_value=[3])
    def test_intent_cleared_when_no_tracked_instance(
            self, mock_remove, mock_broadcast):
        """Stale intent with no tracked instance record must still be cleared.

        NavpyProcessManager.stop() returns False only when NO entry is tracked
        for the sysid (a tracked-but-already-exited process still returns
        True, and then the broadcast fires like the normal stop case).
        """
        mock_mgr = MagicMock()
        mock_mgr.stop.return_value = False
        runtime_mod._navpy_mgr = mock_mgr
        runtime_mod._auto_managed_sysids.add(3)

        asyncio.run(vehicles_mod.disconnect_vehicle(3))

        mock_mgr.stop.assert_called_once_with(3)
        self.assertNotIn(3, runtime_mod._auto_managed_sysids)
        # No instance record was removed, so no status broadcast is needed.
        mock_broadcast.assert_not_awaited()

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(vehicles_mod.vehicle_mgr, "remove_vehicle", return_value=[3])
    def test_disconnect_safe_without_manager(self, mock_remove, mock_broadcast):
        """No manager (e.g. non-sim mode): disconnect still clears intent."""
        runtime_mod._auto_managed_sysids.add(3)

        result = asyncio.run(vehicles_mod.disconnect_vehicle(3))

        self.assertIsNone(runtime_mod._navpy_mgr)
        self.assertNotIn(3, runtime_mod._auto_managed_sysids)
        mock_broadcast.assert_not_awaited()
        self.assertEqual(result["removed"], [3])


class TestDisconnectWatchdogRace(unittest.TestCase):
    """The watchdog must never race an in-flight disconnect into starting an
    unmanaged orphan: intent is cleared on the event loop BEFORE the process
    kill is offloaded to the executor."""

    def setUp(self):
        _reset_runtime_state()

    def tearDown(self):
        _reset_runtime_state()

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(vehicles_mod.vehicle_mgr, "remove_vehicle", return_value=[3])
    def test_intent_already_cleared_when_process_stop_runs(
            self, mock_remove, mock_broadcast):
        intent_at_stop_time = []
        mock_mgr = MagicMock()

        def record_stop(rid):
            intent_at_stop_time.append(rid in runtime_mod._auto_managed_sysids)
            return True

        mock_mgr.stop.side_effect = record_stop
        runtime_mod._navpy_mgr = mock_mgr
        runtime_mod._auto_managed_sysids.add(3)

        asyncio.run(vehicles_mod.disconnect_vehicle(3))

        # The ordering invariant: by the time the (executor-side) kill runs,
        # the loop-side intent discard has already happened.
        self.assertEqual(intent_at_stop_time, [False])

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(vehicles_mod.vehicle_mgr, "remove_vehicle", return_value=[3])
    def test_watchdog_firing_mid_kill_cannot_start_orphan(
            self, mock_remove, mock_broadcast):
        settings = MagicMock()
        settings.simulation.sim_mode = True
        mock_mgr = MagicMock()
        mock_mgr.get_status.return_value = None
        restarted_mid_kill = []

        def stop_and_fire_watchdog(rid):
            # Simulate the watchdog running with a stale active set while the
            # kill is still in flight: sysid 3 already left the managed set,
            # so wanted must be empty and no start may happen.
            restarted_mid_kill.append(
                runtime_mod.ensure_auto_navpy_sim_running([3], settings))
            return True

        mock_mgr.stop.side_effect = stop_and_fire_watchdog
        runtime_mod._navpy_mgr = mock_mgr
        runtime_mod._auto_managed_sysids.add(3)

        asyncio.run(vehicles_mod.disconnect_vehicle(3))

        self.assertEqual(restarted_mid_kill, [[]])
        mock_mgr.start.assert_not_called()


class TestAutoStartConnectedGate(unittest.TestCase):
    """Route-level auto_start_navpy_sim is gated on the vehicle being connected."""

    def setUp(self):
        _reset_runtime_state()

    def tearDown(self):
        _reset_runtime_state()

    @patch.object(navpy_mod.runtime, "auto_start_navpy_sim", return_value=True)
    @patch.object(vehicle_mgr, "get_vehicle", return_value=None)
    def test_skips_and_reports_false_when_not_connected(
            self, mock_get, mock_auto):
        self.assertFalse(navpy_mod.auto_start_navpy_sim(7))
        mock_auto.assert_not_called()

    @patch.object(navpy_mod.runtime, "auto_start_navpy_sim", return_value=True)
    @patch.object(vehicle_mgr, "get_vehicle", return_value=MagicMock())
    def test_delegates_when_connected(self, mock_get, mock_auto):
        self.assertTrue(navpy_mod.auto_start_navpy_sim(7))
        mock_auto.assert_called_once_with(7)


class TestStaleDiscoveryAfterDisconnect(unittest.TestCase):
    """Barrier regression: a discovery that completes AFTER a disconnect must
    not restart the companion or re-adopt managed intent (issue #163 race)."""

    def setUp(self):
        _reset_runtime_state()

    def tearDown(self):
        _reset_runtime_state()

    def test_stale_discovery_completion_cannot_resurrect_companion(self):
        entry3 = MagicMock()
        entry3.sys_id = 3
        entry3.name = "UAV3"
        connected = {}
        started = threading.Event()
        release = threading.Event()

        def fake_discover(device, timeout):
            connected[3] = entry3      # vehicle registers mid-discovery
            started.set()
            release.wait(5)            # hold completion until disconnect ran
            return [entry3]

        def fake_remove(sys_id):
            return [sys_id] if connected.pop(sys_id, None) else []

        async def scenario():
            task = asyncio.ensure_future(
                vehicles_mod.discover_vehicles("udp:0.0.0.0:14550", 0.1))
            for _ in range(500):       # wait until discovery is in-flight
                if started.is_set():
                    break
                await asyncio.sleep(0.01)
            self.assertTrue(started.is_set())
            await vehicles_mod.disconnect_vehicle(3)
            release.set()              # stale discovery completes only now
            return await task

        with patch.object(vehicles_mod.vehicle_mgr, "discover_and_connect",
                          side_effect=fake_discover), \
             patch.object(vehicles_mod.vehicle_mgr, "remove_vehicle",
                          side_effect=fake_remove), \
             patch.object(vehicle_mgr, "get_vehicle",
                          side_effect=lambda sid: connected.get(sid)), \
             patch.object(navpy_mod.runtime, "auto_start_navpy_sim") as mock_auto, \
             patch.object(navpy_mod, "broadcast_sim_status",
                          new_callable=AsyncMock), \
             patch("gcs.backend.routes.control.auto_start_esp32_sim"):
            result = asyncio.run(scenario())

        self.assertEqual(result["found"], 1)
        mock_auto.assert_not_called()
        self.assertNotIn(3, runtime_mod._auto_managed_sysids)


class TestWatchdogAfterDisconnect(unittest.TestCase):
    """Regression for issue #163: stale intent must not resume restarts."""

    def setUp(self):
        _reset_runtime_state()

    def tearDown(self):
        _reset_runtime_state()

    def _mock_settings(self):
        settings = MagicMock()
        settings.simulation.sim_mode = True
        return settings

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(vehicles_mod.vehicle_mgr, "remove_vehicle", return_value=[3])
    def test_no_restart_after_disconnect_even_if_sysid_active_again(
            self, mock_remove, mock_broadcast):
        mock_mgr = MagicMock()
        mock_mgr.stop.return_value = True
        runtime_mod._navpy_mgr = mock_mgr
        runtime_mod._auto_managed_sysids.add(3)

        asyncio.run(vehicles_mod.disconnect_vehicle(3))

        # Even if the sysid shows up in active_ids again (re-discovery race),
        # the watchdog must not restart the companion: intent is gone.
        mock_mgr.get_status.return_value = None
        restarted = runtime_mod.ensure_auto_navpy_sim_running(
            [3], self._mock_settings())

        self.assertEqual(restarted, [])
        mock_mgr.start.assert_not_called()
