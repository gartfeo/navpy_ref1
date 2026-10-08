import asyncio
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock, patch

from gcs.backend.telemetry_loop import TelemetryLoop, _check_disarm_after_guided, _has_changed


class TestCheckDisarmAfterGuided(unittest.TestCase):
    """Tests for _check_disarm_after_guided: detect armed->disarmed in GUIDED mode."""

    def test_armed_guided_to_disarmed_returns_true(self):
        prev = {"armed": True, "mode": "GUIDED"}
        curr = {"armed": False, "mode": "GUIDED"}
        self.assertTrue(_check_disarm_after_guided(prev, curr))

    def test_disarm_with_mode_change_keeps_previous_guided_semantics(self):
        prev = {"armed": True, "mode": "GUIDED"}
        curr = {"armed": False, "mode": "AUTO"}
        self.assertTrue(_check_disarm_after_guided(prev, curr))

    def test_armed_auto_to_disarmed_returns_false(self):
        prev = {"armed": True, "mode": "AUTO"}
        curr = {"armed": False, "mode": "AUTO"}
        self.assertFalse(_check_disarm_after_guided(prev, curr))

    def test_armed_stays_true_returns_false(self):
        prev = {"armed": True, "mode": "GUIDED"}
        curr = {"armed": True, "mode": "GUIDED"}
        self.assertFalse(_check_disarm_after_guided(prev, curr))

    def test_armed_already_false_returns_false(self):
        prev = {"armed": False, "mode": "GUIDED"}
        curr = {"armed": False, "mode": "GUIDED"}
        self.assertFalse(_check_disarm_after_guided(prev, curr))

    def test_prev_missing_armed_returns_false(self):
        prev = {"mode": "GUIDED"}
        curr = {"armed": False, "mode": "GUIDED"}
        self.assertFalse(_check_disarm_after_guided(prev, curr))


class TestHasChanged(unittest.TestCase):
    """Tests for _has_changed: detect meaningful telemetry changes."""

    def test_armed_changes_returns_true(self):
        prev = {"armed": True}
        curr = {"armed": False}
        self.assertTrue(_has_changed(prev, curr))

    def test_armed_same_returns_false(self):
        prev = {"armed": True}
        curr = {"armed": True}
        self.assertFalse(_has_changed(prev, curr))

    def test_mode_changes_returns_true(self):
        prev = {"mode": "AUTO"}
        curr = {"mode": "GUIDED"}
        self.assertTrue(_has_changed(prev, curr))

    def test_swarm_state_changes_are_broadcast(self):
        free = {"state": "FREE", "boot": 7, "seq": 3, "stale": False}
        for prev, curr in (
            (None, free),
            (free, {**free, "state": "BUSY", "seq": 4}),
            (free, {**free, "seq": 4}),  # orders the beat after step-4 copies
            (free, {**free, "stale": True}),
        ):
            with self.subTest(prev=prev, curr=curr):
                self.assertTrue(_has_changed({"swarm": prev}, {"swarm": curr}))
        self.assertFalse(_has_changed({"swarm": free}, {"swarm": dict(free)}))


class TestTelemetryLoopNavpyRestart(unittest.TestCase):
    """Tests for the sim-status tick's NavPy auto-restart hook."""

    def test_sim_status_tick_restarts_auto_managed_navpy_for_active_ids(self):
        async def run_once():
            mgr = MagicMock()
            mgr.get_all_snapshots.return_value = [{"sys_id": 2}]
            mgr.get_seen_ids.return_value = []
            loop = TelemetryLoop(mgr)
            settings = SimpleNamespace(simulation=SimpleNamespace(sim_mode=True),
                                       connection=SimpleNamespace(ws_broadcast_interval_s=0.0))

            async def stop_after_sleep(_interval):
                loop._stop.set()

            with patch("gcs.backend.telemetry_loop.settings_store.get", return_value=settings), \
                 patch("gcs.backend.navpy_sim_runtime.ensure_auto_navpy_sim_running") as ensure, \
                 patch("gcs.backend.routes.navpy_sim.broadcast_sim_status",
                       new_callable=AsyncMock) as broadcast_status, \
                 patch("gcs.backend.telemetry_loop.ws_manager.broadcast",
                       new_callable=AsyncMock), \
                 patch("gcs.backend.telemetry_loop.asyncio.sleep", new=stop_after_sleep):
                await loop._loop()

            ensure.assert_called_once_with({2}, settings)
            broadcast_status.assert_awaited_once()

        asyncio.run(run_once())

    def test_auto_heal_does_not_block_the_event_loop(self):
        async def run_once():
            mgr = MagicMock()
            mgr.get_all_snapshots.return_value = [{"sys_id": 2}]
            mgr.get_seen_ids.return_value = []
            telemetry = TelemetryLoop(mgr)
            settings = SimpleNamespace(simulation=SimpleNamespace(sim_mode=True),
                                       connection=SimpleNamespace(ws_broadcast_interval_s=0.0))
            entered = threading.Event()

            def blocked_auto_heal(_active_ids, _settings):
                entered.set()
                time.sleep(0.25)

            event_loop = asyncio.get_running_loop()
            probe = event_loop.create_future()
            started = time.perf_counter()
            event_loop.call_later(
                0.02,
                lambda: probe.set_result(time.perf_counter()),
            )

            with patch(
                "gcs.backend.telemetry_loop.settings_store.get",
                return_value=settings,
            ), patch(
                "gcs.backend.navpy_sim_runtime.ensure_auto_navpy_sim_running",
                side_effect=blocked_auto_heal,
            ), patch(
                "gcs.backend.routes.navpy_sim.broadcast_sim_status",
                new_callable=AsyncMock,
            ), patch(
                "gcs.backend.telemetry_loop.ws_manager.broadcast",
                new_callable=AsyncMock,
            ):
                task = asyncio.create_task(telemetry._loop())
                fired_at = await asyncio.wait_for(probe, 0.6)
                telemetry._stop.set()
                await asyncio.wait_for(task, 0.6)

            self.assertTrue(entered.is_set())
            self.assertLess(fired_at - started, 0.1)

        asyncio.run(run_once())


if __name__ == "__main__":
    unittest.main()
