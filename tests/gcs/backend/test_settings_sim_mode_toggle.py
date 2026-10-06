"""Tests for the sim_mode True->False lifecycle hook in the settings route.

Regression for issue #162: disabling ``simulation.sim_mode`` via the settings API
left GCS-managed NavPy companions running and left their sysids in
``_auto_managed_sysids``. With sim_mode off the ``/navpy-sim/*`` stop routes 400
and the watchdog goes inert, so the operator could neither stop the orphans nor
rely on self-heal, and re-enabling sim_mode silently resumed managed restarts
from the stale intent set.

Disabling sim_mode now stops all companions (via ``stop_all_instances`` — same
effect as the operator Stop-All: terminate every companion and clear intent,
keeping the manager) and pushes one status broadcast so an already-open GCS
drops its stale "running" rows and flips the button back to Start.
"""
import asyncio
import threading
import time
import unittest
from unittest.mock import AsyncMock, MagicMock, patch

from pydantic import BaseModel, ValidationError

from gcs.backend import navpy_sim_runtime as runtime_mod
from gcs.backend.routes import navpy_sim as navpy_mod
from gcs.backend.routes import settings as settings_mod
from gcs.backend.routes.settings import (
    _stop_navpy_if_sim_mode_disabled,
    update_settings,
)
from gcs.backend.settings_model import GcsSettings


def _reset_runtime_state():
    runtime_mod._navpy_mgr = None
    runtime_mod._auto_managed_sysids.clear()


def _settings(sim_mode: bool, vision_profile="prof") -> GcsSettings:
    """A settings snapshot built from the REAL model, not a MagicMock.

    ``sim_mode`` and ``camera.vision_profile`` are the only fields the exercised
    production paths read, and both are set explicitly; everything else keeps its
    production default. A bare MagicMock cannot be used here: the route reads
    ``getattr(getattr(before, "simulation", None), "sim_mode", False)``
    (routes/settings.py), and a mock auto-creates that attribute as a truthy child,
    so the ``False`` default could never fire. The real model does not fabricate
    attributes, so production's own fallback behaves here as it does in production.
    Note what that does NOT buy on this path: a renamed field does not raise, it
    quietly takes the ``getattr`` default — which is precisely why a fabricated
    truthy child was undetectable before.

    The default vision profile is a real ``str`` so the profile-change hook no-ops
    (equal before/after) and doesn't interfere with these tests.
    """
    settings = GcsSettings()
    settings.simulation.sim_mode = sim_mode
    settings.camera.vision_profile = vision_profile
    return settings


class TestSimModeDisabledStopsCompanions(unittest.TestCase):
    def setUp(self):
        _reset_runtime_state()

    def tearDown(self):
        _reset_runtime_state()

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    def test_blocked_disable_cleanup_does_not_stall_event_loop(self, _broadcast):
        release = threading.Event()

        def blocked_stop():
            if not release.wait(1.0):
                raise TimeoutError("test did not release blocked stop")

        async def scenario():
            started_s = time.perf_counter()
            route_task = asyncio.create_task(_stop_navpy_if_sim_mode_disabled(
                _settings(True),
                _settings(False),
            ))
            probe_elapsed: list[float] = []

            async def probe():
                await asyncio.sleep(0)
                probe_elapsed.append(time.perf_counter() - started_s)

            probe_task = asyncio.create_task(probe())
            timer = threading.Timer(0.2, release.set)
            timer.start()
            try:
                await asyncio.gather(route_task, probe_task)
            finally:
                release.set()
                timer.cancel()
            return probe_elapsed[0]

        with patch.object(
            settings_mod.runtime,
            "stop_all_instances",
            side_effect=blocked_stop,
        ):
            probe_elapsed_s = asyncio.run(scenario())

        self.assertLess(probe_elapsed_s, 0.05)

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    def test_true_to_false_stops_all_clears_intent_keeps_manager_broadcasts(self, broadcast):
        mgr = MagicMock()
        runtime_mod._navpy_mgr = mgr
        runtime_mod._auto_managed_sysids.update({1, 2, 3})

        asyncio.run(_stop_navpy_if_sim_mode_disabled(_settings(True), _settings(False)))

        mgr.stop_all.assert_called_once()
        self.assertEqual(runtime_mod._auto_managed_sysids, set())
        # Manager is kept (same as the operator Stop-All path), not dropped.
        self.assertIs(runtime_mod._navpy_mgr, mgr)
        # Open clients are told the new (empty) status so they flip to Start.
        broadcast.assert_awaited_once()

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    def test_no_stop_when_sim_mode_stays_on(self, broadcast):
        mgr = MagicMock()
        runtime_mod._navpy_mgr = mgr
        runtime_mod._auto_managed_sysids.update({1, 2})

        asyncio.run(_stop_navpy_if_sim_mode_disabled(_settings(True), _settings(True)))

        mgr.stop_all.assert_not_called()
        self.assertEqual(runtime_mod._auto_managed_sysids, {1, 2})
        broadcast.assert_not_awaited()

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    def test_no_stop_when_sim_mode_stays_off(self, broadcast):
        mgr = MagicMock()
        runtime_mod._navpy_mgr = mgr
        runtime_mod._auto_managed_sysids.add(1)

        asyncio.run(_stop_navpy_if_sim_mode_disabled(_settings(False), _settings(False)))

        mgr.stop_all.assert_not_called()
        self.assertEqual(runtime_mod._auto_managed_sysids, {1})
        broadcast.assert_not_awaited()

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    def test_no_stop_when_sim_mode_enabled(self, broadcast):
        # False->True must not tear anything down.
        mgr = MagicMock()
        runtime_mod._navpy_mgr = mgr

        asyncio.run(_stop_navpy_if_sim_mode_disabled(_settings(False), _settings(True)))

        mgr.stop_all.assert_not_called()
        self.assertIs(runtime_mod._navpy_mgr, mgr)
        broadcast.assert_not_awaited()

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    def test_stop_failure_does_not_propagate_and_still_broadcasts(self, broadcast):
        # The route already persisted settings before calling the hook; a stop
        # failure must not turn a successful PUT into a 500, and clients should
        # still be told the true (possibly partially-stopped) status.
        mgr = MagicMock()
        mgr.stop_all.side_effect = RuntimeError("terminate blew up")
        runtime_mod._navpy_mgr = mgr
        runtime_mod._auto_managed_sysids.add(1)

        # Must not raise.
        asyncio.run(_stop_navpy_if_sim_mode_disabled(_settings(True), _settings(False)))

        # Intent is cleared before the manager stop, so it is gone regardless.
        self.assertEqual(runtime_mod._auto_managed_sysids, set())
        broadcast.assert_awaited_once()

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    def test_broadcast_failure_does_not_propagate(self, broadcast):
        broadcast.side_effect = RuntimeError("ws blew up")
        mgr = MagicMock()
        runtime_mod._navpy_mgr = mgr
        runtime_mod._auto_managed_sysids.add(1)

        # Must not raise even if the status broadcast fails.
        asyncio.run(_stop_navpy_if_sim_mode_disabled(_settings(True), _settings(False)))

        mgr.stop_all.assert_called_once()

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    def test_noop_without_manager(self, broadcast):
        # No companions ever started: intent empty, no manager. Still a clean no-op stop.
        asyncio.run(_stop_navpy_if_sim_mode_disabled(_settings(True), _settings(False)))

        self.assertIsNone(runtime_mod._navpy_mgr)
        self.assertEqual(runtime_mod._auto_managed_sysids, set())
        # The broadcast is unconditional on a true->false transition, manager or
        # not: a client that never saw a companion still needs the empty status.
        broadcast.assert_awaited_once()


class TestSimModeToggleClearsStaleIntent(unittest.TestCase):
    """End-to-end intent semantics from the issue: re-enabling sim_mode must not
    resurrect companions from intent that predates the disable."""

    def setUp(self):
        _reset_runtime_state()

    def tearDown(self):
        _reset_runtime_state()

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    def test_reenabling_sim_mode_does_not_restart_from_stale_intent(self, _broadcast):
        # A companion was auto-managed while sim_mode was on.
        mgr = MagicMock()
        runtime_mod._navpy_mgr = mgr
        runtime_mod._auto_managed_sysids.add(1)

        # Operator disables sim_mode: the hook clears intent (and stops all).
        asyncio.run(_stop_navpy_if_sim_mode_disabled(_settings(True), _settings(False)))
        self.assertEqual(runtime_mod._auto_managed_sysids, set())

        # Operator re-enables sim_mode; the watchdog runs against the same active
        # vehicle. With intent cleared, nothing is restarted from the stale set.
        restarted = runtime_mod.ensure_auto_navpy_sim_running([1], _settings(True))
        self.assertEqual(restarted, [])
        mgr.start.assert_not_called()


class TestUpdateSettingsRouteDrivesHook(unittest.TestCase):
    """Route-contract coverage: PUT /api/settings actually invokes the hook on a
    disable, skips it otherwise, and never runs it when validation fails."""

    def setUp(self):
        _reset_runtime_state()

    def tearDown(self):
        _reset_runtime_state()

    @patch("gcs.backend.instance_ports.chat_index", return_value=None)
    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(settings_mod.runtime, "stop_all_instances")
    @patch.object(settings_mod.runtime, "restart_running_navpy_sim", return_value=[])
    @patch.object(settings_mod.settings_store, "update")
    @patch.object(settings_mod.settings_store, "get")
    def test_put_disable_sim_mode_stops_and_broadcasts(
            self, mock_get, mock_update, _restart, stop_all, broadcast, _chat):
        mock_get.return_value = _settings(True)
        mock_update.return_value = _settings(False)

        result = asyncio.run(update_settings({"simulation": {"sim_mode": False}}))

        stop_all.assert_called_once()
        broadcast.assert_awaited_once()
        # The route answers with the PERSISTED settings, so the caller sees the
        # disable it just made. Only a real model can be asserted here: a mock's
        # model_dump() returns another mock and any dict claim about it is empty.
        # chat_index is None, so _with_instance_device is a pass-through and this
        # stays a settings-route test rather than a per-chat ports test.
        self.assertIsInstance(result, dict)
        self.assertIs(result["simulation"]["sim_mode"], False)

    @patch("gcs.backend.instance_ports.chat_index", return_value=None)
    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(settings_mod.runtime, "stop_all_instances")
    @patch.object(settings_mod.runtime, "restart_running_navpy_sim", return_value=[])
    @patch.object(settings_mod.settings_store, "update")
    @patch.object(settings_mod.settings_store, "get")
    def test_put_partial_body_without_simulation_does_not_stop(
            self, mock_get, mock_update, _restart, stop_all, broadcast, _chat):
        # A partial PUT that omits "simulation" leaves the persisted sim_mode on;
        # no transition, so nothing is stopped.
        mock_get.return_value = _settings(True)
        mock_update.return_value = _settings(True)

        asyncio.run(update_settings({"connection": {"auto_connect": True}}))

        stop_all.assert_not_called()
        broadcast.assert_not_awaited()

    @patch("gcs.backend.instance_ports.chat_index", return_value=None)
    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(settings_mod.runtime, "stop_all_instances")
    @patch.object(settings_mod.runtime, "restart_running_navpy_sim", return_value=[])
    @patch.object(settings_mod.settings_store, "update")
    @patch.object(settings_mod.settings_store, "get")
    def test_put_disable_with_profile_change_does_not_restart(
            self, mock_get, mock_update, restart, stop_all, broadcast, _chat):
        # Disabling sim_mode AND changing the vision profile in one PUT: the
        # disable stops everything and the profile-restart hook must not fire a
        # restart (it is gated on the persisted sim_mode, now off).
        mock_get.return_value = _settings(True, vision_profile="prof_a")
        mock_update.return_value = _settings(False, vision_profile="prof_b")

        asyncio.run(update_settings({"simulation": {"sim_mode": False},
                                     "camera": {"vision_profile": "prof_b"}}))

        stop_all.assert_called_once()
        broadcast.assert_awaited_once()
        # The profile hook is gated on the persisted (now-disabled) sim_mode, so
        # even with a changed profile it must NOT fire a runtime restart.
        restart.assert_not_called()

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(settings_mod.runtime, "stop_all_instances")
    @patch.object(settings_mod.settings_store, "update")
    @patch.object(settings_mod.settings_store, "get")
    def test_put_validation_error_skips_hook(
            self, mock_get, mock_update, stop_all, broadcast):
        from fastapi import HTTPException

        mock_get.return_value = _settings(True)

        class _M(BaseModel):
            x: int

        try:
            _M(x="not-an-int")
            err = None
        except ValidationError as e:
            err = e
        mock_update.side_effect = err

        with self.assertRaises(HTTPException) as ctx:
            asyncio.run(update_settings({"simulation": {"sim_mode": False}}))

        self.assertEqual(ctx.exception.status_code, 422)
        # Settings never changed → the disable hook must never run.
        stop_all.assert_not_called()
        broadcast.assert_not_awaited()


class TestDisableBroadcastEmitsStoppedForPriorCompanions(unittest.TestCase):
    """Integration: after stop_all_instances empties the (kept) manager, the REAL
    broadcast_sim_status emits one 'stopped' per previously-running sysid and
    pushes instances: [] — the mechanism that flips open clients back to Start."""

    def setUp(self):
        _reset_runtime_state()
        self._saved_dedup = dict(navpy_mod._last_diagnostic_status)
        navpy_mod._last_diagnostic_status.clear()

    def tearDown(self):
        navpy_mod._last_diagnostic_status.clear()
        navpy_mod._last_diagnostic_status.update(self._saved_dedup)
        _reset_runtime_state()

    def test_broadcast_after_empty_manager_emits_stopped_and_empty_instances(self):
        mgr = MagicMock()
        mgr.get_all_status.return_value = []   # companions already terminated
        runtime_mod._navpy_mgr = mgr
        # Clients last saw sysids 1, 2, 3 running.
        navpy_mod._last_diagnostic_status.update(
            {1: (True, None), 2: (True, None), 3: (True, None)})

        with patch.object(navpy_mod, "emit") as emit_mock, \
             patch.object(navpy_mod.ws_manager, "broadcast",
                          new_callable=AsyncMock) as ws_broadcast:
            asyncio.run(navpy_mod.broadcast_sim_status())

        emitted = [(c.kwargs["sys_id"], c.kwargs["running"], c.kwargs["outcome"])
                   for c in emit_mock.call_args_list]
        self.assertEqual(
            sorted(emitted),
            [(1, False, "stopped"), (2, False, "stopped"), (3, False, "stopped")],
        )
        ws_broadcast.assert_awaited_once()
        payload = ws_broadcast.await_args.args[0]
        self.assertEqual(payload["type"], "navpy_sim_status")
        self.assertEqual(payload["instances"], [])
        # Dedup cleared so a later restart of the same sysid is recorded fresh.
        self.assertEqual(navpy_mod._last_diagnostic_status, {})


if __name__ == "__main__":
    unittest.main()
