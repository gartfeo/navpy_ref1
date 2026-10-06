"""Tests for NavPy simulation route helpers."""
import asyncio
import os
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from pydantic import ValidationError

from gcs.backend import navpy_sim_runtime as runtime_mod
from gcs.backend.routes import navpy_sim as navpy_mod
from gcs.backend.settings_model import GcsSettings


def _settings(*, sim_mode=True, dev_mode=False, detector_debug_window=False,
              sitl_presets=None, vision_profile=None) -> GcsSettings:
    """Settings for a test: the REAL model, never a MagicMock.

    A bare ``MagicMock`` auto-creates whatever attribute production asks for, and
    a child mock is truthy — so ``getattr(settings.simulation, "flag", False)``
    silently returns True and the default never fires. Not theoretical: it is why
    three tests in this file failed for as long as anyone could remember (read as
    "flaky"), while two others PASSED for the same reason, asserting a ``dev_mode``
    gate that production had already replaced with an explicit opt-in.

    A real ``GcsSettings`` gives unset fields their real defaults and raises on a
    field renamed away instead of fabricating it. Only what a test actually varies
    is passed; everything else stays the production default, so a field added to
    the model later is exercised at its default rather than at "truthy".

    ``sim_mode``, ``dev_mode``, ``detector_debug_window`` and ``vision_profile``
    are always set explicitly, because the model's own defaults are not what most
    tests here want (``dev_mode`` defaults True) and because an unset field
    deciding behaviour is the exact problem being removed. ``sitl_presets``
    deliberately inherits its production default unless a test is exercising
    connection selection.

    Real-shaped and real-defaulted, but not assignment-validated: the model does
    not enable ``validate_assignment``, so a test can still assign an odd value.
    What it can no longer do is invent a field that production renamed away.
    """
    settings = GcsSettings()
    settings.simulation.sim_mode = sim_mode
    settings.simulation.dev_mode = dev_mode
    settings.simulation.detector_debug_window = detector_debug_window
    if sitl_presets is not None:
        settings.simulation.sitl_presets = sitl_presets
    settings.camera.vision_profile = vision_profile
    return settings


@pytest.fixture(autouse=True)
def _isolate_navpy_sim_log_level_env():
    """Keep every test independent of an ambient NAVPY_SIM_LOG_LEVEL.

    start_instance() reads this env var, so a value present in the developer/CI
    environment would add an unexpected ``log_level=`` kwarg and break the
    exact-call assertions below. Tests that need it set do so via patch.dict,
    which layers on top of this cleared baseline.
    """
    saved = os.environ.pop(runtime_mod._ENV_LOG_LEVEL, None)
    try:
        yield
    finally:
        if saved is not None:
            os.environ[runtime_mod._ENV_LOG_LEVEL] = saved


def _reset_runtime_state():
    runtime_mod._navpy_mgr = None
    runtime_mod._auto_managed_sysids.clear()
    runtime_mod._hb_exit_streaks.clear()


class TestNavpySimRouteHelpers(unittest.TestCase):
    """Test the shared NavPy sim runtime manager functions."""

    def setUp(self):
        _reset_runtime_state()

    def tearDown(self):
        _reset_runtime_state()

    def test_stop_all_when_no_manager_is_noop(self):
        runtime_mod.stop_all_navpy_if_running()
        self.assertIsNone(runtime_mod._navpy_mgr)

    def test_stop_all_when_manager_exists_calls_stop_all(self):
        mock_mgr = MagicMock()
        runtime_mod._navpy_mgr = mock_mgr
        runtime_mod.stop_all_navpy_if_running()
        mock_mgr.stop_all.assert_called_once()
        self.assertIsNone(runtime_mod._navpy_mgr)

    def test_lazy_manager_creation(self):
        self.assertIsNone(runtime_mod._navpy_mgr)
        mgr = runtime_mod.get_or_create_mgr()
        self.assertIsNotNone(mgr)
        self.assertIs(runtime_mod._navpy_mgr, mgr)
        # Calling again returns the same instance
        mgr2 = runtime_mod.get_or_create_mgr()
        self.assertIs(mgr, mgr2)


class TestGetSimConnection(unittest.TestCase):
    """Test _get_sim_connection derives correct connection strings."""

    def _mock_settings(self, presets, dev_mode=False, vision_profile=None):
        return _settings(sitl_presets=presets, dev_mode=dev_mode,
                         vision_profile=vision_profile)

    @patch.object(runtime_mod.settings_store, "get")
    def test_uses_preset_by_index(self, mock_get):
        mock_get.return_value = self._mock_settings(
            ["udp:0.0.0.0:14560", "udp:0.0.0.0:14570", "udp:0.0.0.0:14580"],
        )
        self.assertEqual(navpy_mod._get_sim_connection(1), "udp:0.0.0.0:14560")
        self.assertEqual(navpy_mod._get_sim_connection(2), "udp:0.0.0.0:14570")
        self.assertEqual(navpy_mod._get_sim_connection(3), "udp:0.0.0.0:14580")

    @patch.object(runtime_mod.settings_store, "get")
    def test_auto_increments_port_beyond_presets(self, mock_get):
        mock_get.return_value = self._mock_settings(
            ["udp:0.0.0.0:14560", "udp:0.0.0.0:14570"],
        )
        # sys_id 3 → idx 2, beyond 2 presets → last port + 10
        self.assertEqual(navpy_mod._get_sim_connection(3), "udp:0.0.0.0:14580")
        self.assertEqual(navpy_mod._get_sim_connection(4), "udp:0.0.0.0:14590")


@pytest.mark.parametrize("value", (True, 0, -1, 256, 1.0, "1"))
def test_navpy_request_rejects_non_strict_or_out_of_range_sysid(value):
    with pytest.raises(ValidationError):
        navpy_mod.NavpyStartRequest(sys_id=value)


@pytest.mark.parametrize("value", (True, -1, 65536, 2.0, "2"))
def test_restart_request_rejects_invalid_expected_mission_index(value):
    with pytest.raises(ValidationError):
        navpy_mod.NavpyExpectedParams(targ_wps=value, nav_last_wp=2)


class TestNavpySimStartRoutes(unittest.TestCase):
    """Test manual NavPy sim start routes."""

    def setUp(self):
        _reset_runtime_state()

    def tearDown(self):
        _reset_runtime_state()

    def _mock_settings(self, dev_mode=False, vision_profile=None):
        return _settings(dev_mode=dev_mode, vision_profile=vision_profile)

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(navpy_mod.runtime, "sim_connection_from_settings",
                  return_value="tcp:127.0.0.1:5760")
    @patch.object(navpy_mod.settings_store, "get")
    def test_blocked_start_does_not_stall_event_loop(
            self, mock_get, _mock_conn, _mock_broadcast):
        mock_get.return_value = self._mock_settings()
        release = threading.Event()
        instance = SimpleNamespace(process=SimpleNamespace(pid=101))

        def blocked_start(*_args):
            if not release.wait(1.0):
                raise TimeoutError("test did not release blocked start")
            return instance

        async def scenario():
            started_s = time.perf_counter()
            route_task = asyncio.create_task(navpy_mod.start_navpy_instance(
                navpy_mod.NavpyStartRequest(sys_id=1)
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
            navpy_mod.runtime,
            "managed_start_instance",
            side_effect=blocked_start,
        ):
            probe_elapsed_s = asyncio.run(scenario())

        self.assertLess(probe_elapsed_s, 0.05)

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(navpy_mod.runtime, "start_instance")
    @patch.object(navpy_mod.runtime, "sim_connection_from_settings",
                  return_value="tcp:127.0.0.1:5760")
    @patch.object(navpy_mod.settings_store, "get")
    def test_start_route_derives_connection_from_chat(
            self, mock_get, mock_conn, mock_start, _mock_broadcast):
        settings = self._mock_settings(dev_mode=False)
        mock_get.return_value = settings
        inst = MagicMock()
        inst.process.pid = 101
        mock_start.return_value = inst

        # Client sends no connection — the route derives it authoritatively.
        req = navpy_mod.NavpyStartRequest(sys_id=1)
        result = asyncio.run(navpy_mod.start_navpy_instance(req))

        self.assertEqual(result["status"], "started")
        mock_conn.assert_called_once_with(1, settings)
        mock_start.assert_called_once_with(1, "tcp:127.0.0.1:5760", settings)

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(navpy_mod.runtime, "start_instance")
    @patch.object(navpy_mod.runtime, "sim_connection_from_settings",
                  side_effect=lambda sys_id, _s: f"tcp:127.0.0.1:{5760 + 10 * (sys_id - 1)}")
    @patch.object(navpy_mod.settings_store, "get")
    def test_start_all_route_derives_connections(
            self, mock_get, _mock_conn, mock_start, _mock_broadcast):
        settings = self._mock_settings(dev_mode=True, vision_profile="siyi_zr10")
        mock_get.return_value = settings
        inst1 = MagicMock()
        inst1.process.pid = 101
        inst2 = MagicMock()
        inst2.process.pid = 102
        mock_start.side_effect = [inst1, inst2]

        # Connections ignored on the request; derived per sys_id server-side.
        req = navpy_mod.NavpyStartAllRequest(instances=[
            navpy_mod.NavpyStartRequest(sys_id=1),
            navpy_mod.NavpyStartRequest(sys_id=2),
        ])
        result = asyncio.run(navpy_mod.start_all_navpy(req))

        self.assertEqual(result["status"], "started")
        mock_start.assert_any_call(1, "tcp:127.0.0.1:5760", settings)
        mock_start.assert_any_call(2, "tcp:127.0.0.1:5770", settings)


class TestManagedStartInstance(unittest.TestCase):
    """managed_start_instance start-and-mark policy semantics."""

    def setUp(self):
        _reset_runtime_state()

    def tearDown(self):
        _reset_runtime_state()

    def _mock_settings(self):
        return _settings()

    def test_marks_sysid_on_success(self):
        mgr = MagicMock()
        runtime_mod._navpy_mgr = mgr

        runtime_mod.managed_start_instance(1, "udp:0.0.0.0:14560", self._mock_settings())

        mgr.start.assert_called_once()
        self.assertIn(1, runtime_mod._auto_managed_sysids)

    def test_adopts_and_reraises_on_already_running(self):
        mgr = MagicMock()
        mgr.start.side_effect = ValueError("already running")
        runtime_mod._navpy_mgr = mgr

        with self.assertRaises(ValueError):
            runtime_mod.managed_start_instance(1, "udp:0.0.0.0:14560", self._mock_settings())

        self.assertIn(1, runtime_mod._auto_managed_sysids)

    def test_no_mark_on_other_failure(self):
        mgr = MagicMock()
        mgr.start.side_effect = RuntimeError("spawn failed")
        runtime_mod._navpy_mgr = mgr

        with self.assertRaises(RuntimeError):
            runtime_mod.managed_start_instance(1, "udp:0.0.0.0:14560", self._mock_settings())

        self.assertNotIn(1, runtime_mod._auto_managed_sysids)


class TestManualStartMarksAutoManaged(unittest.TestCase):
    """Manual start routes must mark companions auto-managed (self-healing).

    Regression for the live 2026-07-13 finding: a companion started via the UI
    Start NavPy button (or start-all) that later died was never restarted by
    ensure_auto_navpy_sim_running, while auto-started ones self-healed.
    """

    def setUp(self):
        _reset_runtime_state()

    def tearDown(self):
        _reset_runtime_state()

    def _mock_settings(self):
        return _settings()

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(navpy_mod.runtime, "start_instance")
    @patch.object(navpy_mod.runtime, "sim_connection_from_settings",
                  return_value="tcp:127.0.0.1:5760")
    @patch.object(navpy_mod.settings_store, "get")
    def test_start_route_marks_sysid_auto_managed(
            self, mock_get, _mock_conn, mock_start, _mock_broadcast):
        mock_get.return_value = self._mock_settings()
        inst = MagicMock()
        inst.process.pid = 101
        mock_start.return_value = inst

        asyncio.run(navpy_mod.start_navpy_instance(navpy_mod.NavpyStartRequest(sys_id=1)))

        self.assertIn(1, runtime_mod._auto_managed_sysids)

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(navpy_mod.runtime, "start_instance",
                  side_effect=ValueError("already running"))
    @patch.object(navpy_mod.runtime, "sim_connection_from_settings",
                  return_value="tcp:127.0.0.1:5760")
    @patch.object(navpy_mod.settings_store, "get")
    def test_start_route_already_running_adopts_and_keeps_409(
            self, mock_get, _mock_conn, _mock_start, _mock_broadcast):
        mock_get.return_value = self._mock_settings()

        from fastapi import HTTPException
        with self.assertRaises(HTTPException) as ctx:
            asyncio.run(navpy_mod.start_navpy_instance(navpy_mod.NavpyStartRequest(sys_id=1)))

        self.assertEqual(ctx.exception.status_code, 409)
        self.assertIn(1, runtime_mod._auto_managed_sysids)

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(navpy_mod.runtime, "start_instance",
                  side_effect=RuntimeError("spawn failed"))
    @patch.object(navpy_mod.runtime, "sim_connection_from_settings",
                  return_value="tcp:127.0.0.1:5760")
    @patch.object(navpy_mod.settings_store, "get")
    def test_start_route_spawn_failure_leaves_unmanaged(
            self, mock_get, _mock_conn, _mock_start, _mock_broadcast):
        mock_get.return_value = self._mock_settings()

        with self.assertRaises(RuntimeError):
            asyncio.run(navpy_mod.start_navpy_instance(navpy_mod.NavpyStartRequest(sys_id=1)))

        self.assertNotIn(1, runtime_mod._auto_managed_sysids)

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(navpy_mod.runtime, "start_instance")
    @patch.object(navpy_mod.runtime, "sim_connection_from_settings",
                  side_effect=lambda sys_id, _s: f"tcp:127.0.0.1:{5760 + 10 * (sys_id - 1)}")
    @patch.object(navpy_mod.settings_store, "get")
    def test_start_all_marks_each_started_sysid(
            self, mock_get, _mock_conn, mock_start, _mock_broadcast):
        mock_get.return_value = self._mock_settings()
        inst1 = MagicMock()
        inst1.process.pid = 101
        inst2 = MagicMock()
        inst2.process.pid = 102
        mock_start.side_effect = [inst1, inst2]

        req = navpy_mod.NavpyStartAllRequest(instances=[
            navpy_mod.NavpyStartRequest(sys_id=1),
            navpy_mod.NavpyStartRequest(sys_id=2),
        ])
        asyncio.run(navpy_mod.start_all_navpy(req))

        self.assertIn(1, runtime_mod._auto_managed_sysids)
        self.assertIn(2, runtime_mod._auto_managed_sysids)

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(navpy_mod.runtime, "start_instance")
    @patch.object(navpy_mod.runtime, "sim_connection_from_settings",
                  return_value="tcp:127.0.0.1:5760")
    @patch.object(navpy_mod.settings_store, "get")
    def test_start_all_already_running_item_adopted_with_error_result(
            self, mock_get, _mock_conn, mock_start, _mock_broadcast):
        mock_get.return_value = self._mock_settings()
        inst1 = MagicMock()
        inst1.process.pid = 101
        mock_start.side_effect = [inst1, ValueError("already running")]

        req = navpy_mod.NavpyStartAllRequest(instances=[
            navpy_mod.NavpyStartRequest(sys_id=1),
            navpy_mod.NavpyStartRequest(sys_id=2),
        ])
        result = asyncio.run(navpy_mod.start_all_navpy(req))

        # Response contract unchanged: the already-running item still reports error.
        self.assertEqual(result["results"]["1"]["status"], "started")
        self.assertEqual(result["results"]["2"]["status"], "error")
        # But both express keep-alive intent now.
        self.assertIn(1, runtime_mod._auto_managed_sysids)
        self.assertIn(2, runtime_mod._auto_managed_sysids)

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(navpy_mod.runtime, "start_instance")
    @patch.object(navpy_mod.runtime, "sim_connection_from_settings",
                  return_value="tcp:127.0.0.1:5760")
    @patch.object(navpy_mod.settings_store, "get")
    def test_ensure_restarts_exited_manually_started_instance(
            self, mock_get, _mock_conn, mock_start, _mock_broadcast):
        settings = self._mock_settings()
        mock_get.return_value = settings
        inst = MagicMock()
        inst.process.pid = 101
        mock_start.return_value = inst

        asyncio.run(navpy_mod.start_navpy_instance(navpy_mod.NavpyStartRequest(sys_id=1)))

        # Companion dies: manager reports the instance as exited.
        mgr = MagicMock()
        mgr.get_status.return_value = {
            "sys_id": 1,
            "connection": "tcp:127.0.0.1:5760",
            "running": False,
        }
        runtime_mod._navpy_mgr = mgr

        restarted = runtime_mod.ensure_auto_navpy_sim_running([1], settings)

        self.assertEqual(restarted, [1])

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(navpy_mod.runtime, "start_instance")
    @patch.object(navpy_mod.runtime, "sim_connection_from_settings",
                  return_value="tcp:127.0.0.1:5760")
    @patch.object(navpy_mod.settings_store, "get")
    def test_manual_start_then_stop_prevents_restart(
            self, mock_get, _mock_conn, mock_start, _mock_broadcast):
        settings = self._mock_settings()
        mock_get.return_value = settings
        inst = MagicMock()
        inst.process.pid = 101
        mock_start.return_value = inst

        asyncio.run(navpy_mod.start_navpy_instance(navpy_mod.NavpyStartRequest(sys_id=1)))

        mgr = MagicMock()
        mgr.get_status.return_value = {
            "sys_id": 1,
            "connection": "tcp:127.0.0.1:5760",
            "running": False,
        }
        runtime_mod._navpy_mgr = mgr
        runtime_mod.stop_instance(1)
        mock_start.reset_mock()

        restarted = runtime_mod.ensure_auto_navpy_sim_running([1], settings)

        self.assertEqual(restarted, [])
        mock_start.assert_not_called()


class TestStartAllRouteHardening(unittest.TestCase):
    """Batch start-all hardening (issue #161, deferred from the PR #160 review).

    Duplicate sys_ids must be rejected before any process starts, and one
    item's spawn failure must not abort the rest of the batch or skip the
    final broadcast/diagnostic.
    """

    def setUp(self):
        _reset_runtime_state()

    def tearDown(self):
        _reset_runtime_state()

    def _mock_settings(self):
        return _settings()

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(navpy_mod.runtime, "start_instance")
    @patch.object(navpy_mod.runtime, "sim_connection_from_settings",
                  return_value="udp:0.0.0.0:5760")
    @patch.object(navpy_mod.settings_store, "get")
    def test_duplicate_sysids_rejected_before_any_start(
            self, mock_get, _mock_conn, mock_start, _mock_broadcast):
        mock_get.return_value = self._mock_settings()

        req = navpy_mod.NavpyStartAllRequest(instances=[
            navpy_mod.NavpyStartRequest(sys_id=1),
            navpy_mod.NavpyStartRequest(sys_id=2),
            navpy_mod.NavpyStartRequest(sys_id=1),
        ])
        from fastapi import HTTPException
        with self.assertRaises(HTTPException) as ctx:
            asyncio.run(navpy_mod.start_all_navpy(req))

        self.assertEqual(ctx.exception.status_code, 422)
        # The detail must name the offending ids so the operator can fix the
        # request; only the duplicated id (1) appears, not the valid one (2).
        self.assertIn("[1]", ctx.exception.detail)
        mock_start.assert_not_called()
        self.assertEqual(runtime_mod._auto_managed_sysids, set())

    @patch.object(navpy_mod, "emit")
    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(navpy_mod.runtime, "start_instance")
    @patch.object(navpy_mod.runtime, "sim_connection_from_settings",
                  return_value="udp:0.0.0.0:5760")
    @patch.object(navpy_mod.settings_store, "get")
    def test_spawn_failure_reports_item_and_continues_batch(
            self, mock_get, _mock_conn, mock_start, mock_broadcast, mock_emit):
        mock_get.return_value = self._mock_settings()
        inst2 = MagicMock()
        inst2.process.pid = 102
        mock_start.side_effect = [RuntimeError("spawn failed"), inst2]

        req = navpy_mod.NavpyStartAllRequest(instances=[
            navpy_mod.NavpyStartRequest(sys_id=1),
            navpy_mod.NavpyStartRequest(sys_id=2),
        ])
        result = asyncio.run(navpy_mod.start_all_navpy(req))

        self.assertEqual(result["results"]["1"]["status"], "error")
        self.assertIn("spawn failed", result["results"]["1"]["detail"])
        self.assertEqual(result["results"]["2"]["status"], "started")
        # Failed spawn leaves no keep-alive intent; the started one is managed.
        self.assertNotIn(1, runtime_mod._auto_managed_sysids)
        self.assertIn(2, runtime_mod._auto_managed_sysids)
        # The batch still broadcasts status and emits its diagnostic.
        mock_broadcast.assert_awaited_once()
        mock_emit.assert_called_once_with(
            "navpy_batch_finished", source="backend", vehicle_count=2,
            accepted_count=1, rejected_count=1, outcome="partial")


class TestStopRouteManagedIntent(unittest.TestCase):
    """Stop must clear managed intent even when no manager exists (issue #161)."""

    def setUp(self):
        _reset_runtime_state()

    def tearDown(self):
        _reset_runtime_state()

    def _mock_settings(self):
        return _settings()

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(navpy_mod.settings_store, "get")
    def test_stop_without_manager_clears_intent_and_404s(
            self, mock_get, _mock_broadcast):
        mock_get.return_value = self._mock_settings()
        runtime_mod._auto_managed_sysids.add(1)

        from fastapi import HTTPException
        with self.assertRaises(HTTPException) as ctx:
            asyncio.run(navpy_mod.stop_navpy_instance(1))

        self.assertEqual(ctx.exception.status_code, 404)
        # Stop intent must be recorded even when the manager is gone, so a
        # later auto-heal cannot resurrect the companion.
        self.assertNotIn(1, runtime_mod._auto_managed_sysids)

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(navpy_mod.settings_store, "get")
    def test_stop_running_instance_returns_stopped(
            self, mock_get, _mock_broadcast):
        mock_get.return_value = self._mock_settings()
        mgr = MagicMock()
        mgr.stop.return_value = True
        runtime_mod._navpy_mgr = mgr

        result = asyncio.run(navpy_mod.stop_navpy_instance(1))

        self.assertEqual(result, {"status": "stopped", "sys_id": 1})
        mgr.stop.assert_called_once_with(1)

    @patch.object(navpy_mod, "broadcast_sim_status", new_callable=AsyncMock)
    @patch.object(navpy_mod.settings_store, "get")
    def test_stop_unknown_sysid_with_manager_404s(
            self, mock_get, _mock_broadcast):
        mock_get.return_value = self._mock_settings()
        mgr = MagicMock()
        mgr.stop.return_value = False
        runtime_mod._navpy_mgr = mgr

        from fastapi import HTTPException
        with self.assertRaises(HTTPException) as ctx:
            asyncio.run(navpy_mod.stop_navpy_instance(2))

        self.assertEqual(ctx.exception.status_code, 404)


class TestSilentCompanionLinkDiagnosis(unittest.TestCase):
    """Repeated no-heartbeat exits get an escalated, cause-naming log.

    A companion on a black-holed UDP port (old fork script / firewall / wrong
    WIN_IP) exits every ~30 s and is restarted forever by the ensure loop --
    each restart looking healthy in the UI for half a minute. After a few
    consecutive heartbeat-timeout exits, name the candidate causes loudly.
    """

    def setUp(self):
        _reset_runtime_state()
        runtime_mod._auto_managed_sysids.add(1)

    def tearDown(self):
        _reset_runtime_state()

    def _mock_settings(self):
        return _settings()

    def _mgr_with_exit_log(self, log_lines, running=False, uptime=0.0):
        mgr = MagicMock()
        mgr.get_status.return_value = {
            "sys_id": 1,
            "connection": "udp:0.0.0.0:5760",
            "running": running,
            "uptime_s": uptime,
            "log": log_lines,
        }
        runtime_mod._navpy_mgr = mgr
        return mgr

    _HB_EXIT_LOG = ["Error occurred:",
                    "Exception: Vehicle 1 is not broadcasting heartbeats.",
                    "Exiting program..."]

    @patch.object(runtime_mod, "start_instance")
    def test_diagnosis_logged_after_consecutive_heartbeat_exits(self, _start):
        settings = self._mock_settings()
        self._mgr_with_exit_log(self._HB_EXIT_LOG)
        threshold = runtime_mod._STALL_DIAGNOSIS_AFTER

        for _ in range(threshold - 1):
            runtime_mod.ensure_auto_navpy_sim_running([1], settings)
        self.assertEqual(runtime_mod._hb_exit_streaks.get(1), threshold - 1)

        with self.assertLogs(runtime_mod.log, level="ERROR") as captured:
            runtime_mod.ensure_auto_navpy_sim_running([1], settings)
        joined = "\n".join(captured.output)
        self.assertIn("COMPANION_UDP", joined)
        self.assertIn("firewall", joined)
        self.assertIn("WIN_IP", joined)

    @patch.object(runtime_mod, "start_instance")
    def test_other_exit_reason_resets_streak(self, _start):
        settings = self._mock_settings()
        self._mgr_with_exit_log(self._HB_EXIT_LOG)
        runtime_mod.ensure_auto_navpy_sim_running([1], settings)
        self.assertEqual(runtime_mod._hb_exit_streaks.get(1), 1)

        self._mgr_with_exit_log(["Traceback...", "SomeOtherError"])
        runtime_mod.ensure_auto_navpy_sim_running([1], settings)
        self.assertIsNone(runtime_mod._hb_exit_streaks.get(1))

    @patch.object(runtime_mod, "start_instance")
    def test_surviving_past_exit_window_clears_streak(self, _start):
        settings = self._mock_settings()
        self._mgr_with_exit_log(self._HB_EXIT_LOG)
        runtime_mod.ensure_auto_navpy_sim_running([1], settings)
        self.assertEqual(runtime_mod._hb_exit_streaks.get(1), 1)

        # Companion connected this time and outlived the no-heartbeat window.
        self._mgr_with_exit_log([], running=True,
                                uptime=runtime_mod._HB_EXIT_WINDOW_S + 5)
        runtime_mod.ensure_auto_navpy_sim_running([1], settings)
        self.assertIsNone(runtime_mod._hb_exit_streaks.get(1))


class TestAutoManagePolicyBoundaries(unittest.TestCase):
    """Negative lifecycle boundaries of the auto-managed policy set."""

    def setUp(self):
        _reset_runtime_state()

    def tearDown(self):
        _reset_runtime_state()

    def _mock_settings(self, sim_mode=True):
        return _settings(sim_mode=sim_mode, sitl_presets=["udp:0.0.0.0:14560"])

    def test_stop_all_clears_managed_intent(self):
        mgr = MagicMock()
        runtime_mod._navpy_mgr = mgr
        runtime_mod._auto_managed_sysids.update({1, 2})

        runtime_mod.stop_all_instances()

        self.assertEqual(runtime_mod._auto_managed_sysids, set())
        mgr.stop_all.assert_called_once()

    def test_shutdown_clears_managed_intent(self):
        mgr = MagicMock()
        runtime_mod._navpy_mgr = mgr
        runtime_mod._auto_managed_sysids.update({1, 2})

        runtime_mod.stop_all_navpy_if_running()

        self.assertEqual(runtime_mod._auto_managed_sysids, set())
        self.assertIsNone(runtime_mod._navpy_mgr)

    def test_restart_running_does_not_adopt_unmanaged(self):
        mgr = MagicMock()
        mgr.get_all_status.return_value = [
            {"sys_id": 1, "connection": "udp:0.0.0.0:14560", "running": True},
        ]
        mgr.stop.return_value = True
        runtime_mod._navpy_mgr = mgr

        runtime_mod.restart_running_navpy_sim(self._mock_settings())

        self.assertEqual(runtime_mod._auto_managed_sysids, set())

    def test_ensure_noop_when_sim_mode_off(self):
        mgr = MagicMock()
        runtime_mod._navpy_mgr = mgr
        runtime_mod._auto_managed_sysids.add(1)

        restarted = runtime_mod.ensure_auto_navpy_sim_running(
            [1], self._mock_settings(sim_mode=False))

        self.assertEqual(restarted, [])
        mgr.start.assert_not_called()

    def test_watchdog_skips_tick_when_lifecycle_transaction_is_active(self):
        lifecycle = MagicMock()
        lifecycle.acquire.return_value = False

        with patch.object(
            runtime_mod,
            "_configuration_restart_lock",
            lifecycle,
        ):
            restarted = runtime_mod.ensure_auto_navpy_sim_running(
                [1],
                self._mock_settings(),
            )

        self.assertEqual(restarted, [])
        lifecycle.acquire.assert_called_once_with(blocking=False)
        lifecycle.release.assert_not_called()


class TestNavpySimRuntimeStart(unittest.TestCase):
    """Test settings-to-subprocess argument policy in the runtime service."""

    def setUp(self):
        _reset_runtime_state()

    def tearDown(self):
        _reset_runtime_state()

    def _mock_settings(self, dev_mode=False, vision_profile=None,
                       detector_debug_window=False):
        return _settings(dev_mode=dev_mode, vision_profile=vision_profile,
                         detector_debug_window=detector_debug_window)

    def test_detector_debug_enabled_off_by_default_even_when_dev_mode(self):
        # Part C: the cv2 window must NOT open just because dev_mode is on.
        settings = self._mock_settings(dev_mode=True, detector_debug_window=False)
        self.assertFalse(runtime_mod.detector_debug_enabled(settings))

    def test_detector_debug_enabled_true_only_when_window_flag_set(self):
        settings = self._mock_settings(dev_mode=False, detector_debug_window=True)
        self.assertTrue(runtime_mod.detector_debug_enabled(settings))

    def test_start_instance_passes_debug_and_selected_profile(self):
        mgr = MagicMock()
        inst = MagicMock()
        inst.process.pid = 101
        mgr.start.return_value = inst
        runtime_mod._navpy_mgr = mgr
        # Debug window keyed on the explicit flag, NOT dev_mode.
        settings = self._mock_settings(
            dev_mode=False, detector_debug_window=True, vision_profile="siyi_zr10",
        )

        runtime_mod.start_instance(1, "udp:0.0.0.0:14560", settings)

        mgr.start.assert_any_call(
            1,
            "udp:0.0.0.0:14560",
            detector_debug_show=True,
            vision_profile="siyi_zr10",
        )

    def test_start_instance_ignores_blank_profile(self):
        mgr = MagicMock()
        inst = MagicMock()
        inst.process.pid = 101
        mgr.start.return_value = inst
        runtime_mod._navpy_mgr = mgr
        settings = self._mock_settings(dev_mode=False, vision_profile="   ")

        runtime_mod.start_instance(1, "udp:0.0.0.0:14560", settings)

        mgr.start.assert_called_once_with(
            1,
            "udp:0.0.0.0:14560",
            detector_debug_show=False,
            vision_profile=None,
        )

    @patch.dict(os.environ, {runtime_mod._ENV_LOG_LEVEL: "DEBUG"})
    def test_start_instance_forwards_env_log_level(self):
        mgr = MagicMock()
        inst = MagicMock()
        inst.process.pid = 101
        mgr.start.return_value = inst
        runtime_mod._navpy_mgr = mgr
        settings = self._mock_settings(dev_mode=False, vision_profile=None)

        runtime_mod.start_instance(1, "udp:0.0.0.0:14560", settings)

        mgr.start.assert_called_once_with(
            1,
            "udp:0.0.0.0:14560",
            detector_debug_show=False,
            vision_profile=None,
            log_level="DEBUG",
        )

    @patch.dict(os.environ, {}, clear=False)
    def test_start_instance_omits_log_level_when_env_unset(self):
        os.environ.pop(runtime_mod._ENV_LOG_LEVEL, None)
        mgr = MagicMock()
        inst = MagicMock()
        inst.process.pid = 101
        mgr.start.return_value = inst
        runtime_mod._navpy_mgr = mgr
        settings = self._mock_settings(dev_mode=False, vision_profile=None)

        runtime_mod.start_instance(1, "udp:0.0.0.0:14560", settings)

        # log_level kwarg must be absent so the default launch is unchanged.
        _, kwargs = mgr.start.call_args
        self.assertNotIn("log_level", kwargs)


class TestSimLogLevel(unittest.TestCase):
    """sim_log_level() reads the optional companion log-level env override."""

    @patch.dict(os.environ, {}, clear=False)
    def test_returns_none_when_unset(self):
        os.environ.pop(runtime_mod._ENV_LOG_LEVEL, None)
        self.assertIsNone(runtime_mod.sim_log_level())

    @patch.dict(os.environ, {runtime_mod._ENV_LOG_LEVEL: "  DEBUG  "})
    def test_returns_stripped_value(self):
        self.assertEqual(runtime_mod.sim_log_level(), "DEBUG")

    @patch.dict(os.environ, {runtime_mod._ENV_LOG_LEVEL: "   "})
    def test_blank_returns_none(self):
        self.assertIsNone(runtime_mod.sim_log_level())


class TestAutoStartNavpySim(unittest.TestCase):
    """Test auto_start_navpy_sim behavior.

    The route-level wrapper is gated on the vehicle being connected (issue
    #163 stale-discovery race), so these tests stub get_vehicle to a
    connected entry; the gate itself is covered in
    test_vehicles_disconnect.py.
    """

    def setUp(self):
        _reset_runtime_state()
        from gcs.backend.vehicle_manager import vehicle_mgr
        self._gate_patcher = patch.object(
            vehicle_mgr, "get_vehicle", return_value=MagicMock())
        self._gate_patcher.start()

    def tearDown(self):
        self._gate_patcher.stop()
        _reset_runtime_state()

    def _mock_settings(self, sim_mode=True, dev_mode=False, vision_profile=None,
                       detector_debug_window=False):
        return _settings(sim_mode=sim_mode, dev_mode=dev_mode,
                         vision_profile=vision_profile,
                         detector_debug_window=detector_debug_window,
                         sitl_presets=["udp:0.0.0.0:14560"])

    @patch.object(runtime_mod.settings_store, "get")
    def test_skips_when_sim_mode_off(self, mock_get):
        mock_get.return_value = _settings(sim_mode=False)
        self.assertFalse(navpy_mod.auto_start_navpy_sim(1))

    @patch.object(runtime_mod.settings_store, "get")
    @patch("gcs.backend.navpy_process_manager.NavpyProcessManager.start")
    def test_starts_instance_in_sim_mode(self, mock_start, mock_get):
        mock_get.return_value = _settings(sitl_presets=["udp:0.0.0.0:14560"])
        mock_start.return_value = MagicMock()
        self.assertTrue(navpy_mod.auto_start_navpy_sim(1))
        mock_start.assert_called_once_with(
            1,
            "udp:0.0.0.0:14560",
            detector_debug_show=False,
            vision_profile=None,
        )
        self.assertIn(1, runtime_mod._auto_managed_sysids)

    @patch.object(runtime_mod.settings_store, "get")
    @patch("gcs.backend.navpy_process_manager.NavpyProcessManager.start")
    def test_starts_instance_with_debug_when_the_window_is_opted_in(self, mock_start, mock_get):
        """Named for the flag production actually reads.

        This was ``..._when_dev_mode_enabled`` and set ``dev_mode=True``, which
        production has not consulted since the window got its own opt-in. It
        passed only because its MagicMock settings fabricated a truthy
        ``detector_debug_window``; see ``_settings``.
        """
        mock_get.return_value = _settings(
            detector_debug_window=True, sitl_presets=["udp:0.0.0.0:14560"])
        mock_start.return_value = MagicMock()
        self.assertTrue(navpy_mod.auto_start_navpy_sim(1))
        mock_start.assert_called_once_with(
            1,
            "udp:0.0.0.0:14560",
            detector_debug_show=True,
            vision_profile=None,
        )

    @patch.object(runtime_mod.settings_store, "get")
    @patch("gcs.backend.navpy_process_manager.NavpyProcessManager.start")
    def test_the_debug_window_is_decided_by_its_own_flag_not_dev_mode(
            self, mock_start, mock_get):
        """All four combinations, so the independence is pinned both ways.

        The window is pumped on the companion main thread and a stuck one can
        freeze the process under 3-UAV load, so a managed companion must never
        open it just because someone is in dev mode. Asserting only the two
        window=True/False cases would still pass if dev_mode were OR-ed back in.
        """
        for window in (True, False):
            for dev in (True, False):
                with self.subTest(detector_debug_window=window, dev_mode=dev):
                    mock_start.reset_mock()
                    _reset_runtime_state()
                    mock_get.return_value = _settings(
                        detector_debug_window=window, dev_mode=dev,
                        sitl_presets=["udp:0.0.0.0:14560"])
                    mock_start.return_value = MagicMock()
                    self.assertTrue(navpy_mod.auto_start_navpy_sim(1))
                    mock_start.assert_called_once_with(
                        1,
                        "udp:0.0.0.0:14560",
                        detector_debug_show=window,
                        vision_profile=None,
                    )

    @patch.object(runtime_mod.settings_store, "get")
    @patch("gcs.backend.navpy_process_manager.NavpyProcessManager.start")
    def test_auto_start_passes_selected_vision_profile(self, mock_start, mock_get):
        mock_get.return_value = _settings(
            sitl_presets=["udp:0.0.0.0:14560"], vision_profile="novoxy_dual")
        mock_start.return_value = MagicMock()
        self.assertTrue(navpy_mod.auto_start_navpy_sim(1))
        mock_start.assert_called_once_with(
            1,
            "udp:0.0.0.0:14560",
            detector_debug_show=False,
            vision_profile="novoxy_dual",
        )

    @patch.object(runtime_mod.settings_store, "get")
    @patch("gcs.backend.navpy_process_manager.NavpyProcessManager.start")
    def test_skips_already_running(self, mock_start, mock_get):
        mock_get.return_value = _settings(sitl_presets=["udp:0.0.0.0:14560"])
        mock_start.side_effect = ValueError("already running")
        self.assertFalse(navpy_mod.auto_start_navpy_sim(1))
        self.assertIn(1, runtime_mod._auto_managed_sysids)

    def test_ensure_restarts_exited_auto_managed_instance(self):
        settings = self._mock_settings()
        mgr = MagicMock()
        mgr.get_status.return_value = {
            "sys_id": 2,
            "connection": "tcp:127.0.0.1:5770",
            "running": False,
        }
        runtime_mod._navpy_mgr = mgr
        runtime_mod._auto_managed_sysids.add(2)

        restarted = runtime_mod.ensure_auto_navpy_sim_running([1, 2, 3], settings)

        self.assertEqual(restarted, [2])
        mgr.start.assert_called_once_with(
            2,
            "tcp:127.0.0.1:5770",
            detector_debug_show=False,
            vision_profile=None,
        )

    def test_manual_stop_prevents_auto_restart(self):
        settings = self._mock_settings()
        mgr = MagicMock()
        runtime_mod._navpy_mgr = mgr
        runtime_mod._auto_managed_sysids.add(2)

        runtime_mod.stop_instance(2)
        restarted = runtime_mod.ensure_auto_navpy_sim_running([2], settings)

        self.assertEqual(restarted, [])
        mgr.stop.assert_called_once_with(2)
        mgr.start.assert_not_called()


class TestRestartRunningNavpySim(unittest.TestCase):
    """Test profile-change restarts for running NavPy sim instances."""

    def setUp(self):
        _reset_runtime_state()

    def tearDown(self):
        _reset_runtime_state()

    def _mock_settings(self, sim_mode=True, dev_mode=False, vision_profile=None,
                       detector_debug_window=False):
        return _settings(sim_mode=sim_mode, dev_mode=dev_mode,
                         vision_profile=vision_profile,
                         detector_debug_window=detector_debug_window,
                         sitl_presets=["udp:0.0.0.0:14560"])

    def test_returns_empty_without_manager(self):
        settings = self._mock_settings(sim_mode=True, vision_profile="novoxy_dual")

        restarted = navpy_mod.restart_running_navpy_sim(settings)

        self.assertEqual(restarted, [])

    def test_skips_when_sim_mode_off(self):
        mgr = MagicMock()
        runtime_mod._navpy_mgr = mgr
        settings = self._mock_settings(sim_mode=False, vision_profile="novoxy_dual")

        restarted = navpy_mod.restart_running_navpy_sim(settings)

        self.assertEqual(restarted, [])
        mgr.get_all_status.assert_not_called()

    def test_restarts_only_running_instances_with_selected_profile(self):
        mgr = MagicMock()
        mgr.get_all_status.return_value = [
            {"sys_id": 1, "connection": "udp:0.0.0.0:14560", "running": True},
            {"sys_id": 2, "connection": "udp:0.0.0.0:14570", "running": False},
        ]
        mgr.stop.return_value = True
        runtime_mod._navpy_mgr = mgr
        # detector_debug_window, NOT dev_mode. This asked for dev_mode=True and
        # asserted the debug window came on -- a contract production dropped when
        # the window got its own explicit opt-in. It passed anyway because the
        # MagicMock it built settings from fabricated a truthy flag.
        settings = self._mock_settings(
            sim_mode=True,
            detector_debug_window=True,
            vision_profile="novoxy_dual",
        )

        restarted = navpy_mod.restart_running_navpy_sim(settings)

        self.assertEqual(restarted, [1])
        mgr.stop.assert_called_once_with(1)
        mgr.start.assert_called_once_with(
            1,
            "udp:0.0.0.0:14560",
            detector_debug_show=True,
            vision_profile="novoxy_dual",
        )

    def test_configured_restart_touches_exact_sysids_and_waits_for_hydration(self):
        events: list[tuple[str, int]] = []
        mgr = MagicMock()
        mgr.get_all_status.return_value = [
            {
                "sys_id": sys_id,
                "connection": f"udp:0.0.0.0:{14550 + sys_id * 10}",
                "running": True,
                "pid": 100 + sys_id,
            }
            for sys_id in (1, 2, 3, 4)
        ]
        mgr.stop.side_effect = lambda sys_id: events.append(("stop", sys_id)) or True
        mgr.wait_ready.side_effect = lambda sys_id, _pid, _timeout: {
            "mission_items": 10,
            "targ_wps": sys_id + 4,
            "nav_last_wp": 2,
        }
        mgr.stop_generation.return_value = True
        runtime_mod._navpy_mgr = mgr
        runtime_mod._auto_managed_sysids.update({1, 2, 3, 4})
        expected = {
            sys_id: {"targ_wps": sys_id + 4, "nav_last_wp": 2}
            for sys_id in (1, 2, 3)
        }

        def start(sys_id, _connection, _settings):
            events.append(("start", sys_id))
            return SimpleNamespace(process=SimpleNamespace(pid=200 + sys_id))

        with patch.object(runtime_mod, "start_instance", side_effect=start):
            result = runtime_mod.restart_configured_navpy_sim(
                expected,
                self._mock_settings(),
                timeout_s=1.0,
            )

        self.assertEqual(events, [
            ("stop", 1), ("stop", 2), ("stop", 3),
            ("start", 1), ("start", 2), ("start", 3),
        ])
        self.assertEqual([item["sys_id"] for item in result], [1, 2, 3])
        self.assertEqual(runtime_mod._auto_managed_sysids, {1, 2, 3, 4})
        mgr.wait_ready.assert_any_call(1, 201, unittest.mock.ANY)

    def test_configured_restart_rejects_stale_runtime_parameters(self):
        mgr = MagicMock()
        mgr.get_all_status.return_value = [{
            "sys_id": 1,
            "connection": "udp:0.0.0.0:14560",
            "running": True,
            "pid": 101,
        }]
        mgr.stop.return_value = True
        mgr.stop_generation.return_value = True
        mgr.wait_ready.return_value = {
            "mission_items": 10,
            "targ_wps": 99,
            "nav_last_wp": 2,
        }
        runtime_mod._navpy_mgr = mgr
        replacement = SimpleNamespace(process=SimpleNamespace(pid=201))

        with patch.object(runtime_mod, "start_instance", return_value=replacement):
            with self.assertRaisesRegex(RuntimeError, "loaded targ_wps=99"):
                runtime_mod.restart_configured_navpy_sim(
                    {1: {"targ_wps": 5, "nav_last_wp": 2}},
                    self._mock_settings(),
                    timeout_s=1.0,
                )

        mgr.stop_generation.assert_called_once_with(1, 201)

    def test_configured_restart_restores_managed_intent_after_failure(self):
        mgr = MagicMock()
        mgr.get_all_status.return_value = [
            {
                "sys_id": sys_id,
                "connection": f"udp:0.0.0.0:{14550 + sys_id * 10}",
                "running": True,
                "pid": 100 + sys_id,
            }
            for sys_id in (1, 2)
        ]
        mgr.stop.side_effect = [OSError("stop failed"), True]
        mgr.stop_generation.return_value = True
        runtime_mod._navpy_mgr = mgr
        runtime_mod._auto_managed_sysids.update({1, 2, 4})
        replacement = SimpleNamespace(process=SimpleNamespace(pid=201))

        with patch.object(
            runtime_mod,
            "start_instance",
            return_value=replacement,
        ) as start:
            with self.assertRaisesRegex(RuntimeError, "stop failed"):
                runtime_mod.restart_configured_navpy_sim(
                    {
                        1: {"targ_wps": 5, "nav_last_wp": 2},
                        2: {"targ_wps": 6, "nav_last_wp": 2},
                    },
                    self._mock_settings(),
                    timeout_s=1.0,
                )

        start.assert_called_once_with(
            2,
            "udp:0.0.0.0:14570",
            unittest.mock.ANY,
        )
        mgr.stop_generation.assert_called_once_with(2, 201)
        self.assertEqual(runtime_mod._auto_managed_sysids, {1, 2, 4})

    def test_configured_restart_rolls_back_started_generation_on_timeout(self):
        mgr = MagicMock()
        mgr.get_all_status.return_value = [{
            "sys_id": 1,
            "connection": "udp:0.0.0.0:14560",
            "running": True,
            "pid": 101,
        }]
        mgr.stop.return_value = True
        mgr.stop_generation.return_value = True
        mgr.wait_ready.side_effect = TimeoutError("not ready")
        runtime_mod._navpy_mgr = mgr
        replacement = SimpleNamespace(process=SimpleNamespace(pid=201))

        with patch.object(runtime_mod, "start_instance", return_value=replacement):
            with self.assertRaisesRegex(TimeoutError, "not ready"):
                runtime_mod.restart_configured_navpy_sim(
                    {1: {"targ_wps": 5, "nav_last_wp": 2}},
                    self._mock_settings(),
                    timeout_s=1.0,
                )

        mgr.stop_generation.assert_called_once_with(1, 201)

    def test_configured_restart_rolls_back_partial_start_batch(self):
        mgr = MagicMock()
        mgr.get_all_status.return_value = [
            {
                "sys_id": sys_id,
                "connection": f"udp:0.0.0.0:{14550 + sys_id * 10}",
                "running": True,
                "pid": 100 + sys_id,
            }
            for sys_id in (1, 2)
        ]
        mgr.stop.return_value = True
        mgr.stop_generation.return_value = True
        runtime_mod._navpy_mgr = mgr
        replacement = SimpleNamespace(process=SimpleNamespace(pid=201))

        with patch.object(
            runtime_mod,
            "start_instance",
            side_effect=[replacement, RuntimeError("spawn failed")],
        ):
            with self.assertRaisesRegex(RuntimeError, "spawn failed"):
                runtime_mod.restart_configured_navpy_sim(
                    {
                        1: {"targ_wps": 5, "nav_last_wp": 2},
                        2: {"targ_wps": 6, "nav_last_wp": 2},
                    },
                    self._mock_settings(),
                    timeout_s=1.0,
                )

        mgr.stop_generation.assert_called_once_with(1, 201)

    def test_configured_restart_uses_one_decreasing_batch_deadline(self):
        mgr = MagicMock()
        mgr.get_all_status.return_value = [
            {
                "sys_id": sys_id,
                "connection": f"udp:0.0.0.0:{14550 + sys_id * 10}",
                "running": True,
                "pid": 100 + sys_id,
            }
            for sys_id in (1, 2, 3)
        ]
        mgr.stop.return_value = True
        mgr.wait_ready.side_effect = lambda sys_id, _pid, _timeout: {
            "mission_items": 10,
            "targ_wps": sys_id + 4,
            "nav_last_wp": 2,
        }
        runtime_mod._navpy_mgr = mgr
        expected = {
            sys_id: {"targ_wps": sys_id + 4, "nav_last_wp": 2}
            for sys_id in (1, 2, 3)
        }

        def start(sys_id, _connection, _settings):
            return SimpleNamespace(process=SimpleNamespace(pid=200 + sys_id))

        with patch.object(runtime_mod, "start_instance", side_effect=start), patch(
            "gcs.backend.navpy_sim_configured_restart.time.monotonic",
            side_effect=(100.0, 100.1, 100.4, 100.9),
        ):
            runtime_mod.restart_configured_navpy_sim(
                expected,
                self._mock_settings(),
                timeout_s=1.0,
            )

        for actual, expected_timeout in zip(
            [call.args[2] for call in mgr.wait_ready.call_args_list],
            [0.9, 0.6, 0.1],
            strict=True,
        ):
            self.assertAlmostEqual(actual, expected_timeout)

    def test_operator_stop_waits_for_restart_then_clears_restored_intent(self):
        restart_stop_entered = threading.Event()
        release_restart_stop = threading.Event()
        operator_stop_attempted = threading.Event()
        manual_stop_finished = threading.Event()
        stop_calls = 0
        mgr = MagicMock()
        mgr.get_all_status.return_value = [{
            "sys_id": 1,
            "connection": "udp:0.0.0.0:14560",
            "running": True,
            "pid": 101,
        }]

        def stop(_sys_id):
            nonlocal stop_calls
            stop_calls += 1
            if stop_calls == 1:
                restart_stop_entered.set()
                self.assertTrue(release_restart_stop.wait(1.0))
            return True

        mgr.stop.side_effect = stop
        mgr.wait_ready.return_value = {
            "mission_items": 10,
            "targ_wps": 5,
            "nav_last_wp": 2,
        }
        runtime_mod._navpy_mgr = mgr
        runtime_mod._auto_managed_sysids.add(1)
        replacement = SimpleNamespace(process=SimpleNamespace(pid=201))
        errors = []

        def configured_restart():
            try:
                runtime_mod.restart_configured_navpy_sim(
                    {1: {"targ_wps": 5, "nav_last_wp": 2}},
                    self._mock_settings(),
                    timeout_s=1.0,
                )
            except BaseException as error:
                errors.append(error)

        def operator_stop():
            operator_stop_attempted.set()
            try:
                runtime_mod.stop_instance(1)
            finally:
                manual_stop_finished.set()

        with patch.object(
            runtime_mod,
            "start_instance",
            return_value=replacement,
        ):
            restart_thread = threading.Thread(target=configured_restart)
            stop_thread = threading.Thread(target=operator_stop)
            restart_thread.start()
            self.assertTrue(restart_stop_entered.wait(1.0))
            stop_thread.start()
            try:
                self.assertTrue(operator_stop_attempted.wait(1.0))
                self.assertFalse(manual_stop_finished.wait(0.1))
            finally:
                release_restart_stop.set()
            restart_thread.join(1.0)
            stop_thread.join(1.0)

        self.assertFalse(restart_thread.is_alive())
        self.assertFalse(stop_thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(stop_calls, 2)
        self.assertNotIn(1, runtime_mod._auto_managed_sysids)

    def test_restart_ready_route_rejects_duplicate_sysids_before_restart(self):
        request = navpy_mod.NavpyRestartReadyRequest(instances=[
            navpy_mod.NavpyRestartReadyInstance(
                sys_id=1,
                expected_params=navpy_mod.NavpyExpectedParams(
                    targ_wps=5,
                    nav_last_wp=2,
                ),
            ),
            navpy_mod.NavpyRestartReadyInstance(
                sys_id=1,
                expected_params=navpy_mod.NavpyExpectedParams(
                    targ_wps=6,
                    nav_last_wp=2,
                ),
            ),
        ])
        with patch.object(navpy_mod.settings_store, "get", return_value=self._mock_settings()), \
             patch.object(runtime_mod, "restart_configured_navpy_sim") as restart:
            with self.assertRaisesRegex(Exception, "Duplicate sys_ids"):
                asyncio.run(navpy_mod.restart_navpy_ready(request))
        restart.assert_not_called()


if __name__ == "__main__":
    unittest.main()
