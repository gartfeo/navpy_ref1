"""Confirmation cards are reset on estop / restart, and disarm_after_guided still fires.

These exercise the WS broadcasts the frontend uses to clear cards; there is no
backend confirmation store anymore (a per-UAV abort is the destructive E-STOP).
"""
from __future__ import annotations

import asyncio
from contextlib import contextmanager
from unittest.mock import MagicMock, AsyncMock, patch

import pytest
from fastapi import FastAPI, HTTPException
from fastapi.testclient import TestClient

from gcs.backend.settings_model import GcsSettings, LaunchSettings


@pytest.fixture
def control_client():
    mock_entry = MagicMock()
    mock_entry.vehicle = MagicMock()
    mock_entry.vehicle.is_armed = True
    mock_entry.mission_uploaded = True
    # A restart counts once a newer heartbeat shows AUTO.
    from navpy.core.ports.flight_mode import FlightMode
    mock_entry.vehicle.heartbeat_count = 0
    mock_entry.vehicle.get_mode = None

    def set_mode(mode, **_):
        mock_entry.vehicle.heartbeat_count += 1
        mock_entry.vehicle.get_mode = FlightMode(mode)

    mock_entry.vehicle.set_mode.side_effect = set_mode

    mock_mgr = MagicMock()
    mock_mgr.get_vehicle = MagicMock(return_value=mock_entry)
    mock_mgr.vehicles = {1: mock_entry}

    with patch("gcs.backend.routes.control.vehicle_mgr", mock_mgr), \
         patch("gcs.backend.routes.control.ws_manager") as mock_ws:
        mock_ws.broadcast = AsyncMock()
        from gcs.backend.routes.control import router
        app = FastAPI()
        app.include_router(router, prefix="/api/control")
        yield TestClient(app), mock_ws


def _broadcast_types(mock_ws):
    return [c.args[0].get("type") for c in mock_ws.broadcast.call_args_list]


class TestEstopBroadcastsReset:
    def test_estop_broadcasts_reset_for_poi(self, control_client):
        tc, mock_ws = control_client
        resp = tc.post("/api/control/command", json={"command": "estop", "sys_ids": [1]})
        assert resp.status_code == 200
        assert "task_confirm_reset" in _broadcast_types(mock_ws)
        reset = next(c.args[0] for c in mock_ws.broadcast.call_args_list
                     if c.args[0].get("type") == "task_confirm_reset")
        assert reset["sys_ids"] == [1]
        assert "reason" not in reset, "E-STOP keeps the map's handled docks"


class TestRestartBroadcastsReset:
    def test_restart_broadcasts_reset(self, control_client):
        tc, mock_ws = control_client
        resp = tc.post("/api/control/restart", json={"sys_ids": [1], "force": True})
        assert resp.status_code == 200
        assert "task_confirm_reset" in _broadcast_types(mock_ws)
        reset = next(c.args[0] for c in mock_ws.broadcast.call_args_list
                     if c.args[0].get("type") == "task_confirm_reset")
        assert reset["reason"] == "new_mission"

    def test_one_uav_restarted_starts_a_new_mission(self, control_client):
        tc, mock_ws = control_client
        from gcs.backend.routes import control
        entry = control.vehicle_mgr.get_vehicle.return_value
        control.vehicle_mgr.get_vehicle.side_effect = lambda sys_id: entry if sys_id == 1 else None
        resp = tc.post("/api/control/restart", json={"sys_ids": [1, 2], "force": True})
        assert resp.json()["results"] == {"1": "restarted", "2": "not_connected"}
        mock_ws.broadcast.assert_awaited_once_with(
            {"type": "task_confirm_reset", "sys_ids": [1, 2], "reason": "new_mission"})

    def test_a_restart_that_moved_no_uav_keeps_every_clients_map(self, control_client):
        tc, mock_ws = control_client
        from gcs.backend.routes import control
        control.vehicle_mgr.get_vehicle.return_value = None
        resp = tc.post("/api/control/restart", json={"sys_ids": [1, 2], "force": True})
        assert resp.json()["results"] == {"1": "not_connected", "2": "not_connected"}
        mock_ws.broadcast.assert_awaited_once_with({"type": "task_confirm_reset", "sys_ids": [1, 2]})

    def test_a_restart_that_failed_on_every_uav_keeps_every_clients_map(self, control_client):
        tc, mock_ws = control_client
        from gcs.backend.routes import control
        control.vehicle_mgr.get_vehicle.return_value.vehicle.restart_mission.side_effect = RuntimeError("no link")
        resp = tc.post("/api/control/restart", json={"sys_ids": [1], "force": True})
        assert resp.json()["results"] == {"1": "error: no link"}
        mock_ws.broadcast.assert_awaited_once_with({"type": "task_confirm_reset", "sys_ids": [1]})


@contextmanager
def _launch_client(launch_type, container_launch=None):
    """A /launch route on a fleet of UAVs 1 and 2, with every gate open."""
    entries = {sid: MagicMock(mission_uploaded=True, probed_search_pattern=None) for sid in (1, 2)}
    mgr = MagicMock(vehicles=entries, get_vehicle=MagicMock(side_effect=entries.get))
    store = MagicMock()
    store.get.return_value = GcsSettings(launch=LaunchSettings(launch_type=launch_type))
    ws = MagicMock(broadcast=AsyncMock())
    from gcs.backend.routes import control
    with patch.object(control, "vehicle_mgr", mgr), \
         patch.object(control, "settings_store", store), \
         patch.object(control, "ws_manager", ws), \
         patch.object(control, "_maybe_auto_preflight_cal", AsyncMock(return_value=[])), \
         patch.object(control, "_bungee_launch", AsyncMock(return_value={"status": "launch_complete"})), \
         patch.object(control, "_container_launch",
                      container_launch or AsyncMock(return_value={"status": "container_prepared"})):
        app = FastAPI()
        app.include_router(control.router, prefix="/api/control")
        yield TestClient(app), ws


class TestLaunchStartsANewMission:
    def test_a_container_launch_resets_every_client_for_the_whole_fleet(self):
        with _launch_client("container") as (tc, ws):
            resp = tc.post("/api/control/launch", json={"sys_ids": [1], "force": True})
        assert resp.status_code == 200
        assert resp.json() == {"status": "container_prepared"}
        ws.broadcast.assert_awaited_once_with(
            {"type": "task_confirm_reset", "sys_ids": [1, 2], "reason": "new_mission"})

    @pytest.mark.parametrize("status", [409, 503])
    def test_a_refused_container_launch_clears_nothing(self, status):
        refused = AsyncMock(side_effect=HTTPException(status, "refused"))
        with _launch_client("container", refused) as (tc, ws):
            resp = tc.post("/api/control/launch", json={"sys_ids": [1, 2], "force": True})
        assert resp.status_code == status
        ws.broadcast.assert_not_awaited()

    def test_a_launch_blocked_by_readiness_clears_nothing(self):
        from gcs.backend.routes import control
        with _launch_client("container") as (tc, ws), \
             patch.object(control, "_check_launch_readiness", return_value=["low battery"]):
            resp = tc.post("/api/control/launch", json={"sys_ids": [1, 2]})
        assert resp.status_code == 409
        ws.broadcast.assert_not_awaited()

    def test_a_bungee_launch_continues_the_mission(self):
        with _launch_client("bungee") as (tc, ws):
            resp = tc.post("/api/control/launch", json={"sys_ids": [1], "force": True})
        assert resp.status_code == 200
        ws.broadcast.assert_not_awaited()


class TestDisarmAfterGuidedStillBroadcasts:
    def test_disarm_after_guided_event_broadcast(self):
        """An armed->disarmed-in-GUIDED transition still broadcasts disarm_after_guided
        (the frontend uses it to clear an approved card)."""
        from gcs.backend.telemetry_loop import TelemetryLoop

        snaps = [
            [{"sys_id": 1, "armed": True, "mode": "GUIDED",
              "lat": 1.0, "lon": 2.0, "alt": 3.0}],
            [{"sys_id": 1, "armed": False, "mode": "GUIDED",
              "lat": 1.0, "lon": 2.0, "alt": 3.0}],
        ]
        seq = iter(snaps)
        holder = {}

        def get_snaps():
            try:
                return next(seq)
            except StopIteration:
                holder["loop"]._stop.set()
                return snaps[-1]

        mgr = MagicMock()
        mgr.get_all_snapshots.side_effect = get_snaps
        mgr.get_seen_ids.return_value = [1]

        seen_types = []

        async def run():
            holder["loop"] = TelemetryLoop(mgr)
            with patch("gcs.backend.telemetry_loop.ws_manager") as ws, \
                 patch("gcs.backend.telemetry_loop.settings_store") as ss, \
                 patch("gcs.backend.telemetry_loop.launch_controller") as lc:
                async def _capture(payload):
                    seen_types.append(payload.get("type"))
                ws.broadcast = AsyncMock(side_effect=_capture)
                ss.get.return_value.simulation.sim_mode = False
                ss.get.return_value.connection.ws_broadcast_interval_s = 0.001
                lc.is_prepared = False
                lc.is_running = False
                await holder["loop"].start()
                for _ in range(400):
                    if holder["loop"]._stop.is_set():
                        break
                    await asyncio.sleep(0.005)
                await holder["loop"].stop()

        asyncio.run(run())
        assert "disarm_after_guided" in seen_types
