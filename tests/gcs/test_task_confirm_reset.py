"""Confirmation cards are reset on estop / restart, and disarm_after_guided still fires.

These exercise the WS broadcasts the frontend uses to clear cards; there is no
backend confirmation store anymore (a per-UAV abort is the destructive E-STOP).
"""
from __future__ import annotations

import asyncio
from unittest.mock import MagicMock, AsyncMock, patch

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient


@pytest.fixture
def control_client():
    mock_entry = MagicMock()
    mock_entry.vehicle = MagicMock()
    mock_entry.vehicle.is_armed = True
    mock_entry.mission_uploaded = True

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
    def test_estop_broadcasts_reset_for_target(self, control_client):
        tc, mock_ws = control_client
        resp = tc.post("/api/control/command", json={"command": "estop", "sys_ids": [1]})
        assert resp.status_code == 200
        assert "task_confirm_reset" in _broadcast_types(mock_ws)
        reset = next(c.args[0] for c in mock_ws.broadcast.call_args_list
                     if c.args[0].get("type") == "task_confirm_reset")
        assert reset["sys_ids"] == [1]


class TestRestartBroadcastsReset:
    def test_restart_broadcasts_reset(self, control_client):
        tc, mock_ws = control_client
        resp = tc.post("/api/control/restart", json={"sys_ids": [1], "force": True})
        assert resp.status_code == 200
        assert "task_confirm_reset" in _broadcast_types(mock_ws)


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
