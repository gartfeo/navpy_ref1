"""Tests for manual control over WebSocket."""
from __future__ import annotations

from unittest.mock import MagicMock, patch

import orjson
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from gcs.backend.routes import telemetry as tel_mod
from gcs.backend.routes.telemetry import router


def _make_entry(sys_id: int):
    entry = MagicMock()
    entry.sys_id = sys_id
    entry.vehicle = MagicMock()
    entry.vehicle.send_manual_control = MagicMock()
    return entry


@pytest.fixture
def client():
    entries = {1: _make_entry(1), 2: _make_entry(2)}
    mock_mgr = MagicMock()
    mock_mgr.get_vehicle = MagicMock(side_effect=lambda sid: entries.get(sid))
    mock_mgr.get_all_snapshots = MagicMock(return_value=[])

    with patch("gcs.backend.routes.telemetry.vehicle_mgr", mock_mgr):
        app = FastAPI()
        app.include_router(router)
        yield TestClient(app), entries

    # Clean up module-level state between tests
    tel_mod._mc_state.clear()
    tel_mod._mc_task = None


def _send_mc(ws, msg: dict):
    """Send a manual-control JSON message and a ping to flush processing."""
    ws.send_text(orjson.dumps(msg).decode())
    ws.send_text("ping")
    assert ws.receive_text() == "pong"


class TestManualControlWs:
    def test_sends_manual_control(self, client):
        tc, entries = client
        with tc.websocket_connect("/ws/telemetry") as ws:
            _send_mc(ws, {
                "type": "manual_control",
                "sys_id": 1, "x": 500, "y": -300, "z": 700, "r": 100,
            })
        entries[1].vehicle.send_manual_control.assert_called_with(500, -300, 700, 100)

    def test_vehicle_not_found_silent(self, client):
        tc, entries = client
        with tc.websocket_connect("/ws/telemetry") as ws:
            _send_mc(ws, {
                "type": "manual_control",
                "sys_id": 99, "x": 0, "y": 0, "z": 500, "r": 0,
            })
        for e in entries.values():
            e.vehicle.send_manual_control.assert_not_called()

    def test_clamps_values(self, client):
        tc, entries = client
        with tc.websocket_connect("/ws/telemetry") as ws:
            _send_mc(ws, {
                "type": "manual_control",
                "sys_id": 2, "x": 2000, "y": -2000, "z": -100, "r": 5000,
            })
        entries[2].vehicle.send_manual_control.assert_called_with(1000, -1000, 0, 1000)

    def test_neutral_message(self, client):
        tc, entries = client
        with tc.websocket_connect("/ws/telemetry") as ws:
            _send_mc(ws, {
                "type": "manual_control",
                "sys_id": 1, "x": 0, "y": 0, "z": 500, "r": 0,
            })
        entries[1].vehicle.send_manual_control.assert_called_with(0, 0, 500, 0)

    def test_stop_sends_neutral_and_clears_state(self, client):
        tc, entries = client
        with tc.websocket_connect("/ws/telemetry") as ws:
            _send_mc(ws, {
                "type": "manual_control",
                "sys_id": 1, "x": 500, "y": 300, "z": 800, "r": -200,
            })
            entries[1].vehicle.send_manual_control.reset_mock()
            _send_mc(ws, {"type": "manual_control_stop", "sys_id": 1})
        entries[1].vehicle.send_manual_control.assert_called_with(0, 0, 500, 0)
        assert 1 not in tel_mod._mc_state

    def test_stop_unknown_vehicle_silent(self, client):
        tc, entries = client
        with tc.websocket_connect("/ws/telemetry") as ws:
            _send_mc(ws, {"type": "manual_control_stop", "sys_id": 99})
        for e in entries.values():
            e.vehicle.send_manual_control.assert_not_called()

    def test_updates_repeater_state(self, client):
        tc, entries = client
        with tc.websocket_connect("/ws/telemetry") as ws:
            _send_mc(ws, {
                "type": "manual_control",
                "sys_id": 1, "x": 100, "y": 200, "z": 600, "r": -50,
            })
            assert tel_mod._mc_state.get(1) == (100, 200, 600, -50)
