"""Tests for the manual preflight-calibration command (gyro + gated baro).

Baro (p3=1) re-zeros the airspeed sensor on pitot-equipped ArduPilot vehicles,
so the handler only sends it when the vehicle definitely has no pitot
(airspeed_present is False) or the operator acknowledged the pitot is covered
(params.pitot_covered). Unknown presence (None) is treated conservatively.
"""
from __future__ import annotations

from unittest.mock import MagicMock, call, patch

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_PREFLIGHT_CALIBRATION

from gcs.backend.routes.control import router

GYRO = call(MAV_CMD_PREFLIGHT_CALIBRATION, p1=1)
BARO = call(MAV_CMD_PREFLIGHT_CALIBRATION, p3=1)


def _make_entry(sys_id: int, armed: bool = False, airspeed_present=False):
    entry = MagicMock()
    entry.sys_id = sys_id
    entry.vehicle = MagicMock()
    entry.vehicle.is_armed = armed
    entry.vehicle.airspeed_present = airspeed_present
    entry.vehicle.send_command_long = MagicMock()
    return entry


@pytest.fixture
def client():
    entries = {
        1: _make_entry(1, armed=False, airspeed_present=False),  # no pitot
        2: _make_entry(2, armed=True),                           # armed
        3: _make_entry(3, armed=False, airspeed_present=True),   # pitot present
        4: _make_entry(4, armed=False, airspeed_present=None),   # presence unknown
    }
    mock_mgr = MagicMock()
    mock_mgr.get_vehicle = MagicMock(side_effect=lambda sid: entries.get(sid))
    mock_mgr.vehicles = entries

    with patch("gcs.backend.routes.control.vehicle_mgr", mock_mgr):
        app = FastAPI()
        app.include_router(router, prefix="/api/control")
        yield TestClient(app), entries


def _cmd(tc, sys_ids, pitot_covered=None):
    body = {"command": "preflight_cal", "sys_ids": sys_ids}
    if pitot_covered is not None:
        body["params"] = {"pitot_covered": pitot_covered}
    return tc.post("/api/control/command", json=body)


class TestManualPreflightCal:
    def test_no_pitot_sends_gyro_then_baro(self, client):
        tc, entries = client
        res = _cmd(tc, [1])
        assert res.status_code == 200
        assert res.json()["results"]["1"] == "preflight_cal_sent"
        entries[1].vehicle.send_command_long.assert_has_calls([GYRO, BARO])
        assert entries[1].vehicle.send_command_long.call_count == 2

    def test_pitot_without_ack_sends_gyro_only(self, client):
        tc, entries = client
        res = _cmd(tc, [3])
        assert res.json()["results"]["3"] == "preflight_cal_sent_gyro_only"
        entries[3].vehicle.send_command_long.assert_called_once_with(
            MAV_CMD_PREFLIGHT_CALIBRATION, p1=1)

    def test_unknown_pitot_without_ack_sends_gyro_only(self, client):
        tc, entries = client
        res = _cmd(tc, [4])
        assert res.json()["results"]["4"] == "preflight_cal_sent_gyro_only"
        entries[4].vehicle.send_command_long.assert_called_once_with(
            MAV_CMD_PREFLIGHT_CALIBRATION, p1=1)

    def test_pitot_with_ack_sends_gyro_and_baro(self, client):
        tc, entries = client
        res = _cmd(tc, [3], pitot_covered=True)
        assert res.json()["results"]["3"] == "preflight_cal_sent"
        entries[3].vehicle.send_command_long.assert_has_calls([GYRO, BARO])
        assert entries[3].vehicle.send_command_long.call_count == 2

    def test_unknown_pitot_with_ack_sends_gyro_and_baro(self, client):
        tc, entries = client
        res = _cmd(tc, [4], pitot_covered=True)
        assert res.json()["results"]["4"] == "preflight_cal_sent"
        assert entries[4].vehicle.send_command_long.call_count == 2

    def test_refused_when_armed(self, client):
        tc, entries = client
        res = _cmd(tc, [2])
        assert res.status_code == 200
        assert res.json()["results"]["2"] == "refused_armed"
        entries[2].vehicle.send_command_long.assert_not_called()

    def test_not_connected(self, client):
        tc, entries = client
        res = _cmd(tc, [99])
        assert res.json()["results"]["99"] == "not_connected"

    def test_mixed_pois(self, client):
        tc, entries = client
        res = _cmd(tc, [1, 2])
        results = res.json()["results"]
        assert results["1"] == "preflight_cal_sent"
        assert results["2"] == "refused_armed"
        assert entries[1].vehicle.send_command_long.call_count == 2
        entries[2].vehicle.send_command_long.assert_not_called()


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
