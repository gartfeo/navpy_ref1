"""Tests for the auto preflight-calibration START step (_maybe_auto_preflight_cal).

Covers the pitot-aware branching: gyro is always sent to disarmed vehicles; baro
(which also re-zeros airspeed on ArduPilot) is sent to no-pitot vehicles always,
and to pitot-equipped vehicles only when the operator acknowledged pitot_covered.
"""
from __future__ import annotations

import asyncio
import types
from unittest.mock import MagicMock, call, patch

from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_PREFLIGHT_CALIBRATION

from gcs.backend.routes import control

GYRO = call(MAV_CMD_PREFLIGHT_CALIBRATION, p1=1)
BARO = call(MAV_CMD_PREFLIGHT_CALIBRATION, p3=1)


def _entry(armed: bool = False, airspeed_present=False):
    entry = MagicMock()
    entry.vehicle = MagicMock()
    entry.vehicle.is_armed = armed
    entry.vehicle.airspeed_present = airspeed_present
    entry.vehicle.send_command_long = MagicMock()
    return entry


def _settings(enabled: bool, settle: float = 0.0):
    return types.SimpleNamespace(
        launch=types.SimpleNamespace(
            auto_preflight_cal=enabled,
            preflight_cal_settle_s=settle,
        )
    )


def _run(sys_ids, entries, settings, pitot_covered=False):
    mock_mgr = MagicMock()
    mock_mgr.get_vehicle = MagicMock(side_effect=lambda sid: entries.get(sid))
    with patch.object(control, "vehicle_mgr", mock_mgr):
        return asyncio.run(
            control._maybe_auto_preflight_cal(sys_ids, settings, pitot_covered)
        )


class TestEnableGate:
    def test_disabled_does_nothing(self):
        entries = {1: _entry()}
        assert _run([1], entries, _settings(False)) == []
        entries[1].vehicle.send_command_long.assert_not_called()

    def test_armed_skipped(self):
        entries = {1: _entry(armed=True)}
        assert _run([1], entries, _settings(True)) == []
        entries[1].vehicle.send_command_long.assert_not_called()

    def test_not_connected_skipped(self):
        assert _run([99], {}, _settings(True)) == []


class TestNoPitot:
    def test_sends_gyro_then_baro(self):
        entries = {1: _entry(airspeed_present=False)}
        assert _run([1], entries, _settings(True)) == [1]
        entries[1].vehicle.send_command_long.assert_has_calls([GYRO, BARO])
        assert entries[1].vehicle.send_command_long.call_count == 2


class TestPitotPresent:
    def test_gyro_only_without_ack(self):
        entries = {1: _entry(airspeed_present=True)}
        assert _run([1], entries, _settings(True), pitot_covered=False) == [1]
        entries[1].vehicle.send_command_long.assert_called_once_with(
            MAV_CMD_PREFLIGHT_CALIBRATION, p1=1,
        )

    def test_gyro_and_baro_with_ack(self):
        entries = {1: _entry(airspeed_present=True)}
        assert _run([1], entries, _settings(True), pitot_covered=True) == [1]
        entries[1].vehicle.send_command_long.assert_has_calls([GYRO, BARO])
        assert entries[1].vehicle.send_command_long.call_count == 2

    def test_unknown_presence_treated_conservatively(self):
        # airspeed_present None (no SYS_STATUS yet) → skip baro unless acked.
        entries = {1: _entry(airspeed_present=None)}
        _run([1], entries, _settings(True), pitot_covered=False)
        entries[1].vehicle.send_command_long.assert_called_once_with(
            MAV_CMD_PREFLIGHT_CALIBRATION, p1=1,
        )


class TestMixedFleet:
    def test_per_vehicle_branching(self):
        entries = {
            1: _entry(airspeed_present=False),  # no pitot → gyro + baro
            2: _entry(airspeed_present=True),   # pitot, no ack → gyro only
            3: _entry(armed=True),              # armed → skipped
        }
        caled = _run([1, 2, 3], entries, _settings(True), pitot_covered=False)
        assert caled == [1, 2]
        assert entries[1].vehicle.send_command_long.call_count == 2
        entries[2].vehicle.send_command_long.assert_called_once_with(
            MAV_CMD_PREFLIGHT_CALIBRATION, p1=1,
        )
        entries[3].vehicle.send_command_long.assert_not_called()


if __name__ == "__main__":
    import sys
    import pytest
    sys.exit(pytest.main([__file__, "-v"]))
