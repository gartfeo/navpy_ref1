"""Every RTL goes to the mission's default delivery hub (owner ruling 2026-11-09).

Mechanism level: the rally item built from the DDH, the rally params written,
stale rally points cleared when the mission has no DDH, and every failure
reported per vehicle instead of raised or hidden.
"""
from __future__ import annotations

import threading
from unittest.mock import MagicMock

import pytest
from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_NAV_RALLY_POINT,
    MAV_FRAME_GLOBAL_RELATIVE_ALT,
    MAV_MISSION_TYPE_RALLY,
)

from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.parameter_exchange import (
    ExchangeOutcome, ExchangePolicy, ExchangeReason, ParameterExchange,
)
from gcs.backend.planner.rally_builder import (
    RALLY_PARAM_WRITES, build_rally_items, rally_altitude_m,
)
from gcs.backend.routes.mission_rally import RALLY_UPLOAD_ATTEMPTS, apply_rally

HUB = {"lat": 40.123456789, "lon": 44.987654321, "type": "dock"}


CONFIRMED = ParameterExchange(ExchangeOutcome.CONFIRMED, None, 1)
PAST_DEADLINE = ParameterExchange(ExchangeOutcome.UNATTEMPTED, ExchangeReason.DEADLINE, 0)


def _read(value):
    if value is None:
        return ParameterExchange(ExchangeOutcome.UNCONFIRMED, ExchangeReason.NO_ECHO, 3)
    return ParameterExchange(ExchangeOutcome.CONFIRMED, None, 1, value)


def _vehicle(*, upload_ok=True, write_ok=True, rally_total=0.0):
    vehicle = MagicMock()
    vehicle.target_system = 7
    vehicle.mission_lock = threading.RLock()
    vehicle.upload_rally = MagicMock(return_value=upload_ok)
    vehicle.write_parameter = MagicMock(return_value=CONFIRMED if write_ok else PAST_DEADLINE)
    vehicle.read_parameter = MagicMock(return_value=_read(rally_total))
    return vehicle


def _params(vehicle) -> dict:
    return {c.args[0]: c.args[1] for c in vehicle.write_parameter.call_args_list}


class TestRallyItems:
    def test_one_rally_point_at_the_ddh_at_the_rtl_altitude(self):
        items = build_rally_items(HUB, 120.4, target_system=7)

        assert len(items) == 1
        item = items[0]
        assert item.command == MAV_CMD_NAV_RALLY_POINT
        assert item.mission_type == MAV_MISSION_TYPE_RALLY
        # Above HOME, the same reference as RTL_ALTITUDE.
        assert item.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT
        assert item.seq == 0 and item.target_system == 7
        assert item.x == round(HUB["lat"] * 1e7)
        assert item.y == round(HUB["lon"] * 1e7)
        assert item.z == 120.0

    def test_altitude_outside_ardupilot_int16_is_refused_before_sending(self):
        with pytest.raises(ValueError):
            rally_altitude_m(40000.0)

    def test_rally_params_always_use_the_hub_point(self):
        # INCL_HOME=0: HOME is never chosen over the hub. LIMIT_KM=0: a far hub
        # is never dropped for HOME (Plane's 5 km default would drop it).
        assert dict(RALLY_PARAM_WRITES) == {"RALLY_INCL_HOME": 0.0, "RALLY_LIMIT_KM": 0.0}


class TestSetHubRally:
    def test_uploads_the_hub_point_and_writes_the_rally_params(self):
        vehicle = _vehicle()
        cache = MagicMock()

        status = apply_rally(vehicle, 7, HUB, 150.0, cache=cache)

        assert status == {"applied": True, "action": "set", "total": 1,
                          "error": None, "failed_params": []}
        items = vehicle.upload_rally.call_args.args[0]
        assert [(i.x, i.y, i.z) for i in items] == [
            (round(HUB["lat"] * 1e7), round(HUB["lon"] * 1e7), 150.0)]
        assert _params(vehicle) == {"RALLY_INCL_HOME": 0.0, "RALLY_LIMIT_KM": 0.0}
        vehicle.clear_rally.assert_not_called()
        cache.invalidate_keys.assert_called_once_with(
            7, ["RALLY_TOTAL", "RALLY_INCL_HOME", "RALLY_LIMIT_KM"])

    def test_unacknowledged_upload_is_retried_then_reported_without_param_writes(self):
        vehicle = _vehicle(upload_ok=False)

        status = apply_rally(vehicle, 7, HUB, 150.0, cache=MagicMock())

        assert status["applied"] is False and status["action"] == "set"
        assert "not acknowledged" in status["error"]
        assert vehicle.upload_rally.call_count == RALLY_UPLOAD_ATTEMPTS
        vehicle.write_parameter.assert_not_called()

    def test_rejected_param_write_is_named(self):
        vehicle = _vehicle()
        vehicle.write_parameter.side_effect = lambda name, value, policy: (
            CONFIRMED if name != "RALLY_LIMIT_KM"
            else ParameterExchange(ExchangeOutcome.UNCONFIRMED, ExchangeReason.NO_ECHO, 3))

        status = apply_rally(vehicle, 7, HUB, 150.0, cache=MagicMock())

        assert status["applied"] is False
        assert status["failed_params"] == ["RALLY_LIMIT_KM"]
        assert "RALLY_LIMIT_KM" in status["error"]

    def test_transport_error_is_reported_not_raised_and_cache_invalidated(self):
        vehicle = _vehicle()
        vehicle.upload_rally.side_effect = RuntimeError("link down")
        cache = MagicMock()

        status = apply_rally(vehicle, 7, HUB, 150.0, cache=cache)

        assert status["applied"] is False and status["error"] == "link down"
        cache.invalidate_keys.assert_called_once()


class TestClearRally:
    def test_no_ddh_clears_stale_rally_points_and_confirms_from_rally_total(self):
        vehicle = _vehicle(rally_total=0.0)

        status = apply_rally(vehicle, 7, None, 150.0, cache=MagicMock())

        assert status == {"applied": True, "action": "clear", "total": 0,
                          "error": None, "failed_params": []}
        vehicle.clear_rally.assert_called_once_with()
        assert vehicle.read_parameter.call_args.args[0] == "RALLY_TOTAL"
        vehicle.upload_rally.assert_not_called()
        vehicle.write_parameter.assert_not_called()

    def test_points_left_on_the_vehicle_are_a_failure(self):
        vehicle = _vehicle(rally_total=2.0)

        status = apply_rally(vehicle, 7, None, 150.0, cache=MagicMock())

        assert status["applied"] is False and status["total"] == 2
        assert "2 rally point" in status["error"]

    @pytest.mark.parametrize("raw", [None, float("nan")])
    def test_unreadable_rally_total_is_unknown_not_cleared(self, raw):
        vehicle = _vehicle(rally_total=raw)

        status = apply_rally(vehicle, 7, None, 150.0, cache=MagicMock())

        assert status["applied"] is False
        assert "could not be read" in status["error"]


class TestDeadline:
    @pytest.mark.parametrize("hub", [HUB, None])
    def test_past_the_deadline_no_vehicle_work_starts(self, hub):
        vehicle = _vehicle()
        policy = ExchangePolicy(3, 0.0, stop_on_silence=True)  # deadline long gone

        status = apply_rally(vehicle, 7, hub, 150.0, cache=MagicMock(), policy=policy)

        assert status["applied"] is False and "deadline" in status["error"]
        vehicle.upload_rally.assert_not_called()
        vehicle.clear_rally.assert_not_called()
        vehicle.write_parameter.assert_not_called()


class TestParamExchangesKeepTheRequestDeadline:
    """Codex round 1, finding 3: every rally parameter exchange is bounded by
    the request's own policy, so one that starts just before the deadline
    cannot retry past it, and a write after it is not sent."""

    def test_deadline_passing_during_the_first_write_stops_the_second(self, monkeypatch):
        now = [100.0]
        monkeypatch.setattr(rt, "monotonic", lambda: now[0])
        policy = ExchangePolicy(3, 105.0, stop_on_silence=True)
        seen = []

        def write_parameter(name, value, exchange_policy):
            # The vehicle's exchange loop stops at exchange_policy's deadline.
            seen.append((name, exchange_policy))
            if now[0] >= exchange_policy.deadline_s:
                return PAST_DEADLINE
            now[0] = 106.0  # this exchange used up the request's time
            return CONFIRMED

        vehicle = _vehicle()
        vehicle.write_parameter = MagicMock(side_effect=write_parameter)

        status = apply_rally(vehicle, 7, HUB, 150.0, cache=MagicMock(), policy=policy)

        assert [name for name, _ in seen] == ["RALLY_INCL_HOME", "RALLY_LIMIT_KM"]
        assert all(p is policy for _, p in seen)
        assert status["applied"] is False
        assert status["failed_params"] == ["RALLY_LIMIT_KM"]

    def test_rally_total_read_is_bounded_by_the_request_policy(self):
        policy = ExchangePolicy(3, rt.monotonic() + 60.0, stop_on_silence=True)
        vehicle = _vehicle()
        vehicle.read_parameter = MagicMock(return_value=PAST_DEADLINE)

        status = apply_rally(vehicle, 7, None, 150.0, cache=MagicMock(), policy=policy)

        assert vehicle.read_parameter.call_args.args == ("RALLY_TOTAL", policy)
        assert status["applied"] is False and "deadline" in status["error"]
