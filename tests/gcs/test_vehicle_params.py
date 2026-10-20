"""Tests for per-vehicle AAS parameter download/upload routes."""
from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest
from fastapi.testclient import TestClient

from gcs.backend.aas_params import AAS_PARAM_MAP, AAS_BOOL_FIELDS, AAS_INT_FIELDS
from navpy.core.ports.parameter_exchange import (
    ExchangeOutcome,
    ExchangeReason,
    ParameterExchange,
)


def _confirmed(value: float = 0.0) -> ParameterExchange:
    return ParameterExchange(ExchangeOutcome.CONFIRMED, None, 1, value)


@pytest.fixture
def mock_vehicle():
    """Create a mock VehicleMav with get_parameter / write_parameter."""
    vehicle = MagicMock()
    # Default: return a value for every AAS param
    param_store = {
        "AAS_DEL_PITCH": 5.0,
        "AAS_DEL_THR": -1.0,
        "AAS_DEL_DIR": 0.0,
        "AAS_DEL_P_KP": 1.5,
        "AAS_DEL_PLD": 100.0,
        "AAS_DEL_PLRD": 2.0,
        "AAS_DEL_CTRL": 1.0,
        "AAS_USE_TRN": 1.0,
        "AAS_TARG_WPS": 16.0,
        "AAS_TARG_ALT": 150.0,
        "AAS_NAV_LAST_WP": 3.0,
        "AAS_NAV_MIN_ALT": 150.0,
        "AAS_NAV_CWT": 30.0,
        "AAS_NAV_CGT": 15.0,
        "AAS_NAV_AUTO_CM": 1.0,
        "AAS_NAV_CM_FL": 1.0,
        "AAS_LOG_DEFER": 0.0,
        "AAS_LOG_RATE": 2.0,
    }
    vehicle.get_parameter = MagicMock(side_effect=lambda name: param_store.get(name))
    vehicle.write_parameter = MagicMock(side_effect=lambda name, value, policy: _confirmed(value))
    return vehicle


@pytest.fixture
def mock_entry(mock_vehicle):
    """Create a mock VehicleEntry."""
    entry = MagicMock()
    entry.sys_id = 1
    entry.vehicle = mock_vehicle
    return entry


@pytest.fixture
def client(mock_entry):
    """Create a test client with a mocked vehicle manager."""
    mock_mgr = MagicMock()
    mock_mgr.get_vehicle = MagicMock(return_value=mock_entry)
    with patch("gcs.backend.routes.params.vehicle_mgr", mock_mgr):
        from gcs.backend.main import app
        yield TestClient(app)


class TestGetVehicleParams:
    def test_download_params(self, client, mock_vehicle):
        resp = client.get("/api/vehicles/1/params")
        assert resp.status_code == 200
        data = resp.json()
        assert data["sys_id"] == 1
        params = data["params"]
        # Check boolean conversion
        assert params["del_dir"] is False
        assert params["use_trn"] is True
        assert params["nav_auto_cm"] is True
        assert params["log_defer"] is False
        # Check int conversion
        assert params["del_ctrl"] == 1
        assert params["targ_wps"] == 16
        assert params["nav_last_wp"] == 3
        assert params["nav_cgt"] == 15
        # Check float pass-through
        assert params["del_p_kp"] == 1.5
        assert params["del_thr"] == -1.0
        assert params["targ_alt"] == 150.0

    def test_download_vehicle_not_found(self):
        mock_mgr = MagicMock()
        mock_mgr.get_vehicle = MagicMock(return_value=None)
        with patch("gcs.backend.routes.params.vehicle_mgr", mock_mgr):
            from gcs.backend.main import app
            c = TestClient(app)
            resp = c.get("/api/vehicles/99/params")
            assert resp.status_code == 404

    def test_download_partial_params(self):
        """When some params are missing from vehicle, only return available ones."""
        vehicle = MagicMock()
        vehicle.get_parameter = MagicMock(
            side_effect=lambda name: 42.0 if name == "AAS_TARG_ALT" else None,
        )
        entry = MagicMock()
        entry.sys_id = 1
        entry.vehicle = vehicle
        mock_mgr = MagicMock()
        mock_mgr.get_vehicle = MagicMock(return_value=entry)
        with patch("gcs.backend.routes.params.vehicle_mgr", mock_mgr):
            from gcs.backend.main import app
            c = TestClient(app)
            resp = c.get("/api/vehicles/1/params")
            assert resp.status_code == 200
            params = resp.json()["params"]
            assert params == {"targ_alt": 42.0}


class TestSetVehicleParams:
    def test_upload_params(self, client, mock_vehicle):
        resp = client.put(
            "/api/vehicles/1/params",
            json={"params": {"del_pitch": 10.0, "use_trn": False}},
        )
        assert resp.status_code == 200
        data = resp.json()
        assert data["results"]["del_pitch"] is True
        assert data["results"]["use_trn"] is True
        # Check that write_parameter was called with correct MAVLink values
        calls = {c.args[0]: c.args[1] for c in mock_vehicle.write_parameter.call_args_list}
        assert calls["AAS_DEL_PITCH"] == 10.0
        assert calls["AAS_USE_TRN"] == 0.0  # bool False → 0.0

    def test_upload_bool_conversion(self, client, mock_vehicle):
        resp = client.put(
            "/api/vehicles/1/params",
            json={"params": {"del_dir": True, "log_defer": True}},
        )
        assert resp.status_code == 200
        calls = {c.args[0]: c.args[1] for c in mock_vehicle.write_parameter.call_args_list}
        assert calls["AAS_DEL_DIR"] == 1.0
        assert calls["AAS_LOG_DEFER"] == 1.0

    def test_upload_ignores_unknown_fields(self, client, mock_vehicle):
        resp = client.put(
            "/api/vehicles/1/params",
            json={"params": {"unknown_field": 99.0, "del_pitch": 5.0}},
        )
        assert resp.status_code == 200
        assert "unknown_field" not in resp.json()["results"]
        assert resp.json()["results"]["del_pitch"] is True

    def test_writes_use_the_gcs_budget_and_one_request_deadline(self, client, mock_vehicle):
        """radio_budget.md, todo 101: 12 sends, a link check, one deadline for the request."""
        from gcs.backend.routes import params

        with patch.object(params.rt, "monotonic", return_value=100.0):
            client.put("/api/vehicles/1/params",
                       json={"params": {"del_pitch": 5.0, "use_trn": True}, "timeout_s": 20})
        policies = {c.args[2] for c in mock_vehicle.write_parameter.call_args_list}
        assert len(policies) == 1  # shared by every parameter
        (policy,) = policies
        assert (policy.attempts, policy.deadline_s, policy.stop_on_silence) == (
            params.GCS_PARAM_ATTEMPTS, 120.0, True) and params.GCS_PARAM_ATTEMPTS == 12

    def test_a_failed_write_says_why_and_an_unattempted_one_is_not_invalidated(
            self, client, mock_vehicle):
        outcomes = {
            "AAS_DEL_PITCH": ParameterExchange(ExchangeOutcome.UNCONFIRMED, ExchangeReason.LINK_LOST, 4),
            "AAS_USE_TRN": ParameterExchange(ExchangeOutcome.UNATTEMPTED, ExchangeReason.DEADLINE, 0),
        }
        mock_vehicle.write_parameter = MagicMock(side_effect=lambda name, value, policy: outcomes[name])
        with patch("gcs.backend.routes.params.full_param_cache") as cache:
            resp = client.put("/api/vehicles/1/params",
                              json={"params": {"del_pitch": 5.0, "use_trn": True}})
        assert resp.json()["results"] == {
            "del_pitch": {"ok": False, "outcome": "unconfirmed", "reason": "link_lost"},
            "use_trn": {"ok": False, "outcome": "unattempted", "reason": "deadline"},
        }
        cache.invalidate_keys.assert_called_once_with(1, ["AAS_DEL_PITCH"])

    @pytest.mark.parametrize("budget", ["0", "-1", "Infinity", "NaN", '"20"', "true", "[20]", "1" + "0" * 400])
    def test_a_budget_that_is_not_a_finite_positive_number_is_refused(self, client, budget):
        # Raw JSON: Python's parser takes Infinity and NaN, which a strict client cannot send.
        body = '{"params": {"del_pitch": 5.0}, "timeout_s": ' + budget + "}"
        resp = client.put("/api/vehicles/1/params", content=body,
                          headers={"Content-Type": "application/json"})
        assert resp.status_code == 400

    def test_a_named_read_refuses_an_infinite_budget(self, client):
        assert client.get("/api/vehicles/1/params/named?names=A&timeout_s=inf").status_code == 400

    def test_upload_vehicle_not_found(self):
        mock_mgr = MagicMock()
        mock_mgr.get_vehicle = MagicMock(return_value=None)
        with patch("gcs.backend.routes.params.vehicle_mgr", mock_mgr):
            from gcs.backend.main import app
            c = TestClient(app)
            resp = c.put(
                "/api/vehicles/99/params",
                json={"params": {"del_pitch": 5.0}},
            )
            assert resp.status_code == 404


class TestNamedParamReads:
    """``GET /params/named``: fresh reads by name for the lockstep command bounds."""

    def test_reads_each_name_fresh_in_order(self, client, mock_vehicle):
        values = {"PTCH_LIM_MIN_DEG": -25.0, "TRIM_THROTTLE": 45.0}
        mock_vehicle.read_parameter = MagicMock(side_effect=lambda name, policy: _confirmed(values[name]))
        resp = client.get("/api/vehicles/1/params/named?names=TRIM_THROTTLE,ptch_lim_min_deg")
        assert resp.status_code == 200
        assert resp.json() == {"sys_id": 1, "params": [
            {"name": "TRIM_THROTTLE", "value": 45.0},
            {"name": "PTCH_LIM_MIN_DEG", "value": -25.0},
        ]}
        mock_vehicle.get_parameter.assert_not_called()  # never the cache

    def test_an_unanswered_name_is_an_error_that_lists_it(self, client, mock_vehicle):
        mock_vehicle.read_parameter = MagicMock(side_effect=lambda name, policy: (
            _confirmed(1.0) if name == "TRIM_THROTTLE"
            else ParameterExchange(ExchangeOutcome.UNCONFIRMED, ExchangeReason.NO_ECHO, 12)))
        resp = client.get("/api/vehicles/1/params/named?names=TRIM_THROTTLE,NOPE_A,NOPE_B")
        assert resp.status_code == 502
        assert resp.json()["detail"] == {"unanswered": ["NOPE_A", "NOPE_B"]}

    def test_empty_or_duplicate_names_are_refused(self, client):
        assert client.get("/api/vehicles/1/params/named?names=,").status_code == 400
        assert client.get("/api/vehicles/1/params/named?names=A,a").status_code == 400

    def test_vehicle_not_connected(self):
        mock_mgr = MagicMock()
        mock_mgr.get_vehicle = MagicMock(return_value=None)
        with patch("gcs.backend.routes.params.vehicle_mgr", mock_mgr):
            from gcs.backend.main import app
            resp = TestClient(app).get("/api/vehicles/9/params/named?names=A")
        assert resp.status_code == 404
