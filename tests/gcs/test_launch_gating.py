"""Tests for launch gating on mission_uploaded flag."""
from __future__ import annotations

from contextlib import contextmanager
from unittest.mock import MagicMock, patch, AsyncMock

import pytest
from fastapi.testclient import TestClient

from gcs.backend.settings_model import GcsSettings, LaunchSettings
from gcs.backend.routes.control import _resolve_channel_map


class TestResolveChannelMap:
    """Auto-assign valid ESP32 channels so out-of-range sys_ids don't error."""

    def test_out_of_range_sysids_get_valid_channels(self):
        # sys_ids 10/20/30 would be rejected as channels; auto-assign 1/2/3
        assert _resolve_channel_map({}, [30, 10, 20]) == {10: 1, 20: 2, 30: 3}

    def test_explicit_mapping_preserved_and_skipped(self):
        m = _resolve_channel_map({"10": 5}, [10, 20, 30])
        assert m[10] == 5            # explicit kept
        assert m[20] == 1 and m[30] == 2  # auto skips the used channel 5

    def test_string_keys_from_settings(self):
        m = _resolve_channel_map({"1": 4}, [1, 2])
        assert m[1] == 4 and m[2] == 1

    def test_more_vehicles_than_channels_leaves_surplus_unmapped(self):
        m = _resolve_channel_map({}, [1, 2, 3, 4, 5, 6, 7])
        assert sorted(m.values()) == [1, 2, 3, 4, 5, 6]
        assert 7 not in m  # surplus unmapped (logged)

    def test_in_range_sysids_map_to_themselves_when_free(self):
        # Common case: sys_ids 1/2/3 → channels 1/2/3
        assert _resolve_channel_map({}, [1, 2, 3]) == {1: 1, 2: 2, 3: 3}


def _mock_settings_store(launch_type: str = "bungee", **launch_kw):
    """Mock settings_store whose .get() returns real GcsSettings.

    The /launch readiness gate now reads tunable gate fields off
    settings.launch, so the store must expose a real LaunchSettings (not a
    bare MagicMock) or those fields would be MagicMocks and break comparisons.
    """
    settings = GcsSettings(launch=LaunchSettings(launch_type=launch_type, **launch_kw))
    store = MagicMock()
    store.get.return_value = settings
    return store


def _mgr_for(entries):
    """Mock vehicle_mgr backed by an entries dict."""
    mgr = MagicMock()
    mgr.companion_status.return_value = "ok"
    mgr.get_vehicle = MagicMock(side_effect=lambda sid: entries.get(sid))
    mgr.vehicles = entries
    return mgr


def _make_entry(sys_id: int, mission_uploaded: bool = False,
                 probed_search_pattern: str | None = None):
    entry = MagicMock()
    entry.sys_id = sys_id
    entry.mission_uploaded = mission_uploaded
    entry.probed_search_pattern = probed_search_pattern
    entry.vehicle = MagicMock()
    entry.vehicle.is_armed = True
    entry.vehicle.set_mode = MagicMock()
    entry.vehicle.send_command_long = MagicMock()
    entry.vehicle.restart_mission = MagicMock()
    # Readiness check fields — defaults pass all checks
    entry.vehicle.prearm_ok = True
    entry.vehicle.prearm_check_state = "ok"
    entry.vehicle.gps_fix_type = 3
    entry.vehicle.gps_hacc = 0.5
    entry.vehicle.rc3_raw = 1000
    entry.vehicle.battery_level = 80
    entry.vehicle.mission_items_count = 10
    return entry


@pytest.fixture
def entries():
    return {
        1: _make_entry(1, mission_uploaded=True),
        2: _make_entry(2, mission_uploaded=False),
        3: _make_entry(3, mission_uploaded=True),
    }


@contextmanager
def _client_for(entries):
    """Yield a TestClient with vehicle_mgr/settings/launch_controller mocked."""
    mock_mgr = MagicMock()
    mock_mgr.companion_status.return_value = "ok"
    mock_mgr.get_vehicle = MagicMock(side_effect=lambda sid: entries.get(sid))
    mock_mgr.vehicles = entries
    mock_store = _mock_settings_store()
    with (
        patch("gcs.backend.routes.control.vehicle_mgr", mock_mgr),
        patch("gcs.backend.routes.control.settings_store", mock_store),
        patch("gcs.backend.routes.control.launch_controller", MagicMock(
            is_running=False, is_prepared=False,
        )),
    ):
        from gcs.backend.main import app
        yield TestClient(app)


@pytest.fixture
def client(entries):
    # Re-probe returns invalid for vehicle 2 (simulates failed probe)
    mock_probe_invalid = MagicMock()
    mock_probe_invalid.valid = False
    mock_mgr = MagicMock()
    mock_mgr.companion_status.return_value = "ok"
    mock_mgr.get_vehicle = MagicMock(side_effect=lambda sid: entries.get(sid))
    mock_mgr.vehicles = {sid: e for sid, e in entries.items()}
    mock_mgr.probe_existing_mission = MagicMock(return_value=mock_probe_invalid)
    mock_store = _mock_settings_store()
    with (
        patch("gcs.backend.routes.control.vehicle_mgr", mock_mgr),
        patch("gcs.backend.routes.control.settings_store", mock_store),
        patch("gcs.backend.routes.control.launch_controller", MagicMock(
            is_running=False, is_prepared=False,
        )),
    ):
        from gcs.backend.main import app
        yield TestClient(app)


class TestCompanionReadiness:
    """Exercise real routes/readiness while mocking all vehicle actions."""

    @contextmanager
    def _endpoint(self, manager, *, launch_type="bungee", **launch_kw):
        from gcs.backend.main import app
        from gcs.backend.routes import control

        launch = AsyncMock(return_value={"status": "launch_reached"})
        with (
            patch.object(control, "vehicle_mgr", manager),
            patch.object(control, "settings_store", _mock_settings_store(launch_type, **launch_kw)),
            patch.object(control, "_maybe_auto_preflight_cal", AsyncMock(return_value=[])),
            patch.object(control, "_bungee_launch", launch),
            patch.object(control, "_container_launch", launch),
            patch.object(control, "ws_manager", MagicMock(broadcast=AsyncMock())),
        ):
            client = TestClient(app)
            try:
                yield client, launch
            finally:
                client.close()

    @pytest.mark.parametrize("endpoint", ["launch", "restart"])
    @pytest.mark.parametrize("status", ["down", "unexpected", None, RuntimeError("status failed")])
    def test_unavailable_companion_blocks_actions(self, endpoint, status):
        entry = _make_entry(1, mission_uploaded=True)
        manager = _mgr_for({1: entry})
        if isinstance(status, Exception):
            manager.companion_status.side_effect = status
        else:
            manager.companion_status.return_value = status
        with self._endpoint(manager) as (client, launch):
            response = client.post(f"/api/control/{endpoint}", json={"sys_ids": [1]})
        assert response.status_code == 409
        assert "Vehicle 1: companion" in response.json()["detail"]
        launch.assert_not_awaited()
        entry.vehicle.restart_mission.assert_not_called()
        manager.companion_status.assert_called_once_with(1)

    @pytest.mark.parametrize("endpoint", ["launch", "restart"])
    @pytest.mark.parametrize("status", ["ok", "checking"])
    def test_live_or_transient_companion_allows_actions(self, endpoint, status):
        entry = _make_entry(1, mission_uploaded=True)
        manager = _mgr_for({1: entry})
        manager.companion_status.return_value = status
        with self._endpoint(manager) as (client, launch):
            response = client.post(f"/api/control/{endpoint}", json={"sys_ids": [1]})
        assert response.status_code == 200
        if endpoint == "launch":
            launch.assert_awaited_once()
        else:
            entry.vehicle.restart_mission.assert_called_once()
        manager.companion_status.assert_called_once_with(1)

    @pytest.mark.parametrize("endpoint", ["launch", "restart"])
    def test_force_preserves_existing_bypass(self, endpoint):
        entry = _make_entry(1, mission_uploaded=True)
        manager = _mgr_for({1: entry})
        manager.companion_status.return_value = "down"
        with self._endpoint(manager) as (client, launch):
            response = client.post(f"/api/control/{endpoint}", json={"sys_ids": [1], "force": True})
        assert response.status_code == 200
        manager.companion_status.assert_not_called()
        if endpoint == "launch":
            launch.assert_awaited_once()
        else:
            entry.vehicle.restart_mission.assert_called_once()

    def test_container_cannot_omit_down_companion_from_fleet(self):
        entries = {sid: _make_entry(sid, mission_uploaded=True) for sid in (1, 2, 3)}
        manager = _mgr_for(entries)
        manager.companion_status.side_effect = lambda sid: "down" if sid == 3 else "ok"
        with self._endpoint(manager, launch_type="container") as (client, launch):
            response = client.post("/api/control/launch", json={"sys_ids": [1, 2]})
        assert response.status_code == 409
        assert "Vehicle 3: companion" in response.json()["detail"]
        assert manager.companion_status.call_count == 3
        launch.assert_not_awaited()

    def test_disabling_tunable_checks_does_not_disable_companion_gate(self):
        manager = _mgr_for({1: _make_entry(1, mission_uploaded=True)})
        manager.companion_status.return_value = "down"
        with self._endpoint(manager, check_prearm_enabled=False, check_gps_enabled=False,
                            check_gps_acc_enabled=False, check_throttle_enabled=False,
                            check_battery_enabled=False) as (client, launch):
            response = client.post("/api/control/launch", json={"sys_ids": [1]})
        assert response.status_code == 409
        assert "companion" in response.json()["detail"]
        launch.assert_not_awaited()

    def test_never_observed_heartbeat_blocks_readiness(self):
        from gcs.backend.vehicle_manager import VehicleManager
        from gcs.backend.routes.control_readiness import _check_launch_readiness

        manager = _mgr_for({1: _make_entry(1, mission_uploaded=True)})
        manager._newest_companion_hb.return_value = None
        manager.companion_status.side_effect = lambda sid: VehicleManager.companion_status(manager, sid)
        assert _check_launch_readiness([1], vehicle_mgr=manager) == [
            "Vehicle 1: companion computer not connected",
        ]
        manager._newest_companion_hb.assert_called_once_with(1)

    def test_disconnected_vehicle_does_not_query_companion(self):
        from gcs.backend.routes.control_readiness import _check_launch_readiness

        manager = _mgr_for({})
        assert _check_launch_readiness([99], vehicle_mgr=manager) == ["Vehicle 99: not connected"]
        manager.companion_status.assert_not_called()


class TestLaunchGating:
    def test_registered_endpoint_uses_current_dependencies(self):
        """An empty real manager must not stand in for the patched readiness path."""
        from gcs.backend.main import app

        for sys_id in (901, 902):
            entry = _make_entry(sys_id, mission_uploaded=True)
            entry.vehicle.gps_fix_type = 0
            manager = _mgr_for({sys_id: entry})
            store = _mock_settings_store()
            with (
                patch("gcs.backend.routes.control.vehicle_mgr", manager),
                patch("gcs.backend.routes.control.settings_store", store),
            ):
                response = TestClient(app).post(
                    "/api/control/launch", json={"sys_ids": [sys_id]},
                )
            assert response.status_code == 409
            assert response.json()["detail"] == f"Vehicle {sys_id}: no GPS 3D fix"
            manager.get_vehicle.assert_called_with(sys_id)
            store.get.assert_called_once()

    def test_launch_blocked_without_upload(self, client):
        """Launch request blocked when a vehicle is missing verified upload."""
        resp = client.post("/api/control/launch", json={"sys_ids": [1, 2]})
        assert resp.status_code == 409
        assert "2" in resp.json()["detail"]

    def test_launch_allowed_with_verified_upload(self, client):
        """Launch proceeds when all vehicles have verified uploads."""
        resp = client.post("/api/control/launch", json={"sys_ids": [1, 3]})
        assert resp.status_code == 200

    def test_partial_upload_blocks_launch(self, client):
        """A mix of uploaded and not-uploaded vehicles blocks launch."""
        resp = client.post("/api/control/launch", json={"sys_ids": [1, 2, 3]})
        assert resp.status_code == 409
        data = resp.json()
        assert "2" in data["detail"]

    def test_nonexistent_vehicle_blocks_launch(self, client):
        """Vehicle not in manager blocks launch (entry is None)."""
        resp = client.post("/api/control/launch", json={"sys_ids": [99]})
        assert resp.status_code == 409

    def test_restart_blocked_without_upload(self, client):
        """Restart skips vehicles without mission_uploaded."""
        resp = client.post("/api/control/restart", json={"sys_ids": [2]})
        assert resp.status_code == 200
        data = resp.json()
        assert data["results"]["2"] == "no_verified_upload"

    def test_restart_allowed_with_upload(self, client):
        """Restart proceeds for vehicles with verified upload."""
        resp = client.post("/api/control/restart", json={"sys_ids": [1]})
        assert resp.status_code == 200
        data = resp.json()
        assert data["results"]["1"] == "restarted"


class TestContainerConnectedRoster:
    """The backend, not the client, owns container launch membership."""

    def test_omitted_connected_vehicle_still_participates_in_readiness(self):
        entries = {
            1: _make_entry(1, mission_uploaded=True),
            2: _make_entry(2, mission_uploaded=True),
            3: _make_entry(3, mission_uploaded=True),
        }
        entries[3].vehicle.battery_level = 5
        store = _mock_settings_store(launch_type="container")
        with (
            patch("gcs.backend.routes.control.vehicle_mgr", _mgr_for(entries)),
            patch("gcs.backend.routes.control.settings_store", store),
            patch("gcs.backend.routes.control.launch_controller", MagicMock(
                is_running=False, is_prepared=False,
            )),
        ):
            from gcs.backend.main import app
            resp = TestClient(app).post(
                "/api/control/launch", json={"sys_ids": [1, 2]},
            )
        assert resp.status_code == 409
        assert "Vehicle 3" in resp.json()["detail"]

    def test_launch_endpoint_replaces_partial_client_roster(self):
        entries = {
            161: _make_entry(161, mission_uploaded=True),
            162: _make_entry(162, mission_uploaded=True),
            163: _make_entry(163, mission_uploaded=True),
        }
        store = _mock_settings_store(launch_type="container")

        async def prepared(req, _settings):
            return {"status": "container_prepared", "sys_ids": req.sys_ids}

        container_launch = AsyncMock(side_effect=prepared)
        with (
            patch("gcs.backend.routes.control.vehicle_mgr", _mgr_for(entries)),
            patch("gcs.backend.routes.control.settings_store", store),
            patch("gcs.backend.routes.control._container_launch", container_launch),
        ):
            from gcs.backend.main import app
            resp = TestClient(app).post(
                "/api/control/launch", json={"sys_ids": [161, 162]},
            )

        assert resp.status_code == 200
        assert resp.json()["sys_ids"] == [161, 162, 163]
        assert container_launch.await_args.args[0].sys_ids == [161, 162, 163]

    def test_container_prepare_receives_and_returns_connected_fleet(self):
        from gcs.backend.models import LaunchRequest
        from gcs.backend.routes.control import _container_launch

        entries = {
            161: _make_entry(161, mission_uploaded=True),
            162: _make_entry(162, mission_uploaded=True),
            163: _make_entry(163, mission_uploaded=True),
        }
        controller = MagicMock(is_running=False, is_prepared=False)

        async def prepare(**kwargs):
            controller.is_prepared = True

        controller.prepare = AsyncMock(side_effect=prepare)
        settings = _mock_settings_store(launch_type="container").get.return_value
        req = LaunchRequest(sys_ids=[161, 162, 163])
        with (
            patch("gcs.backend.routes.control.vehicle_mgr", _mgr_for(entries)),
            patch("gcs.backend.routes.control.launch_controller", controller),
        ):
            import asyncio
            result = asyncio.run(_container_launch(req, settings))

        assert result == {
            "status": "container_prepared",
            "sys_ids": [161, 162, 163],
        }
        assert controller.prepare.await_args.kwargs["sys_ids"] == [161, 162, 163]


class TestPrearmModeNotArmableOnly:
    """prearm_mode_not_armable_only=True should not block launch."""

    def test_mode_not_armable_only_does_not_block(self):
        entries = {
            1: _make_entry(1, mission_uploaded=True),
        }
        entries[1].vehicle.prearm_ok = False
        entries[1].vehicle.prearm_check_state = "failed"
        entries[1].prearm_mode_not_armable_only = True
        mock_mgr = MagicMock()
        mock_mgr.companion_status.return_value = "ok"
        mock_mgr.get_vehicle = MagicMock(side_effect=lambda sid: entries.get(sid))
        mock_mgr.vehicles = entries
        mock_store = _mock_settings_store()
        with (
            patch("gcs.backend.routes.control.vehicle_mgr", mock_mgr),
            patch("gcs.backend.routes.control.settings_store", mock_store),
            patch("gcs.backend.routes.control.launch_controller", MagicMock(
                is_running=False, is_prepared=False,
            )),
        ):
            from gcs.backend.main import app
            client = TestClient(app)
            resp = client.post("/api/control/launch", json={"sys_ids": [1]})
            assert resp.status_code == 200

    def test_real_prearm_failure_still_blocks(self):
        entries = {
            1: _make_entry(1, mission_uploaded=True),
        }
        entries[1].vehicle.prearm_ok = False
        entries[1].vehicle.prearm_check_state = "failed"
        entries[1].prearm_mode_not_armable_only = False
        mock_mgr = MagicMock()
        mock_mgr.companion_status.return_value = "ok"
        mock_mgr.get_vehicle = MagicMock(side_effect=lambda sid: entries.get(sid))
        mock_mgr.vehicles = entries
        mock_store = _mock_settings_store()
        with (
            patch("gcs.backend.routes.control.vehicle_mgr", mock_mgr),
            patch("gcs.backend.routes.control.settings_store", mock_store),
            patch("gcs.backend.routes.control.launch_controller", MagicMock(
                is_running=False, is_prepared=False,
            )),
        ):
            from gcs.backend.main import app
            client = TestClient(app)
            resp = client.post("/api/control/launch", json={"sys_ids": [1]})
            assert resp.status_code == 409
            assert "pre-arm" in resp.json()["detail"].lower()


class TestPrearmCheckStateGating:
    """Launch gating on the fine-grained prearm_check_state."""

    def test_no_sys_status_blocks(self):
        entries = {1: _make_entry(1, mission_uploaded=True)}
        entries[1].vehicle.prearm_check_state = "no_sys_status"
        with _client_for(entries) as client:
            resp = client.post("/api/control/launch", json={"sys_ids": [1]})
            assert resp.status_code == 409
            assert "waiting" in resp.json()["detail"].lower()

    def test_not_reported_blocks(self):
        entries = {1: _make_entry(1, mission_uploaded=True)}
        entries[1].vehicle.prearm_check_state = "not_reported"
        with _client_for(entries) as client:
            resp = client.post("/api/control/launch", json={"sys_ids": [1]})
            assert resp.status_code == 409
            assert "not reported" in resp.json()["detail"].lower()

    def test_checks_disabled_allows_launch(self):
        """The core fix: arming checks disabled is armable, not a blocker."""
        entries = {1: _make_entry(1, mission_uploaded=True)}
        entries[1].vehicle.prearm_check_state = "checks_disabled"
        with _client_for(entries) as client:
            resp = client.post("/api/control/launch", json={"sys_ids": [1]})
            assert resp.status_code == 200

    def test_unknown_state_fails_closed(self):
        """An unexpected/future state must block launch, never allow it."""
        entries = {1: _make_entry(1, mission_uploaded=True)}
        entries[1].vehicle.prearm_check_state = "some_future_state"
        with _client_for(entries) as client:
            resp = client.post("/api/control/launch", json={"sys_ids": [1]})
            assert resp.status_code == 409
            assert "unknown pre-arm" in resp.json()["detail"].lower()

    def test_partial_fleet_disabled_ok_but_sibling_waiting_blocks(self):
        """A checks-disabled vehicle is armable, but a no_sys_status sibling
        still blocks the whole request."""
        entries = {
            1: _make_entry(1, mission_uploaded=True),
            2: _make_entry(2, mission_uploaded=True),
        }
        entries[1].vehicle.prearm_check_state = "checks_disabled"
        entries[2].vehicle.prearm_check_state = "no_sys_status"
        with _client_for(entries) as client:
            resp = client.post("/api/control/launch", json={"sys_ids": [1, 2]})
            assert resp.status_code == 409
            assert "2" in resp.json()["detail"]


class TestTunableReadinessGates:
    """Per-check enable/disable and tunable thresholds on the /launch path."""

    def _run(self, entries, store):
        with (
            patch("gcs.backend.routes.control.vehicle_mgr", _mgr_for(entries)),
            patch("gcs.backend.routes.control.settings_store", store),
            patch("gcs.backend.routes.control.launch_controller", MagicMock(
                is_running=False, is_prepared=False,
            )),
        ):
            from gcs.backend.main import app
            return TestClient(app).post("/api/control/launch", json={"sys_ids": [1]})

    def test_low_battery_blocks_by_default(self):
        entries = {1: _make_entry(1, mission_uploaded=True)}
        entries[1].vehicle.battery_level = 10
        assert self._run(entries, _mock_settings_store()).status_code == 409

    def test_battery_check_disabled_allows_low_battery(self):
        entries = {1: _make_entry(1, mission_uploaded=True)}
        entries[1].vehicle.battery_level = 10
        store = _mock_settings_store(check_battery_enabled=False)
        assert self._run(entries, store).status_code == 200

    def test_battery_threshold_tunable(self):
        entries = {1: _make_entry(1, mission_uploaded=True)}
        entries[1].vehicle.battery_level = 25
        # Raising the floor above the pack level blocks an otherwise-OK vehicle.
        store = _mock_settings_store(min_battery_pct=30.0)
        assert self._run(entries, store).status_code == 409

    def test_throttle_threshold_tunable(self):
        entries = {1: _make_entry(1, mission_uploaded=True)}
        entries[1].vehicle.rc3_raw = 1000  # passes default 1050
        store = _mock_settings_store(max_throttle_rc3=900)
        assert self._run(entries, store).status_code == 409

    def test_gps_check_disabled_allows_no_fix(self):
        entries = {1: _make_entry(1, mission_uploaded=True)}
        entries[1].vehicle.gps_fix_type = 0
        assert self._run(entries, _mock_settings_store()).status_code == 409
        store = _mock_settings_store(check_gps_enabled=False)
        assert self._run(entries, store).status_code == 200

    def test_prearm_check_disabled_allows_failed_prearm(self):
        entries = {1: _make_entry(1, mission_uploaded=True)}
        entries[1].vehicle.prearm_check_state = "failed"
        entries[1].prearm_mode_not_armable_only = False
        assert self._run(entries, _mock_settings_store()).status_code == 409
        store = _mock_settings_store(check_prearm_enabled=False)
        assert self._run(entries, store).status_code == 200

    def test_gps_accuracy_gate_off_by_default(self):
        entries = {1: _make_entry(1, mission_uploaded=True)}
        entries[1].vehicle.gps_hacc = 9.0  # poor accuracy
        # Default: accuracy not checked → launch allowed
        assert self._run(entries, _mock_settings_store()).status_code == 200

    def test_gps_accuracy_gate_blocks_when_enabled(self):
        entries = {1: _make_entry(1, mission_uploaded=True)}
        entries[1].vehicle.gps_hacc = 9.0  # worse than 1.0m limit
        store = _mock_settings_store(check_gps_acc_enabled=True, max_gps_hacc_m=1.0)
        resp = self._run(entries, store)
        assert resp.status_code == 409
        assert "accuracy" in resp.json()["detail"].lower()

    def test_gps_accuracy_gate_passes_good_accuracy(self):
        entries = {1: _make_entry(1, mission_uploaded=True)}
        entries[1].vehicle.gps_hacc = 0.3  # within 1.0m
        store = _mock_settings_store(check_gps_acc_enabled=True, max_gps_hacc_m=1.0)
        assert self._run(entries, store).status_code == 200

    def test_gps_accuracy_unknown_does_not_block(self):
        entries = {1: _make_entry(1, mission_uploaded=True)}
        entries[1].vehicle.gps_hacc = None
        store = _mock_settings_store(check_gps_acc_enabled=True, max_gps_hacc_m=1.0)
        assert self._run(entries, store).status_code == 200

    def test_restart_ignores_gps_accuracy_gate(self):
        """Accuracy gate is launch-only; restart never applies it."""
        entries = {1: _make_entry(1, mission_uploaded=True)}
        entries[1].vehicle.gps_hacc = 9.0
        store = _mock_settings_store(check_gps_acc_enabled=True, max_gps_hacc_m=1.0)
        with (
            patch("gcs.backend.routes.control.vehicle_mgr", _mgr_for(entries)),
            patch("gcs.backend.routes.control.settings_store", store),
            patch("gcs.backend.routes.control.launch_controller", MagicMock(
                is_running=False, is_prepared=False,
            )),
        ):
            from gcs.backend.main import app
            resp = TestClient(app).post("/api/control/restart", json={"sys_ids": [1]})
            assert resp.status_code == 200  # not blocked by accuracy

    def test_block_on_unknown_battery(self):
        entries = {1: _make_entry(1, mission_uploaded=True)}
        entries[1].vehicle.battery_level = None
        # Default: unknown battery passes (current behavior)
        assert self._run(entries, _mock_settings_store()).status_code == 200
        # Opt-in: unknown battery blocks
        store = _mock_settings_store(block_on_unknown_battery=True)
        resp = self._run(entries, store)
        assert resp.status_code == 409
        assert "battery" in resp.json()["detail"].lower()

    def test_restart_ignores_tunable_gates(self):
        """/restart keeps fixed gates: a disabled launch gate must not relax restart."""
        entries = {1: _make_entry(1, mission_uploaded=True)}
        entries[1].vehicle.gps_fix_type = 0  # would fail GPS
        # Even with GPS disabled in settings, restart still enforces GPS.
        store = _mock_settings_store(check_gps_enabled=False)
        with (
            patch("gcs.backend.routes.control.vehicle_mgr", _mgr_for(entries)),
            patch("gcs.backend.routes.control.settings_store", store),
            patch("gcs.backend.routes.control.launch_controller", MagicMock(
                is_running=False, is_prepared=False,
            )),
        ):
            from gcs.backend.main import app
            resp = TestClient(app).post("/api/control/restart", json={"sys_ids": [1]})
            assert resp.status_code == 409
            assert "gps" in resp.json()["detail"].lower()


class TestBungeeTiming:
    """Per-path bungee timing knobs (settle / stagger / arm timeout)."""

    def _launch(self, entries, store, sys_ids):
        with (
            patch("gcs.backend.routes.control.vehicle_mgr", _mgr_for(entries)),
            patch("gcs.backend.routes.control.settings_store", store),
            patch("gcs.backend.routes.control.launch_controller", MagicMock(
                is_running=False, is_prepared=False,
            )),
        ):
            from gcs.backend.main import app
            return TestClient(app).post("/api/control/launch", json={"sys_ids": sys_ids})

    def test_timing_overrides_dont_break_launch(self):
        entries = {
            1: _make_entry(1, mission_uploaded=True),
            2: _make_entry(2, mission_uploaded=True),
        }
        store = _mock_settings_store(
            launch_type="bungee",
            bungee_settle_s=0.01, bungee_stagger_s=0.01, bungee_arm_timeout_s=0.1,
        )
        resp = self._launch(entries, store, [1, 2])
        assert resp.status_code == 200
        data = resp.json()
        assert data["results"]["1"] == "launched"
        assert data["results"]["2"] == "launched"

    def test_unarmed_vehicle_proceeds_after_arm_timeout(self):
        entries = {1: _make_entry(1, mission_uploaded=True)}
        entries[1].vehicle.is_armed = False  # never confirms armed
        store = _mock_settings_store(launch_type="bungee", bungee_arm_timeout_s=0.1)
        resp = self._launch(entries, store, [1])
        assert resp.status_code == 200
        assert resp.json()["results"]["1"] == "launched"


class TestReProbeOnLaunch:
    """When a vehicle has mission items but wasn't verified, re-probe on launch."""

    def test_reprobe_succeeds_allows_launch(self):
        """Vehicle with items on board gets re-probed and launch proceeds."""
        entries = {
            1: _make_entry(1, mission_uploaded=False),  # not verified
        }
        entries[1].vehicle.mission_items_count = 10  # has items on board
        mock_probe = MagicMock()
        mock_probe.valid = True
        mock_mgr = MagicMock()
        mock_mgr.companion_status.return_value = "ok"
        mock_mgr.get_vehicle = MagicMock(side_effect=lambda sid: entries.get(sid))
        mock_mgr.vehicles = entries
        mock_mgr.probe_existing_mission = MagicMock(return_value=mock_probe)
        mock_store = _mock_settings_store()
        with (
            patch("gcs.backend.routes.control.vehicle_mgr", mock_mgr),
            patch("gcs.backend.routes.control.settings_store", mock_store),
            patch("gcs.backend.routes.control.launch_controller", MagicMock(
                is_running=False, is_prepared=False,
            )),
        ):
            from gcs.backend.main import app
            client = TestClient(app)
            resp = client.post("/api/control/launch", json={"sys_ids": [1]})
            assert resp.status_code == 200
            mock_mgr.probe_existing_mission.assert_called_once_with(1)

    def test_reprobe_fails_blocks_launch(self):
        """Vehicle with items on board but invalid probe still blocks launch."""
        entries = {
            1: _make_entry(1, mission_uploaded=False),
        }
        entries[1].vehicle.mission_items_count = 10
        mock_probe = MagicMock()
        mock_probe.valid = False
        mock_mgr = MagicMock()
        mock_mgr.companion_status.return_value = "ok"
        mock_mgr.get_vehicle = MagicMock(side_effect=lambda sid: entries.get(sid))
        mock_mgr.vehicles = entries
        mock_mgr.probe_existing_mission = MagicMock(return_value=mock_probe)
        mock_store = _mock_settings_store()
        with (
            patch("gcs.backend.routes.control.vehicle_mgr", mock_mgr),
            patch("gcs.backend.routes.control.settings_store", mock_store),
            patch("gcs.backend.routes.control.launch_controller", MagicMock(
                is_running=False, is_prepared=False,
            )),
        ):
            from gcs.backend.main import app
            client = TestClient(app)
            resp = client.post("/api/control/launch", json={"sys_ids": [1]})
            assert resp.status_code == 409

    def test_no_reprobe_when_already_verified(self):
        """Vehicle with mission_uploaded=True should NOT re-probe."""
        entries = {
            1: _make_entry(1, mission_uploaded=True),
        }
        mock_mgr = MagicMock()
        mock_mgr.companion_status.return_value = "ok"
        mock_mgr.get_vehicle = MagicMock(side_effect=lambda sid: entries.get(sid))
        mock_mgr.vehicles = entries
        mock_store = _mock_settings_store()
        with (
            patch("gcs.backend.routes.control.vehicle_mgr", mock_mgr),
            patch("gcs.backend.routes.control.settings_store", mock_store),
            patch("gcs.backend.routes.control.launch_controller", MagicMock(
                is_running=False, is_prepared=False,
            )),
        ):
            from gcs.backend.main import app
            client = TestClient(app)
            resp = client.post("/api/control/launch", json={"sys_ids": [1]})
            assert resp.status_code == 200
            mock_mgr.probe_existing_mission.assert_not_called()


class TestProbedMissionLaunch:
    """Tests for launching with probed (pre-existing) missions."""

    def test_probed_mission_allows_launch(self):
        """Vehicle probed with valid mission → launch succeeds."""
        entries = {
            1: _make_entry(1, mission_uploaded=True, probed_search_pattern="distributed"),
            2: _make_entry(2, mission_uploaded=True, probed_search_pattern="distributed"),
        }
        mock_mgr = MagicMock()
        mock_mgr.companion_status.return_value = "ok"
        mock_mgr.get_vehicle = MagicMock(side_effect=lambda sid: entries.get(sid))
        mock_mgr.vehicles = entries
        mock_store = _mock_settings_store()
        with (
            patch("gcs.backend.routes.control.vehicle_mgr", mock_mgr),
            patch("gcs.backend.routes.control.settings_store", mock_store),
            patch("gcs.backend.routes.control.launch_controller", MagicMock(
                is_running=False, is_prepared=False,
            )),
        ):
            from gcs.backend.main import app
            client = TestClient(app)
            resp = client.post("/api/control/launch", json={"sys_ids": [1, 2]})
            assert resp.status_code == 200

    def test_search_pattern_mismatch_blocks_launch(self):
        """Vehicles with different probed search_patterns → launch blocked 409."""
        entries = {
            1: _make_entry(1, mission_uploaded=True, probed_search_pattern="distributed"),
            2: _make_entry(2, mission_uploaded=True, probed_search_pattern="corridor"),
        }
        mock_mgr = MagicMock()
        mock_mgr.companion_status.return_value = "ok"
        mock_mgr.get_vehicle = MagicMock(side_effect=lambda sid: entries.get(sid))
        mock_mgr.vehicles = entries
        mock_store = _mock_settings_store()
        with (
            patch("gcs.backend.routes.control.vehicle_mgr", mock_mgr),
            patch("gcs.backend.routes.control.settings_store", mock_store),
            patch("gcs.backend.routes.control.launch_controller", MagicMock(
                is_running=False, is_prepared=False,
            )),
        ):
            from gcs.backend.main import app
            client = TestClient(app)
            resp = client.post("/api/control/launch", json={"sys_ids": [1, 2]})
            assert resp.status_code == 409
            assert "mismatch" in resp.json()["detail"].lower()
