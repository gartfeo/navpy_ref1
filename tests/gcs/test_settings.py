"""Integration tests for settings API routes."""
import json
import tempfile
from pathlib import Path
from unittest.mock import patch

import pytest
from fastapi.testclient import TestClient

from gcs.backend.settings_store import SettingsStore


@pytest.fixture
def client(tmp_path):
    """Create a test client with an isolated settings store."""
    store = SettingsStore(path=tmp_path / "test_settings.json")
    with patch("gcs.backend.routes.settings.settings_store", store):
        from gcs.backend.main import app
        yield TestClient(app)


class TestSettingsAPI:
    def test_get_returns_defaults(self, client):
        resp = client.get("/api/settings")
        assert resp.status_code == 200
        data = resp.json()
        assert data["flight"]["cruise_speed_ms"] == 22.0
        # Camera DATA (pitch, dock_presets) now lives solely in the vision
        # profile (vision_profiles.json); the settings camera block holds only
        # the active selection. Single-source-of-truth consolidation.
        assert "pitch_deg" not in data["camera"]
        assert "dock_presets" not in data["camera"]
        assert set(data["camera"]) == {"vision_profile", "vision_device", "vision_zoom"}
        # Demo (sim) + dev are enabled by default now.
        assert data["simulation"]["sim_mode"] is True
        # AAS parameters are vehicle-owned; the GCS no longer carries them.
        assert "aas" not in data

    def test_put_updates_field(self, client):
        resp = client.put(
            "/api/settings",
            json={"flight": {"cruise_speed_ms": 25.0}},
        )
        assert resp.status_code == 200
        assert resp.json()["flight"]["cruise_speed_ms"] == 25.0

    def test_put_bad_typed_field_returns_422_not_500(self, client):
        # Regression (issue #102): an un-coercible bad-typed field must yield a
        # graceful 422, not a 500 crash (ValidationError used to propagate uncaught).
        resp = client.put("/api/settings", json={"flight": {"cruise_speed_ms": "abc"}})
        assert resp.status_code == 422, resp.text
        detail = resp.json()["detail"]
        assert any("cruise_speed_ms" in str(err.get("loc", "")) for err in detail)

    def test_put_systemic_bad_types_all_422(self, client):
        # The 500 was systemic across un-coercible fields — all must be 422 now.
        for body in (
            {"flight": {"cruise_speed_ms": "abc"}},
            {"map_display": {"default_zoom": "far"}},
            {"fallback_delivery_locations": "not-a-list"},
        ):
            resp = client.put("/api/settings", json=body)
            assert resp.status_code == 422, (body, resp.text)

    def test_put_bad_type_leaves_settings_unchanged(self, client):
        # A rejected (422) update must not corrupt previously-saved settings.
        client.put("/api/settings", json={"flight": {"cruise_speed_ms": 25.0}})
        bad = client.put("/api/settings", json={"map_display": {"default_zoom": "far"}})
        assert bad.status_code == 422
        after = client.get("/api/settings").json()
        assert after["flight"]["cruise_speed_ms"] == 25.0

    def test_put_preserves_other_fields(self, client):
        client.put("/api/settings", json={"flight": {"cruise_speed_ms": 25.0}})
        client.put("/api/settings", json={"flight": {"flight_budget_km": 100.0}})
        resp = client.get("/api/settings")
        data = resp.json()
        assert data["flight"]["cruise_speed_ms"] == 25.0
        assert data["flight"]["flight_budget_km"] == 100.0
        assert data["flight"]["safety_reserve_km"] == 20.0

    def test_reset_restores_defaults(self, client):
        client.put("/api/settings", json={"flight": {"cruise_speed_ms": 99.0}})
        resp = client.post("/api/settings/reset")
        assert resp.status_code == 200
        assert resp.json()["flight"]["cruise_speed_ms"] == 22.0

    def test_update_sim_mode(self, client):
        resp = client.put(
            "/api/settings",
            json={"simulation": {"sim_mode": True}},
        )
        assert resp.status_code == 200
        assert resp.json()["simulation"]["sim_mode"] is True

    def test_profile_change_restarts_running_navpy_when_sim_mode_enabled(self, client):
        with patch("gcs.backend.navpy_sim_runtime.restart_running_navpy_sim") as mock_restart:
            resp = client.put(
                "/api/settings",
                json={
                    "simulation": {"sim_mode": True},
                    "camera": {"vision_profile": "novoxy_dual"},
                },
            )

        assert resp.status_code == 200
        mock_restart.assert_called_once()
        settings_arg = mock_restart.call_args.args[0]
        assert settings_arg.camera.vision_profile == "novoxy_dual"

    def test_profile_change_does_not_restart_navpy_when_sim_mode_disabled(self, client):
        # sim_mode is on by default now, so disable it explicitly to exercise
        # the "no restart when sim mode is off" path.
        with patch("gcs.backend.navpy_sim_runtime.restart_running_navpy_sim") as mock_restart:
            resp = client.put(
                "/api/settings",
                json={"simulation": {"sim_mode": False},
                      "camera": {"vision_profile": "novoxy_dual"}},
            )

        assert resp.status_code == 200
        mock_restart.assert_not_called()

    def test_unrelated_settings_change_does_not_restart_navpy(self, client):
        with patch("gcs.backend.navpy_sim_runtime.restart_running_navpy_sim"):
            client.put(
                "/api/settings",
                json={
                    "simulation": {"sim_mode": True},
                    "camera": {"vision_profile": "novoxy_dual"},
                },
            )

        with patch("gcs.backend.navpy_sim_runtime.restart_running_navpy_sim") as mock_restart:
            resp = client.put(
                "/api/settings",
                json={"flight": {"cruise_speed_ms": 25.0}},
            )

        assert resp.status_code == 200
        mock_restart.assert_not_called()

    def test_dev_mode_defaults_true(self, client):
        # Demo + dev are enabled by default now.
        resp = client.get("/api/settings")
        assert resp.json()["simulation"]["dev_mode"] is True

    def test_update_dev_mode(self, client):
        resp = client.put(
            "/api/settings",
            json={"simulation": {"dev_mode": True}},
        )
        assert resp.status_code == 200
        assert resp.json()["simulation"]["dev_mode"] is True

    def test_removed_aas_payload_is_rejected(self, client):
        before = client.get("/api/settings").json()
        response = client.put("/api/settings", json={"aas": {"del_pitch": 10.0}})
        assert response.status_code == 422
        assert client.get("/api/settings").json() == before
