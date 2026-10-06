"""Tests for the /api/vision-profiles endpoint."""
import pytest
from fastapi.testclient import TestClient
from unittest.mock import MagicMock, patch

from gcs.backend.main import app


@pytest.fixture
def client():
    return TestClient(app)


def _mock_gcs_settings(sim_mode=True, vision_profile=None):
    settings = MagicMock()
    settings.simulation.sim_mode = sim_mode
    settings.camera.vision_profile = vision_profile
    return settings


class TestVisionProfilesRoute:
    def test_returns_profiles_with_default(self, client):
        resp = client.get("/api/vision-profiles")
        assert resp.status_code == 200
        data = resp.json()
        assert "default_profile" in data
        assert "profiles" in data
        assert data["default_profile"] in data["profiles"]

    def test_profile_has_devices(self, client):
        data = client.get("/api/vision-profiles").json()
        for name, profile in data["profiles"].items():
            assert "devices" in profile
            assert isinstance(profile["devices"], list)
            assert len(profile["devices"]) > 0

    def test_device_has_required_fields(self, client):
        data = client.get("/api/vision-profiles").json()
        for profile in data["profiles"].values():
            for device in profile["devices"]:
                assert "name" in device
                assert "gimbal_device_id" in device
                assert "publishes_gimbal_telemetry" in device
                assert "image_width" in device
                assert "image_height" in device
                assert "pitch_deg" in device
                assert "zooms" in device
                assert "setup_att" in device
                assert "setup_seq" in device
                assert "gimbal_seq" in device

    def test_zoom_has_ranges_and_no_extra_intrinsics(self, client):
        data = client.get("/api/vision-profiles").json()
        for profile in data["profiles"].values():
            for device in profile["devices"]:
                for level, zoom in device["zooms"].items():
                    assert "fx" in zoom
                    assert "fy" in zoom
                    assert "detect_range_m" in zoom
                    assert "confirm_range_m" in zoom
                    assert zoom["detect_range_m"] > 0
                    assert zoom["confirm_range_m"] > 0
                    assert zoom["detect_range_m"] > zoom["confirm_range_m"]
                    # Should NOT leak cx/cy/dist/skew
                    assert "cx" not in zoom
                    assert "cy" not in zoom
                    assert "dist" not in zoom
                    assert "skew" not in zoom

    def test_known_profile_values(self, client):
        """Verify known values from novoxy_dual profile."""
        data = client.get("/api/vision-profiles").json()
        novoxy = data["profiles"]["novoxy_dual"]
        down = next(d for d in novoxy["devices"] if d["name"] == "novoxy_18_down")
        assert down["image_width"] == 1920
        assert down["image_height"] == 1080
        assert isinstance(down["pitch_deg"], (int, float))
        assert abs(down["zooms"]["1"]["fx"] - 2252.628) < 0.01

    def test_known_gimbal_contract_metadata(self, client):
        data = client.get("/api/vision-profiles").json()
        siyi = data["profiles"]["siyi_zr10"]["devices"][0]
        assert siyi["gimbal_device_id"] == 1
        assert siyi["setup_att"] == [90, 0, 90]
        assert siyi["setup_seq"] == "XYZ"
        assert siyi["gimbal_seq"] == "XYZ"

    def test_multi_device_ids_use_profile_order(self, client):
        data = client.get("/api/vision-profiles").json()
        devices = data["profiles"]["novoxy_dual"]["devices"]
        assert [device["gimbal_device_id"] for device in devices] == [1, 2]

    def test_explicit_gimbal_device_id_override(self):
        from pathlib import Path
        from unittest.mock import patch

        data = {
            "default_profile": "explicit",
            "profiles": {
                "explicit": {
                    "detector": {"reference_height_m": 2.0, "imgsz": 640},
                    "devices": [{
                        "name": "cam",
                        "camera": {
                            "image_width": 1920,
                            "image_height": 1080,
                            "intrinsics": {"zooms": {"1": {"fx": 2000, "fy": 2000}}},
                        },
                        "gimbal": {"camera_pitch": -15, "gimbal_device_id": 4},
                    }],
                },
            },
        }

        def mock_load():
            return data["profiles"], data["default_profile"], Path("dummy")

        with patch("gcs.backend.routes.vision_profiles.load_profiles", mock_load):
            c = TestClient(app)
            device = c.get("/api/vision-profiles").json()["profiles"]["explicit"]["devices"][0]
            assert device["gimbal_device_id"] == 4
            assert device["setup_att"] == [90, 0, 90]

    def test_rejects_out_of_range_gimbal_device_id(self):
        from pathlib import Path
        from unittest.mock import patch

        data = {
            "default_profile": "invalid",
            "profiles": {
                "invalid": {
                    "detector": {"reference_height_m": 2.0, "imgsz": 640},
                    "devices": [{
                        "name": "cam",
                        "camera": {
                            "image_width": 1920,
                            "image_height": 1080,
                            "intrinsics": {"zooms": {"1": {"fx": 2000, "fy": 2000}}},
                        },
                        "gimbal": {"camera_pitch": -15, "gimbal_device_id": 7},
                    }],
                },
            },
        }

        def mock_load():
            return data["profiles"], data["default_profile"], Path("dummy")

        with patch("gcs.backend.routes.vision_profiles.load_profiles", mock_load):
            c = TestClient(app)
            with pytest.raises(ValueError, match="gimbal_device_id must be in range"):
                c.get("/api/vision-profiles")

    def test_rejects_duplicate_gimbal_device_id(self):
        from pathlib import Path
        from unittest.mock import patch

        camera = {
            "image_width": 1920,
            "image_height": 1080,
            "intrinsics": {"zooms": {"1": {"fx": 2000, "fy": 2000}}},
        }
        data = {
            "default_profile": "duplicate",
            "profiles": {
                "duplicate": {
                    "detector": {"reference_height_m": 2.0, "imgsz": 640},
                    "devices": [
                        {
                            "name": "cam_a",
                            "camera": camera,
                            "gimbal": {"camera_pitch": -15, "gimbal_device_id": 2},
                        },
                        {
                            "name": "cam_b",
                            "camera": camera,
                            "gimbal": {"camera_pitch": -25, "gimbal_device_id": 2},
                        },
                    ],
                },
            },
        }

        def mock_load():
            return data["profiles"], data["default_profile"], Path("dummy")

        with patch("gcs.backend.routes.vision_profiles.load_profiles", mock_load):
            c = TestClient(app)
            with pytest.raises(ValueError, match="duplicate gimbal_device_id 2"):
                c.get("/api/vision-profiles")

    def test_profile_has_reference_height_m(self, client):
        """Each profile exposes reference_height_m from detector settings."""
        data = client.get("/api/vision-profiles").json()
        for name, profile in data["profiles"].items():
            assert "reference_height_m" in profile
            assert isinstance(profile["reference_height_m"], (int, float))
            assert profile["reference_height_m"] > 0

    def test_profile_has_imgsz(self, client):
        """Each profile exposes imgsz from detector settings."""
        data = client.get("/api/vision-profiles").json()
        for name, profile in data["profiles"].items():
            assert "imgsz" in profile
            assert isinstance(profile["imgsz"], int)
            assert profile["imgsz"] > 0


class TestVisionProfileSettingsFields:
    """Verify vision_profile/device/zoom fields persist via settings."""

    @pytest.fixture
    def settings_client(self, tmp_path):
        from unittest.mock import patch
        from gcs.backend.settings_store import SettingsStore
        store = SettingsStore(path=tmp_path / "test_settings.json")
        with patch("gcs.backend.routes.settings.settings_store", store):
            yield TestClient(app)

    def test_defaults_to_none(self, settings_client):
        resp = settings_client.get("/api/settings")
        assert resp.status_code == 200
        cam = resp.json()["camera"]
        assert cam["vision_profile"] is None
        assert cam["vision_device"] is None
        assert cam["vision_zoom"] is None

    def test_persist_profile_selection(self, settings_client):
        resp = settings_client.put(
            "/api/settings",
            json={"camera": {
                "vision_profile": "novoxy_dual",
                "vision_device": "novoxy_18_down",
                "vision_zoom": "1",
            }},
        )
        assert resp.status_code == 200
        cam = resp.json()["camera"]
        assert cam["vision_profile"] == "novoxy_dual"
        assert cam["vision_device"] == "novoxy_18_down"
        assert cam["vision_zoom"] == "1"

        # Verify persistence across reads
        cam2 = settings_client.get("/api/settings").json()["camera"]
        assert cam2["vision_profile"] == "novoxy_dual"


class TestSetDefaultProfile:
    """Tests for PUT /api/vision-profiles/default/{profile}."""

    @pytest.fixture
    def profiles_file(self, tmp_path):
        import json
        from unittest.mock import patch

        data = {
            "default_profile": "profile_a",
            "profiles": {
                "profile_a": {
                    "detector": {"reference_height_m": 2.0, "imgsz": 640},
                    "devices": [{"name": "cam_a", "camera": {
                        "image_width": 1920, "image_height": 1080,
                        "intrinsics": {"zooms": {"1": {"fx": 2000, "fy": 2000, "cx": 960, "cy": 540, "skew": 0, "dist": []}}},
                    }, "gimbal": {"camera_pitch": -35}}],
                },
                "profile_b": {
                    "detector": {"reference_height_m": 1.5, "imgsz": 640},
                    "devices": [{"name": "cam_b", "camera": {
                        "image_width": 1280, "image_height": 720,
                        "intrinsics": {"zooms": {"1": {"fx": 1500, "fy": 1500, "cx": 640, "cy": 360, "skew": 0, "dist": []}}},
                    }, "gimbal": {"camera_pitch": -45}}],
                },
            },
        }
        path = tmp_path / "vision_profiles.json"
        path.write_text(json.dumps(data))

        def mock_load():
            d = json.loads(path.read_text())
            return d.get("profiles", {}), d.get("default_profile", ""), path

        with patch("gcs.backend.routes.vision_profiles.load_profiles", mock_load):
            yield path, TestClient(app)

    def test_sets_default_profile(self, profiles_file):
        path, client = profiles_file
        resp = client.put("/api/vision-profiles/default/profile_b")
        assert resp.status_code == 200
        import json
        data = json.loads(path.read_text())
        assert data["default_profile"] == "profile_b"

    def test_returns_updated_catalog(self, profiles_file):
        _, client = profiles_file
        resp = client.put("/api/vision-profiles/default/profile_b")
        catalog = resp.json()
        assert catalog["default_profile"] == "profile_b"

    def test_default_profile_change_restarts_navpy_when_default_is_active(self, profiles_file):
        _, client = profiles_file
        settings = _mock_gcs_settings(sim_mode=True, vision_profile=None)

        with (
            patch("gcs.backend.routes.vision_profiles.settings_store.get", return_value=settings),
            patch("gcs.backend.navpy_sim_runtime.restart_running_navpy_sim") as mock_restart,
        ):
            resp = client.put("/api/vision-profiles/default/profile_b")

        assert resp.status_code == 200
        mock_restart.assert_called_once_with(settings)

    def test_404_unknown_profile(self, profiles_file):
        _, client = profiles_file
        resp = client.put("/api/vision-profiles/default/nonexistent")
        assert resp.status_code == 404


class TestUpdateDeviceParams:
    """Tests for PUT /api/vision-profiles/{profile}/devices/{device}."""

    @pytest.fixture
    def profiles_file(self, tmp_path):
        """Create a temporary vision_profiles.json for mutation tests."""
        import json
        from unittest.mock import patch

        data = {
            "default_profile": "test_profile",
            "profiles": {
                "test_profile": {
                    "detector": {"reference_height_m": 2.0, "imgsz": 640},
                    "devices": [{
                        "name": "test_cam",
                        "camera": {
                            "image_width": 1920,
                            "image_height": 1080,
                            "intrinsics": {
                                "zooms": {"1": {"fx": 2000.0, "fy": 2000.0, "cx": 960, "cy": 540, "skew": 0, "dist": []}}
                            },
                        },
                        "gimbal": {"camera_pitch": -35},
                    }],
                }
            },
        }
        path = tmp_path / "vision_profiles.json"
        path.write_text(json.dumps(data))

        def mock_load():
            d = json.loads(path.read_text())
            return d.get("profiles", {}), d.get("default_profile", ""), path

        with patch("gcs.backend.routes.vision_profiles.load_profiles", mock_load):
            yield path, TestClient(app)

    def test_update_pitch(self, profiles_file):
        path, client = profiles_file
        resp = client.put(
            "/api/vision-profiles/test_profile/devices/test_cam",
            json={"zoom": "1", "pitch_deg": -45},
        )
        assert resp.status_code == 200
        import json
        data = json.loads(path.read_text())
        assert data["profiles"]["test_profile"]["devices"][0]["gimbal"]["camera_pitch"] == -45

    def test_update_fx_fy(self, profiles_file):
        path, client = profiles_file
        resp = client.put(
            "/api/vision-profiles/test_profile/devices/test_cam",
            json={"zoom": "1", "fx": 2500.0, "fy": 2600.0},
        )
        assert resp.status_code == 200
        import json
        zoom = json.loads(path.read_text())["profiles"]["test_profile"]["devices"][0]["camera"]["intrinsics"]["zooms"]["1"]
        assert zoom["fx"] == 2500.0
        assert zoom["fy"] == 2600.0

    def test_update_image_dims(self, profiles_file):
        path, client = profiles_file
        resp = client.put(
            "/api/vision-profiles/test_profile/devices/test_cam",
            json={"zoom": "1", "image_width": 1280, "image_height": 720},
        )
        assert resp.status_code == 200
        import json
        cam = json.loads(path.read_text())["profiles"]["test_profile"]["devices"][0]["camera"]
        assert cam["image_width"] == 1280
        assert cam["image_height"] == 720

    def test_returns_updated_catalog(self, profiles_file):
        _, client = profiles_file
        resp = client.put(
            "/api/vision-profiles/test_profile/devices/test_cam",
            json={"zoom": "1", "pitch_deg": -20},
        )
        catalog = resp.json()
        dev = catalog["profiles"]["test_profile"]["devices"][0]
        assert dev["pitch_deg"] == -20

    def test_active_profile_device_edit_restarts_navpy_sim(self, profiles_file):
        _, client = profiles_file
        settings = _mock_gcs_settings(sim_mode=True, vision_profile="test_profile")

        with (
            patch("gcs.backend.routes.vision_profiles.settings_store.get", return_value=settings),
            patch("gcs.backend.navpy_sim_runtime.restart_running_navpy_sim") as mock_restart,
        ):
            resp = client.put(
                "/api/vision-profiles/test_profile/devices/test_cam",
                json={"zoom": "1", "pitch_deg": -20},
            )

        assert resp.status_code == 200
        mock_restart.assert_called_once_with(settings)

    def test_inactive_profile_device_edit_does_not_restart_navpy_sim(self, profiles_file):
        _, client = profiles_file
        settings = _mock_gcs_settings(sim_mode=True, vision_profile="other_profile")

        with (
            patch("gcs.backend.routes.vision_profiles.settings_store.get", return_value=settings),
            patch("gcs.backend.navpy_sim_runtime.restart_running_navpy_sim") as mock_restart,
        ):
            resp = client.put(
                "/api/vision-profiles/test_profile/devices/test_cam",
                json={"zoom": "1", "pitch_deg": -20},
            )

        assert resp.status_code == 200
        mock_restart.assert_not_called()

    def test_404_unknown_profile(self, profiles_file):
        _, client = profiles_file
        resp = client.put(
            "/api/vision-profiles/nonexistent/devices/test_cam",
            json={"zoom": "1", "pitch_deg": -10},
        )
        assert resp.status_code == 404

    def test_404_unknown_device(self, profiles_file):
        _, client = profiles_file
        resp = client.put(
            "/api/vision-profiles/test_profile/devices/nonexistent",
            json={"zoom": "1", "pitch_deg": -10},
        )
        assert resp.status_code == 404

    def test_404_unknown_zoom(self, profiles_file):
        _, client = profiles_file
        resp = client.put(
            "/api/vision-profiles/test_profile/devices/test_cam",
            json={"zoom": "99", "fx": 3000.0},
        )
        assert resp.status_code == 404


class TestProfilePitchEnvelope:
    """Tests for min_pitch / max_pitch at profile (detector) level."""

    def test_catalog_exposes_min_max_pitch(self, client):
        """Each profile in catalog should have min_pitch, max_pitch, min_altitude."""
        data = client.get("/api/vision-profiles").json()
        for name, profile in data["profiles"].items():
            assert "min_pitch" in profile, f"{name} missing min_pitch"
            assert "max_pitch" in profile, f"{name} missing max_pitch"
            assert "min_altitude" in profile, f"{name} missing min_altitude"
            assert isinstance(profile["min_pitch"], (int, float))
            assert isinstance(profile["max_pitch"], (int, float))
            assert isinstance(profile["min_altitude"], (int, float))
            assert profile["min_pitch"] < profile["max_pitch"]
            assert profile["min_altitude"] > 0

    def test_known_profile_pitch_envelope(self, client):
        """Verify known values from novoxy_dual profile."""
        data = client.get("/api/vision-profiles").json()
        novoxy = data["profiles"]["novoxy_dual"]
        assert novoxy["min_pitch"] == -60
        assert novoxy["max_pitch"] == 20
        assert novoxy["min_altitude"] == 95

    def test_defaults_when_missing(self):
        """Profiles without explicit min/max should get defaults."""
        import json
        from unittest.mock import patch
        from pathlib import Path

        data = {
            "default_profile": "bare",
            "profiles": {
                "bare": {
                    "detector": {"reference_height_m": 2.0, "imgsz": 640},
                    "devices": [{"name": "cam", "camera": {
                        "image_width": 1920, "image_height": 1080,
                        "intrinsics": {"zooms": {"1": {"fx": 2000, "fy": 2000, "cx": 960, "cy": 540, "skew": 0, "dist": []}}},
                    }, "gimbal": {"camera_pitch": -35}}],
                },
            },
        }

        def mock_load():
            return data["profiles"], data["default_profile"], Path("dummy")

        with patch("gcs.backend.routes.vision_profiles.load_profiles", mock_load):
            c = TestClient(app)
            resp = c.get("/api/vision-profiles")
            profile = resp.json()["profiles"]["bare"]
            assert profile["min_pitch"] == -60
            assert profile["max_pitch"] == 20
            assert profile["min_altitude"] == 30


class TestUpdateProfileParams:
    """Tests for PUT /api/vision-profiles/{profile} (profile-level params)."""

    @pytest.fixture
    def profiles_file(self, tmp_path):
        import json
        from unittest.mock import patch

        data = {
            "default_profile": "test_profile",
            "profiles": {
                "test_profile": {
                    "detector": {"reference_height_m": 2.0, "imgsz": 640,
                                 "min_pitch": -60, "max_pitch": 20},
                    "devices": [{"name": "cam", "camera": {
                        "image_width": 1920, "image_height": 1080,
                        "intrinsics": {"zooms": {"1": {"fx": 2000, "fy": 2000, "cx": 960, "cy": 540, "skew": 0, "dist": []}}},
                    }, "gimbal": {"camera_pitch": -35}}],
                },
            },
        }
        path = tmp_path / "vision_profiles.json"
        path.write_text(json.dumps(data))

        def mock_load():
            d = json.loads(path.read_text())
            return d.get("profiles", {}), d.get("default_profile", ""), path

        with patch("gcs.backend.routes.vision_profiles.load_profiles", mock_load):
            yield path, TestClient(app)

    def test_update_min_pitch(self, profiles_file):
        path, client = profiles_file
        resp = client.put(
            "/api/vision-profiles/test_profile",
            json={"min_pitch": -70},
        )
        assert resp.status_code == 200
        import json
        data = json.loads(path.read_text())
        assert data["profiles"]["test_profile"]["detector"]["min_pitch"] == -70

    def test_update_max_pitch(self, profiles_file):
        path, client = profiles_file
        resp = client.put(
            "/api/vision-profiles/test_profile",
            json={"max_pitch": 30},
        )
        assert resp.status_code == 200
        import json
        data = json.loads(path.read_text())
        assert data["profiles"]["test_profile"]["detector"]["max_pitch"] == 30

    def test_update_both(self, profiles_file):
        path, client = profiles_file
        resp = client.put(
            "/api/vision-profiles/test_profile",
            json={"min_pitch": -80, "max_pitch": 10},
        )
        assert resp.status_code == 200
        catalog = resp.json()
        profile = catalog["profiles"]["test_profile"]
        assert profile["min_pitch"] == -80
        assert profile["max_pitch"] == 10

    def test_returns_updated_catalog(self, profiles_file):
        _, client = profiles_file
        resp = client.put(
            "/api/vision-profiles/test_profile",
            json={"min_pitch": -50},
        )
        catalog = resp.json()
        assert catalog["profiles"]["test_profile"]["min_pitch"] == -50

    def test_active_profile_detector_edit_restarts_navpy_sim(self, profiles_file):
        _, client = profiles_file
        settings = _mock_gcs_settings(sim_mode=True, vision_profile="test_profile")

        with (
            patch("gcs.backend.routes.vision_profiles.settings_store.get", return_value=settings),
            patch("gcs.backend.navpy_sim_runtime.restart_running_navpy_sim") as mock_restart,
        ):
            resp = client.put(
                "/api/vision-profiles/test_profile",
                json={"min_pitch": -50},
            )

        assert resp.status_code == 200
        mock_restart.assert_called_once_with(settings)

    def test_update_min_altitude(self, profiles_file):
        path, client = profiles_file
        resp = client.put(
            "/api/vision-profiles/test_profile",
            json={"min_altitude": 80},
        )
        assert resp.status_code == 200
        import json
        data = json.loads(path.read_text())
        assert data["profiles"]["test_profile"]["detector"]["min_altitude"] == 80
        catalog = resp.json()
        assert catalog["profiles"]["test_profile"]["min_altitude"] == 80

    def test_404_unknown_profile(self, profiles_file):
        _, client = profiles_file
        resp = client.put(
            "/api/vision-profiles/nonexistent",
            json={"min_pitch": -50},
        )
        assert resp.status_code == 404
