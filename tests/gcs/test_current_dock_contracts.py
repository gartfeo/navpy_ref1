"""Current GCS JSON names; no old-file adapters or real vehicle I/O."""
import json
import pytest
from types import SimpleNamespace
from unittest.mock import patch
from gcs.backend.settings_store import SettingsStore
from tests.gcs.test_settings import client


def test_dock_settings_round_trip(client):
    fallbackLocations = [{"name": "Review dock", "type": "other", "lat": 0.01, "lon": 0.02}]
    response = client.put("/api/settings", json={"fallback_delivery_locations": fallbackLocations})
    assert response.status_code == 200
    assert response.json()["fallback_delivery_locations"] == fallbackLocations
    assert "objects_of_interest" not in response.json()
    assert client.get("/api/settings").json()["fallback_delivery_locations"] == fallbackLocations


def test_dock_settings_survive_store_reload(tmp_path):
    path = tmp_path / "settings.json"
    store = SettingsStore(path=path)
    fallbackLocations = [{"name": "Review dock", "type": "vehicle", "lat": 0.01, "lon": 0.02}]
    store.update({"fallback_delivery_locations": fallbackLocations})
    assert SettingsStore(path=path).get().model_dump()["fallback_delivery_locations"] == fallbackLocations
    assert "objects_of_interest" not in path.read_text()


def test_unknown_settings_put_is_rejected_without_mutation(client):
    before = client.get("/api/settings").json()
    response = client.put("/api/settings", json={"unrecognized_field": [], "flight": {"cruise_speed_ms": 33}})
    assert response.status_code == 422
    assert client.get("/api/settings").json() == before


def test_current_csv_route_preserves_values(client):
    response = client.post("/api/settings/fallback-delivery-locations/import", files={"file": ("fallbackLocations.csv", b"name,type,lat,lon\nReview dock,vehicle,0.01,0.02\n", "text/csv")})
    assert response.status_code == 200
    assert client.get("/api/settings").json()["fallback_delivery_locations"] == [{"name": "Review dock", "type": "vehicle", "lat": 0.01, "lon": 0.02}]


@pytest.mark.parametrize("extra", [
    {"objects_of_interest": [{"name": "Saved location", "type": "vehicle", "lat": 1, "lon": 2}]},
    {"aas": None, "objects_of_interest": []},
    {"delivery_docks": [{"name": "Saved location", "type": "vehicle", "lat": 1, "lon": 2}]},
    {"unrecognized_field": True},
])
def test_unknown_settings_file_fields_fail_without_rewriting(tmp_path, extra):
    path = tmp_path / "settings.json"
    text = json.dumps({"flight": {"cruise_speed_ms": 28},
                       "connection": {"auto_connect": False}, **extra})
    path.write_text(text)
    before = path.read_bytes()
    with pytest.raises(ValueError) as error:
        SettingsStore(path=path)
    assert str(path) in str(error.value)
    assert next(key for key in extra if key != "aas") in str(error.value)
    assert path.read_bytes() == before


def test_current_settings_file_preserves_all_fields(tmp_path):
    path = tmp_path / "settings.json"
    store = SettingsStore(path=path)
    expected = store.update({"fallback_delivery_locations": [{"name": "Saved dock", "type": "vehicle", "lat": 1, "lon": 2}],
                             "flight": {"cruise_speed_ms": 28}, "connection": {"auto_connect": False}})
    before = path.read_bytes()
    assert SettingsStore(path=path).get().model_dump() == expected.model_dump()
    assert path.read_bytes() == before


def test_config_reads_unchanged_navpy_profile_format():
    from gcs.backend.config import _resolve_presets
    presets = {"small": {"altitude_m": 211, "min_pixel_size": 43, "label": "Fixture"}}
    with patch("navpy.modules.vision.vision_profiles.load_profiles",
               return_value=({"fixture": {"detector": {"dock_presets": presets}}}, "fixture", None)):
        assert _resolve_presets(SimpleNamespace(vision_profile="fixture")) == presets
