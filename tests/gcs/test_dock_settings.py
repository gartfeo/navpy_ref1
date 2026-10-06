"""Tests for DOCK settings model and CSV import endpoint."""
import io
from unittest.mock import patch

import pytest
from fastapi.testclient import TestClient

from gcs.backend.settings_model import FallbackDeliveryLocation, DELIVERY_LOCATION_TYPES, GcsSettings
from gcs.backend.settings_store import SettingsStore


class TestFallbackDeliveryLocation:
    """Unit tests for FallbackDeliveryLocation Pydantic model."""

    def test_defaults(self):
        entry = FallbackDeliveryLocation(name="HQ", lat=40.0, lon=44.0)
        assert entry.name == "HQ"
        assert entry.type == "other"
        assert entry.lat == 40.0
        assert entry.lon == 44.0

    def test_with_type(self):
        entry = FallbackDeliveryLocation(name="Radar", type="antenna", lat=40.1, lon=44.1)
        assert entry.type == "antenna"

    def test_all_types_valid(self):
        for t in DELIVERY_LOCATION_TYPES:
            entry = FallbackDeliveryLocation(name=f"test_{t}", type=t, lat=0.0, lon=0.0)
            assert entry.type == t

    def test_serialization(self):
        entry = FallbackDeliveryLocation(name="Bridge", type="bridge", lat=40.2, lon=44.2)
        d = entry.model_dump()
        assert d == {"name": "Bridge", "type": "bridge", "lat": 40.2, "lon": 44.2}

    def test_from_dict(self):
        entry = FallbackDeliveryLocation(**{"name": "Fuel", "type": "fuel", "lat": 40.3, "lon": 44.3})
        assert entry.name == "Fuel"
        assert entry.type == "fuel"


class TestGcsSettingsDock:
    """DOCK list in GcsSettings."""

    def test_default_empty(self):
        s = GcsSettings()
        assert s.fallback_delivery_locations == []

    def test_with_docks(self):
        s = GcsSettings(fallback_delivery_locations=[
            FallbackDeliveryLocation(name="A", lat=40.0, lon=44.0),
            FallbackDeliveryLocation(name="B", type="building", lat=40.1, lon=44.1),
        ])
        assert len(s.fallback_delivery_locations) == 2
        assert s.fallback_delivery_locations[0].name == "A"
        assert s.fallback_delivery_locations[1].type == "building"

    def test_round_trip(self):
        s = GcsSettings(fallback_delivery_locations=[
            FallbackDeliveryLocation(name="C", type="operations_site", lat=40.2, lon=44.2),
        ])
        d = s.model_dump()
        s2 = GcsSettings(**d)
        assert len(s2.fallback_delivery_locations) == 1
        assert s2.fallback_delivery_locations[0].name == "C"
        assert s2.fallback_delivery_locations[0].type == "operations_site"


@pytest.fixture
def client(tmp_path):
    """Create a test client with an isolated settings store."""
    store = SettingsStore(path=tmp_path / "test_settings.json")
    with patch("gcs.backend.routes.settings.settings_store", store):
        from gcs.backend.main import app
        yield TestClient(app)


class TestDockCsvImport:
    """Tests for POST /api/settings/fallback-delivery-locations/import."""

    def test_import_basic_csv(self, client):
        csv_content = "name,type,lat,lon\nHQ,building,40.0,44.0\nRadar,antenna,40.1,44.1\n"
        resp = client.post(
            "/api/settings/fallback-delivery-locations/import",
            files={"file": ("fallbackLocations.csv", io.BytesIO(csv_content.encode()), "text/csv")},
        )
        assert resp.status_code == 200
        data = resp.json()
        fallbackLocations = data["fallback_delivery_locations"]
        assert len(fallbackLocations) == 2
        assert fallbackLocations[0]["name"] == "HQ"
        assert fallbackLocations[0]["type"] == "building"
        assert fallbackLocations[1]["name"] == "Radar"
        assert fallbackLocations[1]["type"] == "antenna"

    def test_import_unknown_type_defaults(self, client):
        csv_content = "name,type,lat,lon\nThing,spaceship,40.0,44.0\n"
        resp = client.post(
            "/api/settings/fallback-delivery-locations/import",
            files={"file": ("fallbackLocations.csv", io.BytesIO(csv_content.encode()), "text/csv")},
        )
        assert resp.status_code == 200
        fallbackLocations = resp.json()["fallback_delivery_locations"]
        assert fallbackLocations[0]["type"] == "other"

    def test_import_merges_with_existing(self, client):
        # Add one DOCK via settings update
        client.put("/api/settings", json={
            "fallback_delivery_locations": [{"name": "Existing", "type": "fuel", "lat": 40.0, "lon": 44.0}]
        })
        # Import CSV
        csv_content = "name,type,lat,lon\nNew,vehicle,40.1,44.1\n"
        resp = client.post(
            "/api/settings/fallback-delivery-locations/import",
            files={"file": ("fallbackLocations.csv", io.BytesIO(csv_content.encode()), "text/csv")},
        )
        assert resp.status_code == 200
        fallbackLocations = resp.json()["fallback_delivery_locations"]
        assert len(fallbackLocations) == 2
        assert fallbackLocations[0]["name"] == "Existing"
        assert fallbackLocations[1]["name"] == "New"

    def test_import_without_type_column(self, client):
        csv_content = "name,lat,lon\nNoType,40.0,44.0\n"
        resp = client.post(
            "/api/settings/fallback-delivery-locations/import",
            files={"file": ("fallbackLocations.csv", io.BytesIO(csv_content.encode()), "text/csv")},
        )
        assert resp.status_code == 200
        fallbackLocations = resp.json()["fallback_delivery_locations"]
        assert fallbackLocations[0]["type"] == "other"
