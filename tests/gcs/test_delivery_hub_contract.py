"""Delivery hub inventory keeps coordinates and uses one current JSON format."""
from tests.gcs.test_settings import client


def test_delivery_hubs_settings_round_trip(client):
    locations = [{"name": "Fallback fixture", "type": "vehicle", "lat": 0.01, "lon": 0.02}]
    response = client.put("/api/settings", json={"default_delivery_hubs": locations})
    assert response.status_code == 200
    assert response.json()["default_delivery_hubs"] == locations
    assert client.get("/api/settings").json()["default_delivery_hubs"] == locations


def test_delivery_hubs_csv_import(client):
    response = client.post("/api/settings/default-delivery-hubs/import", files={
        "file": ("locations.csv", b"name,type,lat,lon\nFallback fixture,vehicle,0.01,0.02\n", "text/csv")})
    assert response.status_code == 200
    assert response.json()["default_delivery_hubs"][-1] == {
        "name": "Fallback fixture", "type": "vehicle", "lat": 0.01, "lon": 0.02}


def test_superseded_delivery_docks_field_rejected_without_mutation(client):
    before = client.get("/api/settings").json()
    response = client.put("/api/settings", json={"delivery_docks": []})
    assert response.status_code == 422
    assert client.get("/api/settings").json() == before
