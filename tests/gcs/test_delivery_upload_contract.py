"""Upload request names only; transports and vehicle behavior are mocked."""
from unittest.mock import patch

import pytest
from pydantic import ValidationError

from gcs.backend.models import VehicleAssignment
from gcs.backend.planner.waypoint_builder import UploadResult
from tests.gcs.test_routes_missions import client, mock_entry, mock_vehicle


def request_data():
    return dict(sys_id=1, zone_index=0, waypoints=[dict(lat=0, lon=0)],
                altitude_m=100, dock_classes=["medium"],
                fallback_delivery_location=dict(lat=0.01, lon=0.02, type="other"))


def test_new_fields_preserve_values_in_model():
    data = request_data()
    model = VehicleAssignment.model_validate(data)
    assert model.model_dump()["dock_classes"] == data["dock_classes"]
    assert model.model_dump()["fallback_delivery_location"] == data["fallback_delivery_location"]


@pytest.mark.parametrize("key,value", [("poi_classes", ["old"]), ("default_poi", {"lat": 0, "lon": 0})])
def test_old_request_fields_are_rejected(key, value):
    with pytest.raises(ValidationError):
        VehicleAssignment.model_validate({**request_data(), key: value})


def test_unknown_dock_field_is_rejected():
    data = request_data()
    data["fallback_delivery_location"]["unexpected"] = True
    with pytest.raises(ValidationError):
        VehicleAssignment.model_validate(data)


def test_upload_passes_new_values_to_existing_builder(client):
    data = request_data()
    result = UploadResult(success=True, uploaded_count=4, expected_count=4, attempts=1)
    with patch("gcs.backend.routes.missions.upload_mission_with_retry", return_value=result) as upload:
        response = client.post("/api/vehicles/upload", json={"assignments": [data]})
    assert response.status_code == 200
    assert upload.call_args.kwargs["dock_classes"] == data["dock_classes"]
    assert upload.call_args.kwargs["fallback_delivery_location"] == data["fallback_delivery_location"]


def test_old_field_returns_422_before_upload(client):
    with patch("gcs.backend.routes.missions.upload_mission_with_retry", return_value=UploadResult(success=True, uploaded_count=4, expected_count=4, attempts=1)) as upload:
        response = client.post("/api/vehicles/upload", json={"assignments": [{**request_data(), "poi_classes": ["old"]}]})
    assert response.status_code == 422
    upload.assert_not_called()
