"""Search-pattern upload contract, including rejection before route execution."""
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from gcs.backend.models import VehicleAssignment
from gcs.backend.routes.missions import router


@pytest.fixture
def client():
    app = FastAPI()
    app.include_router(router, prefix="/api/vehicles")
    return TestClient(app)


def _assignment():
    return {"sys_id": 999, "zone_index": 0, "waypoints": [], "altitude_m": 120}


@pytest.mark.parametrize("pattern", ["distributed", "corridor"])
def test_assignment_serializes_current_search_pattern(pattern):
    payload = VehicleAssignment(**_assignment(), search_pattern=pattern).model_dump()
    assert payload["search_pattern"] == pattern
    assert "tactic" not in payload


@pytest.mark.parametrize("extra", [{}, {"search_pattern": "distributed"}])
def test_upload_rejects_old_pattern_key_before_execution(client, extra):
    response = client.post("/api/vehicles/upload", json={
        "assignments": [{**_assignment(), "tactic": "corridor", **extra}],
    })
    assert response.status_code == 422
    assert any(error["loc"][-1] == "tactic" and error["type"] == "extra_forbidden"
               for error in response.json()["detail"])
