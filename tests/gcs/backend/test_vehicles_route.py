"""Route tests for `GET /api/vehicles` (list_vehicles)."""
from __future__ import annotations

import asyncio
from unittest.mock import MagicMock, patch

from gcs.backend.routes import vehicles


def _snapshot(sys_id: int, **overrides) -> dict:
    """Minimal telemetry snapshot as produced by VehicleEntry.snapshot()."""
    snap = {
        "sys_id": sys_id,
        "name": f"UAV {sys_id}",
        "battery": 90,
        "mode": "AUTO",
        "armed": False,
        "lat": 50.0,
        "lon": 30.0,
        "alt": 120.0,
        "heading": 90.0,
        "ground_speed": 22.0,
        "link_ok": True,
        "mission_progress": 2,
        "mission_total": 10,
    }
    snap.update(overrides)
    return snap


def _list_vehicles(snapshots: list[dict]):
    mock_mgr = MagicMock()
    mock_mgr.get_all_snapshots = MagicMock(return_value=snapshots)
    with patch("gcs.backend.routes.vehicles.vehicle_mgr", mock_mgr):
        return asyncio.run(vehicles.list_vehicles())


class TestListVehiclesArmed:
    def test_armed_reflects_snapshot(self):
        """`armed` is exposed on REST from the same snapshot the WS loop uses."""
        result = _list_vehicles([
            _snapshot(1, armed=True),
            _snapshot(2, armed=False),
        ])
        by_id = {v.sys_id: v for v in result}
        assert by_id[1].armed is True
        assert by_id[2].armed is False

    def test_armed_serialized_in_response_payload(self):
        """The field survives model serialization (what REST clients receive)."""
        result = _list_vehicles([_snapshot(1, armed=True)])
        assert result[0].model_dump()["armed"] is True

    def test_armed_defaults_to_none_when_missing(self):
        """A snapshot without arm state yields armed=None, not an error."""
        snap = _snapshot(3)
        del snap["armed"]
        result = _list_vehicles([snap])
        assert result[0].armed is None
