"""Tests for mission download/upload route endpoints."""
from __future__ import annotations

from unittest.mock import MagicMock, patch, PropertyMock

import pytest
from fastapi.testclient import TestClient

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_NAV_LOITER_UNLIM, MAV_CMD_NAV_WAYPOINT, MAV_CMD_NAV_TAKEOFF,
)
from gcs.backend.planner.waypoint_builder import (
    CORRIDOR_END_MARKER, META_POLYGON_VERTEX, META_CORRIDOR_VERTEX,
    META_LAUNCH_POINT, META_FALLBACK_DELIVERY_LOCATION, encode_meta_z, encode_location_type_into_z,
)


def _make_wp(command, lat=0.0, lon=0.0, alt=0.0, param1=0.0, param2=0.0, param3=0.0, z=0.0):
    """Create a mock mission item."""
    wp = MagicMock()
    wp.command = command
    wp.x = int(lat * 1e7)
    wp.y = int(lon * 1e7)
    wp.z = z
    wp.param1 = param1
    wp.param2 = param2
    wp.param3 = param3
    return wp


def _make_location(lat, lon, alt):
    """Create a mock Location."""
    loc = MagicMock()
    loc.lat = lat
    loc.lng = lon
    loc.alt = alt
    return loc


@pytest.fixture
def mock_vehicle():
    """Create a mock VehicleMav with mission download support."""
    vehicle = MagicMock()
    vehicle.download_mission = MagicMock(return_value=0)
    vehicle.upload_mission = MagicMock(return_value=True)
    vehicle.clear_mission = MagicMock()
    vehicle.load_mission_items = MagicMock()
    vehicle.set_parameter = MagicMock(return_value=True)
    return vehicle


@pytest.fixture
def mock_entry(mock_vehicle):
    entry = MagicMock()
    entry.sys_id = 1
    entry.vehicle = mock_vehicle
    entry.mission_uploaded = False
    entry.cached_mission = None
    return entry


@pytest.fixture
def client(mock_entry):
    mock_mgr = MagicMock()
    mock_mgr.get_vehicle = MagicMock(return_value=mock_entry)
    with patch("gcs.backend.routes.missions.vehicle_mgr", mock_mgr):
        from gcs.backend.main import app
        yield TestClient(app)


class TestDownloadMission:
    def test_vehicle_not_found(self):
        mock_mgr = MagicMock()
        mock_mgr.get_vehicle = MagicMock(return_value=None)
        with patch("gcs.backend.routes.missions.vehicle_mgr", mock_mgr):
            from gcs.backend.main import app
            c = TestClient(app)
            resp = c.get("/api/vehicles/99/mission")
            assert resp.status_code == 404

    def test_empty_mission(self, client, mock_vehicle):
        mock_vehicle.download_mission.return_value = 0
        resp = client.get("/api/vehicles/1/mission")
        assert resp.status_code == 404

    def test_simple_mission_download(self, client, mock_vehicle):
        """Download a mission with home + takeoff + 2 nav waypoints."""
        items = [
            _make_wp(MAV_CMD_NAV_WAYPOINT, lat=32.0, lon=34.0),     # home (seq 0)
            _make_wp(MAV_CMD_NAV_TAKEOFF, lat=32.0, lon=34.0),      # takeoff
            _make_wp(MAV_CMD_NAV_WAYPOINT, lat=32.001, lon=34.001), # wp 1
            _make_wp(MAV_CMD_NAV_WAYPOINT, lat=32.002, lon=34.002), # wp 2
        ]
        mock_vehicle.download_mission.return_value = len(items)
        mock_vehicle.get_mission_item = MagicMock(side_effect=lambda i: items[i])
        mock_vehicle.get_mission_item_location = MagicMock(
            side_effect=lambda i: _make_location(
                items[i].x / 1e7, items[i].y / 1e7, 150.0,
            ) if i >= 2 else None,
        )

        resp = client.get("/api/vehicles/1/mission")
        assert resp.status_code == 200
        data = resp.json()
        assert data["sys_id"] == 1
        assert len(data["waypoints"]) == 2
        assert data["altitude_m"] == 150.0
        assert data["search_pattern"] == "distributed"
        assert data["corridor_end_index"] is None
        # Each waypoint carries its per-point alt
        for wp in data["waypoints"]:
            assert wp["alt"] == 150.0
        assert [wp["mission_sequence"] for wp in data["waypoints"]] == [2, 3]
        assert [wp["nav_waypoint_ordinal"] for wp in data["waypoints"]] == [1, 2]
        assert all(wp["command"] == MAV_CMD_NAV_WAYPOINT for wp in data["waypoints"])

    def test_non_waypoint_navigation_command_remains_in_route_without_target_ordinal(
        self,
        client,
        mock_vehicle,
    ):
        items = [
            _make_wp(MAV_CMD_NAV_WAYPOINT, lat=32.0, lon=34.0),
            _make_wp(MAV_CMD_NAV_TAKEOFF, lat=32.0, lon=34.0),
            _make_wp(MAV_CMD_NAV_WAYPOINT, lat=32.001, lon=34.001),
            _make_wp(MAV_CMD_NAV_LOITER_UNLIM, lat=39.0, lon=43.0),
            _make_wp(MAV_CMD_NAV_WAYPOINT, lat=32.002, lon=34.002),
        ]
        mock_vehicle.download_mission.return_value = len(items)
        mock_vehicle.get_mission_item = MagicMock(side_effect=lambda index: items[index])
        mock_vehicle.get_mission_item_location = MagicMock(
            side_effect=lambda index: _make_location(
                items[index].x / 1e7,
                items[index].y / 1e7,
                150.0,
            )
        )

        response = client.get("/api/vehicles/1/mission")

        assert response.status_code == 200
        rows = response.json()["waypoints"]
        assert [row["mission_sequence"] for row in rows] == [2, 3, 4]
        assert [row["command"] for row in rows] == [
            MAV_CMD_NAV_WAYPOINT,
            MAV_CMD_NAV_LOITER_UNLIM,
            MAV_CMD_NAV_WAYPOINT,
        ]
        assert [row.get("nav_waypoint_ordinal") for row in rows] == [1, None, 2]

    def test_per_waypoint_altitude(self, client, mock_vehicle):
        """Corridor approach waypoints have different altitude than scanning ones."""
        items = [
            _make_wp(MAV_CMD_NAV_WAYPOINT, lat=32.0, lon=34.0),       # home
            _make_wp(MAV_CMD_NAV_TAKEOFF, lat=32.0, lon=34.0),        # takeoff
            _make_wp(MAV_CMD_NAV_WAYPOINT, lat=32.001, lon=34.001),   # corridor wp
            _make_wp(MAV_CMD_NAV_WAYPOINT, lat=32.002, lon=34.002),   # corridor wp
            _make_wp(MAV_CMD_NAV_WAYPOINT, lat=32.003, lon=34.003),   # scanning wp
            _make_wp(MAV_CMD_NAV_WAYPOINT, lat=32.004, lon=34.004),   # scanning wp
        ]
        # Corridor approach at 200m, scanning at 120m
        altitudes = {2: 200.0, 3: 200.0, 4: 120.0, 5: 120.0}
        mock_vehicle.download_mission.return_value = len(items)
        mock_vehicle.get_mission_item = MagicMock(side_effect=lambda i: items[i])
        mock_vehicle.get_mission_item_location = MagicMock(
            side_effect=lambda i: _make_location(
                items[i].x / 1e7, items[i].y / 1e7, altitudes[i],
            ) if i in altitudes else None,
        )

        resp = client.get("/api/vehicles/1/mission")
        assert resp.status_code == 200
        data = resp.json()
        wps = data["waypoints"]
        assert len(wps) == 4
        # altitude_m is from the first waypoint
        assert data["altitude_m"] == 200.0
        # Per-point altitudes preserved
        assert wps[0]["alt"] == 200.0
        assert wps[1]["alt"] == 200.0
        assert wps[2]["alt"] == 120.0
        assert wps[3]["alt"] == 120.0

    def test_mission_with_metadata(self, client, mock_vehicle):
        """Download a mission with polygon/corridor/launch metadata."""
        meta_z = encode_meta_z("corridor", ["small", "medium"])
        items = [
            _make_wp(MAV_CMD_NAV_WAYPOINT, lat=32.0, lon=34.0),       # home
            _make_wp(MAV_CMD_NAV_TAKEOFF, lat=32.0, lon=34.0),        # takeoff
            _make_wp(MAV_CMD_NAV_WAYPOINT, lat=32.01, lon=34.01),     # corridor wp
            # metadata items
            _make_wp(CORRIDOR_END_MARKER, lat=32.1, lon=34.1,
                     param1=float(META_POLYGON_VERTEX)),
            _make_wp(CORRIDOR_END_MARKER, lat=32.2, lon=34.2,
                     param1=float(META_CORRIDOR_VERTEX)),
            _make_wp(CORRIDOR_END_MARKER, lat=32.05, lon=34.05,
                     param1=float(META_LAUNCH_POINT), z=meta_z),
            # track wp
            _make_wp(MAV_CMD_NAV_WAYPOINT, lat=32.003, lon=34.003),
        ]
        mock_vehicle.download_mission.return_value = len(items)
        mock_vehicle.get_mission_item = MagicMock(side_effect=lambda i: items[i])
        mock_vehicle.get_mission_item_location = MagicMock(
            side_effect=lambda i: _make_location(
                items[i].x / 1e7, items[i].y / 1e7, 120.0,
            ) if items[i].command == MAV_CMD_NAV_WAYPOINT and i >= 2 else None,
        )

        resp = client.get("/api/vehicles/1/mission")
        assert resp.status_code == 200
        data = resp.json()
        assert data["search_pattern"] == "corridor"
        assert "small" in data["dock_classes"]
        assert "medium" in data["dock_classes"]
        assert data["corridor_end_index"] == 1  # after 1 nav wp
        assert len(data["polygon"]) == 1
        assert len(data["corridor_backbone"]) == 1
        assert data["launch_point"] is not None

    def test_mission_with_fallback_delivery_location(self, client, mock_vehicle):
        """Download a mission with META_FALLBACK_DELIVERY_LOCATION metadata and trailing NAV_WAYPOINT."""
        meta_z = encode_meta_z("distributed", ["small", "medium"])
        target_lat, target_lon = 32.05, 34.05
        items = [
            _make_wp(MAV_CMD_NAV_WAYPOINT, lat=32.0, lon=34.0),           # home (seq 0)
            _make_wp(MAV_CMD_NAV_TAKEOFF, lat=32.0, lon=34.0),            # takeoff
            _make_wp(MAV_CMD_NAV_WAYPOINT, lat=32.001, lon=34.001),       # track wp 1
            _make_wp(MAV_CMD_NAV_WAYPOINT, lat=32.002, lon=34.002),       # track wp 2
            # metadata
            _make_wp(CORRIDOR_END_MARKER, lat=32.1, lon=34.1,
                     param1=float(META_POLYGON_VERTEX)),
            _make_wp(CORRIDOR_END_MARKER, lat=target_lat, lon=target_lon,
                     param1=float(META_FALLBACK_DELIVERY_LOCATION),
                     z=encode_location_type_into_z(meta_z, "antenna")),
            # trailing NAV_WAYPOINT duplicating fallback_delivery_location coords
            _make_wp(MAV_CMD_NAV_WAYPOINT, lat=target_lat, lon=target_lon),
        ]
        mock_vehicle.download_mission.return_value = len(items)
        mock_vehicle.get_mission_item = MagicMock(side_effect=lambda i: items[i])
        mock_vehicle.get_mission_item_location = MagicMock(
            side_effect=lambda i: _make_location(
                items[i].x / 1e7, items[i].y / 1e7, 120.0,
            ) if items[i].command == MAV_CMD_NAV_WAYPOINT and i >= 2 else None,
        )

        resp = client.get("/api/vehicles/1/mission")
        assert resp.status_code == 200
        data = resp.json()
        # fallback_delivery_location extracted from metadata
        assert data["fallback_delivery_location"] is not None
        assert abs(data["fallback_delivery_location"]["lat"] - target_lat) < 1e-5
        assert abs(data["fallback_delivery_location"]["lon"] - target_lon) < 1e-5
        assert data["fallback_delivery_location"]["type"] == "antenna"
        assert "default_target" not in data
        assert "target_classes" not in data
        # Trailing NAV_WAYPOINT trimmed — only track wp 1 and wp 2 remain
        assert len(data["waypoints"]) == 2
        # search_pattern + dock_classes decoded from the same metadata item's z
        assert data["search_pattern"] == "distributed"
        assert "small" in data["dock_classes"]
        assert "medium" in data["dock_classes"]
        # polygon vertex also decoded
        assert len(data["polygon"]) == 1


class TestDownloadMissionCacheReuse:
    """The GET must reuse the connect-time probe's cached mission instead of
    running a second full download — a redundant download shows up to the
    operator as the progress bar jumping backwards (16/17 -> 1/17 -> ...)."""

    def test_returns_cache_without_downloading(self, client, mock_entry, mock_vehicle):
        cached = {"sys_id": 1, "waypoints": [{"lat": 1.0, "lon": 2.0, "alt": 3.0}]}
        mock_entry.cached_mission = cached
        resp = client.get("/api/vehicles/1/mission")
        assert resp.status_code == 200
        assert resp.json() == cached
        mock_vehicle.download_mission.assert_not_called()

    def test_reuses_cache_populated_while_waiting_for_lock(self, client, mock_entry, mock_vehicle):
        """Outer pre-lock check sees no cache, but the probe finishes + caches
        while we're blocked on the mission lock — the under-lock re-check must
        catch that and skip the download."""
        cached = {"sys_id": 1, "waypoints": [{"lat": 1.0, "lon": 2.0, "alt": 3.0}]}

        class LockSetsCache:
            def __enter__(self_lock):
                mock_entry.cached_mission = cached  # probe cached while we waited
                return None

            def __exit__(self_lock, *exc):
                return False

        mock_entry.cached_mission = None
        mock_vehicle.mission_lock = LockSetsCache()
        resp = client.get("/api/vehicles/1/mission")
        assert resp.status_code == 200
        assert resp.json() == cached
        mock_vehicle.download_mission.assert_not_called()


class TestDownloadFence:
    """GET /api/vehicles/{sys_id}/fence — parse the vehicle's fence table."""

    def test_vehicle_not_found(self):
        mock_mgr = MagicMock()
        mock_mgr.get_vehicle = MagicMock(return_value=None)
        with patch("gcs.backend.routes.missions.vehicle_mgr", mock_mgr):
            from gcs.backend.main import app
            c = TestClient(app)
            resp = c.get("/api/vehicles/99/fence")
            assert resp.status_code == 404

    def test_download_fence_parsed(self, client, mock_vehicle):
        """download_fence items (built by build_fence_items) parse into the
        inclusion ring + exclusion rings, with the raw item count."""
        from gcs.backend.planner.fence_builder import build_fence_items
        verts = [
            {"lat": 32.0, "lon": 34.0},
            {"lat": 32.01, "lon": 34.0},
            {"lat": 32.0, "lon": 34.01},
        ]
        exclusions = [[
            {"lat": 32.005, "lon": 34.005},
            {"lat": 32.006, "lon": 34.005},
            {"lat": 32.005, "lon": 34.006},
        ]]
        items = build_fence_items(verts, target_system=1, exclusions=exclusions)
        mock_vehicle.download_fence = MagicMock(return_value=items)

        resp = client.get("/api/vehicles/1/fence")
        assert resp.status_code == 200
        data = resp.json()
        assert len(data["vertices"]) == 3
        assert len(data["exclusions"]) == 1
        assert len(data["exclusions"][0]) == 3
        assert data["total_items"] == 6
        assert abs(data["vertices"][0]["lat"] - 32.0) < 1e-5
        assert abs(data["vertices"][0]["lon"] - 34.0) < 1e-5

    def test_download_empty_fence(self, client, mock_vehicle):
        """No fence on the vehicle -> 200 with empty lists."""
        mock_vehicle.download_fence = MagicMock(return_value=[])
        resp = client.get("/api/vehicles/1/fence")
        assert resp.status_code == 200
        data = resp.json()
        assert data["vertices"] == []
        assert data["exclusions"] == []
        assert data["total_items"] == 0

    def test_download_fence_error_returns_502(self, client, mock_vehicle):
        """A download failure returns 502, never a raw 500 trace."""
        mock_vehicle.download_fence = MagicMock(side_effect=RuntimeError("link lost"))
        resp = client.get("/api/vehicles/1/fence")
        assert resp.status_code == 502
        assert "Fence download failed" in resp.json()["detail"]

    def test_download_fence_reports_raw_params(self, client, mock_vehicle):
        """The fence readback carries the vehicle's RAW four fence params, read
        fresh. FENCE_ENABLE=0 with FENCE_AUTOENABLE=1 is a configured
        auto-enable mode, not "off" — the operator display can only tell them
        apart if the raw values survive the route."""
        mock_vehicle.download_fence = MagicMock(return_value=[])
        raw = {"FENCE_ENABLE": 0.0, "FENCE_AUTOENABLE": 1.0,
               "FENCE_TYPE": 4.0, "FENCE_ACTION": 1.0}
        mock_vehicle.get_parameter_fresh = MagicMock(side_effect=lambda n, **kw: raw[n])

        resp = client.get("/api/vehicles/1/fence")
        assert resp.status_code == 200
        data = resp.json()
        assert data["params_readback"] == "ok"
        assert data["params"] == {
            "enable": 0.0, "autoenable": 1.0, "type": 4.0, "action": 1.0,
        }
        # A known-empty fence table is distinct from a failed readback.
        assert data["vertices"] == []
        # Fresh reads only — a cached value could predate an operator's change.
        assert mock_vehicle.get_parameter_fresh.call_count == 4
        mock_vehicle.get_parameter.assert_not_called()

    def test_download_fence_unreadable_params_are_unknown_not_zero(self, client, mock_vehicle):
        """An unreadable param must report failure, never an invented 0. A
        zero would read downstream as "fence off" on a vehicle whose fence is
        actually enabled."""
        mock_vehicle.download_fence = MagicMock(return_value=[])
        mock_vehicle.get_parameter_fresh = MagicMock(return_value=None)

        resp = client.get("/api/vehicles/1/fence")
        assert resp.status_code == 200
        data = resp.json()
        assert data["params_readback"] == "failed"
        assert data["params"] is None

    def test_download_fence_partial_param_read_is_failed(self, client, mock_vehicle):
        """One missing value makes the whole readback unknown — a half-known
        mode cannot be displayed as a mode."""
        mock_vehicle.download_fence = MagicMock(return_value=[])
        raw = {"FENCE_ENABLE": 1.0, "FENCE_AUTOENABLE": None,
               "FENCE_TYPE": 4.0, "FENCE_ACTION": 1.0}
        mock_vehicle.get_parameter_fresh = MagicMock(side_effect=lambda n, **kw: raw[n])

        resp = client.get("/api/vehicles/1/fence")
        assert resp.status_code == 200
        assert resp.json()["params_readback"] == "failed"
        assert resp.json()["params"] is None

    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    def test_download_fence_nonfinite_param_is_unknown_not_a_500(
            self, client, mock_vehicle, bad):
        """A non-finite readback is not a fence mode. JSON cannot carry NaN or
        Infinity, so returning one turned the whole readback into a 500 and the
        operator saw nothing at all — report unknown instead."""
        mock_vehicle.download_fence = MagicMock(return_value=[])
        raw = {"FENCE_ENABLE": bad, "FENCE_AUTOENABLE": 1.0,
               "FENCE_TYPE": 4.0, "FENCE_ACTION": 1.0}
        mock_vehicle.get_parameter_fresh = MagicMock(side_effect=lambda n, **kw: raw[n])

        resp = client.get("/api/vehicles/1/fence")
        assert resp.status_code == 200
        assert resp.json()["params"] is None
        assert resp.json()["params_readback"] == "failed"

    def test_download_fence_param_error_does_not_fail_the_ring(self, client, mock_vehicle):
        """A raising param read degrades to unknown params; the fence table
        itself still returns."""
        from gcs.backend.planner.fence_builder import build_fence_items
        verts = [{"lat": 32.0, "lon": 34.0}, {"lat": 32.01, "lon": 34.0},
                 {"lat": 32.0, "lon": 34.01}]
        mock_vehicle.download_fence = MagicMock(
            return_value=build_fence_items(verts, target_system=1))
        mock_vehicle.get_parameter_fresh = MagicMock(side_effect=RuntimeError("link lost"))

        resp = client.get("/api/vehicles/1/fence")
        assert resp.status_code == 200
        data = resp.json()
        assert len(data["vertices"]) == 3
        assert data["params"] is None
        assert data["params_readback"] == "failed"


class TestUploadMissions:
    def test_upload_success(self, client, mock_entry):
        """Upload succeeds via upload_mission_with_retry mock."""
        from gcs.backend.planner.waypoint_builder import UploadResult
        fake_result = UploadResult(success=True, uploaded_count=4, expected_count=4, attempts=1)
        with patch("gcs.backend.routes.missions.upload_mission_with_retry", return_value=fake_result):
            resp = client.post("/api/vehicles/upload", json={
                "assignments": [{
                    "sys_id": 1,
                    "zone_index": 0,
                    "waypoints": [{"lat": 32.0, "lon": 34.0}, {"lat": 32.001, "lon": 34.001}],
                    "altitude_m": 150.0,
                }],
            })
        assert resp.status_code == 200
        data = resp.json()
        assert data["status"] == "complete"
        assert len(data["results"]) == 1
        assert data["results"][0]["error"] is None
        # Verify mission_uploaded flag was set
        assert mock_entry.mission_uploaded is True

    def test_upload_failure_sets_error(self, client, mock_entry):
        """Upload failure returns error and does not set mission_uploaded."""
        from gcs.backend.planner.waypoint_builder import UploadResult
        fake_result = UploadResult(
            success=False, uploaded_count=0, expected_count=4, attempts=3,
            error="Upload failed after 3 attempts",
        )
        with patch("gcs.backend.routes.missions.upload_mission_with_retry", return_value=fake_result):
            resp = client.post("/api/vehicles/upload", json={
                "assignments": [{
                    "sys_id": 1,
                    "zone_index": 0,
                    "waypoints": [{"lat": 32.0, "lon": 34.0}, {"lat": 32.001, "lon": 34.001}],
                    "altitude_m": 150.0,
                }],
            })
        assert resp.status_code == 200
        data = resp.json()
        assert data["status"] == "partial_failure"
        assert data["results"][0]["error"] is not None
        assert mock_entry.mission_uploaded is False

    def test_upload_vehicle_not_found(self):
        mock_mgr = MagicMock()
        mock_mgr.get_vehicle = MagicMock(return_value=None)
        with patch("gcs.backend.routes.missions.vehicle_mgr", mock_mgr):
            from gcs.backend.main import app
            c = TestClient(app)
            resp = c.post("/api/vehicles/upload", json={
                "assignments": [{
                    "sys_id": 99,
                    "zone_index": 0,
                    "waypoints": [{"lat": 32.0, "lon": 34.0}],
                    "altitude_m": 100.0,
                }],
            })
            assert resp.status_code == 200
            data = resp.json()
            assert data["status"] == "partial_failure"
            assert data["results"][0]["error"] is not None

    def test_upload_forwards_fallback_delivery_location_type(self, client):
        """Upload forwards DOCK type without adding extra mission items."""
        from gcs.backend.planner.waypoint_builder import UploadResult
        fake_result = UploadResult(success=True, uploaded_count=4, expected_count=4, attempts=1)
        with patch("gcs.backend.routes.missions.upload_mission_with_retry", return_value=fake_result) as mock_upload:
            resp = client.post("/api/vehicles/upload", json={
                "assignments": [{
                    "sys_id": 1,
                    "zone_index": 0,
                    "waypoints": [{"lat": 32.0, "lon": 34.0}, {"lat": 32.001, "lon": 34.001}],
                    "altitude_m": 150.0,
                    "fallback_delivery_location": {"lat": 40.5, "lon": 44.5, "type": "bridge"},
                }],
            })
        assert resp.status_code == 200
        kwargs = mock_upload.call_args.kwargs
        assert kwargs["fallback_delivery_location"] == {"lat": 40.5, "lon": 44.5, "type": "bridge"}

    def _fence_body(self, enabled=True, vertices=None):
        if vertices is None:
            vertices = [
                {"lat": 32.0, "lon": 34.0},
                {"lat": 32.01, "lon": 34.0},
                {"lat": 32.0, "lon": 34.01},
            ]
        return {
            "assignments": [{
                "sys_id": 1,
                "zone_index": 0,
                "waypoints": [{"lat": 32.0, "lon": 34.0}, {"lat": 32.001, "lon": 34.001}],
                "altitude_m": 150.0,
            }],
            "fence": {"enabled": enabled, "action": 1, "type": 4, "vertices": vertices},
        }

    def test_apply_fence_reports_vertex_count_total(self, mock_entry):
        """A successful fence upload reports ok=True and the uploaded vertex
        count. Regression: the status dict read a non-existent ``fence_total``
        attribute, so every successful upload raised AttributeError, was
        swallowed, and got reported to the operator as a failed fence."""
        import asyncio
        from gcs.backend.routes.missions import _apply_fence
        from gcs.backend.models import FencePlan
        from gcs.backend.planner.fence_builder import FenceUploadResult
        fence = FencePlan(enabled=True, action=1, type=4, vertices=[
            {"lat": 32.0, "lon": 34.0}, {"lat": 32.01, "lon": 34.0},
            {"lat": 32.0, "lon": 34.01}, {"lat": 32.01, "lon": 34.01},
        ])
        res_obj = FenceUploadResult(success=True, vertex_count=4)

        async def _run():
            loop = asyncio.get_running_loop()
            with patch("gcs.backend.routes.missions.upload_fence_with_retry", return_value=res_obj):
                return await _apply_fence(loop, mock_entry, 1, fence)

        status = asyncio.run(_run())
        # `ok` is retained for the upload_progress websocket payload; the
        # structured fields were added so the HTTP result can distinguish a
        # fence failure from a mission failure.
        assert status == {
            "ok": True, "applied": True, "action": "enable", "enabled": True,
            "total": 4, "error": None, "failed_params": [],
        }

    def test_upload_enabled_fence_sets_params(self, client, mock_vehicle):
        """An enabled fence uploads points and sets FENCE_TYPE/ACTION/ENABLE."""
        from gcs.backend.planner.waypoint_builder import UploadResult
        from gcs.backend.planner.fence_builder import FenceUploadResult
        mission_ok = UploadResult(success=True, uploaded_count=4, expected_count=4, attempts=1)
        fence_ok = FenceUploadResult(success=True, vertex_count=3)
        with patch("gcs.backend.routes.missions.upload_mission_with_retry", return_value=mission_ok), \
             patch("gcs.backend.routes.missions.upload_fence_with_retry", return_value=fence_ok) as mock_fence:
            resp = client.post("/api/vehicles/upload", json=self._fence_body(enabled=True))
        assert resp.status_code == 200
        assert resp.json()["status"] == "complete"
        mock_fence.assert_called_once()
        calls = {c.args[0]: c.args[1] for c in mock_vehicle.set_parameter.call_args_list}
        assert calls["FENCE_TYPE"] == 4.0
        assert calls["FENCE_ACTION"] == 1.0
        assert calls["FENCE_ENABLE"] == 1.0
        assert calls["FENCE_AUTOENABLE"] == 1.0
        # ArduPilot writes FENCE_TOTAL itself — the route must not set it.
        assert "FENCE_TOTAL" not in calls

    def test_upload_disabled_fence_clears_enable(self, client, mock_vehicle):
        """A disabled fence sets FENCE_ENABLE=0 and uploads no points."""
        from gcs.backend.planner.waypoint_builder import UploadResult
        mission_ok = UploadResult(success=True, uploaded_count=4, expected_count=4, attempts=1)
        with patch("gcs.backend.routes.missions.upload_mission_with_retry", return_value=mission_ok), \
             patch("gcs.backend.routes.missions.upload_fence_with_retry") as mock_fence:
            resp = client.post("/api/vehicles/upload", json=self._fence_body(enabled=False))
        assert resp.status_code == 200
        mock_fence.assert_not_called()
        calls = {c.args[0]: c.args[1] for c in mock_vehicle.set_parameter.call_args_list}
        assert calls["FENCE_ENABLE"] == 0.0
        # AUTOENABLE must also be cleared, else the fence re-arms at takeoff.
        assert calls["FENCE_AUTOENABLE"] == 0.0
        assert "FENCE_TYPE" not in calls

    def test_upload_without_fence_touches_no_fence_params(self, client, mock_vehicle):
        """A fence-less upload must not touch any FENCE_* param."""
        from gcs.backend.planner.waypoint_builder import UploadResult
        mission_ok = UploadResult(success=True, uploaded_count=4, expected_count=4, attempts=1)
        with patch("gcs.backend.routes.missions.upload_mission_with_retry", return_value=mission_ok):
            resp = client.post("/api/vehicles/upload", json={
                "assignments": [{
                    "sys_id": 1, "zone_index": 0,
                    "waypoints": [{"lat": 32.0, "lon": 34.0}, {"lat": 32.001, "lon": 34.001}],
                    "altitude_m": 150.0,
                }],
            })
        assert resp.status_code == 200
        names = [c.args[0] for c in mock_vehicle.set_parameter.call_args_list]
        assert not any(n.startswith("FENCE_") for n in names)

    def test_fence_failure_does_not_fail_mission(self, client, mock_entry, mock_vehicle):
        """A failed fence upload leaves the mission result complete."""
        from gcs.backend.planner.waypoint_builder import UploadResult
        from gcs.backend.planner.fence_builder import FenceUploadResult
        mission_ok = UploadResult(success=True, uploaded_count=4, expected_count=4, attempts=1)
        fence_bad = FenceUploadResult(success=False, vertex_count=3, error="no ACK")
        with patch("gcs.backend.routes.missions.upload_mission_with_retry", return_value=mission_ok), \
             patch("gcs.backend.routes.missions.upload_fence_with_retry", return_value=fence_bad):
            resp = client.post("/api/vehicles/upload", json=self._fence_body(enabled=True))
        assert resp.status_code == 200
        data = resp.json()
        assert data["status"] == "complete"
        assert data["results"][0]["error"] is None
        assert mock_entry.mission_uploaded is True

    def test_fence_outcome_is_reported_per_vehicle(self, client, mock_vehicle):
        """A successful fence carries a structured per-vehicle outcome plus a
        top-level fence_ok, so the UI can distinguish mission success from
        fence success."""
        from gcs.backend.planner.waypoint_builder import UploadResult
        from gcs.backend.planner.fence_builder import FenceUploadResult
        mission_ok = UploadResult(success=True, uploaded_count=4, expected_count=4, attempts=1)
        fence_ok = FenceUploadResult(success=True, vertex_count=3)
        with patch("gcs.backend.routes.missions.upload_mission_with_retry", return_value=mission_ok), \
             patch("gcs.backend.routes.missions.upload_fence_with_retry", return_value=fence_ok):
            resp = client.post("/api/vehicles/upload", json=self._fence_body(enabled=True))
        data = resp.json()
        assert data["fence_ok"] is True
        assert data["results"][0]["fence"] == {
            "applied": True, "action": "enable", "enabled": True,
            "total": 3, "error": None, "failed_params": [],
        }

    def test_no_fence_request_reports_no_fence_attempt(self, client):
        """Omitting the fence block means "do not touch the fence" — the
        per-vehicle outcome is null and fence_ok stays true."""
        from gcs.backend.planner.waypoint_builder import UploadResult
        mission_ok = UploadResult(success=True, uploaded_count=4, expected_count=4, attempts=1)
        with patch("gcs.backend.routes.missions.upload_mission_with_retry", return_value=mission_ok):
            resp = client.post("/api/vehicles/upload", json={
                "assignments": [{
                    "sys_id": 1, "zone_index": 0,
                    "waypoints": [{"lat": 32.0, "lon": 34.0}, {"lat": 32.001, "lon": 34.001}],
                    "altitude_m": 150.0,
                }],
            })
        data = resp.json()
        assert data["results"][0]["fence"] is None
        assert data["fence_ok"] is True

    def test_disable_retains_geometry_type_and_action(self, client, mock_vehicle):
        """Explicit Disable writes ONLY ENABLE=0/AUTOENABLE=0: the stored ring,
        FENCE_TYPE and FENCE_ACTION stay as the vehicle has them, and no fence
        table is uploaded or cleared."""
        from gcs.backend.planner.waypoint_builder import UploadResult
        mission_ok = UploadResult(success=True, uploaded_count=4, expected_count=4, attempts=1)
        with patch("gcs.backend.routes.missions.upload_mission_with_retry", return_value=mission_ok), \
             patch("gcs.backend.routes.missions.upload_fence_with_retry") as mock_fence:
            resp = client.post("/api/vehicles/upload", json=self._fence_body(enabled=False))
        mock_fence.assert_not_called()
        mock_vehicle.clear_mission.assert_not_called()
        fence_writes = [c.args[0] for c in mock_vehicle.set_parameter.call_args_list
                        if c.args[0].startswith("FENCE_")]
        assert sorted(fence_writes) == ["FENCE_AUTOENABLE", "FENCE_ENABLE"]
        assert resp.json()["results"][0]["fence"]["action"] == "disable"
        assert resp.json()["results"][0]["fence"]["applied"] is True

    def test_failed_param_write_reports_fence_failure_with_mission_success(
            self, client, mock_vehicle):
        """A rejected FENCE_ENABLE write must surface as a fence failure while
        the mission result stays successful. Regression: set_parameter results
        were never checked, so a silently dropped write was reported to the
        operator as an applied fence."""
        from gcs.backend.planner.waypoint_builder import UploadResult
        from gcs.backend.planner.fence_builder import FenceUploadResult
        mission_ok = UploadResult(success=True, uploaded_count=4, expected_count=4, attempts=1)
        fence_ok = FenceUploadResult(success=True, vertex_count=3)
        mock_vehicle.set_parameter = MagicMock(
            side_effect=lambda name, value, **kw: name != "FENCE_ENABLE")
        with patch("gcs.backend.routes.missions.upload_mission_with_retry", return_value=mission_ok), \
             patch("gcs.backend.routes.missions.upload_fence_with_retry", return_value=fence_ok):
            resp = client.post("/api/vehicles/upload", json=self._fence_body(enabled=True))
        data = resp.json()
        assert data["status"] == "complete"
        assert data["results"][0]["error"] is None
        assert data["fence_ok"] is False
        fence = data["results"][0]["fence"]
        assert fence["applied"] is False
        assert fence["failed_params"] == ["FENCE_ENABLE"]

    def test_failed_write_invalidates_cache_and_asserts_no_rollback(
            self, client, mock_vehicle):
        """A partially applied fence must still invalidate the param cache (the
        vehicle's values changed) and must NOT issue compensating writes — the
        route does not claim a rollback it cannot perform."""
        from gcs.backend.planner.waypoint_builder import UploadResult
        from gcs.backend.planner.fence_builder import FenceUploadResult
        mission_ok = UploadResult(success=True, uploaded_count=4, expected_count=4, attempts=1)
        fence_ok = FenceUploadResult(success=True, vertex_count=3)
        mock_vehicle.set_parameter = MagicMock(
            side_effect=lambda name, value, **kw: name != "FENCE_ENABLE")
        cache = MagicMock()
        with patch("gcs.backend.routes.missions.upload_mission_with_retry", return_value=mission_ok), \
             patch("gcs.backend.routes.missions.upload_fence_with_retry", return_value=fence_ok), \
             patch("gcs.backend.routes.missions.full_param_cache", cache):
            client.post("/api/vehicles/upload", json=self._fence_body(enabled=True))
        invalidated = set()
        for call in cache.invalidate_keys.call_args_list:
            invalidated.update(call.args[1])
        assert {"FENCE_ENABLE", "FENCE_AUTOENABLE", "FENCE_TYPE", "FENCE_ACTION"} <= invalidated
        fence_writes = [(c.args[0], c.args[1]) for c in mock_vehicle.set_parameter.call_args_list
                        if c.args[0].startswith("FENCE_")]
        # Each fence param is written exactly once: no retry, no rollback pass.
        assert len(fence_writes) == len({name for name, _ in fence_writes})

    def test_failed_mission_vehicle_receives_no_fence_write(self, client):
        """A vehicle whose mission upload failed must be left untouched while
        its fleet-mates still get the fence."""
        from gcs.backend.planner.waypoint_builder import UploadResult
        from gcs.backend.planner.fence_builder import FenceUploadResult
        good, bad = MagicMock(), MagicMock()
        for v in (good, bad):
            v.set_parameter = MagicMock(return_value=True)
        entries = {}
        for sys_id, vehicle in ((1, good), (2, bad)):
            entry = MagicMock()
            entry.sys_id = sys_id
            entry.vehicle = vehicle
            entry.cached_mission = None
            entries[sys_id] = entry
        mgr = MagicMock()
        mgr.get_vehicle = MagicMock(side_effect=lambda sid: entries.get(sid))
        results = {
            1: UploadResult(success=True, uploaded_count=4, expected_count=4, attempts=1),
            2: UploadResult(success=False, uploaded_count=0, expected_count=4,
                            attempts=1, error="no ACK"),
        }
        body = self._fence_body(enabled=True)
        body["assignments"].append({**body["assignments"][0], "sys_id": 2})

        def _upload(vehicle, *a, **kw):
            return results[1 if vehicle is good else 2]

        with patch("gcs.backend.routes.missions.vehicle_mgr", mgr), \
             patch("gcs.backend.routes.missions.upload_mission_with_retry", side_effect=_upload), \
             patch("gcs.backend.routes.missions.upload_fence_with_retry",
                   return_value=FenceUploadResult(success=True, vertex_count=3)):
            from gcs.backend.main import app
            resp = TestClient(app).post("/api/vehicles/upload", json=body)
        data = resp.json()
        assert data["status"] == "partial_failure"
        by_id = {r["sys_id"]: r for r in data["results"]}
        assert by_id[1]["fence"]["applied"] is True
        assert by_id[2]["fence"] is None
        assert not any(c.args[0].startswith("FENCE_")
                       for c in bad.set_parameter.call_args_list)
        # A failed mission must not drag fence_ok down: no fence was attempted.
        assert data["fence_ok"] is True

    def test_enabled_fence_with_too_few_vertices_is_rejected_not_disabled(
            self, client, mock_vehicle):
        """An enable request whose geometry is invalid is REJECTED. Converting
        it to a disable would silently switch off a fence the operator asked to
        turn on."""
        from gcs.backend.planner.waypoint_builder import UploadResult
        mission_ok = UploadResult(success=True, uploaded_count=4, expected_count=4, attempts=1)
        body = self._fence_body(enabled=True, vertices=[{"lat": 32.0, "lon": 34.0}])
        with patch("gcs.backend.routes.missions.upload_mission_with_retry", return_value=mission_ok), \
             patch("gcs.backend.routes.missions.upload_fence_with_retry") as mock_fence:
            resp = client.post("/api/vehicles/upload", json=body)
        mock_fence.assert_not_called()
        data = resp.json()
        assert data["fence_ok"] is False
        assert data["results"][0]["fence"]["action"] == "rejected"
        assert data["results"][0]["fence"]["applied"] is False
        assert not any(c.args[0].startswith("FENCE_")
                       for c in mock_vehicle.set_parameter.call_args_list)

    # ---- attempted-geometry cache invalidation ----------------------------
    # upload_fence_with_retry CLEARS the vehicle's fence table and only then
    # uploads the new one. A failure or an exception after that clear leaves the
    # vehicle holding different geometry (FENCE_TOTAL above all) than before, so
    # the cached values are stale whatever the outcome. These cover the attempt,
    # not a rollback: nothing here restores the previous fence.

    def _run_apply_fence(self, entry, fence, upload_fence, cache):
        import asyncio
        from gcs.backend.routes.mission_upload_helpers import apply_fence

        async def _run():
            return await apply_fence(
                asyncio.get_running_loop(), entry, 1, fence,
                upload_fence=upload_fence, cache=cache,
            )

        return asyncio.run(_run())

    def _enable_plan(self):
        from gcs.backend.models import FencePlan
        return FencePlan(enabled=True, action=1, type=4, vertices=[
            {"lat": 32.0, "lon": 34.0}, {"lat": 32.01, "lon": 34.0},
            {"lat": 32.0, "lon": 34.01},
        ])

    def test_failed_fence_upload_invalidates_attempted_geometry(self, mock_entry):
        """A fence upload that returns failure already cleared the vehicle's
        fence table, so the cached fence values must be invalidated."""
        from gcs.backend.planner.fence_builder import FenceUploadResult
        cache = MagicMock()
        status = self._run_apply_fence(
            mock_entry, self._enable_plan(),
            lambda *a, **kw: FenceUploadResult(success=False, vertex_count=3, error="no ACK"),
            cache,
        )
        assert status["applied"] is False
        assert status["action"] == "enable"
        assert status["error"] == "no ACK"
        invalidated = set()
        for call in cache.invalidate_keys.call_args_list:
            assert call.args[0] == 1
            invalidated.update(call.args[1])
        assert "FENCE_TOTAL" in invalidated

    def test_raising_fence_upload_invalidates_attempted_geometry(self, mock_entry):
        """Same for an exception mid-upload: the clear may already have run."""
        cache = MagicMock()

        def _boom(*a, **kw):
            raise RuntimeError("link lost mid-upload")

        status = self._run_apply_fence(mock_entry, self._enable_plan(), _boom, cache)
        assert status["applied"] is False
        assert status["action"] == "enable"
        assert "link lost mid-upload" in status["error"]
        invalidated = set()
        for call in cache.invalidate_keys.call_args_list:
            assert call.args[0] == 1
            invalidated.update(call.args[1])
        assert "FENCE_TOTAL" in invalidated

    def test_rejected_geometry_invalidates_nothing(self, mock_entry):
        """A rejected enable never reaches the vehicle, so nothing is stale."""
        from gcs.backend.models import FencePlan
        cache = MagicMock()
        upload = MagicMock()
        status = self._run_apply_fence(
            mock_entry,
            FencePlan(enabled=True, action=1, type=4, vertices=[{"lat": 32.0, "lon": 34.0}]),
            upload, cache,
        )
        assert status["action"] == "rejected"
        upload.assert_not_called()
        cache.invalidate_keys.assert_not_called()
