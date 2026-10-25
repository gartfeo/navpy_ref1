"""Plan/camera compatibility check (docs/design/plan_camera_check.md)."""
import json
import math
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest
from fastapi.testclient import TestClient

from gcs.backend.main import app
from gcs.backend.plan_camera_check import (
    ZoneInput,
    check_plan,
    max_search_height,
    search_cameras,
    search_window,
    segment_in_view,
)
from navpy.core.perception.profile.vision_profile_loader import load_profiles

_FIXTURE = Path(__file__).resolve().parents[2] / "scripts" / "eval_delivery_scenario.json"


@pytest.fixture(scope="module")
def zr10():
    profiles, _, _ = load_profiles()
    return profiles["siyi_zr10"]


@pytest.fixture(scope="module")
def cam(zr10):
    return search_cameras(zr10, sim_finite=True)[0]


def _fixture_zone(altitude_m, *, dock_alt_m=0.0, use_terrain=False, route_from=0):
    """Zone 0 of the delivery fixture: the offerer's lanes and d1/d2."""
    data = json.loads(_FIXTURE.read_text())
    mission = data["missions"][data["expected"]["offerer_zone_index"]]
    docks = {d["name"]: d for d in data["docks"]["docks"]}
    route = [(w["lat"], w["lon"]) for w in mission["waypoints"]][route_from:]
    return ZoneInput(
        zone_index=0, altitude_m=altitude_m, dock_alt_m=dock_alt_m, use_terrain=use_terrain, route=route,
        docks=[(docks[n]["lat"], docks[n]["lon"], i + 1) for i, n in enumerate(("d1", "d2"))],
    )


class TestSearchGeometry:
    def test_range_is_the_sim_detection_gate(self, zr10, cam):
        # FinitePoiProjector: pixel gate OR max_detect_distance (first-zoom fy, x1.1 margin).
        assert cam.range_m == pytest.approx(829.10, abs=0.01)
        assert cam.fy == pytest.approx(1401.91, abs=0.01)  # first zoom, not the selected one
        real = search_cameras(zr10, sim_finite=False)[0]
        assert real.range_m == pytest.approx(1401.91 * 4.3012 / 8, rel=1e-3)

    def test_max_height_and_window_match_hand_numbers(self, cam):
        # steepest ray: 14 deg pitch + atan((720 - cy) / fy) = 28.03 deg
        d_max = math.radians(14.0) + math.atan((720 - cam.cy) / cam.fy)
        assert max_search_height(cam) == pytest.approx(cam.range_m * math.sin(d_max), abs=1e-6)
        assert max_search_height(cam) == pytest.approx(389.58, abs=0.01)
        near, far = search_window(cam, 260.0)
        assert near == pytest.approx(260.0 / math.tan(d_max), abs=1e-6)
        assert far == pytest.approx(math.sqrt(cam.range_m ** 2 - 260.0 ** 2), abs=1e-6)
        assert (near, far) == pytest.approx((488.45, 787.28), abs=0.01)
        assert search_window(cam, 403.0) is None
        # The fix text says "at most h_max": h_max itself still has a window.
        assert search_window(cam, max_search_height(cam)) is not None
        assert search_window(cam, max_search_height(cam) + 1e-6) is None

    def test_segment_interval_is_exact(self, cam):
        # Dock straight ahead at 1500 m: in view while its ground distance is in [near, far].
        near, far = search_window(cam, 260.0)
        lo, hi = segment_in_view(cam, 260.0, (0.0, 0.0), (3000.0, 0.0), (1500.0, 0.0))
        assert (lo, hi) == pytest.approx((1500.0 - far, 1500.0 - near), abs=1e-6)
        # A segment that ends 1 m into the window still sees it (a 2 m sampler would not).
        end = 1500.0 - far + 1.0
        lo, hi = segment_in_view(cam, 260.0, (0.0, 0.0), (end, 0.0), (1500.0, 0.0))
        assert hi - lo == pytest.approx(1.0, abs=1e-6)

    def test_zero_distance_is_not_in_view(self, cam):
        # Dock level with the aircraft at the route start, flying away: only the
        # zero-distance point passes the edge tests, and the projector rejects it.
        assert segment_in_view(cam, 0.0, (0.0, 0.0), (1000.0, 0.0), (0.0, 0.0)) is None

    def test_dock_abeam_is_never_in_view(self, cam):
        # 500 m to the right of a lane (the next lane at 500 m spacing): never in frame.
        assert segment_in_view(cam, 260.0, (0.0, 0.0), (3000.0, 0.0), (1500.0, 500.0)) is None


class TestCheckPlan:
    def test_dock_at_aircraft_height_is_checked_by_view_not_rejected(self, zr10, cam):
        # A dock level with the aircraft, 200 m ahead, projects inside the image (v about 20 px).
        assert segment_in_view(cam, 0.0, (0.0, 0.0), (100.0, 0.0), (300.0, 0.0)) is not None
        zone = _fixture_zone(150.0, dock_alt_m=150.0)
        result = check_plan(zr10, [zone], sim_mode=True)["zones"][0]
        assert all(p["kind"] != "no_search_window" for p in result["problems"])


    def test_fixture_zone_at_260_m_passes(self, zr10):
        result = check_plan(zr10, [_fixture_zone(260.0)], sim_mode=True)
        assert result["status"] == "ok"
        assert result["zones"][0]["problems"] == []

    def test_403_m_has_no_search_window(self, zr10):
        result = check_plan(zr10, [_fixture_zone(403.0)], sim_mode=True)
        assert result["status"] == "problems"
        (problem,) = result["zones"][0]["problems"]
        assert problem["kind"] == "no_search_window"
        assert problem["severity"] == "error"
        assert problem["max_height_m"] == pytest.approx(389.58, abs=0.01)
        assert problem["near_m"] == pytest.approx(757.09, abs=0.01)

    def test_dock_only_before_detection_start_is_never_in_view(self, zr10):
        # Detection starts at waypoint 5 (the third lane); d1/d2 are on the second lane.
        result = check_plan(zr10, [_fixture_zone(260.0, route_from=4)], sim_mode=True)
        kinds = [(p["kind"], p["wp"]) for p in result["zones"][0]["problems"]]
        assert kinds == [("dock_never_in_view", 1), ("dock_never_in_view", 2)]

    def test_dock_height_lowers_the_height_above_dock(self, zr10):
        # AAS_TARG_ALT 150: 403 m home-relative is 253 m above the dock.
        zone = check_plan(zr10, [_fixture_zone(403.0, dock_alt_m=150.0)], sim_mode=True)["zones"][0]
        assert zone["height_m"] == pytest.approx(253.0)
        assert zone["problems"] == []

    def test_unknown_dock_height_is_not_checked(self, zr10):
        zone = check_plan(zr10, [_fixture_zone(403.0, dock_alt_m=None)], sim_mode=True)["zones"][0]
        assert zone["checked"] is False
        assert zone["reason"] == "dock_alt_unknown"
        assert "problems" not in zone

    @pytest.mark.parametrize("use_terrain", [True, None])
    def test_terrain_on_or_unknown_is_an_estimate(self, zr10, use_terrain):
        zone = check_plan(zr10, [_fixture_zone(403.0, use_terrain=use_terrain)], sim_mode=True)["zones"][0]
        assert zone["estimate"] is True
        assert zone["problems"][0]["severity"] == "warning"

    def test_real_mode_is_an_estimate(self, zr10):
        zone = check_plan(zr10, [_fixture_zone(403.0)], sim_mode=False)["zones"][0]
        assert zone["problems"][0]["severity"] == "warning"
        assert zone["range_m"] == pytest.approx(1401.91 * 4.3012 / 8, rel=1e-3)

    def test_ideal_360_is_skipped(self, zr10):
        profile = {**zr10, "detector": {**zr10["detector"], "ideal_360": True}}
        result = check_plan(profile, [_fixture_zone(403.0)], sim_mode=True)
        assert result == {"status": "skipped", "reason": "ideal_360", "zones": []}

    def test_zone_without_docks_is_not_checked(self, zr10):
        zone = _fixture_zone(403.0)
        empty = ZoneInput(**{**zone.__dict__, "docks": []})
        result = check_plan(zr10, [empty], sim_mode=True)
        assert result["status"] == "ok"
        assert result["zones"][0]["reason"] == "no_docks"


class TestRoute:
    def test_route_uses_the_active_profile(self):
        settings = MagicMock()
        settings.simulation.sim_mode = True
        settings.camera.vision_profile = "siyi_zr10"
        zone = _fixture_zone(403.0)
        body = {"zones": [{
            "zone_index": 0, "altitude_m": 403.0, "dock_alt_m": 0.0, "use_terrain": False,
            "route": [{"lat": lat, "lon": lon} for lat, lon in zone.route],
            "docks": [{"lat": lat, "lon": lon, "wp": wp} for lat, lon, wp in zone.docks],
        }]}
        with patch("gcs.backend.routes.plan_camera_check.settings_store") as store:
            store.get.return_value = settings
            resp = TestClient(app).post("/api/plan-camera-check", json=body)
        assert resp.status_code == 200
        data = resp.json()
        assert data["profile"] == "siyi_zr10"
        assert data["status"] == "problems"
        assert data["zones"][0]["problems"][0]["kind"] == "no_search_window"


# Profiles whose dock flies at the UAV's altitude (horizontal camera), not on
# the ground: a ground-dock stack does not apply to them.
_CO_ALTITUDE_PROFILES = {"siyi_zr10_horizon"}


def test_default_stack_fits_each_ground_dock_camera():
    """The planner's default stack (dock preset at the bottom, one altitude
    separation per UAV above it) stays within reach of the profile's camera
    for a dock on the ground, so a plan built with no altitude edits can find
    its docks."""
    from gcs.backend.settings_model import FlightSettings

    flight = FlightSettings()
    profiles, _, _ = load_profiles()
    for name, profile in profiles.items():
        if name in _CO_ALTITUDE_PROFILES:
            continue
        base = profile["detector"]["dock_presets"]["dock"]["altitude_m"]
        top = base + (flight.uavs_per_set - 1) * flight.altitude_separation_m
        h_max = max(max_search_height(c) for c in search_cameras(profile, sim_finite=False))
        assert top <= h_max, f"{name}: stack top {top} m above camera reach {h_max:.0f} m"
