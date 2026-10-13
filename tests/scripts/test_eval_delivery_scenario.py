"""The delivery 3-UAV eval fixture: planner output, encoding and who cannot see what.

The visibility checks are an offline oracle. They prove who CANNOT see a dock:
a dock farther than the detection limit from every point a route flies after
its detection waypoint cannot be detected from that route. They cannot prove
that the offerer does see both docks; the 1-UAV SITL slice (e2b) does that.
"""

from __future__ import annotations

import json
import math
import shutil

import pytest

from navpy.sim.world.fleet_dock_world import parse_fleet_docks
from scripts import eval_delivery_scenario_gen as gen
from scripts.eval_gcs_demo_scenario_missions import _mission, _require_equal_search_paths

FIXTURE = json.loads(gen.FIXTURE_PATH.read_text(encoding="utf-8"))
CLEARANCE_M = gen.DETECT_LIMIT_M + gen.CLEARANCE_MARGIN_M
SAMPLE_STEP_M = 10.0


def _samples(points: list[dict[str, float]]) -> list[dict[str, float]]:
    out = []
    for a, b in zip(points, points[1:]):
        steps = max(1, math.ceil(gen.ground_distance_m(a, b) / SAMPLE_STEP_M))
        out.extend(
            {"lat": a["lat"] + (b["lat"] - a["lat"]) * i / steps,
             "lon": a["lon"] + (b["lon"] - a["lon"]) * i / steps}
            for i in range(steps + 1)
        )
    return out


def _flown(mission: dict) -> list[dict[str, float]]:
    """Search track plus the leg home: everything after the detection waypoint."""
    hub = mission["default_delivery_hub"]
    return _samples([*mission["waypoints"], {"lat": hub["lat"], "lon": hub["lon"]}])


def _search(mission: dict) -> list[dict[str, float]]:
    return _samples(mission["waypoints"])


def _closest_m(dock: dict, path: list[dict[str, float]]) -> float:
    return min(gen.ground_distance_m(dock, point) for point in path)


DOCKS = {dock["name"]: dock for dock in FIXTURE["docks"]["docks"]}
MISSIONS = FIXTURE["missions"]
EXPECTED = FIXTURE["expected"]
OFFERER = EXPECTED["offerer_zone_index"]


@pytest.mark.skipif(shutil.which("node") is None, reason="the GCS planner needs Node")
def test_fixture_is_what_the_gcs_planner_makes_today():
    assert gen.render(gen.build_fixture()) == gen.FIXTURE_PATH.read_text(encoding="utf-8")


@pytest.mark.skipif(shutil.which("node") is None, reason="the GCS planner needs Node")
def test_planner_output_moves_at_most_the_encoding_tolerance():
    raw = gen._run_planner(gen.INPUTS)
    for zone, mission in zip(raw["plan"]["zones"], MISSIONS):
        assert len(zone["track"]) == len(mission["waypoints"])
        for point, fixed in zip(zone["track"], mission["waypoints"]):
            assert abs(point["lat"] - fixed["lat"]) <= gen.ENCODING_TOLERANCE_DEG + 1e-12
            assert abs(point["lon"] - fixed["lon"]) <= gen.ENCODING_TOLERANCE_DEG + 1e-12


def test_runtime_settings_the_oracle_relies_on_are_recorded():
    assert FIXTURE["runtime"] == {"nav_last_wp": 1, "nav_oneshot": True, "search_zoom": 1}


def test_routes_are_equal_north_south_lines():
    assert FIXTURE["plan"]["grid_angle_deg"] == 90
    for mission in MISSIONS:
        for a, b in gen._route_lines(mission["waypoints"]):
            assert abs(a["lon"] - b["lon"]) <= 1e-7


def test_missions_pass_the_eval_upload_checks():
    for index, mission in enumerate(MISSIONS):
        _mission(mission, f"mission {index}")
    _require_equal_search_paths(MISSIONS)


def test_dock_list_is_a_valid_fleet_dock_file():
    specs = parse_fleet_docks(FIXTURE["docks"])
    assert [spec.name for spec in specs][:2] == [EXPECTED["kept_dock"], EXPECTED["offered_dock"]]


def test_each_route_has_its_own_home_dock_at_its_mission_hub():
    names = EXPECTED["home_dock_by_zone"]
    assert len(set(names)) == len(MISSIONS)
    for name, mission in zip(names, MISSIONS):
        hub = mission["default_delivery_hub"]
        assert (DOCKS[name]["lat"], DOCKS[name]["lon"]) == (hub["lat"], hub["lon"])


@pytest.mark.parametrize("dock", ["d1", "d2"])
def test_delivery_docks_lie_on_the_offerer_route(dock):
    assert _closest_m(DOCKS[dock], _search(MISSIONS[OFFERER])) < SAMPLE_STEP_M


@pytest.mark.parametrize("dock", ["d1", "d2"])
def test_no_other_route_can_see_a_delivery_dock(dock):
    for index, mission in enumerate(MISSIONS):
        if index != OFFERER:
            assert _closest_m(DOCKS[dock], _flown(mission)) >= CLEARANCE_M, index


def test_delivery_dock_spacing_fits_the_view_band():
    """Spacing only: 100-200 m fits a view band about 340 m deep at 260 m.

    Whether both are projected together, and d2 offered, is for the e2b slice.
    """
    gap = gen.ground_distance_m(DOCKS["d1"], DOCKS["d2"])
    assert 100.0 <= gap <= 200.0


@pytest.mark.parametrize("zone", range(3))
def test_no_search_route_can_see_a_home_dock(zone):
    home = DOCKS[EXPECTED["home_dock_by_zone"][zone]]
    for mission in MISSIONS:
        assert _closest_m(home, _search(mission)) >= CLEARANCE_M


@pytest.mark.parametrize("zone", range(3))
def test_no_leg_home_passes_another_home_dock(zone):
    own = EXPECTED["home_dock_by_zone"][zone]
    for name in EXPECTED["home_dock_by_zone"]:
        if name != own:
            assert _closest_m(DOCKS[name], _flown(MISSIONS[zone])) >= CLEARANCE_M, name


def test_home_docks_are_far_from_the_delivery_docks():
    for name in EXPECTED["home_dock_by_zone"]:
        for dock in ("d1", "d2"):
            assert gen.ground_distance_m(DOCKS[name], DOCKS[dock]) >= 2 * CLEARANCE_M


def test_encodable_values_survive_the_builder():
    for value in (40.287150025, 44.4691667, 40.3117414, -33.123456789):
        encoded = gen.encodable(value)
        assert int(encoded * 1e7) / 1e7 == encoded
        assert abs(encoded - value) <= gen.ENCODING_TOLERANCE_DEG + 1e-12


def test_any_flight_to_d2_passes_d1():
    """Known and accepted: the winner sees d1 near d2 (design risk).

    Peer acquisition stays in DETECT, where other docks can still be selected and
    offered, so it may lock d1 or offer d1 as a new task. e5 adds a mechanism test
    for each; the 3-UAV run checks both.
    """
    assert gen.ground_distance_m(DOCKS["d1"], DOCKS["d2"]) < gen.DETECT_LIMIT_M
