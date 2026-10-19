"""The hand demo's d1 and d2 as owner waypoints: params, path and plan resolution."""

from __future__ import annotations

import importlib
from types import SimpleNamespace

import pytest
from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_DO_JUMP, MAV_FRAME_GLOBAL

from gcs.backend.mission_validator import parse_mission_items
from gcs.backend.models import VehicleAssignment
from gcs.backend.planner.mission_items import build_mission
from gcs.backend.routes.mission_upload_helpers import assignment_geometry
from navpy.plugins.vehicle.ardupilot.mission_store import MissionStore
from navpy.plugins.vehicle.ardupilot.nav_waypoints import nav_wp_indices

from scripts import delivery_demo  # noqa: F401  (repository bootstrap)
from scripts.delivery_demo_docks import DEMO_DOCKS, waypoint_dock_scenario

fixture_module = importlib.import_module("scripts.eval_delivery_fixture")
config = importlib.import_module("scripts.eval_gcs_demo_config")
models = importlib.import_module("scripts.eval_gcs_demo_models")
scenario_module = importlib.import_module("scripts.eval_gcs_demo_scenario")
missions_module = importlib.import_module("scripts.eval_gcs_demo_scenario_missions")

FIXTURE = fixture_module.load_fixture(fixture_module.FIXTURE_PATH)
EVAL = fixture_module.delivery_scenario(FIXTURE, fixture_module.FIXTURE_PATH)
DEMO = waypoint_dock_scenario(FIXTURE)
DOCKS = {dock["name"]: dock for dock in FIXTURE["docks"]["docks"]}
SYS_IDS = (1, 2, 3)


def _point(name):
    return {"lat": DOCKS[name]["lat"], "lon": DOCKS[name]["lon"]}


def test_d1_and_d2_become_owner_waypoints_on_the_same_path():
    owner = DEMO.owner_index
    track = DEMO.missions[owner]["waypoints"]

    assert DEMO.poi_nav_waypoint_ordinals == (4, 5)
    assert [track[o - 1] for o in DEMO.poi_nav_waypoint_ordinals] == [_point(n) for n in DEMO_DOCKS]
    assert [p for p in track if p not in map(_point, DEMO_DOCKS)] == EVAL.missions[owner]["waypoints"]
    assert [m for i, m in enumerate(DEMO.missions) if i != owner] == [
        m for i, m in enumerate(EVAL.missions) if i != owner
    ]
    # The fixture itself is not edited.
    assert len(FIXTURE["missions"][owner]["waypoints"]) == len(EVAL.missions[owner]["waypoints"])


def test_only_the_owner_marks_the_dock_waypoints():
    owner_sys_id = SYS_IDS[DEMO.owner_index]

    masks = [config.aas_params_for(DEMO, sys_id, owner_sys_id)["targ_wps"] for sys_id in SYS_IDS]

    assert masks[DEMO.owner_index] == 0b11000
    assert sorted(masks) == [0, 0, 0b11000]


def test_a_dock_off_every_owner_leg_is_refused():
    fixture = dict(FIXTURE, docks={"docks": [dict(DOCKS["d1"], lon=0.0), DOCKS["d2"]]})

    with pytest.raises(models.RegressionError, match="no owner track leg"):
        waypoint_dock_scenario(fixture)


def _onboard(assignment):
    """Encode the owner assignment as the upload route does; hold it as SITL does."""
    model = VehicleAssignment(**assignment)
    track, polygon, backbone, launch_point, hub = assignment_geometry(model)
    loader = build_mission(
        track, model.altitude_m, model.sys_id, model.corridor_count, model.search_pattern,
        polygon, backbone, launch_point, model.corridor_altitude_m, hub, 40.0,
    )
    for seq in range(loader.count()):
        if loader.wp(seq).command == MAV_CMD_DO_JUMP:
            loader.wp(seq).frame = MAV_FRAME_GLOBAL
    store = MissionStore(model.sys_id, message_store=None)
    store.replace(loader)
    return SimpleNamespace(
        get_mission_item=store.get,
        get_mission_item_location=store.get_location,
        mission_items_count=store.count,
    )


def test_the_plan_and_navpy_both_resolve_d1_and_d2_at_ordinals_4_and_5():
    owner = missions_module.bind_missions(DEMO.missions, SYS_IDS)[DEMO.owner_index]
    vehicle = _onboard(owner)
    parsed = parse_mission_items(vehicle, vehicle.mission_items_count, owner["sys_id"])

    plan = scenario_module.resolve_demo_plan(models.UavIds.from_values(SYS_IDS, 3), parsed, DEMO)

    assert [poi.nav_waypoint_ordinal for poi in plan.pois] == [4, 5]
    rows = nav_wp_indices(vehicle)
    for ordinal, name in zip((4, 5), DEMO_DOCKS):
        location = vehicle.get_mission_item_location(rows[ordinal - 1])
        assert (round(location.lat, 7), round(location.lng, 7)) == (DOCKS[name]["lat"], DOCKS[name]["lon"])
