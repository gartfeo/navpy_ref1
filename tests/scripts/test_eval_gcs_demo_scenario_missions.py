"""The demo eval's scenario missions: upload, readback, POI resolution, home."""

from __future__ import annotations

import copy
import importlib
import json
import math
from types import SimpleNamespace

import pytest
from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_DO_JUMP, MAV_CMD_NAV_TAKEOFF, MAV_FRAME_GLOBAL

from gcs.backend.mission_validator import parse_mission_items
from gcs.backend.models import VehicleAssignment
from gcs.backend.planner.mission_items import build_mission
from gcs.backend.routes.mission_upload_helpers import assignment_geometry
from navpy.plugins.vehicle.ardupilot.mission_store import MissionStore
from navpy.plugins.vehicle.ardupilot.nav_waypoints import nav_wp_indices

from scripts import eval_gcs_navigation_demo  # noqa: F401  (repository bootstrap)
from scripts.eval_gcs_demo_execution import SystemClock

scenario = importlib.import_module("scripts.eval_gcs_demo_scenario")
scenario_missions = importlib.import_module("scripts.eval_gcs_demo_scenario_missions")
upload = importlib.import_module("scripts.eval_gcs_demo_upload")
mission_upload = importlib.import_module("scripts.eval_gcs_demo_mission_upload")
models = importlib.import_module("scripts.eval_gcs_demo_models")
mission = importlib.import_module("scripts.eval_gcs_demo_mission")
config = importlib.import_module("scripts.eval_gcs_demo_config")
process = importlib.import_module("scripts.eval_gcs_demo_process")

CERTIFIED = scenario.load_scenario(scenario.CERTIFIED_SCENARIO_PATH)

SYS_IDS = (7, 4, 9)
TAKEOFF_ALTITUDE_M = 40.0  # settings default the eval leaves unset


def _onboard(assignment: dict) -> SimpleNamespace:
    """Encode one assignment as the upload route does; hold it as the vehicle does."""
    model = VehicleAssignment(**assignment)
    track, polygon, backbone, launch_point, hub = assignment_geometry(model)
    loader = build_mission(
        track,
        model.altitude_m,
        model.sys_id,
        model.corridor_count,
        model.search_pattern,
        polygon,
        backbone,
        launch_point,
        model.corridor_altitude_m,
        hub,
        TAKEOFF_ALTITUDE_M,
    )
    for seq in range(loader.count()):
        # ArduPilot keeps no frame for the skip jump and reports MAV_FRAME_GLOBAL.
        if loader.wp(seq).command == MAV_CMD_DO_JUMP:
            loader.wp(seq).frame = MAV_FRAME_GLOBAL
    store = MissionStore(model.sys_id, message_store=None)
    store.replace(loader)
    return SimpleNamespace(
        get_mission_item=store.get,
        get_mission_item_location=store.get_location,
        mission_items_count=store.count,
    )


def _parsed(assignment: dict) -> tuple[SimpleNamespace, dict]:
    vehicle = _onboard(assignment)
    return vehicle, parse_mission_items(
        vehicle,
        vehicle.mission_items_count,
        assignment["sys_id"],
    )


def _write_manifest(tmp_path, edit) -> object:
    raw = json.loads(scenario.CERTIFIED_SCENARIO_PATH.read_text(encoding="utf-8"))
    edit(raw)
    path = tmp_path / "scenario.json"
    path.write_text(json.dumps(raw), encoding="utf-8")
    return path


def test_scenario_binds_one_mission_per_slot_in_upload_api_shape():
    manifest = CERTIFIED

    assignments = scenario_missions.bind_missions(manifest.missions, SYS_IDS)

    assert manifest.poi_nav_waypoint_ordinals == (2, 3, 6)
    assert [(a["sys_id"], a["zone_index"]) for a in assignments] == [(7, 0), (4, 1), (9, 2)]
    for assignment in assignments:
        VehicleAssignment(**assignment)  # extra="forbid": the exact API shape
    assignments[0]["waypoints"].clear()
    assert manifest.missions[0]["waypoints"], "bindings must not alias the manifest"


def test_every_scenario_mission_reads_back_exactly_after_encode_and_parse():
    manifest = CERTIFIED

    for assignment in scenario_missions.bind_missions(manifest.missions, SYS_IDS):
        _vehicle, parsed = _parsed(assignment)
        readback = upload.mission_assignment(
            parsed,
            expected_sys_id=assignment["sys_id"],
            zone_index=assignment["zone_index"],
        )

        assert readback == assignment


def test_resolved_pois_are_the_mission_items_navpy_selects():
    manifest = CERTIFIED
    owner = scenario_missions.bind_missions(manifest.missions, SYS_IDS)[manifest.owner_index]
    vehicle, parsed = _parsed(owner)

    plan = scenario.resolve_demo_plan(models.UavIds.from_values(SYS_IDS, 3), parsed, CERTIFIED)

    navpy_rows = nav_wp_indices(vehicle)
    # NavPy also counts the appended hub row; the GCS parser drops it.
    assert len(navpy_rows) == len(owner["waypoints"]) + 1
    gcs_rows = {row["nav_waypoint_ordinal"]: row for row in parsed["waypoints"]}
    for poi in plan.pois:
        expected = owner["waypoints"][poi.nav_waypoint_ordinal - 1]
        assert (poi.lat, poi.lon) == (expected["lat"], expected["lon"])
        assert (
            navpy_rows[poi.nav_waypoint_ordinal - 1]
            == gcs_rows[poi.nav_waypoint_ordinal]["mission_sequence"]
        )


def test_manifest_rejects_a_poi_ordinal_on_the_appended_hub_row(tmp_path):
    def drop_last_owner_waypoint(raw):
        # Every mission loses its last row so the equal-search-path check still
        # passes and the owner's ordinal 6 lands on the appended hub row.
        for mission in raw["missions"]:
            mission["waypoints"].pop()

    path = _write_manifest(tmp_path, drop_last_owner_waypoint)

    with pytest.raises(ValueError, match="default-delivery-hub row"):
        scenario.load_scenario(path)


def test_scenario_search_paths_are_equal():
    lengths = [
        scenario_missions.search_path_m(
            mission["waypoints"], scenario_missions.planner_ref_lat(mission["polygon"])
        )
        for mission in CERTIFIED.missions
    ]

    assert max(lengths) - min(lengths) < 0.01


def _haversine_m(a, b):
    lat_a, lat_b = math.radians(a["lat"]), math.radians(b["lat"])
    h = (
        math.sin((lat_b - lat_a) / 2) ** 2
        + math.cos(lat_a) * math.cos(lat_b) * math.sin(math.radians(b["lon"] - a["lon"]) / 2) ** 2
    )
    return 2 * 6_371_000.0 * math.asin(math.sqrt(h))


def _one_lane_mission(lat, west_lon, east_lon):
    mission = copy.deepcopy(dict(CERTIFIED.missions[0]))
    mission["waypoints"] = [{"lat": lat, "lon": east_lon}, {"lat": lat, "lon": west_lon}]
    return mission


def test_equal_planner_lanes_at_different_latitudes_pass_and_a_short_lane_fails():
    # The planner's lanes are equal in its flat frame; on the sphere the
    # northern lane is shorter by more than the encoding tolerance.
    south = _one_lane_mission(40.3032, 44.4332, 44.453)
    north = _one_lane_mission(40.3096, 44.4332, 44.453)
    sphere_gap = _haversine_m(*south["waypoints"]) - _haversine_m(*north["waypoints"])
    assert sphere_gap > scenario_missions._SEARCH_PATH_TOLERANCE_M_PER_ROW * 2

    scenario_missions._require_equal_search_paths([south, north])

    # Ends the northern lane 0.00001 degree of longitude (about 0.85 m) short.
    short = _one_lane_mission(40.3096, 44.43321, 44.453)
    with pytest.raises(ValueError, match="search paths must be equal"):
        scenario_missions._require_equal_search_paths([south, short])


def test_manifest_rejects_a_peer_polygon_that_rescales_a_short_path(tmp_path):
    def shorten_peer_and_move_its_polygon(raw):
        # Codex counterexample: about 0.17 m short in the shared frame (over the
        # tolerance), equal if peer 1 were measured from its own polygon's centre.
        raw["missions"][1]["waypoints"][-1]["lon"] = 44.433202
        for point in raw["missions"][1]["polygon"]:
            if point["lat"] == 40.3028:
                point["lat"] = 40.298253

    def shorten_peer(raw):
        raw["missions"][1]["waypoints"][-1]["lon"] = 44.433202

    (tmp_path / "short").mkdir()
    (tmp_path / "moved").mkdir()
    with pytest.raises(ValueError, match="search paths must be equal"):
        scenario.load_scenario(_write_manifest(tmp_path / "short", shorten_peer))
    with pytest.raises(ValueError, match="share the planner's area polygon"):
        scenario.load_scenario(_write_manifest(tmp_path / "moved", shorten_peer_and_move_its_polygon))


def test_manifest_rejects_unequal_search_paths(tmp_path):
    def shorten_peer_line(raw):
        # Ends peer 1's last east-west line 0.0010 degree of longitude (about 85 m) short.
        raw["missions"][1]["waypoints"][-1]["lon"] = 44.4342

    path = _write_manifest(tmp_path, shorten_peer_line)

    with pytest.raises(ValueError, match="search paths must be equal"):
        scenario.load_scenario(path)


def test_manifest_rejects_a_coordinate_the_encoder_would_move(tmp_path):
    def unencodable(raw):
        # int(40.302322 * 1e7) truncates to 403023219.
        raw["missions"][0]["waypoints"][2]["lat"] = 40.302322

    path = _write_manifest(tmp_path, unencodable)

    with pytest.raises(ValueError, match="does not survive mission-item encoding"):
        scenario.load_scenario(path)


@pytest.mark.parametrize(
    "edit, message",
    [
        (lambda raw: raw.update(schema_version=1), "schema_version must be 2"),
        (lambda raw: raw.pop("missions"), "keys must be"),
        (lambda raw: raw["missions"].pop(), "exactly one mission per slot"),
        (lambda raw: raw["missions"][1].update(extra=1), "keys must be"),
        (lambda raw: raw.update(home="40.3,44.4"), "LAT,LON,ALT,HEADING"),
        (lambda raw: raw["missions"][0].update(altitude_m=240.1), "float32"),
        (lambda raw: raw.update(poi_nav_waypoint_ordinals=[2, 3, 7]), r"must be \[2, 3, 6]"),
    ],
)
def test_manifest_schema_is_strict(tmp_path, edit, message):
    path = _write_manifest(tmp_path, edit)

    with pytest.raises(ValueError, match=message):
        scenario.load_scenario(path)


def _upload_context(monkeypatch, tamper=lambda sys_id, parsed: None):
    """GET /mission answers with each scenario mission encoded and parsed for real."""
    expected = scenario_missions.bind_missions(
        CERTIFIED.missions,
        SYS_IDS,
    )
    onboard = {}
    for assignment in expected:
        _vehicle, parsed = _parsed(assignment)
        tamper(assignment["sys_id"], parsed)
        onboard[f"/api/vehicles/{assignment['sys_id']}/mission"] = parsed
    uploaded = []
    monkeypatch.setattr(
        mission_upload,
        "upload_mission_assignments",
        lambda _context, assignments: uploaded.append(assignments),
    )
    api = SimpleNamespace(get_json=lambda path, timeout: copy.deepcopy(onboard[path]))
    context = SimpleNamespace(sys_ids=SYS_IDS, api=api, clock=SystemClock(), scenario=CERTIFIED)
    return context, expected, uploaded


def test_upload_accepts_the_scenario_missions_read_back_onboard(monkeypatch):
    context, expected, uploaded = _upload_context(monkeypatch)

    mission_upload.upload_scenario_missions(context)

    assert uploaded == [expected]


def _set_owner_third_altitude(sys_id, parsed):
    if sys_id == 7:
        parsed["waypoints"][2]["alt"] = 241.0


def _drift_last_vehicle(sys_id, parsed):
    if sys_id == 9:
        parsed["waypoints"][0]["lat"] = round(parsed["waypoints"][0]["lat"] - 1e-7, 7)


@pytest.mark.parametrize(
    "tamper, sys_id",
    [(_set_owner_third_altitude, 7), (_drift_last_vehicle, 9)],
    ids=["non-first-row-altitude", "coordinate"],
)
def test_upload_rejects_an_onboard_mission_that_differs(monkeypatch, tamper, sys_id):
    context, _expected, _uploaded = _upload_context(monkeypatch, tamper)

    with pytest.raises(models.RegressionError, match=f"vehicle {sys_id} onboard mission differs"):
        mission_upload.upload_scenario_missions(context)


def _raw_row(parsed, command):
    return next(row for row in parsed["items"] if row["command"] == command)


def _raise_owner_takeoff(sys_id, parsed):
    if sys_id == 7:
        _raw_row(parsed, MAV_CMD_NAV_TAKEOFF)["z"] = 60.0


def _retarget_skip_jump(sys_id, parsed):
    if sys_id == 4:
        _raw_row(parsed, MAV_CMD_DO_JUMP)["param1"] += 1.0


def _drop_last_metadata_row(sys_id, parsed):
    if sys_id == 9:
        parsed["items"].pop()


@pytest.mark.parametrize(
    "tamper, sys_id",
    [(_raise_owner_takeoff, 7), (_retarget_skip_jump, 4), (_drop_last_metadata_row, 9)],
    ids=["takeoff-altitude", "skip-jump-target", "missing-metadata-row"],
)
def test_upload_rejects_raw_rows_the_parsed_readback_skips(monkeypatch, tamper, sys_id):
    # Takeoff, the skip jump and metadata never reach the parsed waypoints, so
    # only the raw rows can show the onboard mission is the intended one.
    context, _expected, _uploaded = _upload_context(monkeypatch, tamper)

    with pytest.raises(models.RegressionError, match=f"vehicle {sys_id} onboard mission row differs"):
        mission_upload.upload_scenario_missions(context)


def test_parsed_mission_carries_every_stored_row_after_home():
    assignment = scenario_missions.bind_missions(CERTIFIED.missions, SYS_IDS)[0]
    vehicle, parsed = _parsed(assignment)
    assert [row["seq"] for row in parsed["items"]] == list(range(1, vehicle.mission_items_count))
    assert parsed["items"] == mission_upload.expected_items(assignment)


def test_companions_start_once_after_configuration_and_upload(monkeypatch):
    calls = []

    class Stop(Exception):
        pass

    def record(name, result=None):
        def step(*_args, **_kwargs):
            calls.append(name)
            return result
        return step

    def start(*_args, **_kwargs):
        calls.append("start_companions")
        raise Stop

    monkeypatch.setattr(mission, "connect_vehicles", record("connect_vehicles"))
    monkeypatch.setattr(mission, "wait_for_vehicles", record("wait_for_vehicles"))
    monkeypatch.setattr(mission, "configure_vehicle_params", record("configure_vehicle_params"))
    monkeypatch.setattr(mission, "upload_scenario_missions", record("upload_scenario_missions"))
    monkeypatch.setattr(mission, "verify_aas_profile", record("verify_aas_profile"))
    monkeypatch.setattr(mission, "start_companions", start)

    with pytest.raises(Stop):
        mission.run_mission(SimpleNamespace(sys_ids=SYS_IDS), None, 1.0, 0.0)

    # Companions read the parameters and the mission when they start, so they
    # start once, after both, and are never restarted.
    assert calls == [
        "connect_vehicles",
        "wait_for_vehicles",
        "configure_vehicle_params",
        "upload_scenario_missions",
        "verify_aas_profile",
        "start_companions",
    ]


def test_eval_launch_pins_the_scenario_home(monkeypatch, tmp_path):
    commands = []
    monkeypatch.setattr(process, "owned_entry", lambda _env: None)

    def run_process(command, _env, *, timeout_s):
        commands.append(command)
        return SimpleNamespace(returncode=1, stdout="", stderr="stopped by test")

    monkeypatch.setattr(process, "run_process", run_process)

    with pytest.raises(models.RegressionError, match="stopped by test"):
        process.launch_stack(
            {"GCS_LOG_DIR": str(tmp_path / "gcs"), "NAVPY_LOG_DIR": str(tmp_path)},
            1.0,
            clock=SystemClock(),
            scenario=CERTIFIED,
        )

    home = CERTIFIED.home
    assert commands[0][-2:] == ["--home", home]
    assert commands[0][3:5] == ["-n", "3"]
    assert config.launch_command()[-1] == "--headless"
