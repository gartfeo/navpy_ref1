"""Tests for FleetDockWorld: one fixed dock list shared by a fleet run."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from navpy.core.kernel.location import Location
from navpy.sim.detection import sim_render_composition
from navpy.sim.detection.sim_poi_catalog import SimPoiCatalog
from navpy.sim.world.fleet_dock_world import (
    FLEET_DOCKS_ENV,
    FleetDockFileError,
    FleetDockWorld,
    fleet_docks_path,
    load_fleet_docks,
    parse_fleet_docks,
)
from navpy.sim.world.poi_provider import PoiProvider


def _dock(name="d1", lat=40.31, lon=44.45, height_m=0.0, location_type=None):
    return {
        "name": name,
        "lat": lat,
        "lon": lon,
        "height_m": height_m,
        "location_type": location_type,
    }


def _payload(*docks):
    return {"schema_version": 1, "docks": list(docks) or [_dock()]}


def _write(tmp_path: Path, payload) -> Path:
    path = tmp_path / "docks.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def _vehicle(home=Location(40.3, 44.4, 1300.0)):
    return SimpleNamespace(home_location=home)


def _world(docks, vehicle=None):
    args = SimpleNamespace(use_terrain=False)
    return FleetDockWorld(docks, args, vehicle or _vehicle(), None, Mock())


def _terrain_vehicle(home):
    return SimpleNamespace(home_location=home, terrain_height_at=lambda lat, lng: 1310.0 + lat)


def test_vehicles_with_different_homes_see_the_same_world(tmp_path):
    path = _write(tmp_path, _payload(_dock("d1", height_m=2.0), _dock("d2", lat=40.32)))
    worlds = [
        FleetDockWorld(load_fleet_docks(path), SimpleNamespace(use_terrain=True),
                       _terrain_vehicle(home), None, Mock())
        for home in (Location(40.3, 44.4, 1300.0), Location(40.0, 44.0, 900.0))
    ]

    first, second = ([(p.uid, p.g_loc.lat, p.g_loc.lng, p.g_loc.alt) for p in w.pois] for w in worlds)
    assert first == second
    assert first[0][3] == pytest.approx(1310.0 + 40.31 + 2.0)


def test_peer_and_hub_hints_through_the_catalog_add_no_dock():
    world = _world(parse_fleet_docks(_payload(_dock("d1"), _dock("home", lat=40.4))))
    catalog = SimPoiCatalog(world)
    before = [(p.uid, p.g_loc.lat) for p in catalog.snapshot()]

    catalog.set_sim_poi(4, Location(40.5, 44.5, 0.0))
    catalog.set_sim_poi(5, Location(40.6, 44.6, 0.0), location_type="building")
    catalog.refresh()

    assert [(p.uid, p.g_loc.lat) for p in catalog.snapshot()] == before


def test_two_loads_give_the_same_ids(tmp_path):
    path = _write(tmp_path, _payload(_dock("d1"), _dock("d2", lat=40.32), _dock("home-a", lat=40.4)))
    first = _world(load_fleet_docks(path))
    second = _world(load_fleet_docks(path), _vehicle(Location(40.0, 44.0, 900.0)))

    assert [poi.uid for poi in first.pois] == [0, 1, 2]
    assert [(p.uid, p.g_loc.lat, p.g_loc.lng) for p in first.pois] == [
        (p.uid, p.g_loc.lat, p.g_loc.lng) for p in second.pois
    ]


def test_dock_sits_at_its_height_above_home_without_terrain():
    world = _world(parse_fleet_docks(_payload(_dock(height_m=2.0, location_type="vehicle"))))

    (poi,) = world.pois
    assert poi.g_loc.is_absolute
    assert poi.g_loc.alt == pytest.approx(1302.0)
    assert poi.height == 2.0
    assert poi.location_type == "vehicle"


def test_dock_follows_terrain_when_enabled():
    vehicle = SimpleNamespace(
        home_location=Location(40.3, 44.4, 1300.0),
        terrain_height_at=lambda lat, lng: 1325.0,
    )
    world = FleetDockWorld(
        parse_fleet_docks(_payload()),
        SimpleNamespace(use_terrain=True),
        vehicle,
        None,
        Mock(),
    )

    assert world.pois[0].g_loc.alt == pytest.approx(1325.0)


def test_set_sim_poi_is_ignored():
    world = _world(parse_fleet_docks(_payload()))
    before = list(world.pois)

    world.set_sim_poi(5, Location(41.0, 45.0, 0.0), location_type="other")

    assert world.pois == before


def test_no_docks_until_home_is_known_then_refresh_places_them():
    vehicle = _vehicle(home=None)
    world = _world(parse_fleet_docks(_payload()), vehicle)
    assert world.pois == []

    vehicle.home_location = Location(40.3, 44.4, 1300.0)
    world.refresh()

    assert [poi.uid for poi in world.pois] == [0]


@pytest.mark.parametrize(
    "payload",
    [
        {"schema_version": 2, "docks": [_dock()]},
        {"schema_version": True, "docks": [_dock()]},
        {"schema_version": 1, "docks": []},
        {"schema_version": 1},
        {"schema_version": 1, "docks": [_dock(), _dock()]},
        {"schema_version": 1, "docks": [{**_dock(), "extra": 1}]},
        {"schema_version": 1, "docks": [_dock(lat=float("nan"))]},
        {"schema_version": 1, "docks": [_dock(lat=91.0)]},
        {"schema_version": 1, "docks": [_dock(height_m=True)]},
        {"schema_version": 1, "docks": [_dock(name="")]},
        {"schema_version": 1, "docks": [_dock(location_type="")]},
    ],
)
def test_malformed_files_are_refused(payload):
    with pytest.raises(FleetDockFileError):
        parse_fleet_docks(payload)


def test_missing_file_is_refused(tmp_path):
    with pytest.raises(FleetDockFileError):
        load_fleet_docks(tmp_path / "absent.json")


def test_path_comes_from_the_environment():
    assert fleet_docks_path({}) is None
    assert fleet_docks_path({FLEET_DOCKS_ENV: "x/docks.json"}) == Path("x/docks.json")


def test_empty_environment_value_is_refused():
    with pytest.raises(FleetDockFileError):
        fleet_docks_path({FLEET_DOCKS_ENV: " "})


def _dependencies():
    vehicle = Mock()
    vehicle.home_location = Location(40.3, 44.4, 1300.0)
    vehicle.mission_items_count = 0
    vehicle.get_param_or_default.side_effect = lambda name, default: default
    args = SimpleNamespace(AAS_TARG_WPS="16", AAS_TARG_ALT=150, use_terrain=False)
    return SimpleNamespace(args=args, vehicle=vehicle, zc_util=None, logger=Mock())


def test_render_uses_fleet_docks_when_the_run_names_a_file(tmp_path, monkeypatch):
    monkeypatch.setenv(FLEET_DOCKS_ENV, str(_write(tmp_path, _payload())))

    world = sim_render_composition._build_poi_world(_dependencies())

    assert isinstance(world, FleetDockWorld)
    assert [poi.uid for poi in world.pois] == [0]


def test_render_keeps_waypoint_docks_without_a_file(monkeypatch):
    monkeypatch.delenv(FLEET_DOCKS_ENV, raising=False)

    world = sim_render_composition._build_poi_world(_dependencies())

    assert isinstance(world, PoiProvider)


def test_render_refuses_a_bad_file(tmp_path, monkeypatch):
    monkeypatch.setenv(FLEET_DOCKS_ENV, str(tmp_path / "absent.json"))

    with pytest.raises(FleetDockFileError):
        sim_render_composition._build_poi_world(_dependencies())


def test_real_render_pipeline_projects_a_file_dock(tmp_path, monkeypatch):
    from navpy.core.kernel.attitude import Attitude
    from navpy.core.kernel.config.uas_args import UasArgs
    from navpy.core.perception.geo.geo_ref_calc import GeoRefCalc
    from navpy.sim.detection.sim_render_composition import build_pipeline, build_render_foundation
    from tests.modules.vision.test_finite_projection_evidence import rig
    from tests.modules.vision.test_sim_frame_transactions import _coordinator

    _, camera, _, _, loc, poi, *_ = rig(None)
    dock = _dock("d1", lat=poi.g_loc.lat, lon=poi.g_loc.lng, height_m=2.0)
    monkeypatch.setenv(FLEET_DOCKS_ENV, str(_write(tmp_path, _payload(dock))))
    for key, value in dict(NAVPY_SIM_PROJECTION_RECORDER="1", NAVPY_SIM_EVIDENCE_DIR=str(tmp_path),
                           NAVPY_SIM_EVIDENCE_RUN_ID="fleet", NAVPY_SIM_EVIDENCE_MAX_RECORDS="20",
                           NAVPY_SIM_EVIDENCE_MAX_BYTES="100000").items():
        monkeypatch.setenv(key, value)
    mount = SimpleNamespace(get_k=camera.read_matrix, get_gimbal_data=camera.read_gimbal,
                            is_valid=camera.pixel_valid, image_width=1280, image_height=720,
                            name="camera", sync_zoom_from_hardware=lambda: False)
    vehicle = SimpleNamespace(source_system=3,
                              home_location=Location(40.0, 44.0, poi.g_loc.alt - 2.0))
    deps = SimpleNamespace(mount=mount, geo_ref=GeoRefCalc(UasArgs()), logger=Mock(),
                           args=SimpleNamespace(use_terrain=False), zc_util=None, vehicle=vehicle)
    options = SimpleNamespace(ideal_360=False, tracking_config=None, zoom_config=None,
                              sim_assets_path=None)
    coordinator, store, _ = _coordinator()
    source = SimpleNamespace(pose_source=SimpleNamespace(source_now_s=12.5), store=store,
                             record_outcome=Mock(), coordinator=coordinator, clock=Mock(),
                             poll_timer=None)

    render = build_render_foundation(deps, options, source)
    pipeline = build_pipeline(deps, options, source, render)

    assert isinstance(render.poi_provider._provider, FleetDockWorld)
    assert pipeline.detect_pois(loc, Attitude(0, 40, 0), frame_timestamp_s=12.5,
                                frame_generation=coordinator.token)
    render.evidence_recorder.close()
    rows = [json.loads(line) for line in render.evidence_recorder.output.path.read_text().splitlines()]
    projection = next(row for row in rows if row["kind"] == "projection")
    assert '"poi_uid": 0' in json.dumps(projection)
    assert f'"poi_uid": {poi.uid}' not in json.dumps(projection)
