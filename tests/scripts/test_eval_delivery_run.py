"""e4: the delivery fixture as a wall-eval scenario, and its roles from the logs."""

from __future__ import annotations

import json

import pytest

from scripts import eval_delivery_fixture as fixture
from scripts.eval_delivery_roles import HOME, OFFERER, WINNER, resolve_roles
from scripts.eval_gcs_demo_cli import main, parse_args
from scripts.eval_gcs_demo_gate_profiles import (
    DELIVERY_PROFILE,
    execution_profile,
    require_profile_shape,
)
from scripts.eval_gcs_demo_models import RegressionError
from scripts.eval_gcs_demo_scenario_missions import scenario_missions

EXPECTED = {
    "home_dock_by_zone": ["home-east", "home-middle", "home-west"],
    "offerer_zone_index": 0,
}
PEER = "Peer navigation started: task 2\n"
HOMING = "Default delivery hub navigation: home-middle\n"


def test_delivery_scenario_puts_the_offerer_zone_in_the_owner_slot():
    scenario = fixture.load_eval_scenario(fixture.FIXTURE_PATH)
    assert scenario.gate_profile == DELIVERY_PROFILE
    assert [(slot.name, slot.role) for slot in scenario.slots] == [
        ("zone-0", "owner"), ("zone-1", "peer"), ("zone-2", "peer"),
    ]
    assert scenario.owner_slot == "zone-0"
    assert scenario.poi_nav_waypoint_ordinals == ()
    assert len(scenario.missions) == 3


def test_scenario_flag_names_the_fixture():
    assert parse_args(["--scenario", "delivery"]).scenario == fixture.FIXTURE_PATH
    assert str(parse_args(["--scenario", "x.json"]).scenario) == "x.json"


def test_delivery_has_no_lockstep_run():
    with pytest.raises(RegressionError, match="no lockstep"):
        main(["--scenario", "delivery", "--lockstep"])
    with pytest.raises(ValueError):
        execution_profile(DELIVERY_PROFILE, "lockstep")
    assert execution_profile(DELIVERY_PROFILE, "wall") == DELIVERY_PROFILE


@pytest.mark.parametrize(
    ("roles", "ordinals"),
    [(["owner", "peer"], ()), (["owner", "owner", "peer"], ()), (["owner", "peer", "peer"], (1,))],
)
def test_delivery_shape_refuses_other_scenarios(roles, ordinals):
    with pytest.raises(ValueError):
        require_profile_shape(DELIVERY_PROFILE, roles, ordinals)


def test_scenario_missions_accept_no_ordinals():
    missions = json.loads(fixture.FIXTURE_PATH.read_text(encoding="utf-8"))["missions"]
    names = ["zone-0", "zone-1", "zone-2"]
    assert len(scenario_missions(missions, names, "zone-0", ())) == 3


def test_prepare_sim_docks_writes_the_world(tmp_path):
    scenario = fixture.load_eval_scenario(fixture.FIXTURE_PATH)
    path = fixture.prepare_sim_docks(tmp_path, scenario)
    written = json.loads(path.read_text(encoding="utf-8"))
    assert written == json.loads(fixture.FIXTURE_PATH.read_text(encoding="utf-8"))["docks"]


def test_prepare_sim_docks_refuses_a_used_run_dir(tmp_path):
    (tmp_path / "uav_1_navigation.log").write_text("", encoding="utf-8")
    scenario = fixture.load_eval_scenario(fixture.FIXTURE_PATH)
    with pytest.raises(RegressionError, match="already contains"):
        fixture.prepare_sim_docks(tmp_path, scenario)


def test_prepare_sim_docks_skips_other_profiles(tmp_path):
    from scripts.eval_gcs_demo_scenario import CERTIFIED_SCENARIO_PATH

    scenario = fixture.load_eval_scenario(CERTIFIED_SCENARIO_PATH)
    assert fixture.prepare_sim_docks(tmp_path, scenario) is None
    assert not any(tmp_path.iterdir())


@pytest.mark.parametrize("winner", [2, 3])
def test_roles_resolve_either_peer_as_winner(winner):
    home = 5 - winner
    texts = {1: "", winner: PEER, home: HOMING}
    roles = resolve_roles([1, 2, 3], texts, EXPECTED)
    assert roles.errors == []
    assert roles.roles == {1: OFFERER, winner: WINNER, home: HOME}
    assert roles.home_docks == {1: "home-east", 2: "home-middle", 3: "home-west"}


@pytest.mark.parametrize(
    ("texts", "error"),
    [
        ({1: "", 2: PEER, 3: PEER}, "exactly one winner"),
        ({1: "", 2: PEER, 3: ""}, "exactly one home"),
        ({1: PEER, 2: PEER, 3: HOMING}, "offerer 1 flew a peer task"),
        ({1: "", 2: PEER + HOMING, 3: ""}, "exactly one home"),
    ],
)
def test_roles_report_each_failure(texts, error):
    assert any(error in message for message in resolve_roles([1, 2, 3], texts, EXPECTED).errors)


def test_analysis_refuses_delivery_until_its_gates_exist(tmp_path):
    from scripts.eval_gcs_demo_analysis import analyze_run

    scenario = fixture.load_eval_scenario(fixture.FIXTURE_PATH)
    with pytest.raises(ValueError, match="no gates yet"):
        analyze_run(tmp_path, scenario=scenario)


class _Launched(Exception):
    pass


def _no_lock(monkeypatch):
    import contextlib

    from scripts import eval_gcs_demo_cli as cli

    monkeypatch.setattr(cli, "exclusive_evaluator_lock", contextlib.nullcontext)
    return cli


def test_cli_passes_the_dock_file_to_the_live_run(monkeypatch, tmp_path):
    cli = _no_lock(monkeypatch)
    seen = {}

    def live_run(run_dir, *_args, sim_docks, **_kwargs):
        seen["docks"] = sim_docks
        raise _Launched

    monkeypatch.setattr(cli, "execute_live_run", live_run)
    with pytest.raises(_Launched):
        main(["--scenario", "delivery", "--run-dir", str(tmp_path)])
    assert seen["docks"] == tmp_path / fixture.DOCK_FILE_NAME


def test_cli_refuses_a_runtime_mismatch_before_launch(monkeypatch, tmp_path):
    cli = _no_lock(monkeypatch)
    monkeypatch.setattr(cli, "execute_live_run", lambda *_a, **_k: pytest.fail("launched"))
    real = fixture.load_fixture

    def drifted(path):
        payload = real(path)
        return {**payload, "runtime": {**payload["runtime"], "search_zoom": 2}}

    monkeypatch.setattr(fixture, "load_fixture", drifted)
    with pytest.raises(RegressionError, match="fixture runtime"):
        main(["--scenario", "delivery", "--run-dir", str(tmp_path)])


def test_runtime_refuses_a_fractional_zoom(monkeypatch):
    scenario = fixture.load_eval_scenario(fixture.FIXTURE_PATH)
    real = fixture.build_settings

    def fractional(*args):
        settings = real(*args)
        settings["camera"]["vision_zoom"] = "1.5"
        return settings

    monkeypatch.setattr(fixture, "build_settings", fractional)
    with pytest.raises(RegressionError, match="fixture runtime"):
        fixture.require_fixture_runtime(fixture.load_fixture(fixture.FIXTURE_PATH), scenario)


def test_live_run_puts_the_dock_file_in_every_process_env(monkeypatch, tmp_path):
    from scripts import eval_gcs_demo_runtime as runtime
    from scripts.eval_gcs_demo_runtime import FLEET_DOCKS_ENV

    scenario = fixture.load_eval_scenario(fixture.FIXTURE_PATH)
    docks = fixture.prepare_sim_docks(tmp_path, scenario)
    seen = {}

    def launch(env, *_args, **_kwargs):
        seen["env"] = dict(env)
        raise _Launched

    monkeypatch.setattr(runtime, "dem_tiles", lambda *_: None)
    monkeypatch.setattr(runtime, "write_input_manifest", lambda *_: None)
    monkeypatch.setattr(runtime, "with_owned_stack", launch)
    with pytest.raises(_Launched):
        runtime.execute_live_run(tmp_path, 1.0, scenario=scenario, sim_docks=docks)
    assert seen["env"][FLEET_DOCKS_ENV] == str(docks.resolve())
