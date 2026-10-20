"""``eval_gcs_navigation_demo.py --lockstep`` before any launch, and its operator seams (step 9g)."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts import eval_gcs_demo_cli as cli
from scripts import eval_gcs_demo_lockstep as lockstep_eval
from scripts.eval_gcs_demo_lockstep import LockstepOptions, execute_lockstep_run, manifest, require_wired
from scripts.eval_gcs_demo_constants import PARAM_REQUEST_BUDGET_S
from scripts.eval_gcs_demo_models import RegressionError, StackContext
from scripts.eval_gcs_demo_scenario import CERTIFIED_SCENARIO_PATH, load_scenario
from scripts.eval_gcs_demo_stack_ops import configure_vehicle_params
from scripts.lockstep_fleet_actor import NAMED_PARAMETER_NAMES, named_parameter_snapshot
from scripts.lockstep_wsl_sitl import BOOT_BUDGET_US, DRAIN_PERIOD_US, MAX_FRAME_GAP_US

ROOT = Path(__file__).resolve().parents[2]
SLICE_PATH = ROOT / "scripts" / "eval_lockstep_slice_scenario.json"
SLICE = load_scenario(SLICE_PATH)
CERTIFIED = load_scenario(CERTIFIED_SCENARIO_PATH)
OPTIONS = LockstepOptions(seed=1, timeout_s=600.0, confirmation_delay_s=15.0)


@pytest.fixture
def no_claim(monkeypatch):
    def claim(**_kwargs):
        raise AssertionError("a refused run must not claim a slot")

    monkeypatch.setattr(lockstep_eval.lockstep, "claim", claim)


def test_lockstep_runs_the_certified_scenario_up_to_the_slot_claim(tmp_path, monkeypatch, no_claim):
    """Step 10h: the certified fleet is no longer refused; its inputs name three SITLs."""
    import json

    monkeypatch.setattr(lockstep_eval.lockstep_wsl, "merged_defaults", lambda *_a: "X 1\n")
    with pytest.raises(AssertionError, match="must not claim"):
        execute_lockstep_run(tmp_path / "run", CERTIFIED, OPTIONS)
    inputs = json.loads((tmp_path / "run" / "input_manifest.json").read_text(encoding="utf-8"))
    assert len(inputs["sitl"]["topology"]["homes"]) == 3


def test_lockstep_refuses_a_profile_without_wiring(tmp_path, no_claim):
    import dataclasses

    unwired = dataclasses.replace(SLICE, gate_profile="unwired")
    with pytest.raises(RegressionError, match="gate profile 'unwired'"):
        execute_lockstep_run(tmp_path / "run", unwired, OPTIONS)
    assert not (tmp_path / "run").exists()


def test_lockstep_refuses_a_used_run_directory_and_a_timeout_past_the_backstop(tmp_path, no_claim):
    (tmp_path / "old.txt").write_text("x", encoding="utf-8")
    with pytest.raises(RegressionError, match="not empty"):
        execute_lockstep_run(tmp_path, SLICE, OPTIONS)
    with pytest.raises(RegressionError, match="--timeout-s must be within"):
        execute_lockstep_run(tmp_path / "new", SLICE, LockstepOptions(1, 3600.0, 15.0))


def test_cli_lockstep_refuses_a_navigation_speedup(tmp_path, monkeypatch, no_claim):
    monkeypatch.setattr(cli, "execute_lockstep_run", lambda *a: pytest.fail("ran"))
    with pytest.raises(RegressionError, match="no --navigation-speedup"):
        cli.main(["--lockstep", "--scenario", str(SLICE_PATH), "--run-dir", str(tmp_path),
                  "--navigation-speedup", "10"])


def test_cli_lockstep_with_analyze_only_launches_nothing(tmp_path, monkeypatch, no_claim):
    monkeypatch.setattr(cli, "execute_lockstep_run", lambda *a: pytest.fail("launched lockstep"))
    monkeypatch.setattr(cli, "execute_live_run", lambda *a, **k: pytest.fail("launched wall"))
    monkeypatch.setattr(cli, "analyze_run", lambda *a, **k: pytest.fail("analyzed"))
    with pytest.raises(RegressionError, match="exclusive"):
        cli.main(["--lockstep", "--analyze-only", str(tmp_path), "--scenario", str(SLICE_PATH)])


def test_cli_lockstep_reports_the_run_relative_log_dir(tmp_path, monkeypatch):
    run = lockstep_eval.LockstepRun("d" * 64, "operator_returned", [], "RegressionError: stalled")
    monkeypatch.setattr(cli, "execute_lockstep_run", lambda *_a: run)
    status = cli.main(["--lockstep", "--scenario", str(SLICE_PATH), "--run-dir", str(tmp_path)])
    report = (tmp_path / "report.json").read_text(encoding="utf-8")
    assert status == 1
    assert '"log_dir": "."' in report and str(tmp_path) not in report
    assert "lockstep outcome 'operator_returned', not 'ok'" in report
    assert "operator: RegressionError: stalled" in report


def test_lockstep_records_the_sim_profile_realization_in_the_input_manifest(tmp_path, monkeypatch,
                                                                           no_claim):
    """Steps 1-2 of deterministic_dev_sim.md: the default profile's draws are a run input;
    its SITL parameters go into the merged defaults and each SITL gets its noise seed."""
    import json

    from navpy.sim.lockstep.sim_profile import load_profile, realize
    from navpy.sim.lockstep.sitl_models import sitl_parameters, sitl_rng_seeds

    merged = []
    monkeypatch.setattr(lockstep_eval.lockstep_wsl, "merged_defaults",
                        lambda parameters: merged.append(parameters) or "X 1\n")
    with pytest.raises(AssertionError, match="must not claim"):
        execute_lockstep_run(tmp_path / "run", SLICE, OPTIONS)
    inputs = json.loads((tmp_path / "run" / "input_manifest.json").read_text(encoding="utf-8"))
    realization = realize(load_profile("hw"), OPTIONS.seed)
    assert inputs["run_options"]["sim_profile"] == realization.to_json()
    assert merged == [sitl_parameters(realization)] and "SIM_GPS_NOISE" in merged[0]
    assert inputs["sitl"]["profile_parameters"] == merged[0]
    assert inputs["sitl"]["rng_seeds"] == {"1": sitl_rng_seeds(realization, (1,))[1]}


def test_lockstep_launches_each_sitl_with_the_input_manifests_rng_seed(tmp_path, monkeypatch):
    """Step 2: the fleet boots with the defaults and per-sysid RNG seeds the manifest records."""
    import json

    launches = []

    def fleet(_run_dir, _slot, _owner, launch):
        launches.append(launch)
        raise AssertionError("launched")

    monkeypatch.setattr(lockstep_eval.lockstep_wsl, "merged_defaults", lambda *_a: "X 1\n")
    monkeypatch.setattr(lockstep_eval.lockstep, "claim", lambda **_k: (40, "owner"))
    monkeypatch.setattr(lockstep_eval, "WslFleet", fleet)
    with pytest.raises(AssertionError, match="launched"):
        execute_lockstep_run(tmp_path / "run", CERTIFIED, OPTIONS)
    inputs = json.loads((tmp_path / "run" / "input_manifest.json").read_text(encoding="utf-8"))
    (launch,) = launches
    assert launch.defaults == "X 1\n"
    assert len(inputs["sitl"]["rng_seeds"]) == 3
    assert {str(sysid): seed for sysid, seed in launch.rng_seeds.items()} == inputs["sitl"]["rng_seeds"]


def test_lockstep_refuses_an_unknown_sim_profile_before_the_run_directory(tmp_path, no_claim):
    import dataclasses

    with pytest.raises(RegressionError, match="--sim-profile: unknown sim profile 'bogus'"):
        execute_lockstep_run(tmp_path / "run", SLICE, dataclasses.replace(OPTIONS, sim_profile="bogus"))
    assert not (tmp_path / "run").exists()


def test_the_sim_profile_never_reaches_the_coordinators_manifest():
    """The journal digest comes from the Manifest's grid and the run's bytes; a profile
    with no built model must leave both as they were (step 1 keeps the digest)."""
    import dataclasses

    assert manifest(SLICE, dataclasses.replace(OPTIONS, sim_profile="none")) == manifest(SLICE, OPTIONS)


def test_cli_passes_the_sim_profile_and_defaults_to_hw(tmp_path, monkeypatch):
    seen = []

    def run(_run_dir, _scenario, options):
        seen.append(options.sim_profile)
        return lockstep_eval.LockstepRun("d" * 64, "ok", [], None)

    monkeypatch.setattr(cli, "execute_lockstep_run", run)
    monkeypatch.setattr(cli, "analyze_run", lambda *a, **k: pytest.fail("stop after the run"))
    for extra, run_dir in (([], "a"), (["--sim-profile", "stress"], "b")):
        try:
            cli.main(["--lockstep", "--scenario", str(SLICE_PATH), "--run-dir", str(tmp_path / run_dir),
                      *extra])
        except BaseException:  # noqa: BLE001 - the analysis stub ends the run
            pass
    assert seen == ["hw", "stress"]


def test_cli_refuses_a_sim_profile_without_lockstep(tmp_path, monkeypatch):
    monkeypatch.setattr(cli, "execute_live_run", lambda *a, **k: pytest.fail("launched wall"))
    with pytest.raises(RegressionError, match="--sim-profile is lockstep only"):
        cli.main(["--scenario", str(SLICE_PATH), "--run-dir", str(tmp_path), "--sim-profile", "hw"])


def test_cli_refuses_an_unknown_sim_profile_before_any_directory(monkeypatch, capsys):
    monkeypatch.setattr(cli.tempfile, "mkdtemp", lambda **_k: pytest.fail("made a run directory"))
    monkeypatch.setattr(cli, "execute_lockstep_run", lambda *a: pytest.fail("ran"))
    with pytest.raises(SystemExit):
        cli.main(["--lockstep", "--scenario", str(SLICE_PATH), "--sim-profile", "bogus"])
    assert "invalid choice: 'bogus'" in capsys.readouterr().err


def test_the_manifest_takes_the_grid_and_the_slice_roles():
    plan = manifest(SLICE, OPTIONS)
    assert (plan.max_frame_gap_us, plan.drain_period_us) == (MAX_FRAME_GAP_US, DRAIN_PERIOD_US)
    assert plan.mission_budget_us == lockstep_eval.MISSION_BUDGET_S * 1_000_000
    assert set(plan.roles) == {"backend", "navpy1"} and plan.companion_roles == ("navpy1",)
    assert plan.boot_budget_us == BOOT_BUDGET_US  # a SITL that never quiets ends the run


class _Api:
    websocket_url = "lockstep:events"

    def __init__(self) -> None:
        self.paths: list[str] = []

    def get_json(self, path, *, timeout=30.0):
        self.paths.append(path)
        if "/params/named" in path:
            values = {"PTCH_LIM_MIN_DEG": -25.0, "PTCH_LIM_MAX_DEG": 20.0, "AAS_DEL_THR": -1.0,
                      "TRIM_THROTTLE": 50.0, "ROLL_LIMIT_DEG": 45.0}
            return {"sys_id": 1, "params": [{"name": n, "value": v} for n, v in values.items()]}
        if path == "/api/vehicles":
            return [{"sys_id": 1, "link_ok": True}]
        raise AssertionError(f"unexpected read {path}")

    def put_json(self, path, payload, *, timeout=30.0):
        self.paths.append(path)
        return {"results": {name: True for name in payload["params"]}}


def test_command_bounds_come_from_named_reads_under_lockstep_never_mavftp():
    api = _Api()
    context = StackContext(
        api=api, sys_ids=(1,), backend_tail=None, clock=SimpleNamespace(), scenario=SLICE,
        event_stream_factory=None, navigation_tail=None, parameter_reader=named_parameter_snapshot,
    )
    context.clock = SimpleNamespace(monotonic=lambda: 0.0, sleep=lambda _s: None)
    bounds = configure_vehicle_params(context, 5.0)
    named = (f"/api/vehicles/1/params/named?names={','.join(NAMED_PARAMETER_NAMES)}"
             f"&timeout_s={PARAM_REQUEST_BUDGET_S}")  # the request names its budget (todo 97)
    assert named in api.paths
    assert not any("parameters" in path for path in api.paths)  # no MAVFTP read, no full-param write
    (vehicle,) = bounds.vehicles
    assert (vehicle.pitch_min_deg, vehicle.pitch_max_deg, vehicle.throttle) == (-25.0, 20.0, 0.5)


def test_require_wired_accepts_the_slice_and_the_certified_fleet():
    require_wired(SLICE)
    require_wired(CERTIFIED)


def test_the_manifest_hashes_the_merged_defaults_the_sitl_consumes(monkeypatch):
    """Codex 9g P2-1: WSL's stock plane.parm is an input; a changed stock default
    must change the manifest's SITL section, and the hashed text is what the root gets."""
    from scripts import lockstep_wsl
    from scripts.eval_gcs_demo_lockstep_inputs import lockstep_sitl_inputs

    stock = {"value": "ARSPD_FBW_MIN 9\nLOG_BACKEND_TYPE 1\n"}

    def wsl(script, *, timeout=60.0):
        return "/home/u" if "HOME" in script else stock["value"]

    monkeypatch.setattr(lockstep_wsl, "wsl", wsl)
    first = lockstep_wsl.merged_defaults()
    assert "ARSPD_FBW_MIN 9" in first and "LOG_BACKEND_TYPE 0" in first
    stock["value"] = "ARSPD_FBW_MIN 10\nLOG_BACKEND_TYPE 1\n"
    second = lockstep_wsl.merged_defaults()
    tiles = {"N40E044.DAT": b"tile"}
    assert lockstep_sitl_inputs(first, tiles)["defaults_parm_sha256"] != \
        lockstep_sitl_inputs(second, tiles)["defaults_parm_sha256"]
    assert lockstep_sitl_inputs(first, tiles) == lockstep_sitl_inputs(first, tiles)


def test_merged_defaults_take_the_profile_parameters_but_never_replace_a_lockstep_default(monkeypatch):
    """Step 2: the profile's wind and noise parameters join the defaults the SITL
    boots with; one that would replace a lockstep default is refused."""
    from scripts import lockstep_wsl

    monkeypatch.setattr(lockstep_wsl, "wsl", lambda script, **_k: "/home/u" if "HOME" in script
                        else "SIM_WIND_SPD 0\nARSPD_FBW_MIN 9\n")
    merged = lockstep_wsl.merged_defaults({"SIM_WIND_SPD": 6.5, "SIM_WIND_DIR": 270.0})
    assert "SIM_WIND_SPD 6.5" in merged and "SIM_WIND_DIR 270" in merged
    assert merged.count("SIM_WIND_SPD") == 1 and "ARSPD_FBW_MIN 9" in merged
    assert "SIM_WIND_DIR" not in lockstep_wsl.merged_defaults()
    with pytest.raises(ValueError, match="LOG_BACKEND_TYPE"):
        lockstep_wsl.merged_defaults({"LOG_BACKEND_TYPE": 1.0})


def test_the_manifest_hashes_the_terrain_tiles_the_sitl_root_gets():
    """Todo 46: the lockstep SITL's ground is the pinned terrain; a changed tile
    changes the manifest's SITL section, as the wall eval's root files do."""
    import hashlib

    from scripts.eval_gcs_demo_lockstep_inputs import lockstep_sitl_inputs
    from scripts.eval_gcs_demo_sitl_root import terrain_tiles

    tiles = terrain_tiles()
    assert set(tiles) == {"N40E044.DAT"}
    hashed = lockstep_sitl_inputs("X 1\n", tiles)["terrain_tiles_sha256"]
    assert hashed == {name: hashlib.sha256(data).hexdigest() for name, data in tiles.items()}
    changed = {name: data[:-1] + bytes([data[-1] ^ 1]) for name, data in tiles.items()}
    assert lockstep_sitl_inputs("X 1\n", changed)["terrain_tiles_sha256"] != hashed


def test_a_failed_lockstep_report_names_no_host_path(tmp_path, monkeypatch):
    """Codex 9g P2-6: an analysis exception quoting the run directory (and another
    host path) is scrubbed; two failing runs in different directories write equal bytes."""
    import json

    run = lockstep_eval.LockstepRun("d" * 64, "ok", [], None)
    monkeypatch.setattr(cli, "execute_lockstep_run", lambda *_a: run)

    def analyze(run_dir, **_kwargs):
        missing = run_dir.resolve() / "gcs" / "backend.log"
        raise OSError(f"cannot read {missing} (also {missing.as_posix()}) or C:/elsewhere/x.json")

    monkeypatch.setattr(cli, "analyze_run", analyze)
    reports = []
    for name in ("first-run", "second"):
        run_dir = tmp_path / name
        run_dir.mkdir()
        assert cli.main(["--lockstep", "--scenario", str(SLICE_PATH), "--run-dir", str(run_dir)]) == 1
        reports.append((run_dir / "report.json").read_bytes())
    assert reports[0] == reports[1]
    parsed = json.loads(reports[0])
    assert parsed["log_dir"] == "."
    assert parsed["errors"] == [
        "analysis: OSError: cannot read <run>/gcs/backend.log (also <run>/gcs/backend.log) or <abs>"]
    values = json.dumps(parsed)
    for host in (str(tmp_path), tmp_path.as_posix(), "elsewhere"):
        assert host not in values


def test_the_slice_manifest_is_unchanged_by_the_fleet_wiring():
    """10g-3: the manifest comes from the scenario's layout; for the slice it
    is the 1-UAV manifest built from the slice actor's constants (digest kept)."""
    from navpy.sim.lockstep.coordinator_types import Manifest
    from scripts.lockstep_slice_actor import BACKEND, ROUTER, SITL_PORTS, slice_roles
    from scripts.lockstep_slice_actor import SLICE as SLICE_LAYOUT
    from scripts.eval_certificate_source_time import SOURCE_TIME_SUBDIR

    assert manifest(SLICE, OPTIONS) == Manifest(
        seed=OPTIONS.seed, epoch_us=lockstep_eval.EPOCH_US, max_frame_gap_us=MAX_FRAME_GAP_US,
        drain_period_us=DRAIN_PERIOD_US, mission_budget_us=round(lockstep_eval.MISSION_BUDGET_S * 1_000_000),
        roles=slice_roles(SLICE, navpy=True, pose_cadence_dir=SOURCE_TIME_SUBDIR),
        sitl_ports=SITL_PORTS, router=ROUTER, initial=(("backend", BACKEND),),
        companion_roles=SLICE_LAYOUT.companion_roles, expected_fleet=SLICE_LAYOUT.expected_fleet,
        boot_budget_us=BOOT_BUDGET_US)


def test_the_certified_manifest_wires_three_uavs():
    plan = manifest(CERTIFIED, OPTIONS)
    assert set(plan.roles) == {"backend", "navpy1", "navpy2", "navpy3"}
    assert plan.companion_roles == ("navpy1", "navpy2", "navpy3") and plan.expected_fleet == (1, 2, 3)
    assert plan.sitl_ports == tuple((f"lockstep:serial0.{i}", f"sitl{i}.serial1") for i in (1, 2, 3))
    assert plan.initial == (("backend", 4),) and plan.router == "lockstep:router"
    assert set(plan.roles["backend"].config["companions"]) == {"1", "2", "3"}


def test_each_profile_names_its_homes_and_lines_files():
    from scripts.eval_gcs_demo_fleet_gates import navpy_lines_file
    from scripts.eval_gcs_demo_slice_gates import NAVPY_LINES_FILE
    from scripts.lockstep_topology import fleet_homes

    slice_wiring = lockstep_eval.WIRING[SLICE.gate_profile]
    certified = lockstep_eval.WIRING[CERTIFIED.gate_profile]
    assert slice_wiring.homes(SLICE, lockstep_eval.fleet_layout(SLICE)) == (SLICE.home,)
    assert slice_wiring.lines_file(1) == NAVPY_LINES_FILE
    homes = certified.homes(CERTIFIED, lockstep_eval.fleet_layout(CERTIFIED))
    assert homes == fleet_homes(CERTIFIED.home, 3) and len(set(homes)) == 3
    assert [certified.lines_file(i) for i in (1, 2, 3)] == [navpy_lines_file(i) for i in (1, 2, 3)]


@pytest.mark.parametrize("scenario", ["slice", "certified"])
def test_write_lines_writes_each_navpys_lines_with_their_ticks(tmp_path, scenario):
    import json

    chosen = SLICE if scenario == "slice" else CERTIFIED
    layout = lockstep_eval.fleet_layout(chosen)
    journal = SimpleNamespace(
        log_lines={i: [f"line {i}.a", f"line {i}.b"] for i in (*layout.sys_ids, layout.backend)},
        log_ticks={i: [i, 10 + i] for i in (*layout.sys_ids, layout.backend)})
    lockstep_eval.write_lines(tmp_path, chosen, SimpleNamespace(journal=journal))
    name = lockstep_eval.WIRING[chosen.gate_profile].lines_file
    assert sorted(p.name for p in tmp_path.iterdir()) == sorted(name(i) for i in layout.sys_ids)
    for sys_id in layout.sys_ids:
        rows = [json.loads(row) for row in (tmp_path / name(sys_id)).read_text(encoding="utf-8").splitlines()]
        assert rows == [[sys_id, f"line {sys_id}.a"], [10 + sys_id, f"line {sys_id}.b"]]
