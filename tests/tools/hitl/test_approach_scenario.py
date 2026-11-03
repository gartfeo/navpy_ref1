"""Step 14's scenario isolation (tools/hitl/scenario_service.py, scenario_scoring.py, scenario_public.py and the
approach runner, approach_scenario.py): each seed
reproduces its scene hash, the dock pose reaches only Unreal, contact and scoring, and the command side neither reads
it nor changes with it. The runner tests run the real service in its own process; Unreal and the chain are fakes."""
from __future__ import annotations

import ast
import builtins
import io
import json
import os
from pathlib import Path

import pytest

from tools.hitl import scenario_public as sp
from tools.hitl import scenario_scoring as scoring
from tools.hitl import scenario_service as service
from tools.hitl.json_checks import RecordError
from tools.tuning import approach
from tools.tuning.approach_lane import LaneSpec, build_lane, lane_items
from tools.tuning.dock_pass import Point

REPO = Path(__file__).resolve().parents[3]
HOME = Point(40.0, 44.0)
POSE_KEYS = ("east_mm", "north_mm", "yaw_mdeg", "dock", "seed", "seeds", "dock_offset_east_mm",
             "dock_offset_north_mm", "dock_yaw_offset_mdeg", "expected_dock", "applied")


def _series(tmp_path, name="s1", seeds=None, **changes) -> Path:
    series = service.new_series(name, service.datetime(2026, 11, 4, tzinfo=service.timezone.utc))
    if seeds is not None:
        series["seeds"] = seeds
    series.update(changes)
    path = tmp_path / f"{name}.json"
    path.write_text(json.dumps(series), encoding="utf-8")
    return path


def _keys(data) -> set:
    if isinstance(data, dict):
        return set(data) | {key for value in data.values() for key in _keys(value)}
    if isinstance(data, list):
        return {key for value in data for key in _keys(value)}
    return set()


# --- the scene: reproducible, distinct, private --------------------------------------------------------------------

def test_each_seed_reproduces_its_scene_hash_and_ten_seeds_give_ten_scenes(tmp_path):
    path = _series(tmp_path)
    first = [service.prepare(path, index, None)["scene_hash"] for index in range(service.SERIES_SEEDS)]
    again = [service.prepare(path, index, None)["scene_hash"] for index in range(service.SERIES_SEEDS)]
    assert first == again and len(set(first)) == service.SERIES_SEEDS


def test_the_drawn_dock_stays_in_its_region_in_whole_units():
    for seed in range(200):
        dock = service.draw_dock(seed, 30.0)
        assert all(isinstance(value, int) for value in dock.values())
        assert (dock["east_mm"] ** 2 + dock["north_mm"] ** 2) ** 0.5 <= 30_001 and 0 <= dock["yaw_mdeg"] < 360_000


@pytest.mark.parametrize("change, problem", [
    ({"seeds": [1, 2, 3]}, "has 3 seeds"),
    ({"seeds": [1] * 10}, "repeats a seed"),
    ({"dock_region": {"max_offset_m": 500.0}}, "above 200"),
    ({"consumers": ["scoring"]}, "unreal"),
    ({"consumers": ["unreal", "scoring", "navpy"]}, "must be one of unreal, contact, scoring"),
])
def test_a_bad_series_is_refused(tmp_path, change, problem):
    with pytest.raises(RecordError, match=problem):
        service.prepare(_series(tmp_path, **change), 0, None)


def test_a_series_is_written_once_and_only_in_the_private_folder(tmp_path):
    series = service.new_series("s1", service.datetime(2026, 11, 4, tzinfo=service.timezone.utc))
    with pytest.raises(RecordError, match="not in the private folder"):
        service.write_series(series, tmp_path / "public" / "s1.json", tmp_path / "private")
    service.write_series(series, tmp_path / "private" / "s1.json", tmp_path / "private")
    with pytest.raises(RecordError, match="frozen once written"):
        service.write_series(series, tmp_path / "private" / "s1.json", tmp_path / "private")


def test_the_public_half_holds_no_pose_and_no_seed_and_hides_the_hash_input(tmp_path):
    path = _series(tmp_path)
    public = service.prepare(path, 3, tmp_path / "scenario")
    assert not _keys(public) & set(POSE_KEYS)
    assert sp.read_public(tmp_path / "scenario") == public
    text = json.dumps(public)
    series = json.loads(path.read_text(encoding="utf-8"))
    assert str(series["seeds"][3]) not in text
    # The scene's hash covers the seed: the public hash cannot be searched back to a pose.
    scene = service.build_scene(series, 3)
    assert public["scene_hash"] == service.scene_hash(scene)
    assert public["scene_hash"] != service.scene_hash({key: value for key, value in scene.items() if key != "seed"})


# --- deliveries: the three consumers only, every one in the ledger ---------------------------------------------------

def test_prepare_delivers_to_the_series_consumers_and_the_ledger_shows_them(tmp_path):
    folder = tmp_path / "scenario"
    service.prepare(_series(tmp_path), 0, folder)
    audit = scoring.audit(folder)
    assert audit == {"deliveries": {"unreal": 1, "scoring": 1}, "outside_allowed": [], "refused": [],
                     "stray_files": []}
    unreal = json.loads(sp.consumer_file(folder, "unreal").read_text(encoding="utf-8"))
    assert set(unreal) == {"schema_version", "scene_hash", "dock_offset_east_mm", "dock_offset_north_mm",
                           "dock_yaw_offset_mdeg"}


def test_a_consumer_outside_the_three_is_refused_and_the_refusal_is_logged(tmp_path):
    folder = tmp_path / "scenario"
    path = _series(tmp_path)
    service.prepare(path, 0, folder)
    scene = service.build_scene(service.read_series(path), 0)
    with pytest.raises(service.ScenarioError, match="navpy may not receive"):
        service.deliver(folder, "navpy", scene)
    with pytest.raises(RecordError, match="navpy is not a scenario consumer"):
        sp.consumer_file(folder, "navpy")
    assert scoring.audit(folder)["refused"] == ["navpy"] and not (folder / sp.PRIVATE_DIR / "navpy.json").exists()


def test_a_file_the_service_did_not_write_shows_in_the_audit(tmp_path):
    folder = tmp_path / "scenario"
    service.prepare(_series(tmp_path), 0, folder)
    (folder / sp.PRIVATE_DIR / "copy.json").write_text("{}", encoding="utf-8")
    assert scoring.audit(folder)["stray_files"] == [str(Path(sp.PRIVATE_DIR) / "copy.json")]


def test_one_scenario_per_folder(tmp_path):
    path = _series(tmp_path)
    service.prepare(path, 0, tmp_path / "scenario")
    with pytest.raises(service.ScenarioError, match="one scenario per run"):
        service.prepare(path, 1, tmp_path / "scenario")


@pytest.mark.parametrize("consumers, problem", [
    (["unreal"], "scoring"), (["scoring"], "unreal"), (["unreal", "scoring", "gcs"], "must be one of"),
])
def test_a_public_half_missing_a_consumer_or_naming_another_is_refused(tmp_path, consumers, problem):
    public = service.prepare(_series(tmp_path), 0, None)
    with pytest.raises(RecordError, match=problem):
        sp.validate_public({**public, "consumers": consumers})


# --- scoring: Unreal built the delivered scene, and its log carries no pose -----------------------------------------

def _unreal(folder: Path, log: Path, east_off_mm=0, scene_hash=None, terrain="+0.03", extra="",
            applied=True) -> None:
    """What Unreal leaves: its private readback beside its file (numbers as doubles) and its log lines."""
    delivered = json.loads(sp.consumer_file(folder, "scoring").read_text(encoding="utf-8"))
    dock = {**delivered["scene"]["dock"]}
    dock["east_mm"] += east_off_mm
    digest = scene_hash or delivered["scene_hash"]
    if applied:
        (folder / sp.PRIVATE_DIR / scoring.UNREAL_APPLIED_FILE).write_text(json.dumps(
            {"schema_version": 1.0, "scene_hash": digest, **{key: float(value) for key, value in dock.items()},
             "board": {"location_cm": [1.0, 2.0, 3.0]}}), encoding="utf-8")
    log.write_text(f"LogTemp: Display: NavPyScenario applied scene_hash={digest}\n"
                   f"LogTemp: Display: NavPyScenario terrain check: dock base {terrain} m above terrain, "
                   f"origin +0.00 m above terrain\n{extra}", encoding="utf-8")


def _verify(tmp_path, **unreal):
    folder, log = tmp_path / "scenario", tmp_path / "unreal.log"
    service.prepare(_series(tmp_path), 0, folder)
    _unreal(folder, log, **unreal)
    return scoring.verify_unreal(folder, log), folder


def test_scoring_reproduces_the_scene_from_unreals_readback(tmp_path):
    verdict, folder = _verify(tmp_path)
    assert verdict["reproduced"] and verdict["problems"] == []
    assert verdict["rebuilt_scene_hash"] == verdict["scene_hash"]
    assert not _keys(verdict) & set(POSE_KEYS)          # the printed verdict names no pose
    assert scoring.audit(folder)["stray_files"] == []   # the readback and the verdict are expected files


@pytest.mark.parametrize("unreal, problem", [
    ({"east_off_mm": 1}, "does not reproduce"),
    ({"scene_hash": "a" * 64}, "applied another scenario file"),
    ({"terrain": "+0.80"}, "off the terrain"),
    ({"extra": "LogTemp: NavPyBoardPose loc=1,2,3 quat=0,0,0,1\n"}, "carries pose lines"),
    ({"extra": "LogTemp: Error: NavPyScenario refused: no single dock deck\n"}, "refused the scenario file"),
    ({"applied": False}, "wrote no unreal_applied.json"),
])
def test_scoring_refuses_a_scene_unreal_did_not_build_or_a_log_that_shows_it(tmp_path, unreal, problem):
    verdict, _ = _verify(tmp_path, **unreal)
    assert not verdict["reproduced"] and any(problem in item for item in verdict["problems"]), verdict["problems"]


def test_without_an_unreal_log_or_a_scoring_file_nothing_is_reproduced(tmp_path):
    folder = tmp_path / "scenario"
    service.prepare(_series(tmp_path), 0, folder)
    verdict = scoring.verify_unreal(folder, tmp_path / "none.log")
    assert not verdict["reproduced"] and "no Unreal log" in verdict["problems"]
    without = tmp_path / "unscored"
    service.prepare(_series(tmp_path, name="s2", consumers=["unreal", "scoring"]), 0, without)
    sp.consumer_file(without, "scoring").unlink()
    with pytest.raises(RecordError, match="no file"):
        scoring.verify_unreal(without, tmp_path / "none.log")


# --- the runner: truth is never read on the command side, and commands do not change with it ------------------------

class _Group:
    """The runner's process group, recording what it would start."""

    def __init__(self, run_dir, spawners, port_free) -> None:
        self.run_dir, self.started = Path(run_dir), []

    def refusals(self, specs):
        return []

    def start(self, spec) -> None:
        self.started.append(spec)

    def stop_all(self) -> None:
        """Nothing runs."""

    def mark_stopped(self, role, code) -> None:
        """Nothing runs."""

    def producers(self):
        return {}

    def orphans(self):
        return []


class _Board:
    def __init__(self, *args, **kwargs) -> None:
        self.workdir, self.record = Path(args[1]), {"hitl": {"bridge_exit": 0}}

    def start(self) -> None:
        """No board."""

    def stop(self) -> None:
        """No board."""

    def report(self):
        return {"link_health": {"ok": True}}


def _run(tmp_path, monkeypatch, series: Path, root: str) -> dict:
    """One runner run with the real service process and fake chain roles: what it would command."""
    seen: dict = {}
    groups: list = []

    def group(*args):
        groups.append(_Group(*args))
        return groups[-1]

    def fly(args, board, group, run):
        lane = build_lane(HOME, LaneSpec.of(run["scenario"]["lane"]))
        seen["lane"] = [tuple(item) for item in lane_items(lane)]
        seen["limits"] = dict(run["scenario"]["limits"])

    def record(args, run_id, repo, scenario):
        seen["record_scenario"] = scenario
        return {"run": {"run_id": run_id}}

    monkeypatch.setattr(approach, "ProcessGroup", group)
    monkeypatch.setattr(approach, "Board", _Board)
    monkeypatch.setattr(approach, "_fly", fly)
    monkeypatch.setattr(approach, "_record", record)
    monkeypatch.setattr(approach.approach_records, "write_record",
                        lambda record, run_dir: _write(run_dir / "approach_record.json"))
    monkeypatch.setattr(approach, "_addresses", lambda: ("172.0.0.2", "172.0.0.1"))
    monkeypatch.setattr(approach, "_board_scripts", lambda repo, run_dir: None)
    monkeypatch.setattr(approach, "board_param_files", lambda *args: [])
    monkeypatch.setattr(approach.chain, "publisher_profile", lambda paths: tmp_path / "profile.json")
    monkeypatch.setattr(approach.chain, "win_path", lambda path: str(path))
    monkeypatch.chdir(REPO)
    parser = approach.argparse.ArgumentParser()
    approach.add_parser(parser.add_subparsers())
    args = parser.parse_args(["approach", "--mode", "pc-sitl", "--run-root", str(tmp_path / root),
                              "--model-dir", str(tmp_path), "--model-input", "mi.json", "--media-runtime", "mr.json",
                              "--publisher-profile", "p.json", "--aas-hitl", "/aas",
                              "--uproject", "/mnt/c/u/A.uproject", "--windows-python", "/mnt/c/py/python.exe",
                              "--scenario-series", str(series), "--seed-index", "0"])
    seen["exit"] = approach.approach(args)
    run_dir = next((tmp_path / root).iterdir())
    seen["run_dir"], seen["run"] = run_dir, json.loads((run_dir / "approach_run.json").read_text(encoding="utf-8"))
    seen["argv"] = [[arg.replace(str(run_dir), "<run>").replace(run_dir.name, "<id>") for arg in spec.argv]
                    for spec in groups[0].started]
    return seen


def _write(path: Path) -> Path:
    path.write_text("{}", encoding="utf-8")
    return path


def _poison(monkeypatch, truth: list[Path]) -> list[str]:
    """Every open in this process of a private scenario file or the series file is recorded and refused."""
    touched: list[str] = []

    def guarded(opener):
        def open_(file, *args, **kwargs):
            if isinstance(file, (str, bytes, os.PathLike)):
                path = Path(os.fsdecode(file)).resolve()
                if any(path == item or item in path.parents for item in truth) or \
                        (sp.PRIVATE_DIR in path.parts and "scenario" in path.parts):
                    touched.append(str(path))
                    raise PermissionError(f"command side opened scenario truth {path}")
            return opener(file, *args, **kwargs)
        return open_

    monkeypatch.setattr(builtins, "open", guarded(builtins.open))
    monkeypatch.setattr(io, "open", guarded(io.open))
    monkeypatch.setattr(os, "open", guarded(os.open))
    return touched


def test_the_runner_never_opens_scenario_truth_and_unreal_gets_only_its_files_path(tmp_path, monkeypatch):
    series = _series(tmp_path)
    touched = _poison(monkeypatch, [series.resolve()])
    seen = _run(tmp_path, monkeypatch, series, "runs")
    assert touched == []
    run, run_dir = seen["run"], seen["run_dir"]
    assert seen["record_scenario"] == run["scenario"] == sp.read_public(run_dir / approach.scenario_run.SCENARIO_DIR)
    assert not _keys(run["scenario"]) & set(POSE_KEYS)
    unreal = next(argv for argv in seen["argv"] if any(arg.startswith("-NavPyScenario=") for arg in argv))
    assert f"-NavPyScenario=<run>/{approach.scenario_run.SCENARIO_DIR}/{sp.PRIVATE_DIR}/unreal.json" in \
        [arg.replace("\\", "/") for arg in unreal]
    navpy = [argv for argv in seen["argv"] if argv is not unreal]
    assert not any(sp.PRIVATE_DIR in arg or str(series) in arg for argv in navpy for arg in argv)
    # No Unreal ran: the service's own process scores the run as not reproduced, and the run fails.
    assert run["scene"]["reproduced"] is False and "no Unreal log" in run["scene"]["problems"]
    assert seen["exit"] == 1
    manifest = json.loads(Path(run["manifest"]).read_text(encoding="utf-8"))
    assert not any(sp.PRIVATE_DIR in item["name"] for item in manifest["other_files"])


def test_another_scene_changes_no_command_lane_or_limit(tmp_path, monkeypatch):
    first = _run(tmp_path, monkeypatch, _series(tmp_path, name="s1"), "runs-1")
    second = _run(tmp_path, monkeypatch, _series(tmp_path, name="s2"), "runs-2")
    assert first["run"]["scenario"]["scene_hash"] != second["run"]["scenario"]["scene_hash"]
    assert first["argv"] == second["argv"]
    assert first["lane"] == second["lane"] and first["limits"] == second["limits"]


# --- guards: who may import the service, and navpy never sees tools --------------------------------------------------

# Scoring and its live check beside Unreal; the runner reaches both only through their processes.
SERVICE_IMPORTERS = {"tools/hitl/scenario_scene_check.py", "tools/hitl/scenario_scoring.py"}
SCORING_IMPORTERS = {"tools/hitl/scenario_scene_check.py"}


def _imports(path: Path) -> set[str]:
    names = set()
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if isinstance(node, ast.Import):
            names |= {alias.name for alias in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module:
            names |= {node.module, *(f"{node.module}.{alias.name}" for alias in node.names)}
    return names


def test_only_scoring_imports_the_service_and_navpy_imports_no_tools():
    def importers(module: str) -> set[str]:
        return {str(path.relative_to(REPO)).replace("\\", "/") for path in (REPO / "tools").rglob("*.py")
                if module in _imports(path)}

    assert importers("tools.hitl.scenario_service") == SERVICE_IMPORTERS
    assert importers("tools.hitl.scenario_scoring") == SCORING_IMPORTERS
    navpy = [str(path) for path in (REPO / "src").rglob("*.py")
             if any(name == "tools" or name.startswith("tools.") for name in _imports(path))]
    assert navpy == []
