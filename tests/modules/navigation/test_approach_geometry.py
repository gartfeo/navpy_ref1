"""HITL step 23: the explicit HITL orbit radius (core/skills/acquire/orbit_radius_setting.py, approach_planner.py)
and the pre-series orbit check (tools/hitl/scenario_checks.py).

Product: a profile without the setting keeps the 500 m floor; the HITL profile's setting replaces only the floor;
a malformed setting is refused. Check: a feasible plan passes, and a plan failing any one check is refused (100%).
Isolation: the check reads only the scenario's public half, and nothing of its truth inputs reaches navpy."""
from __future__ import annotations

import ast
import builtins
import dataclasses
import io
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from navpy.core.kernel.location import Location
from navpy.core.perception.profile.vision_profile_loader import load_profiles
from navpy.core.skills.acquire.approach_planner import ApproachPlanner
from navpy.core.skills.acquire.approach_strategy import ApproachKind, ApproachPlan
from navpy.core.skills.acquire.orbit_geometry import OrbitNavigationLimits, r_nav_min
from navpy.core.skills.acquire.orbit_radius_setting import (
    FINAL_APPROACH_ORBIT_KEY,
    final_approach_orbit_floor,
    final_approach_orbit_setting,
)
from navpy.core.skills.acquire.peer_orbit_plan import MIN_APPROACH_STANDOFF_M
from navpy.plugins.detection.charuco.charuco_board import CharucoBoardSpec
from tools.hitl import scenario_checks as sc
from tools.hitl import scenario_public as sp

REPO = Path(__file__).resolve().parents[3]
PROFILES = load_profiles()[0]
HITL = PROFILES[sc.HITL_PROFILE]
STEPS = sc._zoom_steps(HITL)
BOARD = sc._board(HITL)


# --- product: the setting and the planner --------------------------------------------------------------------------

class _Log:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def info(self, message, **_):
        pass

    def warning(self, message, **_):
        self.warnings.append(message)


def _planner(profile, *, lim_roll=45.0, min_pitch=-45.0, airspeed=24.0, final_approach=True):
    vehicle = SimpleNamespace(home_location=None, lim_roll=lim_roll, min_pitch=min_pitch, air_speed=airspeed)
    log = _Log()

    def camera_plan(poi, drone, mounts, **_):
        return ApproachPlan(kind=ApproachKind.ORBIT, approach_location=poi, offset_distance=0.0, orbit_radius=900.0)

    planner = ApproachPlanner(vehicle, lambda: [], lambda: final_approach, profile, log, ApproachKind.ORBIT, 80.0,
                              plan_offset=camera_plan)
    return planner, log


def _radius(planner, height=80.0):
    poi = Location(40.0, 44.0, 0.0, is_absolute=False)
    plan, _ = planner.plan(poi, 0, Location(40.01, 44.0, height, is_absolute=False),
                           approach_kind=ApproachKind.ORBIT, approach_alt_rel=height)
    return plan.orbit_radius


def test_non_hitl_profiles_keep_the_500_m_floor_unchanged():
    assert MIN_APPROACH_STANDOFF_M == 500.0
    for name, profile in PROFILES.items():
        if name == sc.HITL_PROFILE:
            continue
        assert final_approach_orbit_setting(profile) is None, name
        assert final_approach_orbit_floor(profile) == 500.0
        planner, _ = _planner(profile)
        limits = OrbitNavigationLimits(24.0, 45.0, -45.0)
        assert _radius(planner) == r_nav_min(limits, 80.0, floor_m=500.0) == 500.0


def test_only_hitl_profiles_carry_the_setting():
    carriers = [name for name, profile in PROFILES.items() if FINAL_APPROACH_ORBIT_KEY in profile]
    assert carriers == [sc.HITL_PROFILE]
    assert all("hitl" in name for name in carriers)


def test_the_hitl_profile_flies_its_explicit_radius():
    assert final_approach_orbit_setting(HITL) == 300.0      # owner, 2026-11-05: at least 300 m for the approach
    planner, log = _planner(HITL)
    assert _radius(planner) == 300.0
    assert log.warnings == []


def test_the_setting_replaces_only_the_floor_and_a_wider_envelope_minimum_is_flown_and_logged():
    planner, log = _planner(HITL, airspeed=40.0)     # turn term 2 v^2 / (g tan 45) = 326 m
    assert _radius(planner) == pytest.approx(2 * 40.0 ** 2 / 9.81)
    assert len(log.warnings) == 1 and "explicit radius 300m" in log.warnings[0]


def test_without_a_final_approach_law_the_camera_plan_stays():
    planner, _ = _planner(HITL, final_approach=False)
    assert _radius(planner) == 900.0


@pytest.mark.parametrize("block", [
    "170", 170.0, {"radius_m": "170"}, {"radius_m": -1.0}, {"radius_m": 0.0}, {"radius_m": math.nan},
    {"radius_m": math.inf}, {"radius_m": True}, {}, {"radius_m": 170.0, "board_height_m": 20.0},
])
def test_a_malformed_setting_is_refused_when_the_planner_is_built(block):
    with pytest.raises(ValueError):
        _planner({FINAL_APPROACH_ORBIT_KEY: block})


# --- the pre-series check ------------------------------------------------------------------------------------------

def _plan(**changes) -> sc.OrbitPlan:
    """A feasible plan: 170 m circle, 90 m above a board on a 16 m mast, zoom 10, floor 7 m, 24 m/s."""
    base = sc.OrbitPlan(radius_m=170.0, lane_height_m=106.0, board_height_m=16.0, floor_m=7.0, airspeed_m_s=24.0,
                        airspeed_band=(14.0, 35.0), min_pitch_deg=-45.0, steps=STEPS, zoom_command_tenths=100,
                        board=BOARD)
    return dataclasses.replace(base, **changes)


def _failed(plan) -> set[str]:
    return {result.name for result in sc.run_checks(plan) if not result.passed}


def test_a_feasible_plan_passes_every_check():
    results = sc.run_checks(_plan())
    assert sc.feasible(results), [(r.name, r.detail) for r in results if not r.passed]
    assert [r.name for r in results] == ["model_speed", "turn_radius", "descent", "reader", "containment",
                                         "recovery_floor"]


def test_the_check_uses_the_measured_zoom_10_optics_and_the_configured_board():
    step = STEPS.by_command(100)
    assert step.optics.fy == pytest.approx(16812.132)
    assert (STEPS.stream.image_width, STEPS.stream.image_height) == (1920, 1080)
    assert BOARD == sc.Board(CharucoBoardSpec(square_mm=400.0, marker_mm=296.0))
    assert (BOARD.width_m, BOARD.length_m, BOARD.square_m) == (2.0, 2.8, 0.4)


def test_the_turn_term_is_the_planners_at_the_nav_bank_cap():
    detail = next(r for r in sc.run_checks(_plan()) if r.name == "turn_radius").detail
    assert detail["turn_radius_m"] == pytest.approx(24.0 ** 2 / (9.81 * math.tan(math.radians(35.0))), abs=0.01)
    assert detail["required_m"] == pytest.approx(167.7, abs=0.05)


@pytest.mark.parametrize("changes, failing", [
    ({"airspeed_m_s": 36.0}, {"model_speed", "turn_radius"}),
    ({"airspeed_band": None}, {"model_speed"}),
    ({"radius_m": 160.0}, {"turn_radius", "descent"}),       # descent 90 / tan(29.25) = 161 m
    ({"airspeed_m_s": 25.0}, {"turn_radius"}),              # 2 v^2 / (g tan 35) = 182 m
    ({"min_pitch_deg": -35.0}, {"descent"}),                 # 90 / tan(22.75) = 214 m
    ({"min_pitch_deg": 0.0}, {"descent"}),
    ({"lane_height_m": 96.0}, {"reader"}),                   # the owner's 80 m above the mast
    ({"reader_min_square_px": 12.0}, {"reader"}),
    ({"zoom_command_tenths": 90}, {"reader", "containment"}),  # 10 still fits: the policy zooms past 9
    ({"zoom_command_tenths": 55}, {"reader", "containment"}),  # not a measured step
    ({"board": sc.Board(CharucoBoardSpec(square_mm=4000.0, marker_mm=2960.0))}, {"containment"}),
    ({"floor_m": 30.0}, {"recovery_floor"}),                 # the series default floor, above the board
    ({"board_height_m": 15.0, "lane_height_m": 105.0}, {"recovery_floor"}),  # 8 m over the floor: under the pull-out
])
def test_a_plan_failing_a_check_is_refused(changes, failing):
    plan = _plan(**changes)
    assert _failed(plan) == failing
    assert not sc.feasible(sc.run_checks(plan))


@pytest.mark.parametrize("changes", [{"radius_m": math.nan}, {"lane_height_m": 16.0}, {"radius_m": 0.0},
                                     {"airspeed_m_s": -1.0}, {"board_height_m": math.inf}])
def test_an_unusable_plan_fails_every_check(changes):
    results = sc.run_checks(_plan(**changes))
    assert len(results) == len(sc.CHECKS) and not any(r.passed for r in results)


def test_the_first_detection_beyond_the_circle_sets_the_ground_ray_error():
    """Codex round 1: 170 m circle 90 m above a 20 m board: the board is first found 211 m out, so the ground ray
    puts the centre 46.9 m off, not 37.8 m, and the far side drops under the reader limit."""
    plan = _plan(board_height_m=20.0, lane_height_m=110.0)
    assert sc.centre_error_m(plan) == pytest.approx(46.89, abs=0.01)
    assert _failed(plan) == {"reader"}


def test_the_centre_error_follows_the_product_solvers_verdict_at_the_reading_range():
    plan, step = _plan(), STEPS.by_command(100)
    reading = sc.reading_slant_m(plan, step)
    assert reading == pytest.approx(math.sqrt(16812.132 * 0.4 * 90.0 / 11.5))      # 229 m
    assert sc.pose_accepted_at(plan, step, 100.0)                                   # step 22 accepts it close in
    assert not sc.pose_accepted_at(plan, step, reading)                             # its 0.3 m gate refuses it there
    reach = math.sqrt(reading ** 2 - 90.0 ** 2)                                     # first found 211 m out
    assert sc.detection_distance_m(plan, step) == pytest.approx(reach)
    assert sc.centre_error_m(plan) == pytest.approx(reach * 16.0 / 90.0)           # so the ground ray's 37.5 m


def test_an_accepted_fit_at_the_reading_range_uses_the_pose_limit(monkeypatch):
    monkeypatch.setattr(sc, "pose_accepted_at", lambda *_: True)
    assert sc.centre_error_m(_plan()) == sc.POSE_CENTRE_LIMIT_M == 0.5
    assert _failed(_plan(lane_height_m=96.0)) == set()           # the owner's 80 m plan would then pass


def test_the_pose_prediction_is_deterministic():
    plan, step = _plan(), STEPS.by_command(100)
    slants = [205.0 + 5.0 * i for i in range(12)]
    first = [sc.pose_accepted_at(plan, step, slant) for slant in slants]
    assert first == [sc.pose_accepted_at(plan, step, slant) for slant in slants]


def test_no_results_is_not_feasible():
    assert not sc.feasible([])


# --- the CLI: refuses on any failed check or unreadable input ------------------------------------------------------

def _inputs(tmp_path, lane_height=106.0, floor=7.0, min_pitch=-45.0, band=(14.0, 35.0)):
    scenario = tmp_path / "scenario"
    (scenario / sp.PRIVATE_DIR).mkdir(parents=True)
    public = {"schema_version": sp.SCHEMA_VERSION, "series_id": "s1", "series_sha256": "a" * 64, "seed_index": 0,
              "scene_hash": "b" * 64, "consumers": ["unreal", "scoring"],
              "lane": {"bearing_deg": 0.0, "start_m": 400.0, "length_m": 2000.0, "height_m": lane_height,
                       "repeats": 30},
              "limits": {"observe_s": 660.0, "min_height_m": floor}}
    (scenario / sp.SCENARIO_FILE).write_text(json.dumps(public), encoding="utf-8")
    (scenario / sp.PRIVATE_DIR / "unreal.json").write_text('{"east_mm": 1}', encoding="utf-8")
    params = tmp_path / "aircraft.parm"
    params.write_text(f"AIRSPEED_CRUISE 24\nPTCH_LIM_MIN_DEG {min_pitch}\n", encoding="utf-8")
    model = tmp_path / "model_input.json"
    envelope = {} if band is None else {"airspeed_m_s": {"min": band[0], "max": band[1]}}
    model.write_text(json.dumps({"envelope": envelope}), encoding="utf-8")
    return ["--scenario", str(scenario), "--params-file", str(params), "--model-input", str(model),
            "--board-height-m", "16"]


def _cli(argv, capsys):
    code = sc.main(argv)
    return code, json.loads(capsys.readouterr().out)


def test_the_cli_refuses_the_300_m_profile_radius_on_the_reader(tmp_path, capsys):
    """The owner's 300 m: ChArUco cannot read the board that far at 10x (a whole-object dock detector is a later
    step), so every series is refused, on the reader alone."""
    code, out = _cli(_inputs(tmp_path), capsys)
    assert code == 1 and out["feasible"] is False
    assert [c["name"] for c in out["checks"] if not c["passed"]] == ["reader"]


@pytest.fixture
def radius_170(monkeypatch):
    """The CLI on a 170 m radius, which the board can be read at, so a refusal is the input's own."""
    real = sc.load_plan
    monkeypatch.setattr(sc, "load_plan", lambda args: dataclasses.replace(real(args), radius_m=170.0))


def test_the_cli_passes_a_feasible_series(tmp_path, capsys, radius_170):
    code, out = _cli(_inputs(tmp_path), capsys)
    assert code == 0 and out["feasible"] is True and len(out["checks"]) == len(sc.CHECKS)


@pytest.mark.parametrize("inputs, extra", [
    ({"lane_height": 96.0}, []),
    ({"lane_height": 140.0}, []),
    ({"floor": 30.0}, []),
    ({"min_pitch": -30.0}, []),
    ({"band": None}, []),
    ({}, ["--zoom", "5.5"]),
    ({}, ["--reader-min-square-px", "20"]),
])
def test_the_cli_refuses_a_series_failing_a_check(tmp_path, capsys, inputs, extra, radius_170):
    code, out = _cli(_inputs(tmp_path, **inputs) + extra, capsys)
    assert code == 1 and out["feasible"] is False and not all(c["passed"] for c in out["checks"])


def test_the_cli_refuses_unreadable_inputs(tmp_path, capsys):
    argv = _inputs(tmp_path)
    (tmp_path / "aircraft.parm").write_text("AIRSPEED_CRUISE 24\n", encoding="utf-8")
    code, out = _cli(argv, capsys)
    assert code == 1 and "PTCH_LIM_MIN_DEG" in out["refused"]
    code, out = _cli(_inputs(tmp_path / "b") + ["--profile", "siyi_zr10"], capsys)
    assert code == 1 and "final_approach_orbit" in out["refused"]
    argv = _inputs(tmp_path / "c")
    code, out = _cli(argv[:1] + [str(tmp_path / "none")] + argv[2:], capsys)
    assert code == 1 and "refused" in out


# --- isolation: no truth geometry reaches product acquisition ------------------------------------------------------

def test_the_check_opens_no_private_scenario_file(tmp_path, capsys, monkeypatch, radius_170):
    argv = _inputs(tmp_path)
    private = (tmp_path / "scenario" / sp.PRIVATE_DIR).resolve()
    touched: list[str] = []

    def guarded(opener):
        def open_(file, *args, **kwargs):
            if isinstance(file, (str, bytes, os.PathLike)):
                path = Path(os.fsdecode(file)).resolve()
                if private == path or private in path.parents:
                    touched.append(str(path))
                    raise PermissionError(path)
            return opener(file, *args, **kwargs)
        return open_

    monkeypatch.setattr(builtins, "open", guarded(builtins.open))
    monkeypatch.setattr(io, "open", guarded(io.open))
    monkeypatch.setattr(os, "open", guarded(os.open))
    code, _ = _cli(argv, capsys)
    assert code == 0 and touched == []


def _imports(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module)
    return names


def test_navpy_names_neither_the_check_nor_its_truth_inputs():
    """The product sees only radius_m: no navpy source imports the check or names the scene's board height."""
    hits = []
    for path in (REPO / "src" / "navpy").rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        hits += [f"{path}: {name}" for name in ("scenario_checks", "board_height_m", "lane_height_m")
                 if name in text]
        hits += [f"{path}: imports tools" for name in _imports(path) if name == "tools" or name.startswith("tools.")]
    assert hits == []


def test_the_product_radius_does_not_change_with_the_scene():
    """Two scenes the check judges differently give navpy the same radius: the planner has no scene input."""
    planner, _ = _planner(HITL)
    assert sc.feasible(sc.run_checks(_plan())) != sc.feasible(sc.run_checks(_plan(lane_height_m=96.0)))
    assert _radius(planner, height=80.0) == _radius(planner, height=90.0) == 300.0
