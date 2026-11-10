"""HITL steps 23 and 23b: the explicit HITL orbit radius (core/skills/acquire/orbit_radius_setting.py,
approach_planner.py) and the pre-series orbit check (tools/hitl/scenario_checks.py) with the whole-dock detector.

Product: a profile without the setting keeps the 500 m floor; the HITL profile's setting replaces only the floor;
a malformed setting is refused. Check: the owner's 300 m series passes with the whole-dock detector, and a plan
failing any one check is refused (100%); the first detection beyond the circle sets the ground ray's error; the dock
on the ground needs no clearance over the floor (released at the operator's confirm).
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
from tools.hitl import orbit_centre as oc
from tools.hitl import orbit_plan as op
from tools.hitl import scenario_checks as sc
from tools.hitl import scenario_public as sp
from tools.hitl.scenario_service import DEFAULT_DOCK

REPO = Path(__file__).resolve().parents[3]
PROFILES = load_profiles()[0]
HITL = PROFILES[op.HITL_PROFILE]
STEPS = op._zoom_steps(HITL)
LIMITS = op.detector_limits(HITL)
DOCK = op.dock_of({"dock_size": DEFAULT_DOCK})
H = 80.0 - 0.979                    # the 80 m lane above the pickup's POI, its mid-height


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
        if name == op.HITL_PROFILE:
            continue
        assert final_approach_orbit_setting(profile) is None, name
        assert final_approach_orbit_floor(profile) == 500.0
        planner, _ = _planner(profile)
        limits = OrbitNavigationLimits(24.0, 45.0, -45.0)
        assert _radius(planner) == r_nav_min(limits, 80.0, floor_m=500.0) == 500.0


def test_only_hitl_profiles_carry_the_setting():
    carriers = [name for name, profile in PROFILES.items() if FINAL_APPROACH_ORBIT_KEY in profile]
    assert carriers == [op.HITL_PROFILE]
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

def _plan(**changes) -> op.OrbitPlan:
    """The series plan: the profile's 300 m circle, the step 13 lane at 80 m, the series' 30 m floor, the default
    pickup on the ground, zoom 10, 24 m/s, the profile's whole-dock detector limits."""
    base = op.OrbitPlan(radius_m=300.0, lane_height_m=80.0, dock=DOCK, floor_m=30.0, airspeed_m_s=24.0,
                        airspeed_band=(14.0, 35.0), min_pitch_deg=-45.0, roll_limit_deg=45.0, steps=STEPS,
                        zoom_command_tenths=100, detector=LIMITS)
    return dataclasses.replace(base, **changes)


def _failed(plan) -> set[str]:
    return {result.name for result in sc.run_checks(plan) if not result.passed}


def _detail(plan, name: str) -> dict:
    return next(r for r in sc.run_checks(plan) if r.name == name).detail


def test_the_owners_300_m_series_passes_every_check_with_the_whole_dock_detector():
    results = sc.run_checks(_plan())
    assert sc.feasible(results), [(r.name, r.detail) for r in results if not r.passed]
    assert [r.name for r in results] == ["model_speed", "turn_radius", "descent", "detector", "containment",
                                         "recovery_floor"]


def test_the_check_uses_the_measured_optics_the_profile_detector_and_the_public_dock():
    assert STEPS.by_command(100).optics.fy == pytest.approx(16812.132)
    assert op.search_step(_plan()).command == "2.0"                     # the search's widest step
    assert (STEPS.stream.image_width, STEPS.stream.image_height) == (1920, 1080)
    assert LIMITS.input_scale(1920, 1080) == (1.0, 1.0)                 # tiles: native pixels
    assert (LIMITS.recall_min_box_px, LIMITS.acq_min_box_px) == (24.0, 24.0)
    assert DOCK == op.Dock(5.887, 2.490, 1.958) and DOCK.poi_height_m == pytest.approx(0.979)


def test_the_dock_box_follows_its_yaw_and_the_depression():
    step, h = STEPS.by_command(100), H
    slant = math.hypot(300.0, h)
    side_on = oc.dock_box_px(step, DOCK, slant, h, 90.0)                # long axis across the line of sight
    end_on = oc.dock_box_px(step, DOCK, slant, h, 0.0)
    assert side_on[0] == pytest.approx(16970.9 * 5.887 / slant, rel=1e-3)
    assert end_on[0] == pytest.approx(16970.9 * 2.490 / slant, rel=1e-3)
    sin_d = h / slant
    assert end_on[1] == pytest.approx(16812.132 * (5.887 * sin_d + 1.958 * math.sqrt(1 - sin_d ** 2)) / slant)


def test_the_first_detection_beyond_the_circle_sets_the_ground_ray_error():
    """The gate's box floor at the search step bounds D: here 444 m, beyond the 300 m circle, and every declared
    term is taken there (hitl_dock_detector.md section 4.1)."""
    plan = _plan()
    d = oc.first_detection_distance_m(plan)
    assert d > plan.radius_m and d == pytest.approx(443.9, abs=0.5)
    side = max(oc.smaller_side_input_px(plan, op.search_step(plan), math.hypot(d, H), yaw)
               for yaw in oc.DOCK_YAWS_DEG)
    assert side == pytest.approx(LIMITS.acq_min_box_px - plan.bounds.box_size_error_px, abs=0.05)
    terms = oc.centre_error_terms(plan)
    assert terms["poi_height"] == pytest.approx(d * 0.979 / H)
    assert terms["lane_height"] == pytest.approx(d * 1.0 / H)
    assert terms["azimuth"] == pytest.approx(math.hypot(d, H) * math.radians(1.0))
    assert terms["capture_skew"] == pytest.approx(2.4)
    assert oc.centre_error_m(plan) == pytest.approx(sum(terms.values())) == pytest.approx(33.15, abs=0.01)


def test_a_higher_gate_floor_brings_the_first_detection_in_and_never_inside_the_circle():
    near = dataclasses.replace(LIMITS, acq_min_box_px=40.0, recall_min_box_px=24.0)
    assert oc.first_detection_distance_m(_plan(detector=near)) < oc.first_detection_distance_m(_plan())
    unreachable = dataclasses.replace(LIMITS, acq_min_box_px=1000.0)
    assert oc.first_detection_distance_m(_plan(detector=unreachable)) == 300.0
    assert oc.acquirable_beyond_radius(_plan()) and not oc.acquirable_beyond_radius(_plan(detector=unreachable))


def test_a_gate_floor_the_search_cannot_reach_beyond_the_circle_is_refused():
    """Codex round 1: a floor of 1000 px (above the 704 px input) or one met only inside the circle never acquires
    there; the check refuses it instead of clamping D_max to r and passing."""
    for floor in (1000.0, 60.0):
        plan = _plan(detector=dataclasses.replace(LIMITS, acq_min_box_px=floor))
        assert _detail(plan, "detector")["acquirable_beyond_radius"] is False
        assert _failed(plan) == {"detector"}


def test_the_dock_on_the_ground_needs_no_clearance_over_the_floor():
    """Option A (owner, 2026-11-08): the floor guards the lane, search and orbit and is released at the operator's
    confirm, so a dock 30 m below the floor passes; the lane must still be above the floor."""
    detail = _detail(_plan(), "recovery_floor")
    assert detail["lane_above_floor_m"] == 50.0 and detail["floor_released_at"] == "operator confirm"
    assert _failed(_plan(floor_m=80.0)) == {"recovery_floor"}
    assert _failed(_plan(floor_m=79.9)) == set()


def test_the_turn_term_is_the_planners_at_the_nav_bank_cap():
    detail = _detail(_plan(), "turn_radius")
    assert detail["turn_radius_m"] == pytest.approx(24.0 ** 2 / (9.81 * math.tan(math.radians(35.0))), abs=0.01)
    assert detail["required_m"] == pytest.approx(167.7, abs=0.05)


@pytest.mark.parametrize("changes, failing", [
    ({"airspeed_m_s": 36.0}, {"model_speed", "turn_radius"}),     # 2 v^2 / (g tan 35) = 377 m
    ({"airspeed_band": None}, {"model_speed"}),
    ({"radius_m": 167.0}, {"turn_radius"}),                        # 167.7 m; nearer, the pickup breaks containment
    ({"min_pitch_deg": -20.0}, {"descent"}),                       # 80 / tan(13) = 346 m
    ({"min_pitch_deg": 0.0}, {"descent"}),
    ({"detector": dataclasses.replace(LIMITS, recall_min_box_px=100.0, acq_min_box_px=100.0)}, {"detector"}),
    ({"detector": dataclasses.replace(LIMITS, acq_min_box_px=20.0)}, {"detector"}),   # gate below the measured size
    ({"zoom_command_tenths": 90}, {"containment"}),               # 10 still fits: the policy zooms past 9
    ({"zoom_command_tenths": 55}, {"detector", "containment"}),   # not a measured step
    ({"dock": op.Dock(60.0, 20.0, 1.5)}, {"containment"}),         # overflows the frame at 10x
    ({"floor_m": 85.0}, {"recovery_floor"}),
])
def test_a_plan_failing_a_check_is_refused(changes, failing):
    plan = _plan(**changes)
    assert _failed(plan) == failing
    assert not sc.feasible(sc.run_checks(plan))


def test_the_detector_check_takes_the_far_side_of_the_circle_not_the_radius():
    """The dock is r + e away on the far side: a limit the box meets at r (136 px) but not at r + e (127 px) fails."""
    plan = _plan(detector=dataclasses.replace(LIMITS, recall_min_box_px=130.0, acq_min_box_px=130.0))
    detail = _detail(plan, "detector")
    at_radius = min(oc.smaller_side_input_px(plan, STEPS.by_command(100), math.hypot(300.0, H), yaw)
                    for yaw in oc.DOCK_YAWS_DEG)
    assert at_radius > 130.0 > detail["smaller_side_input_px"]
    assert detail["farthest_slant_m"] == pytest.approx(math.hypot(300.0 + detail["centre_error_m"], H), abs=0.01)
    assert _failed(plan) == {"detector"}


def test_a_squashed_frame_is_judged_in_its_own_input_pixels():
    """The squash scales 1920 x 1080 to 704 x 704: the orbit box is 122 native px but 47 squash px across."""
    squash = dataclasses.replace(LIMITS, input_scale=lambda w, h: (704 / w, 704 / h))
    side = _detail(_plan(detector=squash), "detector")["smaller_side_input_px"]
    assert side == pytest.approx(_detail(_plan(), "detector")["smaller_side_input_px"] * 704 / 1920, rel=0.05)
    assert _failed(_plan(detector=dataclasses.replace(squash, recall_min_box_px=50.0, acq_min_box_px=50.0))) == {
        "detector"}


@pytest.mark.parametrize("changes", [
    {"radius_m": math.nan}, {"lane_height_m": 0.5}, {"radius_m": 0.0}, {"airspeed_m_s": -1.0},
    {"dock": op.Dock(0.0, 2.490, 1.958)}, {"dock": op.Dock(5.887, 2.490, math.inf)},
    {"detector": dataclasses.replace(LIMITS, recall_min_box_px=math.nan)},
    {"detector": dataclasses.replace(LIMITS, acq_min_box_px=0.0)},
])
def test_an_unusable_plan_fails_every_check(changes):
    results = sc.run_checks(_plan(**changes))
    assert len(results) == len(sc.CHECKS) and not any(r.passed for r in results)


@pytest.mark.parametrize("plan_changes", [
    {},                                                     # 300 m, flown as set
    {"radius_m": 130.0, "airspeed_m_s": 14.0},              # descent 80 / tan(29.25) = 142.85 m
])
def test_the_descent_check_agrees_with_the_planner_on_the_ground_ray_centre(plan_changes):
    """The orbit is always centred on the ground ray (height zero), so the planner sizes its descent from the whole
    lane height. The check passes descent exactly when the planner flies the setting."""
    plan = _plan(**plan_changes)
    assert oc.descent_height_m(plan) == plan.lane_height_m
    planner, log = _planner({FINAL_APPROACH_ORBIT_KEY: {"radius_m": plan.radius_m}}, lim_roll=plan.roll_limit_deg,
                            min_pitch=plan.min_pitch_deg, airspeed=plan.airspeed_m_s)
    flown = _radius(planner, height=plan.lane_height_m)
    descent = _detail(plan, "descent")
    passed = "descent" not in _failed(plan)
    assert passed == (flown == plan.radius_m) == (log.warnings == [])
    if not passed:
        assert flown == pytest.approx(descent["required_m"], abs=0.01) == pytest.approx(142.85, abs=0.01)


def test_an_aircraft_bank_limit_under_the_nav_cap_sets_the_turn_term_as_in_the_planner():
    """ROLL_LIMIT_DEG 20 makes the planner fly 322.64 m, so the 300 m plan is refused."""
    plan = _plan(roll_limit_deg=20.0)
    turn = next(r for r in sc.run_checks(plan) if r.name == "turn_radius")
    assert not turn.passed and turn.detail["bank_deg"] == 20.0
    assert turn.detail["required_m"] == pytest.approx(322.64, abs=0.01)
    planner, log = _planner({FINAL_APPROACH_ORBIT_KEY: {"radius_m": 300.0}}, lim_roll=20.0)
    assert _radius(planner, height=plan.lane_height_m) == pytest.approx(322.64, abs=0.01) and len(log.warnings) == 1
    assert _failed(_plan(roll_limit_deg=0.0)) == {"turn_radius"}


def test_no_results_is_not_feasible():
    assert not sc.feasible([])


def test_the_profile_without_the_whole_dock_detector_is_refused():
    with pytest.raises(ValueError):
        op.detector_limits({"detector": {"rfdetr": {"variant": "large", "classes": {"truck": "dock"},
                                                    "input": "tiles"}}})
    with pytest.raises(ValueError):
        op.dock_of({"dock_size": {**DEFAULT_DOCK, "placement": "mast"}})


# --- the CLI: refuses on any failed check or unreadable input ------------------------------------------------------

def _inputs(tmp_path, lane_height=80.0, floor=30.0, min_pitch=-45.0, band=(14.0, 35.0), dock=None):
    scenario = tmp_path / "scenario"
    (scenario / sp.PRIVATE_DIR).mkdir(parents=True)
    public = {"schema_version": sp.SCHEMA_VERSION, "series_id": "s1", "series_sha256": "a" * 64, "seed_index": 0,
              "scene_hash": "b" * 64, "consumers": ["unreal", "contact", "scoring"],
              "lane": {"bearing_deg": 0.0, "start_m": 400.0, "length_m": 2000.0, "height_m": lane_height,
                       "repeats": 30},
              "limits": {"observe_s": 660.0, "min_height_m": floor}, "dock_size": dock or dict(DEFAULT_DOCK)}
    (scenario / sp.SCENARIO_FILE).write_text(json.dumps(public), encoding="utf-8")
    (scenario / sp.PRIVATE_DIR / "unreal.json").write_text('{"east_mm": 1}', encoding="utf-8")
    params = tmp_path / "aircraft.parm"
    params.write_text(f"AIRSPEED_CRUISE 24\nPTCH_LIM_MIN_DEG {min_pitch}\nROLL_LIMIT_DEG 45\n", encoding="utf-8")
    model = tmp_path / "model_input.json"
    envelope = {} if band is None else {"airspeed_m_s": {"min": band[0], "max": band[1]}}
    model.write_text(json.dumps({"envelope": envelope}), encoding="utf-8")
    return ["--scenario", str(scenario), "--params-file", str(params), "--model-input", str(model)]


def _cli(argv, capsys):
    code = sc.main(argv)
    return code, json.loads(capsys.readouterr().out)


def test_the_cli_passes_the_300_m_profile_with_the_whole_dock_detector(tmp_path, capsys):
    """Step 23's reader refused every 300 m series; the detector check passes it."""
    code, out = _cli(_inputs(tmp_path), capsys)
    assert code == 0 and out["feasible"] is True and len(out["checks"]) == len(sc.CHECKS)
    detector = next(c for c in out["checks"] if c["name"] == "detector")
    assert detector["evidence"].startswith("hitl_final_approach.md step 23b")


@pytest.mark.parametrize("inputs, extra", [
    ({"lane_height": 30.0}, []),
    ({"floor": 90.0}, []),
    ({"min_pitch": -20.0}, []),
    ({"band": None}, []),
    ({}, ["--zoom", "5.5"]),
    ({"dock": {**DEFAULT_DOCK, "length_m": 60.0, "width_m": 20.0}}, []),
])
def test_the_cli_refuses_a_series_failing_a_check(tmp_path, capsys, inputs, extra):
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
    code, out = _cli(_inputs(tmp_path / "d", dock={**DEFAULT_DOCK, "placement": "mast"}), capsys)
    assert code == 1 and "refused" in out                          # the public half refuses a dock off the ground


# --- isolation: no truth geometry reaches product acquisition ------------------------------------------------------

def test_the_check_opens_no_private_scenario_file(tmp_path, capsys, monkeypatch):
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
    """The product sees only radius_m: no navpy source imports the check or names the scene's dock or lane."""
    hits = []
    for path in (REPO / "src" / "navpy").rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        hits += [f"{path}: {name}" for name in ("scenario_checks", "board_height_m", "lane_height_m",
                                                 "poi_height_m", "first_detection_distance")
                 if name in text]
        hits += [f"{path}: imports tools" for name in _imports(path) if name == "tools" or name.startswith("tools.")]
    assert hits == []


def test_the_product_radius_does_not_change_with_the_scene():
    """Two scenes the check judges differently give navpy the same radius: the planner has no scene input."""
    planner, _ = _planner(HITL)
    assert sc.feasible(sc.run_checks(_plan())) != sc.feasible(sc.run_checks(_plan(floor_m=85.0)))
    assert _radius(planner, height=80.0) == _radius(planner, height=90.0) == 300.0
