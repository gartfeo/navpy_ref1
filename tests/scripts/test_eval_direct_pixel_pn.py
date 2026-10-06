from __future__ import annotations

import csv
from pathlib import Path
import pytest
from types import SimpleNamespace

from scripts import eval_direct_pixel_pn as evaluator


def _write_compact(
    path: Path,
    pitch: list[float],
    *,
    roll: float = 1.0,
    bearing: float = 0.2,
) -> None:
    log_dir = path / "navpy-logs"
    log_dir.mkdir()
    with (log_dir / "uav_121_navigation_compact.csv").open(
        "w", encoding="utf-8", newline=""
    ) as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=("dist", "cmd_r", "cmd_p", "act_r", "yaw_err"),
        )
        writer.writeheader()
        for index, command in enumerate(pitch):
            writer.writerow({
                "dist": 600 - index * 10,
                "cmd_r": roll,
                "cmd_p": command,
                "act_r": roll,
                "yaw_err": bearing,
            })


def test_command_stability_reports_gradual_midcourse_change(tmp_path: Path) -> None:
    _write_compact(tmp_path, [-8.0 - index * 0.2 for index in range(50)])

    stability = evaluator._command_stability(tmp_path)

    assert stability["sample_count"] == 19
    assert stability["pitch_span_deg"] == pytest.approx(3.6)
    assert stability["pitch_max_step_deg"] == pytest.approx(0.2)


def test_command_stability_exposes_one_midcourse_jump(tmp_path: Path) -> None:
    pitch = [-8.0 - index * 0.2 for index in range(50)]
    pitch[35] -= 4.0
    _write_compact(tmp_path, pitch)

    stability = evaluator._command_stability(tmp_path)

    assert stability["pitch_max_step_deg"] == pytest.approx(4.2)


def test_command_stability_exposes_persistent_curved_pursuit(tmp_path: Path) -> None:
    _write_compact(
        tmp_path,
        [-8.0 for _ in range(50)],
        roll=8.0,
        bearing=15.0,
    )

    stability = evaluator._command_stability(tmp_path)

    assert stability["alignment_cmd_roll_mean_abs_deg"] == 8.0
    assert stability["alignment_actual_roll_mean_abs_deg"] == 8.0
    assert stability["alignment_bearing_mean_abs_deg"] == 15.0


def _write_trace(path: Path, *, rows: int = 100, period_s: float = 0.01) -> None:
    """Actuator trace whose span is the guided-leg duration."""
    with (path / "flight_control_trace.csv").open(
        "w", encoding="utf-8", newline=""
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=("wall_s", "sent_age_ms"))
        writer.writeheader()
        for index in range(rows):
            writer.writerow({"wall_s": 1000.0 + index * period_s, "sent_age_ms": 1.0})


def test_observation_freshness_counts_frames_overwritten_before_navigation(
    tmp_path: Path,
) -> None:
    # 99 rows of gaps at 10 ms is a 0.99 s wall leg; at 10x that is 9.9 s of
    # vehicle time, which is the timebase the navigation law works in.
    _write_trace(tmp_path)

    freshness = evaluator._observation_freshness(
        tmp_path,
        10.0,
        {"projected_frames": 400, "delivered_frames": 300, "delivery_rejections": 10},
    )

    assert freshness["engagement_s"] == pytest.approx(9.9)
    assert freshness["overwritten_frames"] == 90
    assert freshness["projected_rate_hz"] == pytest.approx(400 / 9.9)
    assert freshness["delivered_rate_hz"] == pytest.approx(300 / 9.9)
    assert freshness["fresh_fraction"] == pytest.approx(0.75)


def test_observation_freshness_rejects_missing_frame_counts(tmp_path: Path) -> None:
    _write_trace(tmp_path)

    with pytest.raises(RuntimeError, match="source frame counts missing"):
        evaluator._observation_freshness(tmp_path, 1.0, {"delivered_frames": 300})


def test_three_uav_verdict_fails_a_run_starved_of_fresh_observations(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A starved run must not be scored: commands stay nominal while stale."""
    from scripts import eval_direct_pixel_pn_three_uav as three

    monkeypatch.setattr(
        evaluator,
        "_command_stability",
        lambda _path: (_ for _ in ()).throw(RuntimeError("not needed")),
    )
    _write_trace(tmp_path)
    scorer = SimpleNamespace(
        result=SimpleNamespace(
            dist_3d_m=0.5,
            horizontal_m=0.3,
            vertical_m=0.4,
            __dict__={"dist_3d_m": 0.5},
        ),
        certification_error=None,
        sample_count=1,
    )

    result = three._verdict(
        {
            "passed": True,
            "snap_3d_m": 0.5,
            "source": {
                "projection_failures": 0,
                "delivered_frames": 570,
                "projected_frames": 1000,
                "delivery_rejections": 0,
            },
        },
        scorer,
        tmp_path,
        1.0,
        10.0,
    )

    assert not result["passed"]
    assert result["observation_freshness"]["fresh_fraction"] == pytest.approx(0.57)
    assert any("fresh observations 57.0%" in error for error in result["errors"])


def _swarm_command(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, **kwargs):
    """Run _start_swarm without launching anything, return the built command."""
    import json
    import subprocess

    import eval_navigation_cases

    class FakePopen:
        def __init__(self, command, **_kwargs) -> None:
            self.command = command
            self.returncode = None

        def poll(self):
            return None

    monkeypatch.setattr(subprocess, "Popen", FakePopen)
    monkeypatch.setattr(
        eval_navigation_cases,
        "wait_for_eval_chat",
        lambda *_args, **_kwargs: SimpleNamespace(chat=41),
    )
    evaluator._start_swarm(Path("python"), tmp_path, 10.0, instances=1, **kwargs)
    return json.loads((tmp_path / "swarm.cmd.json").read_text(encoding="utf-8"))


def test_home_reaches_the_swarm_launch(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Home is fixed at SITL launch; a mission upload cannot move it."""
    command = _swarm_command(tmp_path, monkeypatch, home="43.0,34.0,0,0")

    assert "--home" in command
    assert command[command.index("--home") + 1] == "43.0,34.0,0,0"


def test_swarm_launch_omits_home_when_not_pinned(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    command = _swarm_command(tmp_path, monkeypatch, home=None)

    assert "--home" not in command


def test_wind_is_an_input_not_pinned_to_calm() -> None:
    """The evaluator used to force SIM_WIND_SPD/DIR to zero on every run."""
    args = evaluator._parser().parse_args(["--wind-speed", "10", "--wind-dir", "180"])

    assert args.wind_speed == 10.0
    assert args.wind_dir == 180.0


def test_wind_defaults_to_calm() -> None:
    args = evaluator._parser().parse_args([])

    assert args.wind_speed == 0.0
    assert args.wind_dir == 0.0
    assert args.home == evaluator.DEFAULT_HOME_COORDS


def test_three_uav_shares_one_wind_and_home_definition() -> None:
    """Defining these twice would be a duplicate-argument error and a second
    default to keep in step."""
    from scripts import eval_direct_pixel_pn_three_uav as three

    args = three._parser().parse_args(["--wind-speed", "7", "--wind-dir", "90"])

    assert (args.wind_speed, args.wind_dir) == (7.0, 90.0)
    assert args.home == evaluator.DEFAULT_HOME_COORDS


def test_freshness_gate_rejects_a_high_fraction_at_a_starved_rate() -> None:
    """10 of 10 frames is 100% and still worthless."""
    errors = evaluator.freshness_errors({
        "fresh_fraction": 1.0,
        "delivered_rate_hz": 4.0,
        "projected_rate_hz": 4.0,
    })

    assert any("fresh observation rate 4.0Hz" in error for error in errors)
    assert not any("below 80.0%" in error for error in errors)


def test_freshness_gate_accepts_a_healthy_run() -> None:
    assert evaluator.freshness_errors({
        "fresh_fraction": 0.897,
        "delivered_rate_hz": 35.9,
        "projected_rate_hz": 40.0,
    }) == []


def test_freshness_gate_reports_both_failures_together() -> None:
    errors = evaluator.freshness_errors({
        "fresh_fraction": 0.57,
        "delivered_rate_hz": 21.0,
        "projected_rate_hz": 36.8,
    })

    assert len(errors) == 2


def test_three_uav_verdict_accepts_a_healthy_fresh_observation_rate(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from scripts import eval_direct_pixel_pn_three_uav as three

    monkeypatch.setattr(
        evaluator,
        "_command_stability",
        lambda _path: (_ for _ in ()).throw(RuntimeError("not needed")),
    )
    _write_trace(tmp_path)

    result = three._verdict(
        {
            "passed": True,
            "snap_3d_m": 0.5,
            "source": {
                "projection_failures": 0,
                "delivered_frames": 897,
                "projected_frames": 1000,
                "delivery_rejections": 0,
            },
        },
        SimpleNamespace(result=None, certification_error=None, sample_count=0),
        tmp_path,
        1.0,
        1.0,
    )

    assert result["observation_freshness"]["fresh_fraction"] == pytest.approx(0.897)
    assert not any("fresh observations" in error for error in result["errors"])


def test_three_uav_verdict_reports_bad_snap_when_stability_is_unavailable(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from scripts import eval_direct_pixel_pn_three_uav as three

    monkeypatch.setattr(
        evaluator,
        "_command_stability",
        lambda _path: (_ for _ in ()).throw(
            RuntimeError("insufficient midcourse command evidence: 0 rows")
        ),
    )
    result = three._verdict(
        {
            "passed": True,
            "snap_3d_m": 500.0,
            "source": {
                "projection_failures": 0,
                "delivered_frames": 100,
            },
        },
        SimpleNamespace(result=None, certification_error=None, sample_count=0),
        tmp_path,
        1.0,
        1.0,
    )

    assert not result["passed"]
    assert "SNAP 500.0 exceeds 1m" in result["errors"]
    assert any("command stability unavailable" in error for error in result["errors"])


def test_three_uav_verdict_reports_horizontal_and_vertical_minima(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from scripts import eval_direct_pixel_pn_three_uav as three

    monkeypatch.setattr(
        evaluator,
        "_command_stability",
        lambda _path: (_ for _ in ()).throw(RuntimeError("not needed")),
    )
    scorer = SimpleNamespace(
        result=SimpleNamespace(
            dist_3d_m=5.0,
            horizontal_m=3.0,
            vertical_m=4.0,
            __dict__={
                "dist_3d_m": 5.0,
                "horizontal_m": 3.0,
                "vertical_m": 4.0,
            },
        ),
        certification_error=None,
        sample_count=1,
    )

    result = three._verdict(
        {
            "passed": True,
            "snap_3d_m": 5.0,
            "snap_horizontal_m": 2.5,
            "snap_vertical_m": 4.2,
            "source": {"projection_failures": 0, "delivered_frames": 100},
        },
        scorer,
        tmp_path,
        10.0,
        1.0,
    )

    assert result["snap_horizontal_min_m"] == 2.5
    assert result["snap_vertical_min_m"] == 4.2
    assert result["coordinate_horizontal_min_m"] == 3.0
    assert result["coordinate_vertical_min_m"] == 4.0


def test_geometry_defaults_match_the_uploader_so_runs_are_unchanged() -> None:
    """The offsets are new options, not a new geometry: defaults must be identical."""
    from scripts import upload_north_line_mission as uploader

    args = evaluator._parser().parse_args([])

    assert args.gate_offset == uploader.DEFAULT_GATE_OFFSET_M
    assert args.poi_offset == uploader.DEFAULT_WAYPOINT_OFFSET_M
    assert args.mission_alt == uploader.DEFAULT_ALT_M


def test_matched_glide_geometry_is_expressible() -> None:
    """Case B of the leg-independence test: 300 m leg at the same 21 deg glide."""
    import math

    from scripts import upload_north_line_mission as uploader

    args = evaluator._parser().parse_args([
        "--gate-offset", "1800", "--poi-offset", "2100",
        "--mission-alt", "400", "--poi-alt", "285",
    ])
    leg_m = args.poi_offset - args.gate_offset
    glide_deg = math.degrees(math.atan((args.mission_alt - args.poi_alt) / leg_m))

    assert leg_m == pytest.approx(300.0)
    assert glide_deg == pytest.approx(20.96, abs=0.5)
    # Case A, the shipped default, must sit at the same angle.
    default_leg = uploader.DEFAULT_WAYPOINT_OFFSET_M - uploader.DEFAULT_GATE_OFFSET_M
    default_glide = math.degrees(math.atan((400.0 - 60.0) / default_leg))
    assert default_glide == pytest.approx(glide_deg, abs=0.5)


def test_short_leg_with_a_low_poi_is_a_much_steeper_glide() -> None:
    """Why POI altitude must move with leg length, not stay at 60 m."""
    import math

    steep = math.degrees(math.atan((400.0 - 60.0) / 300.0))

    assert steep == pytest.approx(48.6, abs=0.5)


def test_goal_is_reported_separately_from_the_pass_gate() -> None:
    """A run inside the 1 m gate is not a run that met the 0.1 m goal."""
    args = evaluator._parser().parse_args([])

    assert args.max_distance == 1.0
    assert args.goal_distance == 0.1
    assert args.goal_distance < args.max_distance


def test_goal_count_does_not_follow_the_pass_flag() -> None:
    """The measured headwind case: 3 inside the gate, only 2 inside the goal.

    Counting goes through summarize() on each run's meets_goal decision --
    never re-derived from the child's snap, which is the EKF's error
    projection.
    """
    results = [
        {"passed": True, "passed_accuracy": True, "meets_goal": True,
         "scoring_source": "sim_state_truth"},
        {"passed": True, "passed_accuracy": True, "meets_goal": True,
         "scoring_source": "sim_state_truth"},
        {"passed": True, "passed_accuracy": True, "meets_goal": False,
         "scoring_source": "sim_state_truth"},
    ]

    summary = evaluator.summarize(
        results,
        gate_m=1.0,
        goal_m=0.1,
        source_identity={"sha256": "abc", "files": 1},
        aborted_identity=None,
    )

    assert summary["runs_within_gate"] == 3
    assert summary["runs_within_goal"] == 2
    assert summary["runs_goal_unscored"] == 0


def test_the_default_sim_rate_cannot_carry_the_clock_beat() -> None:
    """SITL steps physics in integer microseconds; 1200 Hz leaves an 8 us
    deficit per 20 ms tick that shows up as a boot-locked 2.082 s beat, an
    EKF3 pitch tone and a 4x worse truth-CPA median (0.516 m at 1200 Hz,
    n=15, vs 0.120 m at 1000 Hz, n=30; 2026-08-24). The default must keep
    1e6/rate an integer."""
    from scripts.eval_sim_parameters import SIM_RATE_HZ_DEFAULT, sim_parameters

    pushed = dict(sim_parameters(evaluator._parser().parse_args([])))

    assert pushed["SIM_RATE_HZ"] == SIM_RATE_HZ_DEFAULT == 1000.0
    assert (1e6 / SIM_RATE_HZ_DEFAULT) == int(1e6 / SIM_RATE_HZ_DEFAULT)


def test_the_template_geofence_is_disabled_for_every_eval_run() -> None:
    """The launcher syncs eeproms from a MUTABLE template; a surviving fence
    auto-enables on takeoff and RTLs the climb before scoring interval (both cases
    of the first truth-scoring sweep, 2026-09-03). Fence-off is isolation
    policy, like ARMING_CHECK=0."""
    from scripts.eval_sim_parameters import sim_parameters

    pushed = dict(sim_parameters(evaluator._parser().parse_args([])))

    assert pushed["FENCE_ENABLE"] == 0.0
    assert pushed["FENCE_AUTOENABLE"] == 0.0


def test_an_extra_sitl_param_is_pushed_after_the_built_ins() -> None:
    """`--sitl-param NAME=VALUE` exists so an intervention run (clock
    stepping, GPS cadence, estimator choice) does not need a harness edit,
    which would abort the batch on the source-identity gate."""
    from scripts.eval_sim_parameters import sim_parameters

    args = evaluator._parser().parse_args(
        ["--sitl-param", "SIM_RATE_HZ=1200", "--sitl-param", "SIM_GPS_HZ=10"]
    )

    pushed = sim_parameters(args)

    assert pushed[-2:] == (("SIM_RATE_HZ", 1200.0), ("SIM_GPS_HZ", 10.0))


def test_a_malformed_sitl_param_fails_at_parse_time() -> None:
    """Silently skipping a typo'd override would fly a run that looks like
    the intervention but is not -- and failing only after SITL boot wastes
    minutes per case, so argparse itself must reject it."""
    import pytest as _pytest

    from scripts.eval_sim_parameters import parse_sitl_param

    # inf/nan/overflow: set_param derives a RELATIVE tolerance, so a
    # requested inf makes any echo pass -- and json would emit Infinity.
    # Non-ASCII names would only fail at name.encode("ascii") after SITL
    # is already up.
    # "ſIM_X" (long s) uppercases INTO ascii "SIM_X" -- the ASCII check
    # must run before upper() or it bypasses validation entirely.
    for raw in ("SIM_RATE_HZ", " =1", "SIM_RATE_HZ=fast", "SIM_X=nan",
                "SIM_X=inf", "SIM_X=-inf", "SIM_X=1e9999", "SÏM_X=1",
                "ſIM_X=1"):
        with _pytest.raises(SystemExit):
            evaluator._parser().parse_args(["--sitl-param", raw])
        with _pytest.raises(ValueError):
            parse_sitl_param(raw)


def test_a_harness_owned_parameter_cannot_be_overridden() -> None:
    """`--sitl-param SIM_SPEEDUP=10` would fly at 10x while freshness and the
    scored results still assume the `--speedups` clock; an overridden wind
    would desynchronise the ground-track summary. Fail closed, name the flag
    that actually controls it."""
    import pytest as _pytest

    from scripts.eval_sim_parameters import HARNESS_OWNED, parse_sitl_param

    # Spelled out, not iterated from HARNESS_OWNED alone: deleting a map
    # entry must fail HERE, not silently delete its test case. The fence
    # pair is isolation policy like ARMING_CHECK -- an escape-hatch
    # FENCE_ENABLE=1 would RTL the scoring interval leg and score nothing.
    required = {
        "ARMING_CHECK", "FENCE_ENABLE", "FENCE_AUTOENABLE",
        "SIM_SPEEDUP", "SIM_WIND_SPD", "SIM_WIND_DIR",
    }
    assert required <= set(HARNESS_OWNED)
    for name in required:
        with _pytest.raises(SystemExit):
            evaluator._parser().parse_args(["--sitl-param", f"{name}=1"])
        with _pytest.raises(ValueError, match=name):
            parse_sitl_param(f"{name.lower()}=1")


def test_the_three_uav_harness_pushes_the_same_parameters() -> None:
    """It reuses this module's parser, so a flag defined here already parses
    there. When it kept its own copy of the parameter tuple, an override
    would have been accepted by a three-UAV run and then quietly ignored."""
    from pathlib import Path as _Path

    from scripts import eval_direct_pixel_pn_three_uav as three
    from scripts import eval_three_uav_mission as mission
    from scripts.eval_sim_parameters import sim_parameters

    args = three._parser().parse_args(["--sitl-param", "SIM_GPS_HZ=10"])
    pushed = dict(sim_parameters(args))

    assert pushed["SIM_GPS_HZ"] == 10.0
    # The behavioural check passes even if the three-UAV path still pushes
    # its own stale copy, so also hold the duplication itself closed.
    source = _Path(mission.__file__).read_text(encoding="utf-8")
    assert "SIM_WIND_SPD" not in source
    # AST, not a substring: `sim_parameters(args)` also appears in verdict
    # RECORDING, so a substring scan stays green with the actual handoff to
    # prepare_vehicles deleted. Assert the call-site keyword itself.
    import ast as _ast

    tree = _ast.parse(_Path(three.__file__).read_text(encoding="utf-8"))
    calls = [
        node for node in _ast.walk(tree)
        if isinstance(node, _ast.Call)
        and getattr(node.func, "id", "") == "prepare_vehicles"
    ]
    assert len(calls) == 1
    keywords = {keyword.arg: keyword.value for keyword in calls[0].keywords}
    parameters = keywords["parameters"]
    assert isinstance(parameters, _ast.Call)
    assert getattr(parameters.func, "id", "") == "sim_parameters"


def test_the_siyi_harness_pushes_the_shared_parameters_too() -> None:
    """It inherits `--sitl-param` from the shared parser, so it must honour
    it. Its private wind/arming tuple was exactly the silent-ignore failure
    the shared module exists to prevent. AST again: the recording line also
    says `sim_parameters(args)`, so a substring scan cannot pin the PUSH."""
    import ast as _ast
    from pathlib import Path as _Path

    from scripts import eval_siyi_geo_tracking_three_uav as siyi

    source = _Path(siyi.__file__).read_text(encoding="utf-8")
    assert "SIM_WIND_SPD" not in source
    tree = _ast.parse(source)
    push_loops = [
        node for node in _ast.walk(tree)
        if isinstance(node, _ast.For)
        and isinstance(node.iter, _ast.Call)
        and getattr(node.iter.func, "id", "") == "sim_parameters"
        and any(
            isinstance(inner, _ast.Call)
            and getattr(inner.func, "id", "") == "set_param"
            for statement in node.body
            for inner in _ast.walk(statement)
        )
    ]
    assert len(push_loops) == 1
