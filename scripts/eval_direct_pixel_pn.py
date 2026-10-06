"""Repeatable live isolation of ideal pixels feeding pure-vision PN."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

from pymavlink import mavutil

WORKTREE = Path(__file__).resolve().parent.parent
SCRIPTS = Path(__file__).resolve().parent
sys.path[:0] = [str(WORKTREE / "src"), str(WORKTREE), str(SCRIPTS)]

from gcs.backend import instance_ports as ip  # noqa: E402

from eval_source_identity import (  # noqa: E402
    script_import_closure, source_identity,
)

# Captured per case AND re-checked before each run. The child's scripts-local
# imports are followed rather than listed, because the list is what went stale:
# see `eval_source_identity`.
SOURCE_IDENTITY_ROOTS = tuple(dict.fromkeys((
    WORKTREE / "src" / "navpy",
    *script_import_closure(SCRIPTS / "direct_pixel_pn_child.py", SCRIPTS),
    # Parent-side scoring and verdict code decides what a case's numbers MEAN,
    # so an edit to it mid-sweep must abort the sweep exactly like a law edit:
    # before this closure was added, a scorer change could not be seen by the
    # identity guard at all.
    *script_import_closure(SCRIPTS / "eval_direct_pixel_pn.py", SCRIPTS),
    # The launcher runs by subprocess, so its CLI/configuration imports are
    # not reached through the evaluator's Python import graph.
    *script_import_closure(SCRIPTS / "swarm_run.py", SCRIPTS),
    # Outside the child's import closure -- but it decides which SITL
    # parameters (clock rate included) the case flies under.
    SCRIPTS / "eval_sim_parameters.py",
)))


def _source_identity() -> dict[str, object]:
    return source_identity(SOURCE_IDENTITY_ROOTS, WORKTREE)

from eval_navigation_cases import (  # noqa: E402
    CoordinateScorer, command_long, download_mission, request_coordinate_score_stream,
    require_nav_solution, resolve_home_abs_alt_m, resolve_poi_expectation, set_param,
    stop_own_stack, wait_for_heartbeat,
)
from eval_direct_pixel_stability import (  # noqa: E402
    MAX_ALIGNMENT_MEAN_ABS_BEARING_DEG, MAX_ALIGNMENT_MEAN_ABS_ROLL_DEG,
    MAX_MIDCOURSE_PITCH_STEP_DEG, MIDCOURSE_MIN_DISTANCE_M,
    _command_stability,
)
from eval_direct_pixel_summary import summarize  # noqa: E402
from eval_direct_pixel_verdict import (  # noqa: E402
    SCORING_POLICIES, SCORING_POLICY_SITL_TRUTH,
    SCORING_POLICY_VEHICLE_ESTIMATE, case_verdict, persist_verdict,
    unscored_result,
)
from eval_navigation_models import PositionStreamAnchor  # noqa: E402
from eval_navigation_telemetry import (  # noqa: E402
    drain_position_messages, request_message_interval_stream,
)
from eval_navigation_truth import (  # noqa: E402
    TRUTH_SCORE_RATE_HZ, TruthRecorder, salvage_truth,
)
from scripts.eval_direct_pixel_command_causality import causality_evidence
from eval_observation_freshness import (  # noqa: E402
    MIN_FRESH_OBSERVATION_FRACTION,
    MIN_FRESH_OBSERVATION_RATE_HZ,
    freshness_errors,
    scoring_interval_span_s as _scoring_interval_span_s,
    observation_freshness as _observation_freshness,
)
from eval_ground_track import GroundTrackRecorder  # noqa: E402
from eval_sim_cpa_config import SIM_CPA_MODES, resolve_mode  # noqa: E402
from eval_sim_cpa_stage import SimCpaStage  # noqa: E402
from eval_sim_parameters import add_sitl_param_argument  # noqa: E402
from eval_sim_parameters import sim_parameters  # noqa: E402
from pixel_pn_case_manifest import write_case_manifest  # noqa: E402
from pixel_pn_child_process import (  # noqa: E402
    launch_child as _launch_child,
    terminate as _terminate,
    wait_ready as _wait_ready,
)
from pixel_pn_final_approach_speed import launch_speedup, final_approach_speed_plan
from upload_north_line_mission import (  # noqa: E402
    DEFAULT_ALT_M, DEFAULT_GATE_OFFSET_M, DEFAULT_LOITER_OFFSET_M,
    DEFAULT_WAYPOINT_OFFSET_M, upload_north_line,
)


MIN_SAFE_POI_REL_ALT_M = 60.0
# How long the flight loop keeps draining after the child's result appears,
# waiting for the truth recorder's post-CPA closure evidence.
TRUTH_CLOSURE_DRAIN_TIMEOUT_S = 5.0
# Fraction of the frames the sim sensor produced that must actually reach the
# navigation law as fresh observations.
#
# The command rate alone cannot show this.  The source keeps a newest-only slot
# and the worker drains at most one frame per command slot; when the slot is
# empty the worker reissues the previous command as a held primitive.  So a run
# that starves the law of fresh observations still emits commands at the full
# nominal rate, and every command-side cadence measure looks healthy while the
# commands themselves carry stale information.  The final-approach LOS-rate filter
# differentiates successive observations, so its usable bandwidth follows the
# fresh-observation rate, not the command rate.
#
from eval_direct_pixel_cli import DEFAULT_HOME_COORDS, build_parser as _parser


def _home_lat_lon(home: str) -> tuple[float, float]:
    """First two fields of a "lat,lon,alt,heading" HOME_COORDS string."""
    parts = [piece.strip() for piece in home.split(",")]
    if len(parts) < 2:
        raise ValueError(f"home must be LAT,LON[,ALT,HEADING]; got {home!r}")
    return float(parts[0]), float(parts[1])


def _speeds(raw: str) -> list[float]:
    result = [float(value.strip()) for value in raw.split(",") if value.strip()]
    if not result or any(value <= 0.0 for value in result):
        raise ValueError("speedups must be positive")
    return result


def _start_swarm(
    python: Path,
    case_dir: Path,
    speedup: float,
    *,
    instances: int,
    home: str | None = None,
    chat: int | None = None,
    dist: int | None = None,
    sitl_defaults: str | None = None,
) -> tuple[subprocess.Popen[bytes], object]:
    import secrets

    from eval_navigation_cases import wait_for_eval_chat

    launch_token = secrets.token_hex(16)
    command = [
        str(python),
        str(SCRIPTS / "swarm_run.py"),
        "--eval",
        "--instances",
        str(instances),
        "--speedup",
        str(speedup),
        "--launch-token",
        launch_token,
    ]
    # `--eval` alone picks the first free slot, which RACES: concurrent
    # launches all choose the same chat and every loser is refused with
    # "chat N already has a live SITL supervisor". Callers running flights in
    # parallel must hand out distinct slots themselves.
    if chat is not None:
        command.extend(("--chat", str(chat)))
    if home:
        command.extend(("--home", home))
    if sitl_defaults:
        # Pre-boot parameter file, as a WSL path. Values read at buffer
        # allocation or sensor calibration time cannot be set any later:
        # `eval_sim_parameters.py` pushes after SITL has booted.
        command.extend(("--defaults", sitl_defaults))
    if dist is not None:
        # `run_swarm.sh` lays vehicles out on a grid whose step is this value
        # (default 50 m, per_row 3), so by default no two aircraft share a
        # start point. A matrix that varies only configuration needs identical
        # initial conditions, and `dist=0` collapses the grid to a single point:
        # `home_coords()` uses it purely as a multiplier, and separate SITL
        # processes have no collision model, so co-located vehicles are safe.
        command.extend(("--dist", str(dist)))
    (case_dir / "swarm.cmd.json").write_text(
        json.dumps(command, indent=2), encoding="utf-8"
    )
    process = subprocess.Popen(
        command,
        cwd=str(WORKTREE),
        stdout=(case_dir / "swarm.out.log").open("wb"),
        stderr=(case_dir / "swarm.err.log").open("wb"),
    )
    verdict = wait_for_eval_chat(
        process,
        600.0,
        speedup=speedup,
        stderr_log=case_dir / "swarm.err.log",
        launch_token=launch_token,
    )
    return process, verdict


def _start_mission(master: mavutil.mavfile) -> None:
    mode_map = master.mode_mapping()
    auto_mode = None if mode_map is None else mode_map.get("AUTO")
    if auto_mode is None:
        raise RuntimeError("AUTO mode is unavailable")
    require_nav_solution(master)
    master.set_mode(auto_mode)
    time.sleep(1.0)
    command_long(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0)
    time.sleep(1.0)
    command_long(master, mavutil.mavlink.MAV_CMD_MISSION_START, 0.0, 0.0)


def _fly(
    master: mavutil.mavfile,
    process: subprocess.Popen[bytes],
    case_dir: Path,
    *,
    sysid: int,
    scorer: CoordinateScorer,
    timeout_s: float,
    track: object | None = None,
    truth: object | None = None,
) -> dict[str, object]:
    from collections import deque

    live_anchors: deque[PositionStreamAnchor] = deque(maxlen=2)
    scoring_interval_seen = False
    deadline_s = time.monotonic() + timeout_s
    result_path = case_dir / "result.json"
    while time.monotonic() < deadline_s:
        now_scoring_active = (case_dir / "scoring_active.marker").exists()
        scoring_interval_seen = scoring_interval_seen or now_scoring_active
        drain_position_messages(
            master,
            sysid=sysid,
            live_anchors=live_anchors,
            scorer=scorer if scoring_interval_seen else None,
            track=track if scoring_interval_seen else None,
            truth=truth,
            truth_scoring_active=scoring_interval_seen,
        )
        if result_path.exists():
            # The child declares its pass two aft frames after CPA, so the
            # truth tail needed for closure certification may still be in
            # flight.  Keep draining (receive-only) until the recorder has its
            # post-CPA evidence or a named deadline expires -- without this a
            # perfectly good run is unscored nondeterministically.  The EKF
            # scorer and ground track are FROZEN here (None): the child's
            # result ends their episode, and feeding them the post-pass
            # trajectory could invalidate the run on post-episode maneuvers
            # or distort the wind classification.  The truth tail is bounded
            # by closure_ready() -- the drain stops at the first sufficient
            # post-CPA evidence, so a later re-approach cannot enter it.
            closure_deadline_s = (
                time.monotonic() + TRUTH_CLOSURE_DRAIN_TIMEOUT_S
            )
            while truth is not None and not truth.closure_ready():
                drain_position_messages(
                    master,
                    sysid=sysid,
                    live_anchors=live_anchors,
                    scorer=None,
                    track=None,
                    truth=truth,
                    truth_scoring_active=scoring_interval_seen,
                )
                if time.monotonic() >= closure_deadline_s:
                    break
                time.sleep(0.01)
            return json.loads(result_path.read_text(encoding="utf-8"))
        if process.poll() is not None:
            raise RuntimeError(f"direct pixel child exited code={process.returncode}")
        time.sleep(0.01)
    raise TimeoutError("direct pixel navigation produced no result")


class SourceChangedError(RuntimeError):
    """Raised when the code under test changes while a sweep is in flight."""


def run_case(
    python: Path,
    root: Path,
    *,
    speedup: float,
    repetition: int,
    args: argparse.Namespace,
    baseline_identity: dict[str, object] | None = None,
) -> dict[str, object]:
    if args.poi_alt < MIN_SAFE_POI_REL_ALT_M:
        return unscored_result(
            [
                "direct POI relative altitude must be at least "
                f"{MIN_SAFE_POI_REL_ALT_M:g}m for terrain-safe isolation; "
                f"got {args.poi_alt:g}m"
            ],
            scoring_policy=args.scoring_policy,
        )
    case_dir = root / f"speed-{speedup:g}-run-{repetition}"
    case_dir.mkdir(parents=True)
    swarm = None
    child = None
    master = None
    chat = None
    truth_recorder = None
    scorer = None
    child_result = None
    # None = never requested (estimate policy); a bool records the ACK.
    # Bound BEFORE the try (with the scorer and child result) so a failure
    # at any point still reports what was actually observed instead of
    # erasing it back to "never requested" / "never produced".
    truth_stream_accepted = None
    sim_cpa = SimCpaStage(args)
    stack_stopped = False
    try:
        stop_own_stack(python)
        swarm, verdict = _start_swarm(
            python,
            case_dir,
            launch_speedup(args.cruise_speedup, speedup),
            instances=1,
            home=getattr(args, "home", None),
        )
        chat = verdict.chat
        sysid = ip.sysids_for_chat(chat)[0]
        # Upload before opening the evaluator's own link: both bind the same
        # monitor port.  Without this the run flies whatever mission SITL boots
        # with -- the launcher's coverage plan -- whose legs turn, so wind
        # relative to the ground track would change mid-run and the sweep would
        # measure the route instead of the navigation.
        upload_north_line(
            ip.monitor_device(chat),
            sysid,
            home=_home_lat_lon(args.home),
            loiter_offset=args.loiter_offset,
            gate_offset=args.gate_offset,
            waypoint_offset=args.poi_offset,
            alt_m=args.mission_alt,
            echo=lambda line: (case_dir / "mission.log").open(
                "a", encoding="utf-8").write(f"{line}\n"),
        )
        master = wait_for_heartbeat(ip.monitor_device(chat), 120.0)
        if master is None:
            raise RuntimeError("no evaluator heartbeat")
        if not request_coordinate_score_stream(master):
            raise RuntimeError("30 Hz coordinate stream was not acknowledged")
        if args.scoring_policy == SCORING_POLICY_SITL_TRUTH:
            # Per-channel request on the evaluator link; it cannot change the
            # companion's dedicated serial0 streams. The ACK does not prove
            # delivery -- the recorder certifies the observed stream.
            truth_stream_accepted = request_message_interval_stream(
                master,
                mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE,
                TRUTH_SCORE_RATE_HZ,
            )
        mission = download_mission(master)
        home_alt = resolve_home_abs_alt_m(master, timeout_s=30.0)
        poi = resolve_poi_expectation(
            mission,
            poi_wp=args.poi_wp,
            poi_rel_alt_m=args.poi_alt,
            home_abs_alt_m=home_alt,
        ).location
        run_navigation_episode = resolve_poi_expectation(
            mission,
            poi_wp=args.scoring_start_wp,
            poi_rel_alt_m=args.poi_alt,
            home_abs_alt_m=home_alt,
        )
        # Checked HERE, not only before the case: SITL bring-up, mission upload
        # and parameter setting take minutes, and the child below is the
        # process that actually loads the law. An edit landing inside that
        # window would otherwise fly new code under an old baseline.
        case_identity = _source_identity()
        if baseline_identity is not None and case_identity != baseline_identity:
            raise SourceChangedError(
                f"source changed during case setup "
                f"({baseline_identity['sha256'][:12]} -> "
                f"{case_identity['sha256'][:12]})"
            )
        speed_plan = final_approach_speed_plan(args, speedup, mission, home_alt)
        # The manifest lands BEFORE any parameter push: a case that dies on
        # an unserved parameter (an old binary, a transport fault) must
        # still leave the record of what it was configured to fly
        # (config-stage legibility).
        write_case_manifest(
            case_dir,
            speedup=speedup,
            launch_speedup=launch_speedup(args.cruise_speedup, speedup),
            repetition=repetition,
            poi=poi,
            scoring_start_seq=run_navigation_episode.mission_seq,
            identity=case_identity,
            speed_plan=speed_plan,
            args=args,
            sitl_params_pushed=sim_parameters(args),
        )
        for name, value in sim_parameters(args):
            if not set_param(master, name, value):
                raise RuntimeError(f"parameter echo failed: {name}")
        sim_cpa.pre_flight(
            master, sysid=sysid, poi=poi, case_dir=case_dir
        )
        child = _launch_child(
            python,
            case_dir,
            device=ip.companion_device(sysid),
            sysid=sysid,
            poi=poi,
            scoring_start_seq=run_navigation_episode.mission_seq,
            timeout_s=args.timeout,
            speed_plan=speed_plan,
        )
        _wait_ready(case_dir / "child.out.log", child, 90.0)
        _start_mission(master)
        scorer = CoordinateScorer(poi)
        track = GroundTrackRecorder(poi)
        if args.scoring_policy == SCORING_POLICY_SITL_TRUTH:
            truth_recorder = TruthRecorder(poi, home_alt)
        child_result = _fly(
            master,
            child,
            case_dir,
            sysid=sysid,
            scorer=scorer,
            timeout_s=args.timeout,
            track=track,
            truth=truth_recorder,
        )
        if truth_recorder is not None:
            truth_recorder.finalize()
            truth_recorder.write_track(case_dir / "truth_track.csv")
        verdict = case_verdict(
            case_dir,
            child_result=child_result,
            scorer=scorer,
            track=track,
            truth=truth_recorder,
            truth_stream_accepted=truth_stream_accepted,
            scoring_policy=args.scoring_policy,
            max_distance_m=args.max_distance,
            goal_distance_m=args.goal_distance,
            wind_speed=args.wind_speed,
            wind_dir_deg=args.wind_dir,
            speedup=speedup,
            persist=False,
        )
        # Ordered teardown BEFORE the module reads anything: the BIN the
        # cross-check parses is only trustworthy once this chat's SITL has
        # exited and flushed it.  The finally below stays as the backstop
        # for every earlier exit path.
        master.close()
        master = None
        _terminate(child)
        child = None
        try:
            stop_own_stack(python, chat=chat)
        except Exception:
            pass
        stack_stopped = True
        _terminate(swarm)
        swarm = None
        verdict["sim_cpa"] = sim_cpa.post_teardown(
            case_dir, sysid=sysid, truth_block=verdict.get("truth")
        )
        persist_verdict(case_dir, verdict)
        return verdict
    except SourceChangedError:
        # Must not degrade into a failed case: a mid-sweep source change
        # invalidates the whole matrix, so it has to reach the caller and stop
        # it. The `finally` below still tears this instance down.
        raise
    except Exception as error:
        errors = [f"{type(error).__name__}: {error}"]
        truth_block = salvage_truth(truth_recorder, case_dir, errors)
        return unscored_result(
            errors,
            scoring_policy=args.scoring_policy,
            truth_stream_acknowledged=truth_stream_accepted,
            truth_block=truth_block,
            child_result=child_result,
            scorer=scorer,
            case_dir=case_dir,
            sim_cpa=sim_cpa.error_block(),
        )
    finally:
        if master is not None:
            master.close()
        _terminate(child)
        if not stack_stopped:
            try:
                stop_own_stack(python, chat=chat)
            except Exception:
                pass
        _terminate(swarm)


def main() -> int:
    args = _parser(scoring_policy=True, sim_cpa=True).parse_args()
    # Resolved (and override-validated) BEFORE anything launches: a
    # redirected cross-check POI must die here, not minutes into a case.
    args.sim_cpa_mode = resolve_mode(args.sim_cpa, args.sitl_param)
    python = args.python.resolve()
    speeds = _speeds(args.speedups)
    if args.repetitions < 1:
        raise ValueError("repetitions must be positive")
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    root = WORKTREE / ".sitl-runs" / f"direct-pixel-pn-{timestamp}"
    root.mkdir(parents=True)
    baseline_identity = _source_identity()
    (root / "source_identity.json").write_text(
        json.dumps(baseline_identity, indent=2), encoding="utf-8"
    )
    results = []
    aborted_identity: dict[str, object] | None = None
    for speedup in speeds:
        if aborted_identity is not None:
            break
        for repetition in range(1, args.repetitions + 1):
            # Recording identity per case is not enough: a matrix that keeps
            # running across an edit produces runs that cannot be compared to
            # each other, which is the failure that voided 20260814-001114.
            # Stop at the boundary instead of emitting a mixed sweep.
            current_identity = _source_identity()
            if current_identity != baseline_identity:
                aborted_identity = current_identity
                print(
                    "ABORT: source changed mid-sweep "
                    f"({baseline_identity['sha256'][:12]} -> "
                    f"{current_identity['sha256'][:12]}); "
                    f"completed {len(results)} run(s) are comparable, "
                    "anything after this point would not be.",
                    file=sys.stderr,
                )
                break
            try:
                result = run_case(
                    python,
                    root,
                    speedup=speedup,
                    repetition=repetition,
                    args=args,
                    baseline_identity=baseline_identity,
                )
            except SourceChangedError as error:
                aborted_identity = _source_identity()
                print(f"ABORT: {error}", file=sys.stderr)
                break
            result.update({"speedup": speedup, "repetition": repetition})
            results.append(result)
            status = "PASS" if result["passed"] else "FAIL"
            print(f"{status} speed={speedup:g} run={repetition}: {result}", flush=True)
    # Final recheck: the per-case guard runs before the child launches, so a
    # change landing while the LAST case was flying would otherwise leave the
    # sweep looking clean.
    if aborted_identity is None:
        final_identity = _source_identity()
        if final_identity != baseline_identity:
            aborted_identity = final_identity
            print(
                "ABORT: source changed while the final case was in flight "
                f"({baseline_identity['sha256'][:12]} -> "
                f"{final_identity['sha256'][:12]})",
                file=sys.stderr,
            )
    summary = summarize(
        results,
        gate_m=args.max_distance,
        goal_m=args.goal_distance,
        source_identity=baseline_identity,
        aborted_identity=aborted_identity,
    )
    (root / "summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(
        f"{summary['runs_within_gate']}/{summary['runs']} within the "
        f"{args.max_distance:g}m gate, "
        f"{summary['runs_within_goal']}/{summary['runs']} within the "
        f"{args.goal_distance:g}m goal "
        f"({summary['runs_goal_unscored']} unscored)",
        flush=True,
    )
    print(f"ARTIFACT {root}")
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
