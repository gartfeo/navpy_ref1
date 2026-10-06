"""Three-UAV load test for direct geo-to-pixel pure-vision PN."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from pymavlink import mavutil

WORKTREE = Path(__file__).resolve().parent.parent
SCRIPTS = Path(__file__).resolve().parent
sys.path[:0] = [str(WORKTREE / "src"), str(WORKTREE), str(SCRIPTS)]

from gcs.backend import instance_ports as ip  # noqa: E402
from scripts import eval_direct_pixel_pn as one  # noqa: E402
from scripts.eval_navigation_cases import (  # noqa: E402
    CoordinateScorer, command_long, require_fleet_nav_solution, stop_own_stack, wait_for_heartbeat,
)
from scripts.eval_ground_track import GroundTrackRecorder  # noqa: E402
from scripts.eval_sim_parameters import sim_parameters  # noqa: E402
from scripts.eval_navigation_telemetry import position_sample_from_message  # noqa: E402
from scripts.eval_three_uav_mission import (  # noqa: E402
    LAUNCH_SPACING_M, geometry_record, prepare_vehicles, upload_missions)


def _parser() -> argparse.ArgumentParser:
    # Wind/home flags come from the shared parser; redefining them here would
    # be a duplicate-argument error.
    parser = one._parser(cruise_speedup=False)  # three vehicles, one clock each
    parser.set_defaults(speedups="10", repetitions=1)
    return parser


def _select(master: object, sys_id: int) -> None:
    master.target_system = sys_id
    master.target_component = 1


def _start_missions(master: object, sys_ids: list[int]) -> None:
    mode_map = master.mode_mapping()
    auto_mode = None if mode_map is None else mode_map.get("AUTO")
    if auto_mode is None:
        raise RuntimeError("AUTO mode is unavailable")
    require_fleet_nav_solution(master, sys_ids, select=_select)
    for sys_id in sys_ids:
        _select(master, sys_id)
        master.set_mode(auto_mode)
        command_long(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0)
        command_long(master, mavutil.mavlink.MAV_CMD_MISSION_START, 0.0, 0.0)


def _collect(
    master: object,
    children: dict[int, object],
    directories: dict[int, Path],
    scorers: dict[int, CoordinateScorer],
    tracks: dict[int, GroundTrackRecorder],
    timeout_s: float,
) -> dict[int, dict[str, object]]:
    results: dict[int, dict[str, object]] = {}
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s and len(results) < len(children):
        for _ in range(500):
            message = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
            if message is None:
                break
            sys_id = int(message.get_srcSystem())
            directory = directories.get(sys_id)
            if directory is not None and (directory / "engaged.marker").exists():
                scorers[sys_id].add(position_sample_from_message(message, time.time()))
                # Same message, velocity fields the scorer's PositionSample drops.
                tracks[sys_id].add(message)
        for sys_id, child in children.items():
            if sys_id in results:
                continue
            result_path = directories[sys_id] / "result.json"
            if result_path.exists():
                results[sys_id] = json.loads(result_path.read_text(encoding="utf-8"))
            elif child.poll() is not None:
                raise RuntimeError(
                    f"direct pixel child sysid={sys_id} exited code={child.returncode}"
                )
        time.sleep(0.01)
    missing = sorted(set(children) - set(results))
    if missing:
        raise TimeoutError(f"direct pixel results timed out for sysids {missing}")
    return results


def _verdict(
    child_result: dict[str, object],
    scorer: CoordinateScorer,
    directory: Path,
    max_distance_m: float,
    speedup: float,
    track: GroundTrackRecorder | None = None,
    wind: tuple[float, float] = (0.0, 0.0),
) -> dict[str, object]:
    closest = scorer.result
    source = child_result.get("source")
    errors: list[str] = []
    if child_result.get("passed") is not True:
        errors.append(f"target pass false: {child_result.get('error', '')}")
    if not isinstance(source, dict):
        errors.append("direct pixel source metrics missing")
    else:
        if source.get("projection_failures") != 0:
            errors.append(
                f"target left renderable sight: {source.get('projection_failures')} failures"
            )
        if not isinstance(source.get("delivered_frames"), int) or source["delivered_frames"] < 10:
            errors.append(f"insufficient direct pixel deliveries: {source.get('delivered_frames')!r}")
    snap = child_result.get("snap_3d_m")
    if not isinstance(snap, (int, float)) or snap > max_distance_m:
        errors.append(f"SNAP {snap!r} exceeds {max_distance_m:g}m")
    if scorer.certification_error:
        errors.append(scorer.certification_error)
    if closest is None or closest.dist_3d_m > max_distance_m:
        measured = None if closest is None else closest.dist_3d_m
        errors.append(f"coordinate CPA {measured!r} exceeds {max_distance_m:g}m")
    stability = None
    try:
        stability = one._command_stability(directory)
    except (OSError, RuntimeError, ValueError) as error:
        errors.append(f"command stability unavailable: {error}")
    freshness = None
    try:
        freshness = one._observation_freshness(
            directory,
            speedup,
            source if isinstance(source, dict) else {},
        )
    except (OSError, RuntimeError, ValueError) as error:
        errors.append(f"observation freshness unavailable: {error}")
    if freshness is not None:
        errors.extend(one.freshness_errors(freshness))
    ground_track = None
    if track is not None:
        try:
            track.write(directory / "ground_track.csv")
            ground_track = track.summary(wind[0], wind[1])
        except (OSError, RuntimeError, ValueError) as error:
            errors.append(f"ground track unavailable: {error}")
    debug_paths = list((directory / "navpy-logs").glob("*_navigation_debug.csv"))
    causality = None
    if len(debug_paths) != 1:
        errors.append(f"expected one debug navigation log, found {len(debug_paths)}")
    else:
        causality = one.causality_evidence(debug_paths[0], errors)
    if stability is not None and (
        stability["pitch_max_step_deg"] > one.MAX_MIDCOURSE_PITCH_STEP_DEG
    ):
        errors.append(
            f"midcourse pitch step {stability['pitch_max_step_deg']:.3f}deg exceeds "
            f"{one.MAX_MIDCOURSE_PITCH_STEP_DEG:.3f}deg"
        )
    if stability is not None:
        # label, stability key, limit -- one row per terminal alignment gate.
        alignment_limits = (
            ("body bearing", "alignment_bearing_mean_abs_deg",
             one.MAX_ALIGNMENT_MEAN_ABS_BEARING_DEG),
            ("desired roll", "alignment_cmd_roll_mean_abs_deg",
             one.MAX_ALIGNMENT_MEAN_ABS_ROLL_DEG),
            ("actual roll", "alignment_actual_roll_mean_abs_deg",
             one.MAX_ALIGNMENT_MEAN_ABS_ROLL_DEG),
        )
        for label, key, limit in alignment_limits:
            measured = stability[key]
            if measured > limit:
                errors.append(
                    f"terminal {label} mean abs {measured:.3f}deg exceeds "
                    f"{limit:.3f}deg"
                )
    return {
        "passed": not errors,
        "errors": errors,
        "snap_horizontal_min_m": child_result.get("snap_horizontal_m"),
        "snap_vertical_min_m": child_result.get("snap_vertical_m"),
        "coordinate_horizontal_min_m": (
            None if closest is None else closest.horizontal_m
        ),
        "coordinate_vertical_min_m": (
            None if closest is None else closest.vertical_m
        ),
        "child": child_result,
        "coordinate": None if closest is None else closest.__dict__,
        "coordinate_samples": scorer.sample_count,
        "command_stability": stability,
        "observation_freshness": freshness,
        "ground_track": ground_track,
        "command_causality": causality,
    }


def run_case(
    python: Path,
    root: Path,
    *,
    speedup: float,
    repetition: int,
    args: argparse.Namespace,
) -> dict[str, object]:
    if args.target_alt < one.MIN_SAFE_TARGET_REL_ALT_M:
        raise ValueError(
            f"target altitude must be at least {one.MIN_SAFE_TARGET_REL_ALT_M:g}m"
        )
    if (args.loiter_offset, args.gate_offset, args.target_offset) != (
            one.DEFAULT_LOITER_OFFSET_M, one.DEFAULT_GATE_OFFSET_M,
            one.DEFAULT_WAYPOINT_OFFSET_M):
        raise ValueError("three-UAV runs cannot apply geometry flags")
    case_dir = root / f"speed-{speedup:g}-run-{repetition}"
    case_dir.mkdir(parents=True)
    swarm = master = None
    children: dict[int, object] = {}
    chat = None
    try:
        stop_own_stack(python)
        swarm, launch = one._start_swarm(
            python,
            case_dir,
            speedup,
            instances=3,
            home=getattr(args, "home", None),
            # Collapse the launcher's 50 m grid: co-located starts make the
            # three runs replicates, and vehicles 2-3 would fail the 25 m home
            # check.
            dist=LAUNCH_SPACING_M,
        )
        chat = launch.chat
        sys_ids = ip.sysids_for_chat(chat)
        upload_missions(
            chat,
            sys_ids,
            home=one._home_lat_lon(args.home),
            loiter_offset=args.loiter_offset,
            gate_offset=args.gate_offset,
            waypoint_offset=args.target_offset,
            alt_m=args.mission_alt,
            case_dir=case_dir,
        )
        master = wait_for_heartbeat(ip.monitor_device(chat), 120.0)
        if master is None:
            raise RuntimeError("no evaluator heartbeat")
        targets, scoring_start_sequences = prepare_vehicles(
            master,
            sys_ids,
            select=_select,
            target_wp=args.target_wp,
            scoring_start_wp=args.scoring_start_wp,
            target_rel_alt_m=args.target_alt,
            parameters=sim_parameters(args),
        )
        directories = {sys_id: case_dir / f"uav-{sys_id}" for sys_id in sys_ids}
        for directory in directories.values():
            directory.mkdir()
        children = {
            sys_id: one._launch_child(
                python,
                directories[sys_id],
                device=ip.companion_device(sys_id),
                sysid=sys_id,
                target=targets[sys_id],
                scoring_start_seq=scoring_start_sequences[sys_id],
                timeout_s=args.timeout,
            )
            for sys_id in sys_ids
        }
        for sys_id, child in children.items():
            one._wait_ready(directories[sys_id] / "child.out.log", child, 90.0)
        _start_missions(master, sys_ids)
        scorers = {sys_id: CoordinateScorer(targets[sys_id]) for sys_id in sys_ids}
        tracks = {sys_id: GroundTrackRecorder(targets[sys_id]) for sys_id in sys_ids}
        child_results = _collect(
            master, children, directories, scorers, tracks, args.timeout
        )
        vehicles = {
            str(sys_id): _verdict(
                child_results[sys_id],
                scorers[sys_id],
                directories[sys_id],
                args.max_distance,
                speedup,
                tracks[sys_id],
                (args.wind_speed, args.wind_dir),
            )
            for sys_id in sys_ids
        }
        result = {
            "passed": all(row["passed"] for row in vehicles.values()),
            # Recorded like the one-UAV manifest: raw overrides AND the
            # effective push list, so a default change stays visible.
            "sitl_params": list(args.sitl_param or []),
            "sitl_params_pushed": [list(pair) for pair in sim_parameters(args)],
            "vehicles": vehicles,
        }
        (case_dir / "verdict.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        return result
    finally:
        if master is not None:
            master.close()
        for child in children.values():
            one._terminate(child)
        try:
            stop_own_stack(python, chat=chat)
        except Exception:
            pass
        one._terminate(swarm)


def main() -> int:
    args = _parser().parse_args()
    root = one.WORKTREE / ".sitl-runs" / f"direct-pixel-pn-3uav-{time.strftime('%Y%m%d-%H%M%S')}"
    root.mkdir(parents=True)
    results = []
    for speedup in one._speeds(args.speedups):
        for repetition in range(1, args.repetitions + 1):
            try:
                result = run_case(
                    args.python.resolve(), root, speedup=speedup, repetition=repetition, args=args
                )
            except Exception as error:
                result = {"passed": False, "errors": [f"{type(error).__name__}: {error}"]}
            result.update({
                "speedup": speedup,
                "repetition": repetition,
                **geometry_record(args),
            })
            results.append(result)
            print(("PASS" if result["passed"] else "FAIL"), speedup, repetition, result, flush=True)
    summary = {"passed": all(row["passed"] for row in results), "results": results}
    (root / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"ARTIFACT {root}")
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
