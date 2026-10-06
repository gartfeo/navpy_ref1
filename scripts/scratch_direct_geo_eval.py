"""SCRATCH DIAGNOSTIC (Step-0, Rule-1 isolation) -- DO NOT COMMIT, DELETE AFTER USE.

Direct GEO POI evaluator: reuses the direct-pixel eval stack (SITL launch,
north-line mission, wind params, coordinate scorer, ground track, freshness)
and launches scratch_direct_geo_child.py (legacy 'pn' on truth geo, -udt true).

Measurement parity with eval_direct_pixel_pn.py (per review) -- the verdict
gates are the base evaluator's, imported or mirrored line for line:
- child pass gate, source metrics gates, SNAP gate, coordinate CPA gate,
  scorer certification, ground track, observation freshness gates
- same summary shape, same gate/goal counts from --max-distance and
  --goal-distance, same repetitions guard
Justified differences (documented, not silent):
- command causality check SKIPPED: it recomputes the vision-law formula from
  TERMINAL_RESPONSE_STATE events, which the legacy law does not emit
- midcourse pitch-step gate SKIPPED (stability numbers still recorded): the
  legacy law's scoring interval feedforward step (~25 deg) is its designed behavior,
  the 2 deg midcourse gate encodes vision-law smoothness expectations
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS))

import eval_direct_pixel_pn as base  # noqa: E402
from eval_navigation_cases import (  # noqa: E402
    CoordinateScorer,
    download_mission,
    request_coordinate_score_stream,
    resolve_home_abs_alt_m,
    resolve_poi_expectation,
    set_param,
    stop_own_stack,
    wait_for_heartbeat,
)
from eval_ground_track import GroundTrackRecorder  # noqa: E402
from gcs.backend import instance_ports as ip  # noqa: E402
from upload_north_line_mission import upload_north_line  # noqa: E402


def run_case(
    python: Path,
    root: Path,
    *,
    speedup: float,
    repetition: int,
    args: argparse.Namespace,
) -> dict[str, object]:
    if args.poi_alt < base.MIN_SAFE_POI_REL_ALT_M:
        return {
            "passed": False,
            "errors": [
                "direct POI relative altitude must be at least "
                f"{base.MIN_SAFE_POI_REL_ALT_M:g}m for terrain-safe "
                f"isolation; got {args.poi_alt:g}m"
            ],
        }
    case_dir = root / f"speed-{speedup:g}-run-{repetition}"
    case_dir.mkdir(parents=True)
    swarm = None
    child = None
    master = None
    chat = None
    try:
        stop_own_stack(python)
        swarm, verdict = base._start_swarm(
            python, case_dir, speedup, instances=1,
            home=getattr(args, "home", None),
        )
        chat = verdict.chat
        sysid = ip.sysids_for_chat(chat)[0]
        lat, lon = base._home_lat_lon(args.home)
        upload_north_line(
            ip.monitor_device(chat),
            sysid,
            home=(lat, lon),
            gate_offset=args.gate_offset,
            waypoint_offset=args.poi_offset,
            alt_m=args.mission_alt,
            echo=lambda line: (case_dir / "mission.log").open(
                "a", encoding="utf-8"
            ).write(f"{line}\n"),
        )
        master = wait_for_heartbeat(ip.monitor_device(chat), 120.0)
        if master is None:
            raise RuntimeError("no evaluator heartbeat")
        if not request_coordinate_score_stream(master):
            raise RuntimeError("30 Hz coordinate stream was not acknowledged")
        mission = download_mission(master)
        home_alt = resolve_home_abs_alt_m(master, timeout_s=30.0)
        poi = resolve_poi_expectation(
            mission,
            poi_wp=args.poi_wp,
            poi_rel_alt_m=args.poi_alt,
            home_abs_alt_m=home_alt,
        ).location
        scoring_start_expectation = resolve_poi_expectation(
            mission,
            poi_wp=args.scoring_start_wp,
            poi_rel_alt_m=args.poi_alt,
            home_abs_alt_m=home_alt,
        )
        for name, value in (
            ("ARMING_CHECK", 0.0),
            ("SIM_WIND_SPD", args.wind_speed),
            ("SIM_WIND_DIR", args.wind_dir),
        ):
            if not set_param(master, name, value):
                raise RuntimeError(f"parameter echo failed: {name}")
        (case_dir / "case.json").write_text(
            json.dumps(
                {
                    "speedup": speedup,
                    "repetition": repetition,
                    "poi": asdict(poi),
                    "engage_seq": scoring_start_expectation.mission_seq,
                    "mode": "direct-geo-legacy-pn",
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        child = base._launch_child(
            python,
            case_dir,
            device=ip.companion_device(sysid),
            sysid=sysid,
            poi=poi,
            scoring_start_seq=scoring_start_expectation.mission_seq,
            timeout_s=args.timeout,
            child_script=SCRIPTS / "scratch_direct_geo_child.py",
        )
        base._wait_ready(case_dir / "child.out.log", child, 90.0)
        base._start_mission(master)
        scorer = CoordinateScorer(poi)
        track = GroundTrackRecorder(poi)
        child_result = base._fly(
            master,
            child,
            case_dir,
            sysid=sysid,
            scorer=scorer,
            timeout_s=args.timeout,
            track=track,
        )
        closest = scorer.result
        coordinate = None if closest is None else asdict(closest)
        errors = []
        ground_track = None
        try:
            track.write(case_dir / "ground_track.csv")
            ground_track = track.summary(args.wind_speed, args.wind_dir)
        except (OSError, RuntimeError, ValueError) as error:
            errors.append(f"ground track unavailable: {error}")
        if child_result.get("passed") is not True:
            errors.append(f"POI pass false: {child_result.get('error', '')}")
        source = child_result.get("source")
        if not isinstance(source, dict):
            errors.append("direct pixel source metrics missing")
        else:
            if source.get("projection_failures") != 0:
                errors.append(
                    "POI left renderable sight: "
                    f"{source.get('projection_failures')} projection failures"
                )
            if not isinstance(source.get("delivered_frames"), int) or (
                source["delivered_frames"] < 10
            ):
                errors.append(
                    "insufficient direct pixel deliveries: "
                    f"{source.get('delivered_frames')!r}"
                )
        snap = child_result.get("snap_3d_m")
        if not isinstance(snap, (int, float)) or snap > args.max_distance:
            errors.append(f"SNAP {snap!r} exceeds {args.max_distance:g}m")
        if scorer.certification_error:
            errors.append(scorer.certification_error)
        if closest is None or closest.dist_3d_m > args.max_distance:
            measured = None if closest is None else closest.dist_3d_m
            errors.append(
                f"coordinate CPA {measured!r} exceeds {args.max_distance:g}m"
            )
        stability = base._command_stability(case_dir)
        freshness = None
        try:
            freshness = base._observation_freshness(
                case_dir,
                speedup,
                source if isinstance(source, dict) else {},
            )
        except (OSError, RuntimeError, ValueError) as error:
            errors.append(f"observation freshness unavailable: {error}")
        if freshness is not None:
            errors.extend(base.freshness_errors(freshness))
        # SKIPPED vs base: command causality (vision-law formula recompute)
        # and the midcourse pitch-step gate; stability is still recorded.
        result = {
            "passed": not errors,
            "errors": errors,
            "child": child_result,
            "coordinate": coordinate,
            "coordinate_samples": scorer.sample_count,
            "command_stability": stability,
            "observation_freshness": freshness,
            "ground_track": ground_track,
        }
        (case_dir / "verdict.json").write_text(
            json.dumps(result, indent=2), encoding="utf-8"
        )
        return result
    except Exception as error:
        return {"passed": False, "errors": [f"{type(error).__name__}: {error}"]}
    finally:
        if master is not None:
            master.close()
        base._terminate(child)
        try:
            stop_own_stack(python, chat=chat)
        except Exception:
            pass
        base._terminate(swarm)


def main() -> int:
    args = base._parser().parse_args()
    python = args.python.resolve()
    speeds = base._speeds(args.speedups)
    if args.repetitions < 1:
        raise ValueError("repetitions must be positive")
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    root = base.WORKTREE / ".sitl-runs" / f"direct-geo-pn-{timestamp}"
    root.mkdir(parents=True)
    results = []
    for speedup in speeds:
        for repetition in range(1, args.repetitions + 1):
            result = run_case(
                python,
                root,
                speedup=speedup,
                repetition=repetition,
                args=args,
            )
            snap = (result.get("child") or {}).get("snap_3d_m")
            result.update({
                "speedup": speedup,
                "repetition": repetition,
                "meets_goal": (
                    isinstance(snap, (int, float)) and snap < args.goal_distance
                ),
            })
            results.append(result)
            status = "PASS" if result["passed"] else "FAIL"
            print(f"{status} speed={speedup:g} run={repetition}: {result}", flush=True)
    goal_met = sum(1 for row in results if row.get("meets_goal"))
    summary = {
        "passed": all(row["passed"] for row in results),
        "gate_m": args.max_distance,
        "goal_m": args.goal_distance,
        "runs": len(results),
        "runs_within_gate": sum(1 for row in results if row["passed"]),
        "runs_within_goal": goal_met,
        "results": results,
    }
    (root / "summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(
        f"{summary['runs_within_gate']}/{summary['runs']} within the "
        f"{args.max_distance:g}m gate, {goal_met}/{summary['runs']} within the "
        f"{args.goal_distance:g}m goal",
        flush=True,
    )
    print(f"ARTIFACT {root}")
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
