"""CLI for the scenario-driven GCS final approach evaluator."""

from __future__ import annotations

import argparse
import dataclasses
import json
import tempfile
from dataclasses import asdict
from pathlib import Path
from typing import Sequence, cast

from navpy.sim.lockstep.host_paths import HostPathScrubber
from navpy.sim.lockstep.sim_profile import DEFAULT_PROFILE, profile_names
from scripts.eval_gcs_demo_air_load import air_load
from scripts.eval_gcs_demo_analysis import analyze_run
from scripts.eval_gcs_demo_fleet_packets import unverified_sources
from scripts.eval_gcs_demo_inputs import recorded_profile_parameters
from scripts.eval_gcs_demo_models import (
    GateLimits,
    RegressionError,
    RunReport,
    finite_number,
)
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_process import exclusive_evaluator_lock
from scripts.eval_gcs_demo_slice_gates import JOURNAL_FILE
from scripts.eval_gcs_demo_lockstep import LockstepOptions, execute_lockstep_run
from scripts.eval_gcs_demo_runtime import execute_live_run
from scripts.eval_delivery_fixture import load_eval_scenario, prepare_sim_docks, scenario_path
from scripts.eval_gcs_demo_gate_profiles import DELIVERY_PROFILE
from scripts.eval_gcs_demo_scenario import CERTIFIED_SCENARIO_PATH, DemoScenario


def report_dict(report: RunReport) -> dict[str, JsonValue]:
    return cast(dict[str, JsonValue], asdict(report))


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Exact source-driven GCS final-approach regression"
    )
    parser.add_argument(
        "--scenario",
        type=scenario_path,
        default=CERTIFIED_SCENARIO_PATH,
        metavar="PATH",
        help="scenario file to fly and gate, or 'delivery' for the delivery fixture "
        "(default: the certified three-UAV scenario)",
    )
    parser.add_argument(
        "--analyze-only",
        type=Path,
        metavar="LOG_DIR",
        help="analyze an existing run directory without launching GCS/SITL",
    )
    parser.add_argument(
        "--run-dir",
        type=Path,
        help="persistent output directory (default: new system temp directory)",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=600.0,
        help="per-phase/mission wall timeout (default: 600)",
    )
    parser.add_argument(
        "--confirmation-delay-s",
        type=float,
        default=15.0,
        help="wall operator-review delay before exact-round approval (default: 15)",
    )
    parser.add_argument(
        "--navigation-speedup",
        type=float,
        default=0.0,
        help="optional NAV call-cadence speedup; zero leaves launch speed unchanged",
    )
    parser.add_argument(
        "--lockstep",
        action="store_true",
        help="run in lockstep on serial-step SITLs (the owner_only_slice or certified scenario); "
        "--timeout-s and --confirmation-delay-s are then sim seconds",
    )
    parser.add_argument(
        "--sim-dock-net-radius-m",
        type=float,
        default=None,
        help="live run only: shrink the sim net (0..0.5 m); 0 forces every pass to miss",
    )
    parser.add_argument("--seed", type=int, default=1, help="lockstep run seed (default: 1)")
    parser.add_argument(
        "--host-delay-seed",
        type=int,
        default=0,
        help="lockstep only: seeded host delays in the relay, coordinator and participants "
        "(acceptance (c)); 0 is none",
    )
    parser.add_argument(
        "--sim-profile",
        default=None,
        choices=profile_names(),
        help="lockstep only: the sim profile whose realism models the seed draws "
        "(src/navpy/sim/lockstep/sim_profiles.json; default: hw)",
    )
    return parser.parse_args(argv)


def run_lockstep(
    args: argparse.Namespace, scenario: DemoScenario, limits: GateLimits,
) -> RunReport:
    """One lockstep run, then the analysis with the scenario's gate profile.

    The report names the run directory as ``.`` (run-relative) and its errors
    name no host path (``HostPathScrubber``): it is one of the run's
    byte-equal artifacts, failed or not.
    """
    if args.navigation_speedup:
        raise RegressionError("--lockstep has no --navigation-speedup: its cadence factor is 1")
    if args.sim_dock_net_radius_m is not None:
        raise RegressionError("--lockstep has no --sim-dock-net-radius-m")
    run_dir = args.run_dir or Path(tempfile.mkdtemp(prefix="navpy-gcs-demo-lockstep-"))
    print(f"GCS demo lockstep artifacts: {run_dir.resolve()}", flush=True)
    options = LockstepOptions(args.seed, args.timeout_s, args.confirmation_delay_s,
                              args.host_delay_seed, args.sim_profile or DEFAULT_PROFILE)
    run = execute_lockstep_run(run_dir, scenario, options)
    errors = [f"lockstep outcome {run.outcome!r}, not 'ok'"] if run.outcome != "ok" else []
    if run.operator_error is not None:
        errors.append(f"operator: {run.operator_error}")
    try:
        report = analyze_run(run_dir, scenario=scenario, approvals=run.approvals, limits=limits)
    except (OSError, ValueError, TypeError, RegressionError) as error:
        report = RunReport(False, ".", [], 1.0, 0.0, run.approvals, {}, [],
                           [f"analysis: {type(error).__name__}: {error}"])
    # Errors may quote host paths (an analysis exception, a missing file):
    # they become run tokens, so report.json stays a byte-equal artifact.
    scrub = HostPathScrubber(run_dir)
    try:
        records = [json.loads(row) for row in
                   (run_dir / JOURNAL_FILE).read_text(encoding="utf-8").splitlines()]
        uart_loss = recorded_profile_parameters(run_dir).get("SIM_UART_LOSS", 0.0) > 0.0
        load: dict[str, JsonValue] = air_load(records, uart_loss)
        fleet = range(1, len(next(r["grid"] for r in records if "grid" in r)["sitl_ports"]) + 1)
        unverified: int | None = len(unverified_sources(records, tuple(fleet), uart_loss))
    except (OSError, ValueError, KeyError, TypeError, StopIteration, RegressionError) as error:
        load = {"error": scrub(f"{type(error).__name__}: {error}")}  # report only: no gate
        unverified = None
    report = dataclasses.replace(report, log_dir=".", passed=report.passed and not errors,
                                 errors=[scrub(error) for error in errors + report.errors],
                                 air_load=load, unverified_sources=unverified)
    (run_dir / "report.json").write_text(json.dumps(report_dict(report), indent=2),
                                         encoding="utf-8")
    print(f"lockstep journal digest: {run.digest}", flush=True)
    return report


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    timeout = finite_number("--timeout-s", args.timeout_s)
    if timeout <= 0.0:
        raise RegressionError("--timeout-s must be positive")
    delay = finite_number("--confirmation-delay-s", args.confirmation_delay_s)
    speedup = finite_number("--navigation-speedup", args.navigation_speedup)
    limits = GateLimits()
    if args.lockstep and args.analyze_only is not None:
        # Analysis reads the run's recorded execution mode; --lockstep would launch.
        raise RegressionError("--lockstep and --analyze-only are exclusive")
    try:
        scenario = load_eval_scenario(args.scenario)
    except (TypeError, ValueError) as error:
        raise RegressionError(f"invalid --scenario: {error}") from error
    if args.sim_profile is not None and not args.lockstep:
        raise RegressionError("--sim-profile is lockstep only")
    if args.lockstep and scenario.gate_profile == DELIVERY_PROFILE:
        raise RegressionError("the delivery scenario has no lockstep run")
    if args.lockstep:
        report = run_lockstep(args, scenario, limits)
    elif args.analyze_only is not None:
        report = analyze_run(
            args.analyze_only,
            scenario=scenario,
            limits=limits,
            navigation_speedup=speedup,
        )
    else:
        with exclusive_evaluator_lock():
            run_dir = args.run_dir or Path(
                tempfile.mkdtemp(prefix="navpy-gcs-demo-regression-")
            )
            print(f"GCS demo regression artifacts: {run_dir.resolve()}", flush=True)
            mission = execute_live_run(
                run_dir,
                timeout,
                delay,
                scenario=scenario,
                navigation_speedup=speedup,
                sim_docks=prepare_sim_docks(run_dir, scenario),
                sim_dock_net_radius_m=args.sim_dock_net_radius_m,
            )
            report = analyze_run(
                run_dir,
                scenario=scenario,
                sys_ids=mission.sys_ids,
                approvals=mission.approvals,
                limits=limits,
                navigation_speedup=speedup,
            )
            (run_dir / "report.json").write_text(
                json.dumps(report_dict(report), indent=2),
                encoding="utf-8",
            )
    print(json.dumps(report_dict(report), indent=2))
    return 0 if report.passed else 1


__all__ = ["main", "parse_args", "report_dict", "run_lockstep"]
