"""CLI for the exact three-UAV GCS final approach evaluator."""

from __future__ import annotations

import argparse
import json
import tempfile
from dataclasses import asdict
from pathlib import Path
from typing import Sequence, cast

from scripts.eval_gcs_demo_analysis import analyze_run
from scripts.eval_gcs_demo_models import (
    GateLimits,
    RegressionError,
    RunReport,
    TruthLimits,
    finite_number,
)
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_process import exclusive_evaluator_lock
from scripts.eval_gcs_demo_runtime import execute_live_run


def report_dict(report: RunReport) -> dict[str, JsonValue]:
    return cast(dict[str, JsonValue], asdict(report))


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Exact source-driven three-UAV GCS final-approach regression"
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
        "--max-snap-distance-m",
        type=float,
        default=GateLimits().max_snap_distance_m,
        help="strict truth SNAP distance gate",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    timeout = finite_number("--timeout-s", args.timeout_s)
    if timeout <= 0.0:
        raise RegressionError("--timeout-s must be positive")
    delay = finite_number("--confirmation-delay-s", args.confirmation_delay_s)
    speedup = finite_number("--navigation-speedup", args.navigation_speedup)
    limits = GateLimits(
        truth=TruthLimits(max_snap_distance_m=args.max_snap_distance_m),
    )
    if args.analyze_only is not None:
        report = analyze_run(
            args.analyze_only,
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
                navigation_speedup=speedup,
            )
            report = analyze_run(
                run_dir,
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


__all__ = ["main", "parse_args", "report_dict"]
