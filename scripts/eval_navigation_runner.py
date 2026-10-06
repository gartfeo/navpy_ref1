"""Matrix execution, durable row recording, and certificate summaries."""

from __future__ import annotations

import argparse
import csv
import json
from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

from scripts import eval_certificate as cert

from eval_navigation_cli import parse_args
from eval_navigation_matrix import matrix_cases
from eval_navigation_models import MatrixCase


@dataclass
class RunArtifacts:
    run_dir: Path
    attempts_csv: Path
    attempts_jsonl: Path
    final_csv: Path
    all_rows: list[dict[str, Any]] = field(default_factory=list)
    final_rows: list[dict[str, Any]] = field(default_factory=list)
    invalid_certificates: list[int] = field(default_factory=list)


def write_csv(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    """Write a union-column CSV while preserving first-seen column order."""
    if not rows:
        return
    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=fieldnames,
            extrasaction="ignore",
        )
        writer.writeheader()
        writer.writerows(rows)


def main(
    argv: Sequence[str] | None,
    *,
    run_case_fn: Callable[..., dict[str, Any]],
    worktree: Path,
) -> int:
    """Run all requested cases and return a process exit status."""
    args = parse_args(argv)
    cases = list(
        matrix_cases(
            args.target_alts,
            args.speedups,
            args.navigation_speedups,
            args.winds,
            args.directions,
        )
    )
    artifacts = _create_artifacts(worktree)
    _announce_policy(artifacts.run_dir, cases, args)
    for index, case in enumerate(cases):
        if args.repetitions is not None:
            _run_certificate_case(
                artifacts,
                index,
                case,
                cases,
                args,
                run_case_fn=run_case_fn,
            )
        else:
            _run_regular_case(
                artifacts,
                index,
                case,
                cases,
                args,
                run_case_fn=run_case_fn,
            )
    failures = [row for row in artifacts.final_rows if not row.get("passed")]
    print(f"FINAL_CSV {artifacts.final_csv}", flush=True)
    print(f"ATTEMPTS_CSV {artifacts.attempts_csv}", flush=True)
    print(f"FAILURES {len(failures)}", flush=True)
    if args.repetitions is not None:
        print(
            f"INVALID_CERTIFICATES {len(artifacts.invalid_certificates)}",
            flush=True,
        )
    return 1 if failures or artifacts.invalid_certificates else 0


def _create_artifacts(worktree: Path) -> RunArtifacts:
    run_id = datetime.now().strftime("eval-navigation-cases-%Y%m%d-%H%M%S")
    run_dir = worktree / ".sitl-runs" / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    return RunArtifacts(
        run_dir,
        run_dir / "attempts.csv",
        run_dir / "attempts.jsonl",
        run_dir / "final.csv",
    )


def _announce_policy(
    run_dir: Path,
    cases: Sequence[MatrixCase],
    args: argparse.Namespace,
) -> None:
    print(f"RUN_DIR {run_dir}", flush=True)
    print(f"CASES {len(cases)}", flush=True)
    print("SITL_POLICY isolated-eval-band one-aircraft", flush=True)
    nav_policy = (
        "match-sitl-speedup"
        if args.navigation_speedups is None
        else args.navigation_speedups
    )
    print(f"NAV_SPEEDUP_POLICY {nav_policy}", flush=True)
    print("SCORE_POLICY snap-plus-requested-coordinate", flush=True)
    if args.repetitions is not None:
        print(f"CERTIFICATE_REPETITIONS {args.repetitions}", flush=True)
        print(
            "CERTIFICATE_POLICY every-run-reported no-retry-substitution",
            flush=True,
        )


def _run_regular_case(
    artifacts: RunArtifacts,
    index: int,
    case: MatrixCase,
    cases: Sequence[MatrixCase],
    args: argparse.Namespace,
    *,
    run_case_fn: Callable[..., dict[str, Any]],
) -> None:
    selected = None
    for attempt in range(1, args.max_attempts + 1):
        row = run_case_fn(index, case, attempt, artifacts.run_dir, args)
        _record_row(
            artifacts,
            row,
            index,
            case,
            cases,
            args,
            label=f"attempt={attempt}",
        )
        selected = row
        if row["passed"]:
            break
    if selected is not None:
        artifacts.final_rows.append(selected)
        write_csv(artifacts.final_csv, artifacts.final_rows)


def _run_certificate_case(
    artifacts: RunArtifacts,
    index: int,
    case: MatrixCase,
    cases: Sequence[MatrixCase],
    args: argparse.Namespace,
    *,
    run_case_fn: Callable[..., dict[str, Any]],
) -> None:
    case_rows: list[dict[str, Any]] = []
    for repetition in range(1, args.repetitions + 1):
        row = run_case_fn(index, case, repetition, artifacts.run_dir, args)
        _record_row(
            artifacts,
            row,
            index,
            case,
            cases,
            args,
            label=f"repetition={repetition}",
        )
        case_rows.append(row)
        artifacts.final_rows.append(row)
        write_csv(artifacts.final_csv, artifacts.final_rows)
    summary = cert.certificate_summary(case_rows)
    (artifacts.run_dir / f"certificate-{index}.json").write_text(
        json.dumps(
            {"case": asdict(case), "summary": summary},
            indent=2,
            default=str,
        ),
        encoding="utf-8",
    )
    if not summary["valid"]:
        artifacts.invalid_certificates.append(index)
    _print_certificate(index, cases, summary)


def _record_row(
    artifacts: RunArtifacts,
    row: dict[str, Any],
    index: int,
    case: MatrixCase,
    cases: Sequence[MatrixCase],
    args: argparse.Namespace,
    *,
    label: str,
) -> None:
    artifacts.all_rows.append(row)
    with artifacts.attempts_jsonl.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(row, sort_keys=True) + "\n")
    write_csv(artifacts.attempts_csv, artifacts.all_rows)
    status = "PASS" if row["passed"] else "FAIL"
    print(
        f"CASE {index + 1}/{len(cases)} talt={case.target_alt_m} "
        f"wp={args.target_wp} s={case.speedup} gsu={case.navigation_speedup:g} "
        f"w={case.wind_speed_mps:g} d={case.wind_direction_deg} "
        f"{label} {status} snap={row.get('dist_3d_m', 'NA')} "
        f"coordinate={row.get('coordinate_dist_3d_m', 'NA')} "
        f"identity={row['identity_gate_passed']} error={row['error']}",
        flush=True,
    )


def _print_certificate(
    index: int,
    cases: Sequence[MatrixCase],
    summary: dict[str, Any],
) -> None:
    metrics = summary["metrics"]
    print(
        f"CERTIFICATE {index + 1}/{len(cases)} "
        f"{'VALID' if summary['valid'] else 'INVALID'} "
        f"runs={summary['runs']} passed={summary['passed_runs']} "
        f"snap={_stat_text(metrics['snap_dist_3d_m'])} "
        f"cpa={_stat_text(metrics['coordinate_dist_3d_m'])} "
        f"|snap-cpa|={_stat_text(metrics['snap_minus_coordinate_m'])} "
        f"rate={_stat_text(metrics['measured_clock_rate'])}",
        flush=True,
    )
    for reason in summary["consistency_errors"]:
        print(f"CERTIFICATE_INCONSISTENT {index + 1}: {reason}", flush=True)


def _stat_text(stats: dict[str, Any] | None) -> str:
    if not stats:
        return "NA"
    deviation = (
        "NA" if stats["sample_sd"] is None else f"{stats['sample_sd']:.3f}"
    )
    return (
        f"{stats['mean']:.3f}+/-{deviation}"
        f"[{stats['min']:.3f}..{stats['max']:.3f}]"
    )
