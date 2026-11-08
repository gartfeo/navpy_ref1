"""The ten-run series report (hitl_final_approach.md section 9, step 27 and "Every ten-run series").

    python -s -m tools.hitl.approach_report --series SERIES_ID [--run-root DIR] [--out report.json]

It reads every run folder of the series under the run root (default: the private folder's runs/approach), in run
order, and scores each with approach_run_checks.py. Every attempt is kept: a miss, an abort or missing evidence breaks
the series. The series passes, **10/10**, only when its first ten attempts all pass, fly the series' ten seeds once
each, and share one frozen configuration (the run record without its run identity and seed). Beside the score it
reports the substitutions (the records' declared scope), each run's timing skew (attitude and gimbal median, p95 and
max against the budget), the truth cross-check and the publisher's video numbers. A run refused before its folder
exists (for example by the orbit check) leaves no attempt and is reported by the runner itself.

Exit: 0 when the series passed, 1 otherwise.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any, Optional

from tools.hitl.approach_run_checks import score_run
from tools.hitl.timing_check import SKEW_BUDGET_S

SERIES_RUNS = 10                # plan: ten consecutive successful approaches
RUN_PREFIX = "approach-"
# The record's parts that name this run or its seed; everything else must be identical across a series.
RUN_FIELDS = ("run",)
SCENARIO_RUN_FIELDS = ("seed", "seed_index", "scene_hash")


def _json(path: Path) -> Any:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def series_runs(run_root: Path, series_id: str) -> list[Path]:
    """The series' run folders in run order (their ids are UTC stamps); a folder without a readable run file is
    still an attempt when its record names the series."""
    found = []
    for folder in sorted(p for p in Path(run_root).glob(RUN_PREFIX + "*") if p.is_dir()):
        for name in ("approach_run.json", "approach_record.json"):
            try:
                scenario = _json(folder / name).get("scenario") or {}
            except (OSError, ValueError, AttributeError):
                continue
            if scenario.get("series_id") == series_id:
                found.append(folder)
            break
    return found


def frozen_fingerprint(record: dict) -> str:
    """The record without its run identity and seed: one value per frozen configuration."""
    kept = {key: value for key, value in record.items() if key not in RUN_FIELDS}
    kept["scenario"] = {key: value for key, value in (record.get("scenario") or {}).items()
                        if key not in SCENARIO_RUN_FIELDS}
    return hashlib.sha256(json.dumps(kept, sort_keys=True).encode("utf-8")).hexdigest()


def _record(run_dir: Path) -> Optional[dict]:
    try:
        return _json(run_dir / "approach_record.json")
    except (OSError, ValueError):
        return None


def _skew_summary(attempts: list[dict]) -> dict:
    out = {}
    for name in ("attitude", "gimbal"):
        values = [a["timing"][name]["max_abs_s"] for a in attempts
                  if isinstance(((a.get("timing") or {}).get(name) or {}).get("max_abs_s"), (int, float))]
        out[name] = {"max_abs_s": max(values, default=None), "runs": len(values), "budget_s": SKEW_BUDGET_S}
    return out


def report(run_root: Path, series_id: str) -> dict:
    folders = series_runs(run_root, series_id)
    attempts = [score_run(folder) for folder in folders]
    scored = attempts[:SERIES_RUNS]
    problems = []
    consecutive = next((i for i, a in enumerate(scored) if not a["passed"]), len(scored))
    if consecutive < len(scored):
        problems.append(f"attempt {consecutive + 1} ({scored[consecutive].get('run_id')}) failed: the series broke")
    if len(scored) < SERIES_RUNS:
        problems.append(f"{len(scored)} of {SERIES_RUNS} attempts")
    seeds = [a.get("seed_index") for a in scored]
    if sorted(s for s in seeds if isinstance(s, int)) != list(range(len(scored))) or len(set(seeds)) != len(seeds):
        problems.append(f"seeds {seeds}: each of the series' seeds once, in its first attempts")
    records = [_record(Path(a["run_dir"])) for a in scored]
    prints = {frozen_fingerprint(r) for r in records if r is not None}
    if None in records:
        problems.append("a run record is unreadable")
    if len(prints) > 1:
        problems.append(f"{len(prints)} configurations: a changed configuration starts a new series")
    substitutions = sorted({json.dumps(r.get("substitutions"), sort_keys=True) for r in records if r is not None})
    return {"series_id": series_id, "run_root": str(run_root), "score": f"{consecutive}/{SERIES_RUNS}",
            "passed": not problems, "problems": problems,
            "substitutions": [json.loads(s) for s in substitutions], "timing_skew": _skew_summary(scored),
            "attempts": attempts, "after_series": len(attempts) - len(scored)}


def summary_lines(result: dict) -> list[str]:
    lines = [f"series {result['series_id']}: {result['score']} {'PASS' if result['passed'] else 'FAIL'}"]
    lines += [f"  problem: {p}" for p in result["problems"]]
    for subs in result["substitutions"]:
        lines += [f"  substitution {key}: {value}" for key, value in sorted((subs or {}).items())]
    for name, skew in result["timing_skew"].items():
        lines.append(f"  timing skew {name}: max |skew| {skew['max_abs_s']} s over {skew['runs']} runs "
                     f"(budget {skew['budget_s']} s)")
    for attempt in result["attempts"]:
        failed = {name: found for name, found in attempt["checks"].items() if found}
        lines.append(f"  {attempt.get('run_id')} seed {attempt.get('seed_index')}: "
                     + ("pass" if attempt["passed"] else f"FAIL {failed}"))
    return lines


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m tools.hitl.approach_report", description=__doc__.split("\n")[0])
    parser.add_argument("--series", required=True, help="the series id (the scenario service's series file)")
    parser.add_argument("--run-root", help="the runs' folder (default: the private folder's runs/approach)")
    parser.add_argument("--out", help="where the JSON report goes")
    args = parser.parse_args(argv)
    if args.run_root:
        root = Path(args.run_root)
    else:
        from tools.tuning.private_data import private_dir
        root = private_dir() / "runs" / "approach"
    result = report(root, args.series)
    if args.out:
        Path(args.out).write_text(json.dumps(result, indent=2, default=str) + "\n", encoding="utf-8")
    print("\n".join(summary_lines(result)), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())


__all__ = ["SERIES_RUNS", "frozen_fingerprint", "main", "report", "series_runs", "summary_lines"]
