"""Offline SIM_CPA re-scoring for a preserved case directory.

Runs exactly the live post-teardown pipeline (parse, certify, common
episode, comparison) over an operator-named BIN, and rewrites ONLY the
verdict's ``sim_cpa`` block.  Stream scoring, validity and every other
field are untouched: a rescore can recover a transient artifact failure
or score a case that predates the integration, never re-judge a flight.

With ``--summary-root`` the sweep's summary.json is rebuilt through the
canonical summarizer after the verdict changes -- counters are never
patched individually, so the embedded rows and the counters cannot
diverge (R14).

Usage:
    python scripts/eval_sim_cpa_rescore.py <case_dir> --bin <path.BIN>
        [--summary-root <sweep_dir>]
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

WORKTREE = Path(__file__).resolve().parent.parent
SCRIPTS = Path(__file__).resolve().parent
sys.path[:0] = [str(WORKTREE / "src"), str(WORKTREE), str(SCRIPTS)]

from eval_direct_pixel_summary import summarize  # noqa: E402
from eval_direct_pixel_verdict import persist_verdict  # noqa: E402
from eval_sim_cpa_score import score_offline  # noqa: E402

_CASE_DIR_RE = re.compile(r"^speed-(?P<speed>[0-9.]+)-run-(?P<run>\d+)$")


def rescore_case(case_dir: Path, bin_path: Path) -> dict[str, object]:
    verdict = json.loads(
        (case_dir / "verdict.json").read_text(encoding="utf-8")
    )
    verdict["sim_cpa"] = score_offline(case_dir, bin_path, verdict=verdict)
    persist_verdict(case_dir, verdict)
    return verdict


def rebuild_summary(root: Path, case_dir: Path, verdict: dict) -> None:
    """Swap the case's row and re-run the canonical summarizer."""
    summary_path = root / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    match = _CASE_DIR_RE.match(case_dir.name)
    if match is None:
        raise ValueError(
            f"case directory name {case_dir.name!r} does not encode "
            "speedup/repetition; cannot locate its summary row"
        )
    speedup = float(match.group("speed"))
    repetition = int(match.group("run"))
    replaced = False
    results = summary.get("results", [])
    for index, row in enumerate(results):
        if (
            row.get("speedup") == speedup
            and row.get("repetition") == repetition
        ):
            updated = dict(verdict)
            updated.update({"speedup": speedup, "repetition": repetition})
            results[index] = updated
            replaced = True
            break
    if not replaced:
        raise ValueError(
            f"summary has no row for speedup={speedup:g} "
            f"repetition={repetition}"
        )
    rebuilt = summarize(
        results,
        gate_m=summary["gate_m"],
        goal_m=summary["goal_m"],
        source_identity=summary["source_identity"],
        aborted_identity=summary.get("aborted_source_identity"),
    )
    staging = summary_path.with_suffix(".json.tmp")
    staging.write_text(json.dumps(rebuilt, indent=2), encoding="utf-8")
    staging.replace(summary_path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case_dir", type=Path)
    parser.add_argument("--bin", type=Path, required=True)
    parser.add_argument("--summary-root", type=Path, default=None)
    args = parser.parse_args()
    if not (args.case_dir / "case.json").exists():
        print(f"not a case directory: {args.case_dir}", file=sys.stderr)
        return 2
    if not args.bin.exists():
        print(f"BIN not found: {args.bin}", file=sys.stderr)
        return 2
    verdict = rescore_case(args.case_dir, args.bin)
    block = verdict["sim_cpa"]
    print(json.dumps(
        {
            "configuration": block["configuration"]["status"],
            "artifact": block["artifact"]["status"],
            "evidence": block["evidence"]["status"],
            "comparison": block["comparison"]["status"],
            "disagreement": block["comparison"]["disagreement"],
            "module_score": block["evidence"]["score"],
            "deltas": block["comparison"]["deltas"],
        },
        indent=2,
    ))
    if args.summary_root is not None:
        rebuild_summary(args.summary_root, args.case_dir, verdict)
        print(f"summary rebuilt: {args.summary_root / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
