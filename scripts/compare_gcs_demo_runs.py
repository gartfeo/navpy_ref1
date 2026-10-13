"""Compare saved GCS demo evidence without starting any vehicle or GCS process."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for directory in (ROOT, ROOT / "src"):
    sys.path.insert(0, str(directory))

from scripts.eval_gcs_demo_scenario import CERTIFIED_SCENARIO_PATH, load_scenario
from scripts.gcs_demo_comparison import compare_runs


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, required=True,
                        help="new report file outside all input directories")
    parser.add_argument("--scenario", type=Path, default=CERTIFIED_SCENARIO_PATH,
                        help="the runs' scenario file (default: the certified scenario)")
    args = parser.parse_args(argv)
    output = args.output.resolve()
    if any(output.is_relative_to(run.resolve()) for run in args.runs):
        parser.error("output must be outside the input run directories")
    if output.exists():
        parser.error("output already exists; preserve previous reports")
    try:
        report = compare_runs(args.runs, load_scenario(args.scenario))
    except (OSError, ValueError, TypeError) as error:
        parser.error(str(error))
    serialized = json.dumps(report, indent=2, allow_nan=False) + "\n"
    with output.open("x", encoding="utf-8") as handle:
        handle.write(serialized)
    print(f"Saved comparison: {output}")
    print(f"Artifacts byte-equal: {report['artifacts_byte_equal']}")
    for pair in report["comparisons"]:
        artifacts = pair["artifacts"]
        if artifacts["status"] == "incomplete":
            print(f"  run {pair['repeat_index']}: incomplete (missing or changing evidence)")
            continue
        print(f"  run {pair['repeat_index']}: {len(artifacts['differing_files'])} differing, "
              f"{len(artifacts['only_in_reference']) + len(artifacts['only_in_repeat'])} unmatched files")
    print(f"Recorded evidence equal: {report['recorded_evidence_equal']}")
    print("Full scenario repeatability: UNVERIFIED (see report limitations)")
    # A matching sampled record is deliberately not a successful qualification exit.
    return 3 if report["recorded_evidence_equal"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
