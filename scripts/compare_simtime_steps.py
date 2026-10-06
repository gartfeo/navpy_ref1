"""Compare measured windows without hiding baseline variation or time offsets."""

import argparse
import csv
import json
from itertools import combinations
from pathlib import Path
import re


HOST_COLUMNS = {"wait_us", "wall_us"}


def read_rows(path: Path) -> list[dict]:
    with path.open(encoding="utf-8") as source:
        next(source)  # boot identity differs intentionally between independent runs
        return list(csv.DictReader(source))


def compare(left: list[dict], right: list[dict]) -> dict:
    if not left or len(left) != len(right):
        raise ValueError("cannot compare empty or unequal windows")
    if left[0].keys() != right[0].keys():
        raise ValueError("different evidence columns")
    differences = {}
    for key in left[0].keys() - HOST_COLUMNS:
        changed = [(index, a[key], b[key])
                   for index, (a, b) in enumerate(zip(left, right), 1) if a[key] != b[key]]
        if changed:
            first, a, b = changed[0]
            differences[key] = {"count": len(changed), "first_step": first,
                                "left": a, "right": b}
    relative = {}
    for key in ("tick", "before_us", "after_us", "complete_us"):
        if key not in left[0]:
            continue
        a0, b0 = int(left[0][key]), int(right[0][key])
        changed = [index for index, (a, b) in enumerate(zip(left, right), 1)
                   if int(a[key]) - a0 != int(b[key]) - b0]
        relative[key] = {"offset": b0 - a0, "different_steps": len(changed),
                         "first_different_step": changed[0] if changed else None}
    return {"exact_equal": not differences, "differences": differences,
            "relative_timing": relative}


def summarize(root: Path) -> list[dict]:
    cases = []
    for path in sorted(root.glob("*/result.json")):
        result = json.loads(path.read_text())
        if result["fault"] == "none" and result["mode"] != "off" and result["passed"]:
            cases.append((path.parent, result))
    reports = []
    baselines = []
    for speed in sorted({result["speed"] for _, result in cases}):
        selected = [(path, result) for path, result in cases if result["speed"] == speed]
        if not any(result["mode"] == "observe" for _, result in selected):
            raise ValueError(f"speed {speed} has no observation baseline")
        rows = {path: read_rows(path / "navpy-step.csv") for path, _ in selected}
        baselines.append(next(path for path, result in selected if result["mode"] == "observe"))
        for (left, a), (right, b) in combinations(selected, 2):
            kind = "baseline-repeat" if a["mode"] == b["mode"] == "observe" else "condition-comparison"
            reports.append({"left": left.name, "right": right.name, "kind": kind,
                            **compare(rows[left], rows[right])})
    for left, right in zip(baselines, baselines[1:]):
        reports.append({"left": left.name, "right": right.name, "kind": "cross-speed-baseline",
                        **compare(read_rows(left / "navpy-step.csv"),
                                  read_rows(right / "navpy-step.csv"))})
    return reports


def revalidate(root: Path) -> list[dict]:
    # Lazy import keeps plain CSV comparison independent of the Windows launcher.
    from eval_simtime_step import validate_evidence
    records = []
    for path in sorted(root.glob("*/result.json")):
        result = json.loads(path.read_text())
        if result["fault"] != "none" or result["mode"] == "off":
            continue
        details = validate_evidence(path.parent, result["mode"], result["vehicle"])
        log = (path.parent / "supervisor.log").read_text(encoding="utf-8", errors="replace")
        attempts = re.findall(r"instances streaming telemetry.*?\(attempt (\d+)\)", log)
        if log.count("Starting sketch 'ArduPlane'") != 1 or attempts != ["1"]:
            raise ValueError(f"case did not verify on its first boot: {path.parent}")
        records.append({"case": path.parent.name, "launch_attempt": 1, **details})
    if not records:
        raise ValueError("no positive windows available for revalidation")
    (root / "review-revalidation.json").write_text(json.dumps(records, indent=2))
    return records


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--revalidate", action="store_true")
    args = parser.parse_args()
    if args.revalidate:
        print(json.dumps({"windows_revalidated": len(revalidate(args.directory))}))
    result = summarize(args.directory)
    output = args.directory / "comparison.json"
    output.write_text(json.dumps(result, indent=2))
    print(json.dumps({"comparisons": len(result),
                      "exact_equal": sum(row["exact_equal"] for row in result),
                      "report": str(output)}))
