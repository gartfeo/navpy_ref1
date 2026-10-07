"""Predeclared full-precision camera flights with a frozen rounded control."""

import argparse
from datetime import datetime
import json
from pathlib import Path

from compare_lockstep_fleet import read_run
from compare_noise_matrix import verify_profile
from eval_lockstep_fleet import ROOT, digest, run
from pose_precision_evidence import bits, read_precision
from run_pose_matrix import canonical_hash, contract_file
from simtime_step_launch import require_base_interpreter


def cells() -> list[dict]:
    return [dict(instances=1, noise_profile="noise-off-1000", pose_capture=True,
                 pose_source=source, speedup=speed, delayed=delay)
            for source, speed, delay in (("rounded", 10., False),
                ("precast", 10., False), ("precast", 10., True),
                ("precast", 1., False), ("precast", 1., True))]


def check_run(directory: Path, source: str, baseline: dict, firmware: dict) -> dict:
    manifest, vehicles = read_run(directory, noise_profile="noise-off-1000",
                                   pose_capture=True, pose_source=source)
    if manifest["instances"] != 1 or baseline["steps"] != 3000:
        raise ValueError("wrong precision experiment size")
    vehicle = manifest["vehicles"][0]
    summary, history = vehicles[vehicle]
    profile = verify_profile(directory / str(vehicle), manifest, history)
    history_hash = canonical_hash(history)
    if source == "rounded" and history_hash != baseline["normalized_history_sha256"]:
        raise ValueError("rounded flight differs from frozen legacy history")
    if (summary["identity"]["binary_sha256"] != firmware["binary_sha256"] or
            summary["identity"]["firmware_head"] != firmware["base_commit"]):
        raise ValueError("unreviewed precision firmware")
    case = directory / str(vehicle)
    lines = (case / "launch-probe.sh").read_text().splitlines()
    if (lines.count("unset NAVPY_RENDER_POSE") != 1 or lines.count("unset NAVPY_POSE_CAPTURE") != 1 or
            [s for s in lines if s.startswith("export NAVPY_RENDER_POSE")] != [f"export NAVPY_RENDER_POSE={source}"] or
            [s for s in lines if s.startswith("export NAVPY_POSE_CAPTURE")] != ["export NAVPY_POSE_CAPTURE=1"]):
        raise ValueError("precision wrapper source mismatch")
    rows, pairs = read_precision(case, json.loads((case / "peer.json").read_text()), source)
    normalized = [{key: value for key, value in row.items() if key not in ("boot0", "boot1")}
                  for row in rows]
    return dict(directory=str(directory.resolve()), run_id=manifest["run_id"],
        boot=summary["boot"], vehicle=vehicle, pose_source=source,
        speedup=manifest["speedup"], delayed=manifest["delayed"], profile=profile,
        pose_hash=canonical_hash(normalized), branches=sorted({int(r["branch"]) for r in rows}),
        differing_rows=sum(bits(p.rounded) != bits(p.precast) for p in pairs),
        normalized_history_sha256=history_hash, manifest_sha256=digest(directory / "fleet.json"),
        identity=summary["identity"], steps=summary["rows"])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware-root", required=True)
    parser.add_argument("--baseline", required=True, type=contract_file,
                        help="frozen legacy-history baseline JSON (navigation-pose-baseline)")
    parser.add_argument("--firmware", required=True, type=contract_file,
                        help="reviewed precision firmware contract JSON (navigation-pose-precision-firmware)")
    parser.add_argument("--peer-python", default="/home/gart/navpy-simtime-env/bin/python")
    args = parser.parse_args()
    require_base_interpreter()
    root = ROOT / ".sitl-runs" / f"pose-precision-{datetime.now():%Y%m%d-%H%M%S}"
    root.mkdir()
    baseline, firmware = json.loads(args.baseline.read_text()), json.loads(args.firmware.read_text())
    ledger = dict(version=2, policy="one attempt per declared cell; stop on first rejection",
        baseline_sha256=digest(args.baseline), firmware_sha256=digest(args.firmware),
        validator_sha256=digest(Path(__file__)),
        attempts=[dict(cell, directory=str(root / f"run-{i:02}"), status="pending")
                  for i, cell in enumerate(cells())])
    path = root / "attempts.json"
    def save() -> None:
        path.write_text(json.dumps(ledger, indent=2))
    save()
    print(f"LEDGER {path}", flush=True)
    checked = []
    for attempt in ledger["attempts"]:
        attempt["status"] = "running"
        save()
        try:
            options = argparse.Namespace(firmware_root=args.firmware_root, peer_python=args.peer_python,
                **{key: attempt[key] for key in cells()[0]})
            run(options, Path(attempt["directory"]))
            result = check_run(Path(attempt["directory"]), attempt["pose_source"], baseline, firmware)
            if any(result["boot"] == r["boot"] or result["run_id"] == r["run_id"] for r in checked):
                raise ValueError("duplicate precision boot")
            if len(checked) > 1 and any(result[k] != checked[1][k] for k in
                    ("pose_hash", "normalized_history_sha256")):
                raise ValueError("corrected flight/pose differs across independent boots")
            # Per-case defaults comments differ; actual PARM values are checked.
            pinned = {k: v for k, v in result["identity"].items() if k != "defaults_sha256"}
            first = ({k: v for k, v in checked[0]["identity"].items() if k != "defaults_sha256"}
                     if checked else pinned)
            if pinned != first:
                raise ValueError("implementation/environment changed during precision matrix")
            checked.append(result)
            attempt.update(status="captured", run_id=result["run_id"])
        except BaseException as error:
            attempt.update(status="rejected", error=str(error))
            save()
            raise
        save()
        print(json.dumps({k: result[k] for k in ("directory", "steps", "pose_source", "differing_rows")}), flush=True)
    report = dict(version=2, passed=True, ledger=str(path.resolve()), ledger_sha256=digest(path),
        baseline_sha256=digest(args.baseline), firmware_sha256=digest(args.firmware),
        method_sha256=digest(Path(__file__)),
        pairs_method_sha256=digest(ROOT / "scripts/pose_precision_evidence.py"),
        steps=sum(r["steps"] for r in checked), runs=checked,
        corrected_history_sha256=checked[1]["normalized_history_sha256"],
        scope="passed covers determinism and evidence validity only; correction acceptance requires pose_precision_analysis")
    (root / "report.json").write_text(json.dumps(report, indent=2))
    print(f"PASSED {root / 'report.json'}", flush=True)


if __name__ == "__main__":
    main()
