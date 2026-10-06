"""Five predeclared boots; require unchanged legacy histories and valid pose pairs."""

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path

from compare_lockstep_fleet import read_run
from compare_noise_matrix import verify_profile
from eval_lockstep_fleet import ROOT, digest, run
from pose_rounding_evidence import read_pairs
from simtime_step_launch import require_base_interpreter

BASELINE = ROOT / "docs/validation/navigation-pose-baseline-20260921.json"


def cells() -> list[dict]:
    return [dict(instances=1, noise_profile="noise-off-1000", pose_capture=capture,
                 speedup=speed, delayed=delay) for capture, speed, delay in (
        (False, 10., False), (True, 10., False), (True, 10., True),
        (True, 1., False), (True, 1., True))]


def canonical_hash(history: list[dict]) -> str:
    return hashlib.sha256(json.dumps(history, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode()).hexdigest()


def check_run(directory: Path, capture: bool, baseline: dict) -> dict:
    manifest, vehicles = read_run(directory, noise_profile="noise-off-1000", pose_capture=capture)
    if manifest["instances"] != 1 or baseline["steps"] != 3000:
        raise ValueError("wrong pose experiment size")
    vehicle = manifest["vehicles"][0]
    summary, history = vehicles[vehicle]
    profile = verify_profile(directory / str(vehicle), manifest, history)
    if canonical_hash(history) != baseline["normalized_history_sha256"]:
        raise ValueError("instrumented flight differs from frozen full legacy history")
    wrapper = (directory / str(vehicle) / "launch-probe.sh").read_text()
    expected_lines = (["export NAVPY_POSE_CAPTURE=1"] if capture else [])
    actual_lines = [line for line in wrapper.splitlines() if line.startswith("export NAVPY_POSE_CAPTURE")]
    if actual_lines != expected_lines or wrapper.splitlines().count("unset NAVPY_POSE_CAPTURE") != 1:
        raise ValueError("pose capture setting differs from wrapper")
    pose_hash = None
    branches = []
    if capture:
        case = directory / str(vehicle)
        rows, _ = read_pairs(case, json.loads((case / "peer.json").read_text()))
        normalized = [{key: value for key, value in row.items() if key not in ("boot0", "boot1")}
                      for row in rows]
        pose_hash = canonical_hash(normalized)
        branches = sorted({int(row["branch"]) for row in rows})
    return dict(directory=str(directory.resolve()), run_id=manifest["run_id"],
        boot=summary["boot"], vehicle=vehicle, pose_capture=capture,
        speedup=manifest["speedup"], delayed=manifest["delayed"],
        profile=profile, pose_hash=pose_hash, branches=branches,
        normalized_history_sha256=canonical_hash(history),
        manifest_sha256=digest(directory / "fleet.json"),
        identity=summary["identity"], steps=summary["rows"])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware-root", required=True)
    parser.add_argument("--peer-python", default="/home/gart/navpy-simtime-env/bin/python")
    args = parser.parse_args()
    require_base_interpreter()
    root = ROOT / ".sitl-runs" / f"pose-matrix-{datetime.now():%Y%m%d-%H%M%S}"
    root.mkdir()
    baseline = json.loads(BASELINE.read_text())
    ledger = dict(version=1, policy="one attempt per declared cell; stop on first rejection",
        baseline_sha256=digest(BASELINE), validator_sha256=digest(Path(__file__)),
        attempts=[dict(cell, directory=str(root / f"run-{i:02}"), status="pending")
                  for i, cell in enumerate(cells())])
    path = root / "attempts.json"
    def save() -> None:
        path.write_text(json.dumps(ledger, indent=2))
    save()
    print(f"LEDGER {path}", flush=True)
    checked, pose_hash = [], None
    for attempt in ledger["attempts"]:
        attempt["status"] = "running"
        save()
        try:
            options = argparse.Namespace(firmware_root=args.firmware_root, peer_python=args.peer_python,
                **{key: attempt[key] for key in cells()[0]})
            run(options, Path(attempt["directory"]))
            result = check_run(Path(attempt["directory"]), attempt["pose_capture"], baseline)
            if any(result["boot"] == row["boot"] or result["run_id"] == row["run_id"] for row in checked):
                raise ValueError("duplicate boot")
            if result["pose_hash"] is not None:
                if pose_hash is not None and result["pose_hash"] != pose_hash:
                    raise ValueError("pose evidence differs across independent boots")
                pose_hash = result["pose_hash"]
            # Defaults carry a per-case provenance comment. verify_profile
            # separately requires identical parameter values from the template.
            pinned = {key: value for key, value in result["identity"].items() if key != "defaults_sha256"}
            first = ({key: value for key, value in checked[0]["identity"].items() if key != "defaults_sha256"}
                     if checked else pinned)
            if pinned != first:
                raise ValueError("implementation/environment changed within pose matrix")
            checked.append(result)
            attempt.update(status="captured", run_id=result["run_id"])
        except BaseException as error:
            attempt.update(status="rejected", error=str(error))
            save()
            raise
        save()
        print(json.dumps({key: result[key] for key in ("directory", "steps", "pose_capture", "branches")}), flush=True)
    report = dict(version=1, passed=True, ledger=str(path.resolve()), ledger_sha256=digest(path),
        baseline_sha256=digest(BASELINE), method_sha256=digest(Path(__file__)),
        pairs_method_sha256=digest(ROOT / "scripts/pose_rounding_evidence.py"),
        steps=sum(row["steps"] for row in checked), runs=checked)
    (root / "report.json").write_text(json.dumps(report, indent=2))
    print(f"PASSED {root / 'report.json'}", flush=True)


if __name__ == "__main__":
    main()
