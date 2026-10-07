"""Verify closed-loop precision results against the predeclared acceptance rules."""

import argparse
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "scripts"), str(ROOT)]
import pymap3d

from analyze_noise_profiles import local_path, command_metrics
from eval_direct_pixel_command_causality import _samples
from pose_precision_evidence import read_precision
from replay_navigation_trace import digest, run as replay_recorded
from run_pose_matrix import contract_file
from run_pose_precision_matrix import cells, check_run
from simtime_navigation_protocol import Snapshot


def require_matrix(report: dict, baseline_path: Path, firmware_path: Path) -> None:
    if report.get("passed") is not True or report.get("version") != 2 or report.get("steps") != 15000:
        raise ValueError("precision analysis requires a complete five-boot matrix")
    for key, path in (("method_sha256", ROOT / "scripts/run_pose_precision_matrix.py"),
            ("pairs_method_sha256", ROOT / "scripts/pose_precision_evidence.py"),
            ("baseline_sha256", baseline_path), ("firmware_sha256", firmware_path)):
        if report[key] != digest(path):
            raise ValueError("precision method/contract changed")
    ledger_path = local_path(report["ledger"])
    if digest(ledger_path) != report["ledger_sha256"]:
        raise ValueError("precision ledger changed")
    ledger = json.loads(ledger_path.read_text())
    attempts, runs, expected = ledger["attempts"], report["runs"], cells()
    if len(attempts) != 5 or len(runs) != 5:
        raise ValueError("missing precision matrix cells")
    baseline, firmware = json.loads(baseline_path.read_text()), json.loads(firmware_path.read_text())
    for attempt, row, cell in zip(attempts, runs, expected):
        if (attempt["status"] != "captured" or any(attempt[k] != v for k, v in cell.items()) or
                any(row[k] != cell[k] for k in ("pose_source", "speedup", "delayed")) or
                attempt["run_id"] != row["run_id"] or local_path(attempt["directory"]).resolve() !=
                local_path(row["directory"]).resolve()):
            raise ValueError("precision cells differ from predeclared ledger")
        actual = check_run(local_path(row["directory"]), row["pose_source"], baseline, firmware)
        actual = json.loads(json.dumps(actual, allow_nan=False))  # JSON encodes cadence-map keys as strings
        if any(actual[k] != v for k, v in row.items() if k != "directory"):
            raise ValueError("precision matrix result differs from verified evidence")
    if len({r["run_id"] for r in runs}) != 5 or len({tuple(r["boot"]) for r in runs}) != 5:
        raise ValueError("duplicate precision boot")
    if (len({r["pose_hash"] for r in runs[1:]}) != 1 or
            {r["normalized_history_sha256"] for r in runs[1:]} != {report["corrected_history_sha256"]}):
        raise ValueError("corrected histories/poses differ")
    pinned = [{k: v for k, v in r["identity"].items() if k != "defaults_sha256"} for r in runs]
    if any(identity != pinned[0] for identity in pinned[1:]):
        raise ValueError("precision matrix mixed implementations")


def phase(peer: dict) -> tuple[list[Snapshot], int, int]:
    snapshots = [Snapshot.decode(bytes.fromhex(r["snapshot"])) for r in peer["records"]]
    first_pass = next((i for i, r in enumerate(peer["records"], 1) if r["evidence"].get("passed")), None)
    seed = peer["seed_step"]
    if first_pass is None or not 1 <= seed < first_pass <= len(snapshots):
        raise ValueError("precision flight lacks a confirmed seed and visual pass")
    return snapshots, seed, first_pass


def closest_approach(pairs: list, dock: list[float], seed: int, first_pass: int) -> dict:
    values = [(tuple(float(x) for x in pymap3d.geodetic2ned(*p.precast, *dock)), step)
              for step, p in enumerate(pairs, 1) if seed <= step <= first_pass]
    sampled, sample_step = min((math.sqrt(sum(x*x for x in p)), step) for p, step in values)
    segments = []
    for (a, step), (b, _) in zip(values, values[1:]):
        delta = tuple(y-x for x,y in zip(a,b))
        squared = sum(d*d for d in delta)
        fraction = max(0., min(1., -sum(x*d for x,d in zip(a,delta))/squared)) if squared else 0.
        distance = math.sqrt(sum((x+fraction*d)**2 for x,d in zip(a,delta)))
        segments.append((distance, step, fraction))
    distance, step, fraction = min(segments)
    if not math.isfinite(distance):
        raise ValueError("nonfinite approach scoring")
    return dict(distance_m=distance, segment_start_step=step, segment_fraction=fraction,
        sampled_distance_m=sampled, sampled_step=sample_step, samples=len(values),
        source="piecewise-linear precast smoothed camera pose in both flights, seed through first visual pass inclusive; interpolation is a scoring estimate")


def acceptance(arms: dict) -> dict:
    rounded, precast = arms["rounded"], arms["precast"]
    variation = [arm["common_window"]["roll"]["variation_per_s"] for arm in (rounded, precast)]
    distances = [arm["closest_approach"]["distance_m"] for arm in (rounded, precast)]
    if not all(math.isfinite(v) for v in variation + distances):
        raise ValueError("nonfinite acceptance metric")
    lower = variation[1] < variation[0]
    approach = distances[1] <= distances[0]
    return dict(accepted=lower and approach, roll_variation_lower=lower,
                closest_approach_not_regressed=approach)


def run(matrix: Path, output: Path, baseline: Path, firmware: Path) -> dict:
    report = json.loads(matrix.read_text())
    require_matrix(report, baseline, firmware)
    output.mkdir(parents=True, exist_ok=False)
    flights = {}
    for row in report["runs"][:2]:
        name = row["pose_source"]
        directory = local_path(row["directory"])
        case = directory / str(row["vehicle"])
        peer = json.loads((case / "peer.json").read_text())
        _, pairs = read_precision(case, peer, name)
        snapshots, seed, first_pass = phase(peer)
        replay = replay_recorded(case, directory, 1, output / name)
        flights[name] = dict(peer=peer, pairs=pairs, snapshots=snapshots, seed=seed,
            first_pass=first_pass, replay=replay, case=case)
    if flights["rounded"]["peer"]["dock"] != flights["precast"]["peer"]["dock"]:
        raise ValueError("different delivery docks")
    start = max(f["snapshots"][f["seed"]-1].identity.source_us for f in flights.values())
    end = min(f["snapshots"][f["first_pass"]-1].identity.source_us for f in flights.values())
    if start >= end:
        raise ValueError("no common pre-pass window")
    arms = {}
    for name, flight in flights.items():
        samples = _samples(output / name / "navigation_debug.csv")
        own_start = flight["snapshots"][flight["seed"]-1].identity.source_us
        own_end = flight["snapshots"][flight["first_pass"]-1].identity.source_us
        common = [r for r in samples if start <= round(r["obs_ts"]*1e6) < end]
        own = [r for r in samples if own_start <= round(r["obs_ts"]*1e6) < own_end]
        arms[name] = dict(common_window=command_metrics(common), own_window=command_metrics(own),
            seed_step=flight["seed"], first_pass_step=flight["first_pass"],
            pass_source_us=own_end, phase_duration_s=(own_end-own_start)/1e6,
            closest_approach=closest_approach(flight["pairs"],flight["peer"]["dock"],flight["seed"],flight["first_pass"]),
            exact_replay=flight["replay"]["exact_replay"], causality=flight["replay"]["causality"],
            evidence_sha256={n:digest(flight["case"] / n) for n in ("peer.json", "navpy-pose.csv")})
    result = dict(version=2, matrix_sha256=digest(matrix), common_window_us=[start,end], arms=arms,
        method_sha256={n:digest(ROOT / "scripts" / n) for n in ("pose_precision_analysis.py",
            "pose_precision_evidence.py", "run_pose_precision_matrix.py", "replay_navigation_trace.py",
            "compare_lockstep_fleet.py", "compare_noise_matrix.py")},
        acceptance=acceptance(arms),
        limits="One vehicle and scenario. Repeatability across speed/delivery timing, not robustness. Camera-pose closest approach and visual pass do not establish docking or cargo receipt. The law, estimator and sensor noise profile were held fixed.")
    (output / "analysis.json").write_text(json.dumps(result, indent=2))
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("matrix", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--baseline", required=True, type=contract_file,
                        help="frozen legacy-history baseline JSON the matrix report pins")
    parser.add_argument("--firmware", required=True, type=contract_file,
                        help="reviewed precision firmware contract JSON the matrix report pins")
    args = parser.parse_args()
    result = run(args.matrix, args.output, args.baseline, args.firmware)
    print(json.dumps(dict(acceptance=result["acceptance"], arms=result["arms"]), indent=2))
    if not result["acceptance"]["accepted"]:
        raise SystemExit("Predeclared precision acceptance failed; investigate without retuning")


if __name__ == "__main__":
    main()
