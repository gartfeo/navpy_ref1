"""Exact replay and descriptive whole-profile contrasts; never a noise spectrum."""

import argparse
from collections import Counter
import json
import math
from pathlib import Path, PureWindowsPath

import numpy as np
import pymap3d

from replay_navigation_trace import ROOT, digest, run as replay
from eval_direct_pixel_command_causality import _samples
from simtime_navigation_protocol import Snapshot


def local_path(value: str) -> Path:
    windows = PureWindowsPath(value)
    if windows.drive and Path("/mnt").is_dir():
        return Path("/mnt") / windows.drive[0].lower() / Path(*windows.parts[1:])
    return Path(value)


def require_matrix(report: dict) -> None:
    comparisons = report["comparisons"]
    profiles = ("stock", "noise-off", "noise-off-1000")
    if (report["control_steps"] != 36000 or len(comparisons) != 18 or
            any(not pair["equal"] for pair in comparisons) or
            Counter(pair["profile"] for pair in comparisons) != Counter({name: 6 for name in profiles}) or
            Counter(row["profile"] for row in report["runs"]) != Counter({name: 4 for name in profiles})):
        raise ValueError("analysis requires the complete accepted noise matrix")
    if report["validator_sha256"] != digest(ROOT / "scripts/compare_noise_matrix.py"):
        raise ValueError("matrix comparator changed since validation")
    if digest(local_path(report["ledger"])) != report["ledger_sha256"]:
        raise ValueError("matrix ledger changed since validation")
    for row in report["runs"]:
        if digest(local_path(row["directory"]) / "fleet.json") != row["manifest_sha256"]:
            raise ValueError("matrix manifest changed since validation")


def command_metrics(samples: list[dict]) -> dict:
    if len(samples) < 2:
        raise ValueError("insufficient fresh commands")
    duration = samples[-1]["obs_ts"] - samples[0]["obs_ts"]
    if duration <= 0:
        raise ValueError("nonpositive approach duration")
    result = dict(fresh_commands=len(samples), duration_s=duration,
                  law_branches=dict(Counter(sample["plan_reason"] for sample in samples)))
    for axis in ("roll", "pitch"):
        key = f"cmd_{axis}_deg"
        changes = [abs(right[key] - left[key]) for left, right in zip(samples, samples[1:])]
        clipped = sum(sample[f"raw_{axis}_deg"] != sample[key] for sample in samples)
        result[axis] = dict(max_step_deg=max(changes), p95_step_deg=float(np.percentile(changes, 95)),
            total_variation_deg=sum(changes), variation_per_s=sum(changes) / duration,
            clipped_commands=clipped, clipped_fraction=clipped / len(samples))
    index = max(range(1, len(samples)), key=lambda i:
                abs(samples[i]["cmd_roll_deg"] - samples[i-1]["cmd_roll_deg"]))
    left, right = samples[index-1], samples[index]
    result["largest_roll_step"] = dict(source_s=right["obs_ts"], dt_s=right["obs_ts"]-left["obs_ts"],
        before_deg=left["cmd_roll_deg"], after_deg=right["cmd_roll_deg"],
        before_branch=left["plan_reason"], after_branch=right["plan_reason"],
        visual_rate_change_deg_s=right["lateral_rate_deg_s"]-left["lateral_rate_deg_s"],
        own_turn_change_deg_s=right["aircraft_turn_rate_deg_s"]-left["aircraft_turn_rate_deg_s"])
    return result


def describe(case: Path, replay_directory: Path, summary: dict) -> dict:
    peer = json.loads((case / "peer.json").read_text())
    snapshots = [Snapshot.decode(bytes.fromhex(row["snapshot"])) for row in peer["records"]]
    seed = peer["seed_step"] - 1
    end = summary["first_pass_step"]
    if end is None:
        raise ValueError("profile did not visually pass; score separately")
    start_us = snapshots[seed].identity.source_us
    end_us = snapshots[end-1].identity.source_us
    samples = [row for row in _samples(replay_directory / "navigation_debug.csv")
               if start_us <= round(row["obs_ts"]*1e6) < end_us]
    metrics = command_metrics(samples)
    ranges = []
    for index in range(seed, end):
        truth = snapshots[index].truth
        ned = pymap3d.geodetic2ned(truth.latitude, truth.longitude, truth.altitude, *peer["dock"])
        ranges.append((float(np.linalg.norm(ned)), index))
    distance, index = min(ranges)
    metrics.update(seed_source_s=start_us/1e6, pass_source_s=end_us/1e6,
        guided_source_s=snapshots[peer["guided_step"]-1].identity.source_us/1e6,
        sampled_closest_approach_m=distance,
        closest_source_s=snapshots[index].identity.source_us/1e6,
        closest_truth_s=snapshots[index].truth_us/1e6,
        closest_at_window_boundary=index in (seed, end-1),
        closest_window="seed through first visual-pass snapshot inclusive",
        source_minus_truth_us=dict(Counter(snapshot.identity.source_us-snapshot.truth_us for snapshot in snapshots)),
        closest_since_seed_s=(snapshots[index].identity.source_us-start_us)/1e6)
    at_step = {snapshot.identity.source_us: snapshot for snapshot in snapshots}
    largest = metrics["largest_roll_step"]
    truth = at_step[round(largest["source_s"]*1e6)].truth
    largest["sampled_range_m"] = float(np.linalg.norm(pymap3d.geodetic2ned(
        truth.latitude, truth.longitude, truth.altitude, *peer["dock"])))
    largest["since_seed_s"] = largest["source_s"] - start_us/1e6
    raw = json.loads((replay_directory / "report.json").read_text())
    inputs = [row for row in raw["inputs"] if start_us <= row["source_us"] < end_us]
    metrics["estimate_minus_truth_rms_deg"] = {axis: math.sqrt(sum(
        row[f"{axis}_estimate_minus_truth_deg"]**2 for row in inputs) / len(inputs)) for axis in ("roll", "pitch")}
    metrics["rms_limit"] = "Residual combines bias, dynamics and timing, not isolated sensor noise."
    metrics.update(causality=raw["causality"], capture_intervals_us=raw["capture_intervals_us"],
                   noncapture_reissues=raw["noncapture_reissues"])
    return metrics


def run(matrix: Path, output: Path) -> dict:
    report = json.loads(matrix.read_text())
    require_matrix(report)
    output.mkdir(parents=True, exist_ok=False)
    profiles = {}
    for row in report["runs"]:
        name = row["profile"]
        if name in profiles:
            continue
        directory = local_path(row["directory"])
        case = directory / str(row["summary"]["vehicle"])
        destination = output / name
        replay(case, directory, 1, destination)
        metrics = describe(case, destination, row["summary"])
        profiles[name] = dict(case=str(case), metrics=metrics, evidence=row["evidence"],
            replay_sha256={file: digest(destination / file) for file in (
                "report.json", "navigation_debug.csv", "navigation_compact.csv")})
    contrasts = []
    for factor, left, right in (("noise mask at 1200 Hz", "stock", "noise-off"),
                                ("simulator rate with mask 8191", "noise-off", "noise-off-1000")):
        a, b = profiles[left]["metrics"], profiles[right]["metrics"]
        contrasts.append(dict(factor=factor, baseline=left, intervention=right,
            closest_approach_difference_m=b["sampled_closest_approach_m"]-a["sampled_closest_approach_m"],
            roll_variation_per_s_difference=b["roll"]["variation_per_s"]-a["roll"]["variation_per_s"],
            roll_max_step_difference_deg=b["roll"]["max_step_deg"]-a["roll"]["max_step_deg"]))
    result = dict(matrix_sha256=digest(matrix), method_sha256=digest(Path(__file__)),
        profiles=profiles, contrasts=contrasts,
        limits="One fixed realization per profile; whole-boot effects include estimator, cadence and trajectory changes. Variation is not isolated noise. Source-time phase windows differ. Closest approach uses sampled centimetre-grid truth; it is not physical docking or cargo receipt. No spectral attribution or filter recommendation.")
    (output / "analysis.json").write_text(json.dumps(result, indent=2))
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("matrix", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = run(args.matrix, args.output)
    print(json.dumps(dict(contrasts=result["contrasts"], profiles={name: value["metrics"]
                     for name, value in result["profiles"].items()}), indent=2))


if __name__ == "__main__":
    main()
