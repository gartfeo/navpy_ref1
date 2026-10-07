"""Recover full-rate diagnostics offline, accepting only an exact flight replay.

Run in the captured peer's Python environment. This never launches a simulator
or changes a law. The scoped logger replacement changes only diagnostic cadence.
The capture validator binds required approach parameters to the recorded BIN;
full-rate replay changes diagnostic cadence only.
"""

from __future__ import annotations

import argparse
from collections import Counter
import csv
from dataclasses import asdict
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "scripts")]
from scripts import simtime_navigation_runtime as runtime
from scripts.compare_simtime_navigation import validate
from scripts.eval_direct_pixel_command_causality import analyze
from scripts.simtime_navigation_protocol import (
    ATTITUDE, DRAIN_STEPS, GUIDED, TAKEOFF_ARM, Snapshot, StepCommand,
)
from navpy.logger.navigation_logger import NavigationLogger
from navpy.modules.common.models.location import Location


class FullRateLogger(NavigationLogger):
    MIN_LOG_INTERVAL = 0.0

    def log_event(self, *args: object, **kwargs: object) -> None:
        super().log_event(*args, **kwargs)
        # Offline playback has no live producer clock: fence each event instead
        # of outrunning the bounded live queue. Never loosen its capacity.
        self.drain()


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replay(peer: dict, output: Path) -> dict:
    """Core replay; complete-flight/identity validation belongs to run()."""
    output.mkdir(parents=True, exist_ok=False)
    records = peer["records"]
    wall = [0.0]
    captures, commands, held, inputs, fresh_times = [], 0, 0, [], []
    with patch.object(runtime, "NavigationLogger", FullRateLogger):
        engine = runtime.SynchronousNavigation(
            Location(*peer["dock"], is_absolute=True), speedup=peer["speedup"],
            camera_fraction=(4, 5) if peer["camera_hz"] == 40 else (1, 1),
            wall_now=lambda: wall[0], output=output,
            trim_throttle_percent=peer.get("throttle_policy", {}).get("trim_throttle_percent"),
            configured_throttle_percent=peer.get("throttle_policy", {}).get("configured_throttle_percent"))
        try:
            for index, record in enumerate(records):
                snapshot = Snapshot.decode(bytes.fromhex(record["snapshot"]))
                now = record["wall_s"] + record["delay_s"]
                if not math.isfinite(now) or now < wall[0]:
                    raise ValueError(f"nonmonotonic wall clock at step {index + 1}")
                wall[0] = now
                if index >= len(records) - DRAIN_STEPS:
                    command, evidence = StepCommand(), engine.drain()
                else:
                    command, evidence = engine.advance(snapshot, record["wall_s"])
                if index == 0:
                    command = StepCommand(TAKEOFF_ARM)
                if index == peer["guided_step"] - 1:
                    command = StepCommand(GUIDED)
                if command.reply(snapshot.identity).hex() != record["reply"]:
                    raise ValueError(f"reply mismatch at step {index + 1}")
                if evidence != record["evidence"]:
                    raise ValueError(f"evidence mismatch at step {index + 1}")
                if command.kind == ATTITUDE:
                    commands += 1
                    held += not evidence.get("captured", False)
                    if evidence.get("captured"):
                        fresh_times.append(snapshot.identity.source_us / 1_000_000)
                if evidence.get("captured"):
                    captures.append(snapshot.identity.source_us)
                    obs, truth = snapshot.observation, snapshot.truth
                    # Diagnostics after command formation, never fed back to the law.
                    inputs.append(dict(step=index + 1, source_us=snapshot.identity.source_us,
                        passed=evidence.get("passed", False), command_kind=command.kind,
                        pixels=evidence["pixels"],
                        roll_estimate_minus_truth_deg=math.degrees(obs.roll_rad) - truth.roll_deg,
                        pitch_estimate_minus_truth_deg=math.degrees(obs.pitch_rad) - truth.pitch_deg,
                        observed_airspeed_mps=obs.airspeed_mps,
                        observed_body_rates_rad_s=obs.rates_rad_s))
            if (engine.seed_step, engine.command_count) != (peer["seed_step"], peer["command_count"]):
                raise ValueError("seed or command count mismatch")
        finally:
            engine.close()
    return dict(records=len(records), commands=commands, noncapture_reissues=held,
                fresh_command_times=fresh_times,
                captures=len(captures), capture_intervals_us=dict(Counter(
                    b - a for a, b in zip(captures, captures[1:]))), inputs=inputs)


def verify_live_rows(case: Path, output: Path) -> dict:
    """Require every live diagnostic row; only CONFIG's cadence may differ."""
    counts = {}
    for name in ("navigation_debug.csv", "navigation_compact.csv"):
        live = (case / name).read_text().splitlines()
        full = (output / name).read_text().splitlines()
        if name == "navigation_debug.csv":
            config = [r for r in full if ",EVENT:CONFIG," in r]
            if len(config) != 1 or "min_log_interval_s=0.0;" not in config[0]:
                raise ValueError("full-rate logger configuration missing")
            live = [r for r in live if ",EVENT:CONFIG," not in r]
            full = [r for r in full if ",EVENT:CONFIG," not in r]
        missing = Counter(live) - Counter(full)
        if missing:
            first = next(iter(missing))
            raise ValueError(f"live row mismatch in {name} at source time {first.split(',')[0]}")
        if name == "navigation_debug.csv" and any(
                ",EVENT:FINAL_APPROACH_CMD," not in row and ",EVENT:FINAL_APPROACH_RESPONSE_STATE," not in row
                for row in Counter(full) - Counter(live)):
            raise ValueError("replay added an event unrelated to diagnostic cadence")
        counts[name] = dict(live=len(live), replay=len(full))
    return counts


def run(case: Path, lifecycle: Path, count: int, output: Path) -> dict:
    summary, _ = validate(case, lifecycle_count=count, lifecycle_dir=lifecycle)
    identity = summary["identity"]
    recorded = identity["runtime"]
    versions = {n: importlib.metadata.version(n) for n in recorded["packages"]}
    if sys.version != recorded["python"] or versions != recorded["packages"]:
        raise ValueError("replay must use the recorded Python/package environment")
    bound = {name: digest(case / name) for name in (
        "peer.json", "identity.json", "navpy-navigation.csv", "navigation_debug.csv", "navigation_compact.csv")}
    fleet_bound = (lifecycle / "fleet.json").is_file()
    if count == 3 or fleet_bound:
        manifest = json.loads((lifecycle / "fleet.json").read_text())
        for name, value in bound.items():
            if manifest["evidence_sha256"].get(f"{case.name}/{name}") != value:
                raise ValueError(f"unbound fleet evidence: {name}")
    result = replay(json.loads((case / "peer.json").read_text()), output)
    result["live_rows"] = verify_live_rows(case, output)
    result["causality"] = asdict(analyze(output / "navigation_debug.csv"))
    with (output / "navigation_debug.csv").open() as stream:
        states = [r for r in csv.reader(stream) if len(r) > 2 and r[1] == "EVENT:FINAL_APPROACH_RESPONSE_STATE"]
    if len(states) != result["commands"] - result["noncapture_reissues"]:
        raise ValueError("unexplained response-state count")
    times = [float(dict(token.split("=", 1) for token in row[2].split(";") if "=" in token)["obs_ts"])
             for row in states]
    if Counter(times) != Counter(result.pop("fresh_command_times")):
        raise ValueError("response-state steps differ from captured ATTITUDE steps")
    result.update(case=str(case), capture_identity=identity, evidence_sha256=bound,
                  lifecycle=str(lifecycle), instances=count, fleet_bound=fleet_bound,
                  method_sha256={name: digest(ROOT / "scripts" / name) for name in (
                      "replay_navigation_trace.py", "compare_simtime_navigation.py",
                      "eval_direct_pixel_command_causality.py")}, exact_replay=True,
                  parameter_attribution="required approach parameters verified by the capture validator",
                  limits="Input residuals include bias, dynamics and timing; they are not isolated noise. No spectral or causal-noise attribution.")
    (output / "report.json").write_text(json.dumps(result, indent=2))
    return {k: v for k, v in result.items() if k not in ("inputs", "capture_identity")}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", type=Path)
    parser.add_argument("--lifecycle", type=Path)
    parser.add_argument("--instances", type=int, choices=(1, 3), default=1)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.case, args.lifecycle or args.case, args.instances, args.output), indent=2))


if __name__ == "__main__":
    main()
