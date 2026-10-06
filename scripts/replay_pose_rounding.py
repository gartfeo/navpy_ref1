"""Offline frozen-trajectory controls; counterfactual commands never reach SITL."""

from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "scripts"), str(ROOT)]
from scripts import simtime_navigation_runtime as runtime
from replay_navigation_trace import FullRateLogger
from simtime_navigation_protocol import ATTITUDE, DRAIN_STEPS, GUIDED, TAKEOFF_ARM, Snapshot, StepCommand
from navpy.modules.common.models.location import Location
from eval_direct_pixel_command_causality import analyze, _samples


def replay_arm(peer: dict, pairs: list, output: Path, *, arm: str) -> dict:
    if arm not in ("legacy", "precast") or len(pairs) != len(peer["records"]):
        raise ValueError("invalid pose replay arm or count")
    output.mkdir(parents=True, exist_ok=False)
    wall = [0.]
    records, fresh_sources = [], []
    first_pass = None
    with patch.object(runtime, "NavigationLogger", FullRateLogger):
        engine = runtime.SynchronousNavigation(Location(*peer["dock"], is_absolute=True),
            speedup=peer["speedup"], camera_fraction=(4, 5), wall_now=lambda: wall[0], output=output,
            trim_throttle_percent=peer.get("throttle_policy", {}).get("trim_throttle_percent"),
            configured_throttle_percent=peer.get("throttle_policy", {}).get("configured_throttle_percent"))
        try:
            for index, (record, pair) in enumerate(zip(peer["records"], pairs)):
                original = Snapshot.decode(bytes.fromhex(record["snapshot"]))
                position = pair.rounded if arm == "legacy" else pair.precast
                snapshot = replace(original, truth=replace(original.truth,
                    latitude=position[0], longitude=position[1], altitude=position[2]))
                now = record["wall_s"] + record["delay_s"]
                if not math.isfinite(now) or now < wall[0]:
                    raise ValueError("nonmonotonic replay wall clock")
                wall[0] = now
                if index >= len(pairs)-DRAIN_STEPS:
                    command, evidence = StepCommand(), engine.drain()
                else:
                    command, evidence = engine.advance(snapshot, record["wall_s"])
                if index == 0:
                    command = StepCommand(TAKEOFF_ARM)
                if index == peer["guided_step"]-1:
                    command = StepCommand(GUIDED)
                reply = command.reply(snapshot.identity).hex()
                if arm == "legacy" and (reply != record["reply"] or evidence != record["evidence"]):
                    raise ValueError(f"legacy pose control differs at step {index+1}")
                if evidence.get("passed") and first_pass is None:
                    first_pass = index+1
                if command.kind == ATTITUDE and evidence.get("captured"):
                    fresh_sources.append(snapshot.identity.source_us)
                records.append(dict(step=index+1, source_us=snapshot.identity.source_us,
                    reply=reply, evidence=evidence))
            seed = engine.seed_step
        finally:
            engine.close()
    samples = _samples(output / "navigation_debug.csv")
    if [round(row["obs_ts"]*1e6) for row in samples] != fresh_sources:
        raise ValueError("pose replay diagnostics differ from fresh commands")
    result = dict(arm=arm, seed_step=seed, first_pass_step=first_pass,
        records=len(records), fresh_commands=len(fresh_sources),
        causality=asdict(analyze(output / "navigation_debug.csv")),
        exact_control=arm == "legacy", command_records=records,
        interpretation="Frozen recorded aircraft trajectory; replies are offline computations, not firmware acknowledgments.")
    (output / "report.json").write_text(json.dumps(result, indent=2))
    return result
