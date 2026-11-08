"""The ten-seed scene check of step 14 (hitl_guidance.md section 9): every seed of a series builds its scene.

For each seed the scenario service draws the scene twice (the same hash both times) and prepares its scenario folder;
Unreal then loads the level with the approach runner's own arguments and the seed's -NavPyScenario file, is stopped
once it has logged its moved dock's terrain check, and scoring (scenario_scoring.verify_unreal) rebuilds the scene from
Unreal's readback. A seed passes when the hash is reproduced, the dock stands on the terrain, and the folder's ledger
shows deliveries to the allowed consumers only and no other file. No flight, SITL or navpy runs. From WSL, in the
repository root:

    python -s -m tools.hitl.scenario_scene_check --series SERIES.json --uproject /mnt/c/.../AasSimProject.uproject
        [--unreal-affinity MASK] [--out DIR]

The summary (scene_check.json in the check's folder, also printed) names hashes and verdicts, never a pose.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

from tools.hitl import scenario_public as sp
from tools.hitl import scenario_scoring as scoring
from tools.hitl import scenario_service as service
from tools.tuning import approach_chain as chain
from tools.tuning.approach_procs import ProcessGroup, RunnerError
from tools.tuning.private_data import private_dir

TERRAIN_WAIT_S = 120.0          # my value: Cesium sampled the terrain 14 s after the level loaded in step 13's run
POLL_S = 1.0
SUMMARY_FILE = "scene_check.json"


def _wait_terrain(log: Path, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
        if scoring.TERRAIN_LINE.search(text) or scoring.REFUSED_LINE.search(text):
            return True
        time.sleep(POLL_S)
    return False


def check_seed(series_path: Path, index: int, seed_dir: Path, paths: chain.ChainPaths, lens: Path) -> dict[str, Any]:
    """One seed: two draws, its scenario folder, Unreal with its scene, the verdict and the ledger's audit."""
    first, second = service.prepare(series_path, index, None), service.prepare(series_path, index, None)
    scenario_dir = seed_dir / "scenario"
    public = service.prepare(series_path, index, scenario_dir)
    group = ProcessGroup(seed_dir, {"windows": chain.spawn_unreal(paths.unreal_affinity)}, chain.port_free)
    error: Optional[str] = None
    try:
        group.start(chain.unreal(paths, lens, sp.consumer_file(scenario_dir, "unreal"), public["scene_hash"]))
        if not _wait_terrain(seed_dir / "unreal.log", TERRAIN_WAIT_S):
            error = f"no terrain check within {TERRAIN_WAIT_S:g} s"
    except RunnerError as exc:
        error = str(exc)
    finally:
        group.stop_all()
    verdict = scoring.verify_unreal(scenario_dir, seed_dir / "unreal.log")
    audit = scoring.audit(scenario_dir)
    redrawn = first["scene_hash"] == second["scene_hash"] == public["scene_hash"]
    clean = not any(audit[key] for key in ("outside_allowed", "refused", "missing", "repeated", "other_scene",
                                           "stray_files"))
    return {"seed_index": index, "scene_hash": public["scene_hash"], "redrawn_same": redrawn,
            "reproduced": verdict["reproduced"], "problems": verdict["problems"], "audit": audit,
            "unreal_error": error, "orphans": group.orphans(),
            "passed": redrawn and verdict["reproduced"] and clean and error is None and not group.orphans()}


def run_check(args: argparse.Namespace) -> dict[str, Any]:
    series_path = Path(args.series)
    series = service.read_series(series_path)
    check_id = "scene-check-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = Path(args.out or private_dir() / "runs" / "scenario") / check_id
    out.mkdir(parents=True, exist_ok=False)
    wsl_ip, windows_ip = chain_addresses()
    repo = Path.cwd()
    lens = repo / "src/navpy/core/perception/profile/vision_profiles.json"
    seeds = []
    for index in range(len(series["seeds"])):
        seed_dir = out / f"seed-{index:02d}"
        seed_dir.mkdir()
        paths = chain.ChainPaths(seed_dir, repo, sys.executable, Path("/unused"), Path("/unused"),
                                 Path(args.uproject), wsl_ip, windows_ip, unreal_affinity=args.unreal_affinity)
        seeds.append(check_seed(series_path, index, seed_dir, paths, lens))
        print(f"[scene-check] seed {index}: passed {seeds[-1]['passed']}", flush=True)
    deliveries: dict[str, int] = {}
    for seed in seeds:
        for consumer, count in seed["audit"]["deliveries"].items():
            deliveries[consumer] = deliveries.get(consumer, 0) + count
    summary = {"check_id": check_id, "series_id": series["series_id"],
               "series_sha256": service.jc.sha256_file(series_path), "seeds": seeds,
               "reproduced": sum(seed["reproduced"] for seed in seeds), "deliveries": deliveries,
               "outside_allowed": sorted({c for seed in seeds for c in seed["audit"]["outside_allowed"]}),
               "passed": all(seed["passed"] for seed in seeds)}
    (out / SUMMARY_FILE).write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def chain_addresses() -> tuple[str, str]:
    from tools.tuning.approach import _addresses        # WSL's own address and the Windows host's
    return _addresses()


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="scenario_scene_check", description=__doc__.split("\n\n")[0])
    parser.add_argument("--series", required=True, help="the private series file")
    parser.add_argument("--uproject", required=True, help="WSL path of the Unreal project (/mnt/c/...)")
    parser.add_argument("--unreal-affinity", type=int, help="Unreal's processor mask (the CPU share granted)")
    parser.add_argument("--out", help="folder for the check's folder (default: the private folder's runs/scenario)")
    summary = run_check(parser.parse_args(argv))
    print(json.dumps({key: summary[key] for key in ("check_id", "reproduced", "deliveries", "outside_allowed",
                                                    "passed")}))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
