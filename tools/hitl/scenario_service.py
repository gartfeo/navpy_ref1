"""The scenario service: one seed's private dock pose, delivered only to its allowed consumers (hitl_guidance.md
section 9, stage 1 step 14, and the hard rule that simulator truth belongs to Unreal, contact and scoring).

A series file, in the private folder, freezes a ten-run series: its seeds, the dock region, the dock's design size
(step 23b), the consumers, the search lane and the limits. For one seed the service draws the scene, the dock's offset from the level's own dock and its yaw
added to the level's, as whole millimetres and millidegrees, and hashes it (scene_hash). ``prepare`` writes the run's
scenario folder:

- scenario.json, the public half (scenario_public.py): identity, lane, limits and dock size, no pose and no seed;
- private/<consumer>.json, one file per consumer the series names (only unreal, contact and scoring are allowed);
- deliveries.jsonl, a ledger line per delivered file.

The runner runs it as its own process and reads only scenario.json; Unreal reads private/unreal.json
(-NavPyScenario=, NavPyDockBoardSubsystem), moves its dock, sets it on the terrain and writes what it built to
private/unreal_applied.json; scoring (scenario_scoring.py) checks that against its own file.

    python -s -m tools.hitl.scenario_service new-series --series-id ID [--out FILE]
    python -s -m tools.hitl.scenario_service prepare --series FILE --index N --out DIR [--dry-run]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
import secrets
from collections.abc import Mapping
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

from tools.hitl import json_checks as jc
from tools.hitl import scenario_public as sp
from tools.hitl.json_checks import RecordError
from tools.tuning import approach_lane
from tools.tuning.flight_floor import MIN_HEIGHT_M
from tools.tuning.private_data import private_dir

SERIES_SCHEMA_VERSION = 2       # 2: the dock's design size (step 23b)
SCENE_SCHEMA_VERSION = 1
UNREAL_SCENE_SCHEMA_VERSION = 1
SERIES_SEEDS = 10                       # a ten-run series (section 9 "Every ten-run series")
DEFAULT_LEVEL = "/Game/AAS/Levels/Yeghvard_Default"     # approach_chain.UNREAL_LEVEL
# My value (owner, 2026-11-04: offset and yaw; Unreal sets the moved dock on the terrain): the Yeghvard airfield
# around the level's dock, where the step 13 lane passes.
DEFAULT_MAX_OFFSET_M = 30.0
MAX_OFFSET_LIMIT_M = 200.0              # a larger region is a wrong series file, not a scenario
DEFAULT_OBSERVE_S = 660.0               # step 13's run length on the lane
# Step 23b, phase A: a pickup truck on the ground (hitl_dock_detector.md section 3), measured in Unreal: a 2020
# Silverado 1500 crew cab, its width with mirrors. A series freezes the value it was made with.
DEFAULT_DOCK = {"kind": "pickup", "length_m": 5.887, "width_m": 2.490, "height_m": 1.958, "placement": "ground"}
SERIES_SPEC: Mapping[str, jc.Check] = {
    "schema_version": jc.equals(SERIES_SCHEMA_VERSION),
    "series_id": jc.folder_name,
    "created_utc": jc.utc,
    "level": jc.text,
    "seeds": jc.list_of(jc.index, True),
    "dock_region": jc.obj({"max_offset_m": jc.positive}),
    "consumers": sp.consumers_check,
    "lane": jc.obj(sp.LANE_SPEC),
    "limits": jc.obj(sp.LIMITS_SPEC),
    "dock_size": jc.obj(sp.DOCK_SPEC),
}


class ScenarioError(RuntimeError):
    """A delivery the service refuses: a consumer outside the three, or a scenario folder that already exists."""


def _series_problems(series: Mapping[str, Any]) -> list[str]:
    problems = []
    if len(series["seeds"]) != SERIES_SEEDS:
        problems.append(f"series.seeds has {len(series['seeds'])} seeds, not {SERIES_SEEDS}")
    if len(set(series["seeds"])) != len(series["seeds"]):
        problems.append("series.seeds repeats a seed")
    if series["dock_region"]["max_offset_m"] > MAX_OFFSET_LIMIT_M:
        problems.append(f"series.dock_region.max_offset_m is above {MAX_OFFSET_LIMIT_M:g}")
    return problems


def validate_series(data: Any) -> None:
    jc.require(data, SERIES_SPEC, "series", _series_problems)


def read_series(path: Path) -> dict[str, Any]:
    data = jc.read_json(Path(path))
    validate_series(data)
    return data


def new_series(series_id: str, created: datetime, max_offset_m: float = DEFAULT_MAX_OFFSET_M,
               level: str = DEFAULT_LEVEL) -> dict[str, Any]:
    """A series of SERIES_SEEDS fresh seeds, the step 13 lane and limits, Unreal and scoring as consumers."""
    seeds: list[int] = []
    while len(seeds) < SERIES_SEEDS:
        seed = secrets.randbits(63)
        if seed not in seeds:
            seeds.append(seed)
    series = {
        "schema_version": SERIES_SCHEMA_VERSION, "series_id": series_id,
        "created_utc": created.astimezone(timezone.utc).strftime(jc.UTC_FORMAT), "level": level, "seeds": seeds,
        "dock_region": {"max_offset_m": max_offset_m}, "consumers": list(sp.REQUIRED_CONSUMERS),
        "lane": {"bearing_deg": approach_lane.LANE_BEARING_DEG, "start_m": approach_lane.LANE_START_M,
                 "length_m": approach_lane.LANE_LENGTH_M, "height_m": approach_lane.LANE_HEIGHT_M,
                 "repeats": approach_lane.LANE_REPEATS},
        "limits": {"observe_s": DEFAULT_OBSERVE_S, "min_height_m": MIN_HEIGHT_M},
        "dock_size": dict(DEFAULT_DOCK),
    }
    validate_series(series)
    return series


def write_series(series: Mapping[str, Any], path: Path, private_root: Optional[Path] = None) -> Path:
    """A series is private: written once, only inside the private folder."""
    validate_series(series)
    path, root = Path(path).resolve(), Path(private_dir() if private_root is None else private_root).resolve()
    if root not in path.parents:
        raise RecordError([f"series file {path} is not in the private folder {root}; seeds are private data"])
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with path.open("x", encoding="utf-8") as handle:
            handle.write(json.dumps(series, indent=2, sort_keys=True) + "\n")
    except FileExistsError:
        raise RecordError([f"{path} exists: a series is frozen once written"]) from None
    return path


def canonical(data: Any) -> bytes:
    return json.dumps(data, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")


def scene_hash(scene: Mapping[str, Any]) -> str:
    return hashlib.sha256(canonical(scene)).hexdigest()


def draw_dock(seed: int, max_offset_m: float) -> dict[str, int]:
    """The seed's dock: an offset uniform over the disc of max_offset_m about the level's dock, and a yaw."""
    rng = random.Random(seed)
    radius_m, bearing = max_offset_m * math.sqrt(rng.random()), 2.0 * math.pi * rng.random()
    return {"east_mm": round(1000.0 * radius_m * math.sin(bearing)),
            "north_mm": round(1000.0 * radius_m * math.cos(bearing)), "yaw_mdeg": rng.randrange(360_000)}


def build_scene(series: Mapping[str, Any], index: int) -> dict[str, Any]:
    if not 0 <= index < len(series["seeds"]):
        raise RecordError([f"seed index {index} is outside the series' {len(series['seeds'])} seeds"])
    seed = series["seeds"][index]
    return {"schema_version": SCENE_SCHEMA_VERSION, "series_id": series["series_id"], "seed_index": index,
            "seed": seed, "level": series["level"],
            "dock": draw_dock(seed, series["dock_region"]["max_offset_m"])}


def public_half(series: Mapping[str, Any], series_sha256: str, scene: Mapping[str, Any]) -> dict[str, Any]:
    public = {"schema_version": sp.SCHEMA_VERSION, "series_id": series["series_id"], "series_sha256": series_sha256,
              "seed_index": scene["seed_index"], "scene_hash": scene_hash(scene),
              "consumers": list(series["consumers"]), "lane": dict(series["lane"]),
              "limits": dict(series["limits"]), "dock_size": dict(series["dock_size"])}
    sp.validate_public(public)
    return public


def consumer_payload(consumer: str, scene: Mapping[str, Any]) -> dict[str, Any]:
    """What each consumer gets: Unreal its dock move; scoring and contact the whole scene."""
    digest = scene_hash(scene)
    if consumer == "unreal":
        dock = scene["dock"]
        return {"schema_version": UNREAL_SCENE_SCHEMA_VERSION, "scene_hash": digest,
                "dock_offset_east_mm": dock["east_mm"], "dock_offset_north_mm": dock["north_mm"],
                "dock_yaw_offset_mdeg": dock["yaw_mdeg"]}
    return {"scene_hash": digest, "scene": json.loads(canonical(scene))}


def _utc() -> str:
    return datetime.now(timezone.utc).isoformat()


def deliver(scenario_dir: Path, consumer: str, scene: Mapping[str, Any]) -> Path:
    """Write one consumer's private file and its ledger line; a consumer outside the three is refused and logged."""
    ledger = Path(scenario_dir) / sp.LEDGER_FILE
    digest = scene_hash(scene)
    if consumer not in sp.ALLOWED_CONSUMERS:
        _append(ledger, {"consumer": consumer, "scene_hash": digest, "refused": True, "utc": _utc()})
        raise ScenarioError(f"{consumer} may not receive the dock pose (only {', '.join(sp.ALLOWED_CONSUMERS)})")
    path = sp.consumer_file(scenario_dir, consumer)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as handle:
        handle.write(json.dumps(consumer_payload(consumer, scene), indent=2, sort_keys=True) + "\n")
    _append(ledger, {"consumer": consumer, "scene_hash": digest, "file": str(path.relative_to(scenario_dir)),
                     "utc": _utc()})
    return path


def _append(path: Path, line: Mapping[str, Any]) -> None:
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(line, sort_keys=True) + "\n")


def prepare(series_path: Path, index: int, scenario_dir: Optional[Path]) -> dict[str, Any]:
    """The seed's public half; with scenario_dir (a new folder) also its private files, ledger and scenario.json."""
    series = read_series(series_path)
    scene = build_scene(series, index)
    public = public_half(series, jc.sha256_file(Path(series_path)), scene)
    if scenario_dir is None:
        return public
    scenario_dir = Path(scenario_dir)
    try:
        scenario_dir.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        raise ScenarioError(f"{scenario_dir} exists: one scenario per run") from None
    for consumer in series["consumers"]:
        deliver(scenario_dir, consumer, scene)
    (scenario_dir / sp.SCENARIO_FILE).write_text(json.dumps(public, indent=2, sort_keys=True) + "\n",
                                                 encoding="utf-8")
    return public


def _new_series(args: argparse.Namespace) -> int:
    out = Path(args.out) if args.out else private_dir() / "scenarios" / f"{args.series_id}.json"
    series = new_series(args.series_id, datetime.now(timezone.utc), args.max_offset_m)
    print(write_series(series, out))
    return 0


def _prepare(args: argparse.Namespace) -> int:
    print(json.dumps(prepare(Path(args.series), args.index, None if args.dry_run else Path(args.out)),
                     sort_keys=True))
    return 0


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="scenario_service", description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(required=True)
    n = sub.add_parser("new-series", help="freeze a new series of seeds in the private folder")
    n.add_argument("--series-id", required=True)
    n.add_argument("--out", help="the series file (default: the private folder's scenarios/<id>.json)")
    n.add_argument("--max-offset-m", type=float, default=DEFAULT_MAX_OFFSET_M)
    n.set_defaults(func=_new_series)
    p = sub.add_parser("prepare", help="one seed's scenario folder; prints its public half")
    p.add_argument("--series", required=True)
    p.add_argument("--index", type=int, required=True)
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument("--out", help="the new scenario folder")
    group.add_argument("--dry-run", action="store_true", help="print the public half only; deliver nothing")
    p.set_defaults(func=_prepare)
    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except (RecordError, ScenarioError) as exc:
        print(f"error: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
