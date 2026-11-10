"""The public half of one seed's scenario (hitl_final_approach.md section 9, stage 1 step 14).

The scenario service (scenario_service.py) writes it into the run's scenario folder as SCENARIO_FILE: the scenario's
identity (series, seed index, scene hash), the dock-independent search lane, the run's limits and the dock's design
size (step 23b: phase A's dock is a pickup truck standing on the ground; only the pre-series check reads it, never
navpy). It holds no dock pose and no seed: the runner and navpy (through the run record, approach_records.py) receive only this. The dock
pose goes only to the consumers in ALLOWED_CONSUMERS, each in its own file under PRIVATE_DIR, which the command side
never opens; the runner hands Unreal its file's path without reading it.
"""
from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

from tools.hitl import json_checks as jc
from tools.hitl.json_checks import RecordError

SCHEMA_VERSION = 2              # 2: the dock's design size (step 23b)
SCENARIO_FILE = "scenario.json"
LEDGER_FILE = "deliveries.jsonl"
PRIVATE_DIR = "private"
# The three consumers the plan lets see the dock pose: the renderer, the contact service (step 26) and scoring.
ALLOWED_CONSUMERS = ("unreal", "contact", "scoring")
# Every series needs all three: Unreal renders the dock, the contact service judges contact (step 26) and scoring
# checks what Unreal built.
REQUIRED_CONSUMERS = ("unreal", "contact", "scoring")

LANE_SPEC: Mapping[str, jc.Check] = {
    "bearing_deg": jc.rule(lambda v: jc.is_number(v) and 0 <= v < 360, "must be a number in [0, 360)"),
    "start_m": jc.non_negative,
    "length_m": jc.positive,
    "height_m": jc.positive,
    "repeats": jc.index,
}
LIMITS_SPEC: Mapping[str, jc.Check] = {
    "observe_s": jc.positive,        # the deadline on the lane after acquisition starts
    "min_height_m": jc.positive,     # the recovery floor (flight_floor.HeightFloor), released at the confirm (23b)
}
# Step 23b, phase A: a pickup truck on the ground at the dock POI, detected as COCO truck (hitl_dock_detector.md
# section 5). A scene design fact, the same in every seed; its pose stays private.
DOCK_KINDS = ("pickup",)
DOCK_PLACEMENTS = ("ground",)
DOCK_SPEC: Mapping[str, jc.Check] = {
    "kind": jc.choice(DOCK_KINDS),
    "length_m": jc.positive,
    "width_m": jc.positive,
    "height_m": jc.positive,
    "placement": jc.choice(DOCK_PLACEMENTS),
}


def consumer_problems(consumers: Any, path: str) -> list[str]:
    """A consumer list that repeats one, or lacks one every series needs now."""
    problems = [f"{path} repeats a consumer"] if len(set(consumers)) != len(consumers) else []
    return problems + [f"{path} lacks {name}" for name in REQUIRED_CONSUMERS if name not in consumers]


def consumers_check(value: Any, path: str, problems: list) -> None:
    """Allowed consumers only, none repeated, every required one present."""
    before = len(problems)
    jc.list_of(jc.choice(ALLOWED_CONSUMERS), True)(value, path, problems)
    if len(problems) == before:
        problems.extend(consumer_problems(value, path))


PUBLIC_SPEC: Mapping[str, jc.Check] = {
    "schema_version": jc.equals(SCHEMA_VERSION),
    "series_id": jc.folder_name,
    "series_sha256": jc.hex_digits(64),
    "seed_index": jc.index,
    "scene_hash": jc.hex_digits(64),
    "consumers": consumers_check,
    "lane": jc.obj(LANE_SPEC),
    "limits": jc.obj(LIMITS_SPEC),
    "dock_size": jc.obj(DOCK_SPEC),
}


def validate_public(data: Any, what: str = "scenario") -> None:
    jc.require(data, PUBLIC_SPEC, what, lambda _: [])


def read_public(scenario_dir: Path) -> dict[str, Any]:
    """The scenario folder's public half, validated."""
    data = jc.read_json(Path(scenario_dir) / SCENARIO_FILE)
    validate_public(data)
    return data


def consumer_file(scenario_dir: Path, consumer: str) -> Path:
    """Where the service delivers a consumer's private file; the command side only passes the path on."""
    if consumer not in ALLOWED_CONSUMERS:
        raise RecordError([f"{consumer} is not a scenario consumer ({', '.join(ALLOWED_CONSUMERS)})"])
    return Path(scenario_dir) / PRIVATE_DIR / f"{consumer}.json"


__all__ = ["ALLOWED_CONSUMERS", "DOCK_KINDS", "DOCK_PLACEMENTS", "DOCK_SPEC", "LANE_SPEC", "LEDGER_FILE", "LIMITS_SPEC", "PRIVATE_DIR", "PUBLIC_SPEC",
           "REQUIRED_CONSUMERS", "SCENARIO_FILE", "SCHEMA_VERSION", "consumer_file", "consumer_problems",
           "consumers_check", "read_public", "validate_public"]
