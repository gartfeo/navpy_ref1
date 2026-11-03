"""Scoring of one seed's scenario (hitl_final_approach.md section 9, stage 1 step 14): did Unreal build the delivered
scene, and did the dock pose reach anyone but the allowed consumers?

Scoring is one of the three consumers (scenario_public.ALLOWED_CONSUMERS): it reads its own private file and Unreal's
readback, private/unreal_applied.json, which Unreal writes beside its scenario file (NavPyDockBoardSubsystem: the
offset and yaw it read back after the move, the terrain snap and the board's pose). It rebuilds the scene from that
readback: the hash matches only when Unreal built the delivered scene. It also checks the moved dock stands on the
terrain, that Unreal's log (which the runner reads) carries no pose, and the scenario folder's ledger. The printed
verdict names hashes and problems, never a pose; the details go to private/scoring_verdict.json.

    python -s -m tools.hitl.scenario_scoring --scenario DIR --unreal-log LOG
"""
from __future__ import annotations

import argparse
import json
import re
from collections.abc import Mapping
from pathlib import Path
from typing import Any, Optional

from tools.hitl import json_checks as jc
from tools.hitl import scenario_public as sp
from tools.hitl import scenario_service as service
from tools.hitl.json_checks import RecordError

# Unreal sets the moved dock's base on the sampled terrain (owner, 2026-11-04), so its own terrain check reads about
# 0; my value allows the Cesium tile's refinement between the two samples.
TERRAIN_TOLERANCE_M = 0.05
VERDICT_FILE = "scoring_verdict.json"
UNREAL_APPLIED_FILE = "unreal_applied.json"     # NavPyScenario::AppliedFileName, written by Unreal beside its file

APPLIED_LINE = re.compile(r"NavPyScenario applied scene_hash=([0-9a-f]{64})\s*$", re.MULTILINE)
REFUSED_LINE = re.compile(r"NavPyScenario refused: (.*)")
TERRAIN_LINE = re.compile(r"NavPyScenario terrain check: dock base ([+-]?\d+\.\d+) m above terrain")
# Unreal's log lines that carry the dock's or the board's pose; with a scenario they must not appear (the runner reads
# the log for Unreal's ready line).
POSE_LINES = ("NavPyBoardPose ", "NavPy ChArUco board on dock", "NavPy ChArUco board centre lat",
              "NavPy dock terrain check (WGS84 ellipsoid)")
# Unreal writes JSON numbers as doubles. The board's pose and the terrain snap are kept, not scored.
UNSCORED_FIELDS = ("board", "terrain_snap_cm")
APPLIED_SPEC: Mapping[str, jc.Check] = {
    "schema_version": jc.rule(lambda v: jc.is_number(v) and v == service.UNREAL_SCENE_SCHEMA_VERSION,
                              f"must be {service.UNREAL_SCENE_SCHEMA_VERSION}"),
    "scene_hash": jc.hex_digits(64),
    "east_mm": jc.finite,
    "north_mm": jc.finite,
    "yaw_mdeg": jc.finite,
}


def audit(scenario_dir: Path) -> dict[str, Any]:
    """The ledger's deliveries by consumer, refused requests, deliveries outside the three, and any file in the folder
    that neither the service, Unreal's readback nor scoring wrote."""
    scenario_dir = Path(scenario_dir)
    lines = [json.loads(text) for text in (scenario_dir / sp.LEDGER_FILE).read_text(encoding="utf-8").splitlines()
             if text.strip()]
    delivered = [line for line in lines if not line.get("refused")]
    expected = {scenario_dir / sp.SCENARIO_FILE, scenario_dir / sp.LEDGER_FILE,
                scenario_dir / sp.PRIVATE_DIR / VERDICT_FILE, scenario_dir / sp.PRIVATE_DIR / UNREAL_APPLIED_FILE,
                *(sp.consumer_file(scenario_dir, line["consumer"]) for line in delivered
                  if line["consumer"] in sp.ALLOWED_CONSUMERS)}
    counts: dict[str, int] = {}
    for line in delivered:
        counts[line["consumer"]] = counts.get(line["consumer"], 0) + 1
    return {"deliveries": counts,
            "outside_allowed": sorted({line["consumer"] for line in delivered
                                       if line["consumer"] not in sp.ALLOWED_CONSUMERS}),
            "refused": sorted({line["consumer"] for line in lines if line.get("refused")}),
            "stray_files": sorted(str(path.relative_to(scenario_dir)) for path in scenario_dir.rglob("*")
                                  if path.is_file() and path not in expected)}


def _whole(value: Any) -> Optional[int]:
    whole = isinstance(value, (int, float)) and not isinstance(value, bool) and value == int(value)
    return int(value) if whole else None


def _readback(scenario_dir: Path) -> tuple[Optional[dict[str, Any]], list[str]]:
    """Unreal's private readback: its echoed hash and dock move in whole mm/mdeg, or the problems with it."""
    path = scenario_dir / sp.PRIVATE_DIR / UNREAL_APPLIED_FILE
    if not path.is_file():
        return None, [f"Unreal wrote no {UNREAL_APPLIED_FILE}"]
    try:
        data = jc.read_json(path)
        if not isinstance(data, dict):
            raise RecordError([f"{UNREAL_APPLIED_FILE} must be an object"])
        jc.require({key: value for key, value in data.items() if key not in UNSCORED_FIELDS}, APPLIED_SPEC,
                   UNREAL_APPLIED_FILE, lambda _: [])
    except RecordError as exc:
        return None, [f"{UNREAL_APPLIED_FILE}: {exc}"]
    dock = {key: _whole(data[key]) for key in ("east_mm", "north_mm", "yaw_mdeg")}
    if None in dock.values():
        return None, [f"{UNREAL_APPLIED_FILE} has a dock move that is not whole mm/mdeg"]
    return {"scene_hash": data["scene_hash"], **dock}, []


def verify_unreal(scenario_dir: Path, unreal_log: Path) -> dict[str, Any]:
    """Did Unreal build the delivered scene on the terrain, with no pose in its log and no delivery outside the
    allowed consumers? The result names the hash and the verdict, never the pose."""
    scenario_dir = Path(scenario_dir)
    delivered = jc.read_json(sp.consumer_file(scenario_dir, "scoring"))
    expected = delivered["scene"]
    text = Path(unreal_log).read_text(encoding="utf-8", errors="replace") if Path(unreal_log).is_file() else ""
    logged, refused, terrain = APPLIED_LINE.findall(text), REFUSED_LINE.findall(text), TERRAIN_LINE.findall(text)
    applied, problems = _readback(scenario_dir)
    details: dict[str, Any] = {"refused": refused, "applied": applied, "terrain_m": None}
    if not text:
        problems.append("no Unreal log")
    if refused:
        problems.append("Unreal refused the scenario file")
    if logged != [delivered["scene_hash"]]:
        problems.append(f"Unreal logged {len(logged)} applied scenes, not the delivered one once")
    rebuilt_hash = None
    if applied is not None:
        dock = {key: applied[key] for key in ("east_mm", "north_mm", "yaw_mdeg")}
        rebuilt_hash = service.scene_hash({**expected, "dock": dock})
        if applied["scene_hash"] != delivered["scene_hash"]:
            problems.append("Unreal applied another scenario file")
        if rebuilt_hash != delivered["scene_hash"]:
            problems.append("the scene Unreal built does not reproduce the delivered scene hash")
    if len(terrain) != 1:
        problems.append(f"Unreal logged {len(terrain)} dock terrain checks, not 1")
    else:
        details["terrain_m"] = float(terrain[0])
        if abs(float(terrain[0])) > TERRAIN_TOLERANCE_M:
            problems.append(f"the moved dock is more than {TERRAIN_TOLERANCE_M:g} m off the terrain")
    leaks = sorted({marker.strip() for marker in POSE_LINES if marker in text})
    if leaks:
        problems.append(f"Unreal's log carries pose lines: {', '.join(leaks)}")
    ledger = audit(scenario_dir)
    if ledger["outside_allowed"] or ledger["refused"] or ledger["stray_files"]:
        problems.append("the scenario folder shows a delivery outside the allowed consumers or a file the service "
                        "did not write")
    verdict = {"scene_hash": delivered["scene_hash"], "rebuilt_scene_hash": rebuilt_hash,
               "reproduced": not problems, "problems": problems, "audit": ledger}
    out = scenario_dir / sp.PRIVATE_DIR / VERDICT_FILE
    out.write_text(json.dumps({**verdict, "expected_dock": expected["dock"], **details}, indent=2) + "\n",
                   encoding="utf-8")
    return verdict


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="scenario_scoring", description=__doc__.split("\n\n")[0])
    parser.add_argument("--scenario", required=True, help="the run's scenario folder")
    parser.add_argument("--unreal-log", required=True)
    args = parser.parse_args(argv)
    try:
        verdict = verify_unreal(Path(args.scenario), Path(args.unreal_log))
    except RecordError as exc:
        print(f"error: {exc}")
        return 2
    print(json.dumps(verdict, sort_keys=True))
    return 0 if verdict["reproduced"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
