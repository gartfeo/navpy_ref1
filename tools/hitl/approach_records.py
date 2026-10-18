"""Run record of one HITL approach (hitl_final_approach.md section 9, stage 1 step 1).

One JSON manifest per run, approach_record.json, in the run's folder under the private folder
(tools/tuning/private_data.py), never in a repository checkout. It names what the run used: the run's identity and
mode, the source, Layer 1's model input (model_input.py, embedded whole), the bridge as the run starts it, the
law and what the mode substitutes for the real aircraft chain. Two runs can then be shown to share their
inputs, and a changed input starts a new series.

A record is refused (RecordError, every problem listed) when a field is missing, unknown or malformed, when
its model input is refused, or when the run disagrees with it: a plant version the model was not validated
with, a step that is not the bridge's 1/N s or is longer than the model's maximum, a servo speed other than
the declared one, a sensor map that is not the bridge's, a law other than the pure-vision law (AGENTS.md), or a
substitution of the mode left out or reworded. new_record also refuses a model folder that differs from the model input.
"""
from __future__ import annotations

import hashlib
import json
import subprocess
from collections.abc import Mapping
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

from navpy.core.kernel.config.navigation_args import NavigationAlgorithm
from tools.hitl import json_checks as jc
from tools.hitl.json_checks import RecordError
from tools.hitl.model_input import (BRIDGE_SENSOR_MAP, MODEL_INPUT_SPEC, PLANTS, check_model_files,
                                    model_input_problems)
from tools.tuning.hitl_bridge.board_clock import step_rate_hz
from tools.tuning.private_data import private_dir

RECORD_FILE = "approach_record.json"
SCHEMA_VERSION = 1
REPO_ROOT = Path(__file__).resolve().parents[2]
NAVPY_SOURCE = Path("src", "navpy")     # the package folder every navpy checkout holds at its top

# What each mode runs in place of the real aircraft chain (section 9; later stages add their mode).
# The world (Unreal) and the physics (JSBSim) are simulated in every mode: that is the scope, not a substitute.
MODE_SUBSTITUTIONS: Mapping[str, Mapping[str, str]] = {
    "pc-sitl": {
        "autopilot": "ArduPilot SITL on the PC, not the Pixhawk 6X",
        "companion_computer": "navpy on the PC, not the Orin",
        "gimbal": "ZR10 emulator (tools/hitl/zr10_emulator.py), not the real ZR10",
        "camera": "Unreal render encoded on the PC, not the ZR10 camera",
        "links": "PC router over local sockets, not the flight radios and companion UART",
    },
}
# The final approach flies the pure-vision law only; the legacy laws read dock geometry.
APPROACH_LAWS = (NavigationAlgorithm.VISION_NAV_PN.value,)

_SOURCE = jc.obj({"git_head": jc.hex_digits(40), "uncommitted_diff_sha256": jc.hex_digits(64),
                  "untracked_sha256": jc.hex_digits(64)})
RECORD_SPEC: Mapping[str, jc.Check] = {
    "schema_version": jc.equals(SCHEMA_VERSION),
    "run": jc.obj({"run_id": jc.folder_name, "mode": jc.choice(tuple(MODE_SUBSTITUTIONS)), "created_utc": jc.utc}),
    "sources": jc.map_of(_SOURCE, required=("navpy",)),
    "model_input": jc.obj(MODEL_INPUT_SPEC),
    "bridge": jc.obj({"plant": jc.choice(PLANTS), "jsbsim_version": jc.text, "integration_dt_s": jc.positive,
                      "loop_rate_hz": jc.positive, "servo_speed_s": jc.non_negative,
                      "sensor_map": jc.map_of(jc.text)}),
    "law": jc.obj({"algorithm": jc.text}),
    "substitutions": jc.map_of(jc.text),
}


@dataclass(frozen=True)
class BridgeRun:
    """The bridge as this run starts it (hitl_bridge/bridge.py options)."""
    plant: str
    jsbsim_version: str
    integration_dt_s: float
    loop_rate_hz: float
    servo_speed_s: float


def _run_problems(record: Mapping[str, Any]) -> list[str]:
    model_input, bridge = record["model_input"], record["bridge"]
    problems = model_input_problems(model_input, "record.model_input")
    if dict(bridge["sensor_map"]) != BRIDGE_SENSOR_MAP:
        problems.append("record.bridge.sensor_map is not the bridge's state properties (jsbsim_plant.py)")
    validated = model_input["jsbsim_versions"].get(bridge["plant"])
    if validated != bridge["jsbsim_version"]:
        problems.append(f"record.bridge: plant {bridge['plant']} JSBSim {bridge['jsbsim_version']} is not a "
                        f"version the model was validated with ({validated or 'none for this plant'})")
    try:
        step_rate_hz(bridge["integration_dt_s"])
    except ValueError as exc:
        problems.append(f"record.bridge.integration_dt_s: {exc}")
    except OverflowError:
        problems.append(f"record.bridge.integration_dt_s {bridge['integration_dt_s']} is too short to be 1/N s")
    if bridge["integration_dt_s"] > model_input["max_integration_dt_s"]:
        problems.append(f"record.bridge.integration_dt_s {bridge['integration_dt_s']} is longer than the "
                        f"model's maximum {model_input['max_integration_dt_s']}")
    declared_speed = model_input["actuator_lag"]["bridge_servo_speed_s"]
    if bridge["servo_speed_s"] != declared_speed:
        problems.append(f"record.bridge.servo_speed_s {bridge['servo_speed_s']} is not the model's {declared_speed}")
    if record["law"]["algorithm"] not in APPROACH_LAWS:
        problems.append(f"record.law.algorithm {record['law']['algorithm']!r} is not an approach law "
                        f"({', '.join(APPROACH_LAWS)})")
    mode, substitutions = record["run"]["mode"], record["substitutions"]
    for name, text in sorted(MODE_SUBSTITUTIONS[mode].items()):
        if name not in substitutions:
            problems.append(f"record.substitutions.{name} is missing for mode {mode}")
        elif substitutions[name] != text:
            problems.append(f"record.substitutions.{name} {substitutions[name]!r} is not mode {mode}'s {text!r}")
    return problems


def validate_record(data: Any) -> None:
    """Refuse a record that is incomplete, malformed or inconsistent (see the module docstring)."""
    jc.require(data, RECORD_SPEC, "record", _run_problems)


def _git(repo: Path, *args: str) -> bytes:
    result = subprocess.run(["git", "-C", str(repo), *args], capture_output=True, timeout=60, check=False)
    if result.returncode != 0:
        raise RecordError([f"git {' '.join(args)} failed in {repo}: {result.stderr.decode(errors='replace')}"])
    return result.stdout


def source_identity(repo: Path = REPO_ROOT) -> dict[str, str]:
    """The checkout's commit, uncommitted diff and untracked files, in the demo eval's input-manifest fields
    (scripts/eval_gcs_demo_inputs.py)."""
    untracked = hashlib.sha256()
    listing = _git(repo, "ls-files", "--others", "--exclude-standard", "-z").decode("utf-8")
    for name in sorted(n for n in listing.split("\0") if n):
        path = Path(repo) / name
        untracked.update(f"{name}\0{jc.sha256_file(path) if path.is_file() else 'absent'}\n".encode("utf-8"))
    return {"git_head": _git(repo, "rev-parse", "HEAD").decode("ascii").strip(),
            "uncommitted_diff_sha256": hashlib.sha256(_git(repo, "diff", "HEAD", "--binary")).hexdigest(),
            "untracked_sha256": untracked.hexdigest()}


def new_record(*, run_id: str, mode: str, created: datetime, model_input: Mapping[str, Any], model_dir: Path,
               bridge: BridgeRun, law: str, sources: Mapping[str, Mapping[str, str]],
               extra_substitutions: Optional[Mapping[str, str]] = None) -> dict[str, Any]:
    """The validated record of a run about to start; the model folder must match the model input."""
    if created.tzinfo is None:
        raise RecordError(["created must carry its time zone"])
    record = {
        "schema_version": SCHEMA_VERSION,
        "run": {"run_id": run_id, "mode": mode,
                "created_utc": created.astimezone(timezone.utc).strftime(jc.UTC_FORMAT)},
        "sources": {name: dict(source) for name, source in sources.items()},
        "model_input": json.loads(json.dumps(model_input)),
        "bridge": {"plant": bridge.plant, "jsbsim_version": bridge.jsbsim_version,
                   "integration_dt_s": bridge.integration_dt_s, "loop_rate_hz": bridge.loop_rate_hz,
                   "servo_speed_s": bridge.servo_speed_s, "sensor_map": dict(BRIDGE_SENSOR_MAP)},
        "law": {"algorithm": law},
        "substitutions": {**MODE_SUBSTITUTIONS.get(mode, {}), **(extra_substitutions or {})},
    }
    validate_record(record)
    check_model_files(record["model_input"], model_dir)
    return record


def _inside(path: Path, folder: Path) -> bool:
    return path == folder or folder in path.parents


def _navpy_checkout(path: Path) -> Optional[Path]:
    """This repository, or the nearest folder at or above path that holds a navpy source tree (the main checkout, a
    worktree, a clone or a copy), whatever git can read there. The private folder's own git repository holds none."""
    if _inside(path, REPO_ROOT):
        return REPO_ROOT
    return next((folder for folder in (path, *path.parents) if (folder / NAVPY_SOURCE).is_dir()), None)


def write_record(record: Mapping[str, Any], run_dir: Path, private_root: Optional[Path] = None) -> Path:
    """Write the run's one record into its run folder, which must be in the private folder (private_root, by
    default tools/tuning/private_data.private_dir()) and in no navpy checkout; refuses an invalid record and a
    second record for the same run."""
    validate_record(record)
    run_dir = Path(run_dir).resolve()
    root = Path(private_dir() if private_root is None else private_root).resolve()
    if not _inside(run_dir, root):
        raise RecordError([f"run folder {run_dir} is not in the private folder {root}; records are private data"])
    checkout = _navpy_checkout(run_dir)
    if checkout is not None:
        raise RecordError([f"run folder {run_dir} is in navpy checkout {checkout}; records are private data"])
    path = run_dir / RECORD_FILE
    run_dir.mkdir(parents=True, exist_ok=True)
    try:
        with path.open("x", encoding="utf-8") as handle:
            handle.write(json.dumps(record, indent=2, sort_keys=True, allow_nan=False) + "\n")
    except FileExistsError:
        raise RecordError([f"{path} exists: one record per run"]) from None
    return path


def read_record(path: Path) -> dict[str, Any]:
    """A run's record, validated."""
    data = jc.read_json(Path(path))
    validate_record(data)
    return data
