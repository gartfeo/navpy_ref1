"""Layer 1's aircraft model as a read-only plug-in of the HITL bridge (hitl_final_approach.md sections 9, 10).

Layer 1 supplies a JSBSim model folder (the bridge's --model-dir; jsbsim_files.prepare_run_folder copies all
of it) and a private JSON declaration of that folder, its "model input":

    {"schema_version": 1, "name": <folder name>, "version": <Layer-1 version label>,
     "files_sha256": {<every file in the folder, relative POSIX path>: <sha256>},
     "jsbsim_versions": {<"program" | "package">: <JSBSim version it was validated with>},
     "control_map": {"left_elevon": i, "right_elevon": i, "throttle": i},    HIL_ACTUATOR_CONTROLS indices
     "control_inputs": {"aileron": p, "elevator": p, "throttle": p},         properties the model's FCS reads
     "max_integration_dt_s": s,                                               the longest step it supports
     "actuator_lag": {"owner": "bridge" | "model", "bridge_servo_speed_s": s},
     "envelope": {<quantity, unit in the name>: {"min": x, "max": x}}}

A declaration is refused when a field is missing, unknown or malformed, when its control map or inputs are not
the bridge's (physics.py, jsbsim_plant.py), when the servo lag would be applied by both model and bridge (or by
neither while the bridge owns it), or when an envelope band is empty. check_model_files refuses a folder whose
name or files differ from the declaration, or that holds a link. Nothing here writes to the model folder.
"""
from __future__ import annotations

import os
import stat
from collections.abc import Mapping
from pathlib import Path, PurePosixPath
from typing import Any

from tools.hitl import json_checks as jc
from tools.hitl.json_checks import RecordError
from tools.tuning.hitl_bridge import jsbsim_plant as plant
from tools.tuning.hitl_bridge.bridge import PLANT_PACKAGE, PLANT_PROGRAM
from tools.tuning.hitl_bridge.physics import LEFT_ELEVON_INDEX, RIGHT_ELEVON_INDEX, THROTTLE_INDEX

MODEL_INPUT_SCHEMA_VERSION = 1
LAG_OWNER_BRIDGE = "bridge"
LAG_OWNER_MODEL = "model"
PLANTS = (PLANT_PROGRAM, PLANT_PACKAGE)

# The bridge's side of the contract.
BRIDGE_CONTROL_MAP = {"left_elevon": LEFT_ELEVON_INDEX, "right_elevon": RIGHT_ELEVON_INDEX,
                      "throttle": THROTTLE_INDEX}
BRIDGE_CONTROL_INPUTS = {"aileron": plant.AILERON_CMD, "elevator": plant.ELEVATOR_CMD,
                         "throttle": plant.THROTTLE_CMD}
# The JSBSim properties the bridge turns into HIL_STATE_QUATERNION (state_fields.py), by field.
BRIDGE_SENSOR_MAP = {
    "lat": plant.LAT_DEG, "lon": plant.LON_DEG, "alt_msl": plant.ALT_MSL_M,
    "roll": plant.ROLL_RAD, "pitch": plant.PITCH_RAD, "yaw": plant.YAW_RAD,
    "p": plant.P_RAD_S, "q": plant.Q_RAD_S, "r": plant.R_RAD_S,
    "vn": plant.VN_FPS, "ve": plant.VE_FPS, "vd": plant.VD_FPS,
    "ind_airspeed": plant.IAS_FPS, "true_airspeed": plant.TAS_FPS,
    "fx": plant.FORCE_LBS[0], "fy": plant.FORCE_LBS[1], "fz": plant.FORCE_LBS[2], "mass": plant.MASS_SLUGS,
}

MODEL_INPUT_SPEC: Mapping[str, jc.Check] = {
    "schema_version": jc.equals(MODEL_INPUT_SCHEMA_VERSION),
    "name": jc.folder_name,
    "version": jc.text,
    "files_sha256": jc.map_of(jc.hex_digits(64)),
    "jsbsim_versions": jc.map_of(jc.text, keys=PLANTS),
    "control_map": jc.map_of(jc.index),
    "control_inputs": jc.map_of(jc.text),
    "max_integration_dt_s": jc.positive,
    "actuator_lag": jc.obj({"owner": jc.choice((LAG_OWNER_BRIDGE, LAG_OWNER_MODEL)),
                            "bridge_servo_speed_s": jc.non_negative}),
    "envelope": jc.map_of(jc.obj({"min": jc.finite, "max": jc.finite})),
}


def _plain_relative(name: str) -> bool:
    pure = PurePosixPath(name)
    return "\\" not in name and not pure.is_absolute() and ".." not in pure.parts and str(pure) == name


def model_input_problems(model_input: Mapping[str, Any], path: str = "model_input") -> list[str]:
    """What in a well-formed declaration disagrees with the bridge or with itself."""
    problems = []
    if dict(model_input["control_map"]) != BRIDGE_CONTROL_MAP:
        problems.append(f"{path}.control_map {dict(model_input['control_map'])} is not the bridge's "
                        f"{BRIDGE_CONTROL_MAP}")
    if dict(model_input["control_inputs"]) != BRIDGE_CONTROL_INPUTS:
        problems.append(f"{path}.control_inputs {dict(model_input['control_inputs'])} are not the bridge's "
                        f"{BRIDGE_CONTROL_INPUTS}")
    main_file = f"{model_input['name']}.xml"
    if main_file not in model_input["files_sha256"]:
        problems.append(f"{path}.files_sha256 lacks the model file {main_file}")
    problems.extend(f"{path}.files_sha256 entry {name!r} is not a plain relative POSIX path"
                    for name in model_input["files_sha256"] if not _plain_relative(name))
    lag = model_input["actuator_lag"]
    if lag["owner"] == LAG_OWNER_MODEL and lag["bridge_servo_speed_s"] != 0:
        problems.append(f"{path}.actuator_lag: the model owns the servo lag, so the bridge's servo speed must "
                        f"be 0 (else the lag is applied twice)")
    if lag["owner"] == LAG_OWNER_BRIDGE and lag["bridge_servo_speed_s"] == 0:
        problems.append(f"{path}.actuator_lag: the bridge owns the servo lag, so its servo speed must be above 0")
    problems.extend(f"{path}.envelope.{name}: min {band['min']} is not below max {band['max']}"
                    for name, band in model_input["envelope"].items() if not band["min"] < band["max"])
    return problems


def validate_model_input(data: Any) -> None:
    """Refuse a declaration that is incomplete or does not fit the bridge."""
    jc.require(data, MODEL_INPUT_SPEC, "model_input", model_input_problems)


def read_model_input(path: Path) -> dict[str, Any]:
    """Layer 1's declaration from its private JSON file, validated."""
    data = jc.read_json(Path(path))
    validate_model_input(data)
    return data


def _is_link(path: Path) -> bool:
    """A symbolic link or a Windows junction (any reparse point)."""
    info = os.lstat(path)
    return stat.S_ISLNK(info.st_mode) or bool(getattr(info, "st_file_attributes", 0)
                                              & stat.FILE_ATTRIBUTE_REPARSE_POINT)


def _folder_files(folder: Path, links: list) -> list[Path]:
    """The files under folder; links are collected, not followed."""
    files = []
    for entry in os.scandir(folder):
        path = Path(entry.path)
        if _is_link(path):
            links.append(path)
        elif entry.is_dir(follow_symlinks=False):
            files.extend(_folder_files(path, links))
        else:
            files.append(path)
    return files


def model_files_sha256(model_dir: Path) -> dict[str, str]:
    """sha256 of every file in the model folder, by relative POSIX path; read only. A link in the folder is
    refused: the bridge's copy (shutil.copytree) follows it to files the declaration cannot pin."""
    model_dir = Path(model_dir)
    links: list[Path] = []
    files = _folder_files(model_dir, links)
    if links:
        raise RecordError([f"model folder entry {path.relative_to(model_dir).as_posix()} is a link"
                           for path in sorted(links)])
    return {path.relative_to(model_dir).as_posix(): jc.sha256_file(path) for path in sorted(files)}


def check_model_files(model_input: Mapping[str, Any], model_dir: Path) -> None:
    """Refuse a model folder whose name or files differ from the declaration: every file the bridge copies
    must be declared with its hash."""
    model_dir = Path(model_dir)
    if not model_dir.is_dir():
        raise RecordError([f"no model folder {model_dir}"])
    problems = []
    if model_dir.name != model_input["name"]:
        problems.append(f"model folder {model_dir.name!r} is not the declared model {model_input['name']!r}")
    actual, declared = model_files_sha256(model_dir), model_input["files_sha256"]
    problems.extend(f"model file {name} is declared but missing" for name in sorted(declared.keys() - actual.keys()))
    problems.extend(f"model file {name} is not declared" for name in sorted(actual.keys() - declared.keys()))
    problems.extend(f"model file {name} differs from its declared sha256"
                    for name in sorted(declared.keys() & actual.keys()) if declared[name] != actual[name])
    if problems:
        raise RecordError(problems)
