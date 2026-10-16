"""The lockstep run's input manifest: what it consumes, hashed (step 3's, for lockstep).

The same ``input_manifest.json`` as wall mode (``eval_gcs_demo_inputs``):
source, scenario, NavPy's DEM, assets, vision profiles and dialect. The SITL
part is the serial-step firmware lock, the merged pre-boot ``defaults.parm``
(WSL's stock ``plane.parm`` plus the lockstep overrides), the deployed Lua and the
pinned SITL terrain tiles (the wall eval's ``seed/terrain``) and the topology
(``lockstep_topology``); the run options are the seed, the operator's budgets, the
tick grid and the participants' role configs. The host delay seed is a host
perturbation (acceptance (c)) and stays out: a delayed run must consume the
same inputs. The WSL root's path is host data (``lockstep_run_host.json``).
"""

from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping
from pathlib import Path
from typing import TYPE_CHECKING

from navpy.sim.lockstep.coordinator_spawn import PYTHON_HASH_SEED
from navpy.sim.lockstep.coordinator_types import Manifest
from scripts import lockstep_wsl
from scripts.eval_gcs_demo_inputs import write_input_manifest
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_scenario import DemoScenario
from scripts.eval_gcs_demo_sitl_root import SITL_LUA_DIR, sha256_bytes, sha256_file

if TYPE_CHECKING:
    from scripts.eval_gcs_demo_lockstep import LockstepOptions


def _canonical_sha256(value: object) -> str:
    text = json.dumps(value, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def lockstep_sitl_inputs(defaults: str, tiles: Mapping[str, bytes]) -> dict[str, JsonValue]:
    """``defaults`` is the merged ``defaults.parm`` the SITL boots with (WSL's
    stock ``plane.parm`` plus the overrides, ``lockstep_wsl.merged_defaults``);
    ``tiles`` the terrain tiles its root gets (``terrain_tiles``)."""
    return {
        "firmware_lock_sha256": sha256_file(lockstep_wsl.FIRMWARE_LOCK),
        "default_overrides": dict(sorted(lockstep_wsl.LOCKSTEP_DEFAULT_OVERRIDES.items())),
        "defaults_parm_sha256": hashlib.sha256(defaults.encode("utf-8")).hexdigest(),
        "aas_params_lua_sha256": sha256_file(SITL_LUA_DIR / "aas_params.lua"),
        "terrain_tiles_sha256": {name: sha256_bytes(data) for name, data in sorted(tiles.items())},
    }


def write_lockstep_input_manifest(run_dir: Path, scenario: DemoScenario, plan: Manifest,
                                  options: "LockstepOptions", defaults: str,
                                  tiles: Mapping[str, bytes],
                                  topology: Mapping[str, JsonValue]) -> dict[str, JsonValue]:
    """``topology`` is ``lockstep_topology.topology_inputs``: the wall launcher's
    hash, the SITLs' homes, the router entry table and binary, ``FIFO_CAP``."""
    roles = {name: {"hooks": spec.hooks, "devices": list(spec.devices), "config": spec.config}
             for name, spec in sorted(plan.roles.items())}
    run_options: dict[str, JsonValue] = {
        "lockstep": True,
        "seed": options.seed,
        "timeout_s": options.timeout_s,
        "confirmation_delay_s": options.confirmation_delay_s,
        "epoch_us": plan.epoch_us,
        "max_frame_gap_us": plan.max_frame_gap_us,
        "drain_period_us": plan.drain_period_us,
        "mission_budget_us": plan.mission_budget_us,
        "roles_sha256": _canonical_sha256(roles),
    }
    # A participant's whole environment besides its record and host start
    # variables (``coordinator_spawn.hermetic_environment``).
    environment = {"PYTHONHASHSEED": PYTHON_HASH_SEED}
    sitl = {**lockstep_sitl_inputs(defaults, tiles), "topology": dict(topology)}
    return write_input_manifest(run_dir, environment, sitl, "", run_options, scenario)


__all__ = ["lockstep_sitl_inputs", "write_lockstep_input_manifest"]
