"""Bind the synchronized adapter to the normal approach configuration."""
from __future__ import annotations

import hashlib
import math
from pathlib import Path
import struct

from scripts.eval_param_file import parse_parm
from navpy.modules.navigation.nav.final_approach_airframe_config import (
    FinalApproachAirframeConfigProvider, FinalApproachParameterPort,
)
from navpy.modules.navigation.nav.vision_nav.law_config import FinalApproachLawConfig

LIMIT_PARAMETERS = ("PTCH_LIM_MIN_DEG", "PTCH_LIM_MAX_DEG", "ROLL_LIMIT_DEG", "PTCH2SRV_TCONST")


def read_defaults(path: Path, configured_throttle_percent: float | None = None) -> tuple[dict[str, float], dict]:
    validate_override(configured_throttle_percent)
    payload = path.read_bytes()
    parameters = parse_parm(payload.decode("utf-8"))
    trim = parameters.get("TRIM_THROTTLE")
    if trim is None or not math.isfinite(trim) or not 0 <= trim <= 100:
        raise ValueError("missing or invalid throttle configuration: TRIM_THROTTLE")
    required = (*LIMIT_PARAMETERS, "TRIM_THROTTLE")
    if any(name not in parameters for name in required):
        raise ValueError("missing approach parameter in boot defaults")
    return {name: parameters[name] for name in required}, {
        "policy": "normal-approach-config", "defaults_sha256": hashlib.sha256(payload).hexdigest(),
        "trim_throttle_percent": trim, "configured_throttle_percent": configured_throttle_percent,
    }


def resolve_config(limits: tuple[float, ...], trim_throttle_percent: float | None,
                   configured_throttle_percent: float | None = None) -> FinalApproachLawConfig:
    validate_override(configured_throttle_percent)
    if len(limits) != 5 or not all(math.isfinite(x) for x in limits):
        raise ValueError("invalid approach limits")
    throttle = limits[4]
    if throttle > 1:
        raise ValueError("invalid throttle configuration: normalized override exceeds one")
    if configured_throttle_percent is None and throttle < 0 and (trim_throttle_percent is None or
            not math.isfinite(trim_throttle_percent) or not 0 <= trim_throttle_percent <= 100):
        raise ValueError("missing or invalid throttle configuration: TRIM_THROTTLE")
    parameters = dict(zip(LIMIT_PARAMETERS, limits[:4]))
    parameters["TRIM_THROTTLE"] = trim_throttle_percent
    provider = FinalApproachAirframeConfigProvider(FinalApproachParameterPort(parameters.get),
                                              lambda: configured_throttle_percent if configured_throttle_percent is not None
                                              else (None if throttle < 0 else throttle * 100))
    config = provider.read()
    if config is None or (config.throttle is None and configured_throttle_percent != -1):
        raise ValueError("unavailable approach throttle configuration or limits")
    return config


def expected_thrust(limits: tuple[float, ...], trim_throttle_percent: float,
                    configured_throttle_percent: float | None = None) -> float | None:
    # The wire command is float32, including the provider's percent conversion.
    value = resolve_config(limits, trim_throttle_percent, configured_throttle_percent).throttle
    return None if value is None else struct.unpack("<f", struct.pack("<f", value))[0]


def validate_override(value: float | None) -> None:
    if value is not None and (isinstance(value, bool) or not math.isfinite(value) or
                              not (value == -1 or 0 <= value <= 100)):
        raise ValueError("invalid throttle configuration: use -1 to mask or 0..100 percent")


def validate_evidence(case: Path, lifecycle: Path, peer: dict, identity: dict) -> dict[str, float]:
    """Require both boot-file binding and the actual unchanged BIN parameters."""
    import json
    import os
    import subprocess
    from scripts.eval_simtime_step import wsl_path
    from scripts.noise_log_artifacts import new_log

    def launch_path(path: Path) -> str:
        return wsl_path(path) if path.drive else path.as_posix()

    def flag(command: list[str], name: str) -> str:
        if command.count(name) != 1 or command.index(name) + 1 == len(command):
            raise ValueError(f"missing or duplicate configuration flag: {name}")
        return command[command.index(name) + 1]

    peer_command = json.loads((case / "peer-command.json").read_text())
    override = float(flag(peer_command, "--throttle-percent")) if "--throttle-percent" in peer_command else None
    parameters, policy = read_defaults(case / "navigation-defaults.parm", override)
    if peer.get("version") != 4 or peer.get("throttle_policy") != policy:
        raise ValueError("missing or conflicting throttle policy evidence")
    if identity["defaults_sha256"] != policy["defaults_sha256"]:
        raise ValueError("throttle defaults identity mismatch")
    command = json.loads((lifecycle / "command.json").read_text())
    if flag(peer_command, "--defaults") != launch_path(case / "navigation-defaults.parm"):
        raise ValueError("peer throttle defaults path mismatch")
    if flag(peer_command, "--fault") != "none":
        raise ValueError("fault-injected run cannot certify throttle parity")
    # Fleets boot one shared file; every per-vehicle copy must match its bytes.
    shared = lifecycle / str(json.loads((lifecycle / "fleet.json").read_text())["shared_profile_vehicle"]) if (lifecycle / "fleet.json").exists() else case
    if (flag(command, "--defaults") != launch_path(shared / "navigation-defaults.parm") or
            hashlib.sha256((shared / "navigation-defaults.parm").read_bytes()).hexdigest() != policy["defaults_sha256"]):
        raise ValueError("SITL throttle defaults path/content mismatch")
    directory = flag(peer_command, "--directory")
    if directory != flag(command, "--sitl-root").rstrip("/") + "/" + str(_vehicle(peer)):
        raise ValueError("throttle evidence instance mismatch")
    before, after = [json.loads((case / f"logs-{when}.json").read_text()) for when in ("before", "after")]
    name = new_log(before, after)
    binding = json.loads((case / "bin-binding.json").read_text())
    if (before["directory"] != directory or binding["source"] != directory + "/logs/" + name or
            binding["sha256"] != hashlib.sha256((case / "flight.BIN").read_bytes()).hexdigest() or
            (case / "flight.BIN").stat().st_size != after["entries"][name]["size"]):
        raise ValueError("throttle BIN binding mismatch")
    # Use the recorded reader environment, as the existing noise-profile gate does.
    prefix = ["wsl.exe", "--exec"] if os.name == "nt" else []
    root = Path(__file__).resolve().parent.parent
    parsed = subprocess.run([*prefix, identity["peer_python"], launch_path(root / "scripts/read_noise_parameters.py"),
                             launch_path(case), json.dumps(parameters)], check=True,
                            capture_output=True, text=True, timeout=60)
    observed = json.loads(parsed.stdout)
    if observed["binary_sha256"] != binding["sha256"]:
        raise ValueError("throttle parameter reader used another BIN")
    return parameters


def _vehicle(peer: dict) -> int:
    from scripts.simtime_navigation_protocol import Snapshot
    return Snapshot.decode(bytes.fromhex(peer["records"][0]["snapshot"])).identity.vehicle


def validate_command(limits: tuple[float, ...], kind: int, mask: int, thrust: float, parameters: dict[str, float],
                     configured_throttle_percent: float | None = None) -> None:
    from scripts.simtime_navigation_protocol import ATTITUDE
    if limits[:4] != tuple(parameters[name] for name in LIMIT_PARAMETERS):
        raise ValueError("snapshot approach limits differ from verified boot parameters")
    expected = expected_thrust(limits, parameters["TRIM_THROTTLE"], configured_throttle_percent)
    wanted_mask, wanted_thrust = (196, 0.) if expected is None else (132, expected)
    if kind == ATTITUDE and (mask != wanted_mask or thrust != wanted_thrust):
        raise ValueError("attitude command delegates throttle or differs from configured throttle policy")
