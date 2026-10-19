"""Input manifest for the demo eval: what a run consumes, hashed.

Two runs can then be shown to start from identical inputs. Host-specific paths
go to a ``*_host`` sidecar. The SITL side is built in ``eval_gcs_demo_sitl_root``.
"""

from __future__ import annotations

import hashlib
import json
import math
import subprocess
from collections.abc import Iterator, Mapping
from pathlib import Path

from scripts.eval_gcs_demo_constants import ROOT, SRC
from scripts.eval_gcs_demo_models import RegressionError
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_scenario import DemoScenario
from scripts.eval_gcs_demo_sitl_root import sha256_bytes, sha256_file

from navpy.core.perception.geo.dem_data import DemData


INPUT_MANIFEST = "input_manifest.json"
INPUT_MANIFEST_HOST = "input_manifest_host.json"
INPUT_MANIFEST_SCHEMA = 1
DEM_DIR = ROOT / ".terrain" / "dem"
SIM_ASSETS_DIR = ROOT / ".sim"
VISION_PROFILES = SRC / "navpy" / "core" / "perception" / "profile" / "vision_profiles.json"
#: Every file ``SimFrameGenerator`` looks for; an absent one changes the frames.
SIM_ASSET_FILES = (
    "background.png", "dock.png", "building.png", "platform_vehicle.png",
    "antenna.png", "operations_site.png", "bridge.png", "fuel.png", "other.png",
)
#: Environment variables that name an input file the run consumes.
FILE_INPUT_ENV = ("GCS_SETTINGS_PATH", "NAVPY_SIM_DOCKS")


def _coordinates(value: JsonValue) -> Iterator[tuple[float, float]]:
    if isinstance(value, Mapping):
        lat, lon = value.get("lat"), value.get("lon")
        if isinstance(lat, (int, float)) and isinstance(lon, (int, float)):
            yield float(lat), float(lon)
        for item in value.values():
            yield from _coordinates(item)
    elif isinstance(value, list):
        for item in value:
            yield from _coordinates(item)


def dem_tiles(points: list[tuple[float, float]], directory: Path = DEM_DIR) -> dict[str, str]:
    """Hash the DEM tiles NavPy needs; a missing or unreadable tile is an input fault.

    NavPy would otherwise try a download and, if that fails, run without
    terrain (``dem_tile_repository.py``), so the run's inputs would depend on
    the network.
    """
    tiles: dict[str, str] = {}
    by_name: dict[str, list[tuple[float, float]]] = {}
    for point in points:
        by_name.setdefault(DemData.get_hgt_file_name(point), []).append(point)
    for name, covered in sorted(by_name.items()):
        path = directory / name
        if not path.is_file():
            raise RegressionError(f"input fault: DEM tile {path} is missing")
        try:
            tile = DemData(str(path))
        except (OSError, ValueError) as error:
            raise RegressionError(f"input fault: DEM tile {path} is unreadable: {error}") from error
        for point in covered:
            height = tile.get_height(point)
            if height is None or not math.isfinite(height):
                raise RegressionError(f"input fault: DEM tile {path} has no height at {point}")
        tiles[name] = sha256_file(path)
    return tiles


def scenario_points(scenario: DemoScenario) -> list[tuple[float, float]]:
    lat, lon = (float(part) for part in scenario.home.split(",")[:2])
    return [(lat, lon), *_coordinates(list(scenario.missions))]


def _optional_hash(path: Path) -> str | None:
    return sha256_file(path) if path.is_file() else None


def navpy_inputs(scenario: DemoScenario) -> dict[str, JsonValue]:
    from pymavlink import __version__ as pymavlink_version
    from pymavlink.dialects.v20 import ardupilotmega

    return {
        "dem": dict(dem_tiles(scenario_points(scenario))),
        "sim_assets": {name: _optional_hash(SIM_ASSETS_DIR / name) for name in SIM_ASSET_FILES},
        "vision_profiles_sha256": sha256_file(VISION_PROFILES),
        "mavlink_dialect": {
            "pymavlink": pymavlink_version,
            "ardupilotmega_sha256": sha256_file(Path(ardupilotmega.__file__)),
        },
    }


def _git(*args: str) -> str:
    result = subprocess.run(["git", "-C", str(ROOT), *args], capture_output=True, timeout=60)
    if result.returncode != 0:
        raise RegressionError(f"git {' '.join(args)} failed")
    return result.stdout.decode("utf-8")


def _untracked_digest() -> str:
    """Untracked, not ignored files (new sources); run directories are ignored."""
    listing = _git("ls-files", "--others", "--exclude-standard", "-z")
    names = sorted(name for name in listing.split("\0") if name)
    digest = hashlib.sha256()
    for name in names:
        path = ROOT / name
        content = sha256_file(path) if path.is_file() else "absent"
        digest.update(f"{name}\0{content}\n".encode("utf-8"))
    return digest.hexdigest()


def environment_inputs(env: Mapping[str, str], run_dir: Path) -> dict[str, JsonValue]:
    """The child environment, with this run's directory written as ``<run_dir>``."""
    prefix = str(run_dir.resolve())
    entries: dict[str, JsonValue] = {
        key: value.replace(prefix, "<run_dir>") for key, value in sorted(env.items())
    }
    files: dict[str, JsonValue] = {}
    for key in FILE_INPUT_ENV:
        if key in env:
            files[key] = _optional_hash(Path(env[key]))
    return {"variables": entries, "files": files}


def write_input_manifest(
    run_dir: Path,
    env: Mapping[str, str],
    sitl: Mapping[str, JsonValue],
    sitl_root_host: str,
    run_options: Mapping[str, JsonValue],
    scenario: DemoScenario,
) -> dict[str, JsonValue]:
    diff = _git("diff", "HEAD", "--binary")
    manifest: dict[str, JsonValue] = {
        "schema": INPUT_MANIFEST_SCHEMA,
        "source": {
            "git_head": _git("rev-parse", "HEAD").strip(),
            "uncommitted_diff_sha256": sha256_bytes(diff.encode("utf-8")),
            "untracked_sha256": _untracked_digest(),
        },
        "run_options": dict(run_options),
        "scenario_sha256": sha256_file(scenario.path),
        "sitl": dict(sitl),
        "navpy": navpy_inputs(scenario),
        "environment": environment_inputs(env, run_dir),
    }
    (run_dir / INPUT_MANIFEST).write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (run_dir / INPUT_MANIFEST_HOST).write_text(
        json.dumps(
            {"run_dir": str(run_dir.resolve()), "sitl_root": sitl_root_host},
            indent=2,
            sort_keys=True,
        ) + "\n",
        encoding="utf-8",
    )
    return manifest


def recorded_scenario_sha256(run_dir: Path) -> str | None:
    """The scenario hash a run recorded, or None for a run without a manifest."""
    path = run_dir / INPUT_MANIFEST
    if not path.is_file():
        return None
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RegressionError(f"invalid input manifest {path}: {error}") from error
    recorded = manifest.get("scenario_sha256") if isinstance(manifest, dict) else None
    if type(recorded) is not str or not recorded:
        raise RegressionError(f"input manifest {path} records no scenario_sha256")
    return recorded


def recorded_lockstep(run_dir: Path) -> bool:
    """Whether the run's input manifest records a lockstep run (``run_options.lockstep``)."""
    path = run_dir / INPUT_MANIFEST
    if not path.is_file():
        return False
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RegressionError(f"invalid input manifest {path}: {error}") from error
    options = manifest.get("run_options") if isinstance(manifest, dict) else None
    return isinstance(options, dict) and options.get("lockstep") is True


def recorded_profile_parameters(run_dir: Path) -> dict[str, float]:
    """The SITL parameters a lockstep run's sim profile set (``sitl.profile_parameters``);
    empty for a run without a manifest or without them."""
    path = run_dir / INPUT_MANIFEST
    if not path.is_file():
        return {}
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RegressionError(f"invalid input manifest {path}: {error}") from error
    sitl = manifest.get("sitl") if isinstance(manifest, dict) else None
    parameters = sitl.get("profile_parameters", {}) if isinstance(sitl, dict) else {}
    if not isinstance(parameters, dict) or not all(
            isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
            for value in parameters.values()):
        raise RegressionError(f"input manifest {path} has invalid profile_parameters")
    return {str(name): float(value) for name, value in parameters.items()}


def require_recorded_scenario(run_dir: Path, scenario: DemoScenario) -> None:
    """Refuse to analyze a run against a scenario other than the one it flew."""
    recorded = recorded_scenario_sha256(run_dir)
    if recorded is None:
        return
    actual = sha256_file(scenario.path)
    if actual != recorded:
        raise RegressionError(
            f"scenario {scenario.path} (sha256 {actual}) is not the run's recorded "
            f"scenario (sha256 {recorded})"
        )


__all__ = [
    "INPUT_MANIFEST",
    "INPUT_MANIFEST_HOST",
    "dem_tiles",
    "environment_inputs",
    "navpy_inputs",
    "recorded_lockstep",
    "recorded_profile_parameters",
    "recorded_scenario_sha256",
    "require_recorded_scenario",
    "scenario_points",
    "write_input_manifest",
]
