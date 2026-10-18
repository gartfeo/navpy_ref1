"""The delivery fixture as a wall-eval scenario (e4).

``scripts/eval_delivery_scenario.json`` (made by ``eval_delivery_scenario_gen.py``)
holds three GCS-planned missions, the shared dock world and the expected
outcome. ``eval_gcs_navigation_demo.py --scenario delivery`` flies it: one
slot per zone in zone order, the offerer's zone in the owner slot, no
waypoint docks (``targ_wps=0``), and the fixture's ``docks`` block as the run's
``NAVPY_SIM_DOCKS`` file. Who wins the offered dock and who goes home is
decided by the run (``eval_delivery_roles.py``), never by the fixture.

See docs/design/delivery_eval_scenario.md, "Remaining plan", step e4.
"""

from __future__ import annotations

import json
from collections.abc import Mapping
from pathlib import Path
from typing import cast

from navpy.sim.world.fleet_dock_world import parse_fleet_docks
from scripts.eval_gcs_demo_config import aas_params_for, build_settings
from scripts.eval_gcs_demo_constants import EXPECTED_NAV_LAST_WP_ORDINAL
from scripts.eval_gcs_demo_gate_profiles import DELIVERY_PROFILE, require_profile_shape
from scripts.eval_gcs_demo_models import RegressionError
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_scenario import (
    SCENARIO_SCHEMA_VERSION,
    DemoScenario,
    ScenarioSlot,
    load_scenario,
)
from scripts.eval_gcs_demo_scenario_missions import parse_home, scenario_missions

_REPO = Path(__file__).resolve().parents[1]
FIXTURE_PATH = _REPO / "scripts" / "eval_delivery_scenario.json"
#: ``--scenario delivery`` names the fixture.
DELIVERY_SCENARIO_NAME = "delivery"
DOCK_FILE_NAME = "fleet_docks.json"
_FIXTURE_KEYS = {
    "docks", "expected", "generator", "inputs", "missions", "plan", "runtime",
    "schema_version",
}


def zone_slot(zone_index: int) -> str:
    return f"zone-{zone_index}"


def load_fixture(path: Path) -> dict[str, JsonValue]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid delivery fixture {path}: {error}") from error
    if not isinstance(payload, dict) or set(payload) != _FIXTURE_KEYS:
        raise ValueError(f"delivery fixture {path} must hold exactly {sorted(_FIXTURE_KEYS)}")
    return cast(dict[str, JsonValue], payload)


def is_delivery_fixture(path: Path) -> bool:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    return isinstance(payload, dict) and set(payload) == _FIXTURE_KEYS


def delivery_scenario(fixture: Mapping[str, JsonValue], path: Path) -> DemoScenario:
    """Three slots in zone order; the offerer's zone is the owner slot."""
    missions = cast(list[JsonValue], fixture["missions"])
    expected = cast(Mapping[str, JsonValue], fixture["expected"])
    offerer = cast(int, expected["offerer_zone_index"])
    if not 0 <= offerer < len(missions):
        raise ValueError(f"offerer zone {offerer} is not a fixture zone")
    slots = tuple(
        ScenarioSlot(zone_slot(index), "owner" if index == offerer else "peer")
        for index in range(len(missions))
    )
    names = [slot.name for slot in slots]
    profile = require_profile_shape(DELIVERY_PROFILE, [slot.role for slot in slots], ())
    home = cast(Mapping[str, float], cast(Mapping[str, JsonValue], fixture["inputs"])["home"])
    return DemoScenario(
        SCENARIO_SCHEMA_VERSION,
        profile,
        slots,
        zone_slot(offerer),
        (),
        parse_home(f"{home['lat']},{home['lon']},{home['alt_m']},0.0"),
        scenario_missions(missions, names, zone_slot(offerer), ()),
        path,
    )


def load_delivery_scenario(path: Path) -> DemoScenario:
    return delivery_scenario(load_fixture(path), path)


def scenario_path(value: str) -> Path:
    """``--scenario``: a scenario file, or ``delivery`` for the fixture."""
    return FIXTURE_PATH if value == DELIVERY_SCENARIO_NAME else Path(value)


def load_eval_scenario(path: Path) -> DemoScenario:
    """A scenario manifest, or the delivery fixture."""
    if is_delivery_fixture(path):
        return load_delivery_scenario(path)
    return load_scenario(path)


def prepare_sim_docks(run_dir: Path, scenario: DemoScenario) -> Path | None:
    """The delivery run's dock world file, after its runtime check; else None."""
    if scenario.gate_profile != DELIVERY_PROFILE:
        return None
    fixture = load_fixture(scenario.path)
    require_fixture_runtime(fixture, scenario)
    run_dir.mkdir(parents=True, exist_ok=True)
    if any(run_dir.glob("uav_*_navigation.log")):
        raise RegressionError(f"run directory already contains navigation logs: {run_dir}")
    return write_dock_file(fixture, run_dir)[0]


def require_fixture_runtime(fixture: Mapping[str, JsonValue], scenario: DemoScenario) -> None:
    """The settings the fixture's oracle assumes are the ones every UAV gets."""
    zoom = build_settings(scenario)["camera"]["vision_zoom"]
    # A zoom profile key such as "1"; anything but a whole number fails the match.
    zoom = int(zoom) if zoom.isdigit() else zoom
    owner = scenario.owner_index + 1
    for sys_id in range(1, len(scenario.slots) + 1):
        aas = aas_params_for(scenario, sys_id, owner)
        applied = {
            "nav_last_wp": aas["nav_last_wp"],
            "nav_oneshot": aas["nav_oneshot"],
            "search_zoom": zoom,
        }
        if fixture["runtime"] != applied or aas["targ_wps"] != 0:
            raise RegressionError(
                f"fixture runtime {fixture['runtime']} != applied {applied} "
                f"(slot {sys_id}, targ_wps={aas['targ_wps']})"
            )
        if applied["nav_last_wp"] != EXPECTED_NAV_LAST_WP_ORDINAL:
            raise RegressionError("nav_last_wp drifted from the eval constant")


def write_dock_file(fixture: Mapping[str, JsonValue], run_dir: Path) -> tuple[Path, list[str]]:
    """The fixture's dock world as the run's ``NAVPY_SIM_DOCKS`` file.

    A detection's obj_id indexes the returned names (file order)."""
    docks = parse_fleet_docks(fixture["docks"])
    path = run_dir / DOCK_FILE_NAME
    path.write_text(json.dumps(fixture["docks"], indent=2, sort_keys=True), encoding="utf-8")
    return path, [dock.name for dock in docks]


__all__ = [
    "DELIVERY_SCENARIO_NAME",
    "DOCK_FILE_NAME",
    "FIXTURE_PATH",
    "delivery_scenario",
    "is_delivery_fixture",
    "load_delivery_scenario",
    "load_eval_scenario",
    "load_fixture",
    "prepare_sim_docks",
    "scenario_path",
    "require_fixture_runtime",
    "write_dock_file",
    "zone_slot",
]
