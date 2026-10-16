"""e2b: one-UAV wall slice of the delivery scenario.

UAV A flies its zone of ``scripts/eval_delivery_scenario.json`` alone, with
``NAVPY_SIM_DOCKS`` naming the fixture's dock world. It reuses the wall eval's
launch path (isolated launcher, own slot, mission upload with readback) and
never approves. The gate (``eval_delivery_slice_gates.py``) reads the NavPy
log: A selects d1 and asks for approval, and offers d2 inside that window.

    python scripts/eval_delivery_slice.py --run-dir .logs/e2b-1

Sim evidence only: it shows neither physical docking nor cargo receipt.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Mapping, Sequence

_REPO = Path(__file__).resolve().parents[1]
for _path in (_REPO, _REPO / "src"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from navpy.sim.world.fleet_dock_world import FLEET_DOCKS_ENV, parse_fleet_docks  # noqa: E402
from scripts.eval_delivery_slice_gates import SliceResult, evaluate_slice  # noqa: E402
from scripts.eval_gcs_demo_api import navigation_log_path  # noqa: E402
from scripts.eval_gcs_demo_config import (  # noqa: E402
    aas_params_for, build_environment, build_settings,
)
from scripts.eval_gcs_demo_constants import EXPECTED_NAV_LAST_WP_ORDINAL  # noqa: E402
from scripts.eval_gcs_demo_inputs import dem_tiles, scenario_points, write_input_manifest  # noqa: E402
from scripts.eval_gcs_demo_lifecycle import connect_vehicles, start_companions  # noqa: E402
from scripts.eval_gcs_demo_mission import prepare_container_launch, trigger_all  # noqa: E402
from scripts.eval_gcs_demo_mission_upload import upload_scenario_missions  # noqa: E402
from scripts.eval_gcs_demo_models import RegressionError, StackContext  # noqa: E402
from scripts.eval_gcs_demo_process import (  # noqa: E402
    SystemClock, exclusive_evaluator_lock, with_owned_stack,
)
from scripts.eval_gcs_demo_scenario import DemoScenario, ScenarioSlot  # noqa: E402
from scripts.eval_gcs_demo_sitl_root import prepare_sitl_root  # noqa: E402
from scripts.eval_gcs_demo_stack_ops import (  # noqa: E402
    configure_vehicle_params, start_esp32_simulator, verify_aas_profile,
    wait_for_missions, wait_for_vehicles,
)

FIXTURE_PATH = _REPO / "scripts" / "eval_delivery_scenario.json"
GATE_PROFILE = "delivery_offer_slice"
POLL_S = 2.0


def slice_scenario(fixture: Mapping, path: Path) -> DemoScenario:
    """The offerer's zone as a one-slot scenario; no waypoint sim docks (targ_wps=0)."""
    zone = fixture["expected"]["offerer_zone_index"]
    home = fixture["inputs"]["home"]
    return DemoScenario(
        schema_version=2,
        gate_profile=GATE_PROFILE,
        slots=(ScenarioSlot("owner", "owner"),),
        owner_slot="owner",
        poi_nav_waypoint_ordinals=(),
        home=f"{home['lat']},{home['lon']},{home['alt_m']},0.0",
        missions=(fixture["missions"][zone],),
        path=path,
    )


def require_fixture_runtime(fixture: Mapping, scenario: DemoScenario) -> None:
    """The settings the fixture's oracle assumes are the ones this launch applies."""
    aas = aas_params_for(scenario, 1, 1)
    applied = {
        "nav_last_wp": aas["nav_last_wp"],
        "nav_oneshot": aas["nav_oneshot"],
        "search_zoom": int(build_settings(scenario)["camera"]["vision_zoom"]),
    }
    if fixture["runtime"] != applied or aas["targ_wps"] != 0:
        raise RegressionError(f"fixture runtime {fixture['runtime']} != applied {applied}")
    if applied["nav_last_wp"] != EXPECTED_NAV_LAST_WP_ORDINAL:
        raise RegressionError("nav_last_wp drifted from the eval constant")


def write_dock_file(fixture: Mapping, run_dir: Path) -> tuple[Path, list[str]]:
    docks = parse_fleet_docks(fixture["docks"])
    path = run_dir / "fleet_docks.json"
    path.write_text(json.dumps(fixture["docks"], indent=2, sort_keys=True), encoding="utf-8")
    return path, [dock.name for dock in docks]


def _watch(context: StackContext, docks: list[str], kept: str, offered: str,
           timeout_s: float) -> None:
    """Fly until the kept dock's approval window ends or the timeout; never approve."""
    sys_id = context.sys_ids[0]
    tail = context.navigation_tail(sys_id)
    deadline = context.clock.monotonic() + timeout_s
    while context.clock.monotonic() < deadline:
        tail.read_new()
        result = evaluate_slice(tail.text.splitlines(), docks, kept, offered)
        if result.evidence.get("window_end", "open at log end") != "open at log end":
            return
        context.clock.sleep(POLL_S)
    print(f"timeout after {timeout_s:.0f}s with the approval window still open", flush=True)


def _fly(context: StackContext, timeout_s: float, docks: list[str], kept: str,
         offered: str) -> None:
    if len(context.sys_ids) != 1:
        raise RegressionError(f"the slice flies one vehicle, got {context.sys_ids}")
    connect_vehicles(context)
    wait_for_vehicles(context, timeout_s)
    configure_vehicle_params(context, timeout_s)
    upload_scenario_missions(context)
    verify_aas_profile(context)
    start_companions(context, timeout_s)
    start_esp32_simulator(context)
    wait_for_missions(context, timeout_s)
    prepare_container_launch(context, min(timeout_s, 180.0))
    trigger_all(context)
    _watch(context, docks, kept, offered, timeout_s)


def run_slice(run_dir: Path, timeout_s: float, fixture_path: Path = FIXTURE_PATH) -> SliceResult:
    fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
    scenario = slice_scenario(fixture, fixture_path)
    require_fixture_runtime(fixture, scenario)
    run_dir.mkdir(parents=True, exist_ok=True)
    if any(run_dir.glob("uav_*_navigation.log")):
        raise RegressionError(f"run directory already contains navigation logs: {run_dir}")
    dem_tiles(scenario_points(scenario))
    dock_path, docks = write_dock_file(fixture, run_dir)
    settings_path = run_dir / "gcs_settings.json"
    settings_path.write_text(json.dumps(build_settings(scenario), indent=2), encoding="utf-8")
    env = build_environment(settings_path, run_dir)
    env[FLEET_DOCKS_ENV] = str(dock_path.resolve())
    sitl_root, sitl_inputs, sitl_root_host = prepare_sitl_root(run_dir)
    write_input_manifest(run_dir, env, sitl_inputs, sitl_root_host,
                         {"timeout_s": timeout_s, "slice": GATE_PROFILE}, scenario)
    kept, offered = fixture["expected"]["kept_dock"], fixture["expected"]["offered_dock"]
    sys_ids: list[int] = []

    def action(context: StackContext) -> None:
        sys_ids.extend(context.sys_ids)
        _fly(context, timeout_s, docks, kept, offered)

    with_owned_stack(env, timeout_s, action, sitl_root.launch_arguments(),
                     clock=SystemClock(), scenario=scenario)
    # The full log, read after the stack stopped: "unanswered" holds through shutdown.
    log = navigation_log_path(run_dir, sys_ids[0]).read_text(encoding="utf-8", errors="replace")
    result = evaluate_slice(log.splitlines(), docks, kept, offered)
    (run_dir / "slice_report.json").write_text(
        json.dumps({"sys_id": sys_ids[0], **asdict(result)}, indent=2), encoding="utf-8")
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--timeout-s", type=float, default=900.0)
    args = parser.parse_args(argv)
    with exclusive_evaluator_lock():
        result = run_slice(args.run_dir, args.timeout_s)
    print(json.dumps(asdict(result), indent=2))
    return 0 if result.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
