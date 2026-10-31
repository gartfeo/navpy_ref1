"""Hand-driven delivery demo: a normal GCS run set up for the delivery scenario.

Starts this checkout's stack like ``AAS_GCS.bat`` (auto-connect, ``auto``
companion lifecycle, frontend) with only what the scenario needs: a run-local
SITL root at the fixture home, scenario settings pins and the vehicles' static
AAS params. The operator then loads ``scripts/delivery_demo_plan.json`` (or
builds a plan) in the UI, uploads it and presses Launch. See
docs/design/delivery_demo_prep.md, "Normal-run demo".
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import sys
import urllib.request
import webbrowser
from collections.abc import Callable, Mapping
from pathlib import Path

_REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
for _import_root in (_REPOSITORY_ROOT, _REPOSITORY_ROOT / "src"):
    if str(_import_root) not in sys.path:
        sys.path.insert(0, str(_import_root))

from scripts.delivery_demo_plan import PLAN_PATH  # noqa: E402
from scripts.eval_delivery_fixture import FIXTURE_PATH, delivery_scenario, load_fixture  # noqa: E402
from scripts.eval_gcs_demo_config import (  # noqa: E402
    TAKEOFF_ALTITUDE_M,
    UNUSED_ESP32_PORT,
    build_environment,
)
from scripts.eval_gcs_demo_execution import SystemClock, wait_until  # noqa: E402
from scripts.eval_gcs_demo_models import RegressionError  # noqa: E402
from scripts.eval_gcs_demo_ports import ClockPort, JsonValue  # noqa: E402
from scripts.eval_gcs_demo_process import (  # noqa: E402
    PreexistingOwnedStackError,
    launch_stack,
    owned_entry,
)
from scripts.eval_gcs_demo_sitl_root import prepare_sitl_root  # noqa: E402
from scripts.eval_gcs_demo_stack_ops import (  # noqa: E402
    configure_vehicle_params,
    verify_aas_profile,
    wait_for_vehicles,
)
from scripts.eval_certificate_source_time import SOURCE_TIME_ENV  # noqa: E402

from gcs.backend.diagnostics import SESSION_ID_ENV as DIAGNOSTICS_SESSION_ID_ENV  # noqa: E402
from gcs.backend.runtime_paths import LOG_DIR_ENV as GCS_LOG_DIR_ENV  # noqa: E402
from navpy.core.kernel.logging.pose_cadence_naming import NAMING_ENV as POSE_CADENCE_NAMING_ENV  # noqa: E402

RUNS_DIR = _REPOSITORY_ROOT / ".logs" / "delivery-demo"
#: The settings a normal run of this checkout reads (settings_store default).
OWNER_SETTINGS_PATH = _REPOSITORY_ROOT / "gcs_settings.json"
SETTINGS_NAME = "gcs_settings.json"
#: The eval's per-phase wall timeout (``--timeout-s`` default).
STACK_TIMEOUT_S = 600.0
#: Vite's cold start; the launcher does not wait for the frontend.
FRONTEND_TIMEOUT_S = 60.0
#: A shared dock world replaces the waypoint docks the plan marks.
SIM_DOCKS_ENV = "NAVPY_SIM_DOCKS"
#: The demo's CPU share (``bounded_cpu_share.md``): long enough to fly by
#: hand, bounded so a forgotten stack gives the cores back.
CPU_SHARE_DEFAULT_MIN = 30
CPU_SHARE_MAX_MIN = 120
#: Run paths from the eval's environment: settings and logs only.
RUN_ENV = (
    "GCS_SETTINGS_PATH", "NAVPY_LOG_DIR", GCS_LOG_DIR_ENV, SOURCE_TIME_ENV,
    POSE_CADENCE_NAMING_ENV, DIAGNOSTICS_SESSION_ID_ENV,
)


def scenario_pins(fixture: Mapping[str, JsonValue]) -> dict[str, dict[str, JsonValue]]:
    """The settings the scenario needs over the owner's normal settings."""
    inputs = fixture["inputs"]
    count = inputs["uav_count"]
    return {
        "flight": {
            "uavs_per_set": count,
            "takeoff_altitude_m": TAKEOFF_ALTITUDE_M,
            "altitude_separation_m": inputs["altitude_separation_m"],
        },
        "connection": {"auto_connect": True},
        # Container launch, as the scenario was flown; it also starts the
        # ESP32 sim. The persisted endpoint stays off the LAN.
        "launch": {
            "launch_type": "container",
            "esp32_host": "127.0.0.1",
            "esp32_port": UNUSED_ESP32_PORT,
        },
        "simulation": {
            "sim_mode": True,
            "simulated_vehicle_count": count,
            "companion_lifecycle": "auto",
        },
    }


def demo_settings(owner: Mapping[str, JsonValue], fixture: Mapping[str, JsonValue]) -> dict[str, JsonValue]:
    settings = json.loads(json.dumps(owner))
    for section, values in scenario_pins(fixture).items():
        current = settings.get(section)
        settings[section] = {**(current if isinstance(current, dict) else {}), **values}
    return settings


def owner_settings(path: Path = OWNER_SETTINGS_PATH) -> dict[str, JsonValue]:
    if not path.exists():
        return {}
    try:
        # utf-8-sig, as settings_store reads it: PowerShell 5.1 writes a BOM.
        data = json.loads(path.read_text(encoding="utf-8-sig"))
    except (OSError, ValueError) as error:
        raise RegressionError(f"cannot read the normal settings {path}: {error}") from error
    if not isinstance(data, dict):
        raise RegressionError(f"the normal settings {path} are not an object")
    return data


def demo_environment(run_dir: Path, source: Mapping[str, str] = os.environ) -> dict[str, str]:
    """The shell's environment, as a normal run gets it, plus the run's paths."""
    run = build_environment(run_dir / SETTINGS_NAME, run_dir, base_environment=dict(source))
    env = {key: value for key, value in source.items() if key.upper() != SIM_DOCKS_ENV}
    env.update({key: run[key] for key in RUN_ENV})
    return env


def powershell_command(*paths: object) -> str:
    """A PowerShell 5.1 line; literal-quoted paths keep spaces, ``$`` and backticks."""
    return "& " + " ".join("'" + str(path).replace("'", "''") + "'" for path in paths)


def _stop_command() -> str:
    return powershell_command(sys.executable, _REPOSITORY_ROOT / "scripts" / "gcs_stop.py")


def start(
    runs_dir: Path, clock: ClockPort, now: dt.datetime,
    open_browser: Callable[[str], object] = webbrowser.open,
    cpu_share_min: float = CPU_SHARE_DEFAULT_MIN,
) -> Path:
    """Start the demo stack with a CPU share and open the UI; return the run directory."""
    fixture = load_fixture(FIXTURE_PATH)
    scenario = delivery_scenario(fixture, FIXTURE_PATH)
    run_dir = runs_dir / now.strftime("%Y%m%d-%H%M%S")
    if run_dir.exists():
        raise RegressionError(f"run directory already exists: {run_dir}")
    run_dir.mkdir(parents=True)
    (run_dir / SETTINGS_NAME).write_text(
        json.dumps(demo_settings(owner_settings(), fixture), indent=2), encoding="utf-8"
    )
    env = demo_environment(run_dir)
    sitl_root, _, _ = prepare_sitl_root(run_dir)
    try:
        context = launch_stack(
            env, STACK_TIMEOUT_S, sitl_root.launch_arguments(),
            clock=clock, scenario=scenario,
            headless=False, browser=False, launch_token=os.urandom(16).hex(),
            cpu_share_s=cpu_share_min * 60.0,
        )
    except PreexistingOwnedStackError as error:
        raise RegressionError(
            f"this checkout already runs a GCS stack; stop it first: {_stop_command()}"
        ) from error
    try:
        wait_for_frontend(owned_entry(env), clock)
        # Auto-connect connects the vehicles and starts NavPy; the plan
        # upload's companion restart applies these params.
        wait_for_vehicles(context, STACK_TIMEOUT_S)
        configure_vehicle_params(context, STACK_TIMEOUT_S)
        verify_aas_profile(context)
    except RegressionError as error:
        raise RegressionError(f"{error}; stop the stack: {_stop_command()}") from error
    finally:
        context.api.close()
    open_browser(f"http://localhost:{owned_entry(env)['frontend']}")
    return run_dir


def frontend_answers(url: str) -> bool:
    with urllib.request.urlopen(url, timeout=5.0) as response:
        return response.status == 200


def wait_for_frontend(
    entry: Mapping[str, JsonValue], clock: ClockPort,
    answers: Callable[[str], bool] = frontend_answers,
) -> None:
    """The UI the operator will open is served (a dead ``npm run dev`` logs nothing)."""
    url = f"http://localhost:{entry['frontend']}"
    try:
        wait_until("GCS frontend", FRONTEND_TIMEOUT_S, lambda: answers(url) or None,
                   transient_exceptions=(OSError,), clock=clock)
    except RegressionError as error:
        raise RegressionError(
            f"the frontend at {url} does not answer (missing src/gcs/frontend/node_modules? "
            "run npm install there)"
        ) from error


def _share_minutes(text: str) -> float:
    minutes = float(text)
    if not 0 < minutes <= CPU_SHARE_MAX_MIN:
        raise argparse.ArgumentTypeError(f"must be in (0, {CPU_SHARE_MAX_MIN}] minutes")
    return minutes


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--cpu-share-min", type=_share_minutes, default=CPU_SHARE_DEFAULT_MIN,
        help=f"how long the stack holds its CPU share (default {CPU_SHARE_DEFAULT_MIN}, "
             f"at most {CPU_SHARE_MAX_MIN}); afterwards it runs as background",
    )
    args = parser.parse_args(argv)
    try:
        run_dir = start(RUNS_DIR, SystemClock(), dt.datetime.now(),
                        cpu_share_min=args.cpu_share_min)
    except RegressionError as error:
        print(f"DELIVERY DEMO FAILED: {error}", file=sys.stderr)
        return 2
    print(
        "Ready. In the GCS: Load plan -> "
        f"{PLAN_PATH} (or build your own), Upload, then Launch and approve each dock.\n"
        f"Logs in {run_dir}\nStop: {_stop_command()}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
