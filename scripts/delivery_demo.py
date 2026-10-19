"""Hand-driven delivery demo: put the delivery fixture on this checkout's stack.

``launch`` starts the slot's stack with the fixture's dock world, SITL home,
settings and a fresh SITL root (no stored mission); ``prep`` then connects the
vehicles, writes their params, uploads the fixture missions with readback,
starts the companions and opens the UI. The operator presses Launch and
approves each dock. Both steps reuse the delivery eval's helpers; see
docs/design/delivery_demo_prep.md.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
import sys
import webbrowser
from collections.abc import Mapping
from pathlib import Path

_REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
for _import_root in (_REPOSITORY_ROOT, _REPOSITORY_ROOT / "src"):
    if str(_import_root) not in sys.path:
        sys.path.insert(0, str(_import_root))

from navpy.sim.world.fleet_dock_world import FLEET_DOCKS_ENV  # noqa: E402
from gcs.backend.runtime_paths import LOG_DIR_ENV as GCS_LOG_DIR_ENV  # noqa: E402
from scripts.eval_delivery_fixture import (  # noqa: E402
    DOCK_FILE_NAME,
    FIXTURE_PATH,
    delivery_scenario,
    load_fixture,
    prepare_sim_docks,
)
from scripts.eval_gcs_demo_config import build_environment, build_settings  # noqa: E402
from scripts.eval_gcs_demo_execution import SystemClock  # noqa: E402
from scripts.eval_gcs_demo_lifecycle import (  # noqa: E402
    companion_ids,
    connect_vehicles,
    start_companions,
)
from scripts.eval_gcs_demo_mission import resolve_live_plan  # noqa: E402
from scripts.eval_gcs_demo_mission_upload import upload_scenario_missions  # noqa: E402
from scripts.eval_gcs_demo_models import RegressionError, StackContext  # noqa: E402
from scripts.eval_gcs_demo_ports import ClockPort, JsonValue  # noqa: E402
from scripts.eval_gcs_demo_process import (  # noqa: E402
    PreexistingOwnedStackError,
    launch_stack,
    owned_entry,
    owned_stack_context,
)
from scripts.eval_gcs_demo_sitl_root import prepare_sitl_root  # noqa: E402
from scripts.eval_gcs_demo_stack_ops import (  # noqa: E402
    configure_vehicle_params,
    start_esp32_simulator,
    verify_aas_profile,
    wait_for_missions,
    wait_for_vehicles,
)

RUNS_DIR = _REPOSITORY_ROOT / ".logs" / "delivery-demo"
LAUNCH_RECORD_NAME = "demo_launch.json"
SETTINGS_NAME = "gcs_settings.json"
#: The eval's per-phase wall timeout (``--timeout-s`` default).
STACK_TIMEOUT_S = 600.0
#: The frontend's own variables, which the eval's allow-list drops (the map
#: token may come from the shell instead of ``.env.local``).
FRONTEND_ENV = ("VITE_CESIUM_ION_TOKEN",)


def demo_settings(fixture: Mapping[str, JsonValue], scenario) -> dict[str, JsonValue]:
    """The eval's settings, plus the fixture's home hubs for the operator's map."""
    settings = build_settings(scenario)
    settings["default_delivery_hubs"] = fixture["inputs"]["delivery_hubs"]
    return settings


def demo_environment(run_dir: Path, source: Mapping[str, str] = os.environ) -> dict[str, str]:
    env = build_environment(run_dir / SETTINGS_NAME, run_dir, base_environment=dict(source))
    env.update({key: source[key] for key in FRONTEND_ENV if key in source})
    env[FLEET_DOCKS_ENV] = str((run_dir / DOCK_FILE_NAME).resolve())
    return env


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _guarded_files(run_dir: Path) -> dict[str, Path]:
    return {
        "fixture_sha256": FIXTURE_PATH,
        "docks_sha256": run_dir / DOCK_FILE_NAME,
        "settings_sha256": run_dir / SETTINGS_NAME,
    }


def powershell_command(*paths: object, step: str = "") -> str:
    """A PowerShell 5.1 line; literal-quoted paths keep spaces, ``$`` and backticks."""
    quoted = " ".join("'" + str(path).replace("'", "''") + "'" for path in paths)
    return f"& {quoted} {step}".rstrip()


def _command(step: str) -> str:
    return powershell_command(sys.executable, Path(__file__).resolve(), step=step)


def _stop_command() -> str:
    return powershell_command(sys.executable, _REPOSITORY_ROOT / "scripts" / "gcs_stop.py")


def launch(runs_dir: Path, clock: ClockPort, now: dt.datetime) -> Path:
    """Start this checkout's stack for the fixture; return its run directory."""
    fixture = load_fixture(FIXTURE_PATH)
    scenario = delivery_scenario(fixture, FIXTURE_PATH)
    run_dir = runs_dir / now.strftime("%Y%m%d-%H%M%S")
    if run_dir.exists():
        raise RegressionError(f"run directory already exists: {run_dir}")
    prepare_sim_docks(run_dir, scenario)
    (run_dir / SETTINGS_NAME).write_text(
        json.dumps(demo_settings(fixture, scenario), indent=2), encoding="utf-8"
    )
    env = demo_environment(run_dir)
    sitl_root, sitl_section, _ = prepare_sitl_root(run_dir)
    token = os.urandom(16).hex()
    record = {"launch_token": token, "home": scenario.home, "sitl_root": sitl_section}
    record.update({key: file_sha256(path) for key, path in _guarded_files(run_dir).items()})
    # Written first: prep finds the run by this token in the registry.
    (run_dir / LAUNCH_RECORD_NAME).write_text(json.dumps(record, indent=2), encoding="utf-8")
    try:
        context = launch_stack(
            env, STACK_TIMEOUT_S, sitl_root.launch_arguments(),
            clock=clock, scenario=scenario,
            headless=False, browser=False, launch_token=token,
        )
    except PreexistingOwnedStackError as error:
        raise RegressionError(
            f"this checkout already runs a GCS stack; stop it first: {_stop_command()}"
        ) from error
    context.api.close()
    return run_dir


def find_run(runs_dir: Path, token: object) -> Path:
    for path in sorted(runs_dir.glob(f"*/{LAUNCH_RECORD_NAME}")):
        try:
            if json.loads(path.read_text(encoding="utf-8")).get("launch_token") == token:
                return path.parent
        except (OSError, ValueError, AttributeError):
            continue
    raise RegressionError(
        "this checkout's running stack was not started by 'delivery_demo.py launch'; "
        f"stop it ({_stop_command()}) and run: {_command('launch')}"
    )


def require_unchanged(run_dir: Path) -> None:
    record = json.loads((run_dir / LAUNCH_RECORD_NAME).read_text(encoding="utf-8"))
    changed = [
        str(path) for key, path in _guarded_files(run_dir).items()
        if not path.exists() or file_sha256(path) != record.get(key)
    ]
    if changed:
        raise RegressionError(
            f"changed since launch: {', '.join(changed)}; relaunch "
            f"({_stop_command()}, then {_command('launch')})"
        )


def require_backend_settings(context: StackContext, expected: Mapping[str, JsonValue]) -> None:
    """Every setting the launch wrote is still the backend's setting."""
    actual = context.api.get_json("/api/settings")
    if not isinstance(actual, dict):
        raise TypeError("settings response must be an object")
    differences = []
    for section, wanted in expected.items():
        have = actual.get(section)
        if isinstance(wanted, dict):
            have = have if isinstance(have, dict) else {}
            differences += [
                f"{section}.{key}" for key, value in wanted.items() if have.get(key) != value
            ]
        elif have != wanted:
            differences.append(section)
    if differences:
        raise RegressionError(
            f"backend settings differ from the launch: {', '.join(differences)}; relaunch"
        )


def prepare_scenario(context: StackContext, run_dir: Path) -> None:
    """The eval's run_mission steps up to the launch, on a fresh stack."""
    connected = context.api.get_json("/api/vehicles")
    if connected or companion_ids(context):
        raise RegressionError(
            f"prep already ran on this stack; relaunch ({_stop_command()}, then {_command('launch')})"
        )
    connect_vehicles(context)
    wait_for_vehicles(context, STACK_TIMEOUT_S)
    configure_vehicle_params(context, STACK_TIMEOUT_S)
    upload_scenario_missions(context)
    verify_aas_profile(context)
    start_companions(context, STACK_TIMEOUT_S)
    start_esp32_simulator(context)
    wait_for_missions(context, STACK_TIMEOUT_S)
    resolve_live_plan(context, run_dir)


def prep(runs_dir: Path, clock: ClockPort, open_browser=webbrowser.open) -> Path:
    entry = owned_entry(os.environ)
    if entry is None:
        raise RegressionError(f"no GCS stack runs for this checkout; run: {_command('launch')}")
    token = entry.get("sitl_launch_token")
    run_dir = find_run(runs_dir, token)
    require_unchanged(run_dir)
    fixture = load_fixture(FIXTURE_PATH)
    scenario = delivery_scenario(fixture, FIXTURE_PATH)
    env = demo_environment(run_dir)
    context = owned_stack_context(
        env, entry, str(token), Path(env[GCS_LOG_DIR_ENV]) / "backend.log", run_dir,
        clock=clock, scenario=scenario,
    )
    try:
        require_backend_settings(context, demo_settings(fixture, scenario))
        prepare_scenario(context, run_dir)
    finally:
        context.api.close()
    open_browser(f"http://localhost:{entry['frontend']}")
    return run_dir


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("step", choices=("launch", "prep"))
    args = parser.parse_args(argv)
    try:
        if args.step == "launch":
            run_dir = launch(RUNS_DIR, SystemClock(), dt.datetime.now())
            print(f"Stack up; logs in {run_dir}\nNext: {_command('prep')}")
        else:
            run_dir = prep(RUNS_DIR, SystemClock())
            print(
                "Ready. If a GCS tab was already open, reload it. Press Launch, "
                "then approve each dock (30 s each).\n"
                f"Logs in {run_dir}\nStop: {_stop_command()}"
            )
    except RegressionError as error:
        print(f"DELIVERY DEMO {args.step.upper()} FAILED: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
