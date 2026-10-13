"""Exact settings, parameters, commands, and environment for the demo."""

from __future__ import annotations

import os
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path

from scripts.eval_gcs_demo_constants import (
    EXPECTED_UAV_COUNT,
    EXPECTED_NAV_LAST_WP_ORDINAL,
    GCS_LAUNCH,
    GCS_STOP,
    SIM_SPEEDUP,
    VISION_PROFILE,
)
from scripts.eval_gcs_demo_models import finite_number, positive_int
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_scenario import DemoMissionPlan, load_scenario_manifest
from scripts.eval_certificate_source_time import SOURCE_TIME_ENV, SOURCE_TIME_SUBDIR

from navpy.sim.sim_cv_threads import SIM_CV_THREADS_ENV
from navpy.sim.world.navigation_poi_args import encode_wp_bitmask


NAVIGATION_SPEEDUP_ENV = "GCS_SIM_NAVIGATION_SPEEDUP"
#: The only inherited variables a run sees (upper case: Windows keys are). What
#: Windows processes, WSL and the instance registry need, and nothing that can
#: change NavPy, the backend or SITL behind the manifest's back.
INHERITED_ENV = frozenset({
    "APPDATA", "COMSPEC", "GCS_INSTANCE_REGISTRY", "HOME", "HOMEDRIVE",
    "HOMEPATH", "LOCALAPPDATA", "NUMBER_OF_PROCESSORS", "PATH", "PATHEXT",
    "PROCESSOR_ARCHITECTURE", "PROGRAMDATA", "SYSTEMDRIVE", "SYSTEMROOT",
    "TEMP", "TMP", "USERPROFILE", "WINDIR",
})
INHERITED_ENV_PREFIXES = ("WSL",)
#: Pinned for every run: hash order and single-threaded numeric libraries.
PINNED_ENV = {
    "PYTHONHASHSEED": "0",
    "OMP_NUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    SIM_CV_THREADS_ENV: "1",
    "PYTHONUTF8": "1",
}
#: The persisted ESP32 endpoint is unused while the simulator runs (launch and
#: status use the simulator's own loopback port); if it were used, it is the
#: loopback discard port, never a LAN host.
UNUSED_ESP32_PORT = 9
TAKEOFF_ALTITUDE_M = 40.0


def launch_command(
    launch_token: str | None = None,
    home: str | None = None,
    sitl_arguments: Sequence[str] = (),
) -> list[str]:
    """Use only the isolated launcher; its directory owns the slot."""
    command = [sys.executable, str(GCS_LAUNCH), "--no-browser"]
    if launch_token is not None:
        if type(launch_token) is not str or not launch_token:
            raise ValueError("launch token must be a non-empty string")
        command.extend((
            "--speedup",
            str(int(SIM_SPEEDUP)),
            "--launch-token",
            launch_token,
        ))
    if home is not None:
        if type(home) is not str or not home:
            raise ValueError("home must be a non-empty LAT,LON,ALT,HEADING string")
        command.extend(("--home", home))
    command.extend(sitl_arguments)
    return command


def stop_command() -> list[str]:
    return [sys.executable, str(GCS_STOP)]


def build_settings() -> dict[str, JsonValue]:
    return {
        # Stated, not left to the model default: the mission upload builds the
        # takeoff row from it.
        "flight": {
            "uavs_per_set": EXPECTED_UAV_COUNT,
            "takeoff_altitude_m": TAKEOFF_ALTITUDE_M,
        },
        "camera": {
            "vision_profile": VISION_PROFILE,
            "vision_device": VISION_PROFILE,
            "vision_zoom": "1",
        },
        "connection": {"auto_connect": True},
        "launch": {
            "launch_type": "container",
            "esp32_host": "127.0.0.1",
            "esp32_port": UNUSED_ESP32_PORT,
            "default_channel_map": {},
            "launch_order": [],
        },
        "simulation": {
            "sim_mode": True,
            "dev_mode": True,
            "simulated_vehicle_count": EXPECTED_UAV_COUNT,
        },
    }


def _base_aas_params(poi_mask: int) -> dict[str, JsonValue]:
    return {
        "del_ctrl": 2,
        "del_dir": True,
        # The template eeprom values the demo has flown with; a fresh eeprom
        # would register the Lua defaults (1.5 and 150) instead.
        "del_p_kp": 0.35,
        "nav_min_alt": -100,
        "use_trn": True,
        "targ_wps": poi_mask,
        "targ_alt": 0.0,
        "nav_last_wp": EXPECTED_NAV_LAST_WP_ORDINAL,
        "nav_auto_cm": False,
        "nav_cm_fl": False,
        "nav_cwt": 30.0,
        "nav_oneshot": True,
    }


def aas_params_for(sys_id: int, owner_sys_id: int) -> dict[str, JsonValue]:
    """Compatibility helper derived from the checked manifest and encoder."""
    checked_sys_id = positive_int("sys_id", sys_id)
    checked_owner = positive_int("owner_sys_id", owner_sys_id)
    scenario = load_scenario_manifest()
    poi_mask = encode_wp_bitmask(list(scenario.poi_nav_waypoint_ordinals))
    return _base_aas_params(poi_mask if checked_sys_id == checked_owner else 0)


def aas_params_for_plan(sys_id: int, plan: DemoMissionPlan) -> dict[str, JsonValue]:
    checked_sys_id = positive_int("sys_id", sys_id)
    if checked_sys_id not in plan.sys_ids:
        raise ValueError(f"sys_id {checked_sys_id} is not in the resolved plan")
    return aas_params_for(checked_sys_id, plan.owner.sys_id)


def _canonical_number(value: float) -> str:
    return format(value, ".15g")


def build_environment(
    settings_path: Path,
    log_dir: Path,
    *,
    navigation_speedup: float = 0.0,
    base_environment: Mapping[str, str] | None = None,
) -> dict[str, str]:
    """Build from the allow-listed inherited variables plus the run's own."""
    speedup = finite_number("navigation_speedup", navigation_speedup)
    source = os.environ if base_environment is None else base_environment
    if not all(type(key) is str and type(value) is str for key, value in source.items()):
        raise TypeError("base_environment must contain string keys and values")
    env = {key: value for key, value in source.items() if _inherited(key)}
    env.update(PINNED_ENV)
    if speedup != 0.0:
        env[NAVIGATION_SPEEDUP_ENV] = _canonical_number(speedup)
    env["GCS_SETTINGS_PATH"] = str(settings_path.resolve())
    env["NAVPY_LOG_DIR"] = str(log_dir.resolve())
    env[SOURCE_TIME_ENV] = str((log_dir / SOURCE_TIME_SUBDIR).resolve())
    env["NAVPY_SIM_LOG_LEVEL"] = "DEBUG"
    return env


def _inherited(key: str) -> bool:
    upper = key.upper()
    return upper in INHERITED_ENV or upper.startswith(INHERITED_ENV_PREFIXES)


__all__ = [
    "NAVIGATION_SPEEDUP_ENV",
    "aas_params_for",
    "INHERITED_ENV",
    "PINNED_ENV",
    "SIM_CV_THREADS_ENV",
    "aas_params_for_plan",
    "build_environment",
    "build_settings",
    "launch_command",
    "stop_command",
]
