"""Where an owned stack's identity lives: its run logs and its flown sysids."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path

from scripts.eval_gcs_demo_models import RegressionError, UavIds
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_scenario import DemoScenario

from gcs.backend import instance_ports
from gcs.backend.runtime_paths import LOG_DIR_ENV

#: The run directory NavPy writes its logs into (build_environment sets it).
NAVPY_LOG_DIR_ENV = "NAVPY_LOG_DIR"


def flown_sys_ids(entry: Mapping[str, JsonValue], scenario: DemoScenario) -> tuple[int, ...]:
    """The scenario's sysids: the first one per slot of the slot's reservation.

    The registry always lists the slot's whole reservation; ``gcs_launch -n``
    flies its leading sysids (gcs_launch.py, ``flying``).
    """
    reserved = reserved_sys_ids(entry)
    return scenario.uav_ids(reserved.values[: len(scenario.slots)]).values


def reserved_sys_ids(entry: Mapping[str, JsonValue]) -> UavIds:
    """The slot's whole sysid reservation, as the registry lists it."""
    return UavIds.from_values(
        entry.get("sysids", ()),  # type: ignore[arg-type]
        instance_ports.VEHICLES_PER_CHAT,
    )


def navigation_log_dir(env: Mapping[str, str]) -> Path:
    """The run directory NavPy logs into; its navigation logs are the tails."""
    log_dir = env.get(NAVPY_LOG_DIR_ENV, "").strip()
    if not log_dir:
        raise RegressionError(f"the run environment sets no {NAVPY_LOG_DIR_ENV}")
    return Path(log_dir)


def backend_log_file(env: Mapping[str, str]) -> Path:
    """The run's own backend log; a second launch into the same run is refused."""
    log_dir = env.get(LOG_DIR_ENV, "").strip()
    if not log_dir:
        raise RegressionError(f"the run environment sets no {LOG_DIR_ENV}")
    path = Path(log_dir) / "backend.log"
    if path.exists():
        raise RegressionError(f"the run already has a backend log: {path}")
    return path


__all__ = [
    "NAVPY_LOG_DIR_ENV",
    "backend_log_file",
    "flown_sys_ids",
    "navigation_log_dir",
    "reserved_sys_ids",
]
