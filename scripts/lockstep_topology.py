"""The lockstep fleet's topology inputs (``lockstep_fleet.md``, Wall topology to match, Homes).

Wall mode launches its SITLs through the deployed ``run_swarm.sh``; lockstep
reproduces the parts that shape a run: each SITL's home (:func:`fleet_homes`,
the script's ``home_coords``), the router's target table and the firmware
FIFO capacity. :func:`topology_inputs` is the input manifest's ``topology``
section: the script's sha256, the homes, the router entry table's and
binary's sha256 and ``FIFO_CAP``. The script's hash is pinned
(:data:`RUN_SWARM_SHA256`): :func:`fleet_homes` reproduces that version, so a
fleet run refuses another (:func:`require_pinned_script`), and
``tests/scripts/test_lockstep_topology.py`` ties its fixture to the hash.
"""

from __future__ import annotations

import hashlib
import math
import shlex

from navpy.sim.lockstep.coordinator_queues import FIFO_CAP
from navpy.sim.lockstep.router_entries import TABLE_FILE, load_entry_table
from scripts import lockstep_wsl
from scripts.eval_gcs_demo_models import RegressionError
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_sitl_root import WSL_HOST_FILES

#: The wall launcher, under the WSL home (``eval_gcs_demo_sitl_root.WSL_HOST_FILES``).
RUN_SWARM_SCRIPT = WSL_HOST_FILES[2]
#: sha256 of the deployed ``run_swarm.sh`` whose topology lockstep
#: reproduces (read 2026-10-20, with the SITL feeds' router filter; ``home_coords``
#: at its lines 253-271).
RUN_SWARM_SHA256 = "381e265eca2859aca003ab52dd6c06eed7dbb89c69d0299c998f24d0ee897a98"
#: The wall eval's ``-d``: swarm_run's default vehicle spacing (``swarm_run_cli``
#: ``--dist``), which the script reads as ``DIST_M``.
WALL_SWARM_DIST_M = 50
#: ``run_swarm.sh``'s ``PER_ROW`` default: vehicles per row of the home grid.
HOMES_PER_ROW = 3
#: The script's constants: metres per degree of latitude and its pi.
_M_PER_DEG = 111111
_PI = 3.14159


def fleet_homes(base: str, count: int, dist_m: float = WALL_SWARM_DIST_M) -> tuple[str, ...]:
    """SITL 1..``count``'s ``--home``, as ``run_swarm.sh``'s ``home_coords``
    gives them from ``HOME_COORDS`` = ``base``: a grid of ``HOMES_PER_ROW``
    per row, ``dist_m`` apart, latitude and longitude printed ``%.8f``, the
    altitude as given, heading 0.0."""
    lat_text, lon_text, alt = base.split(",")[:3]
    lat, lon = float(lat_text), float(lon_text)
    lat_step = dist_m / _M_PER_DEG
    lon_step = dist_m / (_M_PER_DEG * math.cos(lat * _PI / 180))
    homes = []
    for index in range(count):
        row, col = divmod(index, HOMES_PER_ROW)
        homes.append(f"{lat + row * lat_step:.8f},{lon + col * lon_step:.8f},{alt},0.0")
    return tuple(homes)


def run_swarm_sha256() -> str:
    """The deployed script's sha256, read in WSL."""
    home = lockstep_wsl.wsl("printf %s \"$HOME\"")
    return lockstep_wsl.wsl(f"sha256sum {shlex.quote(f'{home}/{RUN_SWARM_SCRIPT}')}").split()[0]


def require_pinned_script(sha256: str) -> None:
    if sha256 != RUN_SWARM_SHA256:
        raise RegressionError(f"run_swarm.sh changed ({sha256}, pinned {RUN_SWARM_SHA256}): "
                              "re-read its topology and re-pin lockstep_topology")


def topology_inputs(homes: tuple[str, ...], script_sha256: str) -> dict[str, JsonValue]:
    """The input manifest's ``topology``: what the fleet's wiring depends on."""
    table = load_entry_table()
    return {
        "run_swarm_sha256": script_sha256,
        "homes": list(homes),
        "router_entries_sha256": hashlib.sha256(TABLE_FILE.read_bytes()).hexdigest(),
        "router_binary_sha256": table.binary_sha256,
        "fifo_cap": FIFO_CAP,
    }


__all__ = [
    "HOMES_PER_ROW", "RUN_SWARM_SCRIPT", "RUN_SWARM_SHA256", "WALL_SWARM_DIST_M", "fleet_homes",
    "require_pinned_script", "run_swarm_sha256", "topology_inputs",
]
