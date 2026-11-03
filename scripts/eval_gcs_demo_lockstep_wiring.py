"""Each gate profile's lockstep fleet wiring: its SITL homes and its lines files.

The fleet is the scenario's (``FleetLayout``: sysid i flies SITL i and NavPy i).
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from scripts import lockstep_topology
from scripts.eval_gcs_demo_fleet_gates import navpy_lines_file
from scripts.eval_gcs_demo_gate_profiles import CERTIFIED_PROFILE, OWNER_ONLY_SLICE_PROFILE
from scripts.eval_gcs_demo_models import RegressionError
from scripts.eval_gcs_demo_scenario import DemoScenario
from scripts.eval_gcs_demo_slice_gates import NAVPY_LINES_FILE
from scripts.lockstep_fleet_actor import FleetLayout


@dataclass(frozen=True)
class Wiring:
    #: SITL 1..N's ``--home`` from the scenario's.
    homes: Callable[[DemoScenario, FleetLayout], tuple[str, ...]]
    #: NavPy ``sys_id``'s lines file.
    lines_file: Callable[[int], str]


#: Gate profile -> its fleet wiring. The slice's one SITL boots at the home as
#: given and keeps its one lines file; the certified fleet's homes are
#: ``run_swarm.sh``'s grid, one lines file per UAV (``eval_gcs_demo_fleet_gates``).
WIRING: dict[str, Wiring] = {
    OWNER_ONLY_SLICE_PROFILE: Wiring(lambda scenario, _layout: (scenario.home,),
                                     lambda _sys_id: NAVPY_LINES_FILE),
    CERTIFIED_PROFILE: Wiring(lambda scenario, layout: lockstep_topology.fleet_homes(
        scenario.home, layout.size), navpy_lines_file),
}


def require_wired(scenario: DemoScenario) -> None:
    """The scenario's profile has its lockstep wiring (its shape is the loader's check)."""
    if scenario.gate_profile not in WIRING:
        raise RegressionError(
            f"--lockstep runs the profiles {sorted(WIRING)}; {scenario.path.name} has gate profile "
            f"{scenario.gate_profile!r}"
        )


def fleet_layout(scenario: DemoScenario) -> FleetLayout:
    """The scenario's fleet: one UAV per slot."""
    return FleetLayout(len(scenario.slots))


__all__ = ["WIRING", "Wiring", "fleet_layout", "require_wired"]
