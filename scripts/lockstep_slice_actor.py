"""The 1-UAV slice's scripted API actor and participant roles (steps 9e, 9f).

The slice is the fleet layout with N = 1 (``scripts.lockstep_fleet_actor``):
SITL 1's serial0 is NavPy's ``lockstep:serial0.1``, its serial1 the
coordinator's router endpoint, the backend owns ``lockstep:router``
(``lockstep_fleet.md``, 10c) and NavPy's role is ``navpy1`` (10f). The actor
runs on the operator's baton and makes the operator's calls through the
lockstep ports (``lockstep_slice.md``, "Operator"):

- 9e's setup (``lockstep_fleet_actor.setup_script``): connect
  ``lockstep:router``, read the command bounds by name, write the scenario's
  AAS profile, upload its mission;
- :func:`companion_script` (9f, ``scripts.lockstep_fleet_companions``): the setup, then POST start-all (the
  backend's launcher port spawns NavPy), readiness through the status route,
  then stop-all, every outcome ``STOP_ACKED`` (the fleet's companion script).

:func:`slice_roles` is the fleet's roles for the slice
(``lockstep_fleet_actor.fleet_roles``).
"""

from __future__ import annotations

from collections.abc import Callable

from navpy.sim.lockstep.coordinator_types import RoleSpec
from navpy.sim.lockstep.operator_baton import OperatorBaton
from scripts.eval_gcs_demo_config import build_settings
from scripts.eval_gcs_demo_scenario import DemoScenario
from scripts.lockstep_fleet_actor import FleetLayout, companion_argv, fleet_roles
from scripts.lockstep_fleet_companions import STATUS_ROUTE
from scripts.lockstep_fleet_companions import companion_script as fleet_companion_script

#: The slice's fleet: one UAV.
SLICE = FleetLayout(1)
#: The slice's vehicle and its participants' ordinals (0 is the operator;
#: NavPy i is sysid i, the backend N+1).
SYS_ID = 1
NAVPY, BACKEND = SLICE.navpy(SYS_ID), SLICE.backend
#: NavPy's role name in the manifest.
NAVPY_ROLE = SLICE.companion_role(SYS_ID)
#: The slice's devices (``Manifest.sitl_ports`` and ``Manifest.router``):
#: NavPy's companion link and the backend's router endpoint.
SERIAL0, ROUTER = SLICE.serial0(SYS_ID), SLICE.router
SITL_PORTS = SLICE.sitl_ports


def companion_script(scenario: DemoScenario) -> Callable[[OperatorBaton], dict]:
    """9e's setup, then start-all, readiness through status, then stop-all."""
    return fleet_companion_script(scenario, SLICE)


def slice_argv(settings: dict) -> list[str]:
    """The command line the launcher port starts the companion with."""
    return companion_argv(settings, SLICE, SYS_ID)


def slice_roles(scenario: DemoScenario, *, companion: RoleSpec | None = None,
                navpy: bool = False, pose_cadence_dir: str | None = None) -> dict[str, RoleSpec]:
    """The backend role; with ``navpy`` (or a stand-in ``companion``) the
    companion role :data:`NAVPY_ROLE` that its launcher port spawns. A
    run-relative ``pose_cadence_dir`` makes both record the pose-cadence CSVs
    there (``navpy.sim.lockstep.pose_recording``)."""
    return fleet_roles(build_settings(scenario), SLICE, navpy=navpy,
                       companion=None if companion is None else (lambda _sys_id: companion),
                       pose_cadence_dir=pose_cadence_dir)


__all__ = [
    "BACKEND", "NAVPY", "NAVPY_ROLE", "ROUTER", "SERIAL0", "SITL_PORTS", "SLICE", "STATUS_ROUTE", "SYS_ID",
    "companion_script", "slice_argv", "slice_roles",
]
