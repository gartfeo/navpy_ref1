"""The 1-UAV slice's scripted API actor and participant roles (steps 9e, 9f).

The slice is the fleet layout with N = 1 (``scripts.lockstep_fleet_actor``):
SITL 1's serial0 is NavPy's ``lockstep:serial0.1``, its serial1 the
coordinator's router endpoint, and the backend owns ``lockstep:router``
(``lockstep_fleet.md``, 10c). The actor runs on the operator's baton and
makes the operator's calls through the lockstep ports (``lockstep_slice.md``,
"Operator"):

- 9e's setup (``lockstep_fleet_actor.setup_script``): connect
  ``lockstep:router``, read the command bounds by name, write the scenario's
  AAS profile, upload its mission;
- :func:`companion_script` (9f): the setup, then POST start-all (the
  backend's launcher port spawns NavPy), readiness through the status route,
  then stop-all, every outcome ``STOP_ACKED``.

:func:`slice_roles` builds the backend role and, with ``navpy``, the NavPy
companion role: one command line, :func:`companion_argv`, in both the
backend's launcher config and NavPy's spawn record. Each role names its
device in its config too (``ParticipantContext.device``).
"""

from __future__ import annotations

from collections.abc import Callable

from gcs.backend.navpy_process_launch import lockstep_navpy_argv
from navpy.sim.lockstep.coordinator_types import RoleSpec
from navpy.sim.lockstep.operator_baton import OperatorBaton
from navpy.sim.lockstep.pose_recording import POSE_CADENCE_DIR
from scripts.eval_gcs_demo_config import build_settings
from scripts.eval_gcs_demo_execution import wait_until
from scripts.eval_gcs_demo_lifecycle import start_companions, stop_companions
from scripts.eval_gcs_demo_models import StackContext
from scripts.eval_gcs_demo_scenario import DemoScenario
from scripts.lockstep_fleet_actor import NAVPY_HOOKS, FleetLayout, backend_role, role, setup_script, stack_context

#: The slice's fleet: one UAV.
SLICE = FleetLayout(1)
#: The slice's vehicle and its participants' ordinals (0 is the operator;
#: NavPy i is sysid i, the backend N+1).
SYS_ID = 1
NAVPY, BACKEND = SLICE.navpy(SYS_ID), SLICE.backend
#: The slice's devices (``Manifest.sitl_ports`` and ``Manifest.router``):
#: NavPy's companion link and the backend's router endpoint.
SERIAL0, ROUTER = SLICE.serial0(SYS_ID), SLICE.router
SITL_PORTS = SLICE.sitl_ports
STATUS_ROUTE = "/api/control/navpy-sim/status"
#: Sim-time budgets of the actor's companion waits (the boot runs on ticks).
START_TIMEOUT_S = 30.0
READY_TIMEOUT_S = 60.0


def _ready_companion(context: StackContext) -> dict | None:
    (status,) = context.api.get_json(STATUS_ROUTE)["instances"]
    return status if status["ready"] else None


def companion_script(scenario: DemoScenario) -> Callable[[OperatorBaton], dict]:
    """9e's setup, then start-all, readiness through status, then stop-all."""
    setup = setup_script(scenario, SLICE)

    def script(operator: OperatorBaton) -> dict:
        result = setup(operator)
        context = stack_context(operator, scenario, SLICE)
        start_companions(context, START_TIMEOUT_S)
        ready = wait_until("the companion ready", READY_TIMEOUT_S,
                           lambda: _ready_companion(context), clock=context.clock)
        stop_companions(context)  # every outcome must be STOP_ACKED
        return {**result, "ready": ready, "stopped": context.api.get_json(STATUS_ROUTE)}

    return script


def companion_argv(settings: dict) -> list[str]:
    """The command line the launcher port starts the companion with (the
    settings' vision profile, the logical device, the lifecycle stop)."""
    return lockstep_navpy_argv(SYS_ID, SERIAL0, vision_profile=settings["camera"]["vision_profile"])


def slice_roles(scenario: DemoScenario, *, companion: RoleSpec | None = None,
                navpy: bool = False, pose_cadence_dir: str | None = None) -> dict[str, RoleSpec]:
    """The backend role; with ``navpy`` (or a stand-in ``companion``) the
    companion role ``navpy`` that its launcher port spawns. A run-relative
    ``pose_cadence_dir`` makes both record the pose-cadence CSVs there
    (``navpy.sim.lockstep.pose_recording``)."""
    settings = build_settings(scenario)
    recording = {} if pose_cadence_dir is None else {POSE_CADENCE_DIR: pose_cadence_dir}
    roles, companions = {}, None
    if navpy or companion is not None:
        argv = companion_argv(settings)
        companions = {str(SYS_ID): {"role": "navpy", "ordinal": NAVPY, "device": SERIAL0, "argv": argv}}
        roles["navpy"] = companion or role(NAVPY_HOOKS, SERIAL0, {"argv": argv, **recording})
    roles["backend"] = backend_role(settings, SLICE, companions=companions,
                                    pose_cadence_dir=pose_cadence_dir)
    return roles


__all__ = [
    "BACKEND", "NAVPY", "ROUTER", "SERIAL0", "SITL_PORTS", "SLICE", "STATUS_ROUTE", "SYS_ID",
    "companion_argv", "companion_script", "slice_roles",
]
