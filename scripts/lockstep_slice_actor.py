"""The 1-UAV slice's scripted API actor and participant roles (steps 9e, 9f).

The actor runs on the operator's baton and makes the operator's calls
through the lockstep ports (``lockstep_slice.md``, "Operator"):

- :func:`setup_script` (9e): connect ``lockstep:serial1``, read the command
  bounds by name, write the scenario's AAS profile, upload its mission;
- :func:`companion_script` (9f): the setup, then POST start-all (the
  backend's launcher port spawns NavPy), readiness through the status route,
  then stop-all, every outcome ``STOP_ACKED``.

:func:`slice_roles` builds the backend role and, with ``navpy``, the NavPy
companion role: one command line, :func:`companion_argv`, in both the
backend's launcher config and NavPy's spawn record.
"""

from __future__ import annotations

from collections.abc import Callable

from gcs.backend.navpy_process_launch import lockstep_navpy_argv
from navpy.sim.lockstep.coordinator_types import RoleSpec, backend_ordinal
from navpy.sim.lockstep.operator_baton import OperatorBaton
from navpy.sim.lockstep.operator_ports import BatonClock, LockstepApi, LockstepEventStream
from navpy.sim.lockstep.operator_tail import LockstepLogTail
from navpy.sim.lockstep.pose_recording import POSE_CADENCE_DIR
from scripts.eval_gcs_demo_command_bounds import command_bounds_from_snapshot
from scripts.eval_gcs_demo_config import aas_params_for, build_settings
from scripts.eval_gcs_demo_constants import FINAL_APPROACH_ROLL_LIMIT_DEG
from scripts.eval_gcs_demo_execution import wait_until
from scripts.eval_gcs_demo_lifecycle import connect_vehicles, start_companions, stop_companions
from scripts.eval_gcs_demo_mission_upload import upload_scenario_missions
from scripts.eval_gcs_demo_models import RegressionError, StackContext
from scripts.eval_gcs_demo_ports import JsonApiPort, JsonValue
from scripts.eval_gcs_demo_scenario import DemoScenario
from scripts.eval_gcs_demo_stack_ops import require_write_results, wait_for_vehicles

#: The slice's vehicle and its participants' ordinals (0 is the operator;
#: NavPy i is sysid i, the backend N+1).
SYS_ID = 1
NAVPY, BACKEND = SYS_ID, backend_ordinal(1)
SERIAL0, SERIAL1 = "lockstep:serial0", "lockstep:serial1"
BACKEND_HOOKS = "gcs.backend.lockstep.participant:backend"
NAVPY_HOOKS = "navpy.app.lockstep.participant:navpy"
#: The command-bound parameters the operator reads by name under lockstep.
COMMAND_BOUND_NAMES = ("PTCH_LIM_MIN_DEG", "PTCH_LIM_MAX_DEG", "AAS_DEL_THR", "TRIM_THROTTLE")
#: The full eval's named read: the bounds, and ROLL_LIMIT_DEG, which the
#: lockstep pre-boot defaults set (so no full-parameter write follows).
NAMED_PARAMETER_NAMES = (*COMMAND_BOUND_NAMES, "ROLL_LIMIT_DEG")
STATUS_ROUTE = "/api/control/navpy-sim/status"
#: Sim-time budgets of the actor's companion waits (the boot runs on ticks).
START_TIMEOUT_S = 30.0
READY_TIMEOUT_S = 60.0


def named_parameter_snapshot(api: JsonApiPort, sys_id: int) -> JsonValue:
    """Lockstep's command-bound read: fresh named reads, never MAVFTP (owner ruling 7)."""
    names = ",".join(NAMED_PARAMETER_NAMES)
    return api.get_json(f"/api/vehicles/{sys_id}/params/named?names={names}", timeout=90.0)


def stack_context(operator: OperatorBaton, scenario: DemoScenario, *,
                  epoch_us: int = 0) -> StackContext:
    """The operator's ports on the baton; ``epoch_us`` makes the clock's ``unix``
    the run's (the manifest epoch plus ``t_k``)."""
    return StackContext(
        api=LockstepApi(operator, BACKEND, error=RegressionError), sys_ids=(SYS_ID,),
        backend_tail=LockstepLogTail(operator, BACKEND), clock=BatonClock(operator, epoch_us),
        scenario=scenario,
        event_stream_factory=lambda _url: LockstepEventStream(operator, BACKEND),
        navigation_tail=lambda _sys_id: LockstepLogTail(operator, NAVPY), monitor_device=SERIAL1,
        parameter_reader=named_parameter_snapshot,
    )


def _received(events: LockstepEventStream) -> list[str]:
    """Every delivery queued so far (a zero wait never parks)."""
    received = []
    while True:
        try:
            received.append(events.receive(0.0))
        except TimeoutError:
            return received


def setup_script(scenario: DemoScenario) -> Callable[[OperatorBaton], dict]:
    """Connect, read the command bounds by name, write the AAS profile, upload."""

    def script(operator: OperatorBaton) -> dict:
        operator.wait_ready(BACKEND)
        context = stack_context(operator, scenario)
        api = context.api
        with context.event_stream_factory(api.websocket_url) as events:
            connect_vehicles(context)
            wait_for_vehicles(context, 30.0)
            names = ",".join(COMMAND_BOUND_NAMES)
            snapshot = api.get_json(f"/api/vehicles/{SYS_ID}/params/named?names={names}")
            bounds = command_bounds_from_snapshot(
                SYS_ID, snapshot, roll_limit_deg=FINAL_APPROACH_ROLL_LIMIT_DEG)
            aas = aas_params_for(scenario, SYS_ID, SYS_ID)
            require_write_results(api.put_json(f"/api/vehicles/{SYS_ID}/params", {"params": aas}),
                                  aas, "AAS")
            upload_scenario_missions(context)
            received = _received(events)
        return {"events": received, "bounds": repr(bounds), "aas": aas}

    return script


def _ready_companion(context: StackContext) -> dict | None:
    (status,) = context.api.get_json(STATUS_ROUTE)["instances"]
    return status if status["ready"] else None


def companion_script(scenario: DemoScenario) -> Callable[[OperatorBaton], dict]:
    """9e's setup, then start-all, readiness through status, then stop-all."""
    setup = setup_script(scenario)

    def script(operator: OperatorBaton) -> dict:
        result = setup(operator)
        context = stack_context(operator, scenario)
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
    config: dict = {"settings": settings, **recording}
    roles = {}
    if navpy or companion is not None:
        argv = companion_argv(settings)
        config["companions"] = {str(SYS_ID): {"role": "navpy", "ordinal": NAVPY,
                                              "device": SERIAL0, "argv": argv}}
        roles["navpy"] = companion or RoleSpec(NAVPY_HOOKS, (SERIAL0,), {"argv": argv, **recording})
    roles["backend"] = RoleSpec(BACKEND_HOOKS, (SERIAL1,), config)
    return roles


__all__ = [
    "BACKEND", "BACKEND_HOOKS", "COMMAND_BOUND_NAMES", "NAMED_PARAMETER_NAMES", "NAVPY", "NAVPY_HOOKS",
    "SERIAL0", "SERIAL1", "STATUS_ROUTE", "SYS_ID", "companion_argv", "companion_script",
    "named_parameter_snapshot", "setup_script", "slice_roles", "stack_context",
]
