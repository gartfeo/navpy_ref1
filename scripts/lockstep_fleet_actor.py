"""The lockstep fleet's actor wiring: per-UAV devices, ordinals, context and setup (step 10e).

``lockstep_fleet.md``, "Remaining packages" item 4. A :class:`FleetLayout`
maps each UAV (sysid i, SITL i) to its devices and participant ordinals by
the fleet's naming scheme ("Links and names", "Participants"):

- NavPy i owns ``lockstep:serial0.<i>`` and is ordinal i;
- the backend owns ``lockstep:router`` and is ordinal N+1 (the operator is 0);
- SITL i's serial1 is the coordinator's router endpoint ``sitl<i>``.

The operator's :func:`stack_context` gives every sysid its own NavPy log
tail, and the backend role (:func:`backend_role`) owns the router device
with its settings. :func:`setup_script` is 9e's setup for every sysid of the
layout (connect ``lockstep:router``, the command bounds read by name, the AAS
profile, the scenario's missions with their readback); :func:`fleet_script`
adds the ESP32 launch controller's simulator, in process (10e). The slice is
the layout with N = 1 (``scripts.lockstep_slice_actor``).
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.sim.lockstep.coordinator_types import (
    ROUTER_DEVICE,
    RoleSpec,
    backend_ordinal,
    fleet_sitl_ports,
    serial0_device,
)
from navpy.sim.lockstep.operator_baton import OperatorBaton
from navpy.sim.lockstep.operator_ports import BatonClock, LockstepApi, LockstepEventStream
from navpy.sim.lockstep.operator_tail import LockstepLogTail
from navpy.sim.lockstep.participant_hooks import DEVICE_KEY
from navpy.sim.lockstep.pose_recording import POSE_CADENCE_DIR
from scripts.eval_gcs_demo_command_bounds import command_bounds_from_snapshot
from scripts.eval_gcs_demo_config import aas_params_for
from scripts.eval_gcs_demo_constants import FINAL_APPROACH_ROLL_LIMIT_DEG
from scripts.eval_gcs_demo_lifecycle import connect_vehicles
from scripts.eval_gcs_demo_mission_upload import upload_scenario_missions
from scripts.eval_gcs_demo_models import RegressionError, StackContext
from scripts.eval_gcs_demo_ports import JsonApiPort, JsonValue
from scripts.eval_gcs_demo_scenario import DemoScenario
from scripts.eval_gcs_demo_stack_ops import require_write_results, start_esp32_simulator, wait_for_vehicles

BACKEND_HOOKS = "gcs.backend.lockstep.participant:backend"
NAVPY_HOOKS = "navpy.app.lockstep.participant:navpy"
#: The command-bound parameters the operator reads by name under lockstep.
COMMAND_BOUND_NAMES = ("PTCH_LIM_MIN_DEG", "PTCH_LIM_MAX_DEG", "AAS_DEL_THR", "TRIM_THROTTLE")
#: The full eval's named read: the bounds, and ROLL_LIMIT_DEG, which the
#: lockstep pre-boot defaults set (so no full-parameter write follows).
NAMED_PARAMETER_NAMES = (*COMMAND_BOUND_NAMES, "ROLL_LIMIT_DEG")
ESP32_STATUS_ROUTE = "/api/control/launch/esp32-status"
#: Sim-time budget of the setup's wait for every vehicle's link.
VEHICLES_TIMEOUT_S = 30.0


@dataclass(frozen=True)
class FleetLayout:
    """UAVs 1..``size``: sysid i flies SITL i and NavPy i."""

    size: int

    def __post_init__(self) -> None:
        if type(self.size) is not int or self.size < 1:
            raise ValueError(f"a fleet has at least one UAV, not {self.size!r}")

    @property
    def sys_ids(self) -> tuple[int, ...]:
        return tuple(range(1, self.size + 1))

    @property
    def backend(self) -> int:
        """The backend's participant ordinal, after NavPy 1..N."""
        return backend_ordinal(self.size)

    @property
    def router(self) -> str:
        """The backend's router endpoint device."""
        return ROUTER_DEVICE

    @property
    def sitl_ports(self) -> tuple[tuple[str, str], ...]:
        """``Manifest.sitl_ports``: each SITL's (serial0, serial1) devices."""
        return fleet_sitl_ports(self.size)

    def navpy(self, sys_id: int) -> int:
        """NavPy ``sys_id``'s participant ordinal."""
        return self._check(sys_id)

    def serial0(self, sys_id: int) -> str:
        """SITL ``sys_id``'s serial0: NavPy ``sys_id``'s companion device."""
        return serial0_device(self._check(sys_id))

    def _check(self, sys_id: int) -> int:
        # An exact int: ``True`` or ``1.0`` would pass the membership test and
        # name a device ``lockstep:serial0.True``.
        if type(sys_id) is not int or sys_id not in self.sys_ids:
            raise ValueError(f"sysid {sys_id!r} is not in the fleet {self.sys_ids}")
        return sys_id


def named_parameter_snapshot(api: JsonApiPort, sys_id: int) -> JsonValue:
    """Lockstep's command-bound read: fresh named reads, never MAVFTP (owner ruling 7)."""
    names = ",".join(NAMED_PARAMETER_NAMES)
    return api.get_json(f"/api/vehicles/{sys_id}/params/named?names={names}", timeout=90.0)


def stack_context(operator: OperatorBaton, scenario: DemoScenario, layout: FleetLayout, *,
                  epoch_us: int = 0) -> StackContext:
    """The operator's ports on the baton, one NavPy tail per sysid; ``epoch_us``
    makes the clock's ``unix`` the run's (the manifest epoch plus ``t_k``)."""
    backend = layout.backend
    return StackContext(
        api=LockstepApi(operator, backend, error=RegressionError), sys_ids=layout.sys_ids,
        backend_tail=LockstepLogTail(operator, backend), clock=BatonClock(operator, epoch_us),
        scenario=scenario,
        event_stream_factory=lambda _url: LockstepEventStream(operator, backend),
        navigation_tail=lambda sys_id: LockstepLogTail(operator, layout.navpy(sys_id)),
        monitor_device=layout.router, parameter_reader=named_parameter_snapshot,
    )


def role(hooks: str, device: str, config: dict) -> RoleSpec:
    """A product role: the device it owns, also named in its config for its hooks."""
    return RoleSpec(hooks, (device,), {DEVICE_KEY: device, **config})


def backend_role(settings: dict, layout: FleetLayout, *, companions: dict | None = None,
                 pose_cadence_dir: str | None = None) -> RoleSpec:
    """The backend on the router device with its settings document; with
    ``companions`` its launcher port's ``{sys_id: spec}`` map, and with a
    run-relative ``pose_cadence_dir`` the pose-cadence recording."""
    config: dict = {"settings": settings}
    if pose_cadence_dir is not None:
        config[POSE_CADENCE_DIR] = pose_cadence_dir
    if companions is not None:
        config["companions"] = companions
    return role(BACKEND_HOOKS, layout.router, config)


def _received(events: LockstepEventStream) -> list[str]:
    """Every delivery queued so far (a zero wait never parks)."""
    received = []
    while True:
        try:
            received.append(events.receive(0.0))
        except TimeoutError:
            return received


def setup_script(scenario: DemoScenario, layout: FleetLayout) -> Callable[[OperatorBaton], dict]:
    """Connect every sysid in order, then per sysid read its command bounds by
    name and write its AAS profile, then upload the scenario's missions."""

    def script(operator: OperatorBaton) -> dict:
        operator.wait_ready(layout.backend)
        context = stack_context(operator, scenario, layout)
        api = context.api
        owner = scenario.owner_sys_id(scenario.uav_ids(context.sys_ids))
        bounds, aas = {}, {}
        with context.event_stream_factory(api.websocket_url) as events:
            connect_vehicles(context)
            wait_for_vehicles(context, VEHICLES_TIMEOUT_S)
            names = ",".join(COMMAND_BOUND_NAMES)
            for sys_id in context.sys_ids:
                snapshot = api.get_json(f"/api/vehicles/{sys_id}/params/named?names={names}")
                bounds[str(sys_id)] = repr(command_bounds_from_snapshot(
                    sys_id, snapshot, roll_limit_deg=FINAL_APPROACH_ROLL_LIMIT_DEG))
                written = aas_params_for(scenario, sys_id, owner)
                require_write_results(api.put_json(f"/api/vehicles/{sys_id}/params", {"params": written}),
                                      written, "AAS")
                aas[str(sys_id)] = written
            upload_scenario_missions(context)
            received = _received(events)
        return {"events": received, "bounds": bounds, "aas": aas}

    return script


def fleet_script(scenario: DemoScenario, layout: FleetLayout) -> Callable[[OperatorBaton], dict]:
    """10e's scripted actor: :func:`setup_script`, then the ESP32 simulator
    started through its route and probed through the in-process port."""
    setup = setup_script(scenario, layout)

    def script(operator: OperatorBaton) -> dict:
        result = setup(operator)
        context = stack_context(operator, scenario, layout)
        start_esp32_simulator(context)
        esp32 = context.api.get_json(ESP32_STATUS_ROUTE)
        if not isinstance(esp32, dict) or esp32.get("reachable") is not True:
            raise RegressionError(f"the ESP32 simulator is not reachable: {esp32}")
        return {**result, "esp32": esp32}

    return script


__all__ = [
    "BACKEND_HOOKS", "COMMAND_BOUND_NAMES", "ESP32_STATUS_ROUTE", "FleetLayout", "NAMED_PARAMETER_NAMES",
    "NAVPY_HOOKS", "VEHICLES_TIMEOUT_S", "backend_role", "fleet_script", "named_parameter_snapshot",
    "role", "setup_script", "stack_context",
]
