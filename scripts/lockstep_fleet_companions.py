"""The lockstep fleet's companion scripts: NavPy up and down, and the launch (steps 9f, 10f).

The operator's scripts on the baton, built on the fleet actor's setup
(``scripts.lockstep_fleet_actor.setup_script``):

- :func:`companion_script` (9f, 10f): the setup, then POST start-all (the
  backend's launcher port spawns NavPy 1..N, one after another), every
  companion's readiness through the status route, then stop-all, every
  outcome ``STOP_ACKED``;
- :func:`navlink_script` (10f's acceptance): the companions up, then
  :func:`launch_until_checked_in` (the launch on ESP32 channels 1..N until
  every NavPy has checked in, then a hold of :data:`PEER_HOLD_S`), then
  stop-all. A NavPy sends no peer NAVLINK on the ground: its swarm task
  actor starts on its first navigation state after ONHOLD (the first
  detection), so the peer path needs the flight.
"""

from __future__ import annotations

from collections.abc import Callable

from navpy.core.swarm.swarm_heartbeat_runtime import HEARTBEAT_INTERVAL_S
from navpy.sim.lockstep.operator_baton import OperatorBaton
from scripts.eval_gcs_demo_execution import wait_until
from scripts.eval_gcs_demo_lifecycle import start_companions, stop_companions
from scripts.eval_gcs_demo_mission import prepare_container_launch, trigger_all
from scripts.eval_gcs_demo_models import StackContext
from scripts.eval_gcs_demo_scenario import DemoScenario
from scripts.eval_gcs_demo_stack_ops import start_esp32_simulator, wait_for_missions
from scripts.lockstep_fleet_actor import VEHICLES_TIMEOUT_S, FleetLayout, setup_script, stack_context

STATUS_ROUTE = "/api/control/navpy-sim/status"
#: Sim-time budgets of the companion script's waits (the boot runs on ticks).
#: Each NavPy spawns after the previous one's start answer (sequential, as
#: in wall mode), so the readiness budget is per companion.
START_TIMEOUT_S = 30.0
READY_TIMEOUT_S = 60.0
#: The launch's sim-time budgets: the container preparation (the wall
#: eval's cap) and every NavPy's first check-in after the triggers (wall
#: ``step5-d``: the last one checks in about 10 s after its launch; the
#: triggers run one vehicle at a time).
LAUNCH_TIMEOUT_S = 180.0
CHECKIN_TIMEOUT_S = 120.0
#: The line NavPy logs once its check-in is broadcast (``TaskMessaging.checkin``).
CHECKIN_LINE = " INFO CheckIn"
#: The hold after the last check-in: two swarm heartbeat periods, so every
#: NavPy sends a heartbeat after every peer's swarm actor runs (the run then
#: shows each ordered pair's NAVLINK path; the acceptance checks it).
PEER_HOLD_S = 2 * HEARTBEAT_INTERVAL_S


def _ready_companions(context: StackContext) -> list[dict] | None:
    """Every companion's status once each one is ready, in sysid order; else None."""
    statuses = sorted(context.api.get_json(STATUS_ROUTE)["instances"], key=lambda s: s["sys_id"])
    ready = [status["sys_id"] for status in statuses if status["ready"]]
    return statuses if ready == list(context.sys_ids) else None


def _start_ready(context: StackContext, layout: FleetLayout) -> list[dict]:
    """Start-all, then every companion's status once all are ready."""
    start_companions(context, START_TIMEOUT_S)
    return wait_until("every companion ready", READY_TIMEOUT_S * layout.size,
                      lambda: _ready_companions(context), clock=context.clock)


def companion_script(scenario: DemoScenario, layout: FleetLayout) -> Callable[[OperatorBaton], dict]:
    """9e's setup, then start-all, every companion's readiness through the
    status route, then stop-all (every outcome ``STOP_ACKED``)."""
    setup = setup_script(scenario, layout)

    def script(operator: OperatorBaton) -> dict:
        result = setup(operator)
        context = stack_context(operator, scenario, layout)
        ready = _start_ready(context, layout)
        stop_companions(context)  # every outcome must be STOP_ACKED
        return {**result, "ready": ready, "stopped": context.api.get_json(STATUS_ROUTE)}

    return script


def checked_in(context: StackContext) -> list[int]:
    """The sysids whose NavPy has logged its check-in, in sysid order."""
    return [sys_id for sys_id in context.sys_ids
            if any(line.endswith(CHECKIN_LINE) for line in context.navigation_tail(sys_id).text.splitlines())]


def launch_until_checked_in(context: StackContext) -> float:
    """With the companions up, in the wall eval's order (``run_mission``):
    the ESP32 simulator, every onboard mission, the container prepared, then
    each sysid's trigger (its ESP32 channel); once every NavPy has checked
    in, :data:`PEER_HOLD_S` of sim time. Returns the sim time of the last
    check-in."""
    start_esp32_simulator(context)
    wait_for_missions(context, VEHICLES_TIMEOUT_S)
    prepare_container_launch(context, LAUNCH_TIMEOUT_S)
    trigger_all(context)
    wait_until("every NavPy checked in", CHECKIN_TIMEOUT_S,
               lambda: checked_in(context) == list(context.sys_ids), clock=context.clock)
    checked_in_s = context.clock.monotonic()
    context.clock.sleep(PEER_HOLD_S)
    return checked_in_s


def navlink_script(scenario: DemoScenario, layout: FleetLayout) -> Callable[[OperatorBaton], dict]:
    """10f's acceptance actor: 9e's setup, the companions up,
    :func:`launch_until_checked_in`, then stop-all."""
    setup = setup_script(scenario, layout)

    def script(operator: OperatorBaton) -> dict:
        result = setup(operator)
        context = stack_context(operator, scenario, layout)
        ready = _start_ready(context, layout)
        checked_in_s = launch_until_checked_in(context)
        stop_companions(context)  # every outcome must be STOP_ACKED
        return {**result, "ready": ready, "checked_in_s": checked_in_s,
                "stopped": context.api.get_json(STATUS_ROUTE)}

    return script


__all__ = [
    "CHECKIN_LINE", "CHECKIN_TIMEOUT_S", "LAUNCH_TIMEOUT_S", "PEER_HOLD_S", "READY_TIMEOUT_S",
    "START_TIMEOUT_S", "STATUS_ROUTE", "checked_in", "companion_script", "launch_until_checked_in",
    "navlink_script",
]
