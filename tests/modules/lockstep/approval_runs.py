"""The real confirmation workflow on the operator's baton with its evidence on.

``run_confirmation_workflow`` runs through the lockstep ports, with the
due-check hook, against the scripted backend of ``test_confirmation_timing``
(deliveries at chosen ticks; each approval answered after the route's three
sends 0.25 s apart). The journal records are built as the coordinator builds
them: each tick's time, the operator's observations and the response sends'
NAVLINK counts. It stops once every expected vehicle is approved, or, with
``stop_at_s``, once they are and the clock has reached that sim time (as
production keeps polling for SNAP).
"""

from __future__ import annotations

import json

from navpy.sim.lockstep.coordinator_types import serial0_device, serial1_device
from navpy.sim.lockstep.operator_baton import OperatorBaton
from navpy.sim.lockstep.operator_ports import (
    BatonClock,
    LockstepApi,
    LockstepEventStream,
    due_check_hook,
    polling_start_hook,
)
from navpy.sim.lockstep.participant_hooks import RequestIds
from scripts.eval_gcs_demo_confirmation import run_confirmation_workflow
from scripts.eval_gcs_demo_confirmation_timing import RESPONSE_DIRECTION, RESPONSE_ID
from tests.modules.lockstep.test_confirmation_timing import BACKEND, DELAY_S, TELEMETRY, UID, ScriptedBackend

__all__ = ["TELEMETRY", "UID", "request", "run_workflow", "uniform"]


def request(sys_id: int, task_id: int = 1, uid: str = UID) -> str:
    return json.dumps({"type": "task_confirm_request", "sys_id": sys_id, "task_id": task_id,
                       "round_uid": uid})


def uniform(grid_us: int, count: int) -> list[tuple[int, int]]:
    return [(k, k * grid_us) for k in range(1, count + 1)]


def run_workflow(frames: list[tuple[int, int]], deliveries: dict[int, list[str]],
                 expected: set[int], *, max_gap_us: int, evidence: bool = True,
                 delay_s: float = DELAY_S, timeout_s: float = 120.0,
                 stop_at_s: float = 0.0) -> tuple[list[dict], list[dict]]:
    """The journal records and the approvals of one run (``expected`` sysids;
    the response sends are counted on vehicle 1's serial1, as the backend
    double makes no per-vehicle split)."""
    backend = ScriptedBackend(dict(frames), deliveries)
    approvals: list[dict] = []

    def script(operator: OperatorBaton) -> None:
        clock = BatonClock(operator)
        run_confirmation_workflow(
            event_stream=LockstepEventStream(operator, BACKEND), api=LockstepApi(operator, BACKEND),
            expected_sys_ids=expected, trigger=lambda: None,
            stop_when=lambda: len(approvals) == len(expected) and clock.monotonic() >= stop_at_s,
            requested_delay_s=delay_s, timeout_s=timeout_s, clock=clock,
            on_approval_accepted=approvals.append, on_polling_start=polling_start_hook(operator),
            on_due_check=due_check_hook(operator))

    operator = OperatorBaton(RequestIds(0), script, evidence=evidence)
    ports = [[serial0_device(sys_id), serial1_device(sys_id)] for sys_id in sorted(expected)]
    link = serial1_device(min(expected))
    records: list[dict] = [{"grid": {"max_frame_gap_us": max_gap_us, "sitl_ports": ports}}]
    for k, t_us in frames:
        backend.handle(k, operator.run_tick(backend.inbox(k)))
        record: dict = {"k": k, "t_us": t_us}
        observed = operator.observations()
        if observed:
            record["operator"] = observed
        if k in backend.sends:
            record["navlink"] = {link: {RESPONSE_DIRECTION: {f"{RESPONSE_ID}:255.0>{min(expected)}":
                                                             backend.sends[k]}}}
        records.append(record)
        if operator.done:
            break
    assert operator.done, "the workflow never approved every vehicle"
    return records, approvals
