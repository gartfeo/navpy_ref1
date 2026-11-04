"""The operator side of a lockstep run: the app, the admission and the pacer.

``app`` (step 11 of ``deterministic_dev_sim.md``): the run is paced at 1x
and serves the GCS app on its slot's ports (``lockstep_app_stack``); the
app's requests enter the run through the coordinator's admission, after the
scripted operator's in each tick. Every run writes its action file
(``lockstep_actions.jsonl``): each admitted request and cancel with its tick
and id, then the tick the operator returned at (``end``), when it did.
``pace`` alone paces a run without the app.
"""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path

from gcs.backend.lockstep.app_refusals import refusal
from navpy.sim.lockstep.coordinator_admission import Admission
from navpy.sim.lockstep.coordinator_operator import OperatorSide
from navpy.sim.lockstep.coordinator_pacing import RealTimePacer
from navpy.sim.lockstep.part_wire_codec import dumps
from navpy.sim.lockstep.participant_hooks import RequestIds
from navpy.sim.lockstep.replay import end_action
from scripts.lockstep_app_stack import start_app_stack

ACTIONS_FILE = "lockstep_actions.jsonl"


@contextmanager
def operator_side(run_dir: Path, slot: int, owner: dict, backend: int, ids: RequestIds, *,
                  app: bool, pace: bool, frontend: bool) -> Iterator[OperatorSide]:
    """Start the app stack when asked; it stops when the block ends, before the fleet."""
    if not app:
        yield OperatorSide(Admission(ids), RealTimePacer()) if pace else OperatorSide(Admission(ids))
        return
    stack = start_app_stack(run_dir, slot, owner, backend, frontend=frontend)
    try:
        print(f"GCS app on lockstep: {stack.url}", flush=True)
        yield OperatorSide(Admission(ids, stack.relay, refusal), RealTimePacer())
    finally:
        stack.stop()


def write_actions(path: Path, actions: list[dict], final_tick: int | None) -> None:
    """The action file: one canonical JSON line per admitted request or cancel,
    then the operator's return (``final_tick``; None: the run ended otherwise)."""
    lines = actions if final_tick is None else [*actions, end_action(final_tick)]
    path.write_bytes(b"".join(dumps(action) + b"\n" for action in lines))


__all__ = ["ACTIONS_FILE", "operator_side", "write_actions"]
