"""Seeded scheduling in a participant's baton (dev sim step 6).

Step 6 of ``docs/design/deterministic_dev_sim.md``. Three models change how
one participant's baton schedules, never what product code does:

- ``thread_order``: the baton picks the next thread to run from its runnable
  queue by a seeded draw instead of first-in first-out, so the order of
  threads runnable at the same time varies by seed (wall threads are
  preempted by the OS and the GIL). The draws come from the model stream
  ``participant.<ordinal>.<generation>``, one per pick among two or more.
- ``reader_budget``: each lockstep bus reader drains its delivery without a
  yield per packet, up to ``packets_per_tick`` packets per tick (the wall
  reader's batching, ``reader_batch.md``). Packets past the budget stay queued,
  in order, until the reader's timer fires. Under a handoff cap it fires only
  if a handoff is left after the input phase (with ``timers_first``, before
  it), and the woken reader then competes for picks with no turn of its own
  (todo 200).
- ``timers_first``: each tick fires its due timers before its inputs, so a
  reply that lands on its waiter's deadline tick loses to the timeout (the
  wall race behind a just-late reply). Off, inputs come first. Under a handoff
  cap a timed-out waiter may stay runnable past the inputs; it then takes a
  reply that came first, as CPython's ``Queue.get`` re-checks the queue.

The NavPy roles and the backend role carry these draws in their role config
(``participant_budget.SIM_MODELS_KEY``); :func:`apply_scheduling` sets them on
the baton's ``runnable.scheduling`` before the first tick. A model that is off sets nothing, so with
none the baton schedules as before step 6.
"""

from __future__ import annotations

from collections.abc import Mapping

from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.baton_parts import ReaderBudget
from navpy.sim.lockstep.sim_profile import Number, model_stream

THREAD_ORDER = "thread_order"
READER_BUDGET = "reader_budget"
TIMERS_FIRST = "timers_first"
#: The models every product participant (each NavPy and the backend) applies to its baton.
SCHEDULING_MODELS = (READER_BUDGET, THREAD_ORDER, TIMERS_FIRST)


def _count(params: Mapping[str, Number], name: str) -> int:
    value = params[name]
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError(f"{name} must be an int of at least 1, got {value!r}")
    return value


def reader_budget(params: Mapping[str, Number]) -> ReaderBudget:
    """The drawn reader budget, checked."""
    return ReaderBudget(_count(params, "packets_per_tick"))


def check_scheduling(models: Mapping[str, Mapping[str, Number]]) -> None:
    """Refuse a bad draw before anything starts."""
    if READER_BUDGET in models:
        reader_budget(models[READER_BUDGET])


def apply_scheduling(baton: BatonRuntime, models: Mapping[str, Mapping[str, Number]], seed: int,
                     ordinal: int, generation: int) -> None:
    """Set the on scheduling models on a participant's baton, before its first tick."""
    scheduling = baton.runnable.scheduling
    if THREAD_ORDER in models:
        scheduling.order = model_stream(seed, THREAD_ORDER, f"participant.{ordinal}.{generation}")
    if READER_BUDGET in models:
        scheduling.reader_budget = reader_budget(models[READER_BUDGET])
    scheduling.timers_first = TIMERS_FIRST in models


def scheduling_subset(models: Mapping[str, Mapping[str, Number]]) -> dict[str, dict[str, Number]]:
    """The scheduling models of a NavPy's draw: what the backend role carries."""
    return {name: dict(models[name]) for name in SCHEDULING_MODELS if name in models}


__all__ = [
    "READER_BUDGET", "SCHEDULING_MODELS", "THREAD_ORDER", "TIMERS_FIRST", "apply_scheduling",
    "check_scheduling", "reader_budget", "scheduling_subset",
]
