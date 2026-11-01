"""A run's scheduled faults against its own evidence (``deterministic_dev_sim.md``, step 7).

``coordinator_faults`` journals what it did; this reads the journal's grid, its
tick times and each NavPy's log lines, and says what it should have done:

- every planned fault either started and healed, or was skipped: at the fence, or a
  restart at the tick the last UAV first checked in, when none may start any more;
- it started at the first tick it was due (its anchor line's time plus its drawn
  start) and clear of the fault before it, whose recovery deadline it waited for;
  a skipped one was not yet due and clear before the skip;
- it healed at the first tick its duration was up, or at the fence's tick when the
  fence came first; a restart heals in the tick its old generation ended;
- its recovery deadline is the heal's time plus its recovery;
- a restart's deadline falls at the fence at the latest, so the approval exchange
  never meets a NavPy coming back;
- the mission's approach starts only after every deadline.

:func:`fault_spans` gives the recovery gates (``eval_gcs_demo_fleet_faults``) each
fault's journaled start, heal and deadline.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from navpy.sim.lockstep.fault_schedule import (
    FENCE_MARKER,
    NAVPY_RESTART,
    PRESENCE_MARKER,
    SKIP_AT_FENCE,
    SKIP_AT_PRESENCE,
)

Lines = Mapping[int, Sequence[tuple[int, str]]]
#: The line that starts a vehicle's approach: no fault may still be recovering by then.
NAV_MARKER = "INIT: NAV MODE"
_GATE = "fault schedule"


def _us(seconds: float) -> int:
    return round(seconds * 1e6)


@dataclass(frozen=True)
class FaultSpan:
    """One fault that ran, from the journal."""

    plan: Mapping[str, Any]
    start_k: int
    heal_k: int
    deadline_us: int
    forced: bool

    @property
    def id(self) -> str:
        return self.plan["id"]


def planned_faults(records: Sequence[Mapping[str, Any]]) -> list[Mapping[str, Any]]:
    """The grid's plans; none in a fault-free run."""
    grid = next((record["grid"] for record in records if "grid" in record), {})
    return list(grid.get("faults", ()))


def fault_events(records: Sequence[Mapping[str, Any]]) -> dict[str, list[tuple[int, list[Any]]]]:
    """Fault id -> its events, each with its tick, in journal order."""
    events: dict[str, list[tuple[int, list[Any]]]] = {}
    for record in records:
        for event in record.get("faults", ()):
            events.setdefault(event[1], []).append((record["k"], event))
    return events


def fault_spans(records: Sequence[Mapping[str, Any]]) -> list[FaultSpan]:
    """Every fault that started and healed, in start order."""
    events = fault_events(records)
    spans = []
    for plan in planned_faults(records):
        mine = events.get(plan["id"], [])
        if [event[0] for _k, event in mine] == ["start", "heal"]:
            heal = mine[1][1]
            spans.append(FaultSpan(plan, mine[0][0], mine[1][0], heal[2], len(heal) > 3))
    return sorted(spans, key=lambda span: span.start_k)


def _first_tick(lines: Lines, marker: str) -> int | None:
    return min((k for rows in lines.values() for k, line in rows if marker in line), default=None)


def _presence_tick(lines: Lines) -> int | None:
    """The tick the last UAV first logged its check-in; None until they all have."""
    firsts = [next((k for k, line in rows if PRESENCE_MARKER in line), None) for rows in lines.values()]
    return None if None in firsts else max(firsts, default=None)


def _exit_tick(records: Sequence[Mapping[str, Any]], plan: Mapping[str, Any]) -> int | None:
    """The tick the target's generation before this fault's respawn ended."""
    ordinal = int(plan["target"].removeprefix("uav"))
    fault = plan["id"]
    respawn = next((entry for record in records for entry in record.get("lifecycle", ())
                    if entry["kind"] == "spawn" and entry["payload"] == {"fault": fault}), None)
    if respawn is None:
        return None
    return next((record["k"] for record in records for entry in record.get("lifecycle", ())
                 if entry["kind"] == "exited" and entry["ordinal"] == ordinal
                 and entry["generation"] == respawn["generation"] - 1), None)


class _Schedule:
    def __init__(self, records: Sequence[Mapping[str, Any]], lines: Lines) -> None:
        self.records = records
        self.times = {record["k"]: record["t_us"] for record in records if "t_us" in record}
        self.ticks = sorted(self.times)
        self.fence = _first_tick(lines, FENCE_MARKER)
        self.presence = _presence_tick(lines)
        self.lines = lines
        self.nav = _first_tick(lines, NAV_MARKER)

    def anchor_us(self, marker: str) -> int | None:
        k = _first_tick(self.lines, marker)
        return None if k is None else self.times[k]

    def first_at(self, t_us: int) -> int | None:
        """The first tick at or after ``t_us``."""
        return next((k for k in self.ticks if self.times[k] >= t_us), None)


def _start_errors(run: _Schedule, span: FaultSpan, clear_us: int) -> list[str]:
    anchor = run.anchor_us(span.plan["anchor"])
    if anchor is None:
        return [f"{_GATE}: {span.id} started at tick {span.start_k} with no {span.plan['anchor']!r} line"]
    due = run.first_at(max(anchor + _us(span.plan["start_s"]), clear_us))
    if span.start_k != due:
        return [f"{_GATE}: {span.id} started at tick {span.start_k}; it was due and clear at tick {due}"]
    if span.plan["kind"] == NAVPY_RESTART and run.presence is not None and span.start_k >= run.presence:
        return [f"{_GATE}: {span.id} started at tick {span.start_k}, once every UAV had checked in at {run.presence}"]
    return []


def _heal_errors(run: _Schedule, span: FaultSpan) -> list[str]:
    if span.plan["kind"] == NAVPY_RESTART:
        expected, forced = _exit_tick(run.records, span.plan), False
    else:
        up = run.first_at(run.times[span.start_k] + _us(span.plan["duration_s"]))
        forced = run.fence is not None and (up is None or run.fence < up)
        expected = run.fence if forced else up
    if span.heal_k != expected or span.forced != forced:
        return [f"{_GATE}: {span.id} healed at tick {span.heal_k} (forced {span.forced}); "
                f"expected tick {expected} (forced {forced})"]
    deadline = run.times[span.heal_k] + _us(span.plan["recovery_s"])
    if span.deadline_us != deadline:
        return [f"{_GATE}: {span.id}'s deadline is {span.deadline_us}, not its heal's time plus its "
                f"recovery {deadline}"]
    if span.plan["kind"] == NAVPY_RESTART and run.fence is not None and deadline > run.times[run.fence]:
        return [f"{_GATE}: {span.id}'s deadline {deadline} is after the fence at tick {run.fence}"]
    return []


def _due_and_clear(run: _Schedule, plan: Mapping[str, Any], spans: Sequence[FaultSpan]) -> int | None:
    """The first tick a plan could start, past each fault that held it back; None with no anchor."""
    anchor = run.anchor_us(plan["anchor"])
    if anchor is None:
        return None
    due_us = anchor + _us(plan["start_s"])
    for span in spans:  # in start order: each is busy from its start to its deadline
        due = run.first_at(due_us)
        if due is not None and span.start_k <= due and due_us < span.deadline_us:
            due_us = span.deadline_us
    return run.first_at(due_us)


def _skip_errors(run: _Schedule, plan: Mapping[str, Any], event: tuple[int, list[Any]],
                 spans: Sequence[FaultSpan]) -> list[str]:
    """A skip is at its cause's tick, and the fault could not have started before it."""
    tick, reason = event[0], event[1][2]
    if reason == SKIP_AT_PRESENCE and plan["kind"] != NAVPY_RESTART:
        return [f"{_GATE}: {plan['id']} skipped at the {reason}, which only closes a restart"]
    cause = {SKIP_AT_FENCE: run.fence, SKIP_AT_PRESENCE: run.presence}.get(reason)
    if tick != cause:
        return [f"{_GATE}: {plan['id']} skipped at tick {tick}, the {reason} came at {cause}"]
    due = _due_and_clear(run, plan, spans)
    if due is not None and due < tick:
        return [f"{_GATE}: {plan['id']} was due and clear at tick {due}, yet skipped at the {reason} {tick}"]
    return []


def _plan_errors(run: _Schedule, plan: Mapping[str, Any], events: Sequence[tuple[int, list[Any]]],
                 spans: Sequence[FaultSpan]) -> list[str]:
    """A plan's events have one of the two shapes: ran, or skipped."""
    kinds = [event[0] for _k, event in events]
    if kinds == ["start", "heal"]:
        return []
    if kinds == ["skip"]:
        return _skip_errors(run, plan, events[0], spans)
    return [f"{_GATE}: {plan['id']} has events {kinds}, not a start and a heal or a skip"]


def fault_schedule_errors(records: Sequence[Mapping[str, Any]], lines: Lines) -> list[str]:
    """The journal's faults against the grid, the tick times and the lines (module docstring)."""
    plans = planned_faults(records)
    events = fault_events(records)
    unknown = sorted(set(events) - {plan["id"] for plan in plans})
    errors = [f"{_GATE}: events for {fault}, which the grid does not plan" for fault in unknown]
    if not plans:
        return errors
    run = _Schedule(records, lines)
    spans = fault_spans(records)
    for plan in plans:
        errors += _plan_errors(run, plan, events.get(plan["id"], []), spans)
    clear_us = 0
    for span in spans:
        errors += _start_errors(run, span, clear_us) + _heal_errors(run, span)
        clear_us = span.deadline_us
        if run.nav is not None and span.deadline_us > run.times[run.nav]:
            errors.append(f"{_GATE}: {span.id}'s deadline {span.deadline_us} is after the first "
                          f"approach at tick {run.nav}")
    return errors


__all__ = ["FaultSpan", "Lines", "NAV_MARKER", "fault_events", "fault_schedule_errors", "fault_spans",
           "planned_faults"]
