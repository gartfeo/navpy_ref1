"""Lifecycle records: spawn and stop results, readiness, retirement.

Each record is committed at a tick (tick step 4) and journaled there. It goes
to its recipient: a participant gets it in its next ``TICK``, the operator
(ordinal 0) at step 5 of the same tick. Records about a participant go to the
ordinal that asked for its spawn, as the launcher port answers today; a
``stop`` goes to its target. Never a PID.

Participant stop (``lockstep_slice.md``): a stop request becomes ``stop`` for
the target; the target's ending ``DONE`` resolves it, ``acked`` for a flushed
end, else ``missed`` (a host fault). It is ``missed`` too after
:data:`STOP_BUDGET_US` of sim time, or when requested during drain (both
faults). A duplicate stop gets the same result; a stale generation an error.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from navpy.sim.lockstep.coordinator_journal import TickRecord
from navpy.sim.lockstep.coordinator_members import Member, Roster
from navpy.sim.lockstep.coordinator_types import STOP_BUDGET_US, HostFault
from navpy.sim.lockstep.part_wire import Lifecycle, RequestId, SpawnRequest, StopRequest
from navpy.sim.lockstep.participant_hooks import RequestIds

OPERATOR = 0


@dataclass
class _Stop:
    ordinal: int
    generation: int
    deadline_us: int
    #: Every (request id, requester) asking for this stop, in arrival order.
    askers: list[tuple[RequestId, int]] = field(default_factory=list)
    result: str | None = None


class LifecycleBook:
    def __init__(self, roster: Roster, ids: RequestIds) -> None:
        self._roster = roster
        #: Commit ids: the operator side's sequence, so every list is in id order.
        self._ids = ids
        self._stops: dict[tuple[int, int], _Stop] = {}
        self._record: TickRecord | None = None
        self.operator: list[Lifecycle] = []
        #: Faults found while committing; the run ends after the tick.
        self.faults: list[HostFault] = []

    def begin_tick(self, record: TickRecord) -> None:
        self._record = record
        self.operator = []

    def commit(self, recipient: int, kind: str, request: RequestId | None, ordinal: int,
               generation: int, result: str | None = None, payload: object = None) -> None:
        assert self._record is not None
        entry = Lifecycle(kind, self._ids.next(), request, ordinal, generation, result, payload)
        self._record.lifecycle(entry, recipient)
        if recipient == OPERATOR:
            self.operator.append(entry)
            return
        member = self._roster.current(recipient)
        if member is not None and member.live:
            member.inputs.lifecycle.append(entry)

    def spawn(self, request: SpawnRequest, requester: int, *, draining: bool) -> Member | None:
        live = self._roster.current(request.ordinal)
        if draining or (live is not None and live.live):
            reason = "draining" if draining else "ordinal is live"
            generation = self._roster.generation(request.ordinal) or 0
            self.commit(requester, "spawn", request.id, request.ordinal, generation, "error", reason)
            return None
        member = self._roster.spawn(request.role, request.ordinal, request.id, requester)
        self.commit(requester, "spawn", request.id, member.ordinal, member.spawn.generation, "ok")
        return member

    def ready(self, member: Member, payload: object) -> None:
        if member.ready:
            return
        member.ready = True
        spawn = member.spawn
        self.commit(spawn.requester, "ready", spawn.spawn_id, spawn.ordinal, spawn.generation,
                    None, payload)

    def retire(self, member: Member, k: int, flushed: bool) -> None:
        """Tick step 4: the participant's ending ``DONE k`` retires it now."""
        member.retired_at = k
        member.flushed = flushed
        spawn = member.spawn
        failed = not (flushed and member.ready)
        self.commit(spawn.requester, "exited", spawn.spawn_id, spawn.ordinal, spawn.generation,
                    None, {"failed": failed})
        stop = self._stops.get((spawn.ordinal, spawn.generation))
        if stop is not None and stop.result is None:
            self._resolve(stop, "acked" if flushed else "missed")
        if not flushed:  # every unflushed end is a host fault: the run tears down
            self.faults.append(HostFault("unclean_end", "ended without a flushed end",
                                         tick=k, ordinal=spawn.ordinal))

    def stop(self, request: StopRequest, requester: int, t_us: int, *, draining: bool) -> None:
        key = (request.ordinal, request.generation)
        if draining:
            self._answer(request.id, requester, key, "missed")
            self.faults.append(HostFault("stop_in_drain", f"stop {request.id} requested during drain",
                                         ordinal=request.ordinal))
            return
        # Every request is checked against the current generation first: once
        # the ordinal was respawned, a stop for the old generation is stale even
        # though that generation's own stop was acked.
        target = self._roster.current(request.ordinal)
        if target is None or target.spawn.generation != request.generation:
            self._answer(request.id, requester, key, "error", "stale generation")
            return
        existing = self._stops.get(key)
        if existing is not None:  # a duplicate of this generation's stop: the same result
            existing.askers.append((request.id, requester))
            if existing.result is not None:
                self._answer(request.id, requester, key, existing.result)
            return
        if not target.live:  # it ended on its own: no stop transaction to ack
            self._answer(request.id, requester, key, "error", "not running")
            return
        self._stops[key] = _Stop(request.ordinal, request.generation, t_us + STOP_BUDGET_US,
                                 [(request.id, requester)])
        self.commit(request.ordinal, "stop", request.id, request.ordinal, request.generation)

    def check_budgets(self, t_us: int) -> None:
        for stop in self._stops.values():
            if stop.result is None and t_us >= stop.deadline_us:
                self._resolve(stop, "missed")
                self.faults.append(HostFault(
                    "stop_budget", f"generation {stop.generation} missed its stop budget",
                    ordinal=stop.ordinal))

    def _resolve(self, stop: _Stop, result: str) -> None:
        stop.result = result
        for request_id, requester in stop.askers:
            self._answer(request_id, requester, (stop.ordinal, stop.generation), result)

    def _answer(self, request_id: RequestId, requester: int, key: tuple[int, int], result: str,
                payload: object = None) -> None:
        self.commit(requester, "stop_result", request_id, key[0], key[1], result, payload)

    def stopped(self, ordinal: int, generation: int) -> bool:
        """The generation retired through an acked participant stop."""
        stop = self._stops.get((ordinal, generation))
        return stop is not None and stop.result == "acked"


__all__ = ["LifecycleBook", "OPERATOR"]
