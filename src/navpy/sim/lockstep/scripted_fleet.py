"""Step 10d's fleet check: N serial-step SITLs under the fleet barrier, scripted (AUTO x N).

The coordinator's SITL side (``SitlFleet``: the launch on the exchange
deadline, frame k from every SITL with the per-SITL checks and the fleet
clock, the boot latch and its budget, the replies, parking, the stop by
ownership) with step 7's scripted operator on each SITL's serial1
(``ScriptedFleetOperator``). There are no participants and no router: no
serial0 bytes go anywhere, and each SITL's serial1 talks only to its own
vehicle's script (the 3-vehicle router path comes with the backend, 10e).
The operator's baton, its startup included, runs on what is left of the
tick's exchange deadline (``ExchangeDeadline.bounded``, as the coordinator's
operator does): a baton wait cut by it is ``HostFault("exchange_budget")``.

Boot ticks reply empty, up to and including the tick of the last
``helpers_quiet`` latch; the operator is admitted at the tick after it, as
the initial participants are (``coordinator_run``). Its baton runs on what
is left of the tick's exchange deadline. After the stop condition, every
SITL's parked frame k_F+1 is read with every check and never replied to.

The journal (``RunJournal``) holds run facts only: the grid, one record per
tick (``k``, phase, ``t_k``, frame and reply hashes per SITL, the SITLs
latched), the parked frames, then the barrier and the outcome. A fault is
journaled with its kind and tick, then every SITL is stopped by ownership.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.sim.lockstep.coordinator_deadline import OPERATOR, ExchangeDeadline
from navpy.sim.lockstep.coordinator_fleet import SitlFleet
from navpy.sim.lockstep.coordinator_journal import RunJournal, sha256
from navpy.sim.lockstep.coordinator_types import CoordinatorFault, Manifest, RunFault
from navpy.sim.lockstep.scripted_operator import ScriptedFleetOperator


@dataclass(frozen=True)
class FleetCheckResult:
    #: The tick the barrier completed (the last latch) and each SITL's latch tick.
    barrier_tick: int
    latches: tuple[int, ...]
    #: The last replied tick (k_F) and the parked one.
    final_tick: int
    parked_tick: int


class ScriptedFleetRun:
    def __init__(self, manifest: Manifest, fleet: SitlFleet, deadline: ExchangeDeadline,
                 operator: ScriptedFleetOperator, *, max_ticks: int,
                 stop_when: Callable[[], bool],
                 delay: Callable[[str], None] = lambda point: None) -> None:
        self._manifest = manifest
        self._fleet = fleet
        self._deadline = deadline
        self._operator = operator
        self._max_ticks = max_ticks
        self._stop_when = stop_when
        self._delay = delay
        self._k = 0
        self.journal = RunJournal()

    def run(self) -> FleetCheckResult:
        plan = self._manifest
        self.journal.add({"grid": {
            "max_frame_gap_us": plan.max_frame_gap_us, "boot_budget_us": plan.boot_budget_us,
            "sitl_ports": [list(ports) for ports in plan.sitl_ports], "max_ticks": self._max_ticks}})
        try:
            self._fleet.launch()
            k_f = self._ticks()
            self._k = k_f + 1
            parked, _egress = self._fleet.park(k_f + 1)
            self.journal.add({"parked": parked})
            self._fleet.snapshot(parked["tick"])
            self._fleet.stop()
        except BaseException as fault:  # any end before a clean one stops by ownership
            self._fail(fault)
            raise
        latches = tuple(self._fleet.boot.ticks)
        self.journal.add({"outcome": "ok", "barrier_tick": max(latches), "latches": list(latches),
                          "final_tick": k_f, "parked_tick": parked["tick"]})
        return FleetCheckResult(max(latches), latches, k_f, parked["tick"])

    def _ticks(self) -> int:
        """Ticks 1..k_F; returns k_F, the tick after which ``stop_when`` held."""
        fleet, boot = self._fleet, self._fleet.boot
        for k in range(1, self._max_ticks + 1):
            self._k = k
            frames, raws = fleet.read(k)
            t_us = frames[0].source_us
            record = {"k": k, "phase": "mission" if boot.complete else "boot", "t_us": t_us,
                      "frame": [sha256(raw) for raw in raws]}
            ingress: dict[str, bytes] = {}
            if boot.complete:
                egress = [frame.ports[1].egress for frame in frames]
                replies = self._deadline.bounded(  # the coordinator's operator wait (step 5)
                    OPERATOR, lambda wall: self._operator.on_tick(k, t_us, egress, wall))
                ingress = {ports[1]: data for ports, data in zip(self._manifest.sitl_ports, replies)}
            else:
                latched = boot.observe(k, t_us, [frame.helpers_quiet for frame in frames])
                if latched:
                    record["latched"] = [index + 1 for index in latched]
            record["reply"] = [sha256(raw) for raw in fleet.reply(k, ingress)]
            self.journal.add(record)
            self._delay("after_reply")
            if record["phase"] == "mission" and self._stop_when():
                return k
        raise CoordinatorFault("tick_budget", f"not finished within {self._max_ticks} ticks",
                               tick=self._max_ticks)

    def _fail(self, fault: BaseException) -> None:
        if isinstance(fault, RunFault):
            self.journal.add(fault.journal_record(self._k))
        else:
            self.journal.add({"fault": "unexpected", "kind": type(fault).__name__,
                              "tick": self._k, "ordinal": None})
        try:
            self._fleet.stop()
        except Exception:  # noqa: BLE001 - the original fault stands; the caller's stop retries
            pass


__all__ = ["FleetCheckResult", "ScriptedFleetRun"]
