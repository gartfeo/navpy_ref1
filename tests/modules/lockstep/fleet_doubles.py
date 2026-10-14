"""Test doubles for the 10a fleet coordinator: three scripted SITLs on recorded frames.

Each SITL plays the relay's side of one serial-step SITL (``ScriptedSitl``)
on the same recorded firmware clock (``serial_step_frames.csv``): three
synchronized frame streams, told apart by an egress tag. Its devices are
data (``lockstep_fleet.md``, Links and names): SITL i's serial0 is
``lockstep:serial0.<i>``, owned by rover i (NavPy-like, ordinal i), and its
serial1 is ``sitl<i>.serial1``; with no router yet (10b), the hub
(backend-like, ordinal N+1 = 4) owns the three serial1 devices directly.

:class:`FleetOperator` starts the rovers one at a time through the hub, as
the backend's launcher port does (each spawn after the previous rover's
readiness), then, once it has seen every expected rover's SNAP, stops them
one at a time and returns.
"""

from __future__ import annotations

from pathlib import Path

from navpy.sim.lockstep.coordinator_deadline import ExchangeDeadline
from navpy.sim.lockstep.coordinator_members import Roster
from navpy.sim.lockstep.coordinator_run import LockstepCoordinator
from navpy.sim.lockstep.coordinator_spawn import PROCESSES_HOST_FILE, HostSidecar, LocalSpawner
from navpy.sim.lockstep.coordinator_types import (
    Manifest,
    OperatorRequest,
    RoleSpec,
    WallBudgets,
    backend_ordinal,
)
from navpy.sim.lockstep.participant_hooks import RequestIds
from tests.modules.lockstep.coordinator_doubles import PYTHON_PATH, AllowAll, HostDelays, ScriptedSitl
from tests.modules.lockstep.test_grid_check_recording import recorded_frames

SITLS = 3
HUB = backend_ordinal(SITLS)
ROVERS = tuple(range(1, SITLS + 1))
TOYS = "tests.modules.lockstep.toy_participants"
#: Each rover's SNAP time, sim seconds (the recording starts at 1.74 s): rover 3 is last.
SNAP_AT_S = {1: 2.6, 2: 2.8, 3: 3.0}


def serial0(sitl: int) -> str:
    return f"lockstep:serial0.{sitl}"


def serial1(sitl: int) -> str:
    return f"sitl{sitl}.serial1"


def recorded_source_us():
    times = dict(recorded_frames())
    return lambda k: times[k]


def fleet_sitls(*, quiet_from=(3, 3, 3), delays: HostDelays | None = None, source_us=None):
    """Three SITLs on the recorded clock; ``source_us`` maps a SITL (1-based) to its own clock."""
    return [ScriptedSitl(quiet_from=quiet, delays=delays, tag=f"S{i}:".encode(),
                         source_us=(source_us or {}).get(i, recorded_source_us()))
            for i, quiet in zip(ROVERS, quiet_from)]


def fleet_manifest(*, hub=None, rovers=None, mission_budget_us=60_000_000, boot_budget_us=None,
                   initial=(("hub", HUB),), expected=ROVERS) -> Manifest:
    rovers = rovers or {}
    roles = {"hub": RoleSpec(f"{TOYS}:hub", tuple(serial1(i) for i in ROVERS),
                             {"ready_after_s": 0.1, "devices": [serial1(i) for i in ROVERS],
                              **(hub or {})})}
    for i in ROVERS:
        roles[f"rover{i}"] = RoleSpec(f"{TOYS}:rover", (serial0(i),), {
            "ready_after_s": 0.06, "snap_at_s": SNAP_AT_S[i], "devices": [serial0(i)],
            **rovers.get(i, {})})
    return Manifest(
        seed=1, epoch_us=1_760_000_000_000_000, max_frame_gap_us=20_825, drain_period_us=20_000,
        mission_budget_us=mission_budget_us, roles=roles, initial=initial,
        companion_roles=tuple(f"rover{i}" for i in ROVERS), expected_fleet=tuple(expected),
        sitl_ports=tuple((serial0(i), serial1(i)) for i in ROVERS), boot_budget_us=boot_budget_us,
    )


class FleetOperator:
    """Sequential spawn through the hub, then sequential stop after every SNAP.

    ``spawn`` limits which rovers it starts; ``stop_before_snap`` stops those
    rovers as soon as they are ready (they retire without a SNAP);
    ``give_up_at_us`` stops what runs and returns without every SNAP.
    """

    def __init__(self, ids: RequestIds, *, spawn=ROVERS, stop_before_snap=(),
                 give_up_at_us: int | None = None) -> None:
        self._ids = ids
        self._spawn = list(spawn)
        self._early = set(stop_before_snap)
        self._give_up_at_us = give_up_at_us
        self.hub_ready = False
        self.ready: set[int] = set()
        self.snaps: dict[int, int] = {}
        self.answers: dict = {}
        self._asked = None
        #: Rovers spawned, in spawn order.
        self.started: list[int] = []
        self._stopped: set[int] = set()
        self.done = False

    def observations(self):
        return {}

    def deliver_drain(self, inbox):
        pass

    def run_tick(self, inbox):
        self._observe(inbox)
        if self._asked is not None and self._asked not in self.answers:
            return ()  # one hub request at a time, as the launcher port awaits each
        out = self._next(inbox.t_us)
        if out is None:
            return ()
        self._asked = out.id
        return (out,)

    def _request(self, payload) -> OperatorRequest:
        return OperatorRequest(HUB, self._ids.next(), payload)

    def _next(self, t_us: int) -> OperatorRequest | None:
        if not self.hub_ready:
            return None
        early = sorted(self._early & self.ready - self._stopped)
        if early:
            self._stopped.add(early[0])
            return self._request({"op": "stop", "ordinal": early[0], "generation": 1})
        if self._spawn and (not self.started or self.started[-1] in self.ready):
            rover = self._spawn.pop(0)
            self.started.append(rover)
            return self._request({"op": "spawn", "role": f"rover{rover}", "ordinal": rover})
        finished = not self._spawn and all(r in self.snaps or r in self._early for r in self.started)
        gave_up = self._give_up_at_us is not None and t_us >= self._give_up_at_us
        if finished or gave_up:
            pending = [r for r in self.started if r not in self._stopped]
            if pending:
                self._stopped.add(pending[0])
                return self._request({"op": "stop", "ordinal": pending[0], "generation": 1})
            self.done = True
        return None

    def _observe(self, inbox) -> None:
        for entry in inbox.lifecycle:
            self.hub_ready = self.hub_ready or (entry.kind == "ready" and entry.ordinal == HUB)
        for _ordinal, response in inbox.responses:
            self.answers[response.id] = response.payload
        for _ordinal, event in inbox.events:
            if event.kind == "lifecycle" and event.payload["kind"] == "ready":
                self.ready.add(event.payload["ordinal"])
        for ordinal, line in inbox.log_lines:
            if ordinal in ROVERS and ordinal not in self.snaps and "SNAP(VISION-NAV" in line:
                self.snaps[ordinal] = inbox.k


def build_fleet(tmp_path: Path, *, plan: Manifest | None = None, sitls=None, delay_seed: int = 0,
                deadline: ExchangeDeadline | None = None, launch=None,
                budgets: WallBudgets = WallBudgets(register_s=60, done_s=60, exit_s=30),
                **operator_options):
    plan = plan or fleet_manifest()
    delays = HostDelays(delay_seed)
    sitls = sitls or fleet_sitls(delays=delays)
    tmp_path.mkdir(parents=True, exist_ok=True)
    roster = Roster(plan, LocalSpawner(tmp_path / "host", PYTHON_PATH, host_delay_seed=delay_seed),
                    AllowAll(), HostSidecar(tmp_path / PROCESSES_HOST_FILE), budgets,
                    deadline=deadline)
    ids = RequestIds(0)
    operator = FleetOperator(ids, **operator_options)
    options = {} if launch is None else {"launch": launch}
    coordinator = LockstepCoordinator(plan, sitls, operator, roster, ids, tmp_path / "host",
                                      delay=delays, **options)
    return coordinator, sitls, operator, roster
