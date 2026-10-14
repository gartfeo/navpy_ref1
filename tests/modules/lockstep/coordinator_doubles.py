"""Test doubles for the 9d coordinator: a scripted SITL behind a relay, an operator.

:class:`ScriptedSitl` plays the relay's TCP side of one serial-step SITL:
frames and replies strictly alternate, as ``lockstep_relay.py`` copies them,
so after reply k_F the next read returns the parked frame k_F+1 and the SITL
then waits for a reply that never comes. Its "CSV" has one row per frame it
produced; the snapshot must end at the parked frame's row.
"""

from __future__ import annotations

import hashlib
import math
import os
import random
import time
from pathlib import Path

from navpy.sim.lockstep.coordinator_members import Roster
from navpy.sim.lockstep.coordinator_run import LockstepCoordinator
from navpy.sim.lockstep.coordinator_spawn import PROCESSES_HOST_FILE, HostSidecar, LocalSpawner
from navpy.sim.lockstep.coordinator_types import (
    Manifest,
    OperatorCancel,
    OperatorRequest,
    RoleSpec,
    WallBudgets,
)
from navpy.sim.lockstep.participant_hooks import RequestIds
from navpy.sim.lockstep.step_wire import Frame, PortState, pack_frame, unpack_reply

REPO = Path(__file__).resolve().parents[3]
HUB, ROVER = 2, 1
#: Upper bound of one injected host delay (seconds).
MAX_DELAY_S = 0.004


class HostDelays:
    """Seeded host delays at named points; seed 0 is none."""

    def __init__(self, seed: int) -> None:
        self._random = random.Random(seed) if seed else None

    def __call__(self, _point: str = "") -> None:
        if self._random is not None:
            time.sleep(self._random.random() * MAX_DELAY_S)


class ScriptedSitl:
    def __init__(self, *, period_us: int = 20_000, quiet_from: int = 3, delays: HostDelays | None = None,
                 source_us=None, advances=None, extra_egress=None, extra_egress1=None,
                 tag: bytes = b"", echo: bool = True, forward: bool = False) -> None:
        self._source = source_us or (lambda k: 1_000_000 + k * period_us)
        self._advances = advances or (lambda k: 0)
        #: Bytes appended to frame k's serial0 egress (a MAVLink packet, say).
        self._extra = extra_egress or (lambda k: b"")
        #: Bytes appended to frame k's serial1 egress (the autopilot's GCS stream).
        self._extra1 = extra_egress1 or (lambda k: b"")
        #: Echo each reply's ingress in the next frame's egress (false: as an
        #: autopilot does, which never sends a packet back out of its port).
        self._echo = echo
        #: Forward each port's ingress out of the other port, as an autopilot
        #: routes a packet between its serial ports (serial0 <-> serial1).
        self._forward = forward
        self._quiet_from = quiet_from
        #: Prefixed to every port's egress: tells a fleet's SITLs apart.
        self._tag = tag
        self._delays = delays or HostDelays(0)
        self._awaiting = False
        self._ingress = (b"", b"")
        self.k = 0
        self.rows: list[tuple[int, int, str]] = []
        self.snapshot_rows: list[tuple[int, int, str]] | None = None
        self.stopped = False

    def recv_frame(self, timeout_s: float = math.inf) -> bytes:
        if self.stopped:
            raise ConnectionError("relay closed")
        if self._awaiting:
            raise ConnectionError("the relay alternates: frame read before its reply")
        self._delays("frame")
        self.k += 1
        extras = (self._extra(self.k), self._extra1(self.k))
        echoed = self._ingress if self._echo else (b"", b"")
        forwarded = self._ingress[::-1] if self._forward else (b"", b"")
        egress = tuple(self._tag + f"s{i}k{self.k}|".encode() + echoed[i] + forwarded[i] + extras[i]
                       for i in range(2))
        frame = Frame((1, 0), self.k, self._source(self.k), self._advances(self.k),
                      self.k >= self._quiet_from,
                      tuple(PortState(0, 0, 0, data) for data in egress))
        raw = pack_frame(frame)
        self.rows.append((self.k, frame.source_us, hashlib.sha256(raw).hexdigest()))
        self._awaiting = True
        return raw

    def send_reply(self, reply: bytes, timeout_s: float = math.inf) -> None:
        self._ingress = unpack_reply(reply, expected_tick=self.k)
        self._awaiting = False
        self._delays("reply")

    def snapshot(self, parked_tick: int) -> None:
        assert self.rows[-1][0] == parked_tick and self._awaiting  # blocked on reply k_F+1
        self.snapshot_rows = list(self.rows)

    def stop(self) -> None:
        self.stopped = True


class ScriptedOperator:
    """Echo the hub, spawn the rover, start and cancel a slow request, stop the rover at SNAP."""

    def __init__(self, ids: RequestIds, *, extra_stops: tuple[int, ...] = (),
                 give_up_at_us: int | None = None, return_after_spawn: bool = False) -> None:
        self._ids = ids
        self._extra_stops = extra_stops
        #: With no SNAP by then, stop the rover anyway and return (a timed-out mission).
        self._give_up_at_us = give_up_at_us
        self._return_after_spawn = return_after_spawn
        self._step = 0
        self.slow_id = None
        self.stop_ids: list = []
        self.answers: dict = {}
        self.hub_ready = self.rover_ready = False
        self.snap_tick: int | None = None
        self.trace: list = []
        self.log_lines: list[str] = []
        self.done = False

    def observations(self):
        return {}

    def deliver_drain(self, inbox):
        pass  # the 9d traces end at the operator's return

    def run_tick(self, inbox):
        self._observe(inbox)
        out = []

        def request(payload):
            out.append(OperatorRequest(HUB, self._ids.next(), payload))
            return out[-1].id

        if self._step == 0 and self.hub_ready:
            request({"op": "echo", "text": "hi"})
            request({"op": "spawn", "role": "rover", "ordinal": ROVER})
            self._step = 1
            self.done = self._return_after_spawn
        elif self._step == 1 and self.rover_ready:
            self.slow_id = request({"op": "slow"})
            self._step = 2
        elif self._step == 2:
            out.append(OperatorCancel(HUB, self.slow_id))
            self._step = 3
        elif self._step == 3 and (self.snap_tick is not None or self._gave_up(inbox.t_us)):
            for generation in (1, *self._extra_stops):
                self.stop_ids.append(request({"op": "stop", "ordinal": ROVER, "generation": generation}))
            self._step = 4
        elif self._step == 4 and all(stop in self.answers for stop in self.stop_ids):
            self.done = True
        self.trace.extend([inbox.k, "request", list(item.id), type(item).__name__] for item in out)
        return tuple(out)

    def _gave_up(self, t_us: int) -> bool:
        return self._give_up_at_us is not None and t_us >= self._give_up_at_us

    def _observe(self, inbox) -> None:
        for entry in inbox.lifecycle:
            self.trace.append([inbox.k, "lifecycle", entry.kind, entry.ordinal, entry.generation])
            self.hub_ready = self.hub_ready or (entry.kind == "ready" and entry.ordinal == HUB)
        for _ordinal, response in inbox.responses:
            self.answers[response.id] = response.payload
            self.trace.append([inbox.k, "response", list(response.id), response.payload])
        for _ordinal, event in inbox.events:
            if event.kind == "lifecycle":
                self.trace.append([inbox.k, "event", event.payload])
                if event.payload["kind"] == "ready" and event.payload["ordinal"] == ROVER:
                    self.rover_ready = True
        self.log_lines.extend(line for _o, line in inbox.log_lines)
        if self.snap_tick is None and any("SNAP(VISION-NAV" in line
                                          for ordinal, line in inbox.log_lines if ordinal == ROVER):
            self.snap_tick = inbox.k


class AllowAll:
    """A child recorder for tests that do not exercise the registry."""

    def __init__(self) -> None:
        self.children: list = []

    def record(self, child, spawn) -> bool:
        self.children.append((child.pid, spawn))
        return True

    def forget(self, child, spawn) -> None:
        self.children.remove((child.pid, spawn))


#: The source roots a participant imports from (the toy hooks live under tests/).
PYTHON_PATH = (str(REPO / "src"), str(REPO))


def child_environment() -> dict[str, str]:
    """For helper processes that are not participants (registry owners)."""
    environment = dict(os.environ)
    environment["PYTHONPATH"] = os.pathsep.join((str(REPO / "src"), str(REPO)))
    return environment


def manifest(*, hub=None, rover=None, initial=(("hub", HUB),), mission_budget_us=60_000_000,
             max_frame_gap_us=25_000, drain_period_us=20_000) -> Manifest:
    toys = "tests.modules.lockstep.toy_participants"
    return Manifest(
        seed=1, epoch_us=1_760_000_000_000_000, max_frame_gap_us=max_frame_gap_us,
        drain_period_us=drain_period_us, mission_budget_us=mission_budget_us,
        roles={
            "hub": RoleSpec(f"{toys}:hub", ("lockstep:serial1",), {"ready_after_s": 0.1, **(hub or {})}),
            "rover": RoleSpec(f"{toys}:rover", ("lockstep:serial0",),
                              {"ready_after_s": 0.06, "snap_at_s": 1.5, **(rover or {})}),
        },
        initial=initial,
        companion_roles=("rover",),
        expected_fleet=(ROVER,),
    )


def build(tmp_path: Path, *, plan: Manifest | None = None, recorder=None, delay_seed: int = 0,
          budgets: WallBudgets = WallBudgets(register_s=60, done_s=60, exit_s=30),
          sitl: ScriptedSitl | None = None, **operator_options):
    plan = plan or manifest()
    delays = HostDelays(delay_seed)
    sitl = sitl or ScriptedSitl(delays=delays)
    tmp_path.mkdir(parents=True, exist_ok=True)
    # The delay seed perturbs the participants too (acceptance (c)).
    roster = Roster(plan, LocalSpawner(tmp_path / "host", PYTHON_PATH, host_delay_seed=delay_seed),
                    recorder or AllowAll(), HostSidecar(tmp_path / PROCESSES_HOST_FILE), budgets)
    ids = RequestIds(0)
    operator = ScriptedOperator(ids, **operator_options)
    coordinator = LockstepCoordinator(plan, (sitl,), operator, roster, ids, tmp_path / "host",
                                      delay=delays)
    return coordinator, sitl, operator, roster
