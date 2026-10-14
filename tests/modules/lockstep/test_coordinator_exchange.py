"""The tick's shared exchange deadline, the boot latch and the first-SNAP latch, on a fake clock.

``lockstep_fleet.md``, Fleet barrier, step 3: every wait of tick k draws on
one absolute deadline ``D_k = a_k + step - margin``, so waits that are each
legal under their own wall budget still fault once together they pass it,
before the firmware's own reply deadline (``a_k + step``) could.
"""

from __future__ import annotations

import dataclasses
import math
import socket
import threading
import time

import pytest

from navpy.sim.lockstep import coordinator_members
from navpy.sim.lockstep.baton import BATON_WALL_BUDGET_S, LockstepFault, handoff_budget
from navpy.sim.lockstep.baton_sync import sleep as baton_sleep
from navpy.sim.lockstep.coordinator import StepLink
from navpy.sim.lockstep.coordinator_deadline import LAUNCH, ExchangeDeadline
from navpy.sim.lockstep.coordinator_journal import TickRecord
from navpy.sim.lockstep.coordinator_latches import BootLatch, SnapLatch
from navpy.sim.lockstep.coordinator_lifecycle import LifecycleBook
from navpy.sim.lockstep.coordinator_members import Roster
from navpy.sim.lockstep.coordinator_router import RoutedPorts
from navpy.sim.lockstep.coordinator_spawn import PROCESSES_HOST_FILE, HostSidecar
from navpy.sim.lockstep.coordinator_tick import TickExchange
from navpy.sim.lockstep.coordinator_types import (
    CoordinatorFault,
    HostFault,
    OperatorInbox,
    RoleSpec,
    WallBudgets,
)
from navpy.sim.lockstep.operator_baton import OperatorBaton
from navpy.sim.lockstep.part_socket import PeerClosed
from navpy.sim.lockstep.part_wire import Done, PartWireError, SpawnRequest
from navpy.sim.lockstep.participant_hooks import RequestIds
from scripts import lockstep_relay
from scripts.lockstep_wsl_sitl import (
    EXCHANGE_MARGIN_S,
    STEP_DEADLINE_US,
    exchange_deadline,
)
from tests.modules.lockstep.coordinator_doubles import AllowAll
from tests.modules.lockstep.fleet_doubles import HUB, ROUTER, build_fleet, fleet_manifest, fleet_sitls

STEP_S = 30.0
MARGIN_S = 0.5
#: Each scripted wait's cost: legal under every own budget below.
COST_S = 10.0
OWN = WallBudgets(register_s=20.0, done_s=20.0, exit_s=20.0)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def spend(self, seconds: float, budget_s: float = math.inf) -> bool:
        """Spend ``seconds`` of a wait bounded by ``budget_s``; False if it timed out."""
        self.now += min(seconds, budget_s)
        return seconds <= budget_s


class FakeChild:
    pid = 0

    def __init__(self, clock: FakeClock, exit_cost_s: float) -> None:
        self._clock = clock
        self._exit_cost_s = exit_cost_s

    def go(self) -> None:
        pass

    def kill(self) -> None:
        pass

    def wait(self, timeout_s: float) -> int | None:
        return 0 if self._clock.spend(self._exit_cost_s, timeout_s) else None


class FakeLink:
    """A participant's NVPART01 link whose ``DONE`` takes ``done_cost_s`` of fake time."""

    def __init__(self, clock: FakeClock, done: Done | None = None, done_cost_s: float = 0.0) -> None:
        self._clock = clock
        self.done = done
        self.done_cost_s = done_cost_s

    def send(self, tick, deadline=None) -> bytes:
        return b"tick"

    def recv(self, deadline=None):
        budget = math.inf if deadline is None else deadline - time.monotonic()
        if not self._clock.spend(self.done_cost_s, budget):
            raise socket.timeout("no DONE")
        return self.done, b"done"

    def close(self) -> None:
        pass


class FakeSpawner:
    def __init__(self, clock: FakeClock) -> None:
        self._clock = clock

    def spawn(self, record, environment, variable) -> FakeChild:
        return FakeChild(self._clock, COST_S)


def _fake_register(clock: FakeClock, missed: list[float]):
    """``await_register`` for a child that takes ``COST_S`` to register;
    ``missed`` gets the fake time of each registration that ran out."""

    def await_register(listener, record, child, budget_s):
        if not clock.spend(COST_S, budget_s):
            missed.append(clock.now)
            raise HostFault("register_missed", "did not register", ordinal=record.ordinal)
        return FakeLink(clock)

    return await_register


def test_cumulative_legal_waits_fire_exchange_budget_before_the_firmware_deadline(tmp_path, monkeypatch):
    clock = FakeClock()
    deadline = ExchangeDeadline(STEP_S, MARGIN_S, clock=clock)
    missed: list[float] = []
    monkeypatch.setattr(coordinator_members, "await_register", _fake_register(clock, missed))
    roles = {"hub": RoleSpec("toy:hub", (ROUTER,)),
             "rover": RoleSpec("toy:rover", ("lockstep:serial0.1",))}
    plan = dataclasses.replace(fleet_manifest(initial=()), roles=roles)
    roster = Roster(plan, FakeSpawner(clock), AllowAll(), HostSidecar(tmp_path / PROCESSES_HOST_FILE),
                    OWN, deadline=deadline)
    ids = RequestIds(0)
    book = LifecycleBook(roster, ids)
    # Before any SITL runs no deadline is open: the spawns keep their own budgets.
    rover = roster.spawn("rover", 1, ids.next(), 0)
    hub = roster.spawn("hub", HUB, ids.next(), 0)
    assert clock.now == 2 * COST_S
    # Tick k = 5: the rover ends (its DONE, then its exit), the hub's DONE asks
    # for a spawn (a registration). Each wait is under its own 20 s budget.
    rover.link.done, rover.link.done_cost_s = Done(5, ended=True), COST_S
    hub.link.done = Done(5, spawn_requests=(SpawnRequest((HUB, 1), "rover", 2),))
    hub.link.done_cost_s = COST_S / 2
    deadline.anchor()
    a_k = clock.now
    deadline.open(5)
    exchange = TickExchange(roster, book, ("rover",), RoutedPorts(plan, roster))
    with pytest.raises(HostFault) as raised:
        exchange.run(5, 1_000_000, TickRecord(5, "mission", 1_000_000), draining=False)
    fault = raised.value
    assert (fault.kind, fault.tick, fault.context) == ("exchange_budget", 5, {"wait": "register"})
    assert fault.journal_record(None)["context"] == {"wait": "register"}
    # The waits so far: a DONE, an exit and a DONE (25 s); the registration got
    # only what was left of D_k, so the fault came at D_k, before the firmware's.
    assert missed == [pytest.approx(a_k + STEP_S - MARGIN_S)]
    assert missed[0] < a_k + STEP_S
    roster.stop_all()


def test_a_wait_returning_after_the_deadline_is_the_deadline_fault():
    clock = FakeClock()
    deadline = ExchangeDeadline(STEP_S, MARGIN_S, clock=clock)
    deadline.anchor()
    deadline.open(3)
    assert deadline.budget("done", 20.0) == 20.0
    clock.now = 25.0
    assert deadline.budget("done", 20.0) == pytest.approx(4.5)
    deadline.check("done")
    clock.now = 29.6
    with pytest.raises(HostFault, match="ran out in the exit wait"):
        deadline.check("exit")
    with pytest.raises(HostFault, match="exchange deadline"):
        deadline.budget("reply", 1.0)
    deadline.close()  # drain: own budgets again
    assert deadline.budget("done", 20.0) == 20.0 and not deadline.spent()


def test_an_infinite_step_deadline_never_binds_and_a_finite_one_needs_an_anchor():
    unbounded = ExchangeDeadline()
    unbounded.open(1)
    assert unbounded.budget("done", 7.0) == 7.0 and unbounded.due is None
    with pytest.raises(HostFault, match="before SITL 1's go"):
        ExchangeDeadline(STEP_S, MARGIN_S).open(1)
    with pytest.raises(ValueError):
        ExchangeDeadline(1.0, 1.0)


def test_a_frame_one_launch_that_overruns_is_an_exchange_budget_fault(tmp_path):
    clock = FakeClock()
    deadline = ExchangeDeadline(STEP_S, MARGIN_S, clock=clock)

    def launch(shared: ExchangeDeadline) -> None:
        shared.anchor()  # SITL 1's go
        shared.open(1)  # the remaining launches draw on frame 1's deadline
        for _sitl in (2, 3):
            clock.spend(COST_S, shared.budget(LAUNCH, 60.0))
        clock.spend(COST_S)  # reading frame 1 is all that is left: 0 s of it

    sitls = fleet_sitls()
    coordinator, *_ = build_fleet(tmp_path, plan=fleet_manifest(initial=()), sitls=sitls,
                                  deadline=deadline, launch=launch)
    with pytest.raises(HostFault, match="frame wait"):
        coordinator.run()
    assert coordinator.journal.records[-1] == {"fault": "HostFault", "kind": "exchange_budget",
                                               "tick": 1, "ordinal": None, "context": {"wait": "frame"}}
    assert all(sitl.stopped and sitl.k == 0 for sitl in sitls)


def test_a_launch_drawing_past_the_deadline_faults_in_the_launch(tmp_path):
    clock = FakeClock()
    deadline = ExchangeDeadline(STEP_S, MARGIN_S, clock=clock)

    def launch(shared: ExchangeDeadline) -> None:
        shared.anchor()
        shared.open(1)
        for _sitl in (2, 3, 4):  # the last one gets what is left and uses it all
            clock.spend(COST_S, shared.budget(LAUNCH, 60.0))
        shared.check(LAUNCH)

    coordinator, *_ = build_fleet(tmp_path, plan=fleet_manifest(initial=()), deadline=deadline,
                                  launch=launch)
    with pytest.raises(HostFault, match="launch wait"):
        coordinator.run()
    assert coordinator.journal.records[-1]["context"] == {"wait": "launch"}


def test_the_exchange_margin_covers_the_relay_delay_inside_the_firmware_deadline():
    assert lockstep_relay.MAX_DELAY_S < EXCHANGE_MARGIN_S < STEP_DEADLINE_US / 1_000_000
    deadline = exchange_deadline()
    deadline.anchor()
    deadline.open(1)
    assert deadline.due == pytest.approx(time.monotonic() + STEP_DEADLINE_US / 1e6 - EXCHANGE_MARGIN_S,
                                         abs=1.0)


def _handoffs(count: int, hold_s: float):
    """An operator script making ``count`` baton handoffs, each holding the baton
    ``hold_s`` of host time (a host primitive) before a zero wait hands it back."""

    def script(operator: OperatorBaton) -> None:
        for _handoff in range(count):
            time.sleep(hold_s)
            baton_sleep(operator.baton, 0.0)  # due at once: the next timer round's handoff

    return script


def test_every_handoff_and_timer_round_of_a_tick_draws_on_its_wall_deadline():
    # Four handoffs of 0.06 s: each is legal alone under a 0.15 s budget, but
    # together they pass the tick's deadline, so the third one faults.
    operator = OperatorBaton(RequestIds(0), _handoffs(4, 0.06))
    started = time.monotonic()
    with pytest.raises(LockstepFault, match="kept the baton for"):
        operator.run_tick(OperatorInbox(1, 1_000_000, wall_deadline=started + 0.15))
    assert time.monotonic() - started < 0.15 + 0.06  # not each handoff's own budget again
    # Without a tick deadline each handoff keeps the baton's own budget.
    unbounded = OperatorBaton(RequestIds(0), _handoffs(4, 0.06))
    unbounded.run_tick(OperatorInbox(1, 1_000_000))
    assert unbounded.done


def test_the_baton_handoff_budget_is_bounded_by_the_deadline():
    assert handoff_budget(None) == BATON_WALL_BUDGET_S
    assert handoff_budget(time.monotonic() + 3600) == BATON_WALL_BUDGET_S
    assert 0.0 < handoff_budget(time.monotonic() + 1.0) <= 1.0
    assert handoff_budget(time.monotonic() - 1.0) == 0.0


class _Trickle:
    """A relay that sends one frame in fragments, ``gap_s`` apart."""

    def __init__(self, gap_s: float, fragments: int = 3) -> None:
        self._server = socket.create_server(("127.0.0.1", 0))
        self.port = self._server.getsockname()[1]
        self._gap_s = gap_s
        self._fragments = fragments
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _serve(self) -> None:
        connection, _address = self._server.accept()
        payload = b"frame-bytes"
        message = len(payload).to_bytes(4, "little") + payload
        size = -(-len(message) // self._fragments)
        with connection:
            for start in range(0, len(message), size):
                time.sleep(self._gap_s)
                try:
                    connection.sendall(message[start:start + size])
                except OSError:
                    return
            time.sleep(1.0)  # keep the link open while the reader finishes

    def close(self) -> None:
        self._server.close()


def test_a_step_link_without_a_limit_keeps_its_per_read_timeout():
    # Each fragment comes within the link's own 0.4 s; the frame takes ~0.75 s.
    relay = _Trickle(0.25)
    link = StepLink(relay.port, timeout_s=0.4)
    try:
        assert link.recv_frame() == b"frame-bytes"
    finally:
        link.close()
        relay.close()


def test_a_step_link_with_a_limit_bounds_the_whole_frame():
    relay = _Trickle(0.25)
    link = StepLink(relay.port, timeout_s=0.4)
    started = time.monotonic()
    try:
        with pytest.raises(socket.timeout):
            link.recv_frame(0.4)  # every fragment within 0.4 s, the frame not
        assert time.monotonic() - started < 0.4 + 0.25
    finally:
        link.close()
        relay.close()


class _FailingLink(FakeLink):
    """A ``DONE`` wait that fails with ``error`` after ``cost_s`` of fake time."""

    def __init__(self, clock: FakeClock, error: BaseException, cost_s: float) -> None:
        super().__init__(clock)
        self._error = error
        self._cost_s = cost_s

    def recv(self, deadline=None):
        self._clock.spend(self._cost_s)
        raise self._error


@pytest.mark.parametrize(("error", "kind"), [
    (PeerClosed("peer closed"), "eof_without_end"),
    (ConnectionResetError("reset"), "eof_without_end"),
    (PartWireError("bad frame"), "done_invalid"),
    (OSError("socket error"), "done_invalid"),
])
@pytest.mark.parametrize("after", [False, True])
def test_every_failed_done_wait_past_the_deadline_is_the_deadline_fault(tmp_path, monkeypatch, error,
                                                                         kind, after):
    clock = FakeClock()
    deadline = ExchangeDeadline(STEP_S, MARGIN_S, clock=clock)
    monkeypatch.setattr(coordinator_members, "await_register", _fake_register(clock, []))
    plan = dataclasses.replace(fleet_manifest(initial=()),
                               roles={"rover": RoleSpec("toy:rover", ("lockstep:serial0.1",))})
    roster = Roster(plan, FakeSpawner(clock), AllowAll(), HostSidecar(tmp_path / PROCESSES_HOST_FILE),
                    OWN, deadline=deadline)
    member = roster.spawn("rover", 1, (0, 1), 0)
    deadline.anchor()
    deadline.open(5)
    member.link = _FailingLink(clock, error, STEP_S if after else COST_S)
    with pytest.raises((CoordinatorFault, HostFault)) as raised:
        roster.receive(member, 5)
    assert raised.value.kind == ("exchange_budget" if after else kind)
    roster.stop_all()


def test_the_boot_latch_completes_at_the_last_sitl_and_faults_past_its_budget():
    latch = BootLatch(3, budget_us=100)
    assert latch.observe(1, 1_000, [False, True, False]) == [1]
    assert latch.observe(2, 1_050, [True, True, False]) == [0]
    assert not latch.complete and latch.ticks == [2, 1, None]
    with pytest.raises(CoordinatorFault, match=r"SITLs \[3\] not quiet") as raised:
        latch.observe(3, 1_100, [True, True, False])
    assert raised.value.context == {"waiting": [3]}
    late = BootLatch(3, budget_us=100)
    late.observe(1, 1_000, [True, True, False])
    assert late.observe(2, 1_100, [True, True, True]) == [2] and late.complete  # just in time
    assert BootLatch(1, None).observe(1, 0, [False]) == []


def test_the_snap_latch_is_the_latest_first_snap_of_the_expected_fleet():
    latch = SnapLatch((1, 2, 3))
    latch.observe(10, [(1, 1), (9, 1)])  # 9 is not expected: ignored
    latch.observe(12, [(1, 1), (3, 1)])  # a later SNAP of 1 changes nothing
    assert not latch.complete
    latch.observe(15, [(2, 2)])
    assert latch.complete and latch.terminal_tick == 15
    assert latch.records() == [[1, 1, 10], [2, 2, 15], [3, 1, 12]]
    assert not SnapLatch(()).complete  # an empty expected fleet never completes
    partial = SnapLatch((1, 2))
    partial.observe(5, [(1, 1)])
    assert not partial.complete  # 2 never spawned, or retired without a SNAP


def test_a_bounded_baton_wait_gets_the_wall_deadline_and_is_classified_by_it():
    """``ExchangeDeadline.bounded``: the one path the coordinator's operator and
    10d's scripted operator share. The call gets ``D_k`` on the baton's clock;
    a baton fault or a return past ``D_k`` is ``exchange_budget``; a fault
    before it stays the call's own."""
    assert ExchangeDeadline().bounded("operator", lambda wall: wall) is None
    clock = FakeClock()
    deadline = ExchangeDeadline(STEP_S, MARGIN_S, clock=clock)
    deadline.anchor()
    deadline.open(1)
    wall = deadline.bounded("operator", lambda wall: wall)
    assert wall == pytest.approx(time.monotonic() + STEP_S - MARGIN_S, abs=1.0)

    def fails(spend_s: float):
        def call(_wall):
            clock.spend(spend_s)
            raise LockstepFault("handoff past its budget")
        return call

    with pytest.raises(LockstepFault) as early:
        deadline.bounded("operator", fails(1.0))
    assert not isinstance(early.value, HostFault)
    with pytest.raises(HostFault, match="operator wait") as late:
        deadline.bounded("operator", fails(STEP_S))
    assert isinstance(late.value.__cause__, LockstepFault)
    deadline.anchor()
    deadline.open(2)
    with pytest.raises(HostFault, match="operator wait"):
        deadline.bounded("operator", lambda _wall: clock.spend(STEP_S))  # returned too late
