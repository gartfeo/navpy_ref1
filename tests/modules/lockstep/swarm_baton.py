"""Three real swarm nodes on one baton: the auction under lockstep (10f).

Each node is a real ``TaskActor`` on a real ``NetworkMavlink`` (with its
``MessageFilter``), bound to a :class:`LockstepRuntime` through a test-only
fresh ``RuntimeBinding``, as ``test_lockstep_components`` binds it: every
lock (the message classes' too), timer, heartbeat thread and clock read
is the baton's. A node's sends
are encoded as MAVLink bytes from its companion (sysid, 191) and reach each
addressee :data:`HOP_TICKS` ticks later (the NavPy-to-NavPy hop measured in
10f-1), through the addressee's own reader thread: a managed thread blocked
on a baton queue, as the MavBus reader is. Deliveries are tick inputs, so on
a deadline tick the input runs before the timer.

A rule can drop a message or move its delivery to a chosen tick. The trace
records each node's log line, each send and each delivery with its tick, and
repeats under host delays.

One process holds the three nodes, so they share the ``MsgMetaProvider``
(one boot id; message sequence numbers interleave but stay increasing per
sender, which is all ``MsgRef.follows`` reads).
"""

from __future__ import annotations

import random
import time as host_time
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from types import SimpleNamespace

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.location import Location
from navpy.core.link.msg_abc import MsgABC, MsgRegistry
from navpy.core.link.msg_meta import MsgMetaProvider
from navpy.core.ports.subscription import Subscription
from navpy.core.swarm.task_actor import TaskActor
from navpy.core.swarm.task_auction_coordinator import TaskAuctionCoordinator
from navpy.plugins.radio.mavlink.network_mavlink import NetworkMavlink
from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.baton_queue import BatonQueue
from navpy.sim.lockstep.lockstep_runtime import LockstepRuntime
from navpy.sim.lockstep.mavlink_wire import StreamDecoder, encoder
from tests.detection_factory import make_detected_poi
from tests.modules.lockstep.test_lockstep_components import _inject_host_delays

TICK_US = 10_000
#: NavPy i's emission to NavPy j's delivery, in ticks (10f-1's traces).
HOP_TICKS = 4
EPOCH_US = 1_760_000_000_000_000
OWNER, NEAR_PEER, FAR_PEER = 1, 2, 3
NODES = (OWNER, NEAR_PEER, FAR_PEER)
POI = Location(40.2967648, 44.4332133, 1339.7, is_absolute=True)
LOCATIONS = {
    OWNER: Location(40.2961244, 44.4334705, 1339.7),
    NEAR_PEER: Location(40.2967648, 44.4332133, 1339.7),
    FAR_PEER: Location(40.2974053, 44.4329561, 1339.7),
}
#: A tick past the first heartbeats (1.0 s) and their delivery.
DETECT_TICK = 150


def tick_of(seconds: float) -> int:
    return round(seconds * 1_000_000 / TICK_US)


class _Logger:
    """The node's ``ILogger``: each line goes into the trace with its tick."""

    def __init__(self, note: Callable[[str], None]) -> None:
        for level in ("debug", "info", "warning", "error"):
            setattr(self, level, lambda msg, *_a, _level=level, **_k: note(f"{_level}: {msg}"))


class _Vehicle:
    """What the actor reads of its aircraft (ETA inputs) and its NAVLINK port."""

    def __init__(self, node_id: int, swarm: "BatonSwarm") -> None:
        self.source_system = node_id
        self._location = LOCATIONS[node_id]
        self._swarm = swarm
        self._link, self._sink = encoder(node_id, mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
        self.ground_speed = 20.0
        self.wind = SimpleNamespace(speed=0.0, direction=0.0)
        self.battery_level = 100.0

    def on_message(self, _name: str, _callback) -> Subscription:
        return Subscription(lambda: None)  # the swarm delivers to the network directly

    def send_mavlink_message(self, message: mavlink.MAVLink_message) -> None:
        self._link.send(message)
        self._swarm.sent(self.source_system, message, self._sink.drain())

    def location(self, _relative: bool = False) -> Location:
        return self._location


@dataclass(frozen=True)
class Sent:
    """One message a node sent: its tick, sender, decoded message and bytes."""

    k: int
    sender: int
    message: MsgABC
    data: bytes
    target: int


#: A delivery rule: None keeps the hop, ``DROP`` loses it, an int is the delivery tick.
DROP = -1
Rule = Callable[[Sent, int], "int | None"]


class BatonSwarm:
    """Owner 1 and peers 2 (nearest the POI) and 3, on one baton."""

    def __init__(self, baton: BatonRuntime) -> None:
        self.baton = baton
        self.trace: list[tuple[int, int, str]] = []
        self.log: list[Sent] = []
        self.rule: Rule = lambda _sent, _receiver: None
        #: Harness errors met on product threads (which may swallow a raise).
        self.faults: list[str] = []
        self._due: dict[int, list[tuple[int, bytes]]] = {}
        self._inboxes = {node: BatonQueue(baton) for node in NODES}
        self._decoders = {node: StreamDecoder() for node in NODES}
        self.vehicles: dict[int, _Vehicle] = {}
        self.networks: dict[int, NetworkMavlink] = {}
        self.actors: dict[int, TaskActor] = {}

    @property
    def k(self) -> int:
        return self.baton.now_us // TICK_US

    @property
    def owner(self) -> TaskActor:
        return self.actors[OWNER]

    def note(self, node: int, what: str) -> None:
        self.trace.append((self.k, node, what))

    def lines(self, node: int, prefix: str = "") -> list[tuple[int, str]]:
        return [(k, what) for k, who, what in self.trace if who == node and what.startswith(prefix)]

    # -- product side (managed threads) -------------------------------------
    def setup(self, *, start: bool = True) -> None:
        for node in NODES:
            vehicle = _Vehicle(node, self)
            network = NetworkMavlink(node, vehicle, _Logger(lambda what, n=node: self.note(n, what)))
            self.vehicles[node], self.networks[node] = vehicle, network
            rt.Thread(target=self._reader, args=(node,), name=f"reader{node}", daemon=True).start()
            if start:
                actor = TaskActor(vehicle, network, network.logger)
                network.set_listener(actor)
                self.actors[node] = actor
                actor.start()

    def _reader(self, node: int) -> None:
        while True:
            message = self._inboxes[node].get()
            self.networks[node]._on_mavlink(message)

    def sent(self, sender: int, message: mavlink.MAVLink_message, data: bytes) -> None:
        decoded = MsgRegistry.get_class_by_mav_id(message.get_msgId())
        navpy = decoded.from_mavlink(message) if decoded is not None else None
        target = getattr(message, "target_system", 0)
        record = Sent(self.k, sender, navpy, data, target)
        self.log.append(record)
        self.note(sender, f"sent {type(navpy).__name__}")
        for receiver in NODES:
            if receiver == sender or target not in (0, receiver):
                continue
            due = self.rule(record, receiver)
            if due == DROP:
                continue
            due = self.k + HOP_TICKS if due is None else due
            if due <= self.k:  # recorded here: a product thread may swallow a raise
                self.faults.append(f"a delivery at {due} is not after its send at {self.k}")
                continue
            self._due.setdefault(due, []).append((receiver, data))

    def detect(self, *task_ids: int) -> None:
        self.owner.notify_pois([
            make_detected_poi(task_id=task_id, obj_id=task_id, p_t_g_l=POI, class_id=0)
            for task_id in task_ids
        ])

    # -- coordinator side ---------------------------------------------------
    def inject(self, k: int, receiver: int, data: bytes) -> None:
        """Bytes a test builds itself, delivered to ``receiver`` at tick ``k``."""
        self._due.setdefault(k, []).append((receiver, data))

    def _deliver(self, k: int) -> None:
        for receiver, data in self._due.pop(k, ()):
            for message in self._decoders[receiver].feed(data):
                self.note(receiver, f"delivered {message.get_type()} from {message.get_srcSystem()}")
                self._inboxes[receiver].deliver(message)

    def run(self, until_k: int, actions: dict[int, Callable[[], None]] | None = None,
            rng: random.Random | None = None) -> None:
        """Ticks up to ``until_k``; an action is spawned at its tick, after the
        tick's inputs, as a managed thread (an operator or nav step)."""
        actions = actions or {}
        for k in range(self.k + 1, until_k + 1):
            self.baton.tick(k * TICK_US, lambda k=k: self._deliver(k))
            if k in actions:
                self.baton.spawn(actions[k], name=f"action{k}")
                self.baton.run_until_quiescent()
            if rng is not None:
                host_time.sleep(rng.random() / 2000)
        if self.baton.threads.errors:
            raise self.baton.threads.errors[0]
        assert self.faults == []

    def query(self, read: Callable[[], object]) -> object:
        """Read product state on a managed thread (its locks are the baton's)."""
        out: list = []
        self.baton.spawn(lambda: out.append(read()), name="query")
        self.baton.run_until_quiescent()
        return out[0]

    def requests(self, kind: type, sender: int | None = None, **fields) -> list[Sent]:
        return [
            s for s in self.log
            if isinstance(s.message, kind) and sender in (None, s.sender)
            and all(getattr(s.message, name) == value for name, value in fields.items())
        ]


def bind_message_locks(mp: pytest.MonkeyPatch) -> None:
    """The message classes' locks are made at import, on the wall runtime: give
    them baton locks, and a fresh ``MsgMetaProvider`` (its boot id from the
    run's entropy) without taking a wall lock; ``mp`` restores all three."""
    mp.setattr(MsgMetaProvider, "_lock", rt.Lock())
    mp.setattr(MsgMetaProvider, "_instance", None)
    mp.setattr(MsgRegistry, "_registry_lock", rt.Lock())


def timer_spy(mp: pytest.MonkeyPatch, swarm: BatonSwarm) -> None:
    """Note the owner's 2 s selection timer when it fires (it logs nothing itself)."""
    select = TaskAuctionCoordinator._select_peer_for_task

    def noted(self, task_id: int, generation: int) -> None:
        swarm.note(OWNER, f"selection timer for task {task_id}")
        select(self, task_id, generation)

    mp.setattr(TaskAuctionCoordinator, "_select_peer_for_task", noted)


@contextmanager
def baton_swarm(jitter_seed: int | None = None, *, start: bool = True) -> Iterator[BatonSwarm]:
    baton = BatonRuntime()
    binding = rt.RuntimeBinding()
    binding.bind(LockstepRuntime.create(baton, epoch_us=EPOCH_US, seed=1, ordinal=1))
    swarm = BatonSwarm(baton)
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(rt, "_binding", binding)
        if jitter_seed is not None:
            _inject_host_delays(mp, random.Random(jitter_seed))
        timer_spy(mp, swarm)
        bind_message_locks(mp)
        try:
            baton.spawn(lambda: swarm.setup(start=start), name="setup")
            baton.run_until_quiescent()
            yield swarm
            if swarm.actors:
                baton.spawn(lambda: [actor.shutdown() for actor in swarm.actors.values()], name="stop")
                baton.run_until_quiescent()
        finally:
            baton.abort()  # unwind the parked readers


__all__ = [
    "DETECT_TICK", "DROP", "FAR_PEER", "HOP_TICKS", "NEAR_PEER", "NODES", "OWNER",
    "BatonSwarm", "Sent", "baton_swarm", "bind_message_locks", "tick_of",
]
