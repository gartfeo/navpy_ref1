"""The backend's listeners on the merged router link, and their fence (10f).

The backend's real ``VehicleManager`` connects vehicles 1-3 on one
``lockstep:router`` link, with the real ``TaskConfirmListener`` and
``TaskAssignListener``, on a baton asyncio loop (``lockstep_fleet.md``, Peer
camera and image transfer; item 7). Three reference autopilots answer their
own vehicle; three companions (sysid i, component 191) send through a real
``NetworkMavlink``, image pacing included (``rt.sleep``: one tick per chunk
on a 10 ms grid). All of it reaches the backend merged into one byte stream,
as the router delivers it: the AP hop is byte-identical (10f-1), so a
companion's bytes enter the stream as sent, one tick later.

- **Sender isolation:** the three companions send their confirmation images
  for their own task 1 in the same ticks, chunk for chunk (a chunk carries no
  task id); each image is reassembled for its own vehicle, byte for byte.
- **More than one vehicle:** a peer's advert is shown once, by the listener
  that owns its sender (``task_assign_listener.py``, the one-vehicle check).
- **The fence:** an image in flight when the fence closes admission never
  completes, a later advert or confirmation request makes no broadcast, and
  no producer is refused (``ProducerFenced``): the subscriptions are gone.
"""

from __future__ import annotations

import asyncio
import base64
import contextvars
import hashlib
import random
import time as host_time
from contextlib import nullcontext

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from gcs.backend.broadcast import ConnectionManager
from gcs.backend.full_params import FullParamCache
from gcs.backend.lockstep.fence import fence
from gcs.backend.producer_inventory import ProducerInventory
from gcs.backend.task_assign_listener import TaskAssignListener
from gcs.backend.task_confirm_listener import TaskConfirmListener
from gcs.backend.vehicle_manager import VehicleManager
from navpy.core.kernel.logging.logger_api import ConsoleLogger
from navpy.core.link.location_msg import LocationMsgData
from navpy.core.link.task_availability_msg import AvailableTaskRequestMsg
from navpy.core.link.task_confirmation_msg import TaskConfirmRequestMsg
from navpy.core.link.task_message_data import TaskMsgData
from navpy.core.link.types import TaskTypeMsgData
from navpy.core.ports.subscription import Subscription
from navpy.plugins.radio.mavlink.network_mavlink import NetworkMavlink
from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.baton_asyncio import BatonEventLoop
from navpy.sim.lockstep.lockstep_link import LockstepLink
from navpy.sim.lockstep.mavlink_wire import StreamDecoder, encoder
from tests.modules.lockstep.lockstep_autopilot import ReferenceAutopilot
from tests.modules.lockstep.swarm_baton import bind_message_locks
from tests.modules.lockstep.test_lockstep_bus import _lockstep

TICK_US = 10_000
ROUTER = "lockstep:router"
SYS_IDS = (1, 2, 3)
#: Ticks of the scenario (10 ms each).
K_SEND, K_ADVERT, K_STRADDLE, K_FENCE, K_LATE, K_END = 300, 400, 496, 500, 520, 600
#: Modules that hold the backend's producer inventory.
PRODUCER_MODULES = ("task_confirm_listener", "task_assign_listener", "vehicle_entry",
                    "full_param_store", "vehicle_mission_probe")
LISTENERS = ("task_confirm_listener", "task_assign_listener")
#: Each owner numbers its own tasks: three vehicles confirm their task 1 at once.
SHARED_TASK = 1


def _image(sys_id: int, task_id: int) -> str:
    """A distinct payload per sender: 5-8 chunks of 253 bytes."""
    size = 1100 + 250 * sys_id + task_id
    data = b"".join(hashlib.sha256(f"{sys_id}:{task_id}:{n}".encode()).digest() for n in range(size // 32 + 1))
    return base64.b64encode(data[:size]).decode("ascii")


def _task(task_id: int) -> TaskMsgData:
    return TaskMsgData(task_id=task_id, task_type=TaskTypeMsgData.DOCK,
                       location=LocationMsgData(40.29 + task_id / 1000, 44.43, 1340.0))


class _Companion:
    """NavPy's NAVLINK and image port: bytes from (sysid, 191) to the merged stream."""

    def __init__(self, sys_id: int) -> None:
        self.source_system = sys_id
        self._link, self.sink = encoder(sys_id, mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)

    def on_message(self, _name: str, _callback) -> Subscription:
        return Subscription(lambda: None)  # nothing comes back in these scenarios

    def send_mavlink_message(self, message: mavlink.MAVLink_message) -> None:
        self._link.send(message)


class _Scopes:
    """The lockstep backend's producer scopes, as far as the listeners use them."""

    def __init__(self, faults: list) -> None:
        self._faults = faults

    def context(self, _name: str) -> contextvars.Context:
        return contextvars.copy_context()

    def entered(self, _name: str):
        return nullcontext()

    def observe(self, producer: str, done) -> None:
        if done.cancelled() or done.exception() is not None:
            self._faults.append(f"{producer}: {done}")


class _Run:
    """The coordinator side: ticks, the merged stream, the autopilots, the journal."""

    def __init__(self, baton: BatonRuntime) -> None:
        self.baton = baton
        self.link = LockstepLink(baton, ROUTER)
        self.autopilots = {i: ReferenceAutopilot(sys_id=i) for i in SYS_IDS}
        self.companions = {i: _Companion(i) for i in SYS_IDS}
        self.networks: dict[int, NetworkMavlink] = {}
        self._backend_decoder = StreamDecoder()
        self._ingress_decoder = StreamDecoder()
        self.trace: list[tuple[int, str]] = []
        self.ingress: list[tuple[int, str, int]] = []  # (tick, type, source system)
        self.faults: list[str] = []
        self.inventory = ProducerInventory()
        self.inventory.scopes = _Scopes(self.faults)
        self.done = False

    @property
    def k(self) -> int:
        return self.baton.now_us // TICK_US

    def note(self, what: str) -> None:
        self.trace.append((self.k, what))

    # -- the backend (its loop thread) ------------------------------------------
    def backend(self) -> None:
        loop = BatonEventLoop(self.baton)
        loop.job_failures.report = lambda error: self.faults.append(f"job: {error!r}")
        try:
            loop.run_until_complete(self._main())
        finally:
            loop.close()
            self.done = True

    async def _main(self) -> None:
        loop = asyncio.get_running_loop()
        ws = ConnectionManager()
        ws.observer = lambda payload: self.note(f"ws {_summary(payload)}")
        for module in ("task_confirm_listener", "task_assign_listener"):
            self._mp.setattr(f"gcs.backend.{module}.ws_manager", ws)
        manager = VehicleManager(open_bus=self.link.open)
        manager._probe_mission_async = lambda entry: None  # a fenced producer of its own (9e)
        manager.set_task_confirm_listener(TaskConfirmListener(loop))
        manager.set_task_assign_listener(TaskAssignListener(loop))
        for sys_id in SYS_IDS:
            await loop.run_in_executor(None, manager.add_vehicle, ROUTER, sys_id)
        self.note("connected 1-3")
        await asyncio.sleep(K_FENCE * TICK_US / 1e6 - loop.time())
        await fence(self.inventory)
        held = [p for p, _s in self.inventory._subscriptions if p in LISTENERS]
        self.note(f"fenced; listener subscriptions left: {len(held)}")
        await asyncio.sleep(K_END * TICK_US / 1e6 - loop.time())
        for sys_id in SYS_IDS:
            await loop.run_in_executor(None, manager.remove_vehicle, sys_id)

    # -- the companions (managed threads) -----------------------------------------
    def setup_companions(self) -> None:
        for sys_id, port in self.companions.items():
            self.networks[sys_id] = NetworkMavlink(sys_id, port, ConsoleLogger())

    def confirm(self, sys_id: int, task_id: int):
        def send() -> None:
            network = self.networks[sys_id]
            network.broadcast(TaskConfirmRequestMsg(sys_id, _task(task_id)))
            chunks = network.send_image(task_id, _image(sys_id, task_id))
            self.note(f"companion {sys_id} sent image {task_id} in {chunks} chunks")
        return send

    def advert(self, sys_id: int, task_id: int):
        return lambda: self.networks[sys_id].broadcast(AvailableTaskRequestMsg(sys_id, [_task(task_id)]))

    # -- the tick loop -------------------------------------------------------------
    def run(self, mp: pytest.MonkeyPatch, rng: random.Random | None) -> None:
        self._mp = mp
        broadcast = self.inventory.broadcast

        def noted_broadcast(producer, coroutine, loop):
            self.note(f"broadcast by {producer}")
            return broadcast(producer, coroutine, loop)

        mp.setattr(self.inventory, "broadcast", noted_broadcast)
        # remove_vehicle forgets the vehicle in this singleton: a fresh one, made
        # on the baton (its lock is the baton's), restored after the run.
        mp.setattr("gcs.backend.full_params.full_param_cache", FullParamCache())
        for module in PRODUCER_MODULES:
            mp.setattr(f"gcs.backend.{module}.producers", self.inventory)
        actions = {K_SEND: [self.confirm(i, SHARED_TASK) for i in SYS_IDS],
                   K_ADVERT: [self.advert(2, 30)],
                   K_STRADDLE: [self.confirm(1, 21)],
                   K_LATE: [self.advert(3, 31), self.confirm(3, 23)]}
        self.baton.spawn(self.setup_companions, name="companions")
        self.baton.spawn(self.backend, name="backend-loop")
        pending = b""
        for k in range(1, K_END + 200):
            self.baton.tick(k * TICK_US, lambda data=pending: data and self.link.deliver(data))
            for index, action in enumerate(actions.get(k, ())):
                self.baton.spawn(action, name=f"companion-{k}-{index}")
            self.baton.run_until_quiescent()
            pending = self._step(k)
            if rng is not None:
                host_time.sleep(rng.random() / 2000)
            if self.done:
                break
        assert self.done, "the backend never finished"
        if self.baton.threads.errors:
            raise self.baton.threads.errors[0]

    def _step(self, k: int) -> bytes:
        """Route the backend's egress to the autopilots; merge what comes back."""
        inbound = {i: [] for i in SYS_IDS}
        for message in self._backend_decoder.feed(self.link.drain_egress()):
            target = getattr(message, "target_system", 0)
            for i in SYS_IDS:
                if target in (0, i):
                    inbound[i].append(message.get_msgbuf())
        merged = [self.autopilots[i].step(k * TICK_US, [b"", b"".join(inbound[i])])[1] for i in SYS_IDS]
        merged += [self.companions[i].sink.drain() for i in SYS_IDS]
        data = b"".join(merged)
        for message in self._ingress_decoder.feed(data):
            self.ingress.append((k + 1, message.get_type(), message.get_srcSystem()))
        return data


def _summary(payload: dict) -> str:
    kind = payload["type"]
    if kind == "task_confirm_image":
        digest = hashlib.sha256(payload["image_b64"].encode()).hexdigest()[:12]
        return f"{kind} sys {payload['sys_id']} task {payload['task_id']} sha {digest}"
    if kind == "task_confirm_request":
        return f"{kind} sys {payload['sys_id']} task {payload['task_id']}"
    if kind == "available_task_request":
        return f"{kind} sender {payload['sender_id']} tasks {[t['task_id'] for t in payload['tasks']]}"
    return kind


def _scenario(jitter_seed: int | None) -> _Run:
    with _lockstep(jitter_seed) as (baton, rng):
        run = _Run(baton)
        try:
            with pytest.MonkeyPatch.context() as mp:
                bind_message_locks(mp)
                run.run(mp, rng)
        finally:
            baton.abort()
    return run


def _ws(run: _Run) -> list[tuple[int, str]]:
    return [(k, what[3:]) for k, what in run.trace if what.startswith("ws ")]


def _sha(sys_id: int, task_id: int) -> str:
    return hashlib.sha256(_image(sys_id, task_id).encode()).hexdigest()[:12]


def test_three_images_on_the_merged_link_reassemble_per_sender_and_the_fence_stops_the_listeners():
    run = _scenario(None)
    assert run.faults == []
    connected = [k for k, what in run.trace if what == "connected 1-3"]
    assert connected and connected[0] < K_SEND

    # The three images crossed the link interleaved: chunks of all three senders in one tick.
    chunks = {}
    for k, kind, source in run.ingress:
        if kind == "ENCAPSULATED_DATA":
            chunks.setdefault(k, set()).add(source)
    assert any(sources == set(SYS_IDS) for sources in chunks.values())

    shown = [what for _k, what in _ws(run)]
    for sys_id in SYS_IDS:  # each vehicle's image, byte for byte, under its own sysid
        assert f"task_confirm_request sys {sys_id} task {SHARED_TASK}" in shown
        assert f"task_confirm_image sys {sys_id} task {SHARED_TASK} sha {_sha(sys_id, SHARED_TASK)}" in shown
    # More than one vehicle: the peer's advert is shown once, by its owner's listener.
    assert shown.count("available_task_request sender 2 tasks [30]") == 1

    # The fence: the straddling image never completes; nothing after it is shown or refused.
    fenced = [k for k, what in run.trace if what.startswith("fenced")]
    assert fenced == [K_FENCE]
    assert (K_FENCE, "fenced; listener subscriptions left: 0") in run.trace
    assert "task_confirm_request sys 1 task 21" in shown
    assert not any("task 21 sha" in what for what in shown)
    assert all(k <= K_FENCE for k, what in run.trace if what.startswith("broadcast by"))
    assert not any("sys 3 task 23" in what or "tasks [31]" in what for what in shown)
    late = {(kind, source) for k, kind, source in run.ingress if k > K_LATE}
    assert {("AVAILABLE_TASK_REQUEST", 3), ("TASK_CONFIRM_REQUEST", 3)} <= late  # they did arrive


def test_the_merged_link_run_repeats_under_host_delays():
    plain = _scenario(None)
    for seed in (1, 2):
        jittered = _scenario(seed)
        assert jittered.trace == plain.trace and jittered.ingress == plain.ingress, seed
