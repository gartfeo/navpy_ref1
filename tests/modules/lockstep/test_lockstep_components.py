"""Real product components on the LockstepRuntime repeat byte for byte under host delays.

``MessageStore`` waits, the ``CacheLogger`` (its written log file),
``TaskDispatch`` timers and ``ThreadLaunchGate`` run unchanged on ``rt``,
bound to a LockstepRuntime through a test-only fresh ``RuntimeBinding`` (the
process binding itself stays once-only). The same scripted ticks run plain
and with random real host sleeps in every handoff, thread start and tick.
"""

from __future__ import annotations

import logging
import random
import time as host_time
from pathlib import Path

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.config.logger_args import LoggerArgsStub
from navpy.core.kernel.logging.logger_factory import LOG_DIR_ENV, initialize_logger
from navpy.core.kernel.thread_launch import ThreadLaunchGate
from navpy.core.link.location_msg import LocationMsgData
from navpy.core.link.task_message_data import TaskMsgData
from navpy.core.link.types import TaskTypeMsgData
from navpy.core.swarm.task_dispatch import TaskDispatch
from navpy.plugins.vehicle.ardupilot.message_store import MessageStore
from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.baton_queue import BatonQueue
from navpy.sim.lockstep.lockstep_runtime import LockstepRuntime
from navpy.sim.lockstep.log_stamp import install_log_stamp

TICK_US = 20_000
TICKS = 20
EPOCH_US = 1_760_000_000_123_456  # 2025-10-09T08:53:20.123456Z
SYS_ID = 9
#: COMMAND_ACKs the coordinator delivers to the store's reader, by tick.
INPUTS = {2: (1,), 3: (2,), 5: (3, 4)}


def _ack(command: int) -> mavlink.MAVLink_command_ack_message:
    return mavlink.MAVLink_command_ack_message(command, 0, 0, 0, 0, 0)


class _Scenario:
    """The product side: runs on managed threads, notes each step with its tick."""

    def __init__(self, baton: BatonRuntime, inbound: BatonQueue) -> None:
        self._baton = baton
        self._inbound = inbound
        self.trace: list[tuple[int, str]] = []
        self.logger = None

    def note(self, what: str) -> None:
        self.trace.append((self._baton.now_us // TICK_US, what))
        self.logger.info(what)

    def setup(self) -> None:
        self.logger = initialize_logger(LoggerArgsStub(), SYS_ID)
        store = MessageStore({"COMMAND_ACK": 4})
        for name, target, args in (
            ("reader", self._reader, (store,)),
            ("waiter", self._waiter, (store,)),
            ("dispatch", self._dispatch, ()),
            ("gates", self._gates, ()),
        ):
            rt.Thread(target=target, args=args, name=name, daemon=True).start()

    def _reader(self, store: MessageStore) -> None:
        while True:
            message = self._inbound.get()
            sample = store.publish("COMMAND_ACK", message, receipt_time_s=rt.monotonic())
            self.note(f"published command {message.command} as {sample.generation}")

    def _waiter(self, store: MessageStore) -> None:
        found = store.wait_after(
            "COMMAND_ACK", store.cursor(), lambda m: m.command == 2, deadline=rt.monotonic() + 0.1,
        )
        self.note(f"command 2 arrived as {found.generation} at {found.receipt_time_s}")
        missing = store.wait_after(
            "COMMAND_ACK", found.generation, lambda m: m.command == 9, deadline=rt.monotonic() + 0.05,
        )
        self.note(f"command 9: {missing}")

    def _dispatch(self) -> None:
        dispatch = TaskDispatch(TaskMsgData(
            task_id=5, task_type=TaskTypeMsgData.DOCK, location=LocationMsgData(1.0, 2.0, 3.0),
        ))
        rounds = []

        def rebroadcast(task_id: int) -> None:
            rounds.append(task_id)
            self.note(f"rebroadcast {task_id} round {len(rounds)}")
            if len(rounds) < 3:
                dispatch.start_rebroadcast(rebroadcast, 0.03)

        dispatch.start_peer_select_timer(lambda task_id: self.note(f"select peer for {task_id}"), 0.05)
        dispatch.start_rebroadcast(rebroadcast, 0.03)
        dispatch.start_confirm_timer(lambda task_id: self.note("confirm fired"), 0.2)
        rt.sleep(0.1)
        dispatch.cancel_confirm_timer()
        self.note(f"confirm cancelled; rebroadcast active: {dispatch.has_active_rebroadcast()}")
        dispatch.shutdown()

    def _gates(self) -> None:
        committed, abandoned = ThreadLaunchGate(), ThreadLaunchGate()

        def target(gate: ThreadLaunchGate, label: str) -> None:
            rt.sleep(0.04)
            self.note(f"{label} enter: {gate.enter()}")

        rt.Thread(target=target, args=(committed, "committed"), daemon=True).start()
        rt.Thread(target=target, args=(abandoned, "abandoned"), daemon=True).start()
        self.note(f"abandoned cancelled: {abandoned.cancel_before_commit()}")
        self.note(f"entered by 0.02 s: {committed.wait_until_entered(0.02)}")
        self.note(f"entered by 0.1 s: {committed.wait_until_entered(0.1)}")
        self.note(f"committed: {committed.committed}, abandoned: {abandoned.committed}")


def _inject_host_delays(mp: pytest.MonkeyPatch, rng: random.Random) -> None:
    """Real host sleeps before every park and at every managed thread's start."""
    park, spawn = BatonRuntime.park, BatonRuntime.spawn

    def delayed_park(self, reason, timeout_s=None, **kwargs):
        host_time.sleep(rng.random() / 2000)
        park(self, reason, timeout_s, **kwargs)

    def delayed_spawn(self, target, *, name):
        def delayed_target():
            host_time.sleep(rng.random() / 2000)
            target()
        return spawn(self, delayed_target, name=name)

    mp.setattr(BatonRuntime, "park", delayed_park)
    mp.setattr(BatonRuntime, "spawn", delayed_spawn)


def _run(log_dir: Path, jitter_seed: int | None) -> tuple[list[tuple[int, str]], bytes]:
    baton = BatonRuntime()
    runtime = LockstepRuntime.create(baton, epoch_us=EPOCH_US, seed=3, ordinal=1)
    binding = rt.RuntimeBinding()
    binding.bind(runtime)
    inbound = BatonQueue(baton)
    scenario = _Scenario(baton, inbound)
    rng = random.Random(jitter_seed)
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(rt, "_binding", binding)
        mp.setenv(LOG_DIR_ENV, str(log_dir))
        if jitter_seed is not None:
            _inject_host_delays(mp, rng)
        uninstall_stamp = install_log_stamp(runtime.clock)
        try:
            baton.spawn(scenario.setup, name="setup")
            baton.run_until_quiescent()
            for k in range(1, TICKS + 1):
                commands = INPUTS.get(k, ())
                baton.tick(k * TICK_US, lambda c=commands: [inbound.deliver(_ack(n)) for n in c])
                if jitter_seed is not None:
                    host_time.sleep(rng.random() / 1000)
            baton.spawn(scenario.logger.close, name="close")
            baton.run_until_quiescent()
            baton.abort()  # unwind the parked reader
        finally:
            uninstall_stamp()
            for handler in logging.getLogger(f"vehicle_logger_{SYS_ID}").handlers:
                handler.close()
    return scenario.trace, (log_dir / f"uav_{SYS_ID}_navigation.log").read_bytes()


def test_real_components_repeat_byte_for_byte_under_host_delays(tmp_path):
    trace, log_bytes = _run(tmp_path / "plain", None)
    for seed in (1, 2):
        assert _run(tmp_path / f"jitter{seed}", seed) == (trace, log_bytes), seed

    # Inputs run before timers; due timers fire in (due, park seq) order.
    assert trace == [
        (0, "abandoned cancelled: True"),
        (1, "entered by 0.02 s: False"),
        (2, "published command 1 as 1"),
        (2, "rebroadcast 5 round 1"),  # due 0.03, before the 0.04 sleeps
        (2, "committed enter: True"),
        (2, "abandoned enter: False"),
        (2, "entered by 0.1 s: True"),
        (2, "committed: True, abandoned: False"),
        (3, "published command 2 as 2"),
        (3, "command 2 arrived as 2 at 0.06"),
        (3, "select peer for 5"),  # due 0.05: the input at tick 3 ran first
        (4, "rebroadcast 5 round 2"),  # due 0.07
        (5, "published command 3 as 3"),
        (5, "published command 4 as 4"),
        (5, "confirm cancelled; rebroadcast active: True"),  # shutdown: no round 3
        (6, "command 9: None"),  # deadline 0.11
    ]
    lines = log_bytes.decode("utf-8").splitlines()
    # asctime is t_k in UTC: tick 2 is 08:53:20.163456, printed as ,163.
    assert lines[2] == "2025-10-09 08:53:20,163 INFO published command 1 as 1"
    assert lines[-1] == "2025-10-09 08:53:20,523 INFO Logger closed"
