"""The in-process ESP32 port answers as the loopback HTTP simulator would.

A trigger answers at once; the recorded producer thread then holds the
channel for the trigger duration on the lockstep clock, and a cancel (the
shutdown's) mid-hold releases it.
"""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager

from gcs.backend.esp32_simulator import TRIGGER_DURATION_MS, Esp32SimulatorState
from gcs.backend.lockstep import esp32_port
from gcs.backend.lockstep.esp32_port import InProcessEsp32
from gcs.backend.lockstep.faults import FaultLatch
from gcs.backend.lockstep.scopes import LockstepProducerScopes
from gcs.backend.producer_inventory import ProducerInventory
from navpy.core.kernel import runtime_time as rt
from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.lockstep_runtime import LockstepRuntime
from navpy.sim.lockstep.participant_hooks import OutboxBuffer


def test_a_client_reaches_the_simulator_only_while_it_runs():
    port = InProcessEsp32()
    client = port.client("127.0.0.1", None)
    assert not client.health_check() and client.get_status() is None
    assert client.reset_all() is False
    assert client.trigger_channel(1).message == "Connection failed"
    simulator = port.simulator(8032)
    simulator.start()
    assert simulator.port is None and client.health_check() and client.reset_all()
    assert client.get_status()["channels"] == [1, 2, 3, 4, 5, 6]
    simulator.stop()
    assert not client.health_check()


@contextmanager
def _lockstep(monkeypatch) -> Iterator[tuple[BatonRuntime, LockstepProducerScopes, OutboxBuffer]]:
    """A baton-bound runtime, a fresh simulator state, and lockstep producer scopes."""
    baton, out = BatonRuntime(), OutboxBuffer()
    binding = rt.RuntimeBinding()
    binding.bind(LockstepRuntime.create(baton, epoch_us=0, seed=1, ordinal=2))
    monkeypatch.setattr(rt, "_binding", binding)
    inventory = ProducerInventory()
    inventory.scopes = scopes = LockstepProducerScopes(FaultLatch(out))
    monkeypatch.setattr(esp32_port, "producers", inventory)
    monkeypatch.setattr(esp32_port, "_state", Esp32SimulatorState())
    yield baton, scopes, out


def _trigger(baton: BatonRuntime, channel: int) -> dict:
    port, box = InProcessEsp32(), {}

    def operator_call() -> None:
        port.simulator(8032).start()
        client = port.client("127.0.0.1", None)
        box["bad"] = client.trigger_channel(7)
        box["result"] = client.trigger_channel(channel)

    baton.spawn(operator_call, name="route-job")
    baton.run_until_quiescent()
    return box


def test_a_trigger_answers_at_once_and_holds_the_channel_on_a_tracked_thread(monkeypatch):
    with _lockstep(monkeypatch) as (baton, _scopes, out):
        box = _trigger(baton, 2)
        assert not box["bad"].success and box["bad"].message == "Invalid channel. Must be 1-6"
        result = box["result"]
        assert (result.success, result.channel, result.duration_ms) == (True, 2, TRIGGER_DURATION_MS)
        assert esp32_port._state.active_channel == 2  # the tracked thread holds it
        [thread] = esp32_port.producers.threads("esp32_simulator")
        baton.tick(TRIGGER_DURATION_MS * 1000)  # the hold ends at t = 0.5 s
        assert not thread.is_alive() and esp32_port._state.active_channel is None
        assert [e["channel"] for e in esp32_port._state.trigger_log] == [2]
        assert baton.threads.alive() == [] and not baton.threads.errors and out.faults == []


def test_a_cancel_mid_hold_releases_the_channel_and_retires_the_thread(monkeypatch):
    with _lockstep(monkeypatch) as (baton, scopes, out):
        _trigger(baton, 3)
        assert esp32_port._state.active_channel == 3
        baton.tick(100_000, lambda: scopes.cancel_all())  # the shutdown's cancel step, mid-hold
        assert esp32_port._state.active_channel is None
        assert baton.threads.alive() == [] and not baton.threads.errors and out.faults == []
