"""The in-process ESP32 port answers as the loopback HTTP simulator would.

A trigger answers at once; the recorded producer thread then holds the
channel for the trigger duration on the lockstep clock, and a cancel (the
shutdown's) mid-hold releases it. The fleet's vehicles 1-3 resolve to
channels 1-3 and share the one simulator (10e): their holds may overlap,
each on its own thread for its own duration.
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
from gcs.backend.routes.control_readiness import _resolve_channel_map
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


def test_the_fleet_sysids_resolve_to_channels_one_to_three():
    # The lockstep settings leave the default map empty (``build_settings``).
    assert _resolve_channel_map({}, [1, 2, 3]) == {1: 1, 2: 2, 3: 3}


def _trigger_at(baton: BatonRuntime, port: InProcessEsp32, now_us: int, channel: int, box: dict) -> None:
    def operator_call() -> None:
        box[channel] = port.client("127.0.0.1", None).trigger_channel(channel)

    baton.tick(now_us, lambda: baton.spawn(operator_call, name=f"route-job-{channel}"))


def _overlapping(baton: BatonRuntime) -> dict:
    """Channels 1, 2 and 3 triggered 0.1 s apart: every hold overlaps the next."""
    port, box = InProcessEsp32(), {}
    baton.spawn(lambda: port.simulator(8032).start(), name="route-job")
    baton.run_until_quiescent()
    for index, channel in enumerate((1, 2, 3)):
        _trigger_at(baton, port, index * 100_000, channel, box)
    return box


def test_overlapping_holds_on_channels_one_to_three_each_last_their_own_duration(monkeypatch):
    with _lockstep(monkeypatch) as (baton, _scopes, out):
        box = _overlapping(baton)
        assert [(box[c].success, box[c].channel) for c in (1, 2, 3)] == [(True, 1), (True, 2), (True, 3)]
        threads = dict(zip((1, 2, 3), esp32_port.producers.threads("esp32_simulator")))
        assert all(thread.is_alive() for thread in threads.values())
        hold_us = TRIGGER_DURATION_MS * 1000
        for index, channel in enumerate((1, 2, 3)):
            baton.tick(index * 100_000 + hold_us)  # channel's own hold ends; later ones still hold
            assert [c for c, thread in threads.items() if thread.is_alive()] == [1, 2, 3][index + 1:]
        log = esp32_port._state.trigger_log
        assert [entry["channel"] for entry in log] == [1, 2, 3]
        # The wall simulator's one ``active`` slot: the latest trigger, cleared by the first end.
        assert esp32_port._state.active_channel is None
        assert baton.threads.alive() == [] and not baton.threads.errors and out.faults == []


def test_a_cancel_during_overlapping_holds_releases_every_channel(monkeypatch):
    with _lockstep(monkeypatch) as (baton, scopes, out):
        _overlapping(baton)
        assert len(esp32_port.producers.threads("esp32_simulator")) == 3
        baton.tick(300_000, lambda: scopes.cancel_all())  # the shutdown's cancel, all three holding
        assert esp32_port._state.active_channel is None
        assert baton.threads.alive() == [] and not baton.threads.errors and out.faults == []
