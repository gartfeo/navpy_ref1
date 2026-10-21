"""Sim closest-approach records in the GCS backend (gcs_sim_closest_approach.md)."""

from __future__ import annotations

import asyncio
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from gcs.backend import navpy_sim_runtime as runtime_mod
from gcs.backend.navpy_process_manager import NavpyInstance, NavpyProcessManager
from gcs.backend.routes import navpy_sim as navpy_mod
from gcs.backend.sim_closest_approach_store import (
    SIM_CLOSEST_APPROACH_KEEP,
    SimClosestApproachStore,
)
from navpy.sim.world.closest_approach_record import (
    OUTCOME_MISSED,
    encode,
    from_mapping,
)


def _record(vehicle=2, episode=1, d3_m=1.84):
    return from_mapping(dict(
        vehicle=vehicle, task_id=7, dock_uid=3, episode=episode,
        outcome=OUTCOME_MISSED, aircraft_time_s=512.3, lat=40.0, lon=44.0,
        alt_amsl_m=101.0, d3_m=d3_m, h_m=1.52, v_m=1.04, dock_lat=40.00001,
        dock_lon=44.00001, dock_alt_amsl_m=100.0, alt_rel_m=41.0,
        dock_alt_rel_m=40.0, roll_deg=-3.0, pitch_deg=2.0, yaw_deg=91.0,
    ))


def test_store_keeps_an_own_record_with_its_key():
    store = SimClosestApproachStore(boot_id="b1")
    assert store.offer(2, 5, encode(_record()))
    [entry] = store.snapshot()
    assert entry["key"] == "b1:5:1"
    assert entry["sys_id"] == 2
    assert entry["d3_m"] == 1.84 and entry["outcome"] == OUTCOME_MISSED
    assert entry["dock_alt_rel_m"] == 40.0 and entry["yaw_deg"] == 91.0  # flat wire keys


def test_store_ignores_other_lines_and_drops_another_vehicles_record(caplog):
    store = SimClosestApproachStore(boot_id="b1")
    assert not store.offer(2, 5, "NAV: hello")
    assert not store.offer(2, 5, encode(_record(vehicle=3)))
    assert store.snapshot() == []
    assert "dropped" in caplog.text


def test_store_is_bounded_per_vehicle():
    store = SimClosestApproachStore(boot_id="b1")
    for episode in range(SIM_CLOSEST_APPROACH_KEEP + 5):
        store.offer(2, 1, encode(_record(episode=episode)))
    store.offer(3, 2, encode(_record(vehicle=3)))
    entries = store.snapshot()
    own = [entry for entry in entries if entry["sys_id"] == 2]
    assert len(own) == SIM_CLOSEST_APPROACH_KEEP
    assert own[0]["episode"] == 5  # oldest dropped
    assert [entry["sys_id"] for entry in entries].count(3) == 1


def test_keys_differ_across_generations_and_backend_boots():
    """Episode numbers restart with NavPy and PIDs can be reused."""
    first, second = SimClosestApproachStore("boot-a"), SimClosestApproachStore("boot-b")
    first.offer(2, 1, encode(_record()))
    first.offer(2, 2, encode(_record()))  # restarted NavPy, same episode 1
    second.offer(2, 1, encode(_record()))  # backend restart, generation 1 again
    keys = [e["key"] for e in first.snapshot() + second.snapshot()]
    assert len(set(keys)) == 3


def test_default_boot_ids_differ():
    assert SimClosestApproachStore()._boot_id != SimClosestApproachStore()._boot_id


def _process(pid, lines):
    proc = MagicMock()
    proc.pid = pid
    proc.poll.return_value = None
    proc.stdout.readline.side_effect = [line.encode() for line in lines] + [b""]
    return proc


def test_a_record_survives_its_process_stop_and_replacement():
    manager = NavpyProcessManager()
    first = NavpyInstance(2, "udp:0.0.0.0:5760", _process(1001, [encode(_record()) + "\n"]))
    manager._instances[2] = first
    manager._read_output(first)
    manager._instances.pop(2)  # stopped before the next broadcast

    second = NavpyInstance(2, "udp:0.0.0.0:5760", _process(1001, [encode(_record(d3_m=0.4)) + "\n"]))
    manager._instances[2] = second  # same PID reused, episode 1 again
    manager._read_output(second)

    records = manager.closest_approach_records()
    assert [r["d3_m"] for r in records] == [1.84, 0.4]
    assert records[0]["key"] != records[1]["key"]
    assert second.generation > first.generation


def test_a_malformed_record_never_stops_the_reader():
    """Later records and the stop ack still arrive."""
    manager = NavpyProcessManager()
    bad = encode(_record()).replace('"outcome":"missed"', '"outcome":["missed"]')
    lines = [bad + "\n", encode(_record(episode=2)) + "\n", "NAVPY_STOPPED_V1\n"]
    inst = NavpyInstance(2, "udp:0.0.0.0:5760", _process(1001, lines))
    manager._instances[2] = inst
    manager._read_output(inst)
    assert [r["episode"] for r in manager.closest_approach_records()] == [2]
    assert inst.stop_acked


def test_a_throwing_line_consumer_never_stops_the_reader():
    from gcs.backend.navpy_process_instance import read_navpy_output

    inst = NavpyInstance(2, "udp:0.0.0.0:5760", _process(1001, ["x\n", "NAVPY_STOPPED_V1\n"]))
    read_navpy_output(inst, on_line=MagicMock(side_effect=RuntimeError("boom")))
    assert inst.stop_acked


def test_reader_still_sees_readiness_and_stop_lines():
    manager = NavpyProcessManager()
    inst = NavpyInstance(2, "udp:0.0.0.0:5760", _process(1001, ["NAVPY_STOPPED_V1\n"]))
    manager._read_output(inst)
    assert inst.stop_acked


def test_status_broadcast_carries_the_records_beside_the_instances():
    manager = MagicMock()
    manager.get_all_status.return_value = []
    manager.closest_approach_records.return_value = [{"key": "b:1:1", "sys_id": 2}]
    saved = runtime_mod._navpy_mgr
    runtime_mod._navpy_mgr = manager
    try:
        with patch.object(navpy_mod, "emit"), \
             patch.object(navpy_mod.ws_manager, "broadcast", new_callable=AsyncMock) as ws:
            asyncio.run(navpy_mod.broadcast_sim_status())
    finally:
        runtime_mod._navpy_mgr = saved
    payload = ws.await_args.args[0]
    assert payload["instances"] == []
    assert payload["closest_approaches"] == [{"key": "b:1:1", "sys_id": 2}]


def test_lockstep_launcher_has_no_records():
    from gcs.backend.lockstep.launcher_port import LockstepLauncher

    assert LockstepLauncher.closest_approach_records(object.__new__(LockstepLauncher)) == []
