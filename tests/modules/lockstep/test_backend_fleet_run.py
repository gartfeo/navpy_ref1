"""10e on reference autopilots: the backend with 3 vehicles on ``lockstep:router``.

Three serial-step SITL doubles (one reference autopilot each, sysids 1-3)
run under the coordinator with the router in the tick path from tick 1, and
the backend participant alone on the router device. The scripted fleet
actor (``scripts.lockstep_fleet_actor.fleet_script``) connects the three
vehicles on the one router device, reads each one's command bounds by name,
writes each one's AAS profile, uploads the fixture's three missions with
their readback, then starts the in-process ESP32 simulator. The journal, the
SITL streams, the actor's result and the backend's logs repeat byte for
byte under two slots, the second with host delays.
"""

from __future__ import annotations

import json

import pytest

from gcs.backend import instance_registry_lockstep as lockstep
from gcs.backend.aas_params import AAS_PARAM_MAP
from scripts.lockstep_fleet_actor import FleetLayout
from tests.modules.lockstep.backend_doubles import build_fleet

FLEET = FleetLayout(3)


MISSION_ITEM_INT = 73


def _packet_events(records: list[dict]) -> list[tuple[int | None, list]]:
    """Every packet event with the tick of the record that holds it (None: parked)."""
    events = [(r["k"], e) for r in records if "k" in r for e in r.get("packets", [])]
    events += [(None, e) for r in records if "parked" in r for e in r["parked"].get("packets", [])]
    return events


def _routing(records: list[dict]) -> tuple[dict, dict, dict, dict]:
    """From the packet records: emissions ``id -> (tick, msgid, target sys, source comp)``, each
    source's router copies ``id -> [destination device]``, each copy's terminal
    disposition, and the ticks whose reply admitted bytes of each copy."""
    emitted, copies, ends, admitted = {}, {}, {}, {}
    for k, event in _packet_events(records):
        kind, packet_id = event[0], tuple(event[1])
        if kind == "emitted":
            emitted[packet_id] = (k, event[4], event[6][0], event[5][1])
        elif kind == "copy":
            copies.setdefault(tuple(event[2]), []).append(packet_id[2])
            ends.setdefault(packet_id, None)
        elif kind == "admitted":
            admitted.setdefault(packet_id, []).append(event[3] if len(event) > 3 else k)
        elif packet_id in ends:
            ends[packet_id] = kind
    return emitted, copies, ends, admitted


def _check_routing(records: list[dict]) -> None:
    """Targeted packets reach only their target's serial1; SITL traffic is routed from tick 1."""
    emitted, copies, ends, admitted = _routing(records)
    backend = [(packet_id, msgid, target) for packet_id, (_k, msgid, target, _c) in emitted.items()
               if packet_id[0] == FLEET.backend]
    items = {sys_id: 0 for sys_id in FLEET.sys_ids}
    for packet_id, msgid, target in backend:
        if target in FLEET.sys_ids:  # exclusive: one copy, to the target's own serial1
            assert copies.get(packet_id) == [f"sitl{target}.serial1"], (packet_id, msgid, copies.get(packet_id))
            if msgid == MISSION_ITEM_INT:
                items[target] += 1
    assert all(count > 6 for count in items.values()), items  # every vehicle got its own mission
    # Tick 1: each SITL's serial1 broadcasts are copied to the backend's endpoint;
    # its peers get its gimbal's but not its autopilot's (each SITL endpoint
    # filters components 1 and 100). Reply 1 admits every copy to a serial1.
    for index in FLEET.sys_ids:
        first = {packet_id: comp for packet_id, (k, _m, target, comp) in emitted.items()
                 if k == 1 and packet_id[:3] == (f"sitl{index}", 1, f"sitl{index}.serial1") and target == -1}
        assert {1, 154} <= set(first.values()), f"SITL {index}: tick-1 broadcasts from {first}"
        peers = [f"sitl{j}.serial1" for j in FLEET.sys_ids if j != index]
        for packet_id, comp in first.items():  # every tick-1 broadcast, not only the first
            expected = [FLEET.router] if comp in (1, 100) else [*peers, FLEET.router]
            assert copies.get(packet_id) == expected, (packet_id, comp, copies.get(packet_id))
    tick1_copies = [packet_id for (k, event) in _packet_events(records)
                    if k == 1 and event[0] == "copy" for packet_id in [tuple(event[1])]
                    if packet_id[2].endswith(".serial1")]
    assert tick1_copies and all(1 in admitted.get(packet_id, []) for packet_id in tick1_copies)
    assert all(ends[packet_id] == "delivered" for packet_id in tick1_copies)


def _claim() -> tuple[int, dict]:
    return lockstep.claim(run_token="seed-1", clone="clone", owner="dir:test", port_check=lambda n: True)


def _run(tmp_path, monkeypatch, name: str, slot: int, delay_seed: int = 0):
    monkeypatch.setenv("GCS_CHAT_INDEX", str(slot))  # host side only
    run = build_fleet(tmp_path / name, FLEET, delay_seed=delay_seed)
    return run, run.run()


def _artifacts(run_dir) -> dict[str, bytes]:
    gcs = run_dir / "host" / "gcs"
    return {path.relative_to(gcs).as_posix(): path.read_bytes()
            for path in sorted(gcs.rglob("*")) if path.is_file()}


def test_the_fleet_actor_for_three_sysids_repeats_byte_for_byte_under_two_slots(tmp_path, monkeypatch):
    monkeypatch.setenv("GCS_INSTANCE_REGISTRY", str(tmp_path / "instances.json"))
    (first, first_owner), (second, second_owner) = _claim(), _claim()  # both held: two slots
    assert first != second
    try:
        plain, plain_result = _run(tmp_path, monkeypatch, "first", first)
        other, other_result = _run(tmp_path, monkeypatch, "second", second, delay_seed=5)
    finally:
        assert lockstep.release_if_owner(first, first_owner)
        assert lockstep.release_if_owner(second, second_owner)
    for result in (plain_result, other_result):
        assert result.outcome == "operator_returned"  # no NavPy, so no SNAP
        assert result.parked_tick == result.final_tick + 1
    assert plain_result == other_result
    assert plain.coordinator.journal.records == other.coordinator.journal.records
    assert plain.coordinator.journal.digest == other.coordinator.journal.digest
    for mine, theirs in zip(plain.sitls, other.sitls):
        assert mine.serial1 == theirs.serial1 and mine.snapshot_rows == theirs.snapshot_rows
    assert plain.operator.result == other.operator.result
    assert plain.operator.journal == other.operator.journal
    backend = FLEET.backend
    log = plain.operator.journal.log_lines[backend]
    for sys_id in FLEET.sys_ids:
        assert any(line.endswith(f"Vehicle {sys_id} (s1-u{sys_id}) connected") for line in log)
    assert _artifacts(tmp_path / "first") == _artifacts(tmp_path / "second")

    result = plain.operator.result
    assert set(result["aas"]) == set(result["bounds"]) == {"1", "2", "3"}
    assert result["esp32"]["reachable"] is True and result["esp32"]["status"]["channels"][:3] == [1, 2, 3]
    records = plain.coordinator.journal.records
    assert records[0]["grid"]["sitl_ports"] == [list(ports) for ports in FLEET.sitl_ports]
    boot = [r for r in records if r.get("phase") == "boot"]
    assert boot and boot[0]["k"] == 1  # the router runs from tick 1
    _check_routing(records)
    for sitl in plain.sitls:
        autopilot = sitl.autopilot
        sys_id = autopilot.sys_id
        # Each vehicle got its own profile and mission through the router, nobody else's.
        written = {AAS_PARAM_MAP[f]: float(v) for f, v in result["aas"][str(sys_id)].items()}
        assert {name: autopilot.parameters[name] for name in written} == pytest.approx(written)
        assert len(autopilot.mission) > 6
        from_backend = [(kind, port) for port, kind, source, _c in autopilot.received if source == 255]
        assert {"PARAM_SET", "MISSION_COUNT", "MISSION_ITEM_INT", "MISSION_REQUEST_LIST"} <= {
            kind for kind, _port in from_backend}
        assert all(port == 1 for _kind, port in from_backend)  # serial1 only: no NavPy on serial0
        # Peer vehicles' traffic reaches it through the router (heartbeats are broadcast).
        peers = {source for port, kind, source, _c in autopilot.received
                 if port == 1 and kind == "HEARTBEAT" and source != 255}
        assert peers == set(FLEET.sys_ids) - {sys_id}
    host = json.loads((tmp_path / "first" / "lockstep_participants_host.json").read_text())
    assert [row["exit_code"] for row in host] == [0]


def test_three_navpys_spawn_boot_and_stop_through_the_launcher_port_and_repeat(tmp_path, monkeypatch):
    """10f on reference autopilots: start-all spawns NavPy 1-3 (roles
    ``navpy<i>``, each on its own serial0) one after another, each boots,
    and stop-all acks every one; two slots, the second with host delays,
    give the same journal. Idle NavPys send no peer NAVLINK (the swarm actor
    starts in flight), so the NAVLINK hops are the real-SITL run's
    (``eval_lockstep_fleet_backend.py --navpy``)."""
    monkeypatch.setenv("GCS_INSTANCE_REGISTRY", str(tmp_path / "instances.json"))
    (first, first_owner), (second, second_owner) = _claim(), _claim()
    try:
        runs = []
        for name, slot, delay_seed in (("first", first, 0), ("second", second, 5)):
            monkeypatch.setenv("GCS_CHAT_INDEX", str(slot))
            run = build_fleet(tmp_path / name, FLEET, delay_seed=delay_seed, navpy=True)
            runs.append((run, run.run()))
    finally:
        assert lockstep.release_if_owner(first, first_owner)
        assert lockstep.release_if_owner(second, second_owner)
    (plain, plain_result), (other, other_result) = runs
    assert plain_result.outcome == "operator_returned" and plain_result == other_result
    assert plain.coordinator.journal.records == other.coordinator.journal.records
    assert plain.operator.result == other.operator.result

    ready = plain.operator.result["ready"]
    assert [(s["sys_id"], s["connection"], s["ordinal"], s["runtime"]["ordinal"]) for s in ready] == [
        (i, FLEET.serial0(i), i, i) for i in FLEET.sys_ids]
    assert plain.operator.result["stopped"]["instances"] == []
    lifecycle = [entry for record in plain.coordinator.journal.records for entry in record.get("lifecycle", ())]
    assert [entry["result"] for entry in lifecycle if entry["kind"] == "stop_result"] == ["acked"] * 3
    spawns = [(record["k"], entry["ordinal"]) for record in plain.coordinator.journal.records
              for entry in record.get("lifecycle", ()) if entry["kind"] == "spawn"]
    assert [ordinal for _k, ordinal in spawns] == list(FLEET.sys_ids)
    assert [k for k, _ordinal in spawns] == sorted({k for k, _ordinal in spawns})  # one per tick, in order
    for sys_id in FLEET.sys_ids:
        log = plain.operator.journal.log_lines[FLEET.navpy(sys_id)]
        assert any(f"connection='{FLEET.serial0(sys_id)}', network_type='mav', source_system={sys_id}" in line
                   for line in log)
    host = json.loads((tmp_path / "first" / "lockstep_participants_host.json").read_text())
    assert sorted((row["role"], row["exit_code"]) for row in host) == [
        ("backend", 0), ("navpy1", 0), ("navpy2", 0), ("navpy3", 0)]
