"""The coordinator on the router path: the 3-link toy with real processes (step 10b).

``lockstep_fleet.md``, Router and delivery: SITL serial1 traffic is routed
from tick 1 (boot included, owner Q2); bytes for an owner not yet admitted
are discarded and recorded; the hub's and the rovers' MAVLink reach their
SITL ports through the queues; every packet ends in exactly one terminal
disposition, and the journal repeats under host delays.
"""

from __future__ import annotations

import pytest

from navpy.sim.lockstep.coordinator_mavlink import ROUTER_LINK
from navpy.sim.lockstep.packet_journal import PacketJournal
from tests.modules.lockstep.coordinator_doubles import HostDelays
from tests.modules.lockstep.fleet_doubles import (
    HUB,
    ROUTER,
    ROVERS,
    build_fleet,
    routed_fleet_manifest,
    routed_fleet_sitls,
    serial0,
    serial1,
)


def _run(tmp_path, name, *, delay_seed=0, plan=None):
    sitls = routed_fleet_sitls(delays=HostDelays(delay_seed))
    coordinator, sitls, operator, roster = build_fleet(
        tmp_path / name, plan=plan or routed_fleet_manifest(), sitls=sitls, delay_seed=delay_seed)
    result = coordinator.run()
    return coordinator, result


@pytest.fixture(scope="module")
def routed(tmp_path_factory):
    return _run(tmp_path_factory.mktemp("routed"), "plain")


def _events(records, kind):
    out = []
    for record in records:
        if "packets" in record:
            out.extend((record["k"], record["phase"], event)
                       for event in record["packets"] if event[0] == kind)
    return out


def test_the_routed_toy_repeats_under_host_delays_and_every_packet_ends_once(routed, tmp_path):
    coordinator, result = routed
    assert result.outcome == "ok"
    delayed = _run(tmp_path, "delayed", delay_seed=7, plan=routed_fleet_manifest(
        hub={"exit_delay_s": 0.2}, rovers={i: {"exit_delay_s": 0.2} for i in ROVERS}))[0]
    assert coordinator.journal.digest == delayed.journal.digest
    assert PacketJournal(coordinator.journal.records).problems() == []
    counts = coordinator.journal.records[-2]["mavlink_counts"]
    assert set(counts) == {serial0(i) for i in ROVERS} | {serial1(i) for i in ROVERS} | {ROUTER_LINK}


def test_boot_routes_sitl_serial1_and_discards_what_an_unadmitted_owner_would_get(routed):
    coordinator, _result = routed
    records = coordinator.journal.records
    boot = [r for r in records if r.get("phase") == "boot"]
    assert boot and all(r["members"] == [] for r in boot)
    copies = [event for k, phase, event in _events(records, "copy") if phase == "boot"]
    assert {event[1][2] for event in copies} >= {serial1(i) for i in ROVERS}
    boot_discards = [event for _k, phase, event in _events(records, "discarded") if phase == "boot"]
    assert boot_discards and {event[2] for event in boot_discards} == {"unadmitted"}
    assert all(r["discarded"].get(ROUTER) for r in boot)  # the backend endpoint's copies


def test_the_hub_and_the_rovers_reach_their_sitl_ports_through_the_queues(routed):
    coordinator, _result = routed
    records = coordinator.journal.records
    emitted = {tuple(e[1]): e for _k, _p, e in _events(records, "emitted")}
    copies = {tuple(e[1]): tuple(e[2]) for _k, _p, e in _events(records, "copy")}
    delivered = [(k, tuple(e[1]), e[2]) for k, _p, e in _events(records, "delivered")]
    # The hub's PARAM_REQUEST_READ to 2/1 reaches SITL 2's serial1, and only it.
    hub_params = {receiver for _k, packet_id, receiver in delivered
                  if packet_id in copies and copies[packet_id][0] == HUB
                  and emitted[copies[packet_id]][4] == 20}
    assert hub_params == {serial1(2)}
    # Rover i's TASK_ASSIGN_REQUEST reaches SITL i's serial0 (point to point).
    for rover in ROVERS:
        receivers = {receiver for _k, packet_id, receiver in delivered
                     if packet_id[0] == rover and emitted[packet_id][4] == 25106}
        assert receivers == {serial0(rover)}
    # Every tick journals the pending bytes per port; parking journals what is left.
    mission = [r for r in records if r.get("phase") in ("boot", "mission", "finalization")]
    assert all(set(r["pending"]) == {serial0(i) for i in ROVERS} | {serial1(i) for i in ROVERS}
               for r in mission)
    (parked,) = [r["parked"] for r in records if "parked" in r]
    assert "undelivered" in parked and isinstance(parked["packets"], list)


def test_a_rovers_navlink_packet_crosses_the_autopilots_and_the_router_to_its_peer(routed):
    """NavPy 1 -> serial0 -> AP 1 -> serial1 -> router -> AP 2 -> serial0 -> NavPy 2,
    each AP hop a new emission correlated by the NavPy UID."""
    coordinator, _result = routed
    records = coordinator.journal.records
    emitted = {tuple(e[1]): (k, e) for k, _p, e in _events(records, "emitted")}
    uid = next(e[7]["uid"] for _k, e in emitted.values() if e[1][0] == 1 and e[4] == 25106)
    hops = {(e[1][0], e[1][2]) for _k, e in emitted.values() if len(e) > 7 and e[7].get("uid") == uid}
    assert {(1, serial0(1)), ("sitl1", serial1(1)), ("sitl2", serial0(2))} <= hops
    delivered = [(k, tuple(e[1]), e[2]) for k, _p, e in _events(records, "delivered")]
    peer = [k for k, packet_id, receiver in delivered if receiver == [2, 1]
            and len(emitted[packet_id][1]) > 7 and emitted[packet_id][1][7].get("uid") == uid]
    source = min(k for k, e in emitted.values() if e[1][0] == 1 and len(e) > 7 and e[7].get("uid") == uid)
    assert peer and min(peer) > source  # the request reached the helper's companion, hops later
    # This scripted autopilot forwards bytes unchanged, so its hop keeps the sha256
    # (the real AP hop is measured in 10f, not assumed).
    shas = {(e[1][0], e[1][2]): e[3] for _k, e in emitted.values()
            if len(e) > 7 and e[7].get("uid") == uid}
    assert shas[(1, serial0(1))] == shas[("sitl1", serial1(1))]
