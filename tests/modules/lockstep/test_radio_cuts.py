"""A scheduled fault's radio cuts on the router path (``radio_cuts``, ``RoutedPorts._route``, step 7)."""

from __future__ import annotations

import pytest

from navpy.sim.lockstep.coordinator_mavlink import RADIO_FAULT, ROUTER_LINK
from navpy.sim.lockstep.coordinator_queues import FIFO_CAP
from navpy.sim.lockstep.radio_cuts import RadioCuts
from tests.modules.lockstep.router_doubles import BACKEND, ROUTER, PortsHarness, Sources, serial1

FAULT = "link_drop:sitl1"


@pytest.fixture
def run():
    return PortsHarness()


def _learned(run, src: Sources, backend) -> None:
    """Tick 1: every SITL's AP heartbeat on serial1, and the backend's."""
    run.tick(1, egress={serial1(i): src.heartbeat(i, 1) for i in (1, 2, 3)}, admitted=[backend],
             done=[(backend, ROUTER, src.heartbeat(255, 190))])


def test_the_cuts_name_their_fault_until_it_heals():
    cuts = RadioCuts()
    assert cuts.source("sitl1") is None and cuts.path("sitl1", "sitl2") is None
    cuts.cut("a", ["sitl1"], [("sitl2", "sitl1")])
    cuts.cut("b", [], [("sitl3", "sitl1")])
    assert cuts.source("sitl1") == "a" and cuts.source("sitl2") is None
    assert cuts.path("sitl2", "sitl1") == "a" and cuts.path("sitl3", "sitl1") == "b"
    assert cuts.path("sitl1", "sitl2") is None  # a path has one direction
    cuts.heal("a")
    assert cuts.source("sitl1") is None and cuts.path("sitl2", "sitl1") is None
    assert cuts.path("sitl3", "sitl1") == "b"


def test_a_fault_free_run_counts_no_radio_fault(run):
    src = Sources()
    _learned(run, src, run.roster.add(BACKEND, ROUTER))
    counts = run.ports.closing()["mavlink_counts"]
    assert all(RADIO_FAULT not in directions for directions in counts.values())
    assert run.events(RADIO_FAULT) == []


def test_a_packet_from_a_cut_source_ends_before_the_router_sees_it(run):
    src = Sources()
    backend = run.roster.add(BACKEND, ROUTER)
    run.ports.radio.cuts.cut(FAULT, ["sitl1"], [])
    beat = src.heartbeat(2, 1)
    delivered, _ = run.tick(1, egress={serial1(1): src.heartbeat(1, 1), serial1(2): beat},
                            admitted=[backend], done=[(backend, ROUTER, src.heartbeat(255, 190))])
    assert delivered == {ROUTER: beat}  # only SITL 2's heartbeat got through
    ((_k, event),) = run.events(RADIO_FAULT)
    assert event[1][0] == "sitl1" and event[2] == FAULT
    assert run.journal().problems() == []
    counts = run.ports.closing()["mavlink_counts"]
    assert counts[serial1(1)][RADIO_FAULT]["messages"] == {"0:1.1>-": 1}  # its source link


def test_a_cut_source_teaches_the_router_nothing(run):
    src = Sources()
    backend = run.roster.add(BACKEND, ROUTER)
    run.ports.radio.cuts.cut(FAULT, ["sitl1"], [])
    run.tick(1, egress={serial1(1): src.heartbeat(1, 1)}, admitted=[backend],
             done=[(backend, ROUTER, src.heartbeat(255, 190))])
    run.ports.radio.cuts.heal(FAULT)
    _, ingress = run.tick(2, admitted=[backend], done=[(backend, ROUTER, src.param((255, 190), 1, 1))])
    assert ingress[serial1(1)] == b""  # the router never saw 1/1: nowhere to route it
    assert [e[1][2] for _k, e in run.events("router_dropped")] == [ROUTER]
    run.tick(3, egress={serial1(1): src.heartbeat(1, 1)}, admitted=[backend])
    param = src.param((255, 190), 1, 1)
    _, ingress = run.tick(4, admitted=[backend], done=[(backend, ROUTER, param)])
    assert ingress[serial1(1)] == param  # heard again, routed again


def test_a_cut_path_ends_only_its_copy(run):
    src = Sources()
    backend = run.roster.add(BACKEND, ROUTER)
    _learned(run, src, backend)
    run.ports.radio.cuts.cut(FAULT, [], [("backend", "sitl1")])
    beat = src.heartbeat(255, 190)
    _, ingress = run.tick(2, admitted=[backend], done=[(backend, ROUTER, beat)])
    assert ingress[serial1(1)] == b"" and ingress[serial1(2)] == beat and ingress[serial1(3)] == beat
    ((_k, event),) = run.events(RADIO_FAULT)
    assert event[1][0] == "router" and event[1][2] == serial1(1) and event[2] == FAULT
    assert run.journal().problems() == []
    # The source ended routed with all its copies, one of them cut.
    (routed,) = [e for k, e in run.events("routed") if k == 2]
    assert routed[2] == 3
    counts = run.ports.closing()["mavlink_counts"]
    assert counts[serial1(1)][RADIO_FAULT]["messages"] == {"0:255.190>-": 1}  # the link it died on
    assert counts[serial1(2)]["routed_out"]["messages"]["0:255.190>-"] == 2


def test_a_cut_path_to_the_backend_tallies_on_the_router_link(run):
    src = Sources()
    backend = run.roster.add(BACKEND, ROUTER)
    _learned(run, src, backend)
    run.ports.radio.cuts.cut(FAULT, [], [("sitl1", "backend")])
    delivered, _ = run.tick(2, egress={serial1(1): src.heartbeat(1, 1)}, admitted=[backend])
    assert delivered == {}
    assert run.journal().problems() == []
    assert run.ports.closing()["mavlink_counts"][ROUTER_LINK][RADIO_FAULT]["messages"] == {"0:1.1>-": 1}


def test_a_healed_path_carries_again(run):
    src = Sources()
    backend = run.roster.add(BACKEND, ROUTER)
    _learned(run, src, backend)
    run.ports.radio.cuts.cut(FAULT, ["sitl1"], [("backend", "sitl1")])
    run.tick(2, admitted=[backend], done=[(backend, ROUTER, src.heartbeat(255, 190))])
    run.ports.radio.cuts.heal(FAULT)
    beat = src.heartbeat(255, 190)
    _, ingress = run.tick(3, admitted=[backend], done=[(backend, ROUTER, beat)])
    assert ingress[serial1(1)] == beat
    up = src.heartbeat(1, 1)
    delivered, _ = run.tick(4, egress={serial1(1): up}, admitted=[backend])
    assert delivered == {ROUTER: up}


def test_bytes_queued_before_the_cut_still_arrive(run):
    src = Sources()
    backend = run.roster.add(BACKEND, ROUTER)
    _learned(run, src, backend)
    beat = src.heartbeat(255, 190)
    narrow = {serial1(1): FIFO_CAP - 5}
    _, ingress = run.tick(2, admitted=[backend], done=[(backend, ROUTER, beat)], fifo=narrow)
    assert ingress[serial1(1)] == beat[:5]
    run.ports.radio.cuts.cut(FAULT, [], [("backend", "sitl1")])
    _, ingress = run.tick(3, admitted=[backend])
    assert ingress[serial1(1)] == beat[5:]  # a radio's own buffer: already on its way
    assert run.events(RADIO_FAULT) == [] and run.journal().problems() == []
