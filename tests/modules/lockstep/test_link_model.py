"""Step 3 of deterministic_dev_sim.md: the serial and radio link models at the port seams.

Each test names the rule it pins; each was checked by breaking that rule in the
product code (the mutation in the docstring) and seeing the test fail. Ports are
driven without processes (``router_doubles``): a tick is 20 ms, so a line rate
of 11 500 B/s carries 230 bytes a tick and 8 000 B/s 160; a line's first tick
carries nothing.
"""

from __future__ import annotations

import copy

import pytest

from navpy.sim.lockstep.link_model import (
    RADIO_LINK,
    SERIAL_LINK,
    ByteRate,
    LinkClock,
    LinkModels,
    LinkSpec,
    link_models,
)
from navpy.sim.lockstep.sim_profile import ModelDraw, ParamRange, Realization, load_profile, model_stream, \
    realize
from tests.modules.lockstep.router_doubles import BACKEND, ROUTER, PortsHarness, Sources, serial0, serial1


def _serial(spec: LinkSpec, seed: int = 1) -> PortsHarness:
    return PortsHarness(LinkModels(seed, serial=spec))


def _radio(spec: LinkSpec, seed: int = 1) -> PortsHarness:
    return PortsHarness(LinkModels(seed, radio=spec))


def _events(run: PortsHarness, kind: str) -> list[tuple[int, list]]:
    return run.events(kind)


# --- the profile draw ------------------------------------------------------------------------


def test_the_draw_becomes_link_specs_and_both_off_is_no_link_model():
    serial = ModelDraw(SERIAL_LINK, True, {"delay_ticks": 1, "jitter_ticks": 1, "bytes_per_s": 11500})
    radio = ModelDraw(RADIO_LINK, True, {"delay_ticks": 4, "jitter_ticks": 2, "loss": 0.05,
                                         "bytes_per_s": 8000})
    draw = Realization("t", "0" * 64, 9, (radio, serial))
    assert link_models(draw) == LinkModels(9, LinkSpec(1, 1, bytes_per_s=11500),
                                           LinkSpec(4, 2, 0.05, 8000))
    off = Realization("t", "0" * 64, 9, (ModelDraw(RADIO_LINK, False), serial))
    assert link_models(off) == LinkModels(9, LinkSpec(1, 1, bytes_per_s=11500), None)
    both_off = Realization("t", "0" * 64, 9, (ModelDraw(RADIO_LINK, False), ModelDraw(SERIAL_LINK, False)))
    assert link_models(both_off) is None
    assert link_models(realize(load_profile("none"), 1)) is None


def test_the_hw_profile_carries_the_designs_link_ranges_and_quiet_has_no_loss():
    hw = load_profile("hw").models
    assert hw[SERIAL_LINK].params == {"delay_ticks": ParamRange(0, 2), "jitter_ticks": ParamRange(0, 1),
                                      "bytes_per_s": 11500}
    assert hw[RADIO_LINK].params == {"delay_ticks": ParamRange(2, 8), "jitter_ticks": ParamRange(0, 5),
                                     "loss": ParamRange(0.0, 0.1), "bytes_per_s": 8000}
    assert hw[SERIAL_LINK].p_on == hw[RADIO_LINK].p_on == 0.5
    assert load_profile("quiet").models[RADIO_LINK].params["loss"] == 0.0
    draws = [link_models(realize(load_profile("hw"), seed)) for seed in range(1, 201)]
    radios = [d.radio for d in draws if d is not None and d.radio is not None]
    assert 70 < len(radios) < 130  # on in about half the seeds
    assert all(2 <= r.delay_ticks <= 8 and 0 <= r.jitter_ticks <= 5 and 0 <= r.loss <= 0.1 for r in radios)


# --- the pure models -------------------------------------------------------------------------


def test_a_link_clock_adds_delay_and_jitter_in_range_and_repeats_per_seed():
    """Mutations: no jitter draw (offsets {2}); jitter span J instead of J + 1 (5 never drawn)."""
    def offsets(seed: int) -> list[int]:
        clock = LinkClock(LinkSpec(2, 3), model_stream(seed, RADIO_LINK, "a-b.jitter"))
        return [clock.arrival(k) - k for k in range(0, 4000, 10)]  # spaced: nothing to overtake

    assert set(offsets(1)) == {2, 3, 4, 5}
    assert offsets(1) == offsets(1) and offsets(1) != offsets(2)


def test_a_link_clock_never_lets_a_later_send_arrive_first():
    """Mutation: drop the ``max`` with the last arrival (a later send overtakes)."""
    clock = LinkClock(LinkSpec(0, 5), model_stream(1, RADIO_LINK, "a-b.jitter"))
    arrivals = [clock.arrival(k) for k in range(500)]
    assert arrivals == sorted(arrivals) and any(a > k for k, a in enumerate(arrivals))


def test_a_byte_rate_grants_its_rate_per_tick_and_carries_only_the_fraction():
    """Mutation: drop the carry (7 B/s over 0.1 s ticks would grant nothing)."""
    rate = ByteRate(11500)
    assert rate.grant(1_000_000) == 0  # the line's first tick
    assert [rate.grant(1_000_000 + 20_000 * k) for k in range(1, 4)] == [230, 230, 230]
    slow = ByteRate(7)
    grants = [slow.grant(100_000 * k) for k in range(0, 21)]
    assert grants[0] == 0 and sum(grants) == 14 and set(grants[1:]) == {0, 1}
    uneven = ByteRate(8000)
    times = [0, 20_825, 39_000, 61_111, 80_000]
    assert sum(uneven.grant(t) for t in times) == 8000 * 80_000 // 1_000_000


# --- the serial link, autopilot to NavPy ------------------------------------------------------


def test_the_autopilots_bytes_reach_navpy_delay_ticks_later():
    """Mutation: ``to_navpy`` returns the frame's bytes (delivered in tick 1)."""
    src, run = Sources(), _serial(LinkSpec(2, 0))
    navpy = run.roster.add(1, serial0(1))
    status = src.heartbeat(1, 1)
    got = [run.tick(k, egress={serial0(1): status} if k == 1 else {}, admitted=[navpy])[0]
           for k in range(1, 5)]
    assert [g.get(serial0(1), b"") for g in got] == [b"", b"", status, b""]
    assert [r["link_held"] for r in run.records] == [{"to_navpy": {serial0(1): len(status)}}] * 2 + [{}, {}]
    (delivered,) = _events(run, "delivered")
    assert delivered[0] == 3 and delivered[1][2] == [1, 1]
    assert run.journal().problems() == []


def test_the_autopilots_bytes_keep_their_order_through_jitter_and_end_link_at_parking():
    """Mutations: ``ByteDelay.release`` takes every due push, not only from the head
    (later bytes overtake); park drops held bytes (an id never ends)."""
    src, run = Sources(), _serial(LinkSpec(2, 3))
    navpy = run.roster.add(1, serial0(1))
    sent = [src.heartbeat(1, 1) for _ in range(40)]
    got = [run.tick(k, egress={serial0(1): sent[k - 1]}, admitted=[navpy])[0].get(serial0(1), b"")
           for k in range(1, 41)]
    received = b"".join(got)
    held = run.records[-1]["link_held"]["to_navpy"][serial0(1)]
    assert b"".join(sent).startswith(received)  # in order, nothing lost or repeated
    assert held == len(b"".join(sent)) - len(received) > 0
    assert {len(g) // len(sent[0]) for g in got} > {0, 1}  # jitter bunched some ticks
    parked = run.park(41)
    assert parked["link_held"][serial0(1)]["bytes"] == held
    reasons = [e[2] for _k, e in _events(run, "discarded")]
    assert reasons and set(reasons) == {"link"}
    assert run.journal().problems() == []
    run.ports.closing()  # every packet ended


# --- the serial link, NavPy to the autopilot --------------------------------------------------


def test_navpys_bytes_are_admitted_delay_ticks_later():
    """Mutation: ``ready`` returns None (admitted by reply 1)."""
    src, run = Sources(), _serial(LinkSpec(1, 0))
    navpy = run.roster.add(2, serial0(2))
    packet = src.heartbeat(2, 191)
    ingress = [run.tick(k, admitted=[navpy], done=[(navpy, serial0(2), packet)] if k == 1 else [])[1]
               for k in range(1, 4)]
    assert [i[serial0(2)] for i in ingress] == [b"", packet, b""]
    assert [r["pending"][serial0(2)] for r in run.records] == [len(packet), 0, 0]
    assert run.journal().problems() == []


def test_navpys_bytes_share_the_line_rate_in_order_and_unused_bytes_are_gone():
    """Mutations: no rate on the port (the burst goes in one reply); carry unused bytes
    over (the late packet goes in one reply after the idle ticks)."""
    src, run = Sources(), _serial(LinkSpec(0, 0, bytes_per_s=11500))
    navpy = run.roster.add(2, serial0(2))
    burst = b"".join(src.blob((2, 191), n) for n in range(8))
    ingress = [run.tick(k, admitted=[navpy], done=[(navpy, serial0(2), burst)] if k == 1 else [])[1][serial0(2)]
               for k in range(1, 30)]
    assert ingress[0] == b"" and all(len(i) <= 230 for i in ingress)
    assert b"".join(ingress) == burst  # all of it, in order
    late = src.blob((2, 191), 99)
    out = [run.tick(k, admitted=[navpy], done=[(navpy, serial0(2), late)] if k == 40 else [])[1][serial0(2)]
           for k in range(30, 45)]
    assert [len(o) for o in out if o] == [230, len(late) - 230]
    assert run.journal().problems() == []


# --- the radio link ---------------------------------------------------------------------------


def test_a_router_copy_arrives_delay_ticks_after_it_left_with_an_arrived_event():
    """Mutation: deliver at routing (the backend gets it in tick 1)."""
    src, run = Sources(), _radio(LinkSpec(3, 0))
    backend = run.roster.add(BACKEND, ROUTER)
    hb = src.heartbeat(1, 1)
    got = [run.tick(k, egress={serial1(1): hb} if k == 1 else {}, admitted=[backend])
           for k in range(1, 6)]
    assert [g[0].get(ROUTER, b"") for g in got] == [b"", b"", b"", hb, b""]
    assert [g[1][serial1(2)] for g in got] == [b"", b"", b"", hb, b""]
    arrived = _events(run, "arrived")
    assert [(k, e[2]) for k, e in arrived] == [(4, 1)] * 3  # sitl2, sitl3, backend
    assert run.records[0]["link_held"] == {"radio": {"air": 3}}
    assert run.journal().problems() == []


def test_the_radio_loses_copies_at_its_rate_per_link_and_journals_each():
    """Mutation: ignore ``loss`` (nothing lost)."""
    src, run = Sources(), _radio(LinkSpec(0, 0, loss=0.3))
    backend = run.roster.add(BACKEND, ROUTER)
    for k in range(1, 1501):
        run.tick(k, egress={serial1(1): src.heartbeat(1, 1)}, admitted=[backend])
    lost = _events(run, "lost")
    by_link: dict[str, int] = {}
    for _k, event in lost:
        by_link[event[1][2]] = by_link.get(event[1][2], 0) + 1
    assert set(by_link) == {serial1(2), serial1(3), ROUTER}
    assert all(0.25 < count / 1500 < 0.35 for count in by_link.values())
    assert {e[2] for _k, e in lost} == {"loss"}
    assert len(lost) + len(_events(run, "arrived")) == 3 * 1500
    assert run.journal().problems() == []


def test_loss_draws_do_not_move_with_the_jitter_draws():
    """Each link's loss has its own stream. Mutation: draw loss from the jitter stream."""
    def lost(jitter: int) -> list:
        src, run = Sources(), _radio(LinkSpec(0, jitter, loss=0.3))
        backend = run.roster.add(BACKEND, ROUTER)
        for k in range(1, 201):
            run.tick(k, egress={serial1(1): src.heartbeat(1, 1)}, admitted=[backend])
        return [e[1] for _k, e in _events(run, "lost")]

    assert lost(0) == lost(3) and lost(0)


def test_one_links_draws_do_not_move_with_other_links_or_the_serial_model():
    """SITL 1 to SITL 2's losses and arrival ticks stay put when other links (SITL 1's own
    to SITL 3 among them) and the serial model carry traffic. Mutations: one loss stream,
    or one jitter stream, for every link; one of each per sender."""
    def watched(busy: bool) -> list:
        spec = LinkSpec(1, 3, loss=0.3)
        src = Sources()
        run = PortsHarness(LinkModels(1, LinkSpec(1, 2, bytes_per_s=11500) if busy else None, spec))
        backend = run.roster.add(BACKEND, ROUTER)
        for k in range(1, 201):
            egress = {serial1(1): src.heartbeat(1, 1)}
            if busy:
                egress |= {serial1(3): src.heartbeat(3, 1), serial0(2): src.heartbeat(2, 1)}
                if k > 1:  # SITL 3 is known from tick 1: SITL 1's request goes to it alone
                    egress[serial1(1)] += src.param((1, 1), 3, 1)
            run.tick(k, egress=egress, admitted=[backend])
        parent = {tuple(e[1]): tuple(e[2]) for _k, e in run.events("copy")}
        return [(k, e[0]) for k, e in run.events("lost") + run.events("arrived")
                if e[1][2] == serial1(2) and parent[tuple(e[1])][2] == serial1(1)]

    quiet = watched(False)
    assert {kind for _k, kind in quiet} == {"lost", "arrived"}
    assert sorted(watched(True)) == sorted(quiet)


def test_an_outage_loses_every_copy_that_leaves_inside_it():
    """Mutation: ``in_outage`` always False."""
    src, run = Sources(), _radio(LinkSpec(0, 0, outages=((5, 8),)))
    backend = run.roster.add(BACKEND, ROUTER)
    for k in range(1, 11):
        run.tick(k, egress={serial1(1): src.heartbeat(1, 1)}, admitted=[backend])
    lost = _events(run, "lost")
    assert sorted({k for k, _e in lost}) == [5, 6, 7] and len(lost) == 9
    assert {e[2] for _k, e in lost} == {"outage"}
    assert run.journal().problems() == []


def test_a_radio_sends_at_its_rate_in_order_and_unused_bytes_are_gone():
    """Mutations: no rate (all leave in tick 2); keep unused credit (the late packet leaves at once)."""
    src, run = Sources(), _radio(LinkSpec(0, 0, bytes_per_s=8000))
    backend = run.roster.add(BACKEND, ROUTER)
    run.tick(1, egress={serial1(i): src.heartbeat(i, 1) for i in (1, 2, 3)}, admitted=[backend],
             done=[(backend, ROUTER, src.heartbeat(255, 190))])
    blobs = [src.blob((255, 190), n) for n in range(4)]
    for k in range(2, 20):
        run.tick(k, admitted=[backend], done=[(backend, ROUTER, b"".join(blobs))] if k == 2 else [])
    size = len(blobs[0])
    # The backend's radio pays 160 bytes a tick from tick 2: blob i leaves once 160 (t - 1) >= size (i + 1).
    expect = [next(t for t in range(2, 40) if 160 * (t - 1) >= size * (i + 1)) for i in range(4)]
    left = sorted({e[2] for _k, e in _events(run, "arrived") if e[1][2] == serial1(2)})
    assert left[-4:] == expect
    for k in range(20, 40):  # idle, then one more: it still takes its bytes' ticks
        run.tick(k, admitted=[backend], done=[(backend, ROUTER, src.blob((255, 190), 9))] if k == 30 else [])
    late = [e[2] for k, e in _events(run, "arrived") if k >= 30 and e[1][2] == serial1(2)]
    assert late == [30 + -(-size // 160) - 1]
    assert run.journal().problems() == []


def test_jitter_never_reorders_one_link_and_parking_ends_copies_in_flight():
    """Mutation: drop the clock's clamp (bytes reach the backend out of order); park
    without ending the radio's copies (``closing`` faults)."""
    src, run = Sources(), _radio(LinkSpec(2, 5))
    backend = run.roster.add(BACKEND, ROUTER)
    sent = [src.heartbeat(1, 1) for _ in range(60)]
    received = b"".join(run.tick(k, egress={serial1(1): sent[k - 1]}, admitted=[backend])[0].get(ROUTER, b"")
                        for k in range(1, 61))
    assert b"".join(sent).startswith(received) and len(received) < len(b"".join(sent))
    run.park(61)
    in_flight = _events(run, "in_flight")
    assert in_flight and all(e[2] == len(sent[0]) for _k, e in in_flight)
    assert run.journal().problems() == []
    run.ports.closing()


# --- off, and repeatable ----------------------------------------------------------------------


def _script(run: PortsHarness) -> list[dict]:
    src = Sources()
    backend = run.roster.add(BACKEND, ROUTER)
    navpys = [run.roster.add(i, serial0(i)) for i in (1, 2, 3)]
    for k in range(1, 30):
        egress = {serial0(i): src.heartbeat(i, 1) for i in (1, 2, 3)}
        egress.update({serial1(i): src.heartbeat(i, 1) for i in (1, 2, 3)})
        done = [(n, serial0(n.ordinal), src.heartbeat(n.ordinal, 191)) for n in navpys]
        done.append((backend, ROUTER, src.heartbeat(255, 190) + src.param((255, 190), 2, 1)))
        run.tick(k, egress=egress, admitted=[backend, *navpys], done=done)
    run.park(30)
    return run.records


def _strip(records: list[dict]) -> list[dict]:
    out = copy.deepcopy(records)
    for record in out:
        record.pop("link_held", None)
        events = record["parked"]["packets"] if "parked" in record else record["packets"]
        events[:] = [e for e in events if e[0] != "arrived"]
    return out


def test_zero_links_change_nothing_but_their_own_notes():
    """With every effect at zero the run is the perfect link's, event for event."""
    plain = _script(PortsHarness())
    zero = _script(PortsHarness(LinkModels(1, LinkSpec(0, 0), LinkSpec(0, 0))))
    assert _strip(zero) == plain
    assert any(e[0] == "arrived" for r in zero if "packets" in r for e in r["packets"])


@pytest.mark.parametrize("make", [_serial, _radio])
def test_one_seed_gives_one_run_and_another_seed_another(make):
    spec = LinkSpec(1, 4, 0.2 if make is _radio else 0.0)
    first, again, other = (_script(make(spec, seed)) for seed in (1, 1, 2))
    assert first == again and first != other
