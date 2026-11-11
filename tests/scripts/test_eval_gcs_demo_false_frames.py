"""Todo 121(a): a frame the radio's byte loss made, with an id in the NAVLINK range, that the
router's id filter drops as designed is no NAVLINK forwarding failure, and nothing else is.

The router check exempts a ``router_dropped`` or ``router_filtered`` packet only when its root
emission's full packet id is in the radio's ``false`` set (``radio_link.radio_events``), whatever
its source. ``stress`` seed 40 on ``bc318089`` failed on exactly such a frame: ``['sitl3', 1,
'sitl3.serial1', 10574]``, msg 25603 from system 0/224, ``router_filtered`` at tick 9262, with
the same id recorded ``false`` (``.logs/handoff/seed40-oracle-review-2026-11-10.md``).
"""

from __future__ import annotations

import copy

import pytest

from navpy.sim.lockstep.coordinator_mavlink import NAVLINK_IDS
from navpy.sim.lockstep.coordinator_types import ROUTER_DEVICE
from scripts.eval_gcs_demo_fleet_gates import certified_lockstep_gate_errors
from scripts.eval_gcs_demo_fleet_packets import forwarding_errors, packet_run_errors, unverified_sources
from scripts.eval_gcs_demo_scenario import CERTIFIED_SCENARIO_PATH, load_resolved_plan, load_scenario
from scripts.eval_gcs_demo_slice_gates import GCS_SYSTEM
from tests.scripts.fleet_journal_evidence import (
    AUCTION_TICK,
    OWNER,
    PEERS,
    SYS_IDS,
    fleet_records,
    vehicle_lines,
    workflow,
    write_fleet_run,
)

#: Seed 40's frame: its full packet id and message id (not in the router entry table).
FALSE_ID = ["sitl3", 1, "sitl3.serial1", 10574]
FALSE_MSGID = 25603
OUTSIDE, FLEET, GCS = [0, 224], [2, 1], [GCS_SYSTEM, 190]
ENDS = ["router_filtered", "router_dropped"]
K = AUCTION_TICK + 2


@pytest.fixture
def fleet():
    base, approvals = workflow()
    lines = vehicle_lines(base)
    return fleet_records(base, lines), lines, approvals


def _forwarding(records):
    return forwarding_errors(records, OWNER, PEERS, SYS_IDS)


def _tick_record(records, k):
    return next(r for r in records if r.get("k") == k)


def _with_false_frame(records, end, source=OUTSIDE):
    """``records`` with seed 40's frame emitted on SITL 3's serial1 at tick ``K`` and ended by
    ``end`` there; no radio evidence yet."""
    assert FALSE_MSGID in NAVLINK_IDS
    changed = copy.deepcopy(records)
    emitted = ["emitted", list(FALSE_ID), [490637, 490806], "0" * 64, FALSE_MSGID, list(source), [-1, -1]]
    _tick_record(changed, K)["packets"] += [emitted, [end, list(FALSE_ID)]]
    return changed, emitted


def _mark_false(records, packet_id, k=K):
    """The radio's ``false`` record of ``packet_id`` in tick ``k``'s record."""
    _tick_record(records, k).setdefault("radio", []).append(["false", packet_id[2], list(packet_id)])


def _navlink_error(end, k=K, packet_id=FALSE_ID):
    return f"forwarding: NAVLINK {list(packet_id)} {end} at tick {k}"


def test_the_passing_fleet_has_no_forwarding_errors(fleet):
    assert _forwarding(fleet[0]) == []


# The exact false frame ---------------------------------------------------------------

@pytest.mark.parametrize("end", ENDS)
@pytest.mark.parametrize("source", [OUTSIDE, FLEET, GCS], ids=["outside", "fleet", "gcs"])
def test_a_false_frame_with_a_navlink_id_the_router_did_not_route_is_no_forwarding_failure(fleet, end, source):
    """Any source: a false frame's source is as random as its id, so it can name a fleet sysid or
    the ground station; ``unverified_sources`` (outside sources only) must not be the exemption."""
    changed, emitted = _with_false_frame(fleet[0], end, source)
    _mark_false(changed, FALSE_ID)
    assert _forwarding(changed) == []
    # The source report keeps its meaning: false emissions from outside the fleet only.
    assert unverified_sources(changed, SYS_IDS) == ([emitted] if source == OUTSIDE else [])


@pytest.mark.parametrize("end", ENDS)
def test_seed40s_frame_without_its_false_record_fails_as_before(fleet, end):
    changed, _emitted = _with_false_frame(fleet[0], end, FLEET)
    assert _forwarding(changed) == [_navlink_error(end)]


# Evidence that is not this packet's ----------------------------------------------------

#: A ``false`` record differing from the frame's id in exactly one part, or naming a router copy.
MISMATCHES = {
    "origin": ["sitl2", 1, "sitl3.serial1", 10574],
    "generation": ["sitl3", 2, "sitl3.serial1", 10574],
    "device": ["sitl3", 1, "sitl2.serial1", 10574],
    "sequence": ["sitl3", 1, "sitl3.serial1", 10575],
    "router_copy": ["router", 1, ROUTER_DEVICE, 10574],
}


@pytest.mark.parametrize("end", ENDS)
@pytest.mark.parametrize("mismatch", sorted(MISMATCHES))
def test_a_false_record_of_any_other_packet_id_excuses_nothing(fleet, end, mismatch):
    changed, _emitted = _with_false_frame(fleet[0], end, FLEET)
    _mark_false(changed, MISMATCHES[mismatch])
    assert _forwarding(changed) == [_navlink_error(end)]


@pytest.mark.parametrize("end", ENDS)
def test_a_mismatched_false_record_leaves_both_the_source_and_the_router_errors(fleet, end):
    changed, _emitted = _with_false_frame(fleet[0], end, OUTSIDE)
    _mark_false(changed, MISMATCHES["sequence"])
    assert _forwarding(changed) == [f"forwarding: {FALSE_ID} from system 0 at tick {K}", _navlink_error(end)]


def test_only_a_false_record_names_a_packet_false(fleet):
    """A ``broken`` record of the same bytes is not ``false``."""
    changed, emitted = _with_false_frame(fleet[0], "router_filtered", FLEET)
    _tick_record(changed, K)["radio"] = [["broken", FALSE_ID[2], K, emitted[2], emitted[3], FALSE_MSGID,
                                          emitted[5], emitted[6], {}]]
    assert _forwarding(changed) == [_navlink_error("router_filtered")]


# Genuine NAVLINK and source failures stay failures ------------------------------------

def _genuine_navlink_unrouted(records, end):
    """``records`` where the first routed (genuine) packet ends at ``end``, its copies gone."""
    dropped = copy.deepcopy(records)
    record, routed = next((r, e) for r in dropped for e in r.get("packets", ()) if e[0] == "routed")
    copy_ids = [e[1] for e in record["packets"] if e[0] == "copy" and e[2] == routed[1]]
    for r in dropped:
        if "packets" in r:
            r["packets"] = [e for e in r["packets"] if not (
                e is routed or (e[0] == "copy" and e[2] == routed[1]) or e[1] in copy_ids)]
    record["packets"].append([end, routed[1]])
    return dropped, routed[1], record["k"]


@pytest.mark.parametrize("end", ENDS)
def test_a_genuine_navlink_packet_the_router_did_not_route_still_fails_beside_a_false_frame(fleet, end):
    """The genuine packet's NAVLINK error stays (its advert's stop is reported too); the excused
    false frame adds and removes nothing."""
    dropped, genuine_id, k = _genuine_navlink_unrouted(fleet[0], end)
    before = _forwarding(dropped)
    assert _navlink_error(end, k, genuine_id) in before
    changed, _emitted = _with_false_frame(dropped, end, FLEET)
    _mark_false(changed, FALSE_ID)
    assert _forwarding(changed) == before


def test_a_genuine_foreign_source_still_fails_beside_a_false_frame(fleet):
    changed, _emitted = _with_false_frame(fleet[0], "router_filtered", OUTSIDE)
    _mark_false(changed, FALSE_ID)
    genuine = next(e for r in changed for e in r.get("packets", ())
                   if e[0] == "emitted" and e[1][0] == "sitl2")
    genuine[5] = [7, 1]
    assert _forwarding(changed) == [f"forwarding: {genuine[1]} from system 7 at tick {K}"]


# Every journal record that carries radio evidence --------------------------------------

def _radio_in_a_later_tick(records):
    _mark_false(records, FALSE_ID, k=K + 3)


def _radio_in_the_parked_record(records):
    records.insert(-1, {"parked": {"tick": records[-2]["k"] + 1, "radio": [["false", FALSE_ID[2], FALSE_ID]]}})


def _radio_in_the_close_record(records):
    records.insert(-1, {"mavlink_counts": {}, "radio": [["false", FALSE_ID[2], FALSE_ID]]})


@pytest.mark.parametrize("place", [_radio_in_a_later_tick, _radio_in_the_parked_record, _radio_in_the_close_record],
                         ids=["later_tick", "parked", "close"])
def test_the_false_record_counts_from_every_record_radio_events_reads(fleet, place):
    changed, _emitted = _with_false_frame(fleet[0], "router_filtered", FLEET)
    place(changed)
    assert _forwarding(changed) == []


# Router copies follow their root -------------------------------------------------------

def test_a_copy_of_a_false_root_is_judged_by_that_root(fleet):
    """A terminal on a router copy is judged by the copy's root emission, as before."""
    changed, _emitted = _with_false_frame(fleet[0], "router_filtered", FLEET)
    record = _tick_record(changed, K)
    record["packets"].remove(["router_filtered", FALSE_ID])
    copy_id = ["router", 1, ROUTER_DEVICE, 99999]
    record["packets"] += [["routed", FALSE_ID, 1], ["copy", copy_id, FALSE_ID], ["router_dropped", copy_id]]
    assert _forwarding(changed) == [_navlink_error("router_dropped", packet_id=copy_id)]
    _mark_false(changed, copy_id)  # the copy is not the root: excuses nothing
    assert _forwarding(changed) == [_navlink_error("router_dropped", packet_id=copy_id)]
    _mark_false(changed, FALSE_ID)
    assert _forwarding(changed) == []


# The run's other gates are untouched ---------------------------------------------------

def test_a_terminal_for_a_packet_never_emitted_is_still_a_journal_problem(fleet):
    changed = copy.deepcopy(fleet[0])
    stray = ["sitl3", 1, "sitl3.serial1", 77777]
    _tick_record(changed, K)["packets"].append(["router_filtered", stray])
    _mark_false(changed, stray)
    assert packet_run_errors(changed) != []


def _certified_errors(tmp_path, records, lines, approvals):
    write_fleet_run(tmp_path, records, lines)
    plan = load_resolved_plan(tmp_path / "demo_mission_plan.json", load_scenario(CERTIFIED_SCENARIO_PATH))
    return certified_lockstep_gate_errors(tmp_path, plan, approvals)


@pytest.mark.parametrize(("kind", "prefix"), [
    ("packets", "packets: "), ("forwarding", "forwarding: "), ("run", "lockstep outcome 'timeout'"),
    ("approval", "vehicle 1: confirmation timing: audited"), ("workflow", "vehicle 2: workflow ticks"),
])
def test_a_false_frame_hides_no_other_failure_of_the_certified_run(tmp_path, fleet, kind, prefix):
    records, lines, approvals = copy.deepcopy(fleet)
    records, _emitted = _with_false_frame(records, "router_filtered", OUTSIDE)
    _mark_false(records, FALSE_ID)
    if kind == "packets":
        record, delivered = next((r, e) for r in records for e in r.get("packets", ()) if e[0] == "delivered")
        record["packets"].append(["discarded", delivered[1], "drain"])
    elif kind == "forwarding":
        next(e for r in records for e in r.get("packets", ())
             if e[0] == "emitted" and e[1] != FALSE_ID)[5] = [7, 1]
    elif kind == "run":
        records[-1] = {**records[-1], "outcome": "timeout"}
    elif kind == "approval":
        approvals[0]["actual_delay_s"] += 0.02
    elif kind == "workflow":
        lines[2] = [row for row in lines[2] if "assigned by owner" not in row[1]]
    errors = _certified_errors(tmp_path, records, lines, approvals)
    assert any(error.startswith(prefix) for error in errors), errors
    assert not any(str(FALSE_ID) in error and "NAVLINK" in error for error in errors), errors


def test_with_only_the_false_frame_the_certified_run_passes(tmp_path, fleet):
    records, lines, approvals = copy.deepcopy(fleet)
    records, _emitted = _with_false_frame(records, "router_filtered", OUTSIDE)
    _mark_false(records, FALSE_ID)
    assert _certified_errors(tmp_path, records, lines, approvals) == []
