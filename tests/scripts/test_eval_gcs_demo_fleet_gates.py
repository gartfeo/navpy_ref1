"""The certified lockstep run's journal gates, each on passing and failing evidence (step 10g-3)."""

from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest

from navpy.core.link.swarm_ack_msg import ACK_STATUS_APPLIED, ACK_STATUS_RECEIVED
from navpy.sim.lockstep.coordinator_types import serial0_device
from scripts.eval_certificate_source_time import SOURCE_TIME_SUBDIR
from scripts.eval_gcs_demo_confirmation import CONFIRMATION_POLL_S
from scripts.eval_gcs_demo_constants import backend_log_path
from scripts.eval_gcs_demo_confirmation_timing import (
    RESPONSE_ID,
    approval_timing_errors,
    confirmation_ticks,
    max_frame_gap_us,
    tick_times,
)
from scripts.eval_gcs_demo_fleet_gates import (
    backend_assignment_errors,
    certified_lockstep_gate_errors,
    fleet_approval_errors,
    fleet_snap_errors,
    navpy_lines_file,
    role_order_errors,
    vehicle_workflow_errors,
)
from scripts import eval_gcs_demo_fleet_gates
from scripts.eval_gcs_demo_fleet_packets import forwarding_errors, held_tasks, packet_run_errors
from scripts.eval_gcs_demo_inputs import INPUT_MANIFEST, recorded_profile_parameters
from scripts.eval_gcs_demo_models import RegressionError
from scripts.eval_gcs_demo_scenario import CERTIFIED_SCENARIO_PATH, load_resolved_plan, load_scenario
from scripts.eval_gcs_demo_slice_gates import JOURNAL_FILE, pose_cadence_errors, run_errors
from tests.scripts.fleet_journal_evidence import (
    ACK,
    ADVERT,
    AUCTION_TICK,
    OFFER,
    OWNER,
    PEERS,
    REQUEST,
    REQUEST_TYPE,
    RESPONSE,
    RESPONSE_TYPE,
    SYS_IDS,
    Packets,
    auction_packets,
    fleet_records,
    vehicle_lines,
    workflow,
    write_backend_log,
    write_fleet_run,
)

CERTIFIED = load_scenario(CERTIFIED_SCENARIO_PATH)


@pytest.fixture
def fleet():
    base, approvals = workflow()
    lines = vehicle_lines(base)
    return fleet_records(base, lines), lines, approvals


def _plan(log_dir: Path):
    return load_resolved_plan(log_dir / "demo_mission_plan.json", CERTIFIED)


def _forwarding(records):
    return forwarding_errors(records, OWNER, PEERS, SYS_IDS)


def _events(records):
    return [(r, e) for r in records for e in r.get("packets", ())]


def _drop(records, match):
    """The journal without the first packet event ``match(event)`` selects."""
    changed = copy.deepcopy(records)
    record, event = next((r, e) for r, e in _events(changed) if match(e))
    record["packets"].remove(event)
    return changed


def _uid_events(records, msgid, sender):
    """Every event of the first packet NavPy ``sender`` emitted with ``msgid``, by its UID."""
    uid = next(e[7]["uid"] for _r, e in _events(records)
               if e[0] == "emitted" and e[4] == msgid and e[1][0] == sender)
    return uid, [e for _r, e in _events(records)
                 if e[0] == "emitted" and len(e) > 7 and e[7]["uid"] == uid]


def test_passing_evidence_passes_every_gate(tmp_path):
    approvals = write_fleet_run(tmp_path)
    assert certified_lockstep_gate_errors(tmp_path, _plan(tmp_path), approvals) == []


def test_a_run_without_a_peers_lines_file_or_with_other_sysids_fails(tmp_path):
    approvals = write_fleet_run(tmp_path)
    plan = _plan(tmp_path)
    (tmp_path / navpy_lines_file(2)).unlink()
    assert certified_lockstep_gate_errors(tmp_path, plan, approvals) == [
        "certified lockstep evidence: the certified lockstep run needs lockstep_navpy2_lines.jsonl"]
    shifted = plan.__class__(**{**plan.__dict__, "vehicles": tuple(
        v.__class__(**{**v.__dict__, "sys_id": v.sys_id + 3}) for v in plan.vehicles)})
    assert certified_lockstep_gate_errors(tmp_path, shifted, approvals) == [
        "certified lockstep: sysids [4, 5, 6] are not the fleet's 1..3"]


# Run -----------------------------------------------------------------------

def test_run_gate_fails_on_a_fault_a_second_disposition_or_an_undelivered_navlink_packet(fleet):
    records, _lines, _approvals = fleet
    assert run_errors(records) + packet_run_errors(records) == []
    faulted = records[:-1] + [{"fault": "CoordinatorFault", "kind": "fleet_clock", "tick": 9, "ordinal": None}]
    assert run_errors(faulted) == ["lockstep fault CoordinatorFault fleet_clock at tick 9",
                                   "the journal records no run outcome"]
    twice = copy.deepcopy(records)
    record, delivered = next((r, e) for r, e in _events(twice) if e[0] == "delivered")
    record["packets"].append(["discarded", delivered[1], "drain"])
    assert packet_run_errors(twice) == [f"packets: {delivered[1]} has 2 terminal dispositions"]
    parked = copy.deepcopy(records)
    record, delivered = next((r, e) for r, e in _events(parked) if e[0] == "delivered"
                             and isinstance(e[2], list))
    record["packets"].remove(delivered)
    parked.append({"parked": {"tick": 999, "packets": [["undelivered", delivered[1], 20, 20]]}})
    assert packet_run_errors(parked) == [
        f"packets: NAVLINK {delivered[1]} (msg {ADVERT}) undelivered at parking (tick 999)"]


# Assignment and forwarding ---------------------------------------------------

def test_every_peer_holds_its_task_by_the_complete_chain(fleet):
    records, _lines, _approvals = fleet
    assert held_tasks(records, OWNER, SYS_IDS) == {2: {2}, 3: {3}}
    assert _forwarding(records) == []


@pytest.mark.parametrize(("msgid", "sender", "hop"), [
    (REQUEST, OWNER, "copy"),  # the request never reached the helper's SITL
    (ACK, 2, "participant"),  # the helper's ACK never reached the owner's NavPy
    (ACK, OWNER, "participant"),  # the owner's APPLIED never reached the helper
])
def test_a_chain_link_that_stops_short_leaves_its_peer_without_a_task(fleet, msgid, sender, hop):
    records, _lines, _approvals = fleet
    uid, emitted = _uid_events(records, msgid, sender)
    ids = {tuple(e[1]) for e in emitted}
    if hop == "copy":
        copies = {tuple(e[1]) for _r, e in _events(records) if e[0] == "copy" and tuple(e[2]) in ids}
        changed = _drop(records, lambda e: e[0] == "delivered" and tuple(e[1]) in copies)
    else:
        changed = _drop(records, lambda e: e[0] == "delivered" and tuple(e[1]) in ids
                        and isinstance(e[2], list))
    assert held_tasks(changed, OWNER, SYS_IDS) == {3: {3}}
    assert "forwarding: peer 2 holds tasks [] by a complete chain, not one" in _forwarding(changed)


@pytest.mark.parametrize(("sender", "status", "ref_type"), [
    (OWNER, ACK_STATUS_RECEIVED, RESPONSE_TYPE),  # the owner's ACK of the response is not APPLIED
    (OWNER, ACK_STATUS_APPLIED, REQUEST_TYPE),  # ... or names the request's type
    (2, ACK_STATUS_APPLIED, REQUEST_TYPE),  # the helper's ACK of the request is not RECEIVED
    (2, ACK_STATUS_RECEIVED, RESPONSE_TYPE),  # ... or names the response's type
])
def test_an_ack_with_another_status_or_ref_type_holds_nothing(fleet, sender, status, ref_type):
    """NavPy routes a SWARM_ACK by its ref's MsgType (task_ack_routing)."""
    records, _lines, _approvals = fleet
    changed = copy.deepcopy(records)
    _uid, emitted = _uid_events(changed, ACK, sender)
    for event in emitted:
        event[7]["ack"]["status"] = status
        event[7]["ack"]["ref"][2] = ref_type
    assert held_tasks(changed, OWNER, SYS_IDS) == {3: {3}}


@pytest.mark.parametrize(("position", "value"), [(1, 9999), (2, 9999)])
def test_a_response_not_after_the_helpers_ack_holds_nothing(fleet, position, value):
    """The response must follow the helper's ACK: same boot, later msg_seq (on_request_ack)."""
    records, _lines, _approvals = fleet
    changed = copy.deepcopy(records)
    _uid, emitted = _uid_events(changed, ACK, 2)
    for event in emitted:
        event[7]["uid"][position] = value
    assert held_tasks(changed, OWNER, SYS_IDS) == {3: {3}}


def test_the_fence_floor_is_the_helpers_first_ack():
    """A later RECEIVED ACK of the same request (a resend's) moves no floor."""
    base, _approvals = workflow()
    lines = vehicle_lines(base)
    packets = auction_packets()
    request = next(e[7]["uid"] for events in packets.events.values() for e in events
                   if e[0] == "emitted" and e[4] == REQUEST and e[6][0] == 2)
    packets.send(AUCTION_TICK + 40, ACK, 2, (OWNER,), OWNER,
                 ack={"ref": [request[1], request[2], REQUEST_TYPE], "status": ACK_STATUS_RECEIVED})
    assert held_tasks(fleet_records(base, lines, packets), OWNER, SYS_IDS) == {2: {2}, 3: {3}}


def test_a_response_for_another_task_holds_nothing(fleet):
    records, _lines, _approvals = fleet
    changed = copy.deepcopy(records)
    _uid, emitted = _uid_events(changed, RESPONSE, 2)
    for event in emitted:
        event[7]["task_id"] = 9
    assert held_tasks(changed, OWNER, SYS_IDS) == {3: {3}}


def test_two_peers_holding_one_task_fail():
    base, _approvals = workflow()
    lines = vehicle_lines(base)
    records = fleet_records(base, lines, auction_packets({2: 2, 3: 2}))
    assert held_tasks(records, OWNER, SYS_IDS) == {2: {2}, 3: {2}}
    assert _forwarding(records) == ["forwarding: peers [2, 3] hold one task [2, 2]"]


def test_an_ambiguous_request_resend_is_not_a_chain():
    """Two sends of one UID with the same bytes: no pairing can attribute them."""
    base, _approvals = workflow()
    lines = vehicle_lines(base)
    packets = auction_packets()
    records = fleet_records(base, lines, packets)
    uid, emitted = _uid_events(records, REQUEST, OWNER)
    first = emitted[0]
    again = Packets(seqs={(OWNER, first[1][2]): 99})
    packet_id = again._id(OWNER, first[1][2])
    records[5]["packets"] += [["emitted", packet_id, *first[2:]],
                              ["discarded", packet_id, "drain"]]
    assert held_tasks(records, OWNER, SYS_IDS) == {3: {3}}


def test_an_advert_that_stops_short_fails_unless_sent_after_stop_all(fleet):
    records, _lines, _approvals = fleet
    _uid, emitted = _uid_events(records, ADVERT, OWNER)
    down = {tuple(e[1]) for e in emitted if e[1][0] == "sitl3"}
    changed = _drop(records, lambda e: e[0] == "delivered" and tuple(e[1]) in down)
    errors = _forwarding(changed)
    assert len(errors) == 1 and errors[0].startswith(f"forwarding: advert {ADVERT} ")
    assert errors[0].endswith("from 1 to 3 stopped at ap_serial0")
    stopped = copy.deepcopy(changed)
    stopped[5]["lifecycle"] = [{"kind": "stop"}]  # stop-all's first request, before the advert
    assert _forwarding(stopped) == []
    # SITL's byte loss explains only a missing SITL re-emission, never a missing coordinator hop.
    assert forwarding_errors(changed, OWNER, PEERS, SYS_IDS, uart_loss=True) == errors


@pytest.mark.parametrize("sitl", ["sitl1", "sitl3"])
def test_with_sitl_byte_loss_an_advert_may_stop_only_at_a_sitls_write(fleet, sitl):
    """Step 3 of deterministic_dev_sim.md: SITL 1's serial1 re-emission or SITL 3's serial0
    re-emission missing is SITL's loss with ``uart_loss`` and a fault without it; a lost router
    copy is a fault either way."""
    records, _lines, _approvals = fleet
    _uid, emitted = _uid_events(records, ADVERT, OWNER)
    gone = copy.deepcopy(records)  # the SITL never wrote its re-emission: every record of it goes
    ids = {tuple(e[1]) for e in emitted if e[1][0] == sitl}
    ids |= {tuple(e[1]) for _r, e in _events(gone) if e[0] == "copy" and tuple(e[2]) in ids}
    for record in gone:
        if "packets" in record:
            record["packets"] = [e for e in record["packets"] if tuple(e[1]) not in ids]
    stop = "sitl_serial0" if sitl == "sitl1" else "router_copy"
    errors = _forwarding(gone)
    assert errors and all(e.startswith(f"forwarding: advert {ADVERT} ") and e.endswith(f"stopped at {stop}")
                          for e in errors)
    assert forwarding_errors(gone, OWNER, PEERS, SYS_IDS, uart_loss=True) == []
    up = {tuple(e[1]) for e in emitted if e[1][0] == "sitl1" and e[1][2] == "sitl1.serial1"}
    copies = {tuple(e[1]) for _r, e in _events(records)
              if e[0] == "copy" and tuple(e[2]) in up and e[1][2] == "sitl3.serial1"}
    no_copy = _drop(records, lambda e: e[0] == "delivered" and tuple(e[1]) in copies)
    lost_copy = forwarding_errors(no_copy, OWNER, PEERS, SYS_IDS, uart_loss=True)
    assert len(lost_copy) == 1 and lost_copy[0].endswith("from 1 to 3 stopped at ap_serial1")


def test_the_certified_gate_skips_adverts_only_when_the_manifest_records_byte_loss(tmp_path, monkeypatch):
    """Step 3 of deterministic_dev_sim.md: ``uart_loss`` comes from the run's own input manifest;
    no manifest, or a loss of 0, keeps the advert check; a broken manifest fails the gate."""
    approvals = write_fleet_run(tmp_path)
    plan = _plan(tmp_path)
    seen = []

    def spy(*args, uart_loss):
        seen.append(uart_loss)
        return []

    monkeypatch.setattr(eval_gcs_demo_fleet_gates, "forwarding_errors", spy)
    manifest = tmp_path / INPUT_MANIFEST
    assert certified_lockstep_gate_errors(tmp_path, plan, approvals) == []
    for loss in (0.0, 0.15):
        manifest.write_text(json.dumps({"sitl": {"profile_parameters": {
            "SIM_GPS_NOISE": 0.5, "SIM_UART_LOSS": loss}}}), encoding="utf-8")
        assert certified_lockstep_gate_errors(tmp_path, plan, approvals) == []
    assert seen == [False, False, True]
    manifest.write_text(json.dumps({"sitl": {"profile_parameters": {"SIM_UART_LOSS": "x"}}}),
                        encoding="utf-8")
    errors = certified_lockstep_gate_errors(tmp_path, plan, approvals)
    assert len(errors) == 1 and errors[0].startswith("certified lockstep evidence: input manifest")


def test_recorded_profile_parameters_reads_the_manifests_sitl_parameters(tmp_path):
    assert recorded_profile_parameters(tmp_path) == {}
    (tmp_path / INPUT_MANIFEST).write_text(json.dumps({"sitl": {}}), encoding="utf-8")
    assert recorded_profile_parameters(tmp_path) == {}
    (tmp_path / INPUT_MANIFEST).write_text(json.dumps({"sitl": {"profile_parameters": {
        "SIM_BAUDLIMIT_EN": 1, "SIM_UART_LOSS": 0.15}}}), encoding="utf-8")
    assert recorded_profile_parameters(tmp_path) == {"SIM_BAUDLIMIT_EN": 1.0, "SIM_UART_LOSS": 0.15}
    for bad in ('{"sitl": {"profile_parameters": [1]}}', '{"sitl": {"profile_parameters": {"A": true}}}',
                '{"sitl": {"profile_parameters": {"SIM_UART_LOSS": Infinity}}}',
                '{"sitl": {"profile_parameters": {"SIM_UART_LOSS": NaN}}}',
                '{"sitl": {"profile_parameters": {"SIM_UART_LOSS": 1e309}}}', "{not json"):
        (tmp_path / INPUT_MANIFEST).write_text(bad, encoding="utf-8")
        with pytest.raises(RegressionError):
            recorded_profile_parameters(tmp_path)


def test_a_source_outside_the_fleet_and_a_router_dropped_navlink_packet_fail(fleet):
    records, _lines, _approvals = fleet
    changed = copy.deepcopy(records)
    emitted = next(e for _r, e in _events(changed) if e[0] == "emitted" and e[1][0] == "sitl2")
    emitted[5] = [7, 1]
    assert _forwarding(changed) == [f"forwarding: {emitted[1]} from system 7 at tick {AUCTION_TICK + 2}"]
    dropped = copy.deepcopy(records)
    record, routed = next((r, e) for r, e in _events(dropped) if e[0] == "routed")
    copies = [e for e in record["packets"] if e[0] == "copy" and e[2] == routed[1]]
    for event in copies + [routed]:
        record["packets"].remove(event)
    record["packets"].append(["router_dropped", routed[1]])
    copy_ids = [c[1] for c in copies]
    for r, e in _events(dropped):
        if e[1] in copy_ids:
            r["packets"].remove(e)
    errors = _forwarding(dropped)
    assert f"forwarding: NAVLINK {routed[1]} router_dropped at tick {AUCTION_TICK + 2}" in errors


def _cross_check(tmp_path, records, packets):
    write_fleet_run(tmp_path, records)
    write_backend_log(tmp_path, packets)
    return backend_assignment_errors(tmp_path, records, _plan(tmp_path))


def _repeat_applied_discarded_at_stop(ref_msgid=RESPONSE):
    """A REPEAT's APPLIED of peer 2's response (or of its ``ref_msgid`` packet), logged by
    the backend, whose copy to the retiring helper is discarded (``coordinator_router``
    gives the backend its copy first)."""
    base, _approvals = workflow()
    lines = vehicle_lines(base)
    packets = auction_packets()
    peer, task, resp, _applied = packets.applied[0]
    if ref_msgid != RESPONSE:
        resp = next(e[7]["uid"] for events in packets.events.values() for e in events
                    if e[0] == "emitted" and e[4] == ref_msgid and e[1][0] == peer)
    k = AUCTION_TICK + 50
    again = packets.send(k, ACK, OWNER, (peer,), peer,
                         ack={"ref": [resp[1], resp[2], RESPONSE_TYPE], "status": ACK_STATUS_APPLIED})
    packets.applied.append((peer, task, resp, again))
    records = fleet_records(base, lines, packets)
    for event in next(r for r in records if r.get("k") == k + 5)["packets"]:
        if event[0] == "delivered" and isinstance(event[2], list):
            event[:] = ["discarded", event[1], "retired"]
    return records, packets


@pytest.mark.parametrize(("status", "ref_type"), [(ACK_STATUS_RECEIVED, RESPONSE_TYPE),
                                                   (ACK_STATUS_APPLIED, REQUEST_TYPE)])
def test_a_logged_applied_needs_the_journals_applied_of_the_response(tmp_path, status, ref_type):
    records, packets = _repeat_applied_discarded_at_stop()
    again = packets.applied[-1][3]
    for _record, event in _events(records):
        if event[0] == "emitted" and len(event) > 7 and event[7].get("uid") == again:
            event[7]["ack"].update(status=status, ref=[*event[7]["ack"]["ref"][:2], ref_type])
    (error,) = _cross_check(tmp_path, records, packets)
    assert error.startswith("assignment cross-check: backend APPLIED [1, 2, 2, ") and error.endswith(
        f"({again[1]}, {again[2]})] is no journal APPLIED")


def test_a_logged_applied_needs_the_owners_own_emission_of_a_response(tmp_path):
    """Neither a SITL's forwarding of the APPLIED nor an APPLIED naming the helper's
    same-task offer (claiming the response's type) is the journal's APPLIED."""
    records, packets = _repeat_applied_discarded_at_stop()
    again = packets.applied[-1][3]
    own = {tuple(e[1]) for _r, e in _events(records)
           if e[0] == "emitted" and e[1][0] == OWNER and len(e) > 7 and e[7]["uid"] == again}
    for record in records:
        if "packets" in record:
            record["packets"] = [e for e in record["packets"] if tuple(e[1]) not in own]
    (error,) = _cross_check(tmp_path, records, packets)
    assert error.endswith(f"({again[1]}, {again[2]})] is no journal APPLIED")
    records, packets = _repeat_applied_discarded_at_stop(OFFER)
    (error,) = _cross_check(tmp_path, records, packets)
    assert error.startswith("assignment cross-check: backend APPLIED [1, 2, 2, ")


def test_the_backend_logs_applied_lines_are_the_journals_chains(tmp_path, fleet):
    records, _lines, _approvals = fleet
    assert _cross_check(tmp_path, records, auction_packets()) == []
    other = auction_packets()
    peer, _task, resp, applied = other.applied[0]
    other.applied[0] = (peer, 3, resp, applied)  # the backend's round names another task
    assert _cross_check(tmp_path, records, other) == [
        f"assignment cross-check: backend APPLIED [1, 2, 3, ({resp[1]}, {resp[2]}), "
        f"({applied[1]}, {applied[2]})] is no journal APPLIED",
        "assignment cross-check: helper 2 holds task 2 with no backend APPLIED"]
    unlogged = auction_packets()
    del unlogged.applied[1]
    assert _cross_check(tmp_path, records, unlogged) == [
        "assignment cross-check: helper 3 holds task 3 with no backend APPLIED"]
    assert _cross_check(tmp_path, *_repeat_applied_discarded_at_stop()) == []
    write_backend_log(tmp_path, auction_packets())
    path = backend_log_path(tmp_path)
    path.write_text(path.read_text(encoding="utf-8").replace("owner=1 helper=3", "owner=2 helper=3"),
                    encoding="utf-8")
    assert backend_assignment_errors(tmp_path, records, _plan(tmp_path))[0].startswith(
        "assignment cross-check: backend APPLIED [2, 3, 3, ")
    path.unlink()
    (error,) = backend_assignment_errors(tmp_path, records, _plan(tmp_path))
    assert error.startswith("assignment cross-check: ")


# Approval timing --------------------------------------------------------------

def test_each_approval_is_timed_by_its_own_identity(fleet):
    records, lines, approvals = fleet
    assert fleet_approval_errors(records, lines, approvals) == []
    ticks = {a["sys_id"]: confirmation_ticks(records, lines[a["sys_id"]], 1, a["sys_id"],
                                             [a["sys_id"], 1, a["round_uid"]]).k for a in approvals}
    assert [ticks[s]["k2"] for s in SYS_IDS] == [100, 103, 106]
    assert len({ticks[s]["k4"] for s in SYS_IDS}) == 3


def test_a_request_due_during_another_post_passes_beyond_the_slice_wait_bound(fleet):
    """The slice's bound (2 polls + 2 gaps past the delay) does not hold for
    vehicle 2, which became due during vehicle 1's POST: the replay explains it."""
    records, lines, approvals = fleet
    t = tick_times(records)
    k = confirmation_ticks(records, lines[2], 1, 2, [2, 1, approvals[1]["round_uid"]]).k
    bound = 15_000_000 + 2 * round(CONFIRMATION_POLL_S * 1e6) + 2 * max_frame_gap_us(records)
    assert t[k["k4"]] - t[k["k3"]] > bound
    assert approval_timing_errors(records, lines[2], approvals[1]) == []


def test_an_approval_fails_on_its_own_k1_sends_or_audit(fleet):
    records, lines, approvals = fleet
    late = {**lines, 2: [(k + 50 if "CONFIRMING" in line else k, line) for k, line in lines[2]]}
    errors = fleet_approval_errors(records, late, approvals)
    assert errors == ["vehicle 2: confirmation timing: not k1 <= k2 <= k3 (153, 103, 103)"]
    unsent = copy.deepcopy(records)
    k4 = confirmation_ticks(records, lines[3], 1, 3, [3, 1, approvals[2]["round_uid"]]).k["k4"]
    first = next(r for r in unsent if r.get("k") == k4 + 1)["packets"]
    first[:] = [e for e in first if not (e[0] == "emitted" and e[4] == RESPONSE_ID)]
    errors = fleet_approval_errors(unsent, lines, approvals)
    assert errors and all(e.startswith("vehicle 3: confirmation timing:") for e in errors)
    audited = copy.deepcopy(approvals)
    audited[0]["actual_delay_s"] += 0.02
    errors = fleet_approval_errors(records, lines, audited)
    assert errors[0].startswith("vehicle 1: confirmation timing: audited actual_delay_s")
    assert errors[-1].startswith("approval replay: approval [1, 1,")


def test_response_sends_are_the_backends_emissions_to_the_approvals_vehicle(fleet):
    """An emission counts at its own tick, however late its SITL admits it; one to
    another vehicle, or ArduPilot forwarding it out of the SITL's serial0, does not count."""
    records, lines, approvals = fleet
    k4 = confirmation_ticks(records, lines[3], 1, 3, [3, 1, approvals[2]["round_uid"]]).k["k4"]
    by_k = {r["k"]: r for r in records if "k" in r}
    emitted = next(e for e in by_k[k4 + 1]["packets"] if e[0] == "emitted" and e[4] == RESPONSE_ID)
    backlog = copy.deepcopy(records)
    later = {r["k"]: r for r in backlog if "k" in r}
    copy_id = next(e[1] for e in later[k4 + 1]["packets"] if e[0] == "copy" and e[2] == emitted[1])
    moved = [e for e in later[k4 + 1]["packets"] if e[0] in ("admitted", "delivered") and e[1] == copy_id]
    later[k4 + 1]["packets"] = [e for e in later[k4 + 1]["packets"] if e not in moved]
    later[k4 + 5]["packets"] += moved
    assert packet_run_errors(backlog) == []
    assert fleet_approval_errors(backlog, lines, approvals) == []
    forwarded = copy.deepcopy(records)
    packet_id = ["sitl3", 1, serial0_device(3), 999]
    {r["k"]: r for r in forwarded if "k" in r}[k4 + 2]["packets"] += [
        ["emitted", packet_id, *emitted[2:7]], ["delivered", packet_id, [3, 1]]]
    assert packet_run_errors(forwarded) == []
    assert fleet_approval_errors(forwarded, lines, approvals) == []
    retargeted = copy.deepcopy(records)
    for event in {r["k"]: r for r in retargeted if "k" in r}[k4 + 1]["packets"]:
        if event[:2] == emitted[:2]:
            event[6][0] = 2
    errors = fleet_approval_errors(retargeted, lines, approvals)
    assert errors and all(e.startswith("vehicle 3: confirmation timing:") for e in errors)


def test_the_replay_runs_over_every_approval(fleet):
    records, lines, approvals = fleet
    late = copy.deepcopy(records)
    record = next(r for r in late if r.get("operator", {}).get("sleep"))
    sleep = record["operator"]["sleep"][0]
    nxt = next(r for r in late if r.get("k") == record["k"] + 1)
    record["operator"]["sleep"].remove(sleep)
    if not record["operator"]["sleep"]:
        del record["operator"]["sleep"]
    nxt.setdefault("operator", {}).setdefault("sleep", []).append([sleep[0], sleep[1] + 1])
    errors = fleet_approval_errors(late, lines, approvals)
    assert len(errors) == 1 and errors[0].startswith("approval replay: pass 0: sleep")


# Workflow ticks ----------------------------------------------------------------

def test_workflow_ticks_hold_per_vehicle_with_its_role_order(tmp_path, fleet):
    _records, lines, _approvals = fleet
    write_fleet_run(tmp_path)
    plan = _plan(tmp_path)
    assert vehicle_workflow_errors(lines, plan) == []
    unassigned = {**lines, 3: [row for row in lines[3] if "assigned by owner" not in row[1]]}
    assert vehicle_workflow_errors(unassigned, plan) == ["vehicle 3: workflow ticks: no assigned marker"]
    early = {**lines, 2: [(k - 50 if "Peer navigation" in line else k, line) for k, line in lines[2]]}
    early[2].sort(key=lambda row: row[0])
    assert vehicle_workflow_errors(early, plan) == [
        "vehicle 2: workflow ticks: assigned -> approach -> request state out of order at "
        f"{[(98, 1), (49, 0), (102, 4)]}"]
    owner = [(k + 10 if "orbit" in line else k, line) for k, line in lines[1]]
    owner.sort(key=lambda row: row[0])
    assert role_order_errors(owner, "owner")[0].startswith(
        "workflow ticks: orbit -> request state out of order")
    reset_late = {**lines, 1: [(k + 100 if "RESET" in line else k, line) for k, line in lines[1]]}
    reset_late[1].sort(key=lambda row: row[0])
    errors = vehicle_workflow_errors(reset_late, plan)
    assert len(errors) == 1 and errors[0].startswith(
        "vehicle 1: workflow ticks: confirmed -> NAV -> reset -> SNAP out of order")


# Pose cadence and SNAP -----------------------------------------------------------

def test_pose_cadence_needs_both_producers_for_every_sysid(tmp_path):
    write_fleet_run(tmp_path)
    assert pose_cadence_errors(tmp_path, SYS_IDS, ("navpy", "gcs")) == []
    missing = tmp_path / SOURCE_TIME_SUBDIR / "pose_cadence_position_arrival_uav_3_gcs.csv"
    missing.unlink()
    assert pose_cadence_errors(tmp_path, SYS_IDS, ("navpy", "gcs")) == [f"pose cadence: no {missing.name}"]
    torn = tmp_path / SOURCE_TIME_SUBDIR / "pose_cadence_attitude_arrival_uav_2_gcs.csv"
    torn.write_text("wall_s,boot_ms\n1000.0,20\n1000.02\n", encoding="utf-8")
    assert f"pose cadence: {torn.name} row 3 is torn" in pose_cadence_errors(
        tmp_path, SYS_IDS, ("navpy", "gcs"))


def test_snap_gate_needs_k_t_as_the_last_first_snap(fleet):
    records, lines, _approvals = fleet
    assert fleet_snap_errors(records, lines) == []
    outcome = records[-1]
    last = outcome["terminal_tick"]
    early = records[:-1] + [{**outcome, "terminal_tick": last - 1}]
    assert fleet_snap_errors(early, lines) == [
        f"SNAP: vehicle 3's first SNAP at tick {last}, after k_T {last - 1}",
        f"SNAP: k_T {last - 1} is not the last first SNAP {last}"]
    late = records[:-1] + [{**outcome, "terminal_tick": last + 1}]
    assert fleet_snap_errors(late, lines) == [f"SNAP: k_T {last + 1} is not the last first SNAP {last}"]
    latched = records[:-1] + [{**outcome, "first_snaps": [[1, 1, 5], *outcome["first_snaps"][1:]]}]
    assert fleet_snap_errors(latched, lines)[0].startswith("SNAP: vehicle 1's first SNAP line at tick")
    no_snap = {**lines, 2: [row for row in lines[2] if "SNAP" not in row[1]]}
    assert fleet_snap_errors(records, no_snap) == ["SNAP: no terminal tick or no SNAP from vehicles [2]"]


def test_snap_gate_needs_the_latched_generation_to_be_that_ticks_member(fleet):
    """The latch names (ordinal, generation, tick); that tick's members must hold that generation."""
    records, lines, _approvals = fleet
    outcome = records[-1]
    k = next(k for k, line in lines[2] if "SNAP" in line)
    other = [[o, 2 if o == 2 else g, tick] for o, g, tick in outcome["first_snaps"]]
    regenerated = records[:-1] + [{**outcome, "first_snaps": other}]
    assert fleet_snap_errors(regenerated, lines) == [
        f"SNAP: vehicle 2's first SNAP line at tick {k} (generation 1), latched [2, {k}]"]
    absent = copy.deepcopy(records)
    record = next(r for r in absent if r.get("k") == k)
    record["members"] = [m for m in record["members"] if m["ordinal"] != 2]
    assert fleet_snap_errors(absent, lines) == [
        f"SNAP: vehicle 2's first SNAP line at tick {k} (generation None), latched [1, {k}]"]



def _break(kind, records, lines, approvals):
    """One gate's failure in otherwise passing evidence (the pose files are the run directory's)."""
    outcome = records[-1]
    if kind == "run":
        records[-1] = {**outcome, "outcome": "timeout"}
    elif kind == "packets":
        record, delivered = next((r, e) for r, e in _events(records) if e[0] == "delivered")
        record["packets"].append(["discarded", delivered[1], "drain"])
    elif kind == "forwarding":
        next(e for _r, e in _events(records) if e[0] == "emitted")[5] = [7, 1]
    elif kind == "approval":
        approvals[0]["actual_delay_s"] += 0.02
    elif kind == "workflow":
        lines[2] = [row for row in lines[2] if "assigned by owner" not in row[1]]
    elif kind == "snap":
        records[-1] = {**outcome, "terminal_tick": outcome["terminal_tick"] + 1}


@pytest.mark.parametrize(("kind", "prefix"), [
    ("run", "lockstep outcome 'timeout'"), ("packets", "packets: "), ("forwarding", "forwarding: "),
    ("approval", "vehicle 1: confirmation timing: audited"), ("workflow", "vehicle 2: workflow ticks"),
    ("snap", "SNAP: k_T"), ("assignment", "assignment cross-check: helper 2"), ("pose", "pose cadence: no pose_cadence_attitude_arrival_uav_2_gcs.csv"),
])
def test_every_gate_feeds_the_certified_lockstep_run_gates(tmp_path, fleet, kind, prefix):
    records, lines, approvals = copy.deepcopy(fleet)
    _break(kind, records, lines, approvals)
    write_fleet_run(tmp_path, records, lines)
    if kind == "pose":
        (tmp_path / SOURCE_TIME_SUBDIR / "pose_cadence_attitude_arrival_uav_2_gcs.csv").unlink()
    if kind == "assignment":
        write_backend_log(tmp_path, Packets())
    errors = certified_lockstep_gate_errors(tmp_path, _plan(tmp_path), approvals)
    assert any(error.startswith(prefix) for error in errors), errors
