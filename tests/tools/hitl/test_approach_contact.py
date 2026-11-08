"""Isolated contact (hitl_final_approach.md section 9, step 26): only authenticated, run/vehicle/task/episode-bound
contact events reach the episode latch, and none carries geometry.

The contact service and navpy's HITL adapter talk through ``contact_rig.Rig`` (no sockets), with the real
``FinalApproachDockContact`` and ``FinalApproachEpisodeLatch`` behind the adapter.
"""
from __future__ import annotations

import json
import math

import pytest

from navpy.plugins.contact import hitl_contact_wire as wire
from navpy.plugins.contact.hitl_contact_sensor import ACCEPTED, CONTACT_MAX_AGE_S, CONTACT_MAX_LEAD_S
from tests.tools.hitl.contact_rig import BRIDGE, KEY, OTHER_KEY, RUN, TASK, VEHICLE, Rig, at, sign

COORDINATE_WORDS = ("lat", "lon", "lng", "alt", "north", "east", "height", "position", "dist", "closest", "range",
                    "pose", "x_", "y_", "z_")


def _armed_rig() -> Rig:
    rig = Rig()
    rig.nav_entry()
    return rig


def _contact_body(rig: Rig, **changes):
    body = rig.last_contact()
    return {**body, **changes}


# --- the one accepted path --------------------------------------------------------------------------------------------

def test_a_pass_through_the_net_docks_the_armed_episode_once_and_carries_no_geometry():
    rig = _armed_rig()
    rig.fly_past()
    rig.fly_past(t0=rig.clock[0] + 1.0)      # same arm: no second contact

    assert rig.verdicts == [ACCEPTED] and rig.docked
    body = rig.last_contact()
    assert set(body) == wire.FIELDS[wire.KIND_CONTACT]
    assert not [name for name in body if any(word in name for word in COORDINATE_WORDS)]
    event = rig.episode.disarm()
    assert event.identity.task_id == TASK and event.identity.vehicle == VEHICLE
    assert all(math.isnan(value) for value in (event.closest_3d_m, event.closest_h_m, event.closest_v_m))
    # The service keeps its own evidence: the closest pass stays on its side.
    assert [r for r in rig.records if r["event"] == "contact"][0]["closest_3d_m"] < 0.5


def test_a_pass_outside_the_net_sends_nothing():
    rig = _armed_rig()
    rig.fly_past(east_m=0.6)
    assert rig.to_navpy == [] and not rig.docked


def test_every_wire_kind_has_no_coordinate_or_distance_field():
    names = {name for fields in wire.FIELDS.values() for name in fields}
    assert not [name for name in names if any(word in name for word in COORDINATE_WORDS)]


# --- wrong: another run, session, key, kind or identity ---------------------------------------------------------------

@pytest.mark.parametrize("change, reason", [
    ({"run_id": "run-27"}, "wrong run"),
    ({"session": "f" * 32}, "wrong session"),
    ({"vehicle": VEHICLE + 1}, "wrong identity"),
    ({"task_id": TASK + 1}, "wrong identity"),
    ({"dock_uid": 99}, "wrong identity"),
    ({"episode": 9}, "wrong identity"),
    ({"arm_id": "e" * 32}, "stale arm"),
])
def test_a_signed_contact_for_another_run_session_or_identity_is_refused(change, reason):
    rig = _armed_rig()
    rig.fly_past(east_m=0.6)
    forged = sign({"schema": wire.SCHEMA, "kind": wire.KIND_CONTACT, "run_id": RUN,
                   "session": rig.sensor._session, "vehicle": VEHICLE, "task_id": TASK, "dock_uid": 4,
                   "episode": 1, "arm_id": f"{2:032x}", "board_time_us": 100_200_000, **change})
    assert rig.sensor.receive(forged) == reason
    assert not rig.docked


@pytest.mark.parametrize("datagram, reason", [
    (lambda body: sign(body, OTHER_KEY), "bad mac"),
    (lambda body: sign({**body, "kind": wire.KIND_ARM}), "wrong kind"),
    (lambda body: sign({**body, "lat_deg": 40.0}), "malformed"),          # a coordinate grew on the wire
    (lambda body: sign({**body, "closest_3d_m": 0.1}), "malformed"),
    (lambda body: sign({**body, "board_time_us": 1.5}), "malformed"),
    (lambda body: sign({**body, "episode": True}), "malformed"),
    (lambda body: sign({k: v for k, v in body.items() if k != "episode"}), "malformed"),
    (lambda body: wire.canonical(body), "malformed"),                      # no MAC at all
    (lambda body: b"{" * 2000, "oversize"),
    (lambda body: b"\xff\xfe", "malformed"),
    (lambda body: sign(body)[:-1] + b"0}", "bad mac"),
])
def test_an_unauthenticated_or_malformed_contact_is_refused(datagram, reason):
    rig = _armed_rig()
    body = {"schema": wire.SCHEMA, "kind": wire.KIND_CONTACT, "run_id": RUN, "session": rig.sensor._session,
            "vehicle": VEHICLE, "task_id": TASK, "dock_uid": 4, "episode": 1, "arm_id": f"{2:032x}",
            "board_time_us": 100_200_000}
    assert rig.sensor.receive(datagram(body)) == reason
    assert not rig.docked
    assert rig.sensor.receive(sign(body)) == ACCEPTED      # the same body, signed, is the arm's contact


def test_encode_refuses_a_body_with_a_field_outside_its_kind():
    with pytest.raises(ValueError):
        wire.encode({"schema": wire.SCHEMA, "kind": wire.KIND_DISARM, "run_id": RUN, "session": "a" * 32,
                     "seq": 1, "arm_id": "b" * 32, "alt_m": 3.0}, KEY)


# --- stale, replayed, disarmed, after abort ---------------------------------------------------------------------------

def test_a_replayed_contact_is_refused_and_counted():
    rig = _armed_rig()
    rig.fly_past()
    assert rig.sensor.receive(rig.to_navpy[-1]) == "replayed"
    assert rig.sensor.refusals["replayed"] == 1


def test_a_previous_episodes_contact_never_docks_a_new_episode():
    rig = _armed_rig()
    rig.fly_past()
    first = rig.to_navpy[-1]
    rig.episode.disarm()
    rig.nav_entry()                      # a new episode, a new arm_id
    assert rig.sensor.receive(first) == "stale arm"
    assert not rig.docked
    rig.fly_past()
    assert rig.verdicts[-1] == ACCEPTED and rig.docked and rig.last_contact()["episode"] == 2


def test_a_contact_after_nav_exit_is_refused():
    rig = _armed_rig()
    rig.fly_past(east_m=0.6)
    body = {"schema": wire.SCHEMA, "kind": wire.KIND_CONTACT, "run_id": RUN, "session": rig.sensor._session,
            "vehicle": VEHICLE, "task_id": TASK, "dock_uid": 4, "episode": 1, "arm_id": f"{2:032x}",
            "board_time_us": 100_200_000}
    rig.episode.disarm()
    assert rig.sensor.receive(sign(body)) == "disarmed"
    assert rig.terminal is None


def test_the_service_stops_judging_at_disarm():
    rig = _armed_rig()
    rig.episode.disarm()
    rig.fly_past()
    assert rig.to_navpy == [] and rig.service.sent == 0


def test_a_contact_after_an_abort_is_not_latched():
    """NAV_FAIL latched first: the adapter's one contact reaches the latch, which keeps FAILED."""
    rig = _armed_rig()
    assert rig.latch.latch_failure("deadline")
    rig.fly_past()
    assert rig.verdicts == [ACCEPTED]
    assert not rig.docked and rig.terminal.value == "failed"
    assert rig.episode.disarm() is None


@pytest.mark.parametrize("arm_clock, now_clock, contact_s, reason", [
    (100.0, 100.3, 99.9, "before arm"),                              # older than the arm (a reboot or a replay)
    (100.0, 100.2 + CONTACT_MAX_AGE_S + 0.01, 100.2, "stale"),       # held back past the age limit
    (100.0, 100.1, 100.1 + CONTACT_MAX_LEAD_S + 0.01, "ahead of the board clock"),
    (100.0, 100.0, 10_000.0, "ahead of the board clock"),           # a pose from before a board reboot
    (None, 100.3, 100.2, "no board time"),
    (100.0, None, 100.2, "no board time"),
    (100.0, math.nan, 100.2, "no board time"),
])
def test_a_contact_outside_the_arms_board_time_window_is_refused(arm_clock, now_clock, contact_s, reason):
    rig = Rig()
    rig.clock[0] = arm_clock
    rig.nav_entry()
    rig.clock[0] = now_clock
    body = {"schema": wire.SCHEMA, "kind": wire.KIND_CONTACT, "run_id": RUN, "session": rig.sensor._session,
            "vehicle": VEHICLE, "task_id": TASK, "dock_uid": 4, "episode": 1, "arm_id": f"{2:032x}",
            "board_time_us": round(contact_s * 1e6)}
    assert rig.sensor.receive(sign(body)) == reason
    assert not rig.docked


@pytest.mark.parametrize("now_s", [100.2 + CONTACT_MAX_AGE_S, 100.2 - CONTACT_MAX_LEAD_S])
def test_a_contact_just_inside_the_age_and_lead_limits_is_accepted(now_s):
    rig = Rig()
    rig.clock[0] = 99.0
    rig.nav_entry()
    rig.clock[0] = now_s
    body = {"schema": wire.SCHEMA, "kind": wire.KIND_CONTACT, "run_id": RUN, "session": rig.sensor._session,
            "vehicle": VEHICLE, "task_id": TASK, "dock_uid": 4, "episode": 1, "arm_id": f"{2:032x}",
            "board_time_us": 100_200_000}
    assert rig.sensor.receive(sign(body)) == ACCEPTED and rig.docked


# --- the service's own refusals ---------------------------------------------------------------------------------------

def _arm_body(rig: Rig, **changes):
    body = wire.decode(rig.to_service[0], KEY, frozenset({wire.KIND_ARM}))
    return {**body, **changes}


@pytest.mark.parametrize("change, key, reason", [
    ({}, KEY, "replayed"),                                   # the same arm again: its sequence is not new
    ({"seq": 99, "run_id": "run-27"}, KEY, "wrong run"),
    ({"seq": 99, "session": "c" * 32}, KEY, "wrong session"),
    ({"seq": 99}, OTHER_KEY, "bad mac"),
])
def test_the_service_refuses_a_replayed_or_foreign_arm(change, key, reason):
    rig = _armed_rig()
    assert rig.service.from_navpy(sign(_arm_body(rig, **change), key)) == reason
    assert rig.service.refusals[reason] == 1


def test_the_service_judges_only_the_armed_vehicles_new_poses():
    rig = _armed_rig()
    assert rig.pose(100.1, at(0.0), sys_id=VEHICLE + 1) == "other vehicle"
    assert rig.pose(100.1, at(-1.0)) == "judged"
    rig.sequence -= 1
    assert rig.pose(100.12, at(0.0)) == "old pose"         # a repeated sequence of the same bridge session
    assert rig.to_navpy == []


def test_the_first_bridge_binds_the_pose_source_and_session():
    """A second sender, or the same sender with another session, never feeds the judge: no false crossing."""
    rig = _armed_rig()
    assert rig.pose(100.10, at(-1.0)) == "judged"
    assert rig.pose(100.12, at(1.0), source=("127.0.0.1", 47999)) == "foreign pose source"
    assert rig.pose(100.12, at(1.0), session="bridge-2") == "other bridge session"
    assert rig.to_navpy == [] and not rig.docked
    assert rig.service.refusals == {"foreign pose source": 1, "other bridge session": 1}
    rig.fly_past()                                  # the bound bridge still docks
    assert rig.docked


def test_a_pose_injected_across_a_bridge_restart_never_docks():
    """Old samples of one session and new ones of another, either side of the net 20 ms apart."""
    rig = _armed_rig()
    assert rig.pose(100.10, at(-1.0), session="bridge-1") == "judged"
    for index, session in enumerate(("bridge-2", "bridge-1-replay")):
        rig.sequence = 0
        assert rig.pose(100.12 + index * 0.02, at(1.0), session=session) == "other bridge session"
    assert not rig.docked and rig.to_navpy == []


def test_a_disarm_naming_another_arm_does_not_stop_judging():
    rig = _armed_rig()
    disarm = {"schema": wire.SCHEMA, "kind": wire.KIND_DISARM, "run_id": RUN, "session": rig.sensor._session,
              "seq": 50, "arm_id": "d" * 32}
    assert rig.service.from_navpy(wire.encode(disarm, KEY)) == "disarm of another arm"
    rig.fly_past()
    assert rig.docked


def test_the_pass_stays_in_the_services_private_evidence_and_never_reaches_navpy():
    rig = _armed_rig()
    rig.fly_past()
    rig.episode.disarm()
    assert [record["event"] for record in rig.records] == ["arm", "bridge", "contact", "closest", "disarm"]
    sent = b"".join(rig.to_navpy).decode("utf-8")
    assert not [word for word in COORDINATE_WORDS if word in json.dumps(json.loads(sent)) and word != "dist"]
    assert json.loads(sent).keys() == wire.FIELDS[wire.KIND_CONTACT] | {wire.MAC_FIELD}


def test_the_services_arm_evidence_names_navpys_identity_for_scoring():
    """Step 27: the series report joins the contact to navpy's terminal episode through the arm's identity, which the
    service logs on its own side (navpy's own task, dock uid and episode; never a coordinate)."""
    rig = _armed_rig()
    arm = [r for r in rig.records if r["event"] == "arm"][0]
    assert arm["task_id"] == TASK and arm["vehicle"] == VEHICLE and {"dock_uid", "episode"} <= set(arm)
    assert not [name for name in arm if any(word in name for word in COORDINATE_WORDS)]
