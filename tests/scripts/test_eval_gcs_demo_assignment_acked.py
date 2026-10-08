"""The demo assignment audit on acked (uid-format) GCS backend logs.

docs/design/swarm-task-assignment-ack.md: every request and response copy is
logged with its uid, and the owner's APPLIED of a helper's accepted response
is logged as a `Task assign ack:` line. Legacy logs keep their own rule
(test_eval_gcs_navigation_demo_review_regressions.py).
"""

from __future__ import annotations

import pytest
from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_NAV_WAYPOINT

from scripts.eval_gcs_demo_assignment_audit import derive_global_assignment_map
from scripts.eval_gcs_demo_assignment_lines import LogFormat, parse_assignment_lines
from scripts.eval_gcs_demo_models import ThreeUavIds
from scripts.eval_gcs_demo_scenario import resolve_demo_plan

OWNER = 7
# Plan tasks 1 and 3 by coordinate; the owner keeps plan task 2.
_COORDINATES = {1: "40.000300, 44.000300", 3: "40.000700, 44.000700"}


def _plan():
    rows = [
        {
            "lat": 40.0 + ordinal / 10_000,
            "lon": 44.0 + ordinal / 10_000,
            "alt": 100.0,
            "mission_sequence": ordinal + 1,
            "command": MAV_CMD_NAV_WAYPOINT,
            "nav_waypoint_ordinal": ordinal,
        }
        for ordinal in range(1, 8)
    ]
    return resolve_demo_plan(
        ThreeUavIds.from_values((OWNER, 4, 9)),
        {"sys_id": OWNER, "waypoints": rows},
    )


def _request(peer, task, seq, plan_task=None):
    return (
        f"INFO Task assign request: sender={OWNER} receiver={peer} task_id={task} "
        f"at ({_COORDINATES[plan_task or task]}) uid=10:{seq}"
    )


def _response(peer, task, seq, accepted=True):
    return (
        f"INFO Task assign response: sender={peer} receiver={OWNER} task_id={task} "
        f"accepted={accepted} uid={peer}0:{seq}"
    )


def _applied(peer, task, ref_seq, seq, owner=OWNER):
    return (
        f"INFO Task assign ack: owner={owner} helper={peer} task_id={task} "
        f"status=APPLIED ref={peer}0:{ref_seq} uid=10:{seq}"
    )


# Peer 4 is first offered task 3 and released (its late accept gets no
# APPLIED); it then gets task 1, in copies. Peer 9 rejects task 3 once, is
# offered it again and accepts.
VALID = [
    _request(4, 3, 40),
    _response(4, 3, 3),
    _request(4, 1, 50),
    _request(4, 1, 52),
    _request(9, 3, 51),
    _response(9, 3, 5, accepted=False),
    _request(9, 3, 60),
    _response(4, 1, 7),
    _response(4, 1, 9),
    _applied(4, 1, 7, 53),
    _applied(4, 1, 9, 54),
    _response(9, 3, 8),
    _applied(9, 3, 8, 61),
]


def _without(*drop):
    return [line for line in VALID if line not in drop]


def test_copies_released_rounds_and_rejects_resolve_to_the_applied_tasks():
    text = "\n".join(VALID)

    assert parse_assignment_lines(text).log_format is LogFormat.ACKED
    assert derive_global_assignment_map(text, _plan()) == {OWNER: 2, 4: 1, 9: 3}


@pytest.mark.parametrize(
    ("lines", "message"),
    [
        (_without(_applied(9, 3, 8, 61)), "APPLIED to peer 9, found 0"),
        (
            _without(_applied(9, 3, 8, 61)) + [_applied(9, 3, 99, 61)],
            "ref 90:99 names no accepted",
        ),
        (
            _without(_applied(9, 3, 8, 61)) + [_applied(9, 3, 5, 61)],
            "ref 90:5 names no accepted",
        ),
        (VALID + [_applied(4, 3, 3, 62)], "APPLIED to peer 4, found 2"),
        (
            _without(_applied(9, 3, 8, 61))
            + [_request(9, 1, 70), _response(9, 1, 12), _applied(9, 1, 12, 71)],
            "same auction task to multiple peers",
        ),
        (
            _without(_applied(9, 3, 8, 61))
            + [_response(9, 2, 13), _applied(9, 2, 13, 72)],
            "P2 of peer 9 has 0 plan coordinates",
        ),
        (VALID + [_applied(9, 3, 8, 62, owner=4)], "non-owner task assignment ack"),
        (
            VALID + [
                "INFO Task assign response: sender=9 receiver=7 task_id=3 "
                "accepted=True",
            ],
            "mixed assignment log formats",
        ),
        (VALID + [_request(9, 3, 60).replace("uid=10:60", "uid=x")], "malformed"),
    ],
    ids=[
        "no-applied", "unknown-ref", "applied-reject", "two-tasks-one-peer",
        "task-applied-to-two-peers", "applied-without-request", "non-owner-ack",
        "mixed-format", "malformed-uid",
    ],
)
def test_acked_evidence_must_be_complete_and_consistent(lines, message):
    with pytest.raises(ValueError, match=message):
        derive_global_assignment_map("\n".join(lines), _plan())


def test_legacy_lines_still_read_as_legacy():
    text = "\n".join((
        f"INFO Task assign request: sender={OWNER} receiver=4 task_id=1 "
        f"at ({_COORDINATES[1]})",
        f"INFO Task assign request: sender={OWNER} receiver=9 task_id=3 "
        f"at ({_COORDINATES[3]})",
        f"INFO Task assign response: sender=4 receiver={OWNER} task_id=1 accepted=True",
        f"INFO Task assign response: sender=9 receiver={OWNER} task_id=3 accepted=True",
    ))

    assert parse_assignment_lines(text).log_format is LogFormat.LEGACY
    assert derive_global_assignment_map(text, _plan()) == {OWNER: 2, 4: 1, 9: 3}
