"""e6: the delivery role gates over synthetic logs.

The pass case, then each failure the plan lists: a missing offer, two
winners, an approach before approval, a home-dock offer, the winner still
searching, and docking at the wrong dock id; plus the review's extra cases.
"""

from __future__ import annotations

import json
import pytest

from navpy.sim.world.fleet_dock_world import parse_fleet_docks
from scripts import eval_delivery_fixture as fixture
from scripts.eval_delivery_gates import (
    DeliveryEvidence,
    delivery_errors,
    delivery_run_gate_errors,
)
from scripts.eval_delivery_roles import resolve_roles
from scripts.eval_gcs_demo_assignment_lines import parse_assignment_lines
from scripts.eval_gcs_demo_evidence import parse_final_approach_episodes
from scripts.eval_gcs_demo_models import (
    DockArmed,
    DockedEvidence,
    FinalApproachCommand,
    FinalApproachEpisode,
)

FIXTURE = json.loads(fixture.FIXTURE_PATH.read_text(encoding="utf-8"))
DOCKS = parse_fleet_docks(FIXTURE["docks"])
NAMES = [dock.name for dock in DOCKS]
D1, D2 = NAMES.index("d1"), NAMES.index("d2")
HOME_WEST = NAMES.index("home-west")
#: The earliest the approval can be posted: request seen + operator delay.
APPROVED_AT = 1000.0
T = "2026-10-17 10:00:00,000 INFO "


def _at(dock: int) -> str:
    return f"{DOCKS[dock].lat:.7f}, {DOCKS[dock].lon:.7f}"


def _review(dock: int, snap: float = 0.3) -> list[str]:
    return [
        f"GimbalNavigation(siyi_zr10): start_tracking obj_id={dock} (prev=None, mode=LOCK)",
        f"POI: P1 (tracking obj_id={dock})",
        "CONFIRMING: P1",
        "POI 1 confirmed by ground station.",
        "INIT: NAV MODE",
        f"t_l (truth): {_at(dock)}, 0.0",
        f"SNAP(VISION-NAV-PN): 3d={snap}(h=0.2; v=0.1)",
    ]


def _text(lines: list[str]) -> str:
    return "\n".join(T + line for line in lines) + "\n"


def _offerer() -> list[str]:
    review = _review(D1)
    offer = f"Task offered: task_id=2 obj_id={D2} at ({_at(D2)}) sent=True"
    return [review[0], "Self-detect orbit: poi=x orbit_r=500m", review[1], offer, *review[2:]]


def _winner() -> list[str]:
    lat, lon = (part.strip() for part in _at(D2).split(","))
    return [
        "Task 2 assigned by owner 1",
        f"GUIDED_LOITER cmd=DO_REPOSITION center={lat},{lon},200.0 radius=500m",
        "Peer navigation started (orbit, offset=0m, orbit_r=500m).",
        *_review(D2),
    ]


def _home() -> list[str]:
    return [f"Default delivery hub navigation: {_at(HOME_WEST)}, 0.0 (orbit)", *_review(HOME_WEST)]


BACKEND = "\n".join([
    f"Task assign request: sender=1 receiver=2 task_id=2 at ({_at(D2)}) uid=1:5",
    "Task assign response: sender=2 receiver=1 task_id=2 accepted=True uid=2:7",
    "Task assign ack: owner=1 helper=2 task_id=2 status=APPLIED ref=2:7 uid=1:6",
])


def _episode(
    sys_id: int, dock: int, wall_s: float = APPROVED_AT + 5.0, docked_at: int | None = -1,
) -> FinalApproachEpisode:
    command = FinalApproachCommand(
        1.0, "frame", 1, 1, dock, 10.0, None, 0.0, 0.0, 0.0, 0.5, True, False,
    )
    armed = DockArmed(sys_id, 1, dock, 1, 9.0, wall_s)
    at = dock if docked_at == -1 else docked_at
    docked = None if at is None else DockedEvidence(2.0, sys_id, 1, at, 1, 10.5, 0.05, 0.04, 0.03)
    return FinalApproachEpisode((command,), (), docked is not None, armed, docked)


def _evidence(texts=None, episodes=None, backend=BACKEND) -> DeliveryEvidence:
    lines = texts or {1: _offerer(), 2: _winner(), 3: _home()}
    return DeliveryEvidence(
        (1, 2, 3),
        FIXTURE["expected"],
        DOCKS,
        {sys_id: _text(items) for sys_id, items in lines.items()},
        episodes or {1: [_episode(1, D1)], 2: [_episode(2, D2)], 3: [_episode(3, HOME_WEST)]},
        parse_assignment_lines(backend),
    )


APPROVALS = [
    {"sys_id": sys_id, "task_id": 1, "request_observed_at_unix_s": APPROVED_AT - 15.0,
     "requested_delay_s": 15.0, "approved_at_unix_s": APPROVED_AT + 0.6} for sys_id in (1, 2, 3)
]


def _errors(evidence: DeliveryEvidence) -> list[str]:
    applied = {(ack.owner, ack.helper, ack.task) for ack in evidence.lines.acks}
    roles = resolve_roles(evidence.sys_ids, evidence.expected, applied)
    return delivery_errors(evidence, roles, APPROVALS)


def _fails(evidence: DeliveryEvidence, text: str) -> None:
    errors = _errors(evidence)
    assert any(text in error for error in errors), errors


def test_the_delivery_run_passes():
    assert _errors(_evidence()) == []


def test_a_missing_offer_fails():
    offerer = [line for line in _offerer() if "Task offered" not in line]
    _fails(_evidence({1: offerer, 2: _winner(), 3: _home()}), "exactly one logical offer")


def test_two_winners_fail():
    backend = BACKEND + "\n" + "\n".join([
        f"Task assign request: sender=1 receiver=3 task_id=3 at ({_at(D2)}) uid=1:8",
        "Task assign response: sender=3 receiver=1 task_id=3 accepted=True uid=3:2",
        "Task assign ack: owner=1 helper=3 task_id=3 status=APPLIED ref=3:2 uid=1:9",
    ])
    _fails(_evidence(backend=backend), "exactly one logical APPLIED")


def test_an_approach_before_approval_fails():
    episodes = {1: [_episode(1, D1, wall_s=APPROVED_AT - 1.0)], 2: [_episode(2, D2)],
                3: [_episode(3, HOME_WEST)]}
    _fails(_evidence(episodes=episodes), "UAV 1 entered NAV for d1 before its approval")


def test_arming_while_the_approval_post_is_in_flight_passes():
    """The POST returns after the backend forwards the approval (e10 r1: 0.5 s)."""
    episodes = {1: [_episode(1, D1, wall_s=APPROVED_AT + 0.05)], 2: [_episode(2, D2)],
                3: [_episode(3, HOME_WEST)]}
    assert _errors(_evidence(episodes=episodes)) == []


def test_nav_before_navpy_logs_the_approval_fails():
    """Inside the time bound, but NavPy entered NAV before its confirmation."""
    winner = [line for line in _winner() if "INIT: NAV" not in line]
    at = next(i for i, line in enumerate(winner) if "confirmed by ground station" in line)
    winner.insert(at, "INIT: NAV MODE")
    _fails(_evidence({1: _offerer(), 2: winner, 3: _home()}),
           "UAV 2 entered NAV before it logged the approval of task 1")


def test_a_home_dock_offer_fails_by_id_and_by_position():
    by_id = f"Task offered: task_id=2 obj_id={HOME_WEST} at ({_at(D2)}) sent=True"
    _fails(_evidence({1: [*_offerer(), by_id], 2: _winner(), 3: _home()}), "offered a home dock")
    near = f"Task offered: task_id=2 obj_id={D2} at ({_at(HOME_WEST)}) sent=True"
    _fails(_evidence({1: [*_offerer(), near], 2: _winner(), 3: _home()}), "offered a home dock")


def test_the_winner_still_searching_fails():
    resumed = [*_winner()[:3], "REJECTED: resuming AUTO mission", *_winner()[3:]]
    _fails(_evidence({1: _offerer(), 2: resumed, 3: _home()}), "resumed its AUTO search")
    no_reposition = [line for line in _winner() if "DO_REPOSITION" not in line]
    _fails(_evidence({1: _offerer(), 2: no_reposition, 3: _home()}), "no DO_REPOSITION to d2")


def test_docking_at_the_wrong_dock_id_fails():
    episodes = {1: [_episode(1, D1)], 2: [_episode(2, D1)], 3: [_episode(3, HOME_WEST)]}
    _fails(_evidence(episodes=episodes), "UAV 2 approached dock ids [0], not d2")


def test_no_docked_row_or_one_at_another_dock_is_not_docked():
    for docked_at in (None, D1):
        episodes = {1: [_episode(1, D1)], 2: [_episode(2, D2, docked_at=docked_at)],
                    3: [_episode(3, HOME_WEST)]}
        _fails(_evidence(episodes=episodes), "UAV 2 has no DOCKED row at d2")


def test_navigation_before_the_assignment_fails():
    winner = _winner()
    reordered = [winner[2], winner[0], winner[1], *winner[3:]]
    _fails(_evidence({1: _offerer(), 2: reordered, 3: _home()}), "winner order is not")


def test_an_offer_of_another_task_fails():
    offerer = [line.replace("task_id=2", "task_id=99") for line in _offerer()]
    _fails(_evidence({1: offerer, 2: _winner(), 3: _home()}), "offered task 99 is not the APPLIED task 2")


def test_an_orbit_before_the_lock_fails():
    offerer = _offerer()
    reordered = [offerer[1], offerer[0], *offerer[2:]]
    _fails(_evidence({1: reordered, 2: _winner(), 3: _home()}), "offerer order is not lock -> orbit")


def test_an_orbit_after_the_selection_fails():
    offerer = _offerer()
    reordered = [offerer[0], offerer[2], offerer[1], *offerer[3:]]
    _fails(_evidence({1: reordered, 2: _winner(), 3: _home()}), "offerer order is not lock -> orbit")


def test_the_parser_keeps_each_episode_s_own_armed_and_docked_rows(tmp_path):
    """Three episodes at docks 11, 12, 13; the last has no DOCKED row and
    must not inherit an earlier episode's row."""
    from tests.scripts.test_eval_gcs_navigation_demo import _write_episode

    _write_episode(tmp_path, 1, [5.0, 3.0, 1.0], snap_m=0.4)
    debug = tmp_path / "uav_1_navigation_debug.csv"
    header, *body = debug.read_text(encoding="utf-8").splitlines(True)

    def moved(episode: int) -> list[str]:
        dock = 10 + episode
        return [
            line.replace("dock=11", f"dock={dock}").replace("obj=11", f"obj={dock}")
            .replace("episode=1", f"episode={episode}")
            .replace("aircraft_ts=1000.5", f"aircraft_ts={1000.4 + episode / 10:g}")
            for line in body if episode < 3 or "EVENT:DOCKED" not in line
        ]

    debug.write_text(header + "".join(moved(1) + moved(2) + moved(3)), encoding="utf-8")
    episodes = parse_final_approach_episodes(debug, vehicle_id=1)
    assert [(item.armed.dock, item.armed.episode) for item in episodes] == [(11, 1), (12, 2), (13, 3)]
    assert [(item.docked_row.dock, item.docked_row.episode, item.docked_row.aircraft_ts)
            for item in episodes[:2]] == [(11, 1, 1000.5), (12, 2, 1000.6)]
    assert episodes[2].docked_row is None and episodes[2].docked is False


def test_a_broken_bid_correlation_fails():
    _fails(_evidence(backend=BACKEND.replace("ref=2:7", "ref=2:9")), "names no accepted bid")
    no_bid = BACKEND.replace("accepted=True", "accepted=False")
    _fails(_evidence(backend=no_bid), "sent no accepted bid")


def test_an_unknown_dock_id_fails():
    offerer = [*_offerer(), "start_tracking obj_id=9 (prev=None)"]
    _fails(_evidence({1: offerer, 2: _winner(), 3: _home()}), "outside the 5 world docks")


def test_a_home_uav_at_another_hub_fails():
    home = [line.replace(_at(HOME_WEST), _at(D1), 1) if "hub" in line else line for line in _home()]
    _fails(_evidence({1: _offerer(), 2: _winner(), 3: home}), "hub navigation is not at home-west")


def test_an_offerer_that_flew_a_peer_task_fails():
    offerer = [*_offerer(), "Peer navigation started (orbit)."]
    _fails(_evidence({1: offerer, 2: _winner(), 3: _home()}), "offerer flew a peer task")


def test_an_assignment_owned_by_another_uav_fails():
    backend = BACKEND.replace("owner=1 helper=2", "owner=3 helper=2")
    _fails(_evidence(backend=backend), "APPLIED owner 3 is not the offerer 1")


def test_the_run_gate_refuses_another_dock_world(tmp_path):
    scenario = fixture.load_eval_scenario(fixture.FIXTURE_PATH)
    docks = json.loads(json.dumps(FIXTURE["docks"]))
    docks["docks"][0]["lat"] += 0.001
    (tmp_path / fixture.DOCK_FILE_NAME).write_text(json.dumps(docks), encoding="utf-8")
    errors = delivery_run_gate_errors(tmp_path, None, [], scenario)
    assert errors == [f"delivery: invalid evidence: {fixture.DOCK_FILE_NAME} is not the fixture's dock world"]


def test_the_run_gate_needs_the_scenario(tmp_path):
    assert delivery_run_gate_errors(tmp_path, None, []) == [
        "delivery: the run gates need the delivery scenario"
    ]


def test_the_analysis_runs_the_delivery_gates():
    from scripts import eval_gcs_demo_analysis as analysis
    from scripts.eval_gcs_demo_gate_profiles import DELIVERY_PROFILE, WALL

    assert analysis._RUN_GATES[(DELIVERY_PROFILE, WALL)] is delivery_run_gate_errors
    assert DELIVERY_PROFILE not in analysis._PLAN_POI_BINDING
