"""Scheduled faults tick by tick (``coordinator_faults``, step 7), over the real book and cuts."""

from __future__ import annotations

from navpy.sim.lockstep.coordinator_faults import FaultRunner
from navpy.sim.lockstep.coordinator_journal import TickRecord
from navpy.sim.lockstep.coordinator_lifecycle import LifecycleBook
from navpy.sim.lockstep.coordinator_members import Member
from navpy.sim.lockstep.coordinator_spawn import HostSidecar, SpawnRecord
from navpy.sim.lockstep.fault_schedule import (
    FENCE_MARKER,
    LATE_HEARTBEAT,
    LAUNCH_MARKER,
    LINK_DROP,
    NAVPY_RESTART,
    PEER_RECOVERY_S,
    PRESENCE_MARKER,
    RADIO_PARTITION,
    RESTART_RECOVERY_S,
    FaultPlan,
)
from navpy.sim.lockstep.participant_hooks import RequestIds
from navpy.sim.lockstep.radio_cuts import RadioCuts

BACKEND = 4
FLEET = 3


def _us(seconds: float) -> int:
    return round(seconds * 1e6)


def _member(ordinal: int, generation: int = 1, spawn_id=(BACKEND, 40), requester: int = BACKEND) -> Member:
    spawn = SpawnRecord(spawn_id, "navpy", ordinal, generation, "cap", requester)
    return Member(spawn, child=None, link=None, devices=(), admitted_at=1, ready=True)


class _Roster:
    def __init__(self) -> None:
        self.members = [_member(ordinal) for ordinal in (1, 2, 3)]
        self.spawned: list[tuple] = []

    def current(self, ordinal: int) -> Member | None:
        return next((m for m in reversed(self.members) if m.ordinal == ordinal), None)

    def spawn(self, role: str, ordinal: int, spawn_id, requester: int) -> Member:
        new = _member(ordinal, self.current(ordinal).spawn.generation + 1, spawn_id, requester)
        self.members.append(new)
        self.spawned.append((role, ordinal, spawn_id, requester))
        return new


class _Run:
    def __init__(self, *plans: FaultPlan) -> None:
        self.roster = _Roster()
        self.book = LifecycleBook(self.roster, RequestIds(0))
        self.cuts = RadioCuts()
        self.runner = FaultRunner(plans, self.roster, self.book, self.cuts, FLEET)
        self.record: dict = {}

    def tick(self, t_s: float, *lines: tuple[int, str]) -> list:
        """The tick's fault events; its lifecycle stays in ``self.record``."""
        record = TickRecord(1, "mission", _us(t_s))
        self.book.begin_tick(record)
        self.runner.tick(_us(t_s), record, lines)
        self.record = record.data
        return record.data.get("faults", [])

    def lifecycle(self) -> list[tuple]:
        return [(e["kind"], e["ordinal"], e["generation"], e["result"], e["payload"], e["to"])
                for e in self.record["lifecycle"]]


def test_a_respawn_has_its_own_row_in_the_host_sidecar(tmp_path):
    sidecar = HostSidecar(tmp_path / "host.json")
    sidecar.note(_member(1).spawn, pid=10)
    sidecar.note(_member(1, generation=2).spawn, pid=20)
    sidecar.note(_member(1).spawn, exit=0)
    assert [(row["generation"], row["pid"], row.get("exit")) for row in sidecar.rows] == [(1, 10, 0), (2, 20, None)]


def test_with_no_plan_nothing_is_noted_whatever_the_lines_say():
    run = _Run()
    assert run.runner.grid() == []
    assert run.tick(1.0, (1, LAUNCH_MARKER), (1, FENCE_MARKER)) == []
    assert "faults" not in run.record


def test_the_grid_records_every_plan_before_the_run():
    plan = FaultPlan(LINK_DROP, "sitl1", 2.0, 3.0)
    assert _Run(plan).runner.grid() == [plan.to_json()]


def test_a_fault_waits_for_its_anchor_then_its_start():
    run = _Run(FaultPlan(LINK_DROP, "sitl1", 2.0, 3.0))
    assert run.tick(50.0) == []  # no anchor yet, whatever the time
    assert run.tick(51.0, (2, f"x {LAUNCH_MARKER} 1")) == []
    assert run.tick(52.5, (1, f"x {LAUNCH_MARKER} 1")) == []  # a second line does not move the anchor
    assert run.tick(53.0) == [["start", "link_drop:sitl1"]]


def test_a_backend_line_is_no_anchor_and_no_fence():
    run = _Run(FaultPlan(LINK_DROP, "sitl1", 0.0, 3.0))
    assert run.tick(1.0, (BACKEND, LAUNCH_MARKER), (BACKEND, FENCE_MARKER)) == []
    assert run.tick(2.0, (1, LAUNCH_MARKER)) == [["start", "link_drop:sitl1"]]  # not skipped


def test_a_partition_counts_from_the_first_presence_line():
    run = _Run(FaultPlan(RADIO_PARTITION, "uav2", 1.0, 3.0))
    assert run.tick(2.0, (1, LAUNCH_MARKER)) == [] and run.tick(9.0) == []
    assert run.tick(10.0, (3, PRESENCE_MARKER)) == []
    assert run.tick(11.0) == [["start", "radio_partition:uav2"]]


def test_a_link_drop_cuts_its_endpoint_both_ways_until_its_duration():
    run = _Run(FaultPlan(LINK_DROP, "sitl2", 0.0, 3.0))
    assert run.tick(1.0, (1, LAUNCH_MARKER)) == [["start", "link_drop:sitl2"]]
    fault = "link_drop:sitl2"
    assert run.cuts.source("sitl2") == fault
    assert {run.cuts.path(other, "sitl2") for other in ("sitl1", "sitl3", "backend")} == {fault}
    assert run.cuts.source("sitl1") is None and run.cuts.path("sitl1", "sitl3") is None
    assert run.tick(3.99) == []  # one tick short of the duration
    assert run.tick(4.0) == [["heal", fault, _us(4.0 + PEER_RECOVERY_S)]]
    assert run.cuts.source("sitl2") is None and run.cuts.path("backend", "sitl2") is None


def test_a_backend_link_drop_cuts_the_ground_station_both_ways():
    run = _Run(FaultPlan(LINK_DROP, "backend", 0.0, 3.0))
    run.tick(1.0, (1, LAUNCH_MARKER))
    assert run.cuts.source("backend") == "link_drop:backend"
    assert {run.cuts.path(f"sitl{i}", "backend") for i in (1, 2, 3)} == {"link_drop:backend"}
    assert run.cuts.path("sitl1", "sitl2") is None


def test_a_partition_cuts_the_uav_from_its_peers_only():
    run = _Run(FaultPlan(RADIO_PARTITION, "uav2", 0.0, 3.0))
    run.tick(1.0, (1, PRESENCE_MARKER))
    fault = "radio_partition:uav2"
    assert {run.cuts.path(a, b) for a, b in (("sitl2", "sitl1"), ("sitl2", "sitl3"),
                                             ("sitl1", "sitl2"), ("sitl3", "sitl2"))} == {fault}
    # The ground station keeps both directions, and the peers keep each other.
    assert {run.cuts.path(a, b) for a, b in (("sitl2", "backend"), ("backend", "sitl2"),
                                             ("sitl1", "sitl3"), ("sitl3", "sitl1"))} == {None}
    assert run.cuts.source("sitl2") is None
    assert run.tick(4.0) == [["heal", fault, _us(4.0 + PEER_RECOVERY_S)]]
    assert run.cuts.path("sitl2", "sitl1") is None


def test_a_late_heartbeat_freezes_its_navpy_then_thaws_it():
    run = _Run(FaultPlan(LATE_HEARTBEAT, "uav3", 0.0, 8.0))
    fault = "late_heartbeat:uav3"
    assert run.tick(1.0, (1, LAUNCH_MARKER)) == [["start", fault]]
    assert run.lifecycle() == [("freeze", 3, 1, None, {"frozen": True, "fault": fault}, 3)]
    assert [e.kind for e in run.roster.current(3).inputs.lifecycle] == ["freeze"]
    assert run.tick(8.9) == [] and run.lifecycle() == []
    assert run.tick(9.0) == [["heal", fault, _us(9.0 + PEER_RECOVERY_S)]]
    assert run.lifecycle() == [("freeze", 3, 1, None, {"frozen": False, "fault": fault}, 3)]


def test_a_freeze_of_a_navpy_already_gone_commits_nothing():
    run = _Run(FaultPlan(LATE_HEARTBEAT, "uav3", 0.0, 2.0))
    run.tick(1.0, (1, LAUNCH_MARKER))
    run.roster.current(3).retired_at = 9
    assert run.tick(3.0) == [["heal", "late_heartbeat:uav3", _us(3.0 + PEER_RECOVERY_S)]]
    assert run.lifecycle() == []


def test_a_restart_stops_the_navpy_then_respawns_it_when_it_retired():
    run = _Run(FaultPlan(NAVPY_RESTART, "uav2", 0.0, 0.0))
    fault = "navpy_restart:uav2"
    assert run.tick(1.0, (1, LAUNCH_MARKER)) == [["start", fault]]
    assert [entry[:2] + entry[4:] for entry in run.lifecycle()] == [("stop", 2, {"fault": fault}, 2)]
    old = run.roster.current(2)
    assert run.tick(1.02) == [] and run.tick(30.0) == [] and run.roster.spawned == []  # still stopping
    run.book.retire(old, 99, flushed=True)
    assert run.book.faults == []
    assert run.tick(30.02) == [["heal", fault, _us(30.02 + RESTART_RECOVERY_S)]]
    assert run.roster.spawned == [("navpy", 2, old.spawn.spawn_id, old.spawn.requester)]
    assert run.lifecycle() == [("spawn", 2, 2, "ok", {"fault": fault}, BACKEND)]
    assert run.book.stopped(2, 1)  # the scheduled stop counts for the end-of-run check


def test_a_due_fault_waits_for_the_last_ones_deadline():
    run = _Run(FaultPlan(LINK_DROP, "sitl1", 0.0, 1.0), FaultPlan(RADIO_PARTITION, "uav2", 0.5, 1.0))
    assert run.tick(1.0, (1, LAUNCH_MARKER), (1, PRESENCE_MARKER)) == [["start", "link_drop:sitl1"]]
    assert run.tick(1.6) == []  # the partition is due, the link drop still active
    deadline = 2.0 + PEER_RECOVERY_S
    assert run.tick(2.0) == [["heal", "link_drop:sitl1", _us(deadline)]]
    assert run.tick(deadline - 0.02) == []  # still inside its recovery
    assert run.tick(deadline) == [["start", "radio_partition:uav2"]]


def test_the_plan_order_decides_a_tie():
    run = _Run(FaultPlan(LINK_DROP, "sitl1", 0.0, 1.0), FaultPlan(LATE_HEARTBEAT, "uav2", 0.0, 7.0))
    assert run.tick(1.0, (1, LAUNCH_MARKER)) == [["start", "link_drop:sitl1"]]


def test_the_fence_ends_what_is_active_and_skips_what_is_left():
    run = _Run(FaultPlan(LINK_DROP, "sitl1", 0.0, 9.0), FaultPlan(LATE_HEARTBEAT, "uav2", 5.0, 7.0))
    run.tick(1.0, (1, LAUNCH_MARKER))
    assert run.tick(2.0, (3, f"x {FENCE_MARKER}1")) == [
        ["heal", "link_drop:sitl1", _us(2.0 + PEER_RECOVERY_S), "forced"],
        ["skip", "late_heartbeat:uav2", "fence"]]
    assert run.cuts.source("sitl1") is None
    assert run.tick(30.0) == []  # nothing starts after the fence


def test_the_fence_thaws_a_freeze_in_its_tick():
    run = _Run(FaultPlan(LATE_HEARTBEAT, "uav2", 0.0, 9.0))
    run.tick(1.0, (1, LAUNCH_MARKER))
    fault = "late_heartbeat:uav2"
    assert run.tick(2.0, (1, FENCE_MARKER)) == [["heal", fault, _us(2.0 + PEER_RECOVERY_S), "forced"]]
    assert run.lifecycle() == [("freeze", 2, 1, None, {"frozen": False, "fault": fault}, 2)]


def test_a_restart_has_no_early_end():
    run = _Run(FaultPlan(NAVPY_RESTART, "uav2", 0.0, 0.0))
    run.tick(1.0, (1, LAUNCH_MARKER))
    assert run.tick(2.0, (1, FENCE_MARKER)) == []  # its stop takes the time it takes
    run.book.retire(run.roster.current(2), 99, flushed=True)
    assert run.tick(2.02) == [["heal", "navpy_restart:uav2", _us(2.02 + RESTART_RECOVERY_S)]]


def test_a_restart_due_once_every_uav_has_checked_in_is_skipped_and_the_others_stay():
    run = _Run(FaultPlan(NAVPY_RESTART, "uav1", 5.0, 0.0), FaultPlan(LINK_DROP, "sitl1", 5.0, 1.0))
    run.tick(1.0, (1, LAUNCH_MARKER), (1, PRESENCE_MARKER))
    assert run.tick(2.0, (1, PRESENCE_MARKER), (2, PRESENCE_MARKER)) == []  # only two so far, one of them twice
    assert run.tick(3.0, (3, PRESENCE_MARKER)) == [["skip", "navpy_restart:uav1", "presence"]]
    assert run.tick(6.0) == [["start", "link_drop:sitl1"]]


def test_the_backends_line_does_not_count_as_a_check_in():
    run = _Run(FaultPlan(NAVPY_RESTART, "uav1", 5.0, 0.0))
    run.tick(1.0, (1, LAUNCH_MARKER), (1, PRESENCE_MARKER), (2, PRESENCE_MARKER))
    assert run.tick(2.0, (BACKEND, PRESENCE_MARKER)) == []
    assert run.tick(6.0) == [["start", "navpy_restart:uav1"]]


def test_a_restart_due_in_the_last_check_ins_own_tick_is_skipped():
    run = _Run(FaultPlan(NAVPY_RESTART, "uav1", 1.0, 0.0))
    run.tick(1.0, (1, LAUNCH_MARKER), (1, PRESENCE_MARKER), (2, PRESENCE_MARKER))
    assert run.tick(2.0, (3, PRESENCE_MARKER)) == [["skip", "navpy_restart:uav1", "presence"]]


def test_the_last_check_in_leaves_a_running_restart_alone():
    run = _Run(FaultPlan(NAVPY_RESTART, "uav1", 0.0, 0.0))
    assert run.tick(1.0, (1, LAUNCH_MARKER)) == [["start", "navpy_restart:uav1"]]
    assert run.tick(2.0, (1, PRESENCE_MARKER), (2, PRESENCE_MARKER), (3, PRESENCE_MARKER)) == []
    run.book.retire(run.roster.current(1), 99, flushed=True)
    assert run.tick(2.02) == [["heal", "navpy_restart:uav1", _us(2.02 + RESTART_RECOVERY_S)]]


def test_a_fence_in_a_fault_free_stretch_skips_every_plan_at_once():
    run = _Run(FaultPlan(LINK_DROP, "sitl1", 50.0, 1.0), FaultPlan(NAVPY_RESTART, "uav1", 60.0, 0.0))
    run.tick(1.0, (1, LAUNCH_MARKER))
    assert run.tick(2.0, (1, FENCE_MARKER)) == [["skip", "link_drop:sitl1", "fence"],
                                                ["skip", "navpy_restart:uav1", "fence"]]
