"""The fleet flies the certified single-case lifecycle, once per aircraft.

The first truth-scoring sweep died on exactly the gaps pinned here: the fleet
pushed a hand-picked three-parameter subset while the single-case harness
pushed `sim_parameters`, so the cloned eeprom's geofence stayed live and RTL'd
the climb -- and its verdict path could not read truth at all. The fleet must
not re-implement the lifecycle; it must call the same certified pieces per
aircraft, and these tests fail if it drifts back to a private subset.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from pymavlink import mavutil

from scripts import eval_direct_pixel_pn as one
from scripts import eval_direct_pixel_pn_fleet as fleet
from scripts import eval_fleet_collect as collect
from scripts import eval_fleet_score as score
from scripts import eval_fleet_setup as setup
from scripts.eval_direct_pixel_verdict import (
    SCORING_POLICY_SITL_TRUTH,
    SCORING_POLICY_VEHICLE_ESTIMATE,
)
from scripts.eval_navigation_truth import TRUTH_SCORE_RATE_HZ
from scripts.eval_sim_parameters import sim_parameters


def _args(policy: str) -> argparse.Namespace:
    return argparse.Namespace(
        scoring_policy=policy,
        target_wp=2,
        scoring_start_wp=2,
        target_alt=60.0,
        wind_speed=0.0,
        wind_dir=0.0,
        sitl_param=None,
        max_distance=1.0,
        goal_distance=0.1,
    )


def _wire_setup(monkeypatch, events: list, *, truth_ack: bool = True) -> None:
    """Replace the per-aircraft MAVLink helpers with recorders.

    The orchestration under test is WHAT is pushed and requested per sysid
    and in what order -- not pymavlink itself, which has its own tests.
    `many._select` stays real, so `master.target_system` at call time is the
    aircraft each recorded event actually addressed.
    """
    monkeypatch.setattr(
        setup, "request_coordinate_score_stream", lambda master: True
    )

    def _interval(master, message_id, rate_hz):
        events.append(("truth_request", master.target_system, (message_id, rate_hz)))
        return truth_ack

    monkeypatch.setattr(setup, "request_message_interval_stream", _interval)
    monkeypatch.setattr(
        setup, "download_mission", lambda master: f"mission-{master.target_system}"
    )
    monkeypatch.setattr(
        setup,
        "resolve_home_abs_alt_m",
        lambda master, timeout_s: 500.0 + master.target_system,
    )
    monkeypatch.setattr(
        setup,
        "resolve_target_expectation",
        lambda mission, *, target_wp, target_rel_alt_m, home_abs_alt_m: (
            SimpleNamespace(location=("target", mission), mission_seq=4)
        ),
    )

    def _set_param(master, name, value):
        events.append(("param", master.target_system, (name, value)))
        return True

    monkeypatch.setattr(setup, "set_param", _set_param)
    monkeypatch.setattr(
        setup,
        "require_nav_solution",
        lambda master, sys_id: events.append(("nav_gate", sys_id, ())),
    )
    monkeypatch.setattr(
        setup,
        "terminal_speed_plan",
        lambda args, speedup, mission, home_alt: ("plan", mission),
    )


def _master() -> SimpleNamespace:
    return SimpleNamespace(target_system=0, target_component=0)


class _Stage:
    def __init__(self, events: list) -> None:
        self._events = events

    def pre_flight(self, master, *, sysid, target, case_dir) -> None:
        self._events.append(("pre_flight", sysid, (target, case_dir)))


def test_each_aircraft_gets_the_full_certified_tuple_with_its_cells_wind(
    monkeypatch,
) -> None:
    """Parity with the single case, per cell -- not a hand-picked subset."""
    events: list = []
    _wire_setup(monkeypatch, events)
    args = _args(SCORING_POLICY_VEHICLE_ESTIMATE)

    setup.prepare_aircraft(
        _master(), [121, 122], [(8.0, 90.0), (5.0, 270.0)],
        args=args, speedup=1.0,
    )

    pushed: dict[int, list] = {}
    for kind, sys_id, payload in events:
        if kind == "param":
            pushed.setdefault(sys_id, []).append(payload)
    assert pushed[121] == list(
        sim_parameters(setup.cell_arguments(args, 8.0, 90.0))
    )
    assert pushed[122] == list(
        sim_parameters(setup.cell_arguments(args, 5.0, 270.0))
    )
    # The exact omissions that killed the first sweep, named.
    names = {name for name, value in pushed[121]}
    assert {"FENCE_ENABLE", "FENCE_AUTOENABLE", "SIM_RATE_HZ"} <= names
    assert ("SIM_WIND_SPD", 8.0) in pushed[121]
    assert ("SIM_WIND_DIR", 270.0) in pushed[122]


def test_a_cell_never_leaks_into_the_shared_namespace(monkeypatch) -> None:
    events: list = []
    _wire_setup(monkeypatch, events)
    args = _args(SCORING_POLICY_VEHICLE_ESTIMATE)

    setup.prepare_aircraft(
        _master(), [121], [(8.0, 90.0)], args=args, speedup=1.0
    )

    assert (args.wind_speed, args.wind_dir) == (0.0, 0.0)


def test_truth_policy_requests_the_stream_per_aircraft_and_records_the_ack(
    monkeypatch,
) -> None:
    events: list = []
    _wire_setup(monkeypatch, events)

    plan = setup.prepare_aircraft(
        _master(), [121, 122], [(0.0, 0.0), (0.0, 0.0)],
        args=_args(SCORING_POLICY_SITL_TRUTH), speedup=1.0,
    )

    requests = [row for row in events if row[0] == "truth_request"]
    assert [sys_id for _, sys_id, _ in requests] == [121, 122]
    assert all(
        payload == (mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE, TRUTH_SCORE_RATE_HZ)
        for _, _, payload in requests
    )
    assert plan.truth_stream_accepted == {121: True, 122: True}
    assert plan.home_alts == {121: 621.0, 122: 622.0}


def test_estimate_policy_neither_requests_truth_nor_claims_an_ack(
    monkeypatch,
) -> None:
    events: list = []
    _wire_setup(monkeypatch, events)

    plan = setup.prepare_aircraft(
        _master(), [121], [(0.0, 0.0)],
        args=_args(SCORING_POLICY_VEHICLE_ESTIMATE), speedup=1.0,
    )

    assert not [row for row in events if row[0] == "truth_request"]
    assert plan.truth_stream_accepted == {121: None}


def test_the_cross_check_binds_after_that_aircrafts_parameters(
    monkeypatch, tmp_path: Path
) -> None:
    """Same lifecycle point as the single case: params, pre_flight, child."""
    events: list = []
    _wire_setup(monkeypatch, events)
    stages = {121: _Stage(events), 122: _Stage(events)}
    directories = {121: tmp_path / "uav-121", 122: tmp_path / "uav-122"}

    setup.prepare_aircraft(
        _master(), [121, 122], [(0.0, 0.0), (0.0, 0.0)],
        args=_args(SCORING_POLICY_SITL_TRUTH), speedup=1.0,
        case_dirs=directories, sim_cpa=stages,
    )

    order = [(kind, sys_id) for kind, sys_id, _ in events]
    for sys_id in (121, 122):
        assert order.index(("pre_flight", sys_id)) > max(
            index for index, row in enumerate(order) if row == ("param", sys_id)
        )
    assert order.index(("pre_flight", 121)) < order.index(("param", 122))
    stage_calls = {
        sys_id: payload for kind, sys_id, payload in events if kind == "pre_flight"
    }
    assert stage_calls[121] == (("target", "mission-121"), directories[121])


# --- the routing drain -------------------------------------------------------


class _Stream:
    """Scripted evaluator link; a None item ends one drain pass."""

    def __init__(self, items: list) -> None:
        self.items = list(items)

    def recv_match(self, type=None, blocking=False):
        # pymavlink wraps any non-list/non-set filter as ONE element, so a
        # tuple filter matches nothing and silently discards every message.
        assert isinstance(type, list), "drain filter must be a list"
        return self.items.pop(0) if self.items else None


def _position(sys_id: int) -> SimpleNamespace:
    return SimpleNamespace(
        get_srcSystem=lambda sys_id=sys_id: sys_id,
        get_type=lambda: "GLOBAL_POSITION_INT",
    )


def _sim_state(sys_id: int) -> SimpleNamespace:
    return SimpleNamespace(
        get_srcSystem=lambda sys_id=sys_id: sys_id,
        get_type=lambda: "SIM_STATE",
    )


class _Recorder:
    def __init__(self) -> None:
        self.samples: list = []

    def add(self, sample) -> None:
        self.samples.append(sample)


class _Truth:
    def __init__(self, closure_after: int) -> None:
        self.scoring_active_flags: list[bool] = []
        self.finalize_calls = 0
        self._closure_after = closure_after

    def add_message(self, message, wall_time_s, *, scoring_active: bool) -> None:
        self.scoring_active_flags.append(scoring_active)

    def closure_ready(self) -> bool:
        return len(self.scoring_active_flags) >= self._closure_after

    def finalize(self) -> None:
        self.finalize_calls += 1


def _fleet_dirs(tmp_path: Path, sys_ids, *, scoring_active, finished) -> dict[int, Path]:
    directories = {}
    for sys_id in sys_ids:
        directory = tmp_path / f"uav-{sys_id}"
        directory.mkdir()
        if sys_id in scoring_active:
            (directory / "engaged.marker").touch()
        if sys_id in finished:
            (directory / "result.json").write_text(
                f'{{"passed": true, "sysid": {sys_id}}}', encoding="utf-8"
            )
        directories[sys_id] = directory
    return directories


def _child() -> SimpleNamespace:
    return SimpleNamespace(poll=lambda: None, returncode=None)


def _passthrough_samples(monkeypatch) -> None:
    monkeypatch.setattr(
        collect,
        "position_sample_from_message",
        lambda message, wall_time_s: ("sample", message.get_srcSystem()),
    )


def test_the_drain_routes_by_srcsystem_not_by_a_single_sysid_filter(
    monkeypatch, tmp_path: Path
) -> None:
    """One shared consumptive stream must feed EVERY aircraft's consumers."""
    _passthrough_samples(monkeypatch)
    sys_ids = [121, 122]
    directories = _fleet_dirs(
        tmp_path, sys_ids, scoring_active={121}, finished={121, 122}
    )
    scorers = {sys_id: _Recorder() for sys_id in sys_ids}
    tracks = {sys_id: _Recorder() for sys_id in sys_ids}
    truths = {sys_id: _Truth(closure_after=1) for sys_id in sys_ids}
    stream = _Stream([
        _position(121),   # scored -> scored
        _position(122),   # not scored -> dropped
        _sim_state(121),  # truth, filed scored
        _sim_state(122),  # truth, filed unengaged
        _sim_state(999),  # not ours -> dropped
    ])

    results, failures = collect.collect_fleet(
        stream, {sys_id: _child() for sys_id in sys_ids}, directories,
        scorers, tracks, truths, timeout_s=5.0,
    )

    assert set(results) == {121, 122} and failures == {}
    assert scorers[121].samples == [("sample", 121)]
    assert tracks[121].samples and scorers[122].samples == []
    assert tracks[122].samples == []
    assert truths[121].scoring_active_flags == [True]
    assert truths[122].scoring_active_flags == [False]
    assert truths[121].finalize_calls == 1
    assert truths[122].finalize_calls == 1


def test_the_scorer_freezes_at_result_while_truth_drains_to_closure(
    monkeypatch, tmp_path: Path
) -> None:
    """The episode ends at result.json; the truth tail is still in flight."""
    _passthrough_samples(monkeypatch)
    directories = _fleet_dirs(tmp_path, [121], scoring_active={121}, finished={121})
    scorer, track = _Recorder(), _Recorder()
    truth = _Truth(closure_after=2)
    stream = _Stream([
        None,              # pass 1: result seen, nothing drained yet
        _position(121),    # after result -> frozen out of scorer and track
        _sim_state(121),   # truth still open
        None,
        _sim_state(121),   # second post-CPA sample completes closure
    ])

    collect.collect_fleet(
        stream, {121: _child()}, directories,
        {121: scorer}, {121: track}, {121: truth}, timeout_s=5.0,
    )

    assert scorer.samples == [] and track.samples == []
    assert truth.scoring_active_flags == [True, True]
    assert truth.finalize_calls == 1


def test_closure_shuts_the_truth_door_against_a_later_reapproach(
    monkeypatch, tmp_path: Path
) -> None:
    _passthrough_samples(monkeypatch)
    directories = _fleet_dirs(tmp_path, [121], scoring_active={121}, finished={121})
    truth = _Truth(closure_after=1)
    stream = _Stream([
        None,
        _sim_state(121),  # reaches closure
        None,
        _sim_state(121),  # a re-approach; must never enter the truth track
    ])

    collect.collect_fleet(
        stream, {121: _child()}, directories,
        {121: _Recorder()}, {121: _Recorder()}, {121: truth}, timeout_s=5.0,
    )

    assert truth.scoring_active_flags == [True]
    assert truth.finalize_calls == 1


def test_a_truth_tail_that_never_closes_is_bounded_not_waited_on(
    monkeypatch, tmp_path: Path
) -> None:
    _passthrough_samples(monkeypatch)
    monkeypatch.setattr(one, "TRUTH_CLOSURE_DRAIN_TIMEOUT_S", 0.05)
    directories = _fleet_dirs(tmp_path, [121], scoring_active={121}, finished={121})
    truth = _Truth(closure_after=99)

    results, failures = collect.collect_fleet(
        _Stream([]), {121: _child()}, directories,
        {121: _Recorder()}, {121: _Recorder()}, {121: truth}, timeout_s=5.0,
    )

    assert set(results) == {121} and failures == {}
    assert truth.finalize_calls == 1


def test_a_closure_window_survives_the_overall_deadline(
    monkeypatch, tmp_path: Path
) -> None:
    """A result landing near the deadline keeps its full closure window --
    the single case grants it unconditionally, so truncating it here would
    void late finishers that the one-aircraft harness would certify."""
    _passthrough_samples(monkeypatch)
    monkeypatch.setattr(one, "TRUTH_CLOSURE_DRAIN_TIMEOUT_S", 5.0)
    directories = _fleet_dirs(tmp_path, [121], scoring_active={121}, finished={121})
    truth = _Truth(closure_after=1)
    stream = _Stream([None, None, None, None, _sim_state(121)])

    results, failures = collect.collect_fleet(
        stream, {121: _child()}, directories,
        {121: _Recorder()}, {121: _Recorder()}, {121: truth},
        timeout_s=0.001,  # expires before the closure sample arrives
    )

    assert set(results) == {121} and failures == {}
    assert truth.scoring_active_flags == [True]
    assert truth.finalize_calls == 1


def test_without_truth_recorders_the_drain_still_scores_and_finishes(
    monkeypatch, tmp_path: Path
) -> None:
    """Estimate policy: same collect loop, no truth consumers at all."""
    _passthrough_samples(monkeypatch)
    directories = _fleet_dirs(tmp_path, [121], scoring_active={121}, finished={121})
    scorer = _Recorder()

    results, failures = collect.collect_fleet(
        _Stream([_position(121)]), {121: _child()}, directories,
        {121: scorer}, {121: _Recorder()}, {}, timeout_s=5.0,
    )

    assert set(results) == {121} and failures == {}
    assert scorer.samples == [("sample", 121)]


def test_a_dead_child_becomes_a_named_failure_not_a_lost_fleet(
    monkeypatch, tmp_path: Path
) -> None:
    """One crash must not discard thirty-five other aircraft's evidence."""
    _passthrough_samples(monkeypatch)
    directories = _fleet_dirs(tmp_path, [121, 122], scoring_active={121}, finished={121})
    truths = {121: _Truth(closure_after=2), 122: _Truth(closure_after=1)}
    crashed = SimpleNamespace(poll=lambda: 1, returncode=1)
    # Pass 1 sees the crash; pass 2 still carries a SIM_STATE for the dead
    # aircraft, which must bounce off its closed truth door while the
    # healthy aircraft's closure evidence keeps flowing.
    stream = _Stream([_sim_state(121), None, _sim_state(122), _sim_state(121)])

    results, failures = collect.collect_fleet(
        stream, {121: _child(), 122: crashed}, directories,
        {121: _Recorder(), 122: _Recorder()},
        {121: _Recorder(), 122: _Recorder()}, truths, timeout_s=5.0,
    )

    assert set(results) == {121}
    assert failures == {122: "direct pixel child sysid=122 exited code=1"}
    assert truths[121].scoring_active_flags == [True, True]
    assert truths[122].scoring_active_flags == []
    # Both stamps land at each aircraft's own door close -- the failure
    # detection for 122, the closure for 121 -- exactly once each.
    assert truths[121].finalize_calls == 1
    assert truths[122].finalize_calls == 1


def test_missing_results_time_out_as_named_failures(
    monkeypatch, tmp_path: Path
) -> None:
    _passthrough_samples(monkeypatch)
    directories = _fleet_dirs(tmp_path, [121], scoring_active=set(), finished=set())
    truth = _Truth(closure_after=99)

    results, failures = collect.collect_fleet(
        _Stream([]), {121: _child()}, directories,
        {121: _Recorder()}, {121: _Recorder()}, {121: truth}, timeout_s=0.05,
    )

    assert results == {}
    assert failures == {121: "no result.json within 0.05s"}
    assert truth.finalize_calls == 1


def test_a_half_written_result_is_retried_while_the_child_lives(
    monkeypatch, tmp_path: Path
) -> None:
    """The publish is atomic now, but an unreadable file must never lose the
    fleet: a living child gets another pass to finish it."""
    _passthrough_samples(monkeypatch)
    directories = _fleet_dirs(tmp_path, [121], scoring_active=set(), finished=set())
    result_path = directories[121] / "result.json"
    result_path.write_text('{"passed": tr', encoding="utf-8")  # mid-write

    class _HealingChild:
        returncode = None

        def __init__(self) -> None:
            self.polls = 0

        def poll(self) -> None:
            self.polls += 1
            if self.polls >= 2:
                result_path.write_text('{"passed": true}', encoding="utf-8")
            return None

    results, failures = collect.collect_fleet(
        _Stream([]), {121: _HealingChild()}, directories,
        {121: _Recorder()}, {121: _Recorder()}, {}, timeout_s=5.0,
    )

    assert failures == {}
    assert results == {121: {"passed": True}}


def test_an_unreadable_result_from_an_exited_child_is_a_named_failure(
    monkeypatch, tmp_path: Path
) -> None:
    _passthrough_samples(monkeypatch)
    directories = _fleet_dirs(tmp_path, [121], scoring_active=set(), finished=set())
    (directories[121] / "result.json").write_text('{"pas', encoding="utf-8")
    exited = SimpleNamespace(poll=lambda: 3, returncode=3)
    truth = _Truth(closure_after=99)

    results, failures = collect.collect_fleet(
        _Stream([]), {121: exited}, directories,
        {121: _Recorder()}, {121: _Recorder()}, {121: truth}, timeout_s=5.0,
    )

    assert results == {}
    assert failures == {
        121: "unreadable result.json from exited child sysid=121 (code=3)"
    }
    assert truth.finalize_calls == 1


def test_an_early_finisher_is_finalized_at_its_own_door_close(
    monkeypatch, tmp_path: Path
) -> None:
    """The first acceptance flight lost 2 of 3 aircraft to the 2 s
    trailing-freshness gate: their finalize stamps were deferred to the
    fleet-wide verdict pass, ~12 s after their own episodes ended. The
    stamp must land when THIS aircraft's truth door closes, not when the
    last sibling lands."""
    _passthrough_samples(monkeypatch)
    events: list = []

    class _StampTruth(_Truth):
        def __init__(self, sys_id: int, closure_after: int) -> None:
            super().__init__(closure_after)
            self._sys_id = sys_id

        def finalize(self) -> None:
            super().finalize()
            events.append(("finalize", self._sys_id))

    directories = _fleet_dirs(
        tmp_path, [121, 122], scoring_active={121, 122}, finished={121}
    )
    late_result = directories[122] / "result.json"

    class _LateChild:
        returncode = None

        def __init__(self) -> None:
            self.polls = 0

        def poll(self) -> None:
            self.polls += 1
            if self.polls == 3:
                events.append(("result", 122))
                late_result.write_text('{"passed": true}', encoding="utf-8")
            return None

    truths = {
        121: _StampTruth(121, closure_after=1),
        122: _StampTruth(122, closure_after=1),
    }
    stream = _Stream([
        _sim_state(121),  # closes 121 on the first pass
        None, None, None,
        _sim_state(122),  # 122's closure once its late result lands
    ])

    results, failures = collect.collect_fleet(
        stream, {121: _child(), 122: _LateChild()}, directories,
        {121: _Recorder(), 122: _Recorder()},
        {121: _Recorder(), 122: _Recorder()}, truths, timeout_s=5.0,
    )

    assert set(results) == {121, 122} and failures == {}
    assert truths[121].finalize_calls == 1
    assert truths[122].finalize_calls == 1
    assert events.index(("finalize", 121)) < events.index(("result", 122))


def test_the_child_publishes_its_result_atomically(tmp_path: Path) -> None:
    """The file's existence is the evaluators' signal; they parse it the
    moment it appears, so a partial write reads as a corrupt result."""
    from scripts.direct_pixel_pn_child import publish_result

    target = tmp_path / "result.json"
    publish_result(target, {"passed": True})
    assert json.loads(target.read_text(encoding="utf-8")) == {"passed": True}
    publish_result(target, {"passed": False})
    assert json.loads(target.read_text(encoding="utf-8")) == {"passed": False}
    assert list(tmp_path.iterdir()) == [target]


def test_the_fleet_parser_defaults_to_certified_truth_scoring() -> None:
    """The bare parser would pin vehicle-estimate scoring silently."""
    args = fleet._parser().parse_args([])

    assert args.scoring_policy == SCORING_POLICY_SITL_TRUTH
    assert args.sim_cpa is None


# --- scoring helpers ---------------------------------------------------------


def test_truth_recorders_bind_each_aircraft_to_its_own_home(monkeypatch) -> None:
    """Co-located starts hide this today; a grid layout must not break it."""
    monkeypatch.setattr(score, "TruthRecorder", lambda target, home: (target, home))
    plan = SimpleNamespace(home_alts={121: 621.0, 122: 622.0})
    targets = {121: "t1", 122: "t2"}

    recorders = score.truth_recorders(
        [121, 122], targets, plan, _args(SCORING_POLICY_SITL_TRUTH)
    )

    assert recorders == {121: ("t1", 621.0), 122: ("t2", 622.0)}
    assert score.truth_recorders(
        [121, 122], targets, plan, _args(SCORING_POLICY_VEHICLE_ESTIMATE)
    ) == {}


def test_seal_fleet_folds_the_cross_check_in_before_each_atomic_write(
    monkeypatch, tmp_path: Path
) -> None:
    """BIN order and schema split: module first, then the one write; wind keys
    live on the fleet row only, the disk file keeps the single-case schema."""
    events: list = []

    class _SealStage:
        def post_teardown(self, directory, *, sysid, truth_block):
            events.append(("post_teardown", sysid, truth_block))
            return {"sysid": sysid}

    monkeypatch.setattr(
        score,
        "persist_verdict",
        lambda directory, verdict: events.append(("persist", dict(verdict))),
    )
    verdicts = {121: {"passed": True, "truth": {"dist_3d_m": 0.1}}}

    vehicles = score.seal_fleet(
        verdicts, {121: _SealStage()}, {121: tmp_path}, {121: (8.0, 90.0)}
    )

    assert [row[0] for row in events] == ["post_teardown", "persist"]
    assert events[0][2] == {"dist_3d_m": 0.1}
    persisted = events[1][1]
    assert persisted["sim_cpa"] == {"sysid": 121}
    assert "wind_speed_mps" not in persisted
    assert vehicles["121"]["wind_speed_mps"] == 8.0
    assert vehicles["121"]["wind_dir_deg"] == 90.0


def test_each_aircraft_writes_the_certified_case_manifest_before_its_params(
    monkeypatch,
) -> None:
    """The SIM_CPA scorer reads `case.json` back; a fleet aircraft without
    one has every BIN cross-check die as parse_failed. Written before the
    parameter push, like the single case, so a death on an unserved
    parameter still leaves the record of what was configured."""
    events: list = []
    _wire_setup(monkeypatch, events)
    manifests: list = []
    monkeypatch.setattr(
        setup,
        "write_case_manifest",
        lambda case_dir, **fields: (
            events.append(("manifest", fields["target"][1][-3:], None)),
            manifests.append((case_dir, fields)),
        ),
    )
    directories = {121: Path("uav-121"), 122: Path("uav-122")}

    setup.prepare_aircraft(
        _master(), [121, 122], [(8.0, 90.0), (5.0, 270.0)],
        args=_args(SCORING_POLICY_SITL_TRUTH), speedup=1.0,
        case_dirs=directories,
        manifest=setup.AircraftManifest(
            identity={"sha256": "abc"}, speedup=1.0,
            launch_speedup=10.0, repetition=1,
        ),
    )

    order = [(kind, sys_id) for kind, sys_id, _ in events]
    assert order.index(("manifest", "121")) < min(
        index for index, row in enumerate(order) if row == ("param", 121)
    )
    directory, fields = manifests[0]
    assert directory == directories[121]
    assert fields["identity"] == {"sha256": "abc"}
    assert (fields["speedup"], fields["launch_speedup"]) == (1.0, 10.0)
    # The cell's EFFECTIVE pushes, wind included -- what the fleet-level
    # manifest cannot carry per aircraft.
    assert ("SIM_WIND_SPD", 8.0) in fields["sitl_params_pushed"]
    assert ("SIM_WIND_DIR", 270.0) in manifests[1][1]["sitl_params_pushed"]


def test_a_failed_aircraft_is_salvaged_into_an_unscored_verdict(
    monkeypatch, tmp_path: Path
) -> None:
    """One aircraft's crash must yield the same audit row a failed single
    case yields -- truth salvaged, cross-check error block, persisted --
    while the healthy aircraft still gets the certified verdict."""
    scored: list = []
    monkeypatch.setattr(
        score,
        "case_verdict",
        lambda case_dir, **kwargs: scored.append(kwargs) or {"passed": True},
    )
    salvaged: list = []
    monkeypatch.setattr(
        score,
        "salvage_truth",
        lambda truth, case_dir, errors: salvaged.append((truth, case_dir))
        or {"dist_3d_m": 0.2},
    )
    unscored_rows: list = []

    def _unscored(errors, **kwargs):
        unscored_rows.append((errors, kwargs))
        return {"passed": False, "scoring_source": "none"}

    monkeypatch.setattr(score, "unscored_result", _unscored)

    class _FinalizeTruth:
        def __init__(self) -> None:
            self.finalize_calls = 0
            self.tracks_written: list[Path] = []

        def finalize(self) -> None:
            self.finalize_calls += 1

        def write_track(self, path: Path) -> None:
            self.tracks_written.append(path)

    class _ErrorStage:
        def error_block(self) -> dict:
            return {"config": {"status": "error"}}

    truths = {121: _FinalizeTruth(), 122: _FinalizeTruth()}
    directories = {121: tmp_path / "uav-121", 122: tmp_path / "uav-122"}
    for directory in directories.values():
        directory.mkdir()
    plan = SimpleNamespace(truth_stream_accepted={121: True, 122: True})

    verdicts, unscored = score.fleet_verdicts(
        [121, 122],
        directories=directories,
        child_results={121: {"passed": True}},
        failures={122: "direct pixel child sysid=122 exited code=1"},
        scorers={121: "scorer-121", 122: "scorer-122"},
        tracks={121: "track-121", 122: "track-122"},
        truths=truths,
        stages={121: _ErrorStage(), 122: _ErrorStage()},
        plan=plan,
        args=_args(SCORING_POLICY_SITL_TRUTH),
        assigned={121: (0.0, 0.0), 122: (8.0, 90.0)},
        speedup=1.0,
    )

    assert unscored == {122}
    assert verdicts[121] == {"passed": True} and len(scored) == 1
    # Neither recorder is re-finalized here: the stamp landed at each
    # aircraft's own truth-door close in collect_fleet, and moving it
    # would measure the fleet's finish-time spread against the 2 s
    # freshness gate. The healthy aircraft still gets its track written.
    assert truths[121].finalize_calls == 0
    assert truths[122].finalize_calls == 0
    assert truths[121].tracks_written == [
        directories[121] / "truth_track.csv"
    ]
    assert salvaged == [(truths[122], directories[122])]
    errors, kwargs = unscored_rows[0]
    assert errors == ["direct pixel child sysid=122 exited code=1"]
    assert kwargs["truth_block"] == {"dist_3d_m": 0.2}
    assert kwargs["case_dir"] == directories[122]
    assert kwargs["sim_cpa"] == {"config": {"status": "error"}}


def test_a_failed_childs_missing_evidence_does_not_cost_the_sibling_its_verdict(
    monkeypatch, tmp_path: Path
) -> None:
    """The common real shape: a child writes passed=false and leaves no
    navigation log, so case_verdict raises on missing evidence. That aircraft
    must become unscored WITH its child result preserved; the healthy
    sibling still gets its certified verdict."""

    def _case_verdict(case_dir, **kwargs):
        if case_dir.name == "uav-122":
            raise RuntimeError("expected one compact navigation log, found 0")
        return {"passed": True, "scored": case_dir.name}

    monkeypatch.setattr(score, "case_verdict", _case_verdict)
    unscored_rows: list = []

    def _unscored(errors, **kwargs):
        unscored_rows.append((errors, kwargs))
        return {"passed": False, "scoring_source": "none"}

    monkeypatch.setattr(score, "unscored_result", _unscored)
    monkeypatch.setattr(
        score, "salvage_truth", lambda truth, case_dir, errors: None
    )

    class _InertTruth:
        def write_track(self, path: Path) -> None:
            pass

    directories = {121: tmp_path / "uav-121", 122: tmp_path / "uav-122"}
    for directory in directories.values():
        directory.mkdir()
    failed_child_result = {"passed": False, "error": "NavigationError: lost lock"}

    class _ErrorStage:
        def error_block(self) -> dict:
            return {"config": {"status": "error"}}

    verdicts, unscored = score.fleet_verdicts(
        [121, 122],
        directories=directories,
        child_results={121: {"passed": True}, 122: failed_child_result},
        failures={},
        scorers={121: "s1", 122: "s2"},
        tracks={121: "t1", 122: "t2"},
        truths={121: _InertTruth(), 122: _InertTruth()},
        stages={121: _ErrorStage(), 122: _ErrorStage()},
        plan=SimpleNamespace(truth_stream_accepted={121: True, 122: True}),
        args=_args(SCORING_POLICY_SITL_TRUTH),
        assigned={121: (0.0, 0.0), 122: (0.0, 0.0)},
        speedup=1.0,
    )

    assert verdicts[121] == {"passed": True, "scored": "uav-121"}
    assert unscored == {122}
    errors, kwargs = unscored_rows[0]
    assert errors == [
        "verdict construction failed: RuntimeError: "
        "expected one compact navigation log, found 0"
    ]
    assert kwargs["child_result"] is failed_child_result


def test_seal_fleet_leaves_unscored_aircraft_sealed(
    monkeypatch, tmp_path: Path
) -> None:
    """An unscored row was persisted with its error block already; running
    the cross-check or re-persisting it would overwrite the salvage."""
    events: list = []

    class _SealStage:
        def post_teardown(self, directory, *, sysid, truth_block):
            events.append(("post_teardown", sysid))
            return {"sysid": sysid}

    monkeypatch.setattr(
        score, "persist_verdict",
        lambda directory, verdict: events.append(("persist", directory.name)),
    )
    verdicts = {
        121: {"passed": True, "truth": None},
        122: {"passed": False, "scoring_source": "none"},
    }

    vehicles = score.seal_fleet(
        verdicts,
        {121: _SealStage(), 122: _SealStage()},
        {121: tmp_path / "uav-121", 122: tmp_path / "uav-122"},
        {121: (0.0, 0.0), 122: (8.0, 90.0)},
        unscored={122},
    )

    assert events == [("post_teardown", 121), ("persist", "uav-121")]
    assert vehicles["122"]["wind_speed_mps"] == 8.0
    assert vehicles["122"]["passed"] is False


def test_the_fleet_identity_closure_contains_the_fleet_harness_itself() -> None:
    """The single-case closure does not reach these files; an edit to the
    fleet path under the old identity flew unrecorded."""
    names = {path.name for path in setup.FLEET_IDENTITY_ROOTS}

    assert {
        "eval_direct_pixel_pn_fleet.py",
        "eval_fleet_cells.py",
        "eval_fleet_clock.py",
        "eval_fleet_collect.py",
        "eval_fleet_manifest.py",
        "eval_fleet_report.py",
        "eval_fleet_score.py",
        "eval_fleet_setup.py",
        "eval_fleet_span.py",
        "eval_direct_pixel_pn_three_uav.py",
        "eval_direct_pixel_pn.py",
    } <= names


def test_every_aircraft_is_held_for_its_ekf_origin_during_preparation(
    monkeypatch,
) -> None:
    """The 2026-09-05 round-2 review regression: the fleet flow launched its
    timed children with no origin wait absorbed, so the arm-site gate was the
    FIRST readiness wait and could eat every child's scoring interval deadline."""
    events: list = []
    _wire_setup(monkeypatch, events)
    setup.prepare_aircraft(
        _master(),
        [121, 122, 123],
        [(0.0, 0.0)] * 3,
        args=_args("sitl-truth"),
        speedup=1.0,
    )
    gated = [sysid for kind, sysid, _ in events if kind == "nav_gate"]
    assert gated == [121, 122, 123]
    # The hold happens during preparation, before any parameter push completes
    # the aircraft: for each sysid the gate precedes its first param event.
    for sysid in gated:
        order = [kind for kind, s, _ in events if s == sysid]
        assert order.index("nav_gate") < order.index("param")
