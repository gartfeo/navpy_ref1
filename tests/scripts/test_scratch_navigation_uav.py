"""Guards on the harness that a reader cannot check by eye.

The first test inspects the abstract syntax tree rather than behaviour, because
what it guards is structural. Stopping the run is part of the command path: a
check that ends the loop decides whether the law is handed any further frame, so
a run stopped on truth range or truth altitude has had its command sequence
shaped by truth. That is invisible to every behavioural test -- the miss still
looks fine -- and it is not fixed by moving the check below the command send,
which only shifts the gate by one iteration.
"""

from __future__ import annotations

import argparse
import ast
import inspect
import math
import statistics
import textwrap

import numpy as np

import pytest

import scripts.scratch_navigation_uav as harness

# Every name a run-ending condition in run_navigation_episode() may read, and why:
#
#   time.monotonic, wall_guard_s   the hung-run guard. Wall clock, but it can
#     first_pose_deadline_s        only abandon a run, never alter one -- and
#                                  the deadline for the FIRST pose has to be
#                                  wall time, because no pose has arrived yet to
#                                  carry a simulated clock.
#   was_ahead, latest_frame.body_x the sensor's own output -- the identical
#                                  field handed to the law.
#   pose, started_t_s              whether a pose arrived at all, and whether
#                                  the scoring interval has begun. Feed state, not
#                                  world state: neither says anything about
#                                  where the POI is.
#   PASSED_BEHIND_MAX              a constant.
#   elapsed_s, options.scoring_duration_s  the simulated clock the law also uses.
#
# Anything else -- range, altitude, POI position, ground speed -- is a truth
# quantity gating the command sequence, whatever its position in the loop.
LEGAL_IN_A_TERMINATING_CONDITION = {
    "time.monotonic",
    "wall_guard_s",
    "first_pose_deadline_s",
    "was_ahead",
    "latest_frame",
    "latest_frame.body_x",
    "pose",
    "started_t_s",
    "PASSED_BEHIND_MAX",
    "elapsed_s",
    'options.scoring_duration_s',
}


def _dotted(node: ast.AST) -> list[str]:
    """Every Name/Attribute chain in an expression, as dotted strings."""
    names: list[str] = []
    for child in ast.walk(node):
        if isinstance(child, ast.Attribute):
            try:
                names.append(ast.unparse(child))
            except AttributeError:  # pragma: no cover - Python < 3.9
                pass
        elif isinstance(child, ast.Name):
            names.append(child.id)
    return names


def _command_path_tree() -> "ast.Module":
    """`run_navigation_episode` AND its estimate pipeline, parsed as one.

    The pipeline moved out of `run_navigation_episode` into `scratch_navigation_estimate` when
    the loop hit its size ceiling. Parsing only `run_navigation_episode` after that move would
    leave these guards passing while checking nothing -- the same way an earlier
    version stopped seeing the truth substitution when it moved between two
    syntaxes. The constraint is about the COMMAND PATH, so the guard follows the
    code wherever the command path lives.
    """
    import scripts.scratch_navigation_estimate as estimate

    return ast.parse(
        textwrap.dedent(inspect.getsource(harness.run_navigation_episode))
        + "\n\n"
        + textwrap.dedent(inspect.getsource(estimate.de_rotating_attitude))
    )


def _terminating_conditions(function) -> list[ast.expr]:
    tree = ast.parse(textwrap.dedent(inspect.getsource(function)))
    tests = []
    for node in ast.walk(tree):
        if isinstance(node, ast.If) and any(
            isinstance(inner, ast.Return) for inner in ast.walk(node)
        ):
            tests.append(node.test)
    return tests


def test_nothing_the_aircraft_lacks_decides_when_the_run_ends():
    """Termination is a command-path decision, so it takes command-path inputs.

    An earlier version stopped on truth range and truth altitude, then moved
    those checks below the command send and called them safe. They were not:
    wherever they sat, they decided whether the loop ran again and therefore
    whether the law received another frame.
    """
    conditions = _terminating_conditions(harness.run_navigation_episode)
    assert conditions, "run_navigation_episode() must have run-ending conditions to guard"
    for condition in conditions:
        used = set(_dotted(condition))
        # A dotted name implies its prefix; keep only the longest form.
        used -= {name for name in used
                 if any(other != name and other.startswith(name + ".")
                        for other in used)}
        illegal = used - LEGAL_IN_A_TERMINATING_CONDITION
        assert not illegal, (
            f"{ast.unparse(condition)!r} ends the run using {sorted(illegal)}, "
            "which the command path may not read"
        )


def test_ground_contact_is_recorded_and_never_acted_on():
    """Altitude may be logged, so it must be logged and nothing else.

    The check exists because a run that flew into the ground is worth knowing
    about. It must set a field and fall through -- returning on it would put
    altitude back in charge of how many commands the law issues.
    """
    tree = ast.parse(textwrap.dedent(inspect.getsource(harness.run_navigation_episode)))
    altitude_checks = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.If) and "flight.alt_now_m" in _dotted(node.test)
    ]
    assert len(altitude_checks) == 1, "one altitude check, for the report"
    assert not [
        inner for inner in ast.walk(altitude_checks[0])
        if isinstance(inner, (ast.Return, ast.Break, ast.Continue))
    ], "altitude must not change control flow"


def test_the_law_never_receives_a_truth_derived_quantity():
    """Only a FinalApproachVisionFrame crosses into the law.

    Pinned at the call site: whatever else run_navigation_episode() computes, the single
    argument to law.plan() is the frame, whose rays are unit vectors and whose
    scalars are the aircraft's own estimate.
    """
    tree = ast.parse(textwrap.dedent(inspect.getsource(harness.run_navigation_episode)))
    calls = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call) and ast.unparse(node.func) == "law.plan"
    ]
    assert len(calls) == 1, "the law is consulted in exactly one place"
    (call,) = calls
    assert not call.keywords
    assert [ast.unparse(argument) for argument in call.args] == ["latest_frame"]


def test_sim_truth_attitude_reaches_the_law_on_the_control_arm_only():
    """The `truth` arm is a diagnostic control, and must stay one.

    It exists to BOUND the question of whether ray/attitude misalignment causes
    the roll limit cycle, and it does that by handing the command path sim-truth
    pitch and roll -- which the flying article does not have. That is legitimate
    for a control and invalid for anything else, so two things are pinned here:
    the default arm is the one the aircraft actually flies, and the truth
    substitution is reachable ONLY under the branch that names it.

    It bounds rather than proves, because it moves fidelity and timing together.
    The causal claim belongs to a delay sweep at FIXED fidelity, which is why
    `--estimate-delay-poses` applies to either source.

    Neither is visible in a result file. A run on the truth arm produces a
    miss, a command count and a reversal count that all look ordinary; nothing
    in the numbers says the aircraft was told what it could not have known.
    """
    assert harness._parser().get_default("estimate_source") == "attitude"

    tree = _command_path_tree()
    truth_reads = {"pose.pitch_deg", "pose.roll_deg"}

    # Written against READS rather than assignments.  An earlier version walked
    # ast.Assign, which stopped seeing the substitution the moment it moved from
    # `a, b = pose.pitch_deg, pose.roll_deg` into `estimates.append((...))` --
    # the guard kept passing while checking nothing.  Every read has to land in
    # one of exactly two places, whatever syntax carries it.
    control_arm = {
        id(inner)
        for node in ast.walk(tree)
        if isinstance(node, ast.If)
        and "options.estimate_source == 'truth'" in ast.unparse(node.test)
        for statement in node.body
        for inner in ast.walk(statement)
    }
    # Truth BUILDS THE RAY, and that is its job -- a camera is never wrong about
    # where a thing appears.  Only the de-rotation is under test.
    ray = {
        id(inner)
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and ast.unparse(node.func) == "build_frame"
        for keyword in node.keywords
        if (keyword.arg or "").startswith("truth_")
        for inner in ast.walk(keyword.value)
    }
    # Truth may also be RECORDED, and recording is not commanding.  The measured
    # attenuation of the roll command rests on the aircraft's own estimate of
    # its roll, so without simulator truth beside it there is no way to tell an
    # airframe that ignores the command from an estimate that smooths it.
    #
    # This is safe rather than merely convenient, and the safety is structural:
    # `test_the_law_never_receives_a_truth_derived_quantity` pins the law's only
    # argument to `latest_frame`, and the `ray` set above pins the frame's only
    # truth arguments.  A value that reaches neither cannot influence a command.
    # That is checked here rather than assumed -- the recorder must not appear
    # anywhere in the frame's construction.
    recorded = {
        id(inner)
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and ast.unparse(node.func) == "roll.observe_command"
        for inner in ast.walk(node)
    }
    # Writes straight into the report are recording too.  The scoring interval-entry
    # state has to carry truth pitch and roll: five groups of runs were compared
    # as repeats of an identical start when only four pose fields had been
    # recorded, and entry attitude was among the unrecorded quantities that
    # could have decided the outcome.  A value written into `result` and read
    # back by nothing cannot reach a command.
    recorded |= {
        id(inner)
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and isinstance(node.targets[0], ast.Subscript)
        and ast.unparse(node.targets[0]).startswith("result[")
        for inner in ast.walk(node.value)
    }
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if ast.unparse(node.func) != "build_frame":
            continue
        # By IDENTIFIER, not substring: `build_frame` takes a keyword called
        # `truth_roll_deg`, so a substring test matches the legitimate ray
        # argument and fails on correct code.
        assert not [
            inner for inner in ast.walk(node)
            if isinstance(inner, ast.Name) and inner.id == "roll"
        ], "the truth RECORDER is feeding the frame; it may only be read"

    supplied = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Attribute):
            continue
        name = ast.unparse(node)
        if name not in truth_reads:
            continue
        assert (id(node) in control_arm or id(node) in ray
                or id(node) in recorded), (
            f"sim truth attitude is read at line {node.lineno} ({name}) from "
            "none of the control arm, the ray, or the report recorder"
        )
        if id(node) in control_arm:
            supplied.add(name)
    assert supplied == truth_reads, (
        "the control arm no longer supplies truth pitch/roll"
    )


@pytest.mark.parametrize(
    "values,median",
    [
        # The shape the statistic exists to detect, and the one the first
        # version got wrong: a command dithering between two extremes.  Taking
        # ordered[n // 2] returns the UPPER middle value on an even-length
        # series, reporting 20 -- indistinguishable from a command that held
        # 20 degrees for the whole run, which is the opposite diagnosis.
        ([0.0, 0.0, 20.0, 20.0], 10.0),
        ([0.0, 20.0] * 500, 10.0),
        ([1.0, 2.0, 3.0, 4.0], 2.5),
        ([1.0, 2.0, 3.0], 2.0),
        ([7.0], 7.0),
        ([2.0] * 100, 2.0),
    ],
)
def test_the_amplitude_summary_reports_the_real_median(values, median):
    assert harness._spread(values)["median_deg"] == pytest.approx(median)
    assert harness._spread(values)["median_deg"] == pytest.approx(
        statistics.median(values)
    )


def test_the_amplitude_summary_never_invents_a_command():
    """p95 must be a magnitude the aircraft was actually given.

    Interpolating quantile definitions return a value between two samples. For
    a diagnostic whose whole job is to say how hard the command swung, a number
    that was never commanded is the same class of error as the median was.
    """
    for count in (1, 2, 19, 20, 100, 999):
        values = [(index * 7919 % 3001) / 100.0 for index in range(count)]
        spread = harness._spread(values)
        rounded = {round(value, 3) for value in values}
        assert spread["p95_deg"] in rounded
        assert spread["max_deg"] == pytest.approx(max(values))
        assert spread["samples"] == count
    assert harness._spread([]) is None


def test_the_frame_gets_an_euler_yaw_rate_not_a_body_rate():
    """MAVLink yawspeed is body r; the law's compensation term wants psi-dot.

    They differ by roughly cos(bank), so passing r understates the term by ~18%
    at 35 degrees -- and the shortfall GROWS WITH BANK, which is a positive
    feedback rather than a bias. The harness did pass r straight through, so
    every run before this measured the law against a yaw rate the real aircraft
    never sees. `real_detected_object_builder.py:122-125` does the conversion on
    the production path.

    Pinned two ways: the frame's yaw rate must be the CONVERTED value, and the
    conversion must be production's function rather than a second copy here --
    two implementations of the same transform is how a harness comes to
    disagree with the thing it is measuring.
    """
    tree = ast.parse(textwrap.dedent(inspect.getsource(harness.run_navigation_episode)))
    built = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call) and ast.unparse(node.func) == "build_frame"
    ]
    (call,) = built
    passed = {
        keyword.arg: ast.unparse(keyword.value) for keyword in call.keywords
    }
    assert "est_yaw_rate_rad_s" not in passed["yaw_rate_rad_s"], (
        "the frame is being handed the raw body rate again"
    )
    converted = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and ast.unparse(node.func) == "aircraft_yaw_rate_rad_s"
    ]
    assert len(converted) == 1, "one conversion, from production's own function"

    # Behavioural, not only structural: in a coordinated turn the body rate and
    # the Euler rate must actually come apart, or the guard is pinning a
    # conversion that does nothing.
    from navpy.modules.vision.models.pixel_observation import (
        aircraft_yaw_rate_rad_s,
    )
    turn_rate = 0.3
    for bank_deg, minimum_error in ((0.0, 0.0), (35.0, 0.15)):
        bank = math.radians(bank_deg)
        rates = (0.0, turn_rate * math.sin(bank), turn_rate * math.cos(bank))
        euler = aircraft_yaw_rate_rad_s(0.0, bank_deg, rates)
        assert euler == pytest.approx(turn_rate)
        assert abs(rates[2] - euler) / turn_rate >= minimum_error


def test_a_command_instant_without_a_fresh_frame_fails_the_run():
    """Holding through a command instant is flying part of the scoring interval open-loop.

    The harness can fail to produce a frame for a pose two ways: the yaw rate
    would not convert, or airspeed was not yet known. Neither is the law
    declining a frame -- it is the harness failing to supply one -- and in both
    cases the aircraft keeps flying on the previous attitude command.

    An earlier version left `latest_frame` untouched and re-planned it. The law
    then saw a non-positive dt, returned a held command, and that command was
    tallied into `commands`, the reversal count and the roll amplitude,
    indistinguishable from one computed for that instant.
    """
    clean = harness.classification_errors(
        scoring_end="passed", miss_m=0.9, certification=None,
    )
    assert clean == []
    held = harness.classification_errors(
        scoring_end="passed", miss_m=0.9, certification=None, held_commands=1,
    )
    assert len(held) == 1
    assert "no frame from that pose" in held[0]
    assert "1 command instants" in held[0]


def test_a_frame_is_never_presented_to_the_law_twice():
    """Pinned structurally: commanding requires the frame to match this pose.

    The freshness test is what stops a stale frame reaching the law, and it is
    invisible in any result -- a run that re-plans stale frames still reports a
    miss, a command count and an amplitude that all look ordinary.
    """
    tree = ast.parse(textwrap.dedent(inspect.getsource(harness.run_navigation_episode)))
    planned = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call) and ast.unparse(node.func) == "law.plan"
    ]
    assert len(planned) == 1
    # The guard has to sit on a branch that ENCLOSES the call, not merely
    # somewhere in the function.
    enclosing = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.If)
        and any(inner is planned[0] for inner in ast.walk(node))
        and "latest_frame_t_s" in ast.unparse(node)
    ]
    assert enclosing, "nothing checks the frame belongs to the current pose"
    stamped = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and ast.unparse(node.targets[0]) == "latest_frame_t_s"
        and ast.unparse(node.value) == "pose.t_s"
    ]
    assert stamped, "the frame is never stamped with the pose it came from"


def _observe_all(values):
    series = harness.RollSeries()
    for value in values:
        series.observe(value)
    return series.summary()


def test_the_entry_state_records_a_checkable_level_entry():
    """Pitch at trim does not mean the climb has stopped.

    A level-POI cell's whole point is a level entry, and entry pitch alone
    cannot certify one: an aircraft can hold trim pitch while still climbing
    out its energy. The ground-frame path angle can -- so it must be IN the
    artifact, and `run_navigation_episode` must pass it, because the parameter's
    None default means forgetting the call site records "unmeasured" silently
    on every run.
    """
    from types import SimpleNamespace

    from scripts.scratch_navigation_result import entry_state

    pose = SimpleNamespace(
        lat_deg=1.0, lon_deg=2.0, alt_m=3.0, yaw_deg=4.0,
        pitch_deg=5.0, roll_deg=6.0, est_pitch_deg=5.1, est_roll_deg=6.1,
        est_roll_rate_rad_s=0.1, est_pitch_rate_rad_s=0.2,
        est_yaw_rate_rad_s=0.3, skew_s=0.01,
    )
    assert entry_state(pose, 26.0, path_angle_deg=1.75)["path_angle_deg"] == 1.75

    # A wind cell names a direction in the SIMULATOR's frame; whether that is a
    # head, tail or cross case depends on the heading flown, and the earlier
    # wind run recorded nothing that could check its own labels. Ground speed
    # against airspeed separates head from tail; course against yaw is the crab.
    windy = entry_state(pose, 26.0, 0.5, ground_speed_mps=33.0, course_deg=280.0)
    assert windy["ground_speed_mps"] == 33.0
    assert windy["course_deg"] == 280.0

    source = textwrap.dedent(inspect.getsource(harness.run_navigation_episode))
    for recorded in ("flight.path_angle_now_deg", "flight.ground_speed_now",
                     "flight.course_now_deg"):
        assert recorded in source, (
            f"run_navigation_episode() no longer records {recorded}; a wind cell that cannot "
            "verify its own case is the defect this exists to prevent"
        )


def test_the_guard_held_roll_commands_are_recorded_with_when_they_fell():
    """The ill-conditioning guard is invisible in every artifact before this.

    `_lateral_is_ill_conditioned` holds the roll command when the horizontal
    LOS norm shrinks or the pitch steepens -- geometry that tightens toward
    closest approach. A run whose roll law was frozen exactly where the miss
    was decided scored identically to one that steered to the end, so the
    guard's activity must reach the result, and with TIMES, not a bare count:
    only the times can say whether it fired in the final seconds.
    """
    source = textwrap.dedent(inspect.getsource(harness.run_navigation_episode))
    assert "plan.lateral_held" in source, (
        "run_navigation_episode() no longer reads the plan's lateral_held flag; the guard "
        "is invisible again"
    )
    assert "roll.held(pose.t_s)" in source, (
        "run_navigation_episode() no longer records WHEN the guard fired"
    )
    recorded = textwrap.dedent(inspect.getsource(harness.RollRecord))
    assert "lateral_held_commands" in recorded
    assert "lateral_held_final_3s" in recorded
    assert "t >= at_t_s - 3.0" in recorded, (
        "the final-seconds window is what says the guard fired where the "
        "miss was decided"
    )


def test_a_held_bank_and_an_oscillation_of_the_same_size_are_told_apart():
    """The distinction the whole `is it benign` question turns on.

    Magnitude alone cannot answer it: a steady 1.3 degree bank and a 1.3 degree
    dither have the SAME median magnitude. The step is what separates them --
    a held bank steps by ~0, an oscillation of amplitude A steps by ~2A -- and
    an earlier version recorded only `abs(roll)`, so it reported the two
    identically while being cited as evidence that the dither was harmless.
    """
    held = _observe_all([1.3] * 200)
    dither = _observe_all([1.3, -1.3] * 100)

    assert held["magnitude"]["median_deg"] == pytest.approx(1.3)
    assert dither["magnitude"]["median_deg"] == pytest.approx(1.3)

    assert held["step"]["median_deg"] == pytest.approx(0.0)
    assert dither["step"]["median_deg"] == pytest.approx(2.6)
    assert held["reversals"] == 0
    assert dither["reversals"] == 199


def test_a_series_inside_the_sign_band_is_not_counted_as_reversing():
    """Below the band a crossing is noise, not behaviour.

    Without it a signal hovering either side of zero registers a reversal every
    sample, and the count would describe the sensor rather than the aircraft.
    """
    assert _observe_all([0.4, -0.4] * 100)["reversals"] == 0
    assert _observe_all([2.0, -2.0] * 50)["reversals"] == 99


def test_an_empty_series_summarises_to_nothing_rather_than_zero():
    """A run that never commanded must not report a 0 degree roll.

    Zero reads as a measurement of a calm aircraft; absent is the truth.
    """
    summary = _observe_all([])
    assert summary["magnitude"] is None
    assert summary["step"] is None
    assert summary["reversals"] == 0
    assert _observe_all([5.0])["step"] is None


def test_command_and_response_are_summarised_by_the_same_code():
    """Both channels through one class, or the comparison is not like-for-like.

    The point of recording the aircraft's actual roll is to read it AGAINST the
    command. Two separate summarisers would let the two drift apart -- a
    different band, a different step convention -- and the comparison would
    silently stop meaning anything.
    """
    tree = ast.parse(textwrap.dedent(inspect.getsource(harness.RollRecord)))
    constructed = [
        ast.unparse(node.targets[0]) for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and ast.unparse(node.value) == "RollSeries()"
    ]
    assert sorted(constructed) == [
        "self.actual", "self.commanded", "self.truth",
    ]
    # run_navigation_episode() must not keep a private set alongside the record's.
    flying = ast.parse(textwrap.dedent(inspect.getsource(harness.run_navigation_episode)))
    assert not [
        node for node in ast.walk(flying)
        if isinstance(node, ast.Assign)
        and ast.unparse(node.value) == "RollSeries()"
    ]
    # Only the two summarisers, by name. Matching every `.observe` in the
    # function would also catch `approach.observe`, which is the closest-
    # approach tracker and nothing to do with roll.
    observed = {
        ast.unparse(node.func) for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and ast.unparse(node.func) in {
            f"{name}.observe" for name in constructed
        }
    }
    assert observed == {
        "self.commanded.observe", "self.actual.observe", "self.truth.observe",
    }


def test_the_law_and_the_pass_test_read_the_same_frame():
    """One frame per pose, or the two could disagree about where the POI is.

    An earlier version built a second frame inside the command block. Both were
    built from the same pose so they agreed by construction, but nothing
    enforced that, and a later edit to one would have silently diverged them.
    """
    tree = ast.parse(textwrap.dedent(inspect.getsource(harness.run_navigation_episode)))
    built = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call) and ast.unparse(node.func) == "build_frame"
    ]
    assert len(built) == 1, "exactly one frame is built per pose"


@pytest.mark.parametrize(
    "scoring_end,miss_m,certification,expected",
    [
        ("passed", 0.9, None, 0),
        ("passed", None, None, 1),
        ("limit", 88.0, None, 1),
        ("stalled", None, None, 2),
        ("passed", 0.9, "pose gap 0.31 s", 1),
        ("limit", None, "stall", 3),
    ],
)
def test_only_a_certified_completed_pass_reports_success(
    scoring_end, miss_m, certification, expected
):
    """A run that stopped early, or flew on a bad feed, must not exit zero.

    `miss_m` is populated either way -- it is the closest range reached before
    stopping -- so the exit code is the only thing separating a score from a
    number that merely looks like one.
    """
    errors = harness.classification_errors(
        scoring_end=scoring_end, miss_m=miss_m, certification=certification,
    )
    assert len(errors) == expected, errors
    assert bool(errors) == (
        scoring_end != "passed" or miss_m is None or bool(certification)
    )


def test_a_clean_pass_is_the_only_thing_that_exits_zero():
    assert harness.classification_errors(
        scoring_end="passed", miss_m=0.4, certification=None,
    ) == []


def test_both_pose_streams_are_asked_for_and_the_reply_is_recorded():
    """SIM_STATE is the truth and ATTITUDE is the clock; one alone is not a pose.

    The reply is evidence, not a gate. The first version aborted the run when
    the acknowledgement did not arrive -- and the request had gone out as a
    broadcast the aircraft was never addressed by, so a healthy aircraft was
    thrown away before it flew. What the streams actually delivered is what
    decides, and `pose_certification` is where that verdict lands.
    """
    source = inspect.getsource(harness.run)
    assert "request_truth_pose_streams(" in source
    assert 'result["stream_requests"]' in source
    assert "options.sysid" in source.split("request_truth_pose_streams(")[1][:80], (
        "addressed by sysid, not by an unset master.target_system"
    )
    # The certification verdict still reaches the errors list.
    assert "certification=result.get(\"pose_certification\")" in source


class _Pose:
    """Just the fields resolve_poi reads."""

    lat_deg = 40.3117414
    lon_deg = 44.4552111
    alt_m = 1694.86
    yaw_deg = 30.0


def _options(**overrides):
    base = dict(poi_lat=None, poi_lon=None, poi_alt=None,
                poi_range_m=None, poi_off_boresight_deg=0.0,
                poi_below_m=None)
    base.update(overrides)
    return argparse.Namespace(**base)


@pytest.mark.parametrize("overrides,fragment", [
    ({}, "no POI"),
    ({"poi_lat": 40.0}, "needs all of"),
    ({"poi_range_m": 2000.0}, "needs all of"),
    ({"poi_lat": 40.0, "poi_lon": 44.0, "poi_alt": 1200.0,
      "poi_range_m": 2000.0, "poi_below_m": 400.0}, "not both"),
])
def test_a_half_stated_poi_is_refused(overrides, fragment):
    """Silently defaulting half a POI would fly a geometry nobody chose."""
    with pytest.raises(SystemExit) as raised:
        harness.check_poi_options(_options(**overrides))
    assert fragment in str(raised.value)


def test_absolute_coordinates_pass_straight_through():
    assert harness.resolve_poi(
        _options(poi_lat=40.5, poi_lon=44.5, poi_alt=1200.0), _Pose(),
    ) == (40.5, 44.5, 1200.0)


def test_placement_lands_at_the_range_and_bearing_asked_for():
    """Resolved coordinates must reproduce the requested geometry exactly."""
    from scripts.navigation_truth_sensor import poi_offset_ned_m
    pose = _Pose()
    poi = harness.resolve_poi(
        _options(poi_range_m=2000.0, poi_below_m=400.0,
                 poi_off_boresight_deg=15.0), pose,
    )
    offset = poi_offset_ned_m(
        lat_deg=pose.lat_deg, lon_deg=pose.lon_deg, alt_m=pose.alt_m,
        poi_lat_deg=poi[0], poi_lon_deg=poi[1],
        poi_alt_m=poi[2],
    )
    assert float(np.linalg.norm(offset)) == pytest.approx(2000.0, abs=1e-6)
    assert offset[2] == pytest.approx(400.0), "below, so down is positive"
    # Bearing measured from north, which is where the nose plus the offset sits.
    bearing = math.degrees(math.atan2(offset[1], offset[0]))
    assert bearing == pytest.approx(pose.yaw_deg + 15.0, abs=1e-6)


def test_placement_is_relative_to_the_nose_not_to_north():
    """Two aircraft that finished their climbs pointing differently must get
    the same scoring interval, which is the entire reason placement exists."""
    from scripts.navigation_truth_sensor import poi_offset_ned_m
    options = _options(poi_range_m=2000.0, poi_below_m=400.0)
    bodies = []
    for yaw in (0.0, 137.0, 300.0):
        pose = _Pose()
        pose.yaw_deg = yaw
        poi = harness.resolve_poi(options, pose)
        offset = poi_offset_ned_m(
            lat_deg=pose.lat_deg, lon_deg=pose.lon_deg, alt_m=pose.alt_m,
            poi_lat_deg=poi[0], poi_lon_deg=poi[1],
            poi_alt_m=poi[2],
        )
        bodies.append(harness.build_frame(
            offset_ned_m=offset, truth_pitch_deg=0.0, truth_roll_deg=0.0,
            truth_yaw_deg=yaw, airspeed_mps=30.0, yaw_rate_rad_s=0.0,
            source_timestamp_s=0.0,
        ).body_ray)
    for ray in bodies[1:]:
        assert np.asarray(ray) == pytest.approx(np.asarray(bodies[0]), abs=1e-9)


@pytest.mark.parametrize("measured,requested,expected", [
    (1.0, 1.0, 0),
    (1.15, 1.0, 0),      # inside the tolerance: scheduler jitter
    (10.0, 1.0, 1),      # the whole-multiple error this gate exists for
    (1.0, 10.0, 1),
    (None, 1.0, 0),      # never measured, so nothing to claim
])
def test_the_clock_rate_is_checked_against_what_was_requested(
    measured, requested, expected
):
    """A run at the wrong rate is not a slower run, it is a different question."""
    errors = harness.classification_errors(
        scoring_end="passed", miss_m=0.5, certification=None,
        measured_speedup=measured, requested_speedup=requested,
    )
    assert len(errors) == expected, errors


@pytest.mark.parametrize("range_m,below_m,fragment", [
    (100.0, 500.0, "cannot equal or exceed"),
    (500.0, 500.0, "cannot equal or exceed"),
    (500.0, -500.0, "cannot equal or exceed"),
    (0.0, 10.0, "must be positive"),
    (-2000.0, 400.0, "must be positive"),
])
def test_an_impossible_placement_is_refused_not_reshaped(range_m, below_m,
                                                         fragment):
    """`below` is a leg of the triangle whose hypotenuse is `range`.

    Asked for 100 m of range 500 m below, an earlier version clamped the
    horizontal distance to zero and resolved a POI 500 m away -- five times
    the range requested, with nothing in the result saying so.
    """
    with pytest.raises(SystemExit) as raised:
        harness.check_poi_options(
            _options(poi_range_m=range_m, poi_below_m=below_m))
    assert fragment in str(raised.value)


def test_resolve_poi_refuses_the_same_geometry_without_the_cli():
    """A caller that skipped validation must not get a quietly wrong POI."""
    with pytest.raises(ValueError):
        harness.resolve_poi(
            _options(poi_range_m=100.0, poi_below_m=500.0), _Pose())


def test_every_parameter_set_is_confirmed_before_the_run_proceeds():
    """A rejected SIM_WIND_SPD leaves the aircraft in still air.

    Filing a calm-air miss as a windy one is worse than having no windy data at
    all, so an unconfirmed parameter ends the run instead of flavouring it.
    """
    source = inspect.getsource(harness.run)
    assert 'result["parameters_set"] = settings' in source
    assert "parameter not confirmed" in source
    tree = ast.parse(textwrap.dedent(source))
    discarded = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call)
        and ast.unparse(node.value.func) == "_set_param"
    ]
    assert not discarded, "every _set_param result is kept and checked"


def test_a_silent_link_is_named_and_not_left_to_the_hung_run_guard():
    """The deadline must be reachable when NO message arrives at all.

    Put inside the `pose is None` branch it was only reached when a message had
    arrived, so a link that went completely silent -- the one case it most
    needed to name -- fell through to the hung-run guard and reported "stalled"
    fifteen minutes later.
    """
    tree = ast.parse(textwrap.dedent(inspect.getsource(harness.run_navigation_episode)))
    loop = next(node for node in ast.walk(tree) if isinstance(node, ast.While))
    # Walk the loop's own statements, in order, until the first `continue`.
    before_any_continue = []
    for statement in loop.body:
        before_any_continue.append(statement)
        if any(isinstance(inner, ast.Continue)
               for inner in ast.walk(statement)):
            break
    guarded = [
        node for statement in before_any_continue
        for node in ast.walk(statement)
        if isinstance(node, ast.If)
        and "first_pose_deadline_s" in _dotted(node.test)
    ]
    assert guarded, (
        "the first-pose deadline must be checked before anything can skip past "
        "it, or a silent link never reaches it"
    )


def test_the_budget_accounts_for_every_wait_the_run_performs():
    """A phase the budget forgets is a phase the parent kills the child inside.

    Three budgets in a row were short -- once below the child's own hung-run
    guard, once omitting the parameter/stream/mode/arming waits, once before
    the TAKEOFF sequence existed -- and each cost a full run that produced no
    result and no reason. This walks `run()` for calls that wait and requires
    each one's bound to appear in `worst_case_wall_s`.
    """
    # One call can hold more than one wait. `_arm` gained a second when the
    # pre-arm EKF-origin gate went in ahead of its attempts loop, and a map of
    # one bound per call is how that wait went unbudgeted.
    waits = {
        "_connect": ("connect_timeout",),
        "_wait_ready": ("ready_timeout",),
        "_set_param": ("PARAM_SET_TIMEOUT_S",),
        "_mode": ("MODE_TIMEOUT_S",),
        "_arm": ("NAV_SOLUTION_TIMEOUT_S", "arm_attempts"),
        "request_truth_pose_streams": ("STREAM_REQUEST_TIMEOUT_S",),
        "census": ("CENSUS_S",),
        "run_navigation_episode": ("scoring_duration_s",),
    }
    run_source = inspect.getsource(harness.run)
    budget = inspect.getsource(harness.worst_case_wall_s)
    for call, bounds in waits.items():
        assert f"{call}(" in run_source, f"{call} is no longer called by run()"
        for bound in bounds:
            assert bound in budget, (
                f"run() waits in {call}() but worst_case_wall_s does not "
                f"account for {bound}"
            )
    # Every Flight.phase() call is a wall guard of its own, not the
    # aircraft-seconds it is asked for. There are three: the TAKEOFF climb-out,
    # the GUIDED climb, and the optional level-off -- optional in the FLIGHT,
    # never in the BUDGET, because the budget is asked on the same options the
    # child flies and a level-off of 0 still costs its guard floor.
    tree = ast.parse(textwrap.dedent(run_source))
    phases = [node for node in ast.walk(tree)
              if isinstance(node, ast.Call)
              and ast.unparse(node.func).endswith(".phase")]
    assert len(phases) == 3, "settle, climb, then level-off"
    assert budget.count("_phase_guard_s(") == len(phases)
    for bound in ("settle_s", "CLIMB_LIMIT_S", "level_settle_s"):
        assert bound in budget


def test_the_aircraft_is_launched_before_it_is_asked_to_fly():
    """ArduPlane has no launch in GUIDED.

    An earlier version armed straight into GUIDED on the runway. The aircraft
    sat still for the whole climb limit, then the scoring interval began at ground
    level with the POI placed below the terrain -- and every signal along the
    way looked healthy: armed, commanded, telemetry flowing. The order is the
    plant harness's (scratch_sitl_uav.py:572-594) and it is load-bearing.
    """
    source = inspect.getsource(harness.run)
    order = [source.index(needle) for needle in (
        '"TAKEOFF"', '"ARMING_CHECK"', "_arm(", "flight.phase(", '"GUIDED"',
    )]
    assert order == sorted(order), (
        "TAKEOFF, arming checks off, arm, climb out, THEN GUIDED"
    )
    # ARMING_CHECK is confirmed, not fired and forgotten: an unconfirmed set
    # leaves the aircraft arming under checks that were never lifted, which is
    # one of the few states that produces exactly "armed but never climbed".
    assert 'if not _set_param(link, options.sysid, "ARMING_CHECK", 0.0):' in source


@pytest.mark.parametrize("speedup,wind", [(1.0, None), (1.0, 8.0), (10.0, None)])
def test_the_budget_exceeds_every_guard_it_covers(speedup, wind):
    """Sanity on the sum itself: longer than the guards it is made of."""
    options = argparse.Namespace(
        connect_timeout=90.0, ready_timeout=180.0, arm_attempts=20,
        arm_timeout=6.0, scoring_duration_s=180.0, settle_s=20.0, level_settle_s=0.0,
        speedup=speedup, wind_speed=wind,
    )
    budget = harness.worst_case_wall_s(options)
    phases = (harness._phase_guard_s(options.settle_s, speedup)
              + harness._phase_guard_s(harness.CLIMB_LIMIT_S, speedup))
    scoring_guard = max(600.0, 5.0 * 180.0 / speedup)
    assert budget > phases + scoring_guard
    assert budget > options.connect_timeout + options.ready_timeout


def test_wind_lengthens_the_budget_because_it_adds_parameter_waits():
    base = dict(connect_timeout=90.0, ready_timeout=180.0, arm_attempts=20,
                arm_timeout=6.0, scoring_duration_s=180.0, settle_s=20.0,
                level_settle_s=0.0, speedup=1.0)
    calm = harness.worst_case_wall_s(argparse.Namespace(**base, wind_speed=None))
    windy = harness.worst_case_wall_s(argparse.Namespace(**base, wind_speed=8.0))
    assert windy > calm, "two more confirmed parameter sets"


def test_jitter_is_an_rms_and_refuses_a_negative():
    """A negative RMS is a typo, and `gauss` would not notice it.

    `random.gauss(0, -2)` returns the same distribution as `gauss(0, 2)`, so a
    sign slip would produce a run recorded as one arm while flying another.
    """
    base = [
        "--connection", "udp:0", "--sysid", "1",
        "--poi-range-m", "3000", "--poi-below-m", "350",
    ]
    options = harness._parser().parse_args(base + ["--estimate-jitter-deg", "2.0"])
    harness.check_poi_options(options)
    assert options.estimate_jitter_deg == pytest.approx(2.0)

    with pytest.raises(SystemExit) as refused:
        harness.check_poi_options(
            harness._parser().parse_args(base + ["--estimate-jitter-deg=-2.0"])
        )
    assert "must not be negative" in str(refused.value)


def test_jitter_and_delay_compose_rather_than_replace():
    """The two knobs answer different questions and must be independent.

    Delay shifts the de-rotating attitude in time; jitter decorrelates it
    without shifting it. Comparing them is the whole point, so one must not
    silently disable the other.

    Order matters as much as independence: jitter is applied AFTER the timing
    selection, so a delayed arm does not smooth the noise it was given.
    """
    tree = _command_path_tree()
    jitter_guards = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.If)
        and "estimate_jitter_deg" in ast.unparse(node.test)
    ]
    assert len(jitter_guards) == 1
    applied = ast.unparse(jitter_guards[0])
    assert "est_pitch_deg +=" in applied and "est_roll_deg +=" in applied, (
        "jitter must ADD to the selected estimate, not replace it"
    )


def test_an_unbracketed_lag_consumes_its_command_instant_like_any_missing_input():
    """A skipped pose must not silently defer the schedule.

    Found in review. The lag path first used a bare `continue`, which jumped
    over the scheduler entirely, and the damage was invisible three ways at
    once: `held_commands` read zero although command instants went unserved, the
    certification called the run clean, and the commands that followed landed
    25 ms apart instead of the 50 ms the run declares -- FASTER than its own
    stated cadence, in exactly the phase-sensitive window this bench measures.

    Pinned structurally: the de-rotation result must reach the frame-building
    chain that already handles a missing input, and must NOT appear in a
    statement that leaves the loop early.
    """
    tree = _command_path_tree()

    # One exception, and only one: WARM-UP. Before `started_t_s` is set there
    # is no scoring interval, no POI and no schedule, so skipping an unbracketable
    # pose there delays the start rather than omitting a command -- and it is
    # the fix for the onset-parity defect, where lagged arms began their
    # scoring interval on poses they could not command from. The guard therefore
    # allows a de_rotation continue ONLY inside the `started_t_s is None`
    # branch, and still refuses one anywhere in the running scoring interval.
    entry_guards = {
        id(inner)
        for node in ast.walk(tree)
        if isinstance(node, ast.If)
        and "started_t_s is None" in ast.unparse(node.test)
        for inner in ast.walk(node)
    }
    early_exits = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.If)
        and "de_rotation" in ast.unparse(node.test)
        and any(isinstance(inner, (ast.Continue, ast.Break))
                for inner in ast.walk(node))
        and id(node) not in entry_guards
    ]
    assert not early_exits, (
        "an unbracketed lag must not skip a pose during scoring: the due "
        "command instant would be deferred instead of consumed, and nothing "
        "would record it"
    )

    # It must gate the FRAME, which is how the existing missing-input contract
    # reaches `held_commands` (see the scheduler's `latest_frame_t_s` branch).
    frame_guards = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.If)
        and "de_rotation is not None" in ast.unparse(node.test)
        and any(isinstance(inner, ast.Call)
                and ast.unparse(inner.func) == "build_frame"
                for inner in ast.walk(node))
    ]
    assert frame_guards, (
        "the de-rotation result must gate frame construction, so an "
        "unbracketed pose falls through to the held-command accounting"
    )


def test_the_command_schedule_starts_from_the_first_commandable_pose():
    """Not from the first pose, or a lagged arm reports a phantom omission.

    A lag cannot be bracketed until the attitude history spans it, so a schedule
    anchored to the first pose has instants falling due before any frame exists.
    Reporting those as held commands is correct and was the first fix; leaving
    them to happen on EVERY lagged run is not, because a real omission would
    then be indistinguishable from the warm-up.
    """
    tree = _command_path_tree()
    starts = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(ast.unparse(target) == "next_command_t_s"
                for target in node.targets)
    ]
    assert starts, "the schedule must be anchored somewhere"
    # Anchored to a pose ONLY where a frame was just built, never unconditionally
    # at scoring interval entry.
    anchored_at_entry = [
        node for node in starts
        if ast.unparse(node.value) == "pose.t_s"
    ]
    assert not anchored_at_entry, (
        "anchoring the cadence at the first pose makes every lagged run report "
        "a warm-up omission, which would hide a real one"
    )


def test_every_arm_begins_its_scoring_interval_on_a_commandable_pose():
    """Onset parity: the scoring interval clock must not start before the law can act.

    Found in review, third defect in one feature. With the schedule anchored to
    the first commandable pose but the SCORING INTERVAL anchored to the first pose,
    a lagged arm started its clock, fixed its POI and began scoring while
    still transmitting the zero attitude initialised at entry -- flying
    uncommanded for a lag-length interval that the certification then called
    clean. The zero-lag arms had no such interval, so the two no longer shared
    onset conditions and the comparison measured entry state as well as lag.

    Pinned: scoring interval start must be gated on the de-rotation existing.
    """
    tree = _command_path_tree()
    starts = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.If)
        and "started_t_s is None" in ast.unparse(node.test)
    ]
    assert starts, "the scoring interval must begin somewhere"
    for node in starts:
        body_source = "".join(ast.unparse(statement) for statement in node.body)
        if "started_t_s = pose.t_s" not in body_source:
            continue
        assert "de_rotation is None" in body_source, (
            "the scoring clock starts without proof the pose is "
            "commandable: a lagged arm would fly its first lag-length "
            "interval uncommanded, scored, and certified clean"
        )
        return
    raise AssertionError("no scoring-start assignment found")


def _straight_run(bearing_rad: float, count: int = 40):
    """A constant-bearing course: range closing to nothing."""
    final_approach = harness.FinalApproachGeometry()
    for step in range(count):
        range_m = 1000.0 * (1.0 - step / count)
        final_approach.observe_geometry(
            range_m,
            np.array([range_m * math.cos(bearing_rad),
                      range_m * math.sin(bearing_rad),
                      0.0]),
        )
    return final_approach


def test_the_window_is_gated_on_range_not_on_sample_count():
    """Only the closing quarter counts, whatever the run's duration."""
    final_approach = harness.FinalApproachGeometry(range_fraction=0.25)
    for range_m in (1000.0, 900.0, 500.0, 260.0, 240.0, 100.0, 10.0):
        final_approach.observe_geometry(range_m, np.array([range_m, 0.0, 0.0]))
    summary = final_approach.summary()
    assert summary["range_gate_m"] == pytest.approx(250.0)
    # 240, 100 and 10 are inside the gate; the four wider ranges are not.
    assert summary["samples"] == 3


def test_a_command_outside_the_final_approach_window_is_not_counted():
    """A dither during the turn-in must not be charged to the final-approach run."""
    final_approach = harness.FinalApproachGeometry(range_fraction=0.25)
    final_approach.observe_geometry(1000.0, np.array([1000.0, 0.0, 0.0]))
    for _ in range(10):
        final_approach.observe_command(20.0, -20.0, _Rate(0.0, 0.0))
    assert final_approach.summary()["roll_commanded"]["magnitude"] is None
    final_approach.observe_geometry(100.0, np.array([100.0, 0.0, 0.0]))
    final_approach.observe_command(3.0, 3.0, _Rate(0.0, 0.0))
    assert final_approach.summary()["roll_commanded"]["magnitude"]["samples"] == 1


def test_a_dithering_command_is_told_from_the_roll_the_airframe_flew():
    """The defect this exists to catch: command reversing, airframe steady.

    A whole-run median reports both as a ~4 degree bank. Only the step size
    and the reversal count separate a command the airframe is filtering from
    one it is following.
    """
    final_approach = harness.FinalApproachGeometry(range_fraction=1.0)
    final_approach.observe_geometry(100.0, np.array([100.0, 0.0, 0.0]))
    for step in range(40):
        final_approach.observe_command(
            4.0 if step % 2 else -4.0, 4.0, _Rate(0.0, 0.0))
    summary = final_approach.summary()
    assert summary["roll_commanded"]["magnitude"]["median_deg"] == (
        pytest.approx(4.0))
    assert summary["roll_actual"]["magnitude"]["median_deg"] == (
        pytest.approx(4.0))
    # The separator: the command jumps 8 degrees a sample, the airframe 0.
    assert summary["roll_commanded"]["step"]["median_deg"] == (
        pytest.approx(8.0))
    assert summary["roll_actual"]["step"]["median_deg"] == pytest.approx(0.0)
    assert summary["roll_commanded"]["reversals"] == 39
    assert summary["roll_actual"]["reversals"] == 0


def test_the_scoring_interval_records_its_final_approach_geometry():
    """Wired into the run, not merely importable."""
    source = inspect.getsource(harness.run_navigation_episode)
    assert "roll.observe_geometry(range_m, offset)" in source
    assert "roll.observe_command(" in source
    record = inspect.getsource(harness.RollRecord)
    assert "self.final_approach.observe_geometry(" in record
    assert "self.final_approach.observe_command(" in record
    assert '"terminal_geometry": self.final_approach.summary()' in record


def test_a_steady_rate_and_a_reversing_rate_are_told_apart():
    """Magnitude alone cannot see the defect; the step is what separates them.

    Both series below have the same magnitude. One is a steady turn rate, the
    other reverses every sample -- and the roll command built from them is a
    held bank in one case and a dither in the other.
    """
    steady = harness.FinalApproachGeometry(range_fraction=1.0)
    shaky = harness.FinalApproachGeometry(range_fraction=1.0)
    for final_approach in (steady, shaky):
        final_approach.observe_geometry(100.0, np.array([100.0, 0.0, 0.0]))
    rate = math.radians(0.4)
    for step in range(20):
        steady.observe_command(0.0, 0.0, _Rate(rate, rate))
        shaky.observe_command(0.0, 0.0, _Rate(
            rate if step % 2 else -rate, rate if step % 2 else -rate))
    a = steady.summary()["lateral_rate_inertial_deg_s"]
    b = shaky.summary()["lateral_rate_inertial_deg_s"]
    assert a["magnitude"]["median_deg"] == pytest.approx(
        b["magnitude"]["median_deg"])
    assert a["step"]["median_deg"] == pytest.approx(0.0)
    assert b["step"]["median_deg"] == pytest.approx(0.8)


def test_the_two_halves_of_the_lateral_rate_are_reported_separately():
    """Visual differentiation and gyro yaw rate have different fixes.

    Reporting only their sum would leave the command dither attributable to
    either one, which is the whole question these two series exist to settle.
    """
    final_approach = harness.FinalApproachGeometry(range_fraction=1.0)
    final_approach.observe_geometry(100.0, np.array([100.0, 0.0, 0.0]))
    for step in range(20):
        # Visual term perfectly smooth, inertial term shaking.
        final_approach.observe_command(0.0, 0.0, _Rate(
            math.radians(0.1), math.radians(0.1 if step % 2 else -0.5)))
    summary = final_approach.summary()
    assert summary["lateral_rate_visual_deg_s"]["step"]["median_deg"] == (
        pytest.approx(0.0))
    assert summary["lateral_rate_inertial_deg_s"]["step"]["median_deg"] > 0.5


def test_a_rate_outside_the_final_approach_window_is_not_counted():
    final_approach = harness.FinalApproachGeometry(range_fraction=0.25)
    final_approach.observe_geometry(1000.0, np.array([1000.0, 0.0, 0.0]))
    final_approach.observe_command(0.0, 0.0, _Rate(1.0, 1.0))
    assert final_approach.summary()["lateral_rate_visual_deg_s"] is None


def test_the_scoring_interval_records_the_rate_the_roll_command_was_built_from():
    assert "plan.lateral_rate" in inspect.getsource(harness.run_navigation_episode)
    built = inspect.getsource(harness.FinalApproachGeometry.observe_command)
    assert "rate.visual_rate_rad_s" in built
    assert "rate.raw_inertial_rate_rad_s" in built


def test_leaving_the_final_approach_window_stops_the_command_and_rate_series():
    """The window has to close again, because range is not monotonic.

    Once the POI is passed the range grows back, and an scoring interval that
    wanders can cross the gate more than once. A gate that only ever opened
    would keep charging commands and rates to a window the aircraft had left,
    while the geometry series correctly stopped -- so the two series would
    describe different stretches of the same run and nothing in the artifact
    would say so.
    """
    final_approach = harness.FinalApproachGeometry(range_fraction=0.25)
    final_approach.observe_geometry(1000.0, np.array([1000.0, 0.0, 0.0]))
    final_approach.observe_command(1.0, 1.0, _Rate(0.1, 0.1))

    final_approach.observe_geometry(100.0, np.array([100.0, 0.0, 0.0]))
    final_approach.observe_command(2.0, 2.0, _Rate(0.2, 0.2))

    # Back outside the gate: passed the POI, or wandered wide.
    final_approach.observe_geometry(900.0, np.array([900.0, 0.0, 0.0]))
    final_approach.observe_command(30.0, 30.0, _Rate(9.9, 9.9))

    summary = final_approach.summary()
    assert summary["samples"] == 1
    assert summary["roll_commanded"]["magnitude"]["samples"] == 1
    assert summary["roll_commanded"]["magnitude"]["max_deg"] == (
        pytest.approx(2.0))
    assert summary["lateral_rate_visual_deg_s"]["magnitude"]["samples"] == 1


def test_the_command_and_geometry_series_cover_the_same_stretch():
    """Whatever the path does, neither series may outlast the other's window."""
    final_approach = harness.FinalApproachGeometry(range_fraction=0.5)
    for range_m in (800.0, 700.0, 300.0, 200.0, 600.0, 900.0, 150.0):
        final_approach.observe_geometry(range_m, np.array([range_m, 0.0, 0.0]))
        final_approach.observe_command(1.0, 1.0, _Rate(0.1, 0.1))
    summary = final_approach.summary()
    # 300, 200 and 150 are inside a 400 m gate; 700, 600 and 900 are not.
    assert summary["samples"] == 3
    assert summary["roll_commanded"]["magnitude"]["samples"] == 3


def _pass_through(miss_m: float, final_approach=None):
    """Fly a dead-straight line past a POI offset by `miss_m`.

    The path is perfectly straight, so every straightness measure must read
    zero -- including across the closest approach, where the bearing to the
    POI swings through half a turn no matter how straight the flying was.
    """
    final_approach = final_approach or harness.FinalApproachGeometry(range_fraction=1.0)
    for step in range(60):
        along = 200.0 - 5.0 * step  # closes, reaches CPA, then flies away
        final_approach.observe_geometry(math.hypot(along, miss_m),
                                  np.array([along, miss_m, 0.0]))
    return final_approach


def test_the_closest_approach_is_read_from_the_ranges_not_assumed():
    """No threshold decides the boundary -- the range series does."""
    final_approach = harness.FinalApproachGeometry(range_fraction=1.0)
    for range_m in (100.0, 60.0, 20.0, 5.0, 40.0, 90.0):
        final_approach.observe_geometry(range_m, np.array([range_m, 1.0, 0.0]))
    assert final_approach.summary()["samples"] == 6
    assert final_approach.summary()["closing_samples"] == 4


class _Rate:
    """The two rate terms a roll command is built from, for the tests."""

    def __init__(self, visual_rad_s: float, inertial_rad_s: float) -> None:
        self.visual_rate_rad_s = visual_rad_s
        self.raw_inertial_rate_rad_s = inertial_rad_s

def test_a_straight_run_past_the_poi_shows_no_curvature():
    """The case that broke the first straightness measure.

    The path is dead straight; only the geometry of passing a POI makes the
    bearing sweep. Measured in metres, across the closest approach and out the
    far side, a straight run has to read zero.
    """
    final_approach = harness.FinalApproachGeometry(range_fraction=1.0)
    for step in range(60):
        along = 200.0 - 5.0 * step  # closes, passes, flies away
        final_approach.observe_geometry(math.hypot(along, 0.02),
                                  np.array([along, 0.02, 0.0]))
    summary = final_approach.summary()
    assert summary["closing_samples"] < summary["samples"]
    assert summary["chord_deviation_m"] < 0.01


def test_a_sustained_bank_is_reported_as_the_metres_it_bends():
    """A curving path must read as its own sagitta, not as a small angle.

    A circular arc of radius R subtending theta departs from its chord by
    R * (1 - cos(theta / 2)). This is the number that made the real matrix
    readable: 30-77 m of bend over a 750 m window is a held bank, not noise.
    """
    radius_m, span_rad = 1250.0, 0.6
    final_approach = harness.FinalApproachGeometry(range_fraction=1.0)
    for step in range(61):
        angle = span_rad * step / 60
        # Aircraft on the arc; POI at the origin of the offset.
        x = radius_m * math.sin(angle)
        y = radius_m * (1.0 - math.cos(angle))
        range_m = 800.0 - 10.0 * step
        final_approach.observe_geometry(range_m, np.array([-x, -y, 0.0]))
    expected = radius_m * (1.0 - math.cos(span_rad / 2))
    assert final_approach.summary()["chord_deviation_m"] == pytest.approx(
        expected, rel=0.05)


def test_the_fly_away_leg_cannot_hide_a_curve_flown_before_it():
    """Truncating at the closest approach must not discard the answer."""
    final_approach = harness.FinalApproachGeometry(range_fraction=1.0)
    for step in range(30):
        range_m = 200.0 - 5.0 * step
        bearing = math.radians(2.0 * step)
        final_approach.observe_geometry(
            range_m,
            np.array([range_m * math.cos(bearing),
                      range_m * math.sin(bearing), 0.0]))
    for step in range(10):  # fly-away, every sample wider than the last
        final_approach.observe_geometry(60.0 + 5.0 * step,
                                  np.array([-(60.0 + 5.0 * step), 0.0, 0.0]))
    summary = final_approach.summary()
    assert summary["samples"] == 40
    assert summary["closing_samples"] == 30
    assert summary["chord_deviation_m"] > 1.0


def test_the_closest_approach_is_read_from_the_ranges_not_assumed():
    """No threshold decides the boundary -- the range series does."""
    final_approach = harness.FinalApproachGeometry(range_fraction=1.0)
    for range_m in (100.0, 60.0, 20.0, 5.0, 40.0, 90.0):
        final_approach.observe_geometry(range_m, np.array([range_m, 1.0, 0.0]))
    summary = final_approach.summary()
    assert summary["samples"] == 6
    assert summary["closing_samples"] == 4


def test_a_window_never_entered_reports_no_straightness_rather_than_zero():
    """Zero is the reading a DEAD-STRAIGHT run gives, so it cannot mean absent.

    A pass wide enough never to reach the range gate is exactly the case where
    straightness is most worth knowing, and reporting 0.0 for it publishes the
    best possible result for missing data. Nothing downstream contradicts it:
    a wide pass is still a valid miss, so it raises no classification error.
    """
    final_approach = harness.FinalApproachGeometry()
    for range_m in (3000.0, 2000.0, 1200.0, 900.0, 1500.0):
        final_approach.observe_geometry(range_m, np.array([range_m, 0.0, 0.0]))
    summary = final_approach.summary()
    assert summary["samples"] == 0
    assert summary["chord_deviation_m"] is None
    assert not harness.classification_errors(
        scoring_end="passed", miss_m=900.0, certification=None), (
        "a wide pass raises no error, which is why the None matters"
    )


def test_a_window_with_no_interior_point_reports_no_straightness():
    """Two points define the chord and leave nothing to be off it."""
    final_approach = harness.FinalApproachGeometry(range_fraction=1.0)
    final_approach.observe_geometry(100.0, np.array([100.0, 0.0, 0.0]))
    final_approach.observe_geometry(50.0, np.array([50.0, 0.0, 0.0]))
    assert final_approach.summary()["chord_deviation_m"] is None


def test_a_window_whose_ends_coincide_reports_no_straightness():
    """A chord of zero length is no line, so nothing can be measured off it."""
    final_approach = harness.FinalApproachGeometry(range_fraction=1.0)
    for offset in ((10.0, 0.0), (5.0, 5.0), (10.0, 0.0)):
        final_approach.observe_geometry(math.hypot(*offset),
                                  np.array([offset[0], offset[1], 0.0]))
    assert final_approach.summary()["chord_deviation_m"] is None


def test_a_measured_bend_is_still_a_number():
    """The None must not swallow the real readings."""
    final_approach = harness.FinalApproachGeometry(range_fraction=1.0)
    for step in range(20):
        final_approach.observe_geometry(
            200.0 - 5.0 * step,
            np.array([200.0 - 5.0 * step, 4.0 * math.sin(step / 6.0), 0.0]))
    assert final_approach.summary()["chord_deviation_m"] > 0.0
