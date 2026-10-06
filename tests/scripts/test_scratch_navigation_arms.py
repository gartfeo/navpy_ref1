"""Guards on the two-arm launch, where every failure returns a valid number.

The arm mechanism has no benign failure. A tree that cannot be selected, a
tree that is stale, and an allocation that misses combinations all produce a
complete set of results with no error anywhere -- and the number they produce
is wrong in a direction nobody can see. So each is refused before the launch,
and each refusal is pinned here.
"""

from __future__ import annotations

import pathlib

import pytest

import scripts.scratch_navigation_arms as arms_module
import scripts.scratch_navigation_sitl as parent


def _tree(root: pathlib.Path, law_body: str, extra: str = "") -> pathlib.Path:
    """A minimal tree with the layout the child needs to be selectable."""
    law = root / arms_module.TREATMENT_RELATIVE
    law.parent.mkdir(parents=True, exist_ok=True)
    law.write_text(law_body, encoding="utf-8")
    child = root / arms_module.CHILD_RELATIVE
    child.parent.mkdir(parents=True, exist_ok=True)
    child.write_text("# child\n", encoding="utf-8")
    shared = root / "src" / "navpy" / "shared.py"
    shared.write_text(extra or "# shared\n", encoding="utf-8")
    return root


def test_arms_parse_as_name_and_root():
    arms = arms_module.parse_arms("gain0=C:/a,gain1=C:/b")
    assert [arm.name for arm in arms] == ["gain0", "gain1"]
    assert arms[0].source_path.name == "src"
    assert arms[0].child_script.name == "scratch_navigation_uav.py"
    assert arms_module.parse_arms(None) == ()


@pytest.mark.parametrize("spec", ["gain0", "=C:/a,gain1=C:/b", "gain0=",
                                  "gain0=C:/a,gain0=C:/b", "only=C:/a"])
def test_a_malformed_or_useless_arm_spec_is_refused(spec):
    # Includes a SINGLE arm: it parses fine and compares nothing, so it would
    # spend a whole launch producing one arm's numbers labelled as a comparison.
    with pytest.raises(ValueError):
        arms_module.parse_arms(spec)


def test_a_tree_without_the_child_script_is_refused(tmp_path):
    """The exact bug this mechanism exists to avoid.

    A copy of `src` alone looks complete and is silently ignored: the child
    rebuilds its import roots from its own location, so the ORIGINAL law flies
    under the arm's name and the comparison reports a difference of zero.
    """
    good = _tree(tmp_path / "good", "GAIN = 1\n")
    partial = tmp_path / "partial"
    (partial / arms_module.TREATMENT_RELATIVE).parent.mkdir(parents=True)
    (partial / arms_module.TREATMENT_RELATIVE).write_text("GAIN = 0\n")
    (partial / "src" / "navpy" / "shared.py").write_text("# shared\n")

    arms = (arms_module.Arm("gain1", good), arms_module.Arm("gain0", partial))
    with pytest.raises(ValueError, match="no scripts"):
        arms_module.verify_arm_trees(arms)


def test_identical_law_contents_are_refused(tmp_path):
    """Two arms with the same law is a null experiment by construction."""
    arms = (
        arms_module.Arm("gain1", _tree(tmp_path / "one", "GAIN = 1\n")),
        arms_module.Arm("gain0", _tree(tmp_path / "two", "GAIN = 1\n")),
    )
    with pytest.raises(ValueError, match="identical"):
        arms_module.verify_arm_trees(arms)


def test_a_difference_outside_the_law_is_refused(tmp_path):
    """Otherwise the miss difference has more than one candidate cause.

    A stale copy passes a location check and passes a law-hash check; only
    comparing the REST of the tree can catch it.
    """
    arms = (
        arms_module.Arm("gain1", _tree(tmp_path / "one", "GAIN = 1\n")),
        arms_module.Arm(
            "gain0", _tree(tmp_path / "two", "GAIN = 0\n", extra="# STALE\n")),
    )
    with pytest.raises(ValueError, match="differ outside"):
        arms_module.verify_arm_trees(arms)


def test_a_valid_pair_returns_its_law_hashes(tmp_path):
    arms = (
        arms_module.Arm("gain1", _tree(tmp_path / "one", "GAIN = 1\n")),
        arms_module.Arm("gain0", _tree(tmp_path / "two", "GAIN = 0\n")),
    )
    digests = arms_module.verify_arm_trees(arms)
    assert set(digests) == {"gain1", "gain0"}
    assert digests["gain1"] != digests["gain0"]
    assert all(len(digest) == 64 for digest in digests.values())


def test_arms_cross_with_conditions_instead_of_cycling_beside_them():
    """Independent cycles can miss half the matrix while looking balanced.

    Two arms cycled with period 2 alongside two conditions cycled with period 2
    yields only (arm0, cell0) and (arm1, cell1): both arms flown, both
    conditions flown, and the arm perfectly confounded with the condition.
    """
    arms = arms_module.parse_arms("gain0=C:/a,gain1=C:/b")
    cells = parent.parse_cells("calm:0:0,cross:8:90")
    cases = arms_module.assign_cases(arms, cells, list(range(121, 129)))

    pairs = {(case.arm.name, case.cell.name) for case in cases.values()}
    assert pairs == {
        ("gain0", "calm"), ("gain1", "calm"),
        ("gain0", "cross"), ("gain1", "cross"),
    }
    # Interleaved, not blocked: the arm alternates between adjacent aircraft,
    # which share launch order and start seconds.
    names = [cases[sysid].arm.name for sysid in sorted(cases)]
    assert names[:4] == ["gain0", "gain1", "gain0", "gain1"]
    # Equal replicates per combination, so no combination carries extra weight.
    counts = {}
    for case in cases.values():
        key = (case.arm.name, case.cell.name)
        counts[key] = counts.get(key, 0) + 1
    assert set(counts.values()) == {2}


def test_rotating_the_arms_changes_which_aircraft_flies_which():
    """The repeat launch that separates the arm from the launch position."""
    arms = arms_module.parse_arms("gain0=C:/a,gain1=C:/b")
    cells = parent.parse_cells("calm:0:0")
    sysids = list(range(121, 125))

    first = arms_module.assign_cases(arms, cells, sysids)
    second = arms_module.assign_cases(arms, cells, sysids, rotate=1)
    assert [first[s].arm.name for s in sysids] != [
        second[s].arm.name for s in sysids
    ]
    # Same matrix, different mapping -- not a different experiment.
    assert {c.arm.name for c in first.values()} == {
        c.arm.name for c in second.values()
    }


def test_no_arms_reduces_to_plain_condition_cycling():
    cells = parent.parse_cells("calm:0:0,cross:8:90,tail:8:180")
    cases = arms_module.assign_cases((), cells, list(range(121, 127)))
    assert all(case.arm is None for case in cases.values())
    assert [case.cell.name for case in cases.values()][:4] == [
        "calm", "cross", "tail", "calm"
    ]
    empty = arms_module.assign_cases((), (), [1, 2])
    assert all(case.arm is None and case.cell is None for case in empty.values())


def test_an_aircraft_count_that_cannot_fill_the_matrix_is_refused():
    """An unbalanced tail is a biased experiment, not a smaller one."""
    arms = arms_module.parse_arms("gain0=C:/a,gain1=C:/b")
    cells = parent.parse_cells("calm:0:0,cross:8:90,tail:8:180")
    arms_module.check_balanced(arms, cells, 12)
    with pytest.raises(ValueError, match="multiple of 6"):
        arms_module.check_balanced(arms, cells, 8)
    # BEHAVIOUR CHANGE, found in review: this used to assert that a launch
    # without arms had no constraint at all. That was wrong for the launches
    # actually flown -- the wind matrix has no code arms, so its CELLS are the
    # comparison, and 3 cells across 8 aircraft gives two cells 3 flights and
    # one cell 2. The bias the arms check exists to prevent is the same bias.
    with pytest.raises(ValueError, match="multiple of 3"):
        arms_module.check_balanced((), cells, 8)
    arms_module.check_balanced((), cells, 9)
    # No cells and no arms is a plain single-condition launch: any count.
    arms_module.check_balanced((), (), 7)


def test_the_child_is_launched_from_its_own_arm_tree(tmp_path):
    """Script AND source root, because either alone flies the wrong law."""
    arm = arms_module.Arm("gain1", tmp_path / "arm-gain1")
    assert parent._child_script(arm) == arm.child_script
    assert parent._child_source_path(arm) == arm.source_path
    # Both must come from the SAME tree; a mixed pair is the mirrored mistake.
    assert parent._child_script(arm).parents[1] == parent._child_source_path(
        arm).parent

    # No arm: the worktree, unchanged.
    assert parent._child_script(None).name == "scratch_navigation_uav.py"
    assert parent._child_source_path(None).name == "src"


def test_the_arm_tree_overrides_a_launch_wide_expected_source(tmp_path):
    """Or the launch would refuse the very tree it was told to fly."""
    options = parent._parser().parse_args([])
    options.expect_law_source = pathlib.Path("C:/repos/navpy-worktrees/x/src")
    arm = arms_module.Arm("gain1", tmp_path / "arm-gain1")
    emitted = parent._child_args(options, 1, pathlib.Path("."), None, arm)
    assert "--expect-law-source" in emitted
    assert emitted[emitted.index("--expect-law-source") + 1] == str(
        arm.source_path)


def test_a_cell_can_carry_timing_and_noise_per_aircraft():
    """The whole point: the sweep flies paired instead of one arm per launch.

    Every earlier timing and noise sweep was flown one arm per launch, which is
    the instrument measured at up to 5.5x spread on IDENTICAL code. An 8x timing
    result from that instrument cannot be separated from the batch.
    """
    cells = arms_module.parse_cells("d0:0:0:0:0,d3:0:0:3,n1:0:0:0:1")
    assert [c.delay_poses for c in cells] == [0, 3, 0]
    assert [c.jitter_deg for c in cells] == [0.0, None, 1.0]

    options = parent._parser().parse_args([])
    options.estimate_delay_poses = 7      # launch-wide, must lose to the cell
    options.estimate_jitter_deg = 9.0

    emitted = " ".join(parent._child_args(options, 1, pathlib.Path("."), cells[0]))
    # An explicit ZERO is an arm of the sweep, not silence: it must SUPPRESS the
    # launch-wide value rather than fall through to it.
    assert "--estimate-delay-poses" not in emitted
    assert "--estimate-jitter-deg" not in emitted

    emitted = " ".join(parent._child_args(options, 1, pathlib.Path("."), cells[1]))
    assert "--estimate-delay-poses 3" in emitted
    # Silent on jitter -> the launch-wide value stands.
    assert "--estimate-jitter-deg 9.0" in emitted


def test_noise_replicates_draw_different_noise():
    """Same magnitude, different draw, or the replicates are one experiment."""
    cell = arms_module.parse_cells("n1:0:0:0:1")[0]
    options = parent._parser().parse_args([])
    seeds = []
    for sysid in (121, 122, 123):
        emitted = parent._child_args(options, sysid, pathlib.Path("."), cell)
        seeds.append(emitted[emitted.index("--estimate-jitter-seed") + 1])
    assert len(set(seeds)) == 3, f"replicates share a noise draw: {seeds}"


def test_a_cell_with_too_many_fields_is_refused():
    # 7 fields is the full form; 8 is not, and 2 is short.
    for spec in ("d0:0:0:0:0:0:attitude:full:extra", "d0:0"):
        with pytest.raises(ValueError):
            arms_module.parse_cells(spec)


def test_a_requested_noise_seed_is_honoured_and_still_varies_per_aircraft():
    """Both halves, because dropping either one breaks a different thing.

    Forwarding only the requested seed makes every replicate of a noise cell
    draw the SAME realisation, so their agreement is zero by construction.
    Forwarding only the sysid silently discards a requested seed -- the
    "accepted but never forwarded" failure this directory exists to catch.
    """
    cell = arms_module.parse_cells("n1:0:0:0:1")[0]
    options = parent._parser().parse_args(["--estimate-jitter-seed", "999"])

    def seed_for(sysid):
        emitted = parent._child_args(options, sysid, pathlib.Path("."), cell)
        return int(emitted[emitted.index("--estimate-jitter-seed") + 1])

    seeds = [seed_for(sysid) for sysid in (121, 122, 123)]
    assert len(set(seeds)) == 3, f"replicates share a noise draw: {seeds}"
    assert all(seed >= 999 for seed in seeds), (
        f"requested seed 999 was discarded: {seeds}")

    # A different request moves every aircraft, so the whole launch is
    # reproducible from the one number the operator gave.
    other = parent._parser().parse_args(["--estimate-jitter-seed", "5000"])
    moved = [
        int((lambda e: e[e.index("--estimate-jitter-seed") + 1])(
            parent._child_args(other, sysid, pathlib.Path("."), cell)))
        for sysid in (121, 122, 123)
    ]
    assert all(m > s for m, s in zip(moved, seeds))


def test_a_cell_can_carry_fidelity_and_a_continuous_lag():
    """The 2x2 that separates a pairing fix from a loop-phase effect.

    A pairing correction cannot help a source whose ray and attitude come from
    ONE message. So if the same lag improves the zero-skew source as much as the
    two-message source, the win is not a pairing fix -- and no sweep that varies
    only timing, or only fidelity, can tell those apart.
    """
    cells = arms_module.parse_cells(
        "a0:0:0:0:0:0:attitude,t60:0:0:0:0:0.06:truth")
    assert cells[0].source == "attitude" and cells[0].lag_s == 0.0
    assert cells[1].source == "truth" and cells[1].lag_s == 0.06

    options = parent._parser().parse_args([])
    for cell, expected_source, expected_lag in (
        (cells[0], "attitude", None), (cells[1], "truth", "0.06"),
    ):
        emitted = parent._child_args(options, 1, pathlib.Path("."), cell)
        assert emitted[emitted.index("--estimate-source") + 1] == expected_source
        if expected_lag is None:
            # Zero lag must not be forwarded: it is the control arm, and the
            # child refuses a lag alongside a pose delay.
            assert "--estimate-lag-s" not in emitted
        else:
            assert emitted[emitted.index("--estimate-lag-s") + 1] == expected_lag


def test_a_cell_silent_on_fidelity_keeps_the_launch_wide_source():
    options = parent._parser().parse_args(["--estimate-source", "truth"])
    cell = arms_module.parse_cells("calm:0:0")[0]
    emitted = parent._child_args(options, 1, pathlib.Path("."), cell)
    assert emitted[emitted.index("--estimate-source") + 1] == "truth"


def test_a_cell_can_pick_the_lag_scope():
    """The consistency discriminator needs both scopes in ONE launch.

    `angles` lags only the de-rotating pitch/roll (the mixed-time arm);
    `full` lags the body rates by the same interval (every law input on one
    clock). Whether the timing win survives `full` is what separates a
    mixed-time artifact from genuine loop-phase compensation, and the two
    arms are only comparable when they fly paired.
    """
    cells = arms_module.parse_cells(
        "a60:0:0:0:0:0.06:attitude,f60:0:0:0:0:0.06:attitude:full")
    assert cells[0].lag_scope is None and cells[1].lag_scope == "full"

    options = parent._parser().parse_args([])
    emitted = parent._child_args(options, 1, pathlib.Path("."), cells[1])
    assert emitted[emitted.index("--estimate-lag-scope") + 1] == "full"
    # Silent cell -> the child default (angles) stands, nothing forwarded.
    assert "--estimate-lag-scope" not in parent._child_args(
        options, 1, pathlib.Path("."), cells[0])
