"""Guards on the parent harness, where a gap is invisible until a run dies.

The parent and the child declare their options separately. Nothing links them,
so an option added to the child and forgotten in the parent produces a launcher
that rejects the very experiment it was extended for -- and it does so at
argparse, before any aircraft starts, which looks like nothing happening at all.
That is exactly how a nine-run jitter batch was launched, failed instantly, and
would have been read as an empty result.
"""

from __future__ import annotations

import pathlib

import scripts.scratch_navigation_sitl as parent
import scripts.scratch_navigation_uav as child


def _declared(module) -> dict[str, object]:
    return {
        action.dest: action
        for action in module._parser()._actions
        if action.dest != "help"
    }


def test_every_shared_option_reaches_the_child():
    """A parent option the child also declares must actually be forwarded.

    Checked by asking `_child_args` for a NON-DEFAULT value of each shared
    option and looking for it in the emitted argv, because a parent that
    accepts a flag and silently drops it is worse than one that rejects it: the
    run completes, the result records the default, and the experiment looks
    like it ran.
    """
    shared = set(_declared(parent)) & set(_declared(child))
    # Deliberately not forwarded: the parent resolves these itself.
    resolved_by_the_parent = {"out", "speedup", "connection", "sysid", "python"}
    candidates = sorted(shared - resolved_by_the_parent)

    # EVERY option set at once, not one at a time. Several are forwarded only
    # when a companion is also set -- the jitter seed means nothing without a
    # jitter magnitude, the wind direction nothing without a wind speed -- and
    # probing them individually reports those legitimate conditions as failures
    # while telling you nothing about the case that actually broke.
    options = parent._parser().parse_args([])
    probes: dict[str, object] = {}
    for name in candidates:
        action = _declared(parent)[name]
        default = action.default
        if action.type is float or isinstance(default, float):
            probes[name] = (default or 0.0) + 3.5
        elif action.type is int or isinstance(default, int):
            probes[name] = (default or 0) + 7
    for name, probe in probes.items():
        setattr(options, name, probe)

    emitted = parent._child_args(options, 1, pathlib.Path("."))
    missing = [
        name
        for name in probes
        if not any(flag in emitted for flag in _declared(child)[name].option_strings)
    ]
    assert not missing, (
        "parent accepts but never forwards: " + ", ".join(missing)
    )
    parsed = child._parser().parse_args(emitted)
    for name, expected in probes.items():
        if name == "estimate_jitter_seed":
            expected += 1  # Existing per-aircraft seed offset: sysid=1 above.
        assert getattr(parsed, name) == expected, name


def test_the_child_accepts_everything_the_parent_forwards():
    """The converse: nothing is forwarded that the child would reject.

    `--estimate-jitter-deg` existed on the child and not the parent, so the
    launcher exited 2 with "unrecognized arguments". The mirror mistake -- the
    parent sending a flag the child does not know -- fails the same way, one
    process further down, where the error lands in a per-uav log nobody reads.
    """
    options = parent._parser().parse_args([])
    options.estimate_jitter_deg = 2.0
    options.estimate_delay_poses = 3
    options.wind_speed = 7.0
    argv = parent._child_args(options, 1, __import__("pathlib").Path("."))
    parsed = child._parser().parse_args(argv)
    assert parsed.estimate_jitter_deg == 2.0
    assert parsed.estimate_delay_poses == 3


def test_cells_spread_conditions_across_one_launch():
    """One launch, every condition -- the point of the per-aircraft cell.

    Flying conditions in SEPARATE launches was measured at up to 5.5x
    batch-to-batch spread on identical code, which is larger than most
    effects worth measuring. Aircraft in ONE launch share their batch
    conditions, so cells make the comparison paired.
    """
    cells = parent.parse_cells("calm:0:0,cross:8:90,tail:8:180")
    assert [cell.name for cell in cells] == ["calm", "cross", "tail"]
    assert cells[1].wind_speed_mps == 8.0 and cells[1].wind_dir_deg == 90.0

    assignment = parent.assign_cells(cells, list(range(121, 133)))
    names = [assignment[sysid].name for sysid in sorted(assignment)]
    # Cycled, not blocked: replicates interleave so one cell cannot collect
    # whatever ordering effect the launch has.
    assert names[:4] == ["calm", "cross", "tail", "calm"]
    assert names.count("calm") == names.count("cross") == names.count("tail") == 4


def test_a_cell_carries_its_own_geometry_and_throttle():
    """The axes that used to cost one LAUNCH each.

    Off-boresight and throttle were launch-wide, so an angle sweep was one
    launch per angle and a throttle sweep one per setting -- and this bench's
    whole finding is that two launches cannot be compared (5.5x spread on
    identical code). A cell that states either must reach the child, and a
    cell that is silent must leave the launch-wide value standing.
    """
    import pathlib

    options = parent._parser().parse_args([])
    options.target_off_boresight_deg = 30.0
    options.throttle = 0.55

    stated = parent.parse_cells("astern:0:0::::::180:0.7")[0]
    assert stated.off_boresight_deg == 180.0
    assert stated.throttle == 0.7
    emitted = " ".join(
        parent._child_args(options, 1, pathlib.Path("."), stated))
    assert "--target-off-boresight-deg 180.0" in emitted
    assert "--throttle 0.7" in emitted

    silent = parent.parse_cells("calm:0:0")[0]
    emitted = " ".join(
        parent._child_args(options, 1, pathlib.Path("."), silent))
    assert "--target-off-boresight-deg 30.0" in emitted
    assert "--throttle 0.55" in emitted
    # One value per flag: a duplicated flag makes the run depend on argparse
    # precedence rather than on the cell.
    assert emitted.count("--target-off-boresight-deg") == 1
    assert emitted.count("--throttle") == 1


def test_a_cell_overrides_the_launch_wide_wind():
    options = parent._parser().parse_args([])
    options.wind_speed = 3.0
    options.wind_dir = 10.0
    cell = parent.Cell("tail", 8.0, 180.0)
    emitted = " ".join(
        parent._child_args(options, 1, pathlib.Path("."), cell)
    )
    assert "--wind-speed 8.0" in emitted
    assert "--wind-dir 180.0" in emitted


def test_no_cells_keeps_the_launch_wide_wind_and_no_assignment():
    options = parent._parser().parse_args([])
    options.wind_speed = 3.0
    options.wind_dir = 10.0
    emitted = " ".join(parent._child_args(options, 1, pathlib.Path("."), None))
    assert "--wind-speed 3.0" in emitted and "--wind-dir 10.0" in emitted
    assert parent.assign_cells((), [1, 2]) == {1: None, 2: None}


def test_a_malformed_cell_is_rejected_before_any_aircraft_starts():
    # argparse-time failure is the whole reason this guard file exists: a
    # launch that dies later looks like an empty result.
    # 4 to 10 fields are legal (delay, jitter, lag, source, scope, geometry,
    # throttle); an eleventh is not.
    for spec in ("calm", "calm:0", ":0:0",
                 "calm:0:0:0:0:0:attitude:full:30:0.5:extra"):
        try:
            parent.parse_cells(spec)
        except ValueError:
            continue
        raise AssertionError(f"accepted malformed cell spec {spec!r}")


def test_the_timeout_covers_the_windiest_cell_not_a_cell_less_run():
    """A cell adds parameter waits the budget must see.

    Found in review: the budget was derived from a CELL-LESS invocation while
    the children flew cells, so a windy cell's two extra parameter waits were
    unbudgeted -- 2705 s against a legitimate 2725 s. That is the same failure
    mode as the two earlier short budgets: the child dies at the parent's
    deadline, before its own, leaving a run with no result and no reason.
    """
    options = parent._parser().parse_args([])
    options.wind_speed = None

    cell_less = parent._child_timeout_s(options, ())
    with_cells = parent._child_timeout_s(
        options, parent.parse_cells("calm:0:0,cross:8:90")
    )

    # Any cell sets wind explicitly -- a calm cell states 0 m/s rather than
    # omitting the parameter -- so every cell costs the same two parameter
    # waits, and all of them cost more than the cell-less invocation the
    # budget used to be derived from.
    assert with_cells > cell_less
    assert with_cells == parent._child_timeout_s(
        options, parent.parse_cells("calm:0:0")
    )
    # The budget is a MAX over the distinct invocations, because one timeout
    # governs every aircraft in the launch; the structure has to hold even
    # while wind happens to make them equal, or the next per-cell option
    # reintroduces the shortfall.
    assert parent._child_timeout_s.__doc__ is not None
    assert with_cells == max(
        parent._child_timeout_s(options, (cell,))
        for cell in parent.parse_cells("calm:0:0,cross:8:90,tail:8:180")
    )


def test_the_timeout_covers_the_level_settle_phase():
    """A level-off the child actually flies must be in the budget.

    The budget lists every phase the child runs, `written the same way so the
    two cannot disagree`. A level-settle outside that list dies at the parent's
    deadline in exactly the runs that use it -- the level-target cells -- while
    every dive cell, which flies with 0, stays green.
    """
    options = parent._parser().parse_args([])
    options.wind_speed = None

    without = parent._child_timeout_s(options, ())
    options.level_settle_s = 60.0
    with_level = parent._child_timeout_s(options, ())
    assert with_level > without


def test_a_run_records_which_law_source_flew():
    """An A/B arm must be provable from the artifact, not from the intent.

    The child rebuilds sys.path from its OWN location (`scratch_navigation_uav`
    import block), so selecting an alternate source tree by PYTHONPATH alone is
    discarded silently and both arms fly the same code. That produces a true
    difference of ZERO with no error anywhere -- indistinguishable from "the
    change had no effect".

    Checked by BUILDING the artifact and comparing it against the law class the
    process actually imported, rather than by reading the source for a field
    name: a field that exists and reports the wrong tree would pass the second
    check and fail the experiment.
    """
    import inspect as inspect_module

    import scripts.scratch_navigation_arms as arms_module
    import scripts.scratch_navigation_uav as uav
    from navpy.modules.navigation.nav.vision_nav.law import VisionNavLaw

    options = uav._parser().parse_args(
        ["--connection", "udp:127.0.0.1:14550",
         "--target-range-m", "3000", "--target-below-m", "350"])
    artifact = uav.initial_result(options, uav.law_source_path())

    imported = pathlib.Path(
        inspect_module.getfile(VisionNavLaw)).resolve()
    assert artifact["law_source"] == str(imported)
    # CONTENT too. A stale or over-edited copy sits at the expected path and
    # passes a location-only check while inventing an effect that belongs to
    # the accident, not the experiment.
    assert artifact["law_sha256"] == arms_module.file_digest(imported)


def test_the_wrong_law_tree_is_refused_before_takeoff():
    """A source-tree arm must fail LOUD at startup, not silently at scoring.

    The failure being guarded has no crash and no error: a child that imported
    the wrong tree flies correctly and returns a good number for the OTHER
    arm's code, so an A/B reports a true difference of zero -- which reads as
    "the change had no effect". Recording the source after the flight cannot
    prevent that; refusing to fly can.
    """
    import scripts.scratch_navigation_uav as uav

    flew = uav.law_source_path()
    options = uav._parser().parse_args(["--connection", "udp:127.0.0.1:14550"])

    # The tree that WAS imported is accepted.
    options.expect_law_source = pathlib.Path(flew).parents[5]
    uav.check_law_source(options)

    # A different tree is refused, and the message names both sides so the
    # operator can see WHICH tree won rather than guessing.
    options.expect_law_source = pathlib.Path(flew).parents[6] / "no-such-tree"
    try:
        uav.check_law_source(options)
    except SystemExit as refusal:
        assert "law source mismatch" in str(refusal)
        assert str(flew) in str(refusal)
    else:
        raise AssertionError("flew with the wrong navigation law tree")

    # Unset means unchecked: the single-arm runs that do not select a tree must
    # not be forced to declare one.
    options.expect_law_source = None
    uav.check_law_source(options)


def test_the_expected_law_tree_reaches_the_child():
    """Forwarded, or the parent's arm selection never arrives.

    Not covered by `test_every_shared_option_reaches_the_child`: that guard
    probes only numeric options, so a PATH option can be declared on both
    sides, accepted by the parent, and dropped in silence.
    """
    options = parent._parser().parse_args([])
    options.expect_law_source = pathlib.Path("C:/tmp/arm-gain1/src")
    argv = parent._child_args(options, 1, pathlib.Path("."))
    assert "--expect-law-source" in argv
    parsed = child._parser().parse_args(argv)
    assert parsed.expect_law_source == options.expect_law_source

    # And absent when unset, so single-arm runs stay unchanged.
    options.expect_law_source = None
    assert "--expect-law-source" not in parent._child_args(
        options, 1, pathlib.Path("."))


def test_the_law_hash_tracks_content_not_location():
    """Two arms are comparable only if the hash differs between them.

    Raised in review: the startup gate proves the law's PATH
    (`is_relative_to`), which a stale copy satisfies. So the artifact records
    the content as well -- within one arm every aircraft must show the same
    hash, between arms it must differ.
    """
    import scripts.scratch_navigation_arms as arms_module
    import scripts.scratch_navigation_uav as uav

    digest = arms_module.file_digest(uav.law_source_path())
    assert len(digest) == 64
    assert digest == arms_module.file_digest(uav.law_source_path())

    # Same location, different bytes -> different hash. Proven by hashing the
    # real file's bytes with one character changed, rather than trusting that
    # sha256 is sensitive.
    import hashlib

    original = uav.law_source_path().read_bytes()
    assert hashlib.sha256(original).hexdigest() == digest
    assert hashlib.sha256(original + b"\n").hexdigest() != digest


def test_cell_rotation_moves_every_treatment_to_new_aircraft():
    """A repeat launch must be able to break the treatment-to-column pinning.

    Found in review: cells cycle onto the same sequential sysids every launch,
    and the launcher spaces aircraft ~100 m apart -- so each treatment also
    owns a fixed longitude column, and its effect cannot be told apart from
    its position however many times the launch replicates.
    """
    argv = ["--cells", "a:0:0,b:0:0,c:0:0", "--instances", "6"]
    plain = parent._parser().parse_args(argv)
    rotated = parent._parser().parse_args(argv + ["--cell-rotate", "1"])
    assert plain.cell_rotate == 0 and rotated.cell_rotate == 1

    cells = parent.parse_cells(plain.cells)
    shift = rotated.cell_rotate % len(cells)
    turned = cells[shift:] + cells[:shift]
    sysids = list(range(121, 127))
    first = parent.assign_cells(cells, sysids)
    second = parent.assign_cells(turned, sysids)
    # Same treatments, different aircraft: every sysid changes its cell.
    assert {c.name for c in first.values()} == {c.name for c in second.values()}
    assert all(first[s].name != second[s].name for s in sysids)
