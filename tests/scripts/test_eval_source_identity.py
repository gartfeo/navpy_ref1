"""A sweep must not be able to straddle an edit to code the child executes.

The identity set used to be written down by hand, and it went stale exactly the
way a hand-maintained list does: the flight child grew an import of
`pixel_pn_terminal_speed`, which decides whether the scored leg runs at the
requested clock, and that module reads its acceptance tolerance from
`swarm_run_verification_model`. Neither appeared in the list, so editing either
changed how a case flew while its recorded SHA stayed put -- and the whole
point of the SHA is to make that impossible.

The regression test is therefore not "the list contains X". It is that the hash
MOVES when a transitively-imported module is edited.
"""

from __future__ import annotations

from pathlib import Path

from scripts.eval_source_identity import script_import_closure, source_identity

WORKTREE = Path(__file__).resolve().parents[2]
SCRIPTS = WORKTREE / "scripts"
CHILD = SCRIPTS / "direct_pixel_pn_child.py"


def test_extracted_evaluator_and_launcher_helpers_are_hashed() -> None:
    from scripts.eval_fleet_setup import FLEET_IDENTITY_ROOTS

    names = {path.name for path in FLEET_IDENTITY_ROOTS}
    assert {
        "eval_param_float32.py", "eval_direct_pixel_cli.py", "eval_fleet_cli.py",
        "eval_fleet_result.py", "swarm_run_cli.py",
    } <= names


def _write(directory: Path, name: str, body: str) -> Path:
    path = directory / f"{name}.py"
    path.write_text(body, encoding="utf-8")
    return path


def test_the_module_that_sets_the_flight_clock_is_part_of_the_identity() -> None:
    """The exact file the hand-written list missed."""
    names = {path.name for path in script_import_closure(CHILD, SCRIPTS)}

    assert "pixel_pn_terminal_speed.py" in names


def test_the_tolerance_that_accepts_the_clock_is_part_of_the_identity() -> None:
    """Reached only THROUGH the speed module, so it needs the transitive walk.

    `CLOCK_RATE_TOLERANCE` decides whether a clock still running at cruise
    speed is accepted. Widening it mid-sweep would change which cases fly.
    """
    names = {path.name for path in script_import_closure(CHILD, SCRIPTS)}

    assert "swarm_run_verification_model.py" in names


def test_the_child_itself_is_in_its_own_closure() -> None:
    assert CHILD.resolve() in script_import_closure(CHILD, SCRIPTS)


def test_imports_that_are_not_scripts_local_are_left_alone(tmp_path: Path) -> None:
    """navpy is hashed as a tree, and the standard library is not under test."""
    entry = _write(
        tmp_path, "entry", "import time\nfrom navpy.modules.navigation import navigation\n"
    )

    assert script_import_closure(entry, tmp_path) == (entry.resolve(),)


def test_the_walk_follows_imports_through_more_than_one_hop(tmp_path: Path) -> None:
    entry = _write(tmp_path, "entry", "import middle\n")
    middle = _write(tmp_path, "middle", "import leaf\n")
    leaf = _write(tmp_path, "leaf", "x = 1\n")

    closure = script_import_closure(entry, tmp_path)

    assert set(closure) == {entry.resolve(), middle.resolve(), leaf.resolve()}


def test_the_package_qualified_form_resolves_to_the_same_file(tmp_path: Path) -> None:
    """Harness modules are imported both bare and as `scripts.X`."""
    entry = _write(tmp_path, "entry", "from scripts.leaf import thing\n")
    leaf = _write(tmp_path, "leaf", "thing = 1\n")

    assert set(script_import_closure(entry, tmp_path)) == {
        entry.resolve(), leaf.resolve()
    }


def test_modules_that_import_each_other_do_not_hang_the_walk(tmp_path: Path) -> None:
    entry = _write(tmp_path, "entry", "import other\n")
    other = _write(tmp_path, "other", "import entry\n")

    assert set(script_import_closure(entry, tmp_path)) == {
        entry.resolve(), other.resolve()
    }


def test_editing_a_module_two_hops_away_moves_the_hash(tmp_path: Path) -> None:
    """The property the whole mechanism exists for.

    If this passed while the file was edited, a sweep could report one source
    SHA across cases that flew two different laws -- which is what forced the
    20260814-001114 sweep to be discarded.
    """
    entry = _write(tmp_path, "entry", "import middle\n")
    _write(tmp_path, "middle", "import leaf\n")
    leaf = _write(tmp_path, "leaf", "TOLERANCE = 0.25\n")
    roots = script_import_closure(entry, tmp_path)
    before = source_identity(roots, tmp_path)

    leaf.write_text("TOLERANCE = 9.99\n", encoding="utf-8")
    after = source_identity(roots, tmp_path)

    assert before["files"] == after["files"] == 3
    assert before["sha256"] != after["sha256"]


def test_a_directory_root_is_hashed_as_a_tree(tmp_path: Path) -> None:
    """`src/navpy` is passed as a directory, not as a file list."""
    package = tmp_path / "package"
    package.mkdir()
    _write(package, "one", "a = 1\n")
    _write(package, "two", "b = 2\n")

    assert source_identity((package,), tmp_path)["files"] == 2
