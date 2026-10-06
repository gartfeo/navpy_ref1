"""D7b: the child says which code it flew, and which case it was told it was.

The harness names the case in ``case.json``, beside the result the child
publishes: the harness's own source identity, the case's name and the sha256
of its definition. The child records those AS GIVEN, and one it could not
read as absent, with the reason; it never makes one up. What it measures
itself is the code on disk it imports from, the files of the navpy tree and
of its own scripts-local imports, hashed when it starts and again at
teardown, so an edit still in place at teardown shows as two different
hashes; equal hashes cannot say that nothing changed in between, nor that
the code in memory is those files. The rest describes the run, and nothing
compares it. Nothing here stops the case but an interrupt. With tracing off
none of it is gathered.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from datetime import datetime, timedelta
from pathlib import Path
from types import SimpleNamespace

import pytest

from navpy.modules.vision.sim import determinism_trace
from scripts import pixel_pn_run_identity as run_identity
from scripts.eval_source_identity import script_import_closure, source_identity
from scripts.pixel_pn_case_manifest import MANIFEST_NAME as CASE_MANIFEST

SCRIPTS = Path(run_identity.__file__).resolve().parent
CHILD = SCRIPTS / "direct_pixel_pn_child.py"
HARNESS = {"sha256": "a" * 64, "files": 12}
DEFINITION_SHA256 = "d" * 64


@pytest.fixture
def tracing(monkeypatch: pytest.MonkeyPatch) -> None:
    """Tracing on, as the child's gate reads it: off the module, at call
    time, where the environment was read once at import."""
    monkeypatch.setattr(determinism_trace, "ENABLED", True)


def _options(case_dir: Path) -> argparse.Namespace:
    """The child's options, as far as its identity reads them."""
    return argparse.Namespace(
        connection="udp:127.0.0.1:14999",
        sysid=1,
        timeout=5.0,
        result=case_dir / "result.json",
        scoring_active=case_dir / "scoring_active.marker",
    )


def _case(
    case_dir: Path, *, drop: tuple[str, ...] = (), **fields: object
) -> None:
    """The ``case.json`` the harness writes, ``fields`` over it and the
    keys in ``drop`` left out."""
    case = {
        "source_identity": HARNESS,
        "case_name": case_dir.name,
        "case_definition_sha256": DEFINITION_SHA256,
        **fields,
    }
    for name in drop:
        del case[name]
    case_dir.mkdir(parents=True, exist_ok=True)
    (case_dir / CASE_MANIFEST).write_text(json.dumps(case), encoding="utf-8")


def _identity(case_dir: Path, entry: Path = CHILD) -> dict:
    return run_identity.start_identity(_options(case_dir), entry).finish()


def test_tracing_off_gathers_nothing(monkeypatch, tmp_path):
    """The production path: no case file read, no tree hashed."""
    monkeypatch.setattr(determinism_trace, "ENABLED", False)

    def hashed(*_: object) -> dict:
        pytest.fail("a tree was hashed with tracing off")

    monkeypatch.setattr(run_identity, "source_identity", hashed)
    _case(tmp_path)

    assert run_identity.start_identity(_options(tmp_path), CHILD) is None


def test_the_case_is_recorded_as_the_harness_gave_it(tracing, tmp_path):
    case_dir = tmp_path / "speed-1-run-2"
    _case(case_dir)

    identity = _identity(case_dir)

    assert list(identity) == [
        "harness", "case", "given_error", "endpoints", "run"
    ]
    assert identity["harness"] == HARNESS
    assert identity["case"] == {
        "name": "speed-1-run-2", "definition_sha256": DEFINITION_SHA256
    }
    assert identity["given_error"] is None


def test_what_the_harness_gave_is_not_judged_here(tracing, tmp_path):
    """Recorded as given: a name that is not this directory's is still the
    name the harness gave. Judging it is the reader's (D8.13)."""
    case_dir = tmp_path / "speed-1-run-2"
    _case(case_dir, case_name="speed-10-run-1")

    assert _identity(case_dir)["case"]["name"] == "speed-10-run-1"


@pytest.mark.parametrize(
    ("written", "error"),
    [(None, "FileNotFoundError("), ("{not json", "JSONDecodeError(")],
    ids=["missing", "not_json"],
)
def test_a_case_file_that_cannot_be_read_is_recorded_as_absent(
    tracing, tmp_path, written, error
):
    if written is not None:
        (tmp_path / CASE_MANIFEST).write_text(written, encoding="utf-8")

    identity = _identity(tmp_path)

    assert identity["harness"] is None
    assert identity["case"] == {"name": None, "definition_sha256": None}
    assert identity["given_error"].startswith(error), identity["given_error"]


@pytest.mark.parametrize("given", ["absent", "null"])
def test_a_case_with_no_definition_is_recorded_without_one(
    tracing, tmp_path, given
):
    """The fleet harness names none: recorded as absent, which leaves the
    case ineligible (D8.13), and never made up here."""
    if given == "absent":
        _case(tmp_path, drop=("case_definition_sha256",))
    else:
        _case(tmp_path, case_definition_sha256=None)

    identity = _identity(tmp_path)

    assert identity["case"] == {"name": tmp_path.name, "definition_sha256": None}
    assert identity["given_error"] is None


def test_the_endpoints_hash_the_loaded_tree_and_the_childs_own_imports(
    tracing, tmp_path
):
    _case(tmp_path)
    closure = script_import_closure(CHILD, SCRIPTS)
    expected = source_identity(
        (run_identity.NAVPY_ROOT, *closure), run_identity.WORKTREE
    )

    identity = _identity(tmp_path)

    assert identity["endpoints"] == {"start": expected, "teardown": expected}
    # The navpy the child imports is its own worktree's...
    assert run_identity.NAVPY_ROOT == (
        run_identity.WORKTREE / "src" / "navpy"
    ).resolve()
    assert expected["files"] > len(closure)
    # ...and this code is among what the child hashes, as the child imports it.
    assert SCRIPTS / "pixel_pn_run_identity.py" in closure


@pytest.mark.parametrize("edited", ["navpy", "closure"])
def test_an_edit_while_the_case_flies_changes_only_the_teardown_hash(
    tracing, tmp_path, monkeypatch, edited
):
    """What D8.13 compares: an edit made once the run started, still on
    disk at teardown, changes the teardown's hash, and only that one."""
    tree = tmp_path.resolve() / "tree"
    navpy_root = tree / "src" / "navpy"
    navpy_root.mkdir(parents=True)
    (navpy_root / "law.py").write_text("GAIN = 3\n", encoding="utf-8")
    scripts = tree / "scripts"
    scripts.mkdir()
    entry = scripts / "child.py"
    entry.write_text("import helper\n", encoding="utf-8")
    helper = scripts / "helper.py"
    helper.write_text("VALUE = 1\n", encoding="utf-8")
    monkeypatch.setattr(run_identity, "NAVPY_ROOT", navpy_root)
    monkeypatch.setattr(run_identity, "WORKTREE", tree)
    case_dir = tmp_path / "case"
    _case(case_dir)
    roots = (navpy_root, entry, helper)
    before = source_identity(roots, tree)

    identity = run_identity.start_identity(_options(case_dir), entry)
    (navpy_root / "law.py" if edited == "navpy" else helper).write_text(
        "CHANGED = True\n", encoding="utf-8"
    )
    endpoints = identity.finish()["endpoints"]

    assert endpoints["start"] == before
    assert endpoints["teardown"] == source_identity(roots, tree)
    assert endpoints["teardown"] != endpoints["start"]


def test_a_hashing_fault_is_recorded_not_raised(tracing, tmp_path, monkeypatch):
    def unreadable(*_: object) -> dict:
        raise OSError("tree unreadable")

    monkeypatch.setattr(run_identity, "source_identity", unreadable)
    _case(tmp_path)

    identity = _identity(tmp_path)

    fault = {"error": repr(OSError("tree unreadable"))}
    assert identity["endpoints"] == {"start": fault, "teardown": fault}
    assert identity["harness"] == HARNESS


@pytest.mark.parametrize("end", ["start", "teardown"])
def test_an_interrupt_while_hashing_is_not_swallowed(
    tracing, tmp_path, monkeypatch, end
):
    """Only an interrupt passes, at either end. At teardown it is the
    identity step's failure, which the teardown raises once the evidence
    is written (``test_pixel_pn_child_teardown``)."""
    interrupt = KeyboardInterrupt("ctrl+c while hashing")
    _case(tmp_path)
    started = None
    if end == "teardown":
        started = run_identity.start_identity(_options(tmp_path), CHILD)

    def interrupted(*_: object) -> dict:
        raise interrupt

    monkeypatch.setattr(run_identity, "source_identity", interrupted)

    try:
        if started is None:
            run_identity.start_identity(_options(tmp_path), CHILD)
        else:
            started.finish()
    except BaseException as escaped:  # noqa: BLE001 - asserted on below
        raised = escaped
    else:
        pytest.fail("the interrupt was swallowed")

    assert raised is interrupt


def test_the_run_is_described_and_nothing_compares_it(tracing, tmp_path):
    _case(tmp_path)

    first = _identity(tmp_path)["run"]
    second = _identity(tmp_path)["run"]

    assert (first["pid"], first["python"]) == (os.getpid(), sys.version)
    assert re.fullmatch("[0-9a-f]{32}", first["run_id"]), first["run_id"]
    assert second["run_id"] != first["run_id"]
    wall_start = datetime.fromisoformat(first["wall_start"])
    assert wall_start.utcoffset() == timedelta(0)
    assert first["options"] == {
        "connection": "udp:127.0.0.1:14999",
        "result": str(tmp_path / "result.json"),
        "scoring_active": str(tmp_path / "scoring_active.marker"),
        "sysid": 1,
        "timeout": 5.0,
    }
    assert list(first["options"]) == sorted(first["options"])
    json.dumps(first)


def test_a_run_that_cannot_be_described_is_recorded_not_raised(
    tracing, tmp_path, monkeypatch
):
    """Delivery step 6's review: the identity is started by run()'s first
    statement, so a description that raised there stopped the case before
    anything was built, where the docstring let only an interrupt pass. It
    is recorded in the description's place, as a hash that could not be
    taken is, and the rest of the identity is kept."""
    failure = OSError("random source unavailable")

    def unavailable() -> object:
        raise failure

    monkeypatch.setattr(
        run_identity, "uuid", SimpleNamespace(uuid4=unavailable)
    )
    _case(tmp_path)

    identity = _identity(tmp_path)

    assert identity["run"] == {"error": repr(failure)}
    assert identity["harness"] == HARNESS
    assert identity["case"]["definition_sha256"] == DEFINITION_SHA256


def test_an_interrupt_while_the_run_is_described_is_not_swallowed(
    tracing, tmp_path, monkeypatch
):
    interrupt = KeyboardInterrupt("ctrl+c while the run is described")

    def interrupted() -> object:
        raise interrupt

    monkeypatch.setattr(
        run_identity, "uuid", SimpleNamespace(uuid4=interrupted)
    )
    _case(tmp_path)

    try:
        run_identity.start_identity(_options(tmp_path), CHILD)
    except BaseException as escaped:  # noqa: BLE001 - asserted on below
        raised = escaped
    else:
        pytest.fail("the interrupt was swallowed")

    assert raised is interrupt


if __name__ == "__main__":  # pragma: no cover
    pytest.main([__file__])
