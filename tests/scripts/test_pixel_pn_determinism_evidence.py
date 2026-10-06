"""D7's files, as the child's teardown leaves them.

The trace file, the summary and the manifest are all made from ONE sealed
capture, so they agree with each other even while a callback still records:
it reaches a sealed store and is refused. Each is written beside itself and
moved into place, the manifest last, and each is attempted whatever the
others did: a file whose write fails costs only itself, and the manifest says
which. It vouches for the trace file's bytes only when that file is in place.
A failure the writer does not handle is raised once every file was
attempted, by the rule ``test_pixel_pn_determinism_summary`` holds the
summary to, and as one group when more than one write failed.

How records are encoded, and what the manifest holds, are tested in
``tests/modules/vision/test_determinism_evidence.py``; the child's identity,
which the manifest carries, in ``test_pixel_pn_run_identity``.
"""

from __future__ import annotations

import builtins
import hashlib
import json
import os
import sys
from collections.abc import Callable
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from navpy.exception_groups import ExceptionGroup
from navpy.modules.vision.sim.determinism_admission_ledger import PassObserver
from navpy.modules.vision.sim.determinism_events import (
    ASSOCIATION_COMMITTED,
    EVENT_TRUTH,
    LIFECYCLE_CLOSED,
    TRUTH_RECORDED,
)
from navpy.modules.vision.sim.determinism_evidence import (
    KIND_COMMAND,
    KIND_LEDGER,
    KIND_PASS,
    KIND_ROW,
    MANIFEST_NAME,
    SUMMARY_NAME,
    TRACE_NAME,
    trace_lines,
)
from navpy.modules.vision.sim.determinism_eligibility import case_verdict
from navpy.modules.vision.sim.determinism_trace import DeterminismTrace
from navpy.modules.vision.sim.determinism_trace_summary import (
    summarise_capture,
)
from scripts import pixel_pn_determinism_evidence as evidence
from scripts.pixel_pn_determinism_evidence import (
    EVIDENCE_UNWRITTEN,
    write_determinism_evidence,
)
from scripts.pixel_pn_determinism_summary import (
    STAGING_SUFFIX,
    UNWRITABLE_MARKER,
)
from tests.modules.vision import determinism_case_factory as factory

PERIOD_US = 20_000  # 50 Hz autopilot scheduler period.
FINALIZATION = {
    "worker": "stopped",
    "mavlink_quiescence": "unverified",
    "teardown_errors": [],
    "admission": None,
}
IDENTITY = {
    "harness": {"sha256": "a" * 64, "files": 12},
    "case": {"name": "speed-1-run-1", "definition_sha256": "d" * 64},
    "given_error": None,
    "endpoints": {
        "start": {"sha256": "e" * 64, "files": 3},
        "teardown": {"sha256": "e" * 64, "files": 3},
    },
    "run": {"pid": 1},
}
# The files, in the order they are written.
FILES = (TRACE_NAME, SUMMARY_NAME, MANIFEST_NAME)


def _ruling(ruling: int, stamp: int | None, accepted: bool) -> SimpleNamespace:
    """What the router's tap hands the ledger: one ruling on one ATTITUDE."""
    return SimpleNamespace(
        ruling=ruling, time_boot_ms=stamp, accepted=accepted, reason=None
    )


def _traced() -> DeterminismTrace:
    """Three rows, one pass, one command entry and one ruling."""
    trace = DeterminismTrace(PERIOD_US)
    trace.ledger.append(_ruling(1, 1_000, True))
    trace.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=1.0
    )
    trace.record_truth(epoch=1, outcome=TRUTH_RECORDED)
    PassObserver(trace.ledger, trace.command_log).note_iteration(1)
    trace.command_log.note_command(1, None, raised=True)
    trace.record_lifecycle(epoch=1, outcome=LIFECYCLE_CLOSED, resulting_epoch=2)
    return trace


def _failing(trace: DeterminismTrace, failure: BaseException) -> DeterminismTrace:
    """``trace``, whose capture raises ``failure`` once its stores are
    sealed."""

    def capture():
        raise failure

    trace.capture = capture
    return trace


def _write(
    directory: Path, trace: DeterminismTrace | None, **overrides: Any
) -> Path | None:
    fields: dict[str, Any] = {
        "epoch": 1,
        "finalization": FINALIZATION,
        "identity": IDENTITY,
        **overrides,
    }
    return write_determinism_evidence(directory, trace, **fields)


def _json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _escaped(call: Callable[[], object]) -> BaseException:
    """What ``call()`` raised. Caught as BaseException rather than with
    ``pytest.raises``: an interrupt escaping a test ends the whole pytest
    session instead of failing the test."""
    try:
        call()
    except BaseException as escaped:  # noqa: BLE001 - the caller asserts on it
        return escaped
    pytest.fail("nothing was raised")


def test_tracing_off_writes_nothing(tmp_path):
    """The production path. No trace, no files, no exception."""
    assert _write(tmp_path, None) is None
    assert list(tmp_path.iterdir()) == []


def test_every_file_is_staged_then_replaced_and_the_manifest_is_last(
    tmp_path, monkeypatch
):
    real_replace = os.replace
    replaced: list[tuple[str, str]] = []

    def replace_(source: Any, destination: Any) -> None:
        replaced.append((Path(source).name, Path(destination).name))
        real_replace(source, destination)

    monkeypatch.setattr(os, "replace", replace_)

    path = _write(tmp_path, _traced())

    # The manifest's path: the file a reader starts from.
    assert path == tmp_path / MANIFEST_NAME
    assert replaced == [(name + STAGING_SUFFIX, name) for name in FILES]
    # Nothing staged is left behind, and nothing else is written.
    assert sorted(entry.name for entry in tmp_path.iterdir()) == sorted(FILES)


def test_the_manifest_vouches_for_the_trace_file_on_disk(tmp_path):
    _write(tmp_path, _traced())

    data = (tmp_path / TRACE_NAME).read_bytes()
    manifest = _json(tmp_path / MANIFEST_NAME)
    assert manifest["trace"] == {
        "file": TRACE_NAME,
        "sha256": hashlib.sha256(data).hexdigest(),
        "lines": 6,
        "kinds": {KIND_ROW: 3, KIND_PASS: 1, KIND_COMMAND: 1, KIND_LEDGER: 1},
        "error": None,
    }
    assert data.endswith(b"\n") and data.count(b"\n") == 6
    assert manifest["summary"] == {"file": SUMMARY_NAME, "error": None}
    assert manifest["identity"] == IDENTITY
    assert manifest["finalization"] == {**FINALIZATION, "epoch": 1}
    assert _json(tmp_path / SUMMARY_NAME)["finalization"] == (
        manifest["finalization"]
    )
    assert manifest["stores"]["journal"]["callback_faults"] == 0
    assert (manifest["eligibility"], manifest["error"]) == (None, None)
    # Nothing held for raising: the evidence step did not fail (D8.12).
    assert manifest["held"] == 0


def test_a_recording_that_lands_while_the_files_are_written_is_refused(
    tmp_path, monkeypatch
):
    """A MAVLink callback still in flight at teardown (``mavlink_quiescence``
    is unverified): it reaches a sealed store, changes nothing and is
    counted, so the three files still agree with each other."""
    trace = _traced()
    real_replace = os.replace
    landed: list[str] = []

    def replace_(source: Any, destination: Any) -> None:
        if not landed:
            trace.record_truth(epoch=1, outcome=TRUTH_RECORDED)
            landed.append(Path(destination).name)
        real_replace(source, destination)

    monkeypatch.setattr(os, "replace", replace_)

    _write(tmp_path, trace)

    assert landed == [TRACE_NAME]
    records = [
        json.loads(line)
        for line in (tmp_path / TRACE_NAME).read_text("ascii").splitlines()
    ]
    assert sum(1 for record in records if record["kind"] == KIND_ROW) == 3
    assert _json(tmp_path / SUMMARY_NAME)["summary"]["rows_recorded"] == 3
    manifest = _json(tmp_path / MANIFEST_NAME)
    assert manifest["stores"]["journal"]["rows"] == 3
    assert manifest["trace"]["kinds"][KIND_ROW] == 3
    # The positive control: the recording really reached the trace.
    later = trace.capture()
    assert (len(later.rows), later.status.refused) == (3, 1)


def test_the_summary_is_reduced_from_the_capture_the_files_share(tmp_path):
    trace = _traced()

    _write(tmp_path, trace)

    payload = _json(tmp_path / SUMMARY_NAME)
    # Sealed, so a capture now holds exactly what the writer's held.
    expected = summarise_capture(trace.capture(), epoch=1)
    assert payload["summary"] == json.loads(json.dumps(expected))
    assert payload["error"] is None
    assert payload["finalization"] == {**FINALIZATION, "epoch": 1}


def test_a_record_with_no_exact_encoding_costs_only_the_trace_file(tmp_path):
    """The encode refuses it (the encoding's own tests say why), and the
    refusal is the trace file's error in the manifest: the summary and the
    manifest are still written, and nothing is raised."""
    trace = _traced()
    real_capture = trace.capture

    def with_a_float_row():
        capture = real_capture()
        float_row = (EVENT_TRUTH, 1, TRUTH_RECORDED, 1.5, None)
        return replace(capture, rows=(*capture.rows, float_row))

    trace.capture = with_a_float_row

    path = _write(tmp_path, trace)

    assert path == tmp_path / MANIFEST_NAME
    assert not (tmp_path / TRACE_NAME).exists()
    assert not (tmp_path / (TRACE_NAME + STAGING_SUFFIX)).exists()
    record = _json(path)["trace"]
    assert (record["sha256"], record["lines"], record["kinds"]) == (
        None, None, None
    )
    assert record["error"].startswith("TypeError("), record["error"]
    assert _json(path)["error"] is None
    assert _json(tmp_path / SUMMARY_NAME)["summary"] is not None


def test_an_unwritable_trace_file_is_announced_and_the_rest_still_land(
    tmp_path, capsys
):
    # A directory where the file has to go: its replace fails, nothing else.
    (tmp_path / TRACE_NAME).mkdir()

    path = _write(tmp_path, _traced())

    assert path == tmp_path / MANIFEST_NAME
    out = capsys.readouterr().out
    assert UNWRITABLE_MARKER in out and TRACE_NAME in out
    record = _json(path)["trace"]
    assert record["sha256"] is None
    # PermissionError on Windows, IsADirectoryError elsewhere.
    assert record["error"].startswith(
        ("PermissionError(", "IsADirectoryError(")
    ), record["error"]
    # Handled, so nothing is held for raising: the file's error says it.
    assert _json(path)["held"] == 0
    assert _json(tmp_path / SUMMARY_NAME)["summary"]["epoch"] == 1


def test_an_unwritable_manifest_is_announced_and_returns_none(
    tmp_path, capsys
):
    (tmp_path / MANIFEST_NAME).mkdir()

    assert _write(tmp_path, _traced()) is None
    out = capsys.readouterr().out
    assert UNWRITABLE_MARKER in out and MANIFEST_NAME in out
    assert (tmp_path / TRACE_NAME).is_file()
    assert (tmp_path / SUMMARY_NAME).is_file()


@pytest.mark.parametrize("how", ["second_interrupt", "broken_stdout"])
def test_a_failed_trace_write_is_raised_as_itself_after_the_rest_land(
    tmp_path, failing_summary_write, how
):
    """A failure the write does not handle, a second Ctrl+C mid-write or
    stdout gone while an unwritable path is announced, is kept; the summary
    and the manifest are still written, and only then is it raised."""
    injected = failing_summary_write(how, TRACE_NAME)

    raised = _escaped(lambda: _write(tmp_path, _traced()))

    assert raised is injected[-1]
    assert not (tmp_path / TRACE_NAME).exists()
    record = _json(tmp_path / MANIFEST_NAME)["trace"]
    assert (record["sha256"], record["error"]) == (None, repr(injected[-1]))
    assert _json(tmp_path / MANIFEST_NAME)["held"] == 1
    assert _json(tmp_path / SUMMARY_NAME)["summary"]["epoch"] == 1


def test_an_interrupted_write_never_touches_the_file_in_place(
    tmp_path, failing_summary_write
):
    """Only the staged copy is written, so an interrupt mid-write leaves a
    half-written copy beside the file and the file as it was, and the
    manifest vouches for neither."""
    (tmp_path / TRACE_NAME).write_bytes(b"left by an earlier write\n")
    trace = _traced()
    injected = failing_summary_write("second_interrupt", TRACE_NAME)

    raised = _escaped(lambda: _write(tmp_path, trace))

    assert raised is injected[-1]
    assert (tmp_path / TRACE_NAME).read_bytes() == b"left by an earlier write\n"
    whole = trace_lines(trace.capture()).data
    half = (tmp_path / (TRACE_NAME + STAGING_SUFFIX)).read_bytes()
    assert 0 < len(half) < len(whole) and whole.startswith(half)
    assert _json(tmp_path / MANIFEST_NAME)["trace"]["sha256"] is None


def test_every_write_failing_is_raised_as_one_group_once_all_were_tried(
    tmp_path, monkeypatch
):
    for name in FILES:
        (tmp_path / name).mkdir()
    real_print = builtins.print
    broken: list[BaseException] = []

    def print_(*args: Any, **kwargs: Any) -> None:
        if args and str(args[0]).startswith(UNWRITABLE_MARKER):
            broken.append(BrokenPipeError("stdout closed"))
            raise broken[-1]
        real_print(*args, **kwargs)

    monkeypatch.setattr(builtins, "print", print_)

    raised = _escaped(lambda: _write(tmp_path, _traced()))

    assert isinstance(raised, ExceptionGroup), repr(raised)
    assert raised.message == EVIDENCE_UNWRITTEN
    assert list(raised.exceptions) == broken
    assert len(broken) == len(FILES)


def test_a_failed_capture_is_the_manifests_error_and_no_store_is_claimed(
    tmp_path,
):
    error = RuntimeError("row store is gone")

    path = _write(tmp_path, _failing(_traced(), error))

    assert path == tmp_path / MANIFEST_NAME
    assert not (tmp_path / TRACE_NAME).exists()
    manifest = _json(path)
    assert manifest["error"] == repr(error)
    for key in ("period_us", "capacity", "stores"):
        assert manifest[key] is None, key
    assert manifest["trace"]["sha256"] is None
    payload = _json(tmp_path / SUMMARY_NAME)
    assert (payload["summary"], payload["error"]) == (None, repr(error))


def test_no_identity_is_written_as_none(tmp_path):
    """What the teardown passes when its identity step failed: a reader
    (D8.13) finds none, never one made up."""
    _write(tmp_path, _traced(), identity=None)

    assert _json(tmp_path / MANIFEST_NAME)["identity"] is None


def test_an_interrupt_while_the_trace_is_encoded_waits_for_every_write(
    tmp_path, monkeypatch
):
    interrupt = KeyboardInterrupt("ctrl+c while encoding")

    def interrupted(_capture: object) -> object:
        raise interrupt

    monkeypatch.setattr(evidence, "trace_lines", interrupted)

    raised = _escaped(lambda: _write(tmp_path, _traced()))

    assert raised is interrupt
    assert not (tmp_path / TRACE_NAME).exists()
    manifest = _json(tmp_path / MANIFEST_NAME)
    assert manifest["trace"]["error"] == repr(interrupt)
    assert manifest["held"] == 1
    assert manifest["error"] is None
    assert manifest["stores"]["journal"]["rows"] == 3
    assert _json(tmp_path / SUMMARY_NAME)["summary"]["epoch"] == 1


def test_an_interrupted_summary_costs_only_the_summary(tmp_path, monkeypatch):
    interrupt = SystemExit(7)
    # Patched where the writer looks it up: it imports its siblings as
    # top-level modules, not as the ``scripts.`` copies imported here.
    summary = sys.modules[evidence.summary_payload.__module__]

    def interrupted(_capture: object, *, epoch: object) -> object:
        raise interrupt

    monkeypatch.setattr(summary, "summarise_capture", interrupted)

    raised = _escaped(lambda: _write(tmp_path, _traced()))

    assert raised is interrupt
    payload = _json(tmp_path / SUMMARY_NAME)
    assert (payload["summary"], payload["error"]) == (None, repr(interrupt))
    manifest = _json(tmp_path / MANIFEST_NAME)
    assert (manifest["trace"]["lines"], manifest["trace"]["error"]) == (6, None)
    assert manifest["error"] is None
    # The manifest's summary record names it, and the evidence step raises
    # it, which the manifest's count of what is held shows (delivery step
    # 6's review).
    assert manifest["summary"]["error"] == repr(interrupt)
    assert manifest["held"] == 1


def test_the_manifest_counts_every_failure_held_for_raising(
    tmp_path, monkeypatch, failing_summary_write
):
    """An interrupted summary and a trace write interrupted mid-write: one
    interrupt held while the files were made, and one write that failed in
    a way the write does not handle. Both are raised once every file was
    attempted, and the manifest counts both (delivery step 6's review)."""
    summary = sys.modules[evidence.summary_payload.__module__]

    def interrupted(_capture: object, *, epoch: object) -> object:
        raise SystemExit(7)

    monkeypatch.setattr(summary, "summarise_capture", interrupted)
    failing_summary_write("second_interrupt", TRACE_NAME)

    _escaped(lambda: _write(tmp_path, _traced()))

    assert _json(tmp_path / MANIFEST_NAME)["held"] == 2


@pytest.mark.parametrize(
    "failure",
    [None, MemoryError("no room to reduce"), SystemExit(7)],
    ids=["reduced", "error", "interrupt"],
)
def test_a_summary_that_fails_to_reduce_fails_the_case(
    tmp_path, monkeypatch, failure
):
    """Review of the whole: a reduction that raised left its error in the
    summary file alone. The manifest's summary record read clean and an
    ordinary error is held for no one, so the case read ELIGIBLE though the
    evidence step had failed. The record now names it, and D8.12 fails the
    case on it; the same case, reduced, is ELIGIBLE."""
    if failure is not None:
        summary = sys.modules[evidence.summary_payload.__module__]

        def failing(_capture: object, *, epoch: object) -> object:
            raise failure

        monkeypatch.setattr(summary, "summarise_capture", failing)

    # Caught here, not by ``_escaped``, which fails a call that raises
    # nothing: two of the three raise nothing.
    raised = None
    try:
        write_determinism_evidence(
            tmp_path,
            factory.eligible_trace(),
            epoch=1,
            finalization=factory.finalization(),
            identity=factory.IDENTITY,
        )
    except BaseException as escaped:  # noqa: BLE001 - asserted below
        raised = escaped

    error = None if failure is None else repr(failure)
    interrupted = failure is not None and not isinstance(failure, Exception)
    assert raised is (failure if interrupted else None)
    assert _json(tmp_path / MANIFEST_NAME)["summary"]["error"] == error
    verdict = case_verdict(tmp_path)
    assert list(verdict.failed_rules) == ([] if failure is None else [12])


class _SilentOSError(OSError):
    """A write error whose text is empty."""

    def __repr__(self) -> str:
        return ""


@pytest.mark.parametrize("reduced", [True, False], ids=["reduced", "unreduced"])
def test_a_write_error_with_empty_text_still_fails_the_case(
    tmp_path, monkeypatch, reduced
):
    """Review of the whole, round 2: the record took an empty write error for
    none, so a summary that was never written read ELIGIBLE. The write's
    error is kept whatever its text, and ahead of a reduction that raised
    too."""
    summary = sys.modules[evidence.summary_payload.__module__]
    real_replace = summary.os.replace

    def failing(source: object, destination: object) -> None:
        if Path(destination).name == SUMMARY_NAME:
            raise _SilentOSError("write failed")
        real_replace(source, destination)

    def unreduced(_capture: object, *, epoch: object) -> object:
        raise MemoryError("no room to reduce")

    monkeypatch.setattr(summary.os, "replace", failing)
    if not reduced:
        monkeypatch.setattr(summary, "summarise_capture", unreduced)

    write_determinism_evidence(
        tmp_path,
        factory.eligible_trace(),
        epoch=1,
        finalization=factory.finalization(),
        identity=factory.IDENTITY,
    )

    assert not (tmp_path / SUMMARY_NAME).exists()
    assert _json(tmp_path / MANIFEST_NAME)["summary"]["error"] == ""
    assert list(case_verdict(tmp_path).failed_rules) == [12]


if __name__ == "__main__":  # pragma: no cover
    pytest.main([__file__])
