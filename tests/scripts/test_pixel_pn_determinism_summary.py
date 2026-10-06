"""The decision trace has to LEAVE the process, scoped to the leg that ENDED.

Landing 1 shipped a recorder and a reduction, both tested, and no caller. A
traced leg recorded every decision perfectly and dropped all of it on exit,
because the rows live in a bounded in-memory ring and nothing drained them.

The writer's first version then worked out WHICH leg ended from the newest row,
and review round 1 showed that names the wrong leg or none (F1, F3): a dispatch
that takes its frame after close() records at the NEXT epoch, and a boundary
with both slots empty recorded nothing. A boundary now has its own row, which
an overflow or a recorder fault can still lose. So the caller supplies the
epoch, from close(), and these tests hold the writer to it. That the child
really calls it, and in the right order, is tested through the real ``run()``
in ``test_pixel_pn_child_teardown``.

Since delivery step 6 the summary is one of three files the evidence writer
leaves, all made from one sealed capture and each moved into place once
written, the manifest last (``test_pixel_pn_determinism_evidence``). These
tests read the summary, and the manifest where the two must agree. The writer
returns the manifest's path: the file a reader starts from.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from navpy.exception_groups import BaseExceptionGroup, ExceptionGroup
from navpy.modules.vision.sim.determinism_events import (
    OUTPUT_EMPTY,
    TRUTH_RECORDED,
)
from navpy.modules.vision.sim.determinism_evidence import MANIFEST_NAME
from navpy.modules.vision.sim.determinism_trace import DeterminismTrace
from scripts.pixel_pn_determinism_evidence import (
    INTERRUPTED_DESCRIBING,
    INTERRUPTED_UNWRITTEN,
    write_determinism_evidence,
)
from scripts.pixel_pn_determinism_summary import (
    REPR_FAILED,
    SUMMARY_NAME,
    UNKNOWN_EPOCH_ERROR,
    UNWRITABLE_MARKER,
    artifact_repr,
)

PERIOD_US = 20_000  # 50 Hz autopilot scheduler period.
FINALIZATION = {
    "worker": "stopped",
    "mavlink_quiescence": "unverified",
    "teardown_errors": [],
}


def _write(directory: Path, trace, epoch: int | None) -> Path | None:
    """The evidence writer as the teardown calls it, with no identity."""
    return write_determinism_evidence(
        directory, trace, epoch=epoch, finalization=FINALIZATION, identity=None
    )


def _payload(directory: Path) -> dict:
    return json.loads((directory / SUMMARY_NAME).read_text(encoding="utf-8"))


def _manifest(directory: Path) -> dict:
    return json.loads((directory / MANIFEST_NAME).read_text(encoding="utf-8"))


def _failing(failure: BaseException) -> DeterminismTrace:
    """A real trace whose capture raises ``failure``. Real, because the
    writer seals every store before it captures them."""
    trace = DeterminismTrace(PERIOD_US)

    def capture():
        raise failure

    trace.capture = capture
    return trace


def test_tracing_off_writes_nothing(tmp_path):
    """The production path. No trace, no files, no exception."""
    assert _write(tmp_path, None, 1) is None
    assert list(tmp_path.iterdir()) == []


def test_the_summary_is_scoped_to_the_epoch_it_is_given(tmp_path):
    trace = DeterminismTrace(PERIOD_US)
    trace.record_truth(epoch=0, outcome=TRUTH_RECORDED)  # cruise
    trace.record_truth(epoch=1, outcome=TRUTH_RECORDED)  # scored
    trace.record_truth(epoch=1, outcome=TRUTH_RECORDED)
    # The worker's last pass, started after close() moved the epoch on.
    trace.record_output(epoch=2, outcome=OUTPUT_EMPTY, taken_at_us=None)

    path = _write(tmp_path, trace, 1)

    assert path == tmp_path / MANIFEST_NAME
    payload = _payload(tmp_path)
    assert payload["error"] is None
    assert payload["finalization"] == {**FINALIZATION, "epoch": 1}
    assert _manifest(tmp_path)["finalization"] == payload["finalization"]
    summary = payload["summary"]
    assert summary["epoch"] == 1
    # The scored leg's rows only: neither the cruise leg nor the row that
    # landed after the boundary decides this leg's verdict.
    assert summary["rows"] == 2
    assert summary["counts"]["truth"] == {TRUTH_RECORDED: 2}
    assert summary["counts"]["output"] == {}
    # Trace-wide on purpose: the status describes the whole ledger.
    assert summary["rows_recorded"] == 4
    # The series Landing 2 needs are present even when empty, so a reader can
    # tell "no dispatches" from "this summary predates the field".
    for series in ("stage_lag_slots", "dispatch_lag_slots", "ready_iterations"):
        assert series in summary


def _ended_leg() -> DeterminismTrace:
    trace = DeterminismTrace(PERIOD_US)
    trace.record_truth(epoch=1, outcome=TRUTH_RECORDED)
    return trace


def test_a_row_at_the_next_epoch_leaves_the_ended_leg_unchanged(tmp_path):
    """F1's shape: the worker's last pass records after close().

    Only the trace-wide figures may move. Everything scoped to the leg that
    ended stays exactly as it was. Two traces, the second with the last
    pass: the writer seals the trace it writes, so a pass recorded after
    one write would be refused, never summarised.
    """
    without, with_pass = tmp_path / "without", tmp_path / "with_pass"
    without.mkdir()
    with_pass.mkdir()
    _write(without, _ended_leg(), 1)
    trace = _ended_leg()
    trace.command_log.note_iteration(1)
    trace.record_output(epoch=2, outcome=OUTPUT_EMPTY, taken_at_us=None)
    _write(with_pass, trace, 1)

    before = _payload(without)["summary"]
    after = _payload(with_pass)["summary"]

    trace_wide = {"rows_recorded", "commands"}
    assert {k: v for k, v in after.items() if k not in trace_wide} == {
        k: v for k, v in before.items() if k not in trace_wide
    }
    assert after["rows_recorded"] == before["rows_recorded"] + 1
    assert (
        after["commands"]["iterations"]
        == before["commands"]["iterations"] + 1
    )


def test_an_unknown_epoch_writes_an_error_not_a_summary_of_every_leg(tmp_path):
    """``summarise(epoch=None)`` means EVERY leg, never "unknown".

    Passing it through would label the cruise and scored legs, merged, as the
    leg that ended. The trace itself is still captured and written: which
    leg ended is the summary's question, not the trace file's.
    """
    trace = DeterminismTrace(PERIOD_US)
    trace.record_truth(epoch=0, outcome=TRUTH_RECORDED)
    trace.record_truth(epoch=1, outcome=TRUTH_RECORDED)

    _write(tmp_path, trace, None)

    payload = _payload(tmp_path)
    assert payload["summary"] is None
    assert payload["error"] == UNKNOWN_EPOCH_ERROR
    assert payload["finalization"] == {**FINALIZATION, "epoch": None}
    manifest = _manifest(tmp_path)
    assert manifest["finalization"]["epoch"] is None
    assert manifest["stores"]["journal"]["rows"] == 2
    assert manifest["error"] is None


def test_a_failed_summary_is_written_beside_its_finalization(tmp_path):
    """Teardown may not raise for this, and it may not go quiet either.

    Swallowing the failure would leave no artifact AND no sign there should
    have been one -- the same defect the recorder's own fault latch exists to
    prevent. An error on its own would not say which leg ended or what else
    failed on the way down, so the finalization block survives it.
    """
    path = _write(tmp_path, _failing(RuntimeError("row store is gone")), 1)

    assert path is not None and path.exists()
    payload = _payload(tmp_path)
    assert payload["summary"] is None
    assert "row store is gone" in payload["error"]
    assert payload["finalization"] == {**FINALIZATION, "epoch": 1}
    assert _manifest(tmp_path)["error"] == payload["error"]


@pytest.mark.parametrize(
    "interrupt",
    [KeyboardInterrupt("ctrl+c"), SystemExit(7)],
    ids=["keyboard_interrupt", "system_exit"],
)
def test_an_interrupted_summary_is_written_and_then_raised_on(
    tmp_path, interrupt
):
    """An interrupt is not a summary fault, so it is not swallowed: the
    operator's stop still stops. But the summary is the teardown's LAST step,
    and review round 3 showed, through the child's real ``run()``, that an
    interrupt there left no artifact -- the finalization block went with it,
    though every step before had kept its own interrupt and run on.

    Caught as BaseException rather than with ``pytest.raises``: an interrupt
    escaping this test would end the whole pytest session instead of failing
    it.
    """
    try:
        _write(tmp_path, _failing(interrupt), 1)
    except BaseException as escaped:  # noqa: BLE001 - asserted on below
        raised = escaped
    else:
        pytest.fail("the interrupt was swallowed")

    assert raised is interrupt
    payload = _payload(tmp_path)
    assert payload["summary"] is None
    assert payload["error"] == repr(interrupt)
    assert payload["finalization"] == {**FINALIZATION, "epoch": 1}
    assert _manifest(tmp_path)["error"] == repr(interrupt)


INTERRUPTS = {
    "keyboard_interrupt": lambda: KeyboardInterrupt("ctrl+c"),
    "system_exit": lambda: SystemExit(7),
}


def _interrupted_summary(tmp_path: Path, failure: BaseException):
    """Run the writer over a capture that raises ``failure``, an interrupt or
    an error whose text is interrupted, and return what the writer raised.
    Caught as BaseException for the reason given above."""
    try:
        _write(tmp_path, _failing(failure), 1)
    except BaseException as escaped:  # noqa: BLE001 - the caller asserts on it
        return escaped
    pytest.fail("the interrupt was swallowed")


@pytest.mark.parametrize("how", ["second_interrupt", "broken_stdout"])
@pytest.mark.parametrize("kind", sorted(INTERRUPTS))
def test_a_failed_write_is_raised_with_the_interrupt_not_in_its_place(
    tmp_path, failing_summary_write, kind, how
):
    """Review round 4. The write made for an interrupt can fail too -- a
    second Ctrl+C mid-write, or stdout gone when an unwritable path is
    announced -- and that failure used to escape INSTEAD of the interrupt.
    Through the child's teardown the run's own error is then chained onto it,
    and the interrupt is nowhere in the exception chain.

    So both are raised, the interrupt first. There is no summary: the write
    that would have made it is the thing that failed. The manifest, written
    after it, says so.
    """
    interrupt = INTERRUPTS[kind]()
    injected = failing_summary_write(how)

    raised = _interrupted_summary(tmp_path, interrupt)

    assert isinstance(raised, BaseExceptionGroup), repr(raised)
    assert not isinstance(raised, ExceptionGroup)
    assert raised.message == INTERRUPTED_UNWRITTEN
    assert list(raised.exceptions) == [interrupt, injected[-1]]
    if how == "broken_stdout":
        # All of the write's failure survives: the pipe broke while the
        # OSError was being announced.
        assert injected[-1].__context__ is injected[0]
    assert not (tmp_path / SUMMARY_NAME).exists()
    assert _manifest(tmp_path)["summary"]["error"] == repr(injected[-1])


@pytest.mark.parametrize("kind", sorted(INTERRUPTS))
def test_an_unwritable_artifact_after_an_interrupt_is_announced_and_raised_alone(
    tmp_path, capsys, failing_summary_write, kind
):
    """The OSError the writer HANDLES is not a failed write: it is announced
    on stdout, as for any unwritable path, and the interrupt is raised alone.
    Only a failure that escapes the write is raised with it."""
    interrupt = INTERRUPTS[kind]()
    injected = failing_summary_write("unwritable")

    raised = _interrupted_summary(tmp_path, interrupt)

    assert raised is interrupt
    assert UNWRITABLE_MARKER in capsys.readouterr().out
    assert not (tmp_path / SUMMARY_NAME).exists()
    assert _manifest(tmp_path)["summary"]["error"] == repr(injected[0])


def test_an_unwritable_destination_is_announced_not_swallowed(
    tmp_path, capsys
):
    trace = DeterminismTrace(PERIOD_US)
    trace.record_truth(epoch=1, outcome=TRUTH_RECORDED)
    # A directory where the file needs to go: its replace fails, nothing else.
    (tmp_path / SUMMARY_NAME).mkdir()

    path = _write(tmp_path, trace, 1)

    assert UNWRITABLE_MARKER in capsys.readouterr().out
    # The manifest still lands, and says the summary did not.
    assert path == tmp_path / MANIFEST_NAME
    assert _manifest(tmp_path)["summary"]["error"] is not None


class _Unrepresentable:
    """A value whose repr raises ``failure``, and so the repr of any
    exception that carries it as an argument."""

    def __init__(self, failure: BaseException | None = None) -> None:
        self._failure = failure or ValueError("no repr")

    def __repr__(self) -> str:
        raise self._failure


def test_the_artifact_repr_is_a_fixed_text_when_repr_raises():
    """Only an interrupt passes, as everywhere in this writer."""
    assert artifact_repr(("rows", 3)) == repr(("rows", 3))
    assert artifact_repr(_Unrepresentable()) == REPR_FAILED
    interrupt = KeyboardInterrupt("ctrl+c while describing")
    try:
        artifact_repr(_Unrepresentable(interrupt))
    except BaseException as escaped:  # noqa: BLE001 - asserted on below
        raised = escaped
    else:
        pytest.fail("the interrupt was swallowed")
    assert raised is interrupt


def test_a_failed_summary_whose_error_has_no_repr_is_still_written(tmp_path):
    """The error's text is made in the handler, where a repr that raised
    escaped and cost the artifact. The fixed text stands in for it."""
    _write(tmp_path, _failing(RuntimeError(_Unrepresentable())), 1)

    payload = _payload(tmp_path)
    assert payload["summary"] is None
    assert payload["error"] == REPR_FAILED
    assert payload["finalization"] == {**FINALIZATION, "epoch": 1}
    assert _manifest(tmp_path)["error"] == REPR_FAILED


@pytest.mark.parametrize(
    "kind",
    [KeyboardInterrupt, SystemExit],
    ids=["keyboard_interrupt", "system_exit"],
)
def test_an_interrupt_whose_repr_raises_is_written_and_raised_as_itself(
    tmp_path, kind
):
    """For an interrupt, a repr that raised escaped in the interrupt's
    PLACE and skipped the write made for it: review round 4's shape
    again. The artifact is written with the fixed text, and the
    interrupt is raised on as itself."""
    interrupt = kind(_Unrepresentable())

    raised = _interrupted_summary(tmp_path, interrupt)

    assert raised is interrupt
    payload = _payload(tmp_path)
    assert payload["summary"] is None
    assert payload["error"] == REPR_FAILED
    assert payload["finalization"] == {**FINALIZATION, "epoch": 1}


def test_a_value_json_cannot_encode_or_repr_is_a_fixed_text(tmp_path):
    """What ``json`` cannot encode degrades to its repr, and a repr that
    raised there cost the whole artifact. The fixed text stands in, in the
    summary and the manifest alike."""
    trace = DeterminismTrace(PERIOD_US)
    finalization = {
        **FINALIZATION, "admission": {"first": _Unrepresentable()}
    }

    write_determinism_evidence(
        tmp_path, trace, epoch=1, finalization=finalization, identity=None
    )

    payload = _payload(tmp_path)
    assert payload["error"] is None
    assert payload["finalization"]["admission"] == {"first": REPR_FAILED}
    assert _manifest(tmp_path)["finalization"]["admission"] == {
        "first": REPR_FAILED
    }


class _InterruptedRepr:
    """A value whose repr is interrupted once: a Ctrl+C lands once, while
    its text is made. Later reprs work, so a failing assertion can still
    show it, and a second description of it would not read REPR_FAILED."""

    def __init__(self, interrupt: BaseException) -> None:
        self._interrupt: BaseException | None = interrupt

    def __repr__(self) -> str:
        interrupt, self._interrupt = self._interrupt, None
        if interrupt is not None:
            raise interrupt
        return "_InterruptedRepr()"


# The summary's own failure, by case: an ordinary one, or an interrupt.
FAILURES = {
    "exception": RuntimeError,
    "keyboard_interrupt": KeyboardInterrupt,
    "system_exit": SystemExit,
}


@pytest.mark.parametrize("kind", sorted(FAILURES))
def test_an_error_whose_repr_is_interrupted_is_written_before_it_is_raised(
    tmp_path, kind
):
    """Delivery step 5, review round 2. The error's text is made before the
    write, and an interrupt landing there escaped at once: no write was
    attempted, and for an interrupted summary it escaped in place of the
    summary's own interrupt, which the child's ``finally`` then lost. The
    artifact is written with the fixed text, and only then is every
    interrupt raised: one as itself, two grouped, the summary's own
    first."""
    describing = KeyboardInterrupt("ctrl+c while the error is described")
    failure = FAILURES[kind](_InterruptedRepr(describing))

    raised = _interrupted_summary(tmp_path, failure)

    payload = _payload(tmp_path)
    assert payload["summary"] is None
    assert payload["error"] == REPR_FAILED
    assert payload["finalization"] == {**FINALIZATION, "epoch": 1}
    if kind == "exception":
        assert raised is describing
    else:
        assert isinstance(raised, BaseExceptionGroup), repr(raised)
        assert not isinstance(raised, ExceptionGroup)
        assert raised.message == INTERRUPTED_DESCRIBING
        assert list(raised.exceptions) == [failure, describing]


@pytest.mark.parametrize("kind", sorted(FAILURES))
def test_a_failed_write_is_raised_after_every_interrupt_it_was_made_for(
    tmp_path, failing_summary_write, kind
):
    """The same when the write fails too, with review round 4's second
    Ctrl+C: its failure is raised with the interrupts, last, and there is
    no summary."""
    describing = KeyboardInterrupt("ctrl+c while the error is described")
    failure = FAILURES[kind](_InterruptedRepr(describing))
    injected = failing_summary_write("second_interrupt")

    raised = _interrupted_summary(tmp_path, failure)

    interrupts = [describing] if kind == "exception" else [failure, describing]
    assert isinstance(raised, BaseExceptionGroup), repr(raised)
    assert not isinstance(raised, ExceptionGroup)
    assert raised.message == INTERRUPTED_UNWRITTEN
    assert list(raised.exceptions) == [*interrupts, injected[-1]]
    assert not (tmp_path / SUMMARY_NAME).exists()


if __name__ == "__main__":  # pragma: no cover
    pytest.main([__file__])
