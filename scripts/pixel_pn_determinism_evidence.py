"""Write a traced case's evidence: the trace file, the summary, the manifest.

D7 of the LANDING2 step-1 plan, and the child's
teardown's last step (``pixel_pn_child_teardown``). Every store is sealed and
then captured ONCE, and all three files are made from that capture
(``determinism_evidence``), so they agree with each other however late a
callback runs. Each is written beside itself and moved into place
(``write_replacing``): the trace file, the summary, then the manifest LAST.
The manifest says what the others are, and vouches for the trace file's bytes
only when that file is in place. Split from the summary's module, which keeps
the text and the writes, so that each stays within its size limit.

WHAT IS RAISED, AND WHEN. The rule the summary's writer grew over four review
rounds, now for every file:

- A fault that can be written down is written down, never raised. A failed
  capture is the summary's error and the manifest's; a record with no exact
  encoding costs the trace file, the reason in the manifest; an OSError a
  write handles is announced on stdout (``UNWRITABLE_MARKER``) and costs that
  file alone.
- An INTERRUPT, a BaseException that is not an Exception such as Ctrl+C or
  SystemExit, is never swallowed, but every file is attempted first: an
  interrupt that skipped the write took the finalization block with it
  (review round 3).
- A failure a write does not handle, such as a second Ctrl+C mid-write or
  stdout gone while an unwritable path is announced, is kept and raised once
  every file was attempted, never in an interrupt's PLACE: raised inside the
  child's ``finally``, the run's error was chained onto it, and the interrupt
  was nowhere in the chain (review round 4).
- Text made for a file comes from ``artifact_repr`` or ``deferred_repr``, and
  an interrupt that lands while it is made waits for the writes too
  (delivery step 5, review round 2).

So: interrupts and failed writes together raise ``INTERRUPTED_UNWRITTEN``:
first the interrupts held while a file's content or text was made, in the
order they landed, then the failed writes, in the order they failed, an
interrupt that lands mid-write among them. Interrupts alone raise one as
itself, several as ``INTERRUPTED_DESCRIBING``. Failed writes alone raise one
as itself, several as ``EVIDENCE_UNWRITTEN``, an ExceptionGroup when every one
is an Exception.

WHAT THE MANIFEST SAYS OF IT. It counts what is held for raising when it is
made (``held``): above zero, the evidence step raises, which D8.12 reads as
that step's failure. Its summary record names why there is no summary: a
write that failed, a capture that failed, an ending epoch close() did not
name, or a reduction that raised. The last was in the summary file alone, so
an ordinary error, held for no one, read ELIGIBLE (the review of the whole).
An interrupt that lands once the manifest is in place is not in it.
"""

from __future__ import annotations

import sys
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

# Importable as ``pixel_pn_determinism_evidence`` (how the teardown loads it)
# and as ``scripts.pixel_pn_determinism_evidence`` (how the tests import it).
# Only the first puts this directory on the path, and the sibling import below
# needs it.
SCRIPTS = str(Path(__file__).resolve().parent)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from navpy.exception_groups import (  # noqa: E402
    BaseExceptionGroup,
    ExceptionGroup,
)
from navpy.modules.vision.sim.determinism_evidence import (  # noqa: E402
    MANIFEST_NAME,
    SUMMARY_NAME,
    TRACE_NAME,
    evidence_manifest,
    file_record,
    sealed_capture,
    trace_lines,
    trace_record,
)
from navpy.modules.vision.sim.determinism_trace import (  # noqa: E402
    DeterminismTrace,
)
from pixel_pn_determinism_summary import (  # noqa: E402
    artifact_json,
    deferred_repr,
    held,
    summary_payload,
    write_replacing,
)

INTERRUPTED_UNWRITTEN = (
    "determinism evidence interrupted, and a write of its files failed"
)
INTERRUPTED_DESCRIBING = (
    "determinism evidence interrupted more than once while it was made"
)
EVIDENCE_UNWRITTEN = "determinism evidence: more than one file was not written"


def write_determinism_evidence(
    directory: Path,
    trace: DeterminismTrace | None,
    *,
    epoch: int | None,
    finalization: Mapping[str, Any],
    identity: Mapping[str, Any] | None,
) -> Path | None:
    """Write the three files. Return the manifest's path, the file a reader
    starts from, or None when it could not be written (announced).

    ``None`` for ``trace`` is tracing off, the production path: nothing is
    written. ``epoch`` is the one ``close()`` returned (``summary_payload``
    says why it must be). ``finalization`` is the teardown's, and
    ``identity`` the case's (D7b), None when there is none to give. What is
    raised, and when, is in the module docstring.
    """
    if trace is None:
        return None
    directory = Path(directory)
    # Every interrupt held while content or text was made, in the order they
    # landed, and every failure a write did not handle, an interrupt that
    # lands mid-write among them.
    interrupts: list[BaseException] = []
    failed: list[BaseException] = []
    capture, capture_error = held(lambda: sealed_capture(trace), interrupts)
    lines, trace_error = None, None
    if capture is not None:
        lines, trace_error = held(lambda: trace_lines(capture), interrupts)
    if lines is not None:
        trace_error = _attempt(
            directory / TRACE_NAME, lambda: lines.data, interrupts, failed
        )
    payload = summary_payload(
        capture,
        epoch=epoch,
        finalization=finalization,
        error=capture_error,
        interrupts=interrupts,
    )
    summary_error = _attempt(
        directory / SUMMARY_NAME,
        lambda: artifact_json(payload),
        interrupts,
        failed,
    )
    # Why there is no summary, when it was written without one. A write's
    # error is kept whatever its text, an empty one included.
    if summary_error is None and payload["summary"] is None:
        summary_error = payload["error"]
    manifest = directory / MANIFEST_NAME
    manifest_error = _attempt(
        manifest,
        lambda: artifact_json(evidence_manifest(
            capture,
            trace=trace_record(lines, trace_error),
            summary=file_record(SUMMARY_NAME, summary_error),
            identity=identity,
            finalization={**finalization, "epoch": epoch},
            error=capture_error,
            held=len(interrupts) + len(failed),
        )),
        interrupts,
        failed,
    )
    _raise(interrupts, failed)
    return manifest if manifest_error is None else None


def _attempt(
    path: Path,
    make: Callable[[], bytes],
    interrupts: list[BaseException],
    failed: list[BaseException],
) -> str | None:
    """Write one file: None once it is in place, else the text of why not.

    A failure the write did not handle is kept in ``failed`` for ``_raise``.
    Raised here, it would skip the files after it and could take an
    interrupt's place (review round 4).
    """
    try:
        return write_replacing(path, make)
    except BaseException as error:  # noqa: BLE001 - raised by _raise
        failed.append(error)
        return deferred_repr(error, interrupts)


def _raise(interrupts: list[BaseException], failed: list[BaseException]) -> None:
    """Raise what was held, by the rule in the module docstring."""
    if interrupts and failed:
        raise BaseExceptionGroup(INTERRUPTED_UNWRITTEN, [*interrupts, *failed])
    if len(interrupts) > 1:
        raise BaseExceptionGroup(INTERRUPTED_DESCRIBING, interrupts)
    if interrupts:
        raise interrupts[0]
    if len(failed) > 1:
        if all(isinstance(error, Exception) for error in failed):
            raise ExceptionGroup(EVIDENCE_UNWRITTEN, failed)
        raise BaseExceptionGroup(EVIDENCE_UNWRITTEN, failed)
    if failed:
        raise failed[0]


__all__ = [
    "EVIDENCE_UNWRITTEN",
    "INTERRUPTED_DESCRIBING",
    "INTERRUPTED_UNWRITTEN",
    "write_determinism_evidence",
]
