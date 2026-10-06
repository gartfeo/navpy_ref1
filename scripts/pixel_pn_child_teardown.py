"""Tear the direct-pixel child down so that no step can strand another.

WHY THIS EXISTS. The child's ``finally`` called each cleanup in turn, so the
first one to raise skipped every step after it. Review round 1 drove the real
``run()`` with the flight-control CSV write raising ``PermissionError``: that
write was the only cleanup that ran -- source, command worker, vehicle and
logger all stayed open -- and no determinism artifact was written (F2). The
same block also took the summary BEFORE the command worker stopped, so the
worker's last pass could record after it (F1).

So every step here is attempted whatever the others did, each failure is kept
with the name of its step, and the determinism evidence is written LAST, after
every other attempt, the logger's and the identity's included. Only then is
anything raised: one failure as itself, several grouped the way
``vehicle_lifecycle`` groups them. The run's own exception is not lost: this
runs inside the child's ``finally``, so Python chains it onto whatever is
raised here as ``__context__``.

WHAT THE EVIDENCE CAN VOUCH FOR is known only here, so the teardown supplies
it to ``pixel_pn_determinism_evidence``, which writes the files:

- ``epoch`` -- the leg that ended, exactly as ``close()`` returned it, never
  resolved from the rows. Unknown when close() raised; the summary then
  carries an error instead.
- ``worker`` -- ``stopped`` only when ``navigation.stop()`` returned normally,
  which it does only once no command-worker thread is alive, including when
  none was ever started (navigation_command_session.py:99-124).
  ``not_confirmed`` means stop did not return normally; it is not proof the
  worker still runs. ``never_started`` means no Navigation was ever built.
- ``mavlink_quiescence`` -- always ``unverified``. ``vehicle.close()`` joins
  the MAVLink reader with a timeout and never checks the outcome
  (mav_bus_lifecycle.py:91, mav_bus_reader.py:49-51), so a callback still in
  flight can reach a store after the evidence sealed it. It is refused and
  counted (D5): it changes no file, and its count is only an observation. A
  known limit of the vehicle layer, recorded here rather than fixed.
- ``teardown_errors`` -- every step that raised before the evidence, by name,
  in order. An error whose text cannot be made reads ``REPR_FAILED``. An
  interrupt that lands while it is made fails the evidence step itself: it
  is raised with the rest once the evidence was attempted, and listed at
  the end as that step's entry, in the fixed text ``REPORT_INTERRUPTED``.
  The evidence's other failures are its files' errors, and the manifest
  counts what its writer holds for raising (``held``). Either tells D8.12
  that the evidence step failed (delivery step 6's review).
- ``admission`` -- the ATTITUDE ledger subscription's evidence
  (``pixel_pn_admission_ledger``), read once the vehicle is closed and the
  subscription cancelled, in a step of its own, so an interrupt there costs
  the evidence and never the files. None when there was no subscription
  to read, or its read raised; a step's error is in ``teardown_errors``.
- ``identity`` -- the case's (D7b, ``pixel_pn_run_identity``), finished in a
  step of its own once the logger closed, which hashes the code's files on
  disk again. None with tracing off, or when that step raised.

So the summary's ``complete`` means recording integrity at the captured
boundary. It does not certify a finished leg.
"""

from __future__ import annotations

import sys
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Protocol

# Importable as ``pixel_pn_child_teardown`` (how the child loads it) and as
# ``scripts.pixel_pn_child_teardown`` (how the tests import it). Only the first
# puts this directory on the path, and the sibling imports below need it.
SCRIPTS = str(Path(__file__).resolve().parent)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from navpy.exception_groups import (  # noqa: E402
    BaseExceptionGroup,
    ExceptionGroup,
)
from navpy.modules.vision.sim.determinism_evidence import (  # noqa: E402
    EVIDENCE_STEP,
    SOURCE_CLOSE_STEP,
    WORKER_STOPPED,
)
from navpy.modules.vision.sim.determinism_trace import (  # noqa: E402
    DeterminismTrace,
)
from pixel_pn_determinism_evidence import (  # noqa: E402
    write_determinism_evidence,
)
from pixel_pn_determinism_summary import deferred_repr  # noqa: E402

# WORKER_STOPPED, the one worker state D8.11 accepts, is the evidence
# module's, as are the two steps D8.12 reads, SOURCE_CLOSE_STEP and
# EVIDENCE_STEP: the reader and this teardown name them alike.
WORKER_NOT_CONFIRMED = "not_confirmed"
WORKER_NEVER_STARTED = "never_started"
MAVLINK_QUIESCENCE_UNVERIFIED = "unverified"
TEARDOWN_FAILED = "direct-pixel child teardown failed"
# The evidence step's own entry in the report, one per interrupt that landed
# while a step's error was described: fixed text, so nothing can stop it
# being made (delivery step 6's review).
REPORT_INTERRUPTED = "<interrupted while a step's error was described>"

# What a step returns when it raised. A sentinel rather than None: a close()
# that returns None has not failed, it has only not named its epoch.
_RAISED = object()
# The last two steps: the identity's, then the evidence's (EVIDENCE_STEP),
# in which the report of every other step is made.
_IDENTITY_STEP = "identity.finish"


class _Writes(Protocol):
    def write(self) -> object: ...


class _Closes(Protocol):
    def close(self) -> object: ...


class _Stops(Protocol):
    def stop(self) -> object: ...


class _Admission(Protocol):
    def cancel(self) -> object: ...

    def evidence(self) -> object: ...


class _Identity(Protocol):
    def finish(self) -> Mapping[str, object]: ...


class _TracedSource(Protocol):
    @property
    def determinism_trace(self) -> DeterminismTrace | None: ...

    def close(self) -> int | None: ...


class _Attempts:
    """Run each step whatever the others did; keep every failure by name."""

    def __init__(self) -> None:
        self.failures: list[tuple[str, BaseException]] = []

    def run(self, step: str, action: Callable[[], object]) -> object:
        """The step's own result, or ``_RAISED`` once its error is kept.

        BaseException, not Exception: a Ctrl+C in one step must not strand
        the steps after it any more than an ordinary fault may. It is raised
        again, with the rest, once every step has run.
        """
        try:
            return action()
        except BaseException as error:  # noqa: BLE001 - raised by finish()
            self.failures.append((step, error))
            return _RAISED

    def report(self) -> list[dict[str, str]]:
        """Every failure so far, by step, as the evidence records it.

        Made inside the evidence step. An interrupt that lands while an
        error is described is kept as that step's failure once the report
        is made, and ``finish()`` raises it with the rest; the entry it
        landed in reads ``REPR_FAILED``. Raised where it landed, it kept the
        evidence from being attempted at all (delivery step 5, review round
        2). Each is listed last, as the evidence step's, in the fixed text
        ``REPORT_INTERRUPTED``, never described: an interrupted description
        and a failed one left the same files, so D8.12 could not see the
        evidence step fail (delivery step 6's review).
        """
        interrupts: list[BaseException] = []
        entries = [
            {"step": step, "error": deferred_repr(error, interrupts)}
            for step, error in self.failures
        ]
        self.failures += [(EVIDENCE_STEP, error) for error in interrupts]
        return entries + [
            {"step": EVIDENCE_STEP, "error": REPORT_INTERRUPTED}
            for _ in interrupts
        ]

    def finish(self) -> None:
        """Raise what failed: one failure as itself, several grouped."""
        errors = [error for _, error in self.failures]
        if not errors:
            return
        if len(errors) == 1:
            raise errors[0]
        if all(isinstance(error, Exception) for error in errors):
            raise ExceptionGroup(TEARDOWN_FAILED, errors)
        raise BaseExceptionGroup(TEARDOWN_FAILED, errors)


def finish_child(
    directory: Path,
    *,
    flight_trace: _Writes,
    source: _TracedSource | None,
    navigation: _Stops | None,
    cadence: _Closes | None,
    vehicle: _Closes | None,
    admission: _Admission | None,
    logger: _Closes,
    identity: _Identity | None,
) -> None:
    """The child's teardown: each step in order, then the evidence, then raise.

    ``None`` is a resource that was never built and has nothing to close.
    ``flight_trace`` and ``logger`` exist before the child's ``try`` starts.
    ``admission`` is cancelled once the vehicle is closed: the reader that
    makes its rulings has been asked to stop, though not confirmed stopped
    (``mavlink_quiescence``). ``identity`` is None with tracing off.
    """
    attempts = _Attempts()
    attempts.run("flight_control_trace.write", flight_trace.write)
    ending: object = None
    if source is not None:
        ending = attempts.run(SOURCE_CLOSE_STEP, source.close)
    worker = WORKER_NEVER_STARTED
    if navigation is not None:
        stopped = attempts.run("navigation.stop", navigation.stop) is not _RAISED
        worker = WORKER_STOPPED if stopped else WORKER_NOT_CONFIRMED
    if cadence is not None:
        attempts.run("cadence.close", cadence.close)
    if vehicle is not None:
        attempts.run("vehicle.close", vehicle.close)
    evidence: object = None
    if admission is not None:
        attempts.run("admission.cancel", admission.cancel)
        evidence = attempts.run("admission.evidence", admission.evidence)
    attempts.run("logger.close", logger.close)
    if source is not None:
        _write_evidence(
            directory,
            attempts,
            source,
            identity,
            epoch=None if ending is _RAISED else ending,
            worker=worker,
            admission=None if evidence is _RAISED else evidence,
        )
    attempts.finish()


def _write_evidence(
    directory: Path,
    attempts: _Attempts,
    source: _TracedSource,
    identity: _Identity | None,
    *,
    epoch: object,
    worker: str,
    admission: object,
) -> None:
    """The last two steps: the identity finished, then the evidence written
    with it, the report of every other step made inside the second."""
    given: object = None
    if identity is not None:
        given = attempts.run(_IDENTITY_STEP, identity.finish)
    attempts.run(
        EVIDENCE_STEP,
        lambda: write_determinism_evidence(
            directory,
            source.determinism_trace,
            epoch=epoch,
            finalization={
                "worker": worker,
                "mavlink_quiescence": MAVLINK_QUIESCENCE_UNVERIFIED,
                "teardown_errors": attempts.report(),
                "admission": admission,
            },
            identity=None if given is _RAISED else given,
        ),
    )


__all__ = [
    "MAVLINK_QUIESCENCE_UNVERIFIED",
    "REPORT_INTERRUPTED",
    "TEARDOWN_FAILED",
    "WORKER_NEVER_STARTED",
    "WORKER_NOT_CONFIRMED",
    "WORKER_STOPPED",
    "finish_child",
]
