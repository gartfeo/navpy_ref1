"""Subscribe the determinism trace's ATTITUDE ledger to the vehicle's router.

The ledger (``determinism_admission_ledger``) records every ruling the router
makes on ATTITUDE, admissions and rejections, as its admission tap hands them
on. The child subscribes it here after the source is built and BEFORE
``source.start()``, so it is owed every ruling the source's own subscriptions
can see (D2 and D8.5 of the LANDING2 step-1 plan).

What the ledger holds is half the evidence. The other half is what the
subscription says it was OWED: ``first`` and ``end`` bound the rulings owed,
``faults`` counts the deliveries that failed, and ``faulted`` says one was
lost, an interrupt's included, which the tap flags and cannot count (D8.6).
The teardown cancels the subscription once the vehicle is closed, then reads
this evidence into the determinism artifact.

One figure is the link's own: ``window_rows``, how many window rows the
journal held once the subscription was in place. The source writes its
OPENED row as it starts, so 0 shows the ledger was subscribed before the
source's window opened (D8.5); a row already there shows it was not, and
None that the journal could not be read then.

Record-only: a vehicle without the tap, or a subscription that raises, costs
the verdict and never the flight. So does an error whose text cannot be made:
every error here is written as ``artifact_repr`` makes it. Imported, never run.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Protocol

# Importable as ``pixel_pn_admission_ledger`` (how the child loads it) and as
# ``scripts.pixel_pn_admission_ledger`` (how the tests import it). Only the
# first puts this directory on the path, and the sibling import below needs it.
SCRIPTS = str(Path(__file__).resolve().parent)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from navpy.modules.vision.sim.determinism_events import (  # noqa: E402
    EVENT_SUBSCRIPTION,
)
from navpy.modules.vision.sim.determinism_trace import (  # noqa: E402
    DeterminismTrace,
)
from pixel_pn_determinism_summary import artifact_repr  # noqa: E402

ADMISSION_MESSAGE_TYPE = "ATTITUDE"
NO_ADMISSION_TAP = "the vehicle has no admission tap"


class _Subscription(Protocol):
    @property
    def first(self) -> int: ...

    @property
    def end(self) -> int | None: ...

    @property
    def faults(self) -> int: ...

    @property
    def faulted(self) -> bool: ...

    def cancel(self) -> None: ...


class _Admits(Protocol):
    def on_admission(self, message_name: str, callback: Any) -> Any: ...


class AdmissionLedgerLink:
    """The ledger's subscription, or the reason there is none."""

    __slots__ = ("_subscription", "_error", "_window_rows")

    def __init__(
        self,
        subscription: _Subscription | None,
        error: str | None,
        window_rows: int | None = None,
    ) -> None:
        self._subscription = subscription
        self._error = error
        self._window_rows = window_rows

    def cancel(self) -> None:
        """Be owed nothing more, which fixes ``end``. Without a subscription
        there is nothing to cancel."""
        if self._subscription is not None:
            self._subscription.cancel()

    def evidence(self) -> dict[str, Any]:
        """The subscription's bounds and fault flags, for the artifact.

        ``live`` says whether a subscription exists at all, and
        ``window_rows`` is the link's own, read as it subscribed. A read of
        the subscription that raises is reported as ``error``, with none of
        its figures claimed; only an interrupt passes.
        """
        subscription = self._subscription
        if subscription is None:
            return _evidence(False, error=self._error)
        try:
            return _evidence(
                True,
                first=subscription.first,
                end=subscription.end,
                faults=subscription.faults,
                faulted=subscription.faulted,
                window_rows=self._window_rows,
            )
        except Exception as error:  # noqa: BLE001 - the artifact still gets written
            return _evidence(
                True, error=artifact_repr(error), window_rows=self._window_rows
            )


def _evidence(
    live: bool,
    *,
    first: int | None = None,
    end: int | None = None,
    faults: int | None = None,
    faulted: bool | None = None,
    error: str | None = None,
    window_rows: int | None = None,
) -> dict[str, Any]:
    return {
        "live": live,
        "first": first,
        "end": end,
        "faults": faults,
        "faulted": faulted,
        "error": error,
        "window_rows": window_rows,
    }


def subscribe_admission_ledger(
    vehicle: _Admits, trace: DeterminismTrace | None
) -> AdmissionLedgerLink | None:
    """Subscribe ``trace.ledger.append`` to every later ATTITUDE ruling.

    None when tracing is off: there is no ledger to fill. A vehicle whose
    ``on_admission`` returns None has no tap, and one that raises is reported
    by its error; either way the link says why, and its evidence is not live.
    An interrupt is not caught.
    """
    if trace is None:
        return None
    try:
        subscription = vehicle.on_admission(
            ADMISSION_MESSAGE_TYPE, trace.ledger.append
        )
    except Exception as error:  # noqa: BLE001 - record-only, never the flight
        return AdmissionLedgerLink(None, artifact_repr(error))
    if subscription is None:
        return AdmissionLedgerLink(None, NO_ADMISSION_TAP)
    return AdmissionLedgerLink(subscription, None, _window_rows(trace))


def _window_rows(trace: DeterminismTrace) -> int | None:
    """The window rows in the journal now. Read once the subscription is in
    place, so a row already there was written by a source that opened its
    window before the ledger could be owed a ruling. None when the read
    fails: the order it would show is then unknown.

    The journal's own read, never the trace's ``capture()``: that one is
    the post-leg read of every store, and the evidence is made from it.
    """
    try:
        rows, _ = trace.journal.capture()
    except Exception:  # noqa: BLE001 - record-only, never the flight
        return None
    return sum(1 for row in rows if row[0] == EVENT_SUBSCRIPTION)


__all__ = [
    "ADMISSION_MESSAGE_TYPE",
    "NO_ADMISSION_TAP",
    "AdmissionLedgerLink",
    "subscribe_admission_ledger",
]
