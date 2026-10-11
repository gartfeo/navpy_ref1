"""The admission tap: every ruling the router makes, numbered, for a recorder.

The inbound router rules on every autopilot telemetry message from its target
system: it admits the message, or rejects it with a reason. A recorder that
has to know which ATTITUDE the associator acted on needs every one of those
rulings, the rejections too, and has to be able to tell a ruling it did not
receive from one that was never made. The callback registry gives neither: it
dispatches admitted messages only, and one raising callback followed by a
raising logger stops the later callbacks for that message.

So the router hands each ruling here after its verdict, before it publishes
the message or dispatches a callback. Each message type has its own ruling
numbers, 1, 2, 3 and on. A subscription is owed exactly the rulings
``first``..``end``: ``first`` is fixed when it subscribes, ``end`` when it
cancels. One small lock, a leaf, orders numbering a ruling together with
taking that type's subscriptions, and subscribe and cancel, so a subscription
is in the snapshot of ruling n exactly when first <= n <= end. The lock is
never held while a ruling is built or dispatched.

A subscriber receives an immutable ruling of plain values, never the message.
A ruling that could not be built, or not dispatched, is a fault, flagged and
counted on every subscription it cost. Nothing raises into the router but an
interrupt: wherever it lands once the ruling is numbered, it flags every
subscription it kept from the ruling and is passed on, uncounted, since its
path takes no lock. With no subscriber a ruling costs one uncontended lock
acquisition, one increment and one emptiness check.

The design is D1 of the LANDING2 step-1 plan.
"""

from __future__ import annotations

import contextlib
import threading
from collections.abc import Callable, Collection
from contextlib import AbstractContextManager
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class AdmissionRuling:
    """One ruling, in plain values only: nothing in it reaches the message."""

    message_type: str
    ruling: int
    time_boot_ms: int | None
    accepted: bool
    reason: str | None


RulingCallback = Callable[[AdmissionRuling], None]


class AdmissionSubscription:
    """One subscriber's claim on the rulings of one message type.

    Owed exactly the rulings ``first``..``end``, and ``end`` is None until
    ``cancel``. A ruling it was owed and did not get is a fault: ``faulted`` is
    set first, a plain store that nothing clears, and ``faults`` then counts
    it under the tap's lock, unless an interrupt cost it, whose path takes no
    lock. So ``faulted`` is the one to read: a count that failed, or never
    ran, still leaves it set, and a lost ruling never reads as none.
    """

    __slots__ = (
        "message_type",
        "first",
        "_callback",
        "_lock",
        "_close",
        "_end",
        "_faults",
        "_faulted",
    )

    def __init__(
        self,
        message_type: str,
        first: int,
        callback: RulingCallback,
        lock: AbstractContextManager[Any],
        close: Callable[[AdmissionSubscription], None],
    ) -> None:
        self.message_type = message_type
        self.first = first
        self._callback = callback
        self._lock = lock
        self._close = close
        self._end: int | None = None
        self._faults = 0
        self._faulted = False

    @property
    def end(self) -> int | None:
        with self._lock:
            return self._end

    @property
    def faults(self) -> int:
        with self._lock:
            return self._faults

    @property
    def faulted(self) -> bool:
        return self._faulted

    def cancel(self) -> None:
        """Be owed nothing after the ruling made last. Idempotent."""
        self._close(self)


class AdmissionTap:
    """One per router, created by it: numbers every ruling and hands it on.

    ``stamp_of`` reads a message's boot stamp as the router reads it, and
    ``ruled_types`` are the message types the router rules on.
    """

    def __init__(
        self,
        stamp_of: Callable[[Any], int | None],
        ruled_types: Collection[str],
    ) -> None:
        self._stamp_of = stamp_of
        self._ruled_types = frozenset(ruled_types)
        # Numbering, the subscriptions, subscribe and cancel: a leaf, never
        # held while a ruling is built or dispatched.
        self._lock = threading.Lock()
        self._numbers: dict[str, int] = {}
        self._owed: dict[str, tuple[AdmissionSubscription, ...]] = {}

    def subscribe(
        self,
        message_type: str,
        callback: RulingCallback,
    ) -> AdmissionSubscription:
        """Every ruling on ``message_type`` after the one made last.

        Only a type the router rules on: a subscription to any other would be
        owed nothing, and would read as complete.
        """
        if message_type not in self._ruled_types:
            raise ValueError(f"the router does not rule on {message_type!r}")
        with self._lock:
            subscription = AdmissionSubscription(
                message_type,
                self._numbers.get(message_type, 0) + 1,
                callback,
                self._lock,
                self._close,
            )
            self._owed[message_type] = (
                *self._owed.get(message_type, ()),
                subscription,
            )
        return subscription

    def rule(
        self,
        message_type: str,
        message: Any,
        accepted: bool,
        reason: str | None,
    ) -> None:
        """Number one ruling, then hand it to each subscription owed it.

        The router calls it after its verdict and before it publishes or
        dispatches anything, on its bus's one reader thread. From the moment
        the ruling is numbered, one guard covers it: an interrupt landing on
        any line, a fault's count included, flags every subscription not yet
        handed the ruling and is passed on. An ordinary error it lets
        through, a failed allocation, flags the same and stays here.
        """
        owed: tuple[AdmissionSubscription, ...] = ()
        handed: list[AdmissionSubscription] | tuple[()] = ()
        try:
            with self._lock:
                # The snapshot first: once ruling n exists, so does the list
                # of the subscriptions owed it.
                owed = self._owed.get(message_type, ())
                ruling = self._numbers.get(message_type, 0) + 1
                self._numbers[message_type] = ruling
            if not owed:
                return
            handed = []
            built = self._build(
                message_type, ruling, message, accepted, reason
            )
            if built is None:
                self._count(owed)
                return
            self._dispatch(owed, built, handed)
        except BaseException as error:
            # Flags only, plain stores, and no lock: so the interrupt is
            # passed on even where Python has left the lock held, as when a
            # trace function raises on a with statement's exit line, before
            # __exit__. It never misses a subscription that lost the ruling;
            # it may flag one that lost nothing, when it lands between the
            # snapshot and the number, or between a dispatch and its note.
            for subscription in owed:
                if subscription not in handed:
                    subscription._faulted = True
            # Nothing but an interrupt raises into the router: an error
            # here costs the tap its ruling, never the router its message.
            if not isinstance(error, Exception):
                raise

    def _build(
        self,
        message_type: str,
        ruling: int,
        message: Any,
        accepted: bool,
        reason: str | None,
    ) -> AdmissionRuling | None:
        """The ruling in plain values, or None if it cannot be built. A
        method of its own, not a try nested in ``rule``'s guard: on CPython
        3.11 that try's own line compiled to an instruction outside every
        handler, and an interrupt landing on it passed the guard by."""
        try:
            return AdmissionRuling(
                message_type, ruling, self._stamp_of(message), accepted, reason
            )
        except Exception:  # noqa: BLE001 - nothing raises into the router
            return None

    def _dispatch(
        self,
        owed: tuple[AdmissionSubscription, ...],
        built: AdmissionRuling,
        handed: list[AdmissionSubscription],
    ) -> None:
        """Each dispatch in its own guard, and each subscription handed the
        ruling goes into ``handed``. A failed dispatch costs its own
        subscription. Nothing escapes but an interrupt."""
        for subscription in owed:
            try:
                subscription._callback(built)
            except Exception:  # noqa: BLE001 - nothing raises into the router
                self._count((subscription,))
                continue
            handed.append(subscription)

    def _count(self, cost: tuple[AdmissionSubscription, ...]) -> None:
        """Flag each subscription first, a plain store, then count it under
        the lock: a count that fails leaves the flags to say it."""
        for subscription in cost:
            subscription._faulted = True
        with contextlib.suppress(Exception):
            with self._lock:
                for subscription in cost:
                    subscription._faults += 1

    def _close(self, subscription: AdmissionSubscription) -> None:
        """``cancel``: ``end`` is the ruling made last, in the acquisition
        that takes the subscription out of every later snapshot."""
        with self._lock:
            if subscription._end is not None:
                return
            message_type = subscription.message_type
            subscription._end = self._numbers.get(message_type, 0)
            remaining = tuple(
                owed
                for owed in self._owed.get(message_type, ())
                if owed is not subscription
            )
            if remaining:
                self._owed[message_type] = remaining
            else:
                self._owed.pop(message_type, None)


__all__ = [
    "AdmissionRuling",
    "AdmissionSubscription",
    "AdmissionTap",
    "RulingCallback",
]
