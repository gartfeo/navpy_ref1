"""The tick's one shared exchange deadline (``lockstep_fleet.md``, Fleet barrier, step 3).

Each SITL arms its frame-k reply deadline (the step deadline) when it sends
frame k, and it sends frame k only after it received reply k-1. So every
firmware deadline of tick k is at or after ``a_k + step``, where ``a_k`` is
the moment reply k-1 went to SITL 1, the first reply of the previous tick
(for k = 1, SITL 1's ``go``). Tick k therefore has one absolute wall
deadline::

    D_k = a_k + step_deadline_s - margin_s

``margin_s`` (``EXCHANGE_MARGIN_S`` of the deployment) covers a reply's
transit to the firmware: the relay's largest injected delay plus a transit
allowance.

The coordinator's waits within a tick add up: the frame reads, the ``TICK``
sends, each ``DONE`` and what applying it holds for (a spawn's registration,
a retirement's exit), the operator's baton and the reply sends. Every one
draws on ``D_k``: its budget is ``min(own, D_k - now)`` (:meth:`budget`), a
wait that fails once ``D_k`` has passed is the deadline's fault
(:meth:`raise_if_spent`), and a wait that returns after it is too
(:meth:`check`). Running out is ``HostFault("exchange_budget")``, journaled
with the tick and the sub-wait; the run then stops by ownership before the
firmware deadline could turn it into a SITL death.

Outside a SITL exchange (initial spawns before any SITL runs, drain after
the parked frames) no deadline is open and each wait keeps its own budget.
An infinite step deadline (the default) never binds: per-operation budgets
only, as the slice ran before step 10.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable

from navpy.sim.lockstep.coordinator_types import HostFault

#: The coordinator's sub-waits within one tick, by the name the fault journals.
FRAME, TICK, DONE, REGISTER, EXIT, OPERATOR, REPLY, LAUNCH = (
    "frame", "tick", "done", "register", "exit", "operator", "reply", "launch")


class ExchangeDeadline:
    def __init__(self, step_deadline_s: float = math.inf, margin_s: float = 0.0, *,
                 clock: Callable[[], float] = time.monotonic) -> None:
        if not 0.0 <= margin_s < step_deadline_s:
            raise ValueError(f"exchange margin {margin_s} s outside [0, {step_deadline_s}) s")
        self._step_s = step_deadline_s
        self._margin_s = margin_s
        self._clock = clock
        self._anchor: float | None = None
        self._k: int | None = None
        self._due: float | None = None

    @property
    def due(self) -> float | None:
        """``D_k`` of the open tick, on this deadline's clock; None if none is open."""
        return self._due

    def anchor(self) -> None:
        """``a_{k+1}``: reply k just went to SITL 1 (or, before frame 1, SITL 1's ``go``)."""
        self._anchor = self._clock()

    def open(self, k: int) -> None:
        """Tick k's exchange begins; its deadline counts from the last anchor."""
        self._k = k
        if math.isinf(self._step_s):
            self._due = None
            return
        if self._anchor is None:
            raise HostFault("exchange_unanchored", f"tick {k} opened before SITL 1's go", tick=k)
        self._due = self._anchor + self._step_s - self._margin_s

    def close(self) -> None:
        """No SITL exchange is in progress (drain): waits keep their own budgets."""
        self._k = self._due = None

    def budget(self, wait: str, own_s: float) -> float:
        """Seconds the sub-wait may take: ``min(own, D_k - now)``; none left is the fault."""
        if self._due is None:
            return own_s
        left = self._due - self._clock()
        if left <= 0.0:
            raise self.fault(wait)
        return min(own_s, left)

    def spent(self) -> bool:
        return self._due is not None and self._clock() >= self._due

    def check(self, wait: str) -> None:
        """After a sub-wait returned: past ``D_k`` is the fault, however it ended."""
        if self.spent():
            raise self.fault(wait)

    def raise_if_spent(self, wait: str, error: BaseException | None = None) -> None:
        """A sub-wait failed: once ``D_k`` has passed, the shared deadline bound it."""
        if self.spent():
            raise self.fault(wait) from error

    def fault(self, wait: str) -> HostFault:
        return HostFault("exchange_budget", f"tick {self._k}: the exchange deadline ran out "
                         f"in the {wait} wait", tick=self._k, context={"wait": wait})


__all__ = ["DONE", "EXIT", "ExchangeDeadline", "FRAME", "LAUNCH", "OPERATOR", "REGISTER", "REPLY",
           "TICK"]
