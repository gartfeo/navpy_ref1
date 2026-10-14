"""Whether a managed executor job's failure reached anyone (``baton_asyncio``).

A job's asyncio future is a :class:`JobFuture`. Its ``await`` records that
the awaiting coroutine took the failure: a task cancelled in the same loop
iteration the failure arrived in (a request's deadline) wakes with
``CancelledError`` instead, and has not. A failure is decided two loop
iterations after it arrives, after the awaiter's wake-up ran:

- cancelled future (its awaiter left before it ended): unobserved;
- awaited: observed only if the coroutine took it;
- neither (a ``gather``, a callback, or nobody yet): decided at
  :meth:`JobFailures.retire`, by whether anyone retrieved it.

The owner's ``report(error, observed)`` decides what is a fault.
"""

from __future__ import annotations

import asyncio
import concurrent.futures
from asyncio import futures as _futures
from collections.abc import Callable, Generator
from typing import Any

Report = Callable[[BaseException, bool], None]


class JobFuture(asyncio.Future):
    """An asyncio future whose ``await`` records what its coroutine took."""

    awaited = False
    taken = False

    def __await__(self) -> Generator[Any, None, Any]:
        self.awaited = True
        try:
            return (yield from super().__await__())
        except asyncio.CancelledError:
            raise  # the coroutine took a cancel, not the job's outcome
        except BaseException:
            self.taken = True
            raise

    __iter__ = __await__


class JobFailures:
    def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop
        self.report: Report | None = None
        self._undecided: list[tuple[BaseException, JobFuture]] = []

    def wrap(self, source: concurrent.futures.Future) -> JobFuture:
        wrapper = JobFuture(loop=self._loop)
        _futures._chain_future(source, wrapper)  # asyncio's own result copy, queued first
        source.add_done_callback(lambda done: self._arrived(done, wrapper))
        return wrapper

    def _arrived(self, source: concurrent.futures.Future, wrapper: JobFuture) -> None:
        if source.cancelled() or source.exception() is None or self.report is None:
            return
        error = source.exception()
        if self._loop.is_closed():
            self.report(error, False)
        else:  # after the copy; then once more, after the awaiter's wake-up
            self._loop.call_soon_threadsafe(
                lambda: self._loop.call_soon(lambda: self._decide(error, wrapper)))

    def _decide(self, error: BaseException, wrapper: JobFuture) -> None:
        if wrapper.cancelled():
            self.report(error, False)
        elif wrapper.awaited:
            self.report(error, wrapper.taken)
        else:
            self._undecided.append((error, wrapper))

    def retire(self) -> None:
        """At shutdown: a failure nobody awaited is observed only if retrieved."""
        undecided, self._undecided = self._undecided, []
        for error, wrapper in undecided:
            self.report(error, wrapper.taken or not getattr(wrapper, "_log_traceback", True))


__all__ = ["JobFailures", "JobFuture"]
