"""Lifecycle cleanup for task-auction generations."""

from __future__ import annotations

from navpy.exception_groups import ExceptionGroup
from navpy.logger.cache_logger import ILogger
from navpy.modules.swarm.task_auction_models import _TaskAuctionStore
from navpy.modules.swarm.task_dispatch import TaskDispatch


class TaskAuctionLifecycle:
    """Fence one shared auction store before cleaning its dispatches."""

    def __init__(self, store: _TaskAuctionStore) -> None:
        self._store = store

    def reset(self, logger: ILogger) -> None:
        with self._store.lock:
            if self._store.closed:
                return
            self._store.generation += 1
            dispatches = list(self._store.dispatches.values())
            self._store.dispatches.clear()
            self._store.peers.clear()
        _shutdown_dispatches(dispatches, logger)

    def shutdown(self, logger: ILogger) -> None:
        with self._store.lock:
            if self._store.closed:
                return
            self._store.closed = True
            self._store.generation += 1
            dispatches = list(self._store.dispatches.values())
            self._store.dispatches.clear()
            self._store.peers.clear()
        _shutdown_dispatches(dispatches, logger)


def _shutdown_dispatches(
    dispatches: list[TaskDispatch],
    logger: ILogger,
) -> None:
    errors: list[Exception] = []
    for dispatch in dispatches:
        try:
            dispatch.shutdown()
        except Exception as exc:  # noqa: BLE001 - cleanup must continue
            logger.error(f"TaskDispatch shutdown error: {exc}")
            errors.append(exc)
    if errors:
        raise ExceptionGroup("Task dispatch cleanup failed", errors)


__all__ = ["TaskAuctionLifecycle"]
