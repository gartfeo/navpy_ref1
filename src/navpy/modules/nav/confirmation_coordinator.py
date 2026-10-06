"""Worker lifetime coordination for target confirmation."""

from __future__ import annotations

import threading
from typing import Callable, Optional

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.confirmation_ports import (
    ConfirmationMessageBroadcaster,
    ConfirmationWorkerStatePort,
)
from navpy.modules.nav.confirmation_round_runner import ConfirmationRoundRunner
from navpy.modules.nav.confirmation_round_transaction import ConfirmationWorkerLease
from navpy.modules.nav.self_assignment_publisher import SelfAssignmentPublisher
from navpy.modules.nav.confirmation_registry_state import ConfirmationStatus
from navpy.modules.vision.models.detect_data import DetectedObject


class ConfirmationCoordinator:
    """Start and retire confirmation workers around exact state leases."""

    def __init__(
        self,
        *,
        sys_id: int,
        state: ConfirmationWorkerStatePort,
        network: Callable[[], Optional[ConfirmationMessageBroadcaster]],
        auto_confirm: Callable[[], bool],
        assignment: SelfAssignmentPublisher,
        round_runner: ConfirmationRoundRunner,
        logger: ILogger,
        event_factory: Callable[[], threading.Event],
        thread_factory: Callable[..., threading.Thread],
    ) -> None:
        self._sys_id = sys_id
        self._state = state
        self._network = network
        self._auto_confirm = auto_confirm
        self._assignment = assignment
        self._round_runner = round_runner
        self._logger = logger
        self._event_factory = event_factory
        self._thread_factory = thread_factory

    def review(self, targets: list[DetectedObject]) -> None:
        for target in targets:
            worker = self._state.start_review(target, self._event_factory)
            self._thread_factory(
                target=self._run,
                args=(target, worker),
                daemon=True,
            ).start()

    def _run(
        self,
        target: DetectedObject,
        worker: ConfirmationWorkerLease,
    ) -> None:
        try:
            target_id = worker.target_id
            if target_id is None or not self._state.worker_is_current(worker):
                return
            self._assignment.publish(target)
            network = self._network()
            if self._auto_confirm() or network is None:
                if not self._state.set_worker_status(
                    worker,
                    ConfirmationStatus.CONFIRMED,
                ):
                    return
                reason = "auto-confirmed" if self._auto_confirm() else "no network"
                self._logger.info(
                    f"T{target_id} confirmed ({reason}).",
                    key="nav_state",
                    dest=LogStatusDest.DRONE,
                )
                return
            self._logger.info(
                f"Asking confirmation for: {target_id}",
                key="nav_state",
                status=f"T{target_id} confirming",
                dest=LogStatusDest.DRONE,
            )
            self._round_runner.run(target, worker)
        finally:
            self._state.finish_worker(worker)


__all__ = ["ConfirmationCoordinator"]
