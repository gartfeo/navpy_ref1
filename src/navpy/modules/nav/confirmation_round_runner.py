"""One generation-fenced operator-confirmation request round."""

from __future__ import annotations

import threading
from typing import Callable, Optional

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.task_confirmation_msg import TaskConfirmRequestMsg
from navpy.modules.comm.messages.task_message_data import (
    TaskMsgData,
)
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.types import class_to_task_type
from navpy.modules.nav.confirmation_dependencies import ConfirmationFailurePolicy
from navpy.modules.nav.confirmation_ports import (
    ConfirmationMessageBroadcaster,
    ConfirmationRoundStatePort,
)
from navpy.modules.nav.confirmation_round_transaction import (
    ConfirmationRequestRef,
    ConfirmationRound,
    ConfirmationWorkerLease,
)
from navpy.modules.nav.confirmation_media import ConfirmationMedia
from navpy.modules.nav.confirmation_registry_state import ConfirmationStatus
from navpy.modules.vision.models.detect_data import DetectedObject


# ConnectionError and TimeoutError are OSError subclasses. Programmer and
# signature defects must escape the worker instead of becoming radio failures.
_OUTBOUND_ERRORS = (OSError,)


class ConfirmationRoundRunner:
    """Own one request deadline and its request-only resend cadence."""

    def __init__(
        self,
        *,
        sys_id: int,
        state: ConfirmationRoundStatePort,
        network: Callable[[], Optional[ConfirmationMessageBroadcaster]],
        wait_time_s: Callable[[], float],
        resend_interval_s: Callable[[], float],
        failure_policy: ConfirmationFailurePolicy,
        media: ConfirmationMedia,
        logger: ILogger,
        monotonic: Callable[[], float],
        event_factory: Callable[[], threading.Event],
    ) -> None:
        self._sys_id = sys_id
        self._state = state
        self._network = network
        self._wait_time_s = wait_time_s
        self._resend_interval_s = resend_interval_s
        self._failure_policy = failure_policy
        self._media = media
        self._logger = logger
        self._monotonic = monotonic
        self._event_factory = event_factory

    def run(
        self,
        target: DetectedObject,
        worker: ConfirmationWorkerLease,
    ) -> None:
        network = self._network()
        if network is None:
            self._state.set_worker_status(worker, ConfirmationStatus.CONFIRMED)
            return
        request = self._request_message(target, worker.target_id)
        request.set_meta_from_provider()
        confirmation = self._state.begin_round(
            target,
            worker,
            self._event_factory,
            ConfirmationRequestRef.from_meta(request.meta),
        )
        if confirmation is None:
            return
        try:
            try:
                self._logger.info(
                    f"Sending confirm request for T{confirmation.target_id}.",
                    key="nav_state",
                    status=(
                        f"T{confirmation.target_id} requesting confirmation"
                    ),
                    dest=LogStatusDest.DRONE,
                )
                if not self._state.send_if_current(
                    confirmation,
                    lambda: network.broadcast(request),
                ):
                    return
                first_send = self._monotonic()
                if not self._state.send_if_current(
                    confirmation,
                    lambda: self._media.send(target, confirmation.target_id),
                ):
                    return
            except _OUTBOUND_ERRORS as exc:
                self._logger.error(
                    f"Failed to send confirm request for target "
                    f"{confirmation.target_id}: {exc}",
                    exc,
                )
                self._state.complete_round(
                    confirmation,
                    self._failure_policy.status(confirmation.target_id),
                )
                return

            wait_time_s = self._wait_time_s()
            if wait_time_s <= 0:
                if self._state.complete_round(
                    confirmation,
                    ConfirmationStatus.CONFIRMED,
                ):
                    self._logger.warning(
                        f"T{confirmation.target_id} Confirmed. No wait time.",
                        key="nav_state",
                        dest=LogStatusDest.DRONE,
                    )
                return

            deadline = first_send + wait_time_s
            self._wait_for_response(confirmation, request, deadline)
            status = self._failure_policy.status(confirmation.target_id)
            if not self._state.complete_round(confirmation, status):
                return
            label = (
                "auto-confirmed"
                if status is ConfirmationStatus.CONFIRMED
                else "auto-rejected (timeout)"
            )
            self._logger.info(
                f"Target {confirmation.target_id} {label} after waiting "
                f"{wait_time_s}s.",
                key="nav_state",
                status=(
                    f"T{confirmation.target_id} {label} after "
                    f"{wait_time_s}s."
                ),
            )
        finally:
            self._state.retire_round(confirmation)

    def _wait_for_response(
        self,
        confirmation: ConfirmationRound,
        request: TaskConfirmRequestMsg,
        deadline: float,
    ) -> None:
        while True:
            remaining = deadline - self._monotonic()
            if remaining <= 0:
                return
            resend_interval = self._resend_interval_s()
            wait_slice = min(resend_interval, remaining)
            if confirmation.response_event.wait(timeout=wait_slice):
                return
            if wait_slice < resend_interval:
                return

            network = self._network()
            if network is None:
                return
            try:
                sent = self._state.send_if_current(
                    confirmation,
                    lambda: network.broadcast(request),
                )
            except _OUTBOUND_ERRORS as exc:
                self._logger.error(
                    f"Failed to resend confirm request for target "
                    f"{confirmation.target_id}: {exc}",
                    exc,
                )
                continue
            if not sent:
                return

    def _request_message(
        self,
        target: DetectedObject,
        target_id: Optional[int],
    ) -> TaskConfirmRequestMsg:
        if target_id is None:
            raise ValueError("confirmation request requires a target id")
        source = target.geo.projected_target_location
        location = (
            LocationMsgData(source.lat, source.lng, source.alt)
            if source is not None
            else LocationMsgData(0.0, 0.0, 0.0)
        )
        class_id = target.classification.class_id
        task = TaskMsgData(
            task_id=target_id,
            task_type=class_to_task_type(class_id),
            location=location,
            class_id=class_id,
        )
        return TaskConfirmRequestMsg(sender_id=self._sys_id, task=task)


__all__ = ["ConfirmationRoundRunner"]
