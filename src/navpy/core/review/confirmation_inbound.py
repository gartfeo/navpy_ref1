from __future__ import annotations

from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.link.task_confirmation_msg import (
    TaskConfirmResponseMsg,
)
from navpy.core.link.msg_abc import MsgABC
from navpy.core.link.swarm_request_msg import (
    REQUEST_TYPE_RESOURCE,
    SUBJECT_TYPE_THUMBNAIL,
    SwarmRequestMsg,
)
from navpy.core.link.types import MsgType
from navpy.core.review.confirmation_ports import ConfirmationInboundStatePort
from navpy.core.review.confirmation_round_transaction import (
    ConfirmationRequestRef,
    ConfirmationResponseKind,
)
from navpy.core.review.confirmation_media import ConfirmationMedia


class ConfirmationInboundHandler:
    """Route responses and resource requests into focused state/media ports."""

    def __init__(
        self,
        *,
        sys_id: int,
        state: ConfirmationInboundStatePort,
        media: ConfirmationMedia,
        logger: ILogger,
    ) -> None:
        self._sys_id = sys_id
        self._state = state
        self._media = media
        self._logger = logger

    def on_message(self, message: MsgABC) -> None:
        if message.receiver_id is not None and message.receiver_id != self._sys_id:
            return
        message_type = message.msg_type()
        if message_type == MsgType.TASK_CONFIRM_RESPONSE:
            self.handle_response(message)
        elif message_type == MsgType.SWARM_REQUEST:
            self.handle_resource_request(message)

    def handle_resource_request(self, message: SwarmRequestMsg) -> None:
        if (
            message.request_type != REQUEST_TYPE_RESOURCE
            or message.subject_type != SUBJECT_TYPE_THUMBNAIL
        ):
            return
        poi = self._state.pending_poi(message.subject_id)
        if poi is None:
            return
        try:
            request_ref = ConfirmationRequestRef.from_meta(message.meta)
            self._media.send(
                poi,
                message.subject_id,
                lambda unit: self._state.send_pending_if_current(
                    message.subject_id, poi, request_ref, unit,
                ),
                resend=True,
            )
        except OSError as exc:
            self._logger.error(
                f"Failed to resend confirmation thumbnail for "
                f"P{message.subject_id}: {exc}",
                exc,
            )

    def handle_response(self, message: TaskConfirmResponseMsg) -> None:
        poi_id = message.task_id
        result = self._state.resolve_response(
            poi_id,
            message.is_confirmed,
            ConfirmationRequestRef.from_meta(message.meta),
        )
        if result is ConfirmationResponseKind.CONFIRMED:
            self._logger.info(
                f"POI {poi_id} confirmed by ground station.",
                key="nav_state",
                status=f"P{poi_id} confirmed by GCS",
                dest=LogStatusDest.DRONE,
            )
        elif result is ConfirmationResponseKind.REJECTED:
            self._logger.info(
                f"POI {poi_id} rejected by ground station.",
                key="nav_state",
                status=f"P{poi_id} rejected by GCS",
                dest=LogStatusDest.DRONE,
            )
        elif result is ConfirmationResponseKind.CANCELLATION_REQUESTED:
            self._logger.info(
                f"Cancellation requested for task {poi_id} by ground station.",
                key="nav_state",
                status=f"P{poi_id} rejected by GCS",
                dest=LogStatusDest.DRONE,
            )
        elif result is ConfirmationResponseKind.LATE_OR_DUPLICATE:
            self._logger.warning(
                f"Ignoring late or duplicate confirmation for P{poi_id}."
            )
        elif result is ConfirmationResponseKind.ALREADY_REJECTED:
            self._logger.warning(
                f"Ignoring confirm for already-rejected P{poi_id}.",
                key="nav_state",
                dest=LogStatusDest.DRONE,
            )
