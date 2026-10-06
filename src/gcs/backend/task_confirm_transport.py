"""Outbound GCS transport for confirmation decisions and thumbnail pulls."""

from __future__ import annotations

import logging
from collections.abc import Callable
from typing import Optional

from navpy.modules.comm.messages.available_task_msg import TaskConfirmResponseMsg
from navpy.modules.comm.messages.msg_meta import MsgMeta, MsgMetaProvider
from navpy.modules.comm.messages.swarm_request_msg import (
    REQUEST_TYPE_RESOURCE,
    SUBJECT_TYPE_THUMBNAIL,
    SwarmRequestMsg,
)
from navpy.modules.comm.messages.types import MsgType

from gcs.backend.task_confirm_ports import MavlinkMessageSender
from gcs.backend.task_confirm_uid import parse_round_uid


log = logging.getLogger(__name__)
GCS_SENDER_ID = 0


def correlated_meta(uid: str, msg_type: MsgType) -> MsgMeta:
    boot_id, msg_seq = parse_round_uid(uid)
    fresh = MsgMetaProvider.get_instance().create_meta_for_type(msg_type)
    return MsgMeta(boot_id, msg_seq, fresh.time_ms, fresh.ttl_ms)


class ConfirmationTransport:
    def __init__(
        self,
        vehicle_for: Callable[[int], Optional[MavlinkMessageSender]],
    ) -> None:
        self._vehicle_for = vehicle_for

    def send_response(
        self,
        sys_id: int,
        task_id: int,
        is_confirmed: bool,
        uid: str,
    ) -> None:
        vehicle = self._vehicle_for(sys_id)
        if vehicle is None:
            return
        response = TaskConfirmResponseMsg(
            receiver_id=sys_id,
            task_id=task_id,
            is_confirmed=is_confirmed,
            meta=correlated_meta(uid, MsgType.TASK_CONFIRM_RESPONSE),
        )
        try:
            vehicle.send_mavlink_message(response.to_mavlink())
            log.info(
                "Re-sent %s decision for vehicle %d task %d",
                "approve" if is_confirmed else "deny",
                sys_id,
                task_id,
            )
        except OSError:
            log.exception(
                "Failed to re-send confirm response for vehicle %d task %d",
                sys_id,
                task_id,
            )

    def request_thumbnail(self, sys_id: int, task_id: int, uid: str) -> None:
        vehicle = self._vehicle_for(sys_id)
        if vehicle is None:
            return
        request = SwarmRequestMsg(
            sender_id=GCS_SENDER_ID,
            receiver_id=sys_id,
            request_type=REQUEST_TYPE_RESOURCE,
            subject_type=SUBJECT_TYPE_THUMBNAIL,
            subject_id=task_id,
            meta=correlated_meta(uid, MsgType.SWARM_REQUEST),
        )
        try:
            vehicle.send_mavlink_message(request.to_mavlink())
            log.info(
                "Requested on-demand thumbnail for vehicle %d task %d",
                sys_id,
                task_id,
            )
        except OSError:
            log.exception(
                "Failed to send thumbnail request for vehicle %d task %d",
                sys_id,
                task_id,
            )


__all__ = ["ConfirmationTransport", "correlated_meta", "parse_round_uid"]
