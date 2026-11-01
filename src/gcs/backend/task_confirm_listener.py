"""Thin listener facade for task confirmation requests and image transfers."""

from __future__ import annotations

import asyncio
import logging
import math
from typing import Optional

from gcs.backend.broadcast import manager as ws_manager
from gcs.backend.companion_identity import is_from_companion
from gcs.backend.producer_inventory import producers
from gcs.backend.task_confirm_images import ConfirmationImageSession
from gcs.backend.task_confirm_ports import TaskConfirmVehiclePort
from gcs.backend.task_confirm_rounds import (
    LEGACY_ROUND_UID,
    ConfirmationRoundRegistry,
    RequestAction,
    round_uid,
)
from gcs.backend.task_confirm_transport import ConfirmationTransport
from navpy.core.kernel import runtime_time as rt
from navpy.core.link.task_confirmation_msg import TaskConfirmRequestMsg
from navpy.core.link.msg_abc import MsgRegistry


log = logging.getLogger(__name__)


def _known(value: float) -> Optional[float]:
    """A request coordinate, or None when the companion sent NaN (unknown)."""
    return value if math.isfinite(value) else None

# Historical private names remain importable for compatibility tests and old
# backend extensions. Concrete ownership lives in the focused modules above.
_LEGACY_ROUND_UID = LEGACY_ROUND_UID


_PRODUCER = "task_confirm_listener"


class TaskConfirmListener:
    """Route confirmation protocol events to focused state/transport owners."""

    _HANDLED_TTL_S = 120.0
    _THUMBNAIL_GRACE_S = 3.0

    def __init__(self, loop: asyncio.AbstractEventLoop):
        self._loop = loop
        self._vehicles: dict[int, TaskConfirmVehiclePort] = {}
        self._rounds = ConfirmationRoundRegistry()
        self._transport = ConfirmationTransport(self._vehicles.get)
        self._images = ConfirmationImageSession(
            self._rounds.open_uid,
            self._rounds.accept_image,
            self._broadcast,
            self._is_from_registered_companion,
            lambda: rt.monotonic(),
        )

    def register_vehicle(self, sys_id: int, vehicle: TaskConfirmVehiclePort) -> None:
        self._vehicles[sys_id] = vehicle
        self._images.register_vehicle(sys_id)
        for name, handler in (
            ("NAVLINK", self._on_navlink),
            ("DATA_TRANSMISSION_HANDSHAKE", self._on_handshake),
            ("ENCAPSULATED_DATA", self._on_chunk),
        ):
            producers.keep_subscription(_PRODUCER, vehicle.on_message(
                name, lambda message, sid=sys_id, handle=handler: handle(sid, message),
            ))

    def _on_navlink(self, sys_id: int, message) -> None:
        message_class = MsgRegistry.get_class_by_mav_id(message.get_msgId())
        if (
            message_class is None
            or not issubclass(message_class, TaskConfirmRequestMsg)
        ):
            return
        request = message_class.from_mavlink(message)
        if not self._is_from_registered_companion(sys_id, message):
            return

        task = request.task
        uid = round_uid(request)
        action, is_confirmed = self._rounds.classify(
            sys_id,
            task.task_id,
            uid,
            rt.monotonic(),
            self._HANDLED_TTL_S,
        )
        if action is RequestAction.ANSWER:
            self._transport.send_response(
                sys_id,
                task.task_id,
                is_confirmed,
                uid,
            )
            return
        if action is RequestAction.REPEAT:
            self._maybe_request_thumbnail(sys_id, task.task_id)
            return

        self._images.abandon(sys_id, task.task_id)
        self._broadcast({
            "type": "task_confirm_request",
            "sys_id": sys_id,
            "task_id": task.task_id,
            "task_type": task.task_type.name,
            "lat": _known(task.location.lat),
            "lon": _known(task.location.lng),
            "alt": _known(task.location.alt),
            "round_uid": uid,
        })

    def clear_handled(self, sys_id: int, task_id: int) -> None:
        self._rounds.clear(sys_id, task_id)

    def mark_decided(
        self,
        sys_id: int,
        task_id: int,
        is_confirmed: bool,
        round_uid: Optional[str] = None,
    ) -> None:
        self._rounds.mark_decided(
            sys_id,
            task_id,
            is_confirmed,
            round_uid,
            rt.monotonic(),
        )

    def _maybe_request_thumbnail(self, sys_id: int, task_id: int) -> None:
        uid = self._rounds.thumbnail_request_due(
            sys_id,
            task_id,
            rt.monotonic(),
            self._THUMBNAIL_GRACE_S,
            self._images.last_frame_at(sys_id, task_id),
        )
        if uid is not None:
            self._transport.request_thumbnail(sys_id, task_id, uid)

    def _on_handshake(self, sys_id: int, message) -> None:
        self._images.on_handshake(sys_id, message)

    def _on_chunk(self, sys_id: int, message) -> None:
        self._images.on_chunk(sys_id, message)

    @staticmethod
    def _is_from_registered_companion(sys_id: int, message) -> bool:
        return is_from_companion(message, sys_id)

    def _broadcast(self, payload: dict) -> None:
        producers.broadcast(_PRODUCER, ws_manager.broadcast(payload), self._loop)


__all__ = ["TaskConfirmListener", "_LEGACY_ROUND_UID"]
