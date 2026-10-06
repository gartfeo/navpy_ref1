"""Narrow structural ports for confirmation workflow leaves."""

from __future__ import annotations

import threading
from collections.abc import Callable
from typing import Optional, Protocol

from navpy.modules.comm.messages.msg_abc import MsgABC
from navpy.modules.nav.confirmation_round_transaction import (
    ConfirmationRequestRef,
    ConfirmationResponseKind,
    ConfirmationRound,
    ConfirmationWorkerLease,
)
from navpy.modules.nav.confirmation_registry_state import ConfirmationStatus
from navpy.modules.vision.models.detect_data import DetectedObject


class ConfirmationMessageBroadcaster(Protocol):
    def broadcast(self, message: MsgABC) -> None: ...


class ConfirmationImageSender(Protocol):
    def send_image(self, poi_id: int, image_b64: str) -> int: ...


class ConfirmationNetworkPort(
    ConfirmationMessageBroadcaster,
    ConfirmationImageSender,
    Protocol,
):
    pass


class ConfirmationWorkerStatePort(Protocol):
    def start_review(
        self,
        poi: DetectedObject,
        event_factory: Callable[[], threading.Event],
    ) -> ConfirmationWorkerLease: ...

    def worker_is_current(self, lease: ConfirmationWorkerLease) -> bool: ...

    def set_worker_status(
        self,
        lease: ConfirmationWorkerLease,
        status: ConfirmationStatus,
    ) -> bool: ...

    def finish_worker(self, lease: ConfirmationWorkerLease) -> None: ...


class ConfirmationRoundStatePort(Protocol):
    def set_worker_status(
        self,
        lease: ConfirmationWorkerLease,
        status: ConfirmationStatus,
    ) -> bool: ...

    def begin_round(
        self,
        poi: DetectedObject,
        lease: ConfirmationWorkerLease,
        event_factory: Callable[[], threading.Event],
        request_ref: Optional[ConfirmationRequestRef] = None,
    ) -> Optional[ConfirmationRound]: ...

    def complete_round(
        self,
        confirmation: ConfirmationRound,
        status: ConfirmationStatus,
    ) -> bool: ...

    def retire_round(self, confirmation: ConfirmationRound) -> bool: ...

    def send_if_current(
        self,
        confirmation: ConfirmationRound,
        send: Callable[[], None],
    ) -> bool: ...


class ConfirmationInboundStatePort(Protocol):
    def pending_poi(self, poi_id: int) -> Optional[DetectedObject]: ...

    def send_pending_if_current(
        self,
        poi_id: int,
        poi: DetectedObject,
        request_ref: Optional[ConfirmationRequestRef],
        send: Callable[[], None],
    ) -> bool: ...

    def resolve_response(
        self,
        poi_id: int,
        is_confirmed: bool,
        response_ref: Optional[ConfirmationRequestRef] = None,
    ) -> ConfirmationResponseKind: ...


__all__ = [
    "ConfirmationImageSender",
    "ConfirmationInboundStatePort",
    "ConfirmationMessageBroadcaster",
    "ConfirmationNetworkPort",
    "ConfirmationRoundStatePort",
    "ConfirmationWorkerStatePort",
]
