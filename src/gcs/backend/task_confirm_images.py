"""Chunked confirmation-thumbnail session ownership."""

from __future__ import annotations

import logging
from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional

from navpy.modules.comm.image_transfer import ImageReassembler
from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink_data_transmission_handshake_message,
    MAVLink_encapsulated_data_message,
)


log = logging.getLogger(__name__)


@dataclass(frozen=True)
class ActiveImage:
    poi_id: int
    round_uid: Optional[str]


class ConfirmationImageSession:
    def __init__(
        self,
        open_round_uid: Callable[[int, int], Optional[str]],
        accept_image: Callable[[int, int, Optional[str]], bool],
        broadcast: Callable[[dict], None],
        is_companion: Callable[[int, object], bool],
    ) -> None:
        self._open_round_uid = open_round_uid
        self._accept_image = accept_image
        self._broadcast = broadcast
        self._is_companion = is_companion
        self.reassemblers: dict[int, ImageReassembler] = {}
        self.current_pois: dict[int, ActiveImage] = {}

    def register_vehicle(self, sys_id: int) -> None:
        self.reassemblers[sys_id] = ImageReassembler(timeout_sec=15.0)

    def abandon(self, sys_id: int, task_id: int) -> None:
        active = self.current_pois.get(sys_id)
        if active is not None and active.poi_id == task_id:
            self.current_pois.pop(sys_id, None)

    def on_handshake(
        self,
        sys_id: int,
        message: MAVLink_data_transmission_handshake_message,
    ) -> None:
        if not self._is_companion(sys_id, message):
            return
        reassembler = self.reassemblers.get(sys_id)
        if reassembler is None:
            return
        poi_id = reassembler.on_handshake(message)
        if poi_id is None:
            return
        self.current_pois[sys_id] = ActiveImage(
            poi_id,
            self._open_round_uid(sys_id, poi_id),
        )
        log.info("Image handshake for vehicle %d, poi_id=%d", sys_id, poi_id)

    def on_chunk(
        self,
        sys_id: int,
        message: MAVLink_encapsulated_data_message,
    ) -> None:
        if not self._is_companion(sys_id, message):
            return
        reassembler = self.reassemblers.get(sys_id)
        active = self.current_pois.get(sys_id)
        if reassembler is None or active is None:
            return
        image_b64 = reassembler.on_chunk(message, active.poi_id)
        if image_b64 is None:
            return
        self.current_pois.pop(sys_id, None)

        if not self._accept_image(sys_id, active.poi_id, active.round_uid):
            log.info(
                "Dropping thumbnail for vehicle %d POI %d: no matching round",
                sys_id,
                active.poi_id,
            )
            return

        self._broadcast({
            "type": "task_confirm_image",
            "sys_id": sys_id,
            "task_id": active.poi_id,
            "image_b64": image_b64,
            "round_uid": active.round_uid,
        })


__all__ = ["ActiveImage", "ConfirmationImageSession"]
