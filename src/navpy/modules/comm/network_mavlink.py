from __future__ import annotations

import threading
from collections.abc import Callable
from typing import Optional, Protocol

from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink_message,
    MAVLink_data_transmission_handshake_message,
    MAVLink_encapsulated_data_message,
)

from navpy.modules.comm.messages.msg_abc import MsgABC, MsgRegistry
from navpy.modules.comm.message_filter import MessageFilter
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.comm.image_transfer import ImageTransferType

# MAVLink ENCAPSULATED_DATA payload size
MAVLINK_CHUNK_SIZE = 253

from navpy.logger.cache_logger import ILogger
from navpy.utils.serializer_abc import JsonSerializer


class MessageSubscription(Protocol):
    def cancel(self) -> None: ...


class MavlinkMessageTransport(Protocol):
    def on_message(
        self,
        message_name: str,
        callback: Callable[[MAVLink_message], None],
    ) -> MessageSubscription: ...

    def send_mavlink_message(
        self,
        mav_msg: MAVLink_message,
        *,
        source_component: int | None = None,
    ) -> None: ...


class NetworkMavlink(NetworkAbc):
    """
    Network layer using MAVLink connections provided by a vehicle.

    Receives custom MAVLink messages and dispatches to listeners after
    filtering (dedup + TTL).
    """

    def __init__(self, node_id: int, vehicle: MavlinkMessageTransport, logger: ILogger,
                 message_filter: Optional[MessageFilter] = None):
        super().__init__(
            node_id, JsonSerializer(), logger, message_filter,
            image_chunk_size=MAVLINK_CHUNK_SIZE,
            inter_chunk_delay_ms=5.0
        )
        self.logger = logger
        self._vehicle = vehicle
        self._callback_lock = threading.RLock()
        self._vehicle_subscription = self._vehicle.on_message(
            "NAVLINK", self._on_mavlink,
        )

    def broadcast(self, message: MsgABC):
        # Auto-fill meta if not present (for dedup/TTL)
        if not message.has_meta():
            message.set_meta_from_provider()
        mav_msg = message.to_mavlink()
        self._vehicle.send_mavlink_message(mav_msg)

    def broadcast_data(self, data):
        self.logger.error("Raw MAVLink data broadcast not supported")
        raise

    def _on_mavlink(self, mav_msg: MAVLink_message):
        """
        Filter to NavPy messages and dispatch through filter pipeline.

        Pipeline:
        1. Convert MAVLink to NavPy message
        2. Apply receiver filtering
        3. Apply dedup + TTL checks
        4. Dispatch to listeners
        """
        with self._callback_lock:
            if self.is_closed:
                return
            msg_cls = MsgRegistry.get_class_by_mav_id(mav_msg.get_msgId())
            if not msg_cls:
                return  # not registered yet

            try:
                msg = msg_cls.from_mavlink(mav_msg)  # build NavPy object
                self.logger.debug(
                    f"NetworkMavlink[{self.node_id}] received: "
                    f"{msg.msg_type().name} from={msg.sender_id} "
                    f"to={msg.receiver_id}"
                )
                self._dispatch_to_listeners(msg)
            except Exception as e:
                self._message_filter.metrics.increment_decode_error()
                self.logger.error(f"Failed to process MAVLink message: {e}")

    def _send_image_header(self, poi_id: int, total_size: int, num_chunks: int) -> None:
        """Send MAVLink DATA_TRANSMISSION_HANDSHAKE for image transfer."""
        msg = MAVLink_data_transmission_handshake_message(
            type=ImageTransferType.POI_CONFIRMATION,
            size=total_size,
            width=poi_id,  # Store poi_id in width field
            height=480,
            packets=num_chunks,
            payload=MAVLINK_CHUNK_SIZE,
            jpg_quality=80,
        )
        self._vehicle.send_mavlink_message(msg)

    def _send_image_chunk(self, poi_id: int, sequence: int, data: bytes) -> None:
        """Send MAVLink ENCAPSULATED_DATA chunk."""
        # Pad to chunk size if needed
        if len(data) < MAVLINK_CHUNK_SIZE:
            data = data + bytes(MAVLINK_CHUNK_SIZE - len(data))

        msg = MAVLink_encapsulated_data_message(
            seqnr=sequence,
            data=list(data),
        )
        self._vehicle.send_mavlink_message(msg)

    def close(self):
        with self._callback_lock:
            if self.is_closed:
                return
            self.is_closed = True
        self._vehicle_subscription.cancel()
