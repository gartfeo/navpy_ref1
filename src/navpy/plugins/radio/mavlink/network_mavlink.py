from __future__ import annotations

import logging
from collections.abc import Callable
from typing import Optional, Protocol

from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink_message,
    MAVLink_data_transmission_handshake_message,
    MAVLink_encapsulated_data_message,
)

from navpy.core.kernel import runtime_time as rt
from navpy.core.link.msg_abc import MsgABC, MsgRegistry
from navpy.core.link.message_filter import MessageFilter
from navpy.core.ports.network_abc import NetworkAbc
from navpy.plugins.radio.mavlink.image_transfer import ImageTransferType

# MAVLink ENCAPSULATED_DATA payload size
MAVLINK_CHUNK_SIZE = 253
# Ends the inbound dispatch thread (queued after the last message on close).
_CLOSED = object()

log = logging.getLogger(__name__)

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.link.serializer_abc import JsonSerializer


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

    Listeners run in arrival order on this network's own thread, never on
    the vehicle's MAVLink reader: a listener may wait (a confirmation
    response waits for the round's paced image send), and waiting on the
    reader stalls every telemetry stream of the vehicle for that long.
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
        self._callback_lock = rt.RLock()
        self._inbound = rt.Queue()
        self._dispatcher = rt.Thread(target=self._dispatch_inbound, daemon=True)
        # Subscribe first: a failed subscription leaves no thread behind.
        self._vehicle_subscription = self._vehicle.on_message(
            "NAVLINK", self._inbound.put,
        )
        with rt.detached():  # owned by this network (closed by it)
            self._dispatcher.start()

    def broadcast(self, message: MsgABC):
        # Auto-fill meta if not present (for dedup/TTL)
        if not message.has_meta():
            message.set_meta_from_provider()
        mav_msg = message.to_mavlink()
        self._vehicle.send_mavlink_message(mav_msg)

    def broadcast_data(self, data):
        self.logger.error("Raw MAVLink data broadcast not supported")
        raise

    def _dispatch_inbound(self) -> None:
        while True:
            mav_msg = self._inbound.get()
            if mav_msg is _CLOSED:
                return
            try:
                self._on_mavlink(mav_msg)
            except Exception:  # noqa: BLE001 - one message never stops dispatch
                log.exception("NetworkMavlink[%s] dispatch error", self.node_id)

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
        self._inbound.put(_CLOSED)
        if rt.current_thread() is not self._dispatcher:
            self._dispatcher.join()
