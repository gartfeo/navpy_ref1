import base64
import math
import time
from abc import ABC, abstractmethod
from typing import Optional

from navpy.modules.comm.listener_abc import ListenerAbc
from navpy.modules.comm.messages.msg_abc import MsgABC
from navpy.modules.comm.message_filter import MessageFilter

# Default chunk size (can be overridden by subclasses)
DEFAULT_CHUNK_SIZE = 4096  # 4KB for WiFi/Serial


class NetworkAbc(ABC):
    """
    Abstract base class for network transports.

    Provides message filtering (dedup + TTL) via MessageFilter.
    Subclasses should call receive_packet() or _dispatch_to_listeners()
    to process incoming messages through the filter pipeline.
    """

    def __init__(self, node_id, serializer, logger,
                 message_filter: Optional[MessageFilter] = None,
                 image_chunk_size: int = DEFAULT_CHUNK_SIZE,
                 inter_chunk_delay_ms: float = 0.0):
        self.node_id = node_id
        self.serializer = serializer
        self.logger = logger
        self.is_closed = False
        self.listeners: list[ListenerAbc] = []
        # Create message filter if not provided
        self._message_filter = message_filter or MessageFilter(logger=logger)
        # Image transfer settings
        self._image_chunk_size = image_chunk_size
        self._inter_chunk_delay_sec = inter_chunk_delay_ms / 1000.0

    @abstractmethod
    def broadcast_data(self, data):
        raise NotImplementedError("This is an interface")

    @abstractmethod
    def close(self):
        raise NotImplementedError("This is an interface")

    def broadcast(self, message: MsgABC):
        # Auto-fill meta if not present (for dedup/TTL)
        if not message.has_meta():
            message.set_meta_from_provider()
        serialized_message = self.serializer.serialize_message(message)
        self.broadcast_data(serialized_message.encode('utf-8'))

    def send_image(self, target_id: int, image_b64: str) -> int:
        """
        Send confirmation image for a target using chunked transfer.

        Splits image into chunks and sends via _send_image_chunk().
        Chunk size is configurable per network type.

        Args:
            target_id: Target ID for this confirmation
            image_b64: Base64 encoded JPEG image

        Returns:
            Number of chunks sent
        """
        if not image_b64:
            return 0

        # Decode base64 to raw bytes
        image_bytes = base64.b64decode(image_b64)
        total_size = len(image_bytes)
        num_chunks = math.ceil(total_size / self._image_chunk_size)

        # Send header/handshake
        self._send_image_header(target_id, total_size, num_chunks)

        if self._inter_chunk_delay_sec > 0:
            time.sleep(self._inter_chunk_delay_sec * 2)  # Extra delay after header

        # Send chunks
        for seq in range(num_chunks):
            start = seq * self._image_chunk_size
            end = min(start + self._image_chunk_size, total_size)
            chunk_data = image_bytes[start:end]

            self._send_image_chunk(target_id, seq, chunk_data)

            if self._inter_chunk_delay_sec > 0 and seq < num_chunks - 1:
                time.sleep(self._inter_chunk_delay_sec)

        return num_chunks

    @abstractmethod
    def _send_image_header(self, target_id: int, total_size: int, num_chunks: int) -> None:
        """
        Send image transfer header/handshake.

        Args:
            target_id: Target ID for this image
            total_size: Total image size in bytes
            num_chunks: Number of chunks that will follow
        """
        raise NotImplementedError

    @abstractmethod
    def _send_image_chunk(self, target_id: int, sequence: int, data: bytes) -> None:
        """
        Send a single image chunk.

        Args:
            target_id: Target ID for this image
            sequence: Chunk sequence number (0-indexed)
            data: Chunk data bytes
        """
        raise NotImplementedError

    def receive_packet(self, received_data):
        """
        Callback function called when a message is received.

        Pipeline order:
        1. Deserialize message
        2. Receiver filtering (self-messages, wrong receiver)
        3. Dedup check
        4. TTL check
        5. Dispatch to listeners
        """
        if not self.listeners:
            return
        try:
            message = self.serializer.deserialize_message(received_data.decode('utf-8'))
            self._dispatch_to_listeners(message)
        except UnicodeDecodeError:
            self._message_filter.metrics.increment_decode_error()
            self.logger.error("Failed to decode message as JSON")
        except Exception as e:
            self._message_filter.metrics.increment_decode_error()
            self.logger.error(f"Failed to process message: {e}")

    def _dispatch_to_listeners(self, message: MsgABC):
        """
        Filter and dispatch a message to all registered listeners.

        Applies:
        1. Receiver filtering
        2. Dedup check
        3. TTL check

        Args:
            message: The deserialized message to dispatch
        """
        # Receiver filtering (GCS node_id=255 sees all messages for monitoring)
        if message.sender_id == message.receiver_id:
            return
        # GCS (255) receives all messages; others filter by receiver
        if self.node_id != 255 and message.receiver_id is not None and message.receiver_id != 0 and message.receiver_id != self.node_id:
            return

        # Dedup and TTL filtering
        if not self._message_filter.should_process(message):
            return

        # Dispatch to all listeners
        for listener in self.listeners:
            listener.on_message(message)

    def set_listener(self, listener: ListenerAbc):
        self.listeners.append(listener)

    def remove_listener(self, listener: ListenerAbc) -> None:
        """Detach one exact listener instance if it is currently registered."""
        self.listeners = [item for item in self.listeners if item is not listener]

    @property
    def message_filter(self) -> MessageFilter:
        """Access the message filter for stats or configuration."""
        return self._message_filter

    def log_filter_stats(self) -> None:
        """Log current message filter statistics."""
        self._message_filter.log_stats()
