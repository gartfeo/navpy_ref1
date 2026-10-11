"""
MAVLink image transfer for POI confirmation images.

Uses standard MAVLink protocol:
1. DATA_TRANSMISSION_HANDSHAKE - announces image transfer
2. ENCAPSULATED_DATA - sends image chunks (253 bytes each)

GCS must reassemble chunks using sequence numbers.
"""
import base64
import math
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Callable, Dict, List, Optional

from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink_data_transmission_handshake_message,
    MAVLink_encapsulated_data_message,
)

# Chunk size for ENCAPSULATED_DATA payload
CHUNK_SIZE = 253


class ImageTransferType(IntEnum):
    """Image transfer types - extends MAVLink data stream types."""
    POI_CONFIRMATION = 100  # Custom type for POI confirmation images


@dataclass
class ImageTransferHeader:
    """Header info for image transfer (from handshake message)."""
    transfer_type: int  # ImageTransferType or MAVLink data stream type
    image_size: int  # Total bytes
    width: int
    height: int
    num_packets: int
    payload_size: int  # Bytes per packet (typically 253)
    quality: int  # JPEG quality (0-100)
    poi_id: int = 0  # POI ID (stored in width for confirmation images)


@dataclass
class ImageTransferChunk:
    """A single chunk of image data."""
    sequence: int
    data: bytes


class ImageChunker:
    """Splits an image into MAVLink-compatible chunks."""

    @staticmethod
    def chunk_image(
        image_b64: str,
        poi_id: int,
        width: int = 640,
        height: int = 480,
        quality: int = 80,
    ) -> tuple[ImageTransferHeader, List[bytes]]:
        """
        Split base64 image into chunks for MAVLink transfer.

        Args:
            image_b64: Base64 encoded JPEG image
            poi_id: POI ID for confirmation
            width: Image width
            height: Image height
            quality: JPEG quality used

        Returns:
            Tuple of (header, list of chunk data)
        """
        # Decode base64 to raw bytes
        image_bytes = base64.b64decode(image_b64)
        image_size = len(image_bytes)

        num_packets = math.ceil(image_size / CHUNK_SIZE)

        header = ImageTransferHeader(
            transfer_type=ImageTransferType.POI_CONFIRMATION,
            image_size=image_size,
            width=width,
            height=height,
            num_packets=num_packets,
            payload_size=CHUNK_SIZE,
            quality=quality,
            poi_id=poi_id,
        )

        # Split into chunks
        chunks = []
        for i in range(num_packets):
            start = i * CHUNK_SIZE
            end = min(start + CHUNK_SIZE, image_size)
            chunk_data = image_bytes[start:end]

            # Pad last chunk to CHUNK_SIZE if needed
            if len(chunk_data) < CHUNK_SIZE:
                chunk_data = chunk_data + bytes(CHUNK_SIZE - len(chunk_data))

            chunks.append(chunk_data)

        return header, chunks

    @staticmethod
    def create_handshake_message(header: ImageTransferHeader) -> MAVLink_data_transmission_handshake_message:
        """Create MAVLink handshake message for image transfer."""
        return MAVLink_data_transmission_handshake_message(
            type=header.transfer_type,
            size=header.image_size,
            width=header.poi_id,  # Store poi_id in width for confirmation images
            height=header.height,
            packets=header.num_packets,
            payload=header.payload_size,
            jpg_quality=header.quality,
        )

    @staticmethod
    def create_chunk_message(sequence: int, data: bytes) -> MAVLink_encapsulated_data_message:
        """Create MAVLink encapsulated data message for a chunk."""
        return MAVLink_encapsulated_data_message(
            seqnr=sequence,
            data=list(data),
        )


class ImageReassembler:
    """Reassembles image chunks received from MAVLink."""

    def __init__(self, timeout_sec: float = 10.0):
        self._transfers: Dict[int, '_PendingTransfer'] = {}  # poi_id -> transfer
        self._timeout_sec = timeout_sec

    def on_handshake(self, msg: MAVLink_data_transmission_handshake_message) -> Optional[int]:
        """
        Handle incoming handshake message.

        Returns:
            poi_id if this is a confirmation image, None otherwise
        """
        if msg.type != ImageTransferType.POI_CONFIRMATION:
            return None

        poi_id = msg.width  # poi_id stored in width field
        self._transfers[poi_id] = _PendingTransfer(
            poi_id=poi_id,
            total_size=msg.size,
            num_packets=msg.packets,
            payload_size=msg.payload,
            quality=msg.jpg_quality,
            start_time=time.time(),
        )
        return poi_id

    def on_chunk(self, msg: MAVLink_encapsulated_data_message, poi_id: int) -> Optional[str]:
        """
        Handle incoming data chunk.

        Args:
            msg: The encapsulated data message
            poi_id: Which transfer this chunk belongs to

        Returns:
            Complete base64 image if transfer is complete, None otherwise
        """
        if poi_id not in self._transfers:
            return None

        transfer = self._transfers[poi_id]
        transfer.chunks[msg.seqnr] = bytes(msg.data)

        # Check if complete
        if len(transfer.chunks) >= transfer.num_packets:
            # Reassemble
            image_bytes = b''
            for i in range(transfer.num_packets):
                if i in transfer.chunks:
                    image_bytes += transfer.chunks[i]

            # Trim to actual size
            image_bytes = image_bytes[:transfer.total_size]

            # Clean up
            del self._transfers[poi_id]

            # Return as base64
            return base64.b64encode(image_bytes).decode('utf-8')

        return None

    def cleanup_stale(self):
        """Remove stale transfers that have timed out."""
        now = time.time()
        stale = [
            tid for tid, t in self._transfers.items()
            if now - t.start_time > self._timeout_sec
        ]
        for tid in stale:
            del self._transfers[tid]


@dataclass
class _PendingTransfer:
    """Internal class tracking a pending image transfer."""
    poi_id: int
    total_size: int
    num_packets: int
    payload_size: int
    quality: int
    start_time: float
    chunks: Dict[int, bytes] = None

    def __post_init__(self):
        if self.chunks is None:
            self.chunks = {}


class ImageTransferSender:
    """
    Sends confirmation images via MAVLink.

    Usage:
        sender = ImageTransferSender(vehicle.send_mavlink_message)
        sender.send_confirmation_image(poi_id=1, image_b64="...")
    """

    def __init__(
        self,
        send_func: Callable,
        inter_packet_delay_ms: float = 5.0,
    ):
        """
        Args:
            send_func: Function to send MAVLink messages (e.g., vehicle.send_mavlink_message)
            inter_packet_delay_ms: Delay between packets to avoid flooding
        """
        self._send = send_func
        self._delay_sec = inter_packet_delay_ms / 1000.0

    def send_confirmation_image(
        self,
        poi_id: int,
        image_b64: str,
        width: int = 640,
        height: int = 480,
    ) -> int:
        """
        Send a confirmation image via MAVLink chunked transfer.

        Args:
            poi_id: POI ID for this confirmation
            image_b64: Base64 encoded JPEG thumbnail
            width: Image width
            height: Image height

        Returns:
            Number of packets sent
        """
        header, chunks = ImageChunker.chunk_image(
            image_b64=image_b64,
            poi_id=poi_id,
            width=width,
            height=height,
        )

        # Send handshake
        handshake = ImageChunker.create_handshake_message(header)
        self._send(handshake)

        # Small delay after handshake
        if self._delay_sec > 0:
            time.sleep(self._delay_sec * 2)

        # Send chunks
        for seq, chunk_data in enumerate(chunks):
            chunk_msg = ImageChunker.create_chunk_message(seq, chunk_data)
            self._send(chunk_msg)

            if self._delay_sec > 0 and seq < len(chunks) - 1:
                time.sleep(self._delay_sec)

        return len(chunks)
