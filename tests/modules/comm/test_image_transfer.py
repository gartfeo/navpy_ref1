"""Tests for image transfer module."""
import unittest
import base64

from navpy.modules.comm.image_transfer import (
    ImageChunker,
    ImageReassembler,
    ImageTransferType,
    CHUNK_SIZE,
)


class TestImageChunker(unittest.TestCase):
    """Tests for ImageChunker class."""

    def _create_test_image_b64(self, size_bytes: int) -> str:
        """Create a test base64 string of approximately given size."""
        # Base64 has ~33% overhead, so we need fewer raw bytes
        raw_size = int(size_bytes * 0.75)
        raw_bytes = bytes(range(256)) * (raw_size // 256 + 1)
        raw_bytes = raw_bytes[:raw_size]
        return base64.b64encode(raw_bytes).decode('utf-8')

    def test_chunk_small_image(self):
        """Test chunking a small image (single chunk)."""
        # Create small image (100 bytes raw)
        raw_bytes = bytes(range(100))
        image_b64 = base64.b64encode(raw_bytes).decode('utf-8')

        header, chunks = ImageChunker.chunk_image(
            image_b64=image_b64,
            poi_id=1,
        )

        self.assertEqual(header.image_size, 100)
        self.assertEqual(header.poi_id, 1)
        self.assertEqual(header.num_packets, 1)
        self.assertEqual(len(chunks), 1)
        self.assertEqual(len(chunks[0]), CHUNK_SIZE)  # Padded to chunk size

    def test_chunk_large_image(self):
        """Test chunking a large image (multiple chunks)."""
        # Create ~1KB image
        raw_bytes = bytes(range(256)) * 4  # 1024 bytes
        image_b64 = base64.b64encode(raw_bytes).decode('utf-8')

        header, chunks = ImageChunker.chunk_image(
            image_b64=image_b64,
            poi_id=5,
        )

        expected_packets = (1024 + CHUNK_SIZE - 1) // CHUNK_SIZE  # ceil division
        self.assertEqual(header.image_size, 1024)
        self.assertEqual(header.poi_id, 5)
        self.assertEqual(header.num_packets, expected_packets)
        self.assertEqual(len(chunks), expected_packets)

    def test_create_handshake_message(self):
        """Test creating MAVLink handshake message."""
        raw_bytes = bytes(100)
        image_b64 = base64.b64encode(raw_bytes).decode('utf-8')

        header, _ = ImageChunker.chunk_image(
            image_b64=image_b64,
            poi_id=42,
            width=640,
            height=480,
            quality=85,
        )

        msg = ImageChunker.create_handshake_message(header)

        self.assertEqual(msg.type, ImageTransferType.POI_CONFIRMATION)
        self.assertEqual(msg.size, 100)
        self.assertEqual(msg.width, 42)  # poi_id stored in width
        self.assertEqual(msg.jpg_quality, 85)

    def test_create_chunk_message(self):
        """Test creating MAVLink chunk message."""
        data = bytes(range(CHUNK_SIZE))
        msg = ImageChunker.create_chunk_message(sequence=7, data=data)

        self.assertEqual(msg.seqnr, 7)
        self.assertEqual(bytes(msg.data), data)


class TestImageReassembler(unittest.TestCase):
    """Tests for ImageReassembler class."""

    def test_reassemble_single_chunk(self):
        """Test reassembling a single-chunk image."""
        # Create original image
        raw_bytes = bytes(range(100))
        image_b64 = base64.b64encode(raw_bytes).decode('utf-8')

        # Chunk it
        header, chunks = ImageChunker.chunk_image(image_b64, poi_id=1)

        # Create messages
        handshake = ImageChunker.create_handshake_message(header)
        chunk_msg = ImageChunker.create_chunk_message(0, chunks[0])

        # Reassemble
        reassembler = ImageReassembler()
        poi_id = reassembler.on_handshake(handshake)
        self.assertEqual(poi_id, 1)

        result = reassembler.on_chunk(chunk_msg, poi_id)
        self.assertIsNotNone(result)

        # Verify content matches
        result_bytes = base64.b64decode(result)
        self.assertEqual(result_bytes, raw_bytes)

    def test_reassemble_multiple_chunks(self):
        """Test reassembling a multi-chunk image."""
        # Create larger image (1KB)
        raw_bytes = bytes(range(256)) * 4
        image_b64 = base64.b64encode(raw_bytes).decode('utf-8')

        # Chunk it
        header, chunks = ImageChunker.chunk_image(image_b64, poi_id=3)

        # Reassemble
        reassembler = ImageReassembler()
        handshake = ImageChunker.create_handshake_message(header)
        poi_id = reassembler.on_handshake(handshake)

        # Send chunks
        result = None
        for seq, chunk_data in enumerate(chunks):
            chunk_msg = ImageChunker.create_chunk_message(seq, chunk_data)
            result = reassembler.on_chunk(chunk_msg, poi_id)

        # Should have complete image after all chunks
        self.assertIsNotNone(result)
        result_bytes = base64.b64decode(result)
        self.assertEqual(result_bytes, raw_bytes)

    def test_reassemble_out_of_order(self):
        """Test reassembling chunks received out of order."""
        raw_bytes = bytes(range(256)) * 4
        image_b64 = base64.b64encode(raw_bytes).decode('utf-8')

        header, chunks = ImageChunker.chunk_image(image_b64, poi_id=2)

        reassembler = ImageReassembler()
        handshake = ImageChunker.create_handshake_message(header)
        poi_id = reassembler.on_handshake(handshake)

        # Send chunks in reverse order
        result = None
        for seq in reversed(range(len(chunks))):
            chunk_msg = ImageChunker.create_chunk_message(seq, chunks[seq])
            result = reassembler.on_chunk(chunk_msg, poi_id)

        self.assertIsNotNone(result)
        result_bytes = base64.b64decode(result)
        self.assertEqual(result_bytes, raw_bytes)

    def test_ignores_non_confirmation_handshake(self):
        """Test that non-confirmation handshakes are ignored."""
        from pymavlink.dialects.v20.ardupilotmega import (
            MAVLink_data_transmission_handshake_message,
            MAVLINK_DATA_STREAM_IMG_JPEG,
        )

        reassembler = ImageReassembler()
        # Create handshake with different type
        msg = MAVLink_data_transmission_handshake_message(
            type=MAVLINK_DATA_STREAM_IMG_JPEG,  # Not POI_CONFIRMATION
            size=100,
            width=640,
            height=480,
            packets=1,
            payload=253,
            jpg_quality=80,
        )

        result = reassembler.on_handshake(msg)
        self.assertIsNone(result)


class TestRoundTrip(unittest.TestCase):
    """End-to-end round-trip tests."""

    def test_full_round_trip_small(self):
        """Test full round trip with small image."""
        original = bytes(50)
        original_b64 = base64.b64encode(original).decode('utf-8')

        header, chunks = ImageChunker.chunk_image(original_b64, poi_id=10)
        handshake = ImageChunker.create_handshake_message(header)

        reassembler = ImageReassembler()
        poi_id = reassembler.on_handshake(handshake)

        result = None
        for seq, data in enumerate(chunks):
            msg = ImageChunker.create_chunk_message(seq, data)
            result = reassembler.on_chunk(msg, poi_id)

        self.assertEqual(base64.b64decode(result), original)

    def test_full_round_trip_realistic_size(self):
        """Test full round trip with realistic image size (~40KB)."""
        # Simulate a real JPEG-like size
        original = bytes(range(256)) * 160  # ~40KB
        original_b64 = base64.b64encode(original).decode('utf-8')

        header, chunks = ImageChunker.chunk_image(original_b64, poi_id=99)

        self.assertGreater(len(chunks), 100)  # Should have many chunks

        handshake = ImageChunker.create_handshake_message(header)
        reassembler = ImageReassembler()
        poi_id = reassembler.on_handshake(handshake)

        result = None
        for seq, data in enumerate(chunks):
            msg = ImageChunker.create_chunk_message(seq, data)
            result = reassembler.on_chunk(msg, poi_id)

        self.assertIsNotNone(result)
        self.assertEqual(base64.b64decode(result), original)


if __name__ == '__main__':
    unittest.main()
