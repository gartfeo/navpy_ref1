"""Tests for image_utils module."""
import unittest
import numpy as np

from navpy.utils.image_utils import (
    create_confirmation_thumbnail,
    decode_confirmation_thumbnail,
    THUMBNAIL_SIZE,
)


class TestCreateConfirmationThumbnail(unittest.TestCase):
    """Tests for create_confirmation_thumbnail function."""

    def test_creates_valid_base64_string(self):
        """Test that a valid base64 string is returned."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame[:] = (100, 150, 200)  # Fill with a color

        bbox = (320.0, 240.0, 100.0, 80.0)  # cx, cy, w, h
        result = create_confirmation_thumbnail(frame, target_id=1, bbox=bbox)

        self.assertIsNotNone(result)
        self.assertIsInstance(result, str)
        self.assertGreater(len(result), 100)

    def test_draws_bounding_box(self):
        """Test that bounding box is drawn on the image."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        bbox = (320.0, 240.0, 100.0, 80.0)

        result = create_confirmation_thumbnail(frame, target_id=5, bbox=bbox)
        decoded = decode_confirmation_thumbnail(result)

        self.assertIsNotNone(decoded)
        # Check that the image is not all black (bounding box was drawn)
        self.assertGreater(np.sum(decoded), 0)

    def test_resizes_to_thumbnail_size(self):
        """Test that output is resized to thumbnail dimensions."""
        # Create a larger frame
        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        bbox = (960.0, 540.0, 200.0, 150.0)

        result = create_confirmation_thumbnail(frame, target_id=1, bbox=bbox)
        decoded = decode_confirmation_thumbnail(result)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.shape[1], THUMBNAIL_SIZE[0])  # width
        self.assertEqual(decoded.shape[0], THUMBNAIL_SIZE[1])  # height

    def test_handles_none_frame(self):
        """Test that None frame returns None."""
        result = create_confirmation_thumbnail(None, target_id=1, bbox=(0, 0, 10, 10))
        self.assertIsNone(result)

    def test_handles_empty_frame(self):
        """Test that empty frame returns None."""
        frame = np.array([], dtype=np.uint8)
        result = create_confirmation_thumbnail(frame, target_id=1, bbox=(0, 0, 10, 10))
        self.assertIsNone(result)

    def test_handles_bbox_at_edge(self):
        """Test bounding box at frame edge."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        bbox = (50.0, 50.0, 120.0, 100.0)  # Partially outside frame

        result = create_confirmation_thumbnail(frame, target_id=1, bbox=bbox)
        self.assertIsNotNone(result)

    def test_crops_around_bbox_for_small_target(self):
        """Test that a tiny target in a large frame becomes visible after cropping."""
        # Simulate a 32x22 rectangular sprite in a 1920x1080 frame (detection at ~700m)
        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        frame[:] = (60, 80, 50)  # greenish background
        # Draw a small white rectangle to simulate the rectangular sprite
        cx, cy, bw, bh = 960, 540, 32, 22
        frame[cy - bh // 2:cy + bh // 2, cx - bw // 2:cx + bw // 2] = (255, 255, 255)

        result = create_confirmation_thumbnail(frame, target_id=0, bbox=(cx, cy, bw, bh))
        decoded = decode_confirmation_thumbnail(result)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.shape[:2], (THUMBNAIL_SIZE[1], THUMBNAIL_SIZE[0]))

        # The crop should zoom in, making the white target pixels take up
        # a significant portion of the thumbnail (not just 10x7 pixels).
        # Count non-background pixels in the decoded image.
        # With cropping, the white rectangle should be much larger than
        # without cropping (where it would be ~10x7 = 70 pixels).
        white_mask = np.all(decoded > 200, axis=2)
        white_pixel_count = np.count_nonzero(white_mask)
        # Without crop: ~70 white pixels. With crop: should be >>200.
        self.assertGreater(white_pixel_count, 200,
                           f"Target too small in thumbnail ({white_pixel_count} white pixels)")

    def test_crop_keeps_bbox_centered(self):
        """Test that the target bbox stays roughly centered after cropping."""
        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        cx, cy, bw, bh = 960, 540, 50, 40

        result = create_confirmation_thumbnail(frame, target_id=1, bbox=(cx, cy, bw, bh))
        decoded = decode_confirmation_thumbnail(result)
        self.assertIsNotNone(decoded)

        # The bounding box color is orange (0, 69, 255) in BGR.
        # Find orange-ish pixels (high red channel in decoded BGR).
        orange_mask = (decoded[:, :, 2] > 200) & (decoded[:, :, 1] < 120)
        orange_coords = np.argwhere(orange_mask)
        self.assertGreater(len(orange_coords), 0, "No bounding box drawn")

        # Check that the bbox center is roughly in the middle of the thumbnail.
        # Vertical tolerance is wider because the label sits above the bbox.
        center_y = orange_coords[:, 0].mean()
        center_x = orange_coords[:, 1].mean()
        self.assertAlmostEqual(center_x, THUMBNAIL_SIZE[0] / 2, delta=THUMBNAIL_SIZE[0] * 0.25)
        self.assertAlmostEqual(center_y, THUMBNAIL_SIZE[1] / 2, delta=THUMBNAIL_SIZE[1] * 0.45)

    def test_crop_clamps_to_frame_corner(self):
        """Test bbox near a corner doesn't crash and produces valid output."""
        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        # Target near top-left corner
        bbox = (30.0, 20.0, 32.0, 22.0)

        result = create_confirmation_thumbnail(frame, target_id=0, bbox=bbox)
        decoded = decode_confirmation_thumbnail(result)
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.shape[:2], (THUMBNAIL_SIZE[1], THUMBNAIL_SIZE[0]))

    def test_bbox_larger_than_target(self):
        """Test that drawn bbox is padded larger than the actual target."""
        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        cx, cy, bw, bh = 960, 540, 32, 22
        # Draw a bright green target (distinct from orange bbox and white label)
        frame[cy - bh // 2:cy + bh // 2, cx - bw // 2:cx + bw // 2] = (0, 255, 0)

        result = create_confirmation_thumbnail(frame, target_id=0, bbox=(cx, cy, bw, bh))
        decoded = decode_confirmation_thumbnail(result)
        self.assertIsNotNone(decoded)

        # Orange bbox pixels (high red, low green)
        orange_mask = (decoded[:, :, 2] > 200) & (decoded[:, :, 1] < 120)
        orange_coords = np.argwhere(orange_mask)
        self.assertGreater(len(orange_coords), 0, "No bounding box drawn")

        # Green target pixels (high green, low red and blue)
        green_mask = (decoded[:, :, 1] > 200) & (decoded[:, :, 0] < 50) & (decoded[:, :, 2] < 50)
        green_coords = np.argwhere(green_mask)
        self.assertGreater(len(green_coords), 0, "Target not visible")

        # Bbox should span a larger area than the target itself
        bbox_span_y = orange_coords[:, 0].max() - orange_coords[:, 0].min()
        bbox_span_x = orange_coords[:, 1].max() - orange_coords[:, 1].min()
        target_span_y = green_coords[:, 0].max() - green_coords[:, 0].min()
        target_span_x = green_coords[:, 1].max() - green_coords[:, 1].min()
        self.assertGreater(bbox_span_x, target_span_x * 1.3, "Bbox not wider than target")
        self.assertGreater(bbox_span_y, target_span_y * 1.3, "Bbox not taller than target")


class TestConfirmationThumbnailCropToggle(unittest.TestCase):
    """Tests for the crop_to_target toggle in create_confirmation_thumbnail."""

    def test_crop_disabled_shows_full_frame_marker_at_corner(self):
        """With crop_to_target=False, a bright corner marker in the original
        frame must be visible in the thumbnail (crop would cut it off)."""
        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        # Bright green 20x20 marker at top-left corner of the source frame.
        frame[0:20, 0:20] = (0, 255, 0)
        # Target is far away (bottom-right quadrant) so a centered crop would
        # discard the top-left corner entirely.
        bbox = (1700.0, 900.0, 60.0, 60.0)

        result = create_confirmation_thumbnail(
            frame, target_id=0, bbox=bbox, crop_to_target=False,
        )
        decoded = decode_confirmation_thumbnail(result)
        self.assertIsNotNone(decoded)

        # Top-left corner of the thumbnail should still contain green pixels.
        corner = decoded[0:20, 0:20]
        green_mask = (corner[:, :, 1] > 180) & (corner[:, :, 0] < 80) & (corner[:, :, 2] < 80)
        self.assertTrue(green_mask.any(), "Corner marker lost — frame was cropped")

    def test_crop_enabled_by_default_cuts_distant_corner(self):
        """Sanity: with the default (crop_to_target=True), the same distant
        corner marker is cropped out of the thumbnail."""
        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        frame[0:20, 0:20] = (0, 255, 0)
        bbox = (1700.0, 900.0, 60.0, 60.0)

        result = create_confirmation_thumbnail(frame, target_id=0, bbox=bbox)
        decoded = decode_confirmation_thumbnail(result)
        self.assertIsNotNone(decoded)

        corner = decoded[0:20, 0:20]
        green_mask = (corner[:, :, 1] > 180) & (corner[:, :, 0] < 80) & (corner[:, :, 2] < 80)
        self.assertFalse(green_mask.any(), "Crop did not remove the distant corner marker")

    def test_frame_bboxes_does_not_expand_crop(self):
        """Other detections (frame_bboxes) must NOT enlarge the crop — the
        selected target stays framed at the same size, so the output is
        byte-identical with or without them. (The old code expanded the crop
        to enclose every detection, shrinking the selected target.)"""
        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        frame[520:560, 940:980] = (0, 255, 0)  # selected target near center
        bbox = (960.0, 540.0, 40.0, 40.0)
        # A far-away detection in the corner the old code would expand to.
        distant = [(100.0, 100.0, 80.0, 80.0)]

        without = create_confirmation_thumbnail(frame, target_id=0, bbox=bbox)
        with_distant = create_confirmation_thumbnail(
            frame, target_id=0, bbox=bbox, frame_bboxes=distant)

        self.assertIsNotNone(without)
        self.assertEqual(without, with_distant)


class TestConfirmationThumbnailDegradedBanner(unittest.TestCase):
    """Tests for the degraded=True LOW-RES banner overlay."""

    def test_degraded_draws_red_banner_in_top_right(self):
        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        bbox = (960.0, 540.0, 80.0, 60.0)
        result = create_confirmation_thumbnail(
            frame, target_id=0, bbox=bbox, degraded=True,
        )
        decoded = decode_confirmation_thumbnail(result)
        self.assertIsNotNone(decoded)

        # Top-right quadrant should contain strong red pixels.
        h, w = decoded.shape[:2]
        quad = decoded[0:h // 4, w // 2:]
        # Banner color is pure red BGR(0,0,220); exclude the orange bbox label
        # BGR(0,69,255) by requiring low green as well.
        red_mask = (quad[:, :, 2] > 180) & (quad[:, :, 0] < 30) & (quad[:, :, 1] < 30)
        self.assertTrue(red_mask.any(), "LOW-RES banner not found in top-right quadrant")

    def test_non_degraded_has_no_red_banner(self):
        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        bbox = (960.0, 540.0, 80.0, 60.0)
        result = create_confirmation_thumbnail(frame, target_id=0, bbox=bbox)
        decoded = decode_confirmation_thumbnail(result)
        self.assertIsNotNone(decoded)

        h, w = decoded.shape[:2]
        quad = decoded[0:h // 4, w // 2:]
        # Banner color is pure red BGR(0,0,220); exclude the orange bbox label
        # BGR(0,69,255) by requiring low green as well.
        red_mask = (quad[:, :, 2] > 180) & (quad[:, :, 0] < 30) & (quad[:, :, 1] < 30)
        self.assertFalse(red_mask.any(), "Banner drawn even though degraded=False")

    def test_degraded_works_with_crop_disabled(self):
        """The crop_to_target=False path also supports the LOW-RES banner."""
        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        bbox = (960.0, 540.0, 80.0, 60.0)
        result = create_confirmation_thumbnail(
            frame, target_id=0, bbox=bbox, crop_to_target=False, degraded=True,
        )
        decoded = decode_confirmation_thumbnail(result)
        self.assertIsNotNone(decoded)

        h, w = decoded.shape[:2]
        quad = decoded[0:h // 4, w // 2:]
        # Banner color is pure red BGR(0,0,220); exclude the orange bbox label
        # BGR(0,69,255) by requiring low green as well.
        red_mask = (quad[:, :, 2] > 180) & (quad[:, :, 0] < 30) & (quad[:, :, 1] < 30)
        self.assertTrue(red_mask.any(), "LOW-RES banner missing on non-cropped path")


class TestDecodeConfirmationThumbnail(unittest.TestCase):
    """Tests for decode_confirmation_thumbnail function."""

    def test_round_trip_encoding(self):
        """Test that encoding and decoding preserves image structure."""
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        bbox = (320.0, 240.0, 100.0, 80.0)

        encoded = create_confirmation_thumbnail(frame, target_id=1, bbox=bbox)
        decoded = decode_confirmation_thumbnail(encoded)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.shape[2], 3)  # 3 channels
        self.assertEqual(decoded.shape[:2], (THUMBNAIL_SIZE[1], THUMBNAIL_SIZE[0]))

    def test_handles_none_input(self):
        """Test that None input returns None."""
        result = decode_confirmation_thumbnail(None)
        self.assertIsNone(result)

    def test_handles_empty_string(self):
        """Test that empty string returns None."""
        result = decode_confirmation_thumbnail("")
        self.assertIsNone(result)

    def test_handles_invalid_base64(self):
        """Test that invalid base64 returns None."""
        result = decode_confirmation_thumbnail("not_valid_base64!!!")
        self.assertIsNone(result)


if __name__ == '__main__':
    unittest.main()
