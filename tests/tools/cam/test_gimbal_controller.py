"""Tests for gimbal_controller pixel_to_delta and undistort_point math."""

import math
import sys
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np
import pytest

from navpy.modules.vision.gimbal_tracking_sample import GimbalAngularSample
from tests.detection_factory import make_detected_target

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from cam.gimbal_controller import (
    ANCHOR_NAMES,
    TrackingIntrinsics,
    build_bbox_tracking_command,
    parse_args,
    pixel_to_delta,
    resolve_tracking_intrinsics,
    undistort_point,
)
from cam.gimbal_tuning_geometry import anchor_pixel, step_zoom
from cam.gimbal_tuning_sample import build_tracker_target, tracking_bbox


# Typical SIYI ZR10 intrinsics at zoom 1
FX = 2066.49
FY = 2082.84
CX = 879.61
CY = 630.24

# SIYI ZR10 distortion coefficients
DIST = np.array([-0.3168865266009212, 0.43220713166267516,
                  -0.0003536970963693812, -0.002499539111554173,
                  -0.43507477093240304], dtype=np.float32)

K = np.array([[FX, 0, CX],
              [0, FY, CY],
              [0,  0,  1]], dtype=np.float64)


class TestParseArgs:
    def test_uses_detector_tracking_defaults(self):
        with patch.object(sys, "argv", ["gimbal_controller.py"]):
            args = parse_args()

        assert args.zoom == pytest.approx(1.0)
        assert args.anchor == 5
        assert not hasattr(args, "kp_yaw")
        assert not hasattr(args, "kp_pitch")
        assert args.max_rate == pytest.approx(100.0)


class TestPixelToDelta:
    """Unit tests for pixel_to_delta conversion."""

    def test_click_at_center_gives_zero_delta(self):
        dy, dp = pixel_to_delta(CX, CY, FX, FY, CX, CY)
        assert abs(dy) < 1e-9
        assert abs(dp) < 1e-9

    def test_click_right_of_center(self):
        px = CX + 200
        dy, dp = pixel_to_delta(px, CY, FX, FY, CX, CY)
        assert dy > 0, "Click right should produce positive yaw delta"
        assert abs(dp) < 1e-9, "Pitch delta should be zero for horizontal offset"
        expected = math.degrees(math.atan2(200, FX))
        assert abs(dy - expected) < 1e-9

    def test_click_left_of_center(self):
        px = CX - 200
        dy, dp = pixel_to_delta(px, CY, FX, FY, CX, CY)
        assert dy < 0, "Click left should produce negative yaw delta"
        assert abs(dp) < 1e-9

    def test_click_below_center(self):
        py = CY + 200
        dy, dp = pixel_to_delta(CX, py, FX, FY, CX, CY)
        assert abs(dy) < 1e-9, "Yaw delta should be zero for vertical offset"
        assert dp > 0, "Click below center should produce positive pitch delta (tilt down)"
        expected = math.degrees(math.atan2(200, FY))
        assert abs(dp - expected) < 1e-9

    def test_click_above_center(self):
        py = CY - 200
        dy, dp = pixel_to_delta(CX, py, FX, FY, CX, CY)
        assert abs(dy) < 1e-9
        assert dp < 0, "Click above center should produce negative pitch delta (tilt up)"

    def test_click_at_corner(self):
        px = CX + 300
        py = CY + 150
        dy, dp = pixel_to_delta(px, py, FX, FY, CX, CY)
        expected_dy = math.degrees(math.atan2(300, FX))
        expected_dp = math.degrees(math.atan2(150, FY))
        assert abs(dy - expected_dy) < 1e-9
        assert abs(dp - expected_dp) < 1e-9

    def test_zoom_scaling_halves_angle(self):
        """Doubling fx/fy (zoom 2x) should roughly halve the angle for the same pixel offset."""
        offset = 200
        dy_z1, dp_z1 = pixel_to_delta(CX + offset, CY + offset, FX, FY, CX, CY)
        dy_z2, dp_z2 = pixel_to_delta(CX + offset, CY + offset, FX * 2, FY * 2, CX, CY)

        # atan2(offset, f) vs atan2(offset, 2f) — not exactly half, but close
        # for small angles. Verify the zoom-2 delta is smaller.
        assert abs(dy_z2) < abs(dy_z1)
        assert abs(dp_z2) < abs(dp_z1)

        # For these typical values the ratio should be close to 2
        ratio_yaw = dy_z1 / dy_z2
        ratio_pitch = dp_z1 / dp_z2
        assert 1.9 < ratio_yaw < 2.1
        assert 1.9 < ratio_pitch < 2.1

    def test_symmetry(self):
        """Clicks equidistant from center should produce equal magnitude deltas."""
        offset = 400
        dy_right, _ = pixel_to_delta(CX + offset, CY, FX, FY, CX, CY)
        dy_left, _ = pixel_to_delta(CX - offset, CY, FX, FY, CX, CY)
        assert abs(dy_right + dy_left) < 1e-9

        _, dp_below = pixel_to_delta(CX, CY + offset, FX, FY, CX, CY)
        _, dp_above = pixel_to_delta(CX, CY - offset, FX, FY, CX, CY)
        assert abs(dp_below + dp_above) < 1e-9


class TestUndistortPoint:
    """Unit tests for undistort_point lens correction."""

    def test_principal_point_unchanged(self):
        """Point at optical center should not move after undistortion."""
        ux, uy = undistort_point(CX, CY, K, DIST)
        assert abs(ux - CX) < 0.1
        assert abs(uy - CY) < 0.1

    def test_edge_point_moves_outward_with_barrel_distortion(self):
        """With negative k1 (barrel distortion), edge points should move outward."""
        # Point far from center
        px, py = CX + 500, CY + 300
        ux, uy = undistort_point(px, py, K, DIST)
        # Barrel distortion compresses edges; undistorting should push outward
        raw_dist = math.hypot(px - CX, py - CY)
        undist_dist = math.hypot(ux - CX, uy - CY)
        assert undist_dist > raw_dist

    def test_no_distortion_is_identity(self):
        """With zero distortion, undistort should return the same point."""
        zero_dist = np.zeros(5, dtype=np.float32)
        px, py = 1200.0, 400.0
        ux, uy = undistort_point(px, py, K, zero_dist)
        assert abs(ux - px) < 0.1
        assert abs(uy - py) < 0.1

    def test_undistort_affects_angle_calculation(self):
        """Undistorted point should produce a different angle than raw point."""
        px, py = CX + 500, CY
        ux, uy = undistort_point(px, py, K, DIST)
        dy_raw, _ = pixel_to_delta(px, py, FX, FY, CX, CY)
        dy_undist, _ = pixel_to_delta(ux, uy, FX, FY, CX, CY)
        # With barrel distortion, undistorted is further out -> larger angle
        assert abs(dy_undist) > abs(dy_raw)


# Image dimensions for anchor tests
IMG_W = 1920
IMG_H = 1080


class TestAnchorPixel:
    """Unit tests for _anchor_pixel helper."""

    def test_all_nine_modes_return_correct_positions(self):
        expected = {
            1: (0, IMG_H),          2: (IMG_W/2, IMG_H),   3: (IMG_W, IMG_H),
            4: (0, IMG_H/2),        5: (CX, CY),           6: (IMG_W, IMG_H/2),
            7: (0, 0),              8: (IMG_W/2, 0),        9: (IMG_W, 0),
        }
        for mode, (ex, ey) in expected.items():
            ax, ay = anchor_pixel(mode, IMG_W, IMG_H, CX, CY)
            assert ax == pytest.approx(ex), f"Mode {mode} x: got {ax}, expected {ex}"
            assert ay == pytest.approx(ey), f"Mode {mode} y: got {ay}, expected {ey}"

    def test_center_anchor_uses_principal_point(self):
        ax, ay = anchor_pixel(5, IMG_W, IMG_H, CX, CY)
        assert ax == pytest.approx(CX)
        assert ay == pytest.approx(CY)

    def test_corners_at_image_boundaries(self):
        # TL = (0,0), BR = (img_w, img_h)
        assert anchor_pixel(7, IMG_W, IMG_H, CX, CY) == (0, 0)
        assert anchor_pixel(3, IMG_W, IMG_H, CX, CY) == (IMG_W, IMG_H)

    def test_invalid_mode_raises(self):
        with pytest.raises(KeyError):
            anchor_pixel(0, IMG_W, IMG_H, CX, CY)

    def test_anchor_names_cover_all_modes(self):
        for mode in range(1, 10):
            assert mode in ANCHOR_NAMES


class TestAnchorMath:
    """Tests for anchor-aware click delta computation."""

    def _compute_delta(self, click_px, click_py, anchor_mode):
        """Replicate the anchor-aware delta math from gimbal_controller."""
        zero_dist = np.zeros(5, dtype=np.float32)
        # Use zero distortion so undistort is identity
        ux, uy = undistort_point(click_px, click_py, K, zero_dist)
        ax, ay = anchor_pixel(anchor_mode, IMG_W, IMG_H, CX, CY)
        uax, uay = undistort_point(ax, ay, K, zero_dist)
        click_yaw, click_pitch = pixel_to_delta(ux, uy, FX, FY, CX, CY)
        anchor_yaw, anchor_pitch = pixel_to_delta(uax, uay, FX, FY, CX, CY)
        return click_yaw - anchor_yaw, click_pitch - anchor_pitch

    def test_center_anchor_matches_raw_pixel_to_delta(self):
        """Anchor=center (5) should produce the same delta as raw pixel_to_delta."""
        click_px, click_py = CX + 300, CY + 150
        dy_anchor, dp_anchor = self._compute_delta(click_px, click_py, 5)
        dy_raw, dp_raw = pixel_to_delta(click_px, click_py, FX, FY, CX, CY)
        assert dy_anchor == pytest.approx(dy_raw, abs=0.01)
        assert dp_anchor == pytest.approx(dp_raw, abs=0.01)

    def test_corner_anchor_shifts_delta(self):
        """Using a corner anchor should shift the delta by the corner's angle offset."""
        click_px, click_py = CX, CY  # click at optical center
        # With center anchor (5), clicking at center gives zero delta
        dy_center, dp_center = self._compute_delta(click_px, click_py, 5)
        assert abs(dy_center) < 1e-6
        assert abs(dp_center) < 1e-6
        # With TL anchor (7), clicking at center should offset by -TL angle
        dy_tl, dp_tl = self._compute_delta(click_px, click_py, 7)
        # TL is at (0,0) which is left-of and above center, so its angle is negative yaw, negative pitch
        # delta = center_angle(0,0) - TL_angle(negative, negative) = positive, positive
        assert dy_tl > 0, "TL anchor should shift yaw positive"
        assert dp_tl > 0, "TL anchor should shift pitch positive"

    def test_opposite_corners_produce_opposite_offsets(self):
        """TL and BR anchors should produce opposite delta offsets for a centered click."""
        click_px, click_py = CX, CY
        dy_tl, dp_tl = self._compute_delta(click_px, click_py, 7)  # TL
        dy_br, dp_br = self._compute_delta(click_px, click_py, 3)  # BR
        # They should be opposite in sign
        assert dy_tl * dy_br < 0, "TL and BR yaw offsets should have opposite sign"
        assert dp_tl * dp_br < 0, "TL and BR pitch offsets should have opposite sign"

    def test_bl_br_same_pitch_opposite_yaw_sign(self):
        """BL and BR anchors should have same pitch offset and opposite yaw sign."""
        click_px, click_py = CX, CY
        dy_bl, dp_bl = self._compute_delta(click_px, click_py, 1)  # BL
        dy_br, dp_br = self._compute_delta(click_px, click_py, 3)  # BR
        assert dp_bl == pytest.approx(dp_br, abs=0.01)
        assert dy_bl > 0, "BL anchor (left edge) should shift yaw positive"
        assert dy_br < 0, "BR anchor (right edge) should shift yaw negative"


class TestResolveTrackingIntrinsics:
    def test_returns_exact_calibration_when_present(self):
        intrinsics = TrackingIntrinsics(
            zoom=2.0,
            fx=2000.0,
            fy=2100.0,
            cx=500.0,
            cy=400.0,
            k=np.array([[2000.0, 0.0, 500.0], [0.0, 2100.0, 400.0], [0.0, 0.0, 1.0]], dtype=np.float64),
            dist_coeffs=np.zeros(5, dtype=np.float32),
            source="calibrated",
            reference_zoom=2.0,
        )

        resolved = resolve_tracking_intrinsics({2.0: intrinsics}, 2.0)

        assert resolved is intrinsics

    def test_scales_from_nearest_reference_zoom(self):
        reference = TrackingIntrinsics(
            zoom=2.0,
            fx=2000.0,
            fy=2200.0,
            cx=500.0,
            cy=400.0,
            k=np.array([[2000.0, 0.0, 500.0], [0.0, 2200.0, 400.0], [0.0, 0.0, 1.0]], dtype=np.float64),
            dist_coeffs=np.zeros(5, dtype=np.float32),
            source="calibrated",
            reference_zoom=2.0,
        )

        resolved = resolve_tracking_intrinsics({2.0: reference}, 3.0)

        assert resolved.source == "estimated"
        assert resolved.reference_zoom == 2.0
        assert resolved.fx == pytest.approx(3000.0)
        assert resolved.fy == pytest.approx(3300.0)
        assert resolved.cx == pytest.approx(500.0)
        assert resolved.cy == pytest.approx(400.0)


class TestBuildBboxTrackingCommand:
    def _intrinsics(self):
        return TrackingIntrinsics(
            zoom=1.0,
            fx=1000.0,
            fy=1000.0,
            cx=500.0,
            cy=500.0,
            k=np.array([[1000.0, 0.0, 500.0], [0.0, 1000.0, 500.0], [0.0, 0.0, 1.0]], dtype=np.float64),
            dist_coeffs=np.zeros(5, dtype=np.float32),
            source="calibrated",
            reference_zoom=1.0,
        )

    def test_center_anchor_has_zero_delta_for_centered_bbox(self):
        command = build_bbox_tracking_command(
            (500.0, 500.0, 120.0, 80.0),
            self._intrinsics(),
            anchor_mode=5,
            image_width=1000,
            image_height=1000,
        )

        assert command is not None
        assert command.anchor_pixel == pytest.approx((500.0, 500.0))
        assert command.delta_yaw_deg == pytest.approx(0.0)
        assert command.delta_pitch_deg == pytest.approx(0.0)

    def test_corner_anchor_offsets_delta_and_tracking_center(self):
        command = build_bbox_tracking_command(
            (500.0, 500.0, 120.0, 80.0),
            self._intrinsics(),
            anchor_mode=7,
            image_width=1000,
            image_height=1000,
        )

        assert command is not None
        assert command.anchor_pixel == pytest.approx((0.0, 0.0))
        assert command.delta_yaw_deg > 0.0
        assert command.delta_pitch_deg > 0.0
        assert command.tracking_k[0, 2] == pytest.approx(command.undistorted_anchor[0])
        assert command.tracking_k[1, 2] == pytest.approx(command.undistorted_anchor[1])


class TestBuildTrackerTarget:
    def test_builds_current_angular_sample_from_source_time(self):
        command = build_bbox_tracking_command(
            (500.0, 500.0, 120.0, 80.0),
            TrackingIntrinsics(
                zoom=1.0,
                fx=1000.0,
                fy=1000.0,
                cx=500.0,
                cy=500.0,
                k=np.array([[1000.0, 0.0, 500.0], [0.0, 1000.0, 500.0], [0.0, 0.0, 1.0]], dtype=np.float64),
                dist_coeffs=np.zeros(5, dtype=np.float32),
                source="calibrated",
                reference_zoom=1.0,
            ),
            anchor_mode=5,
            image_width=1000,
            image_height=1000,
        )
        source_target = make_detected_target(
            obj_id=7,
            timestamp=42.0,
            tracking_bbox_cxcywh=(500.0, 500.0, 120.0, 80.0),
        )

        tracker_target = build_tracker_target(command, source_target)

        assert isinstance(tracker_target, GimbalAngularSample)
        assert tracker_target.source_timestamp_s == pytest.approx(42.0)
        assert tracker_target.yaw_error_rad == pytest.approx(
            math.radians(command.delta_yaw_deg)
        )
        assert tracker_target.pitch_error_rad == pytest.approx(
            math.radians(command.delta_pitch_deg)
        )

    def test_prefers_live_tracking_bbox_over_confirmation_bbox(self):
        command = build_bbox_tracking_command(
            (500.0, 500.0, 120.0, 80.0),
            TrackingIntrinsics(
                zoom=1.0,
                fx=1000.0,
                fy=1000.0,
                cx=500.0,
                cy=500.0,
                k=np.array([[1000.0, 0.0, 500.0], [0.0, 1000.0, 500.0], [0.0, 0.0, 1.0]], dtype=np.float64),
                dist_coeffs=np.zeros(5, dtype=np.float32),
                source="calibrated",
                reference_zoom=1.0,
            ),
            anchor_mode=5,
            image_width=1000,
            image_height=1000,
        )
        source_target = make_detected_target(
            bbox_cxcywh=(300.0, 300.0, 90.0, 60.0),
            tracking_bbox_cxcywh=(520.0, 510.0, 120.0, 80.0),
            timestamp=7.0,
        )

        assert tracking_bbox(source_target) == pytest.approx(
            (520.0, 510.0, 120.0, 80.0)
        )
        tracker_target = build_tracker_target(command, source_target)

        assert isinstance(tracker_target, GimbalAngularSample)
        assert tracker_target.source_timestamp_s == pytest.approx(7.0)


class TestStepZoom:
    def test_steps_forward_and_backward(self):
        levels = [1.0, 2.0, 4.0]

        assert step_zoom(1.0, levels, 1) == pytest.approx(2.0)
        assert step_zoom(4.0, levels, 1) == pytest.approx(4.0)
        assert step_zoom(4.0, levels, -1) == pytest.approx(2.0)
        assert step_zoom(1.0, levels, -1) == pytest.approx(1.0)
