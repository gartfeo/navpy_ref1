"""Jacobian-based gimbal steering for camera calibration.

Steers a gimbal so that a tracked object (e.g. checkerboard) reaches
specific pixel positions in the frame.  Uses an empirical Jacobian
(probe-and-measure) rather than calibrated intrinsics.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.frame_provider import FrameProvider
from navpy.modules.vision.peripheral.gimbal_abc import GimbalAbc


# ---------------------------------------------------------------------------
# Config & result types
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class NavigatorConfig:
    """Tuning knobs for CalibrationNavigator."""
    probe_step_deg: float = 3.0
    tolerance_px: float = 30.0
    max_iterations: int = 10
    step_clamp_deg: float = 5.0
    settle_time: float = 0.8
    flush_count: int = 15
    flush_interval: float = 0.02


@dataclass(frozen=True)
class NavigateResult:
    """Outcome of a single navigate_to() call."""
    reached: bool
    final_xy: tuple[float, float] | None
    final_yaw: float
    final_pitch: float
    pixel_error: float
    iterations: int
    frame: np.ndarray | None


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------

# Center first, then clockwise perimeter from ML
_ANCHOR_LABELS = ("C", "ML", "TL", "TC", "TR", "MR", "BR", "BC", "BL")
_ANCHOR_FRACS = (
    (0.5, 0.5), (0.0, 0.5), (0.0, 0.0),
    (0.5, 0.0), (1.0, 0.0), (1.0, 0.5),
    (1.0, 1.0), (0.5, 1.0), (0.0, 1.0),
)


def compute_target_positions(
    img_w: int,
    img_h: int,
    margin: float = 0.15,
    positions: list[str] | None = None,
    board_half_w: float = 0.0,
    board_half_h: float = 0.0,
    padding_frac: float = 0.25,
) -> list[tuple[float, float, str]]:
    """Return pixel positions for calibration anchors.

    When board_half_w/h are provided (from a detected board bounding box),
    margins are computed adaptively so the entire board stays within the
    frame at every anchor.  Falls back to the fixed fractional margin when
    board dimensions are not available.

    Args:
        img_w: Frame width in pixels.
        img_h: Frame height in pixels.
        margin: Fractional inset from edges (fallback when board size unknown).
        positions: Subset of anchor labels to include, or None for all 9.
        board_half_w: Half the board bounding-box width in pixels.
        board_half_h: Half the board bounding-box height in pixels.
        padding_frac: Fraction of *remaining screen space* (screen_half - board_half)
            used as extra padding beyond the board edge (default 0.25 = 25%).

    Returns:
        List of (x, y, label) tuples.
    """
    if board_half_w > 0 and board_half_h > 0:
        # Remaining space between board edge and frame edge (per side)
        remain_x = img_w / 2.0 - board_half_w
        remain_y = img_h / 2.0 - board_half_h
        # Inset = board half-size + fraction of remaining space as safety
        inset_x = board_half_w + max(remain_x * padding_frac, 0.0)
        inset_y = board_half_h + max(remain_y * padding_frac, 0.0)
        x_min = max(inset_x, 1.0)
        x_max = max(img_w - inset_x, x_min + 1.0)
        y_min = max(inset_y, 1.0)
        y_max = max(img_h - inset_y, y_min + 1.0)
    else:
        x_min = margin * img_w
        x_max = (1 - margin) * img_w
        y_min = margin * img_h
        y_max = (1 - margin) * img_h

    result = []
    for (fx, fy), label in zip(_ANCHOR_FRACS, _ANCHOR_LABELS):
        if positions is not None and label not in positions:
            continue
        x = x_min + fx * (x_max - x_min)
        y = y_min + fy * (y_max - y_min)
        result.append((x, y, label))
    return result


def measure_board_size(corners: np.ndarray) -> tuple[float, float]:
    """Return (half_w, half_h) of the board bounding box from corners.

    Args:
        corners: Corner array from BoardDetector.detect_corners().

    Returns:
        (half_width, half_height) in pixels.
    """
    xs = corners[:, 0, 0]
    ys = corners[:, 0, 1]
    half_w = (float(xs.max()) - float(xs.min())) / 2.0
    half_h = (float(ys.max()) - float(ys.min())) / 2.0
    return half_w, half_h


def solve_angle_delta(
    jacobian: np.ndarray, pixel_error: np.ndarray
) -> np.ndarray:
    """Compute angle correction from pixel error via J^-1 * error.

    Args:
        jacobian: 2x2 array mapping [d_yaw, d_pitch] -> [d_px_x, d_px_y].
        pixel_error: 2-element array [err_x, err_y] in pixels.

    Returns:
        2-element array [d_yaw, d_pitch] in degrees.
    """
    return np.linalg.solve(jacobian, pixel_error)


# ---------------------------------------------------------------------------
# Frame handling
# ---------------------------------------------------------------------------

DetectFn = Callable[[np.ndarray], Optional[tuple[float, float]]]


def flush_and_detect(
    frame_provider: FrameProvider,
    detect_fn: DetectFn,
    settle_time: float,
    flush_count: int,
    flush_interval: float,
) -> tuple[np.ndarray, tuple[float, float]] | None:
    """Wait for fresh frames, then detect the object.

    Uses sequence-number based waiting when the provider supports it
    (all FrameProviders do via wait_for_newer_frame).  Falls back to
    time-based flushing for robustness.

    Returns (frame, (cx, cy)) or None if detection fails after retries.
    """
    time.sleep(settle_time)

    # Drain stale frames by advancing sequence
    seq = frame_provider.get_frame_seq()
    for _ in range(flush_count):
        result = frame_provider.wait_for_newer_frame(seq, timeout=flush_interval)
        if result is not None:
            _, _, _, seq = result

    # Retry detection up to 3 times on fresh frames
    for _ in range(3):
        result = frame_provider.wait_for_newer_frame(seq, timeout=1.0)
        if result is None:
            continue
        frame, _, _, seq = result
        if frame is None:
            continue
        center = detect_fn(frame)
        if center is not None:
            return frame, center

    return None


# ---------------------------------------------------------------------------
# Jacobian measurement
# ---------------------------------------------------------------------------

def probe_axis(
    gimbal: GimbalAbc,
    frame_provider: FrameProvider,
    detect_fn: DetectFn,
    base_yaw: float,
    base_pitch: float,
    base_cx: float,
    base_cy: float,
    axis: str,
    probe_deg: float,
    config: NavigatorConfig,
    log: logging.Logger | None = None,
) -> np.ndarray | None:
    """Probe one gimbal axis and return pixel displacement per degree.

    Moves the gimbal +probe_deg on the given axis, measures pixel shift.
    If the object is lost, tries -probe_deg.  Always restores gimbal to
    the base position.

    Returns [dx_px, dy_px] per degree, or None on failure.
    """
    for sign in (+1, -1):
        step = sign * probe_deg
        if axis == "yaw":
            yaw, pitch = base_yaw + step, base_pitch
        else:
            yaw, pitch = base_yaw, base_pitch + step

        if log:
            log.info("  probe %s %+.1f: set_att(yaw=%.1f, pitch=%.1f)",
                     axis, step, yaw, pitch)

        gimbal.set_att(Attitude(pitch=pitch, yaw=yaw, roll=0))

        result = flush_and_detect(
            frame_provider, detect_fn,
            config.settle_time, config.flush_count, config.flush_interval,
        )

        # Restore base position
        gimbal.set_att(Attitude(pitch=base_pitch, yaw=base_yaw, roll=0))

        if result is not None:
            _, (cx, cy) = result
            dx_px = cx - base_cx
            dy_px = cy - base_cy
            if log:
                log.info("  probe %s %+.1f: base=(%.0f,%.0f) now=(%.0f,%.0f) shift=(%.1f,%.1f)px",
                         axis, step, base_cx, base_cy, cx, cy, dx_px, dy_px)
            return np.array([dx_px / step, dy_px / step])

        if log:
            log.info("  probe %s %+.1f: detection lost", axis, step)

    return None


def measure_jacobian(
    gimbal: GimbalAbc,
    frame_provider: FrameProvider,
    detect_fn: DetectFn,
    base_yaw: float,
    base_pitch: float,
    base_cx: float,
    base_cy: float,
    config: NavigatorConfig,
    log: logging.Logger | None = None,
    probe_step_override: float | None = None,
) -> np.ndarray | None:
    """Probe both axes and build the 2x2 Jacobian.

    Returns a 2x2 array where column 0 = yaw response, column 1 = pitch
    response.  Returns None if either axis fails or the result is singular.
    """
    step = probe_step_override if probe_step_override is not None else config.probe_step_deg

    yaw_col = probe_axis(
        gimbal, frame_provider, detect_fn,
        base_yaw, base_pitch, base_cx, base_cy,
        "yaw", step, config, log=log,
    )
    if yaw_col is None:
        return None

    pitch_col = probe_axis(
        gimbal, frame_provider, detect_fn,
        base_yaw, base_pitch, base_cx, base_cy,
        "pitch", step, config, log=log,
    )
    if pitch_col is None:
        return None

    J = np.column_stack([yaw_col, pitch_col])

    # Reject singular or near-singular matrices
    if abs(np.linalg.det(J)) < 1e-6:
        return None

    return J


# ---------------------------------------------------------------------------
# CalibrationNavigator
# ---------------------------------------------------------------------------

class CalibrationNavigator:
    """Steers a gimbal to place a detected object at target pixel positions."""

    def __init__(
        self,
        gimbal: GimbalAbc,
        frame_provider: FrameProvider,
        detect_fn: DetectFn,
        config: NavigatorConfig | None = None,
        logger: logging.Logger | None = None,
    ):
        self._gimbal = gimbal
        self._fp = frame_provider
        self._detect_fn = detect_fn
        self._config = config or NavigatorConfig()
        self._img_w = 0
        self._img_h = 0
        self._board_half_w = 0.0
        self._board_half_h = 0.0
        self._log = logger or logging.getLogger(__name__)
        self._jacobian: np.ndarray | None = None

    def set_frame_info(self, img_w: int, img_h: int,
                       board_half_w: float, board_half_h: float):
        """Set frame and board dimensions for safe-zone step scaling."""
        self._img_w = img_w
        self._img_h = img_h
        self._board_half_w = board_half_w
        self._board_half_h = board_half_h

    def measure_jacobian(
        self,
        yaw: float,
        pitch: float,
        img_w: int = 0,
        img_h: int = 0,
        board_half_w: float = 0.0,
        board_half_h: float = 0.0,
    ) -> np.ndarray | None:
        """Detect object, compute a safe probe step, probe both axes, cache.

        When img_w/h and board_half_w/h are provided, the probe step is
        computed dynamically: half the remaining space between the board
        edge and the nearest frame edge, converted to degrees using a
        coarse initial 1-degree probe.  This prevents overshooting at
        high zoom where the board fills most of the frame.
        """
        self._gimbal.set_att(Attitude(pitch=pitch, yaw=yaw, roll=0))

        result = flush_and_detect(
            self._fp, self._detect_fn,
            self._config.settle_time,
            self._config.flush_count,
            self._config.flush_interval,
        )
        if result is None:
            self._log.warning("measure_jacobian: object not detected at base")
            return None

        _, (cx, cy) = result
        self._log.info("measure_jacobian: base detection at (%.0f, %.0f)", cx, cy)

        probe_step = self._config.probe_step_deg

        if img_w > 0 and img_h > 0 and board_half_w > 0 and board_half_h > 0:
            # Remaining pixels from board edge to nearest frame edge
            remain_x = min(cx - board_half_w, img_w - cx - board_half_w)
            remain_y = min(cy - board_half_h, img_h - cy - board_half_h)
            remain_px = max(min(remain_x, remain_y), 10.0)

            # Small 1-degree probe to estimate px/deg, then compute safe step
            small_col = probe_axis(
                self._gimbal, self._fp, self._detect_fn,
                yaw, pitch, cx, cy,
                "pitch", 1.0, self._config, log=self._log,
            )
            if small_col is not None:
                px_per_deg = max(abs(small_col[0]), abs(small_col[1]), 1.0)
                # Use half the remaining space as probe distance
                probe_step = max(0.5, (remain_px * 0.5) / px_per_deg)
                self._log.info(
                    "measure_jacobian: remain=%.0fpx, px/deg=%.1f, probe_step=%.2f deg",
                    remain_px, px_per_deg, probe_step,
                )

        J = measure_jacobian(
            self._gimbal, self._fp, self._detect_fn,
            yaw, pitch, cx, cy, self._config,
            log=self._log, probe_step_override=probe_step,
        )
        if J is not None:
            self._jacobian = J
            self._log.info("Jacobian measured: %s", J)
        else:
            self._log.warning("measure_jacobian: failed (singular or lost)")
        return J

    def navigate_to(
        self,
        target_xy: tuple[float, float],
        yaw: float,
        pitch: float,
        jacobian: np.ndarray | None = None,
    ) -> NavigateResult:
        """Iteratively steer the object to target_xy.

        Args:
            target_xy: Target pixel position (x, y).
            yaw: Starting gimbal yaw (degrees).
            pitch: Starting gimbal pitch (degrees).
            jacobian: Optional Jacobian override; uses cached if None.

        Returns:
            NavigateResult with convergence status and final state.
        """
        J = jacobian if jacobian is not None else self._jacobian
        if J is None:
            return NavigateResult(
                reached=False, final_xy=None,
                final_yaw=yaw, final_pitch=pitch,
                pixel_error=float("inf"), iterations=0, frame=None,
            )

        cfg = self._config
        cur_yaw, cur_pitch = yaw, pitch
        good_yaw, good_pitch = yaw, pitch
        last_frame = None
        last_xy: tuple[float, float] | None = None
        last_error = float("inf")

        for iteration in range(1, cfg.max_iterations + 1):
            result = flush_and_detect(
                self._fp, self._detect_fn,
                cfg.settle_time, cfg.flush_count, cfg.flush_interval,
            )

            if result is None:
                # Detection lost — retreat halfway to last good position
                mid_yaw = (good_yaw + cur_yaw) / 2
                mid_pitch = (good_pitch + cur_pitch) / 2
                self._gimbal.set_att(
                    Attitude(pitch=mid_pitch, yaw=mid_yaw, roll=0)
                )
                cur_yaw, cur_pitch = mid_yaw, mid_pitch
                self._log.info(
                    "navigate_to: detection lost, retreated to yaw=%.2f pitch=%.2f",
                    cur_yaw, cur_pitch,
                )
                continue

            frame, (cx, cy) = result
            last_frame = frame
            last_xy = (cx, cy)
            good_yaw, good_pitch = cur_yaw, cur_pitch

            err_x = target_xy[0] - cx
            err_y = target_xy[1] - cy
            error_dist = float(np.hypot(err_x, err_y))
            last_error = error_dist

            if error_dist <= cfg.tolerance_px:
                self._log.info(
                    "navigate_to: converged in %d iterations (error=%.1f px)",
                    iteration, error_dist,
                )
                return NavigateResult(
                    reached=True, final_xy=last_xy,
                    final_yaw=cur_yaw, final_pitch=cur_pitch,
                    pixel_error=error_dist, iterations=iteration,
                    frame=last_frame,
                )

            pixel_error = np.array([err_x, err_y])
            delta = solve_angle_delta(J, pixel_error)

            # Clamp per-axis
            delta = np.clip(delta, -cfg.step_clamp_deg, cfg.step_clamp_deg)

            # Safe-zone scaling: predict pixel displacement, scale down
            # if the board would leave the frame
            if self._img_w > 0 and self._board_half_w > 0:
                predicted_px = J @ delta
                new_cx = cx + predicted_px[0]
                new_cy = cy + predicted_px[1]
                margin_px = 30.0
                safe_min_x = self._board_half_w + margin_px
                safe_max_x = self._img_w - safe_min_x
                safe_min_y = self._board_half_h + margin_px
                safe_max_y = self._img_h - safe_min_y
                scale = 1.0
                if predicted_px[0] != 0:
                    if new_cx < safe_min_x:
                        scale = min(scale, (safe_min_x - cx) / predicted_px[0])
                    elif new_cx > safe_max_x:
                        scale = min(scale, (safe_max_x - cx) / predicted_px[0])
                if predicted_px[1] != 0:
                    if new_cy < safe_min_y:
                        scale = min(scale, (safe_min_y - cy) / predicted_px[1])
                    elif new_cy > safe_max_y:
                        scale = min(scale, (safe_max_y - cy) / predicted_px[1])
                if scale < 1.0:
                    delta = delta * max(scale, 0.1)

            cur_yaw += delta[0]
            cur_pitch += delta[1]
            self._gimbal.set_att(
                Attitude(pitch=cur_pitch, yaw=cur_yaw, roll=0)
            )

        self._log.info(
            "navigate_to: max iterations reached (error=%.1f px)", last_error,
        )
        return NavigateResult(
            reached=False, final_xy=last_xy,
            final_yaw=cur_yaw, final_pitch=cur_pitch,
            pixel_error=last_error, iterations=cfg.max_iterations,
            frame=last_frame,
        )
