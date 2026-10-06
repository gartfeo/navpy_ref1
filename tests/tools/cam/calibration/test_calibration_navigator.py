"""Unit tests for CalibrationNavigator (no hardware required)."""

import sys
import time
import threading
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

# Ensure tools/ and src/ are importable
sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "tools"))
sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "src"))

from cam.calibration.calibration_navigator import (
    CalibrationNavigator,
    NavigatorConfig,
    NavigateResult,
    compute_poi_positions,
    measure_board_size,
    solve_angle_delta,
    flush_and_detect,
    probe_axis,
    measure_jacobian,
)
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.frame_provider import FrameProvider
from navpy.modules.vision.peripheral.gimbal_abc import GimbalAbc, GimbalData


# ---------------------------------------------------------------------------
# Mock infrastructure
# ---------------------------------------------------------------------------

class _NullLogger:
    def info(self, msg, *a): pass
    def warning(self, msg, *a): pass
    def error(self, msg, *a): pass
    def debug(self, msg, *a): pass


class MockGimbal(GimbalAbc):
    """Records set_att() calls, returns configurable attitude."""

    def __init__(self):
        self.calls: list[Attitude] = []
        self._att = Attitude(pitch=0, yaw=0, roll=0)

    def set_att(self, att: Attitude):
        self.calls.append(att)
        self._att = att

    def get_data(self) -> GimbalData:
        return GimbalData(att=self._att)


def _make_push_fp() -> FrameProvider:
    """Create a push-mode FrameProvider (no capture thread)."""
    fp = FrameProvider(source=None, logger=_NullLogger())
    fp.start()
    return fp


# Pusher threads started by _feed_frames, joined after each test by the autouse
# fixture below so none outlives its test. Each sleeps between frames, so a test
# that converges before all frames are pushed would otherwise leave one running
# (and a global time.sleep monkeypatch elsewhere would turn it into a busy spin).
_active_pushers: list[threading.Thread] = []


def _feed_frames(fp: FrameProvider, frames: list[np.ndarray], interval: float = 0.01):
    """Push frames into a FrameProvider from a background thread."""
    def _pusher():
        for f in frames:
            time.sleep(interval)
            fp.push_frame(f)
    t = threading.Thread(target=_pusher, daemon=True)
    _active_pushers.append(t)
    t.start()
    return t


@pytest.fixture(autouse=True)
def _join_frame_pushers():
    """Join every _feed_frames pusher after each test so none leaks.

    A pusher that does not stop within the timeout is surfaced as a teardown
    failure instead of being silently accepted (a timed join that ignores
    is_alive would leave the daemon running and defeat the join's purpose).
    """
    yield
    # Join and drain EVERY pusher (emptying the module-global list so a wedge
    # cannot carry into the next test), collect any survivor, then report -- do
    # not abort on the first wedge.
    survivors = []
    while _active_pushers:
        pusher = _active_pushers.pop()
        pusher.join(timeout=2.0)
        if pusher.is_alive():
            survivors.append(pusher)
    assert not survivors, f"{len(survivors)} calibration frame-pusher(s) did not stop"


# ---------------------------------------------------------------------------
# Simulated detect_fn: pixel position changes with gimbal angle
# ---------------------------------------------------------------------------

def _make_sim_detect(
    base_cx: float, base_cy: float,
    px_per_deg_yaw_x: float, px_per_deg_yaw_y: float,
    px_per_deg_pitch_x: float, px_per_deg_pitch_y: float,
    gimbal: MockGimbal,
    base_yaw: float = 0.0, base_pitch: float = 0.0,
):
    """Return a detect_fn that simulates pixel motion based on gimbal angle."""
    def detect_fn(frame):
        att = gimbal._att
        dy = att.yaw - base_yaw
        dp = att.pitch - base_pitch
        cx = base_cx + dy * px_per_deg_yaw_x + dp * px_per_deg_pitch_x
        cy = base_cy + dy * px_per_deg_yaw_y + dp * px_per_deg_pitch_y
        return (cx, cy)
    return detect_fn


# ---------------------------------------------------------------------------
# TestComputePoiPositions
# ---------------------------------------------------------------------------

class TestComputePoiPositions:
    def test_returns_9_positions_by_default(self):
        pois = compute_poi_positions(1920, 1080)
        assert len(pois) == 9

    def test_all_labels_present(self):
        pois = compute_poi_positions(1920, 1080)
        labels = [t[2] for t in pois]
        assert set(labels) == {"TL", "TC", "TR", "ML", "C", "MR", "BL", "BC", "BR"}

    def test_margin_math(self):
        pois = compute_poi_positions(1000, 500, margin=0.1)
        by_label = {t[2]: (t[0], t[1]) for t in pois}

        # TL: margin=10%, so x=100, y=50
        assert abs(by_label["TL"][0] - 100.0) < 0.01
        assert abs(by_label["TL"][1] - 50.0) < 0.01

        # BR: x=900, y=450
        assert abs(by_label["BR"][0] - 900.0) < 0.01
        assert abs(by_label["BR"][1] - 450.0) < 0.01

        # Center: x=500, y=250
        assert abs(by_label["C"][0] - 500.0) < 0.01
        assert abs(by_label["C"][1] - 250.0) < 0.01

    def test_custom_subset(self):
        pois = compute_poi_positions(1920, 1080, positions=["TL", "BR", "C"])
        assert len(pois) == 3
        labels = {t[2] for t in pois}
        assert labels == {"TL", "BR", "C"}

    def test_zero_margin_uses_full_frame(self):
        pois = compute_poi_positions(1920, 1080, margin=0.0)
        by_label = {t[2]: (t[0], t[1]) for t in pois}
        assert abs(by_label["TL"][0]) < 0.01
        assert abs(by_label["TL"][1]) < 0.01
        assert abs(by_label["BR"][0] - 1920.0) < 0.01
        assert abs(by_label["BR"][1] - 1080.0) < 0.01

    def test_adaptive_board_size(self):
        """With board_half_w/h, POIs are inset by board_half + frac * remaining."""
        # 1920x1080, board half=200x150, padding_frac=0.0 → inset = board half only
        pois = compute_poi_positions(
            1920, 1080, board_half_w=200, board_half_h=150, padding_frac=0.0)
        by_label = {t[2]: (t[0], t[1]) for t in pois}
        # TL at (200, 150) — exactly board half-size from edge
        assert abs(by_label["TL"][0] - 200.0) < 0.01
        assert abs(by_label["TL"][1] - 150.0) < 0.01
        # BR at (1720, 930)
        assert abs(by_label["BR"][0] - 1720.0) < 0.01
        assert abs(by_label["BR"][1] - 930.0) < 0.01
        # C at frame center
        assert abs(by_label["C"][0] - 960.0) < 0.01
        assert abs(by_label["C"][1] - 540.0) < 0.01

    def test_adaptive_with_padding_frac(self):
        """padding_frac uses fraction of remaining screen space."""
        # 1920x1080, board half=200x150
        # remain_x = 960-200=760, remain_y = 540-150=390
        # padding_frac=0.25 → inset_x = 200+190=390, inset_y = 150+97.5=247.5
        pois = compute_poi_positions(
            1920, 1080, board_half_w=200, board_half_h=150, padding_frac=0.25)
        by_label = {t[2]: (t[0], t[1]) for t in pois}
        assert abs(by_label["TL"][0] - 390.0) < 0.01
        assert abs(by_label["TL"][1] - 247.5) < 0.01
        # BR symmetric
        assert abs(by_label["BR"][0] - 1530.0) < 0.01
        assert abs(by_label["BR"][1] - 832.5) < 0.01

    def test_adaptive_large_board_clamps(self):
        """Very large board still produces valid (non-inverted) ranges."""
        pois = compute_poi_positions(
            640, 480, board_half_w=300, board_half_h=220, padding_frac=0.25)
        by_label = {t[2]: (t[0], t[1]) for t in pois}
        for _, (x, y) in by_label.items():
            assert x >= 0
            assert y >= 0

    def test_adaptive_ignores_zero_board(self):
        """Falls back to margin when board size is 0."""
        pois_fixed = compute_poi_positions(1920, 1080, margin=0.1)
        pois_zero = compute_poi_positions(
            1920, 1080, margin=0.1, board_half_w=0, board_half_h=0)
        for a, b in zip(pois_fixed, pois_zero):
            assert abs(a[0] - b[0]) < 0.01
            assert abs(a[1] - b[1]) < 0.01


class TestMeasureBoardSize:
    def test_measures_corners_bbox(self):
        # Fake 4 corners at known positions
        corners = np.array([
            [[100, 200]], [[500, 200]], [[100, 400]], [[500, 400]]
        ], dtype=np.float32)
        half_w, half_h = measure_board_size(corners)
        assert abs(half_w - 200.0) < 0.01  # (500-100)/2
        assert abs(half_h - 100.0) < 0.01  # (400-200)/2

    def test_single_point(self):
        corners = np.array([[[300, 400]]], dtype=np.float32)
        half_w, half_h = measure_board_size(corners)
        assert half_w == 0.0
        assert half_h == 0.0


# ---------------------------------------------------------------------------
# TestSolveAngleDelta
# ---------------------------------------------------------------------------

class TestSolveAngleDelta:
    def test_diagonal_jacobian(self):
        J = np.array([[200.0, 0.0], [0.0, 150.0]])
        error = np.array([100.0, -75.0])
        delta = solve_angle_delta(J, error)
        np.testing.assert_allclose(delta, [0.5, -0.5], atol=0.01)

    def test_coupled_jacobian(self):
        J = np.array([[200.0, 50.0], [10.0, 150.0]])
        error = np.array([100.0, 75.0])
        delta = solve_angle_delta(J, error)
        # Verify: J @ delta ≈ error
        np.testing.assert_allclose(J @ delta, error, atol=0.01)

    def test_identity_jacobian(self):
        J = np.eye(2)
        error = np.array([3.0, -4.0])
        delta = solve_angle_delta(J, error)
        np.testing.assert_allclose(delta, error, atol=0.01)


# ---------------------------------------------------------------------------
# TestFlushAndDetect
# ---------------------------------------------------------------------------

class TestFlushAndDetect:
    def test_returns_detection_from_fresh_frame(self):
        fp = _make_push_fp()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        detect_fn = MagicMock(return_value=(320.0, 240.0))

        # Feed enough frames: flush_count + detection frames
        frames = [frame] * 20
        _feed_frames(fp, frames, interval=0.005)

        result = flush_and_detect(fp, detect_fn, settle_time=0.01,
                                  flush_count=5, flush_interval=0.01)
        assert result is not None
        got_frame, (cx, cy) = result
        assert cx == 320.0
        assert cy == 240.0
        fp.stop()

    def test_returns_none_when_detection_fails(self):
        fp = _make_push_fp()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        detect_fn = MagicMock(return_value=None)

        frames = [frame] * 20
        _feed_frames(fp, frames, interval=0.005)

        result = flush_and_detect(fp, detect_fn, settle_time=0.01,
                                  flush_count=5, flush_interval=0.01)
        assert result is None
        fp.stop()

    def test_retries_detection_on_transient_failure(self):
        fp = _make_push_fp()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        detect_fn = MagicMock(side_effect=[None, None, (100.0, 200.0)])

        frames = [frame] * 25
        _feed_frames(fp, frames, interval=0.005)

        result = flush_and_detect(fp, detect_fn, settle_time=0.01,
                                  flush_count=5, flush_interval=0.01)
        assert result is not None
        _, (cx, cy) = result
        assert cx == 100.0
        assert cy == 200.0
        fp.stop()


# ---------------------------------------------------------------------------
# TestProbeAxis
# ---------------------------------------------------------------------------

class TestProbeAxis:
    def test_normal_positive_probe(self):
        gimbal = MockGimbal()
        fp = _make_push_fp()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        # Simulate: object at (320, 240) at base, moves to (520, 245) when yaw+1°
        call_count = [0]
        def detect_fn(f):
            call_count[0] += 1
            return (520.0, 245.0)

        # Feed frames continuously
        frames = [frame] * 50
        t = _feed_frames(fp, frames, interval=0.005)

        cfg = NavigatorConfig(probe_step_deg=1.0, settle_time=0.01,
                              flush_count=3, flush_interval=0.005)
        result = probe_axis(
            gimbal, fp, detect_fn,
            base_yaw=0.0, base_pitch=0.0,
            base_cx=320.0, base_cy=240.0,
            axis="yaw", probe_deg=1.0, config=cfg,
        )

        assert result is not None
        # (520-320)/1 = 200, (245-240)/1 = 5
        np.testing.assert_allclose(result, [200.0, 5.0], atol=0.01)

        # Verify gimbal was restored to base position
        assert gimbal.calls[-1].yaw == 0.0
        assert gimbal.calls[-1].pitch == 0.0
        fp.stop()

    def test_negative_fallback_when_positive_fails(self):
        gimbal = MockGimbal()
        fp = _make_push_fp()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        call_count = [0]
        def detect_fn(f):
            call_count[0] += 1
            # First call (positive probe): fail
            # Second call (negative probe): succeed
            if call_count[0] <= 3:
                return None
            return (220.0, 240.0)

        frames = [frame] * 80
        _feed_frames(fp, frames, interval=0.005)

        cfg = NavigatorConfig(probe_step_deg=1.0, settle_time=0.01,
                              flush_count=3, flush_interval=0.005)
        result = probe_axis(
            gimbal, fp, detect_fn,
            base_yaw=0.0, base_pitch=0.0,
            base_cx=320.0, base_cy=240.0,
            axis="yaw", probe_deg=1.0, config=cfg,
        )

        assert result is not None
        # (220-320)/(-1) = 100, (240-240)/(-1) = 0
        np.testing.assert_allclose(result, [100.0, 0.0], atol=0.01)

        # Verify gimbal restored
        assert gimbal.calls[-1].yaw == 0.0
        assert gimbal.calls[-1].pitch == 0.0
        fp.stop()

    def test_restores_gimbal_on_success(self):
        gimbal = MockGimbal()
        fp = _make_push_fp()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        detect_fn = MagicMock(return_value=(500.0, 300.0))

        frames = [frame] * 50
        _feed_frames(fp, frames, interval=0.005)

        cfg = NavigatorConfig(probe_step_deg=1.0, settle_time=0.01,
                              flush_count=3, flush_interval=0.005)
        probe_axis(
            gimbal, fp, detect_fn,
            base_yaw=5.0, base_pitch=-3.0,
            base_cx=320.0, base_cy=240.0,
            axis="pitch", probe_deg=1.0, config=cfg,
        )

        # Last call should restore to base
        last = gimbal.calls[-1]
        assert last.yaw == 5.0
        assert last.pitch == -3.0
        fp.stop()


# ---------------------------------------------------------------------------
# TestMeasureJacobian
# ---------------------------------------------------------------------------

class TestMeasureJacobian:
    def test_combines_yaw_and_pitch_columns(self):
        gimbal = MockGimbal()
        fp = _make_push_fp()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        # Simulate different responses for yaw vs pitch probes
        probe_count = [0]
        def detect_fn(f):
            probe_count[0] += 1
            att = gimbal._att
            # Yaw probe: +1° yaw → (520, 250)
            if abs(att.yaw) > 0.5 and abs(att.pitch) < 0.5:
                return (520.0, 250.0)
            # Pitch probe: +1° pitch → (325, 390)
            if abs(att.pitch) > 0.5:
                return (325.0, 390.0)
            return (320.0, 240.0)

        frames = [frame] * 100
        _feed_frames(fp, frames, interval=0.003)

        cfg = NavigatorConfig(probe_step_deg=1.0, settle_time=0.01,
                              flush_count=3, flush_interval=0.005)
        J = measure_jacobian(
            gimbal, fp, detect_fn,
            base_yaw=0.0, base_pitch=0.0,
            base_cx=320.0, base_cy=240.0, config=cfg,
        )

        assert J is not None
        assert J.shape == (2, 2)
        # Yaw column: (520-320)/1=200, (250-240)/1=10
        assert abs(J[0, 0] - 200.0) < 1.0
        assert abs(J[1, 0] - 10.0) < 1.0
        # Pitch column: (325-320)/1=5, (390-240)/1=150
        assert abs(J[0, 1] - 5.0) < 1.0
        assert abs(J[1, 1] - 150.0) < 1.0
        fp.stop()

    def test_rejects_singular_jacobian(self):
        gimbal = MockGimbal()
        fp = _make_push_fp()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        # Both probes return same direction → singular
        def detect_fn(f):
            att = gimbal._att
            shift = abs(att.yaw) + abs(att.pitch)
            return (320.0 + shift * 100, 240.0)

        frames = [frame] * 100
        _feed_frames(fp, frames, interval=0.003)

        cfg = NavigatorConfig(probe_step_deg=1.0, settle_time=0.01,
                              flush_count=3, flush_interval=0.005)
        J = measure_jacobian(
            gimbal, fp, detect_fn,
            base_yaw=0.0, base_pitch=0.0,
            base_cx=320.0, base_cy=240.0, config=cfg,
        )

        assert J is None
        fp.stop()

    def test_returns_none_when_probe_fails(self):
        gimbal = MockGimbal()
        fp = _make_push_fp()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        # Detection always fails → probes fail
        detect_fn = MagicMock(return_value=None)

        frames = [frame] * 100
        _feed_frames(fp, frames, interval=0.003)

        cfg = NavigatorConfig(probe_step_deg=1.0, settle_time=0.01,
                              flush_count=3, flush_interval=0.005)
        J = measure_jacobian(
            gimbal, fp, detect_fn,
            base_yaw=0.0, base_pitch=0.0,
            base_cx=320.0, base_cy=240.0, config=cfg,
        )

        assert J is None
        fp.stop()


# ---------------------------------------------------------------------------
# TestNavigateTo
# ---------------------------------------------------------------------------

class TestNavigateTo:
    def _make_nav(self, gimbal, fp, detect_fn, **cfg_kwargs):
        defaults = dict(
            settle_time=0.01, flush_count=3, flush_interval=0.005,
        )
        defaults.update(cfg_kwargs)
        cfg = NavigatorConfig(**defaults)
        return CalibrationNavigator(
            gimbal, fp, detect_fn, config=cfg, logger=_NullLogger(),
        )

    def test_single_step_convergence(self):
        """Object is already near POI -- converges in 1 iteration."""
        gimbal = MockGimbal()
        fp = _make_push_fp()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        detect_fn = MagicMock(return_value=(325.0, 242.0))
        frames = [frame] * 30
        _feed_frames(fp, frames, interval=0.005)

        nav = self._make_nav(gimbal, fp, detect_fn, tolerance_px=30.0)
        J = np.array([[200.0, 0.0], [0.0, 150.0]])

        result = nav.navigate_to((320.0, 240.0), yaw=0.0, pitch=0.0, jacobian=J)

        assert result.reached is True
        assert result.iterations == 1
        assert result.pixel_error < 30.0
        fp.stop()

    def test_multi_step_convergence(self):
        """Uses simulated Jacobian relationship for realistic multi-step."""
        gimbal = MockGimbal()
        fp = _make_push_fp()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        detect_fn = _make_sim_detect(
            base_cx=320.0, base_cy=240.0,
            px_per_deg_yaw_x=200.0, px_per_deg_yaw_y=0.0,
            px_per_deg_pitch_x=0.0, px_per_deg_pitch_y=150.0,
            gimbal=gimbal,
        )

        frames = [frame] * 200
        _feed_frames(fp, frames, interval=0.003)

        nav = self._make_nav(gimbal, fp, detect_fn,
                             tolerance_px=5.0, max_iterations=10)
        J = np.array([[200.0, 0.0], [0.0, 150.0]])

        result = nav.navigate_to((520.0, 390.0), yaw=0.0, pitch=0.0, jacobian=J)

        assert result.reached is True
        assert result.pixel_error <= 5.0
        assert result.final_xy is not None
        fp.stop()

    def test_max_iterations_returns_not_reached(self):
        """When Jacobian is inaccurate, max_iterations is hit."""
        gimbal = MockGimbal()
        fp = _make_push_fp()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        detect_fn = MagicMock(return_value=(100.0, 100.0))

        frames = [frame] * 200
        _feed_frames(fp, frames, interval=0.003)

        nav = self._make_nav(gimbal, fp, detect_fn,
                             tolerance_px=5.0, max_iterations=3)
        J = np.array([[200.0, 0.0], [0.0, 150.0]])

        result = nav.navigate_to((500.0, 400.0), yaw=0.0, pitch=0.0, jacobian=J)

        assert result.reached is False
        assert result.iterations == 3
        fp.stop()

    def test_no_jacobian_returns_immediately(self):
        """Without a Jacobian, navigate_to returns failure immediately."""
        gimbal = MockGimbal()
        fp = _make_push_fp()
        detect_fn = MagicMock()

        nav = self._make_nav(gimbal, fp, detect_fn)
        result = nav.navigate_to((500.0, 400.0), yaw=0.0, pitch=0.0)

        assert result.reached is False
        assert result.iterations == 0
        assert result.pixel_error == float("inf")
        fp.stop()

    def test_detection_loss_triggers_retreat(self):
        """When detection is lost, navigator retreats halfway."""
        gimbal = MockGimbal()
        fp = _make_push_fp()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        call_count = [0]
        def detect_fn(f):
            call_count[0] += 1
            if call_count[0] == 1:
                return (200.0, 200.0)
            if call_count[0] <= 4:
                return None
            return (498.0, 398.0)

        frames = [frame] * 200
        _feed_frames(fp, frames, interval=0.003)

        nav = self._make_nav(gimbal, fp, detect_fn,
                             tolerance_px=10.0, max_iterations=10)
        J = np.array([[200.0, 0.0], [0.0, 150.0]])

        result = nav.navigate_to((500.0, 400.0), yaw=0.0, pitch=0.0, jacobian=J)

        assert result.reached is True
        fp.stop()

    def test_step_clamping(self):
        """Large pixel errors are clamped to step_clamp_deg per iteration."""
        gimbal = MockGimbal()
        fp = _make_push_fp()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        detect_fn = _make_sim_detect(
            base_cx=100.0, base_cy=100.0,
            px_per_deg_yaw_x=200.0, px_per_deg_yaw_y=0.0,
            px_per_deg_pitch_x=0.0, px_per_deg_pitch_y=150.0,
            gimbal=gimbal,
        )

        frames = [frame] * 200
        _feed_frames(fp, frames, interval=0.003)

        nav = self._make_nav(gimbal, fp, detect_fn,
                             tolerance_px=5.0, max_iterations=20,
                             step_clamp_deg=2.0)
        J = np.array([[200.0, 0.0], [0.0, 150.0]])

        result = nav.navigate_to((1100.0, 850.0), yaw=0.0, pitch=0.0, jacobian=J)

        first_move = gimbal.calls[0]
        assert abs(first_move.yaw) <= 2.01
        assert abs(first_move.pitch) <= 2.01

        assert result.reached is True
        fp.stop()

    def test_measure_jacobian_method(self):
        """CalibrationNavigator.measure_jacobian caches the result."""
        gimbal = MockGimbal()
        fp = _make_push_fp()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        # Simulate: base→(320,240), yaw+1°→(520,250), pitch+1°→(325,390)
        def detect_fn(f):
            att = gimbal._att
            if abs(att.yaw) > 0.5 and abs(att.pitch) < 0.5:
                return (520.0, 250.0)
            if abs(att.pitch) > 0.5:
                return (325.0, 390.0)
            return (320.0, 240.0)

        frames = [frame] * 150
        _feed_frames(fp, frames, interval=0.003)

        nav = self._make_nav(gimbal, fp, detect_fn)
        J = nav.measure_jacobian(yaw=0.0, pitch=0.0)

        assert J is not None
        assert J.shape == (2, 2)
        # Cached — should be accessible for navigate_to
        assert nav._jacobian is not None
        np.testing.assert_allclose(nav._jacobian, J)
        fp.stop()
