#!/usr/bin/env python
"""
Visual test for CalibrationNavigator with SIYI gimbal hardware.

Connects to a SIYI gimbal, opens the RTSP stream, detects a checkerboard,
measures the Jacobian, then steers the board to each of the 9 anchor
positions while showing live video with overlays.

Usage:
    python tools/cam/calibration/run_calibration_navigator.py
    python tools/cam/calibration/run_calibration_navigator.py --ip 192.168.144.25
    python tools/cam/calibration/run_calibration_navigator.py --camera 0 --no-gimbal
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

# Ensure project paths are importable
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

from cam.calibration.board_detector import BoardDetector
from cam.calibration.calibration_navigator import (
    CalibrationNavigator,
    NavigatorConfig,
    compute_poi_positions,
    measure_board_size,
)
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.frame_provider import FrameProvider
from navpy.modules.vision.peripheral.gimbal_abc import GimbalAbc, GimbalData

WINDOW_NAME = "Calibration Navigator"

# Arrow key codes from cv2.waitKeyEx on Windows
_KEY_LEFT = 0x250000
_KEY_RIGHT = 0x270000
_KEY_UP = 0x260000
_KEY_DOWN = 0x280000
_PAN_STEP_DEG = 1.0


# ---------------------------------------------------------------------------
# Logger
# ---------------------------------------------------------------------------

class _Logger:
    def info(self, msg, *a):
        print(f"[INFO]  {msg % a}" if a else f"[INFO]  {msg}")

    def warning(self, msg, *a):
        print(f"[WARN]  {msg % a}" if a else f"[WARN]  {msg}")

    def error(self, msg, *a):
        print(f"[ERR]   {msg % a}" if a else f"[ERR]   {msg}")

    def debug(self, msg, *a):
        pass


logger = _Logger()


# ---------------------------------------------------------------------------
# Shared overlay state between main (display) thread and worker thread
# ---------------------------------------------------------------------------

class OverlayState:
    """Thread-safe overlay state shared between display and worker."""

    def __init__(self):
        self._lock = threading.Lock()
        self.pois: list = []
        self.active_idx: int = -1
        self.poi_xy: tuple[float, float] | None = None
        self.status_lines: list[str] = []
        self.badge: tuple[str, bool] | None = None
        self.center: tuple[float, float] | None = None
        self.phase: str = "preview"  # preview, jacobian, navigate, done
        self.quit_flag = False
        self.space_flag = False
        self.manual_yaw: float = 0.0
        self.manual_pitch: float = 0.0
        self.zoom_level: float = 1.0

    def set_pois(self, pois, active_idx=-1):
        with self._lock:
            self.pois = list(pois)
            self.active_idx = active_idx

    def set_poi_xy(self, xy):
        with self._lock:
            self.poi_xy = xy

    def set_status(self, *lines):
        with self._lock:
            self.status_lines = list(lines)

    def set_badge(self, label, reached=False):
        with self._lock:
            self.badge = (label, reached) if label else None

    def set_center(self, center):
        with self._lock:
            self.center = center

    def get_center(self):
        with self._lock:
            return self.center

    def set_phase(self, phase):
        with self._lock:
            self.phase = phase

    def get_phase(self):
        with self._lock:
            return self.phase

    def signal_space(self):
        with self._lock:
            self.space_flag = True

    def signal_quit(self):
        with self._lock:
            self.quit_flag = True

    def consume_space(self):
        with self._lock:
            v = self.space_flag
            self.space_flag = False
            return v

    def is_quit(self):
        with self._lock:
            return self.quit_flag

    def snapshot(self):
        with self._lock:
            return (
                list(self.pois),
                self.active_idx,
                self.poi_xy,
                list(self.status_lines),
                self.badge,
                self.center,
            )


# ---------------------------------------------------------------------------
# Background detector
# ---------------------------------------------------------------------------

class BackgroundDetector:
    """Runs BoardDetector in a background thread."""

    def __init__(self, detector: BoardDetector, state: OverlayState):
        self._detector = detector
        self._state = state
        self._lock = threading.Lock()
        self._pending: np.ndarray | None = None
        self._has_work = threading.Event()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def submit(self, frame: np.ndarray):
        with self._lock:
            self._pending = frame
        self._has_work.set()

    def stop(self):
        self._stop.set()
        self._has_work.set()
        self._thread.join(timeout=2.0)

    def _loop(self):
        while not self._stop.is_set():
            self._has_work.wait()
            self._has_work.clear()
            if self._stop.is_set():
                break
            with self._lock:
                frame = self._pending
                self._pending = None
            if frame is None:
                continue
            result = self._detector.detect(frame)
            self._state.set_center(result)


# ---------------------------------------------------------------------------
# Dummy gimbal for --no-gimbal mode
# ---------------------------------------------------------------------------

class DummyGimbal(GimbalAbc):
    def __init__(self):
        self._att = Attitude(pitch=0, yaw=0, roll=0)

    def set_att(self, att: Attitude):
        self._att = att

    def get_data(self) -> GimbalData:
        return GimbalData(att=self._att)


# ---------------------------------------------------------------------------
# Drawing helpers
# ---------------------------------------------------------------------------

def draw_overlay(frame, pois, active_idx, poi_xy, status_lines, badge, center):
    disp = frame.copy()
    # POIs
    for i, (tx, ty, label) in enumerate(pois):
        color = (0, 255, 255) if i == active_idx else (80, 80, 80)
        thickness = 2 if i == active_idx else 1
        pt = (int(tx), int(ty))
        cv2.drawMarker(disp, pt, color, cv2.MARKER_DIAMOND, 20, thickness)
        cv2.putText(disp, label, (pt[0] + 12, pt[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    # Detection cross + line to POI
    if center is not None:
        cx, cy = int(center[0]), int(center[1])
        cv2.drawMarker(disp, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        if poi_xy is not None:
            tx, ty = int(poi_xy[0]), int(poi_xy[1])
            cv2.line(disp, (cx, cy), (tx, ty), (255, 0, 255), 1)
            error = np.hypot(poi_xy[0] - center[0], poi_xy[1] - center[1])
            mid_x, mid_y = (cx + tx) // 2, (cy + ty) // 2
            cv2.putText(disp, f"{error:.0f}px", (mid_x + 5, mid_y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
    # Status
    for i, line in enumerate(status_lines):
        cv2.putText(disp, line, (10, 25 + i * 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    # Badge
    if badge:
        label, reached = badge
        color = (0, 200, 0) if reached else (0, 0, 200)
        text = f"{label}: {'OK' if reached else 'FAIL'}"
        cv2.putText(disp, text, (disp.shape[1] - 150, disp.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return disp


# ---------------------------------------------------------------------------
# Connection helpers
# ---------------------------------------------------------------------------

def connect_gimbal(ip, port):
    """Connect to a SIYI gimbal, center it, and return the instance."""
    from navpy.modules.vision.peripheral.gimbal_siyi import GimbalSiyi

    data = GimbalData(
        att=Attitude(0, 0, 0),
        roll_stabilize=True,
        pitch_stabilize=True,
    )
    gimbal = GimbalSiyi(data, ip, port, logger)
    gimbal.start()
    if not gimbal.is_connected():
        raise RuntimeError(f"Failed to connect to SIYI gimbal at {ip}:{port}")

    # Switch to Lock mode for absolute yaw+pitch control
    sdk = getattr(gimbal, "_sdk", None)
    if sdk is not None:
        sdk.requestLockMode()
        logger.info("Gimbal set to Lock mode")
        sdk.requestCenterGimbal()
        logger.info("Gimbal centered")
    gimbal.set_att(Attitude(pitch=0, yaw=0, roll=0))

    logger.info("Gimbal connected at %s:%d", ip, port)
    time.sleep(1.0)
    return gimbal


def open_stream(source):
    """Open a video source via FrameProvider and wait for first frame."""
    fp = FrameProvider(source, logger)
    fp.start()
    for _ in range(50):
        frame, w, h = fp.get_frame()
        if frame is not None:
            logger.info("Stream opened: %dx%d", w, h)
            return fp
        time.sleep(0.1)
    raise RuntimeError(f"No frames from source: {source}")


def parse_source(raw):
    try:
        return int(raw)
    except ValueError:
        return raw


# ---------------------------------------------------------------------------
# Worker thread — runs Jacobian + navigation in background
# ---------------------------------------------------------------------------

def worker_thread(nav, state, pois, no_gimbal, detector, fp, img_w, img_h, margin, positions):
    """Runs calibration phases in a background thread."""
    try:
        # Wait for SPACE from preview
        while not state.is_quit():
            if state.consume_space() and state.get_center() is not None:
                break
            time.sleep(0.05)

        if state.is_quit():
            return

        if no_gimbal:
            state.set_status("No-gimbal mode -- press Q to exit")
            state.set_phase("done")
            return

        # Measure board size from current frame and recompute POIs
        half_w, half_h = 0.0, 0.0
        frame, _, _ = fp.get_frame()
        if frame is not None:
            corners = detector.detect_corners(frame)
            if corners is not None:
                half_w, half_h = measure_board_size(corners)
                pois = compute_poi_positions(
                    img_w, img_h,
                    board_half_w=half_w, board_half_h=half_h,
                    positions=positions,
                )
                state.set_pois(pois)
                nav.set_frame_info(img_w, img_h, half_w, half_h)
                logger.info("Board size: %.0fx%.0f px, adaptive POIs computed",
                            half_w * 2, half_h * 2)
            else:
                logger.warning("Board not detected for size measurement, using fixed margin")

        # Phase 2: Jacobian — start from current pan position
        state.set_phase("jacobian")
        yaw, pitch = state.manual_yaw, state.manual_pitch
        state.set_status("Measuring Jacobian...", f"yaw={yaw:.1f} pitch={pitch:.1f}")
        logger.info("Phase: Measuring Jacobian at yaw=%.1f pitch=%.1f", yaw, pitch)

        J = nav.measure_jacobian(
            yaw, pitch,
            img_w=img_w, img_h=img_h,
            board_half_w=half_w, board_half_h=half_h,
        )
        if J is None:
            logger.error("Jacobian measurement failed")
            state.set_status("Jacobian FAILED -- press Q to exit")
            state.set_phase("done")
            return

        logger.info("Jacobian:\n  yaw  col: [%.1f, %.1f] px/deg\n  pitch col: [%.1f, %.1f] px/deg",
                     J[0, 0], J[1, 0], J[0, 1], J[1, 1])
        state.set_status(
            f"Jacobian: yaw=[{J[0,0]:.1f}, {J[1,0]:.1f}] pitch=[{J[0,1]:.1f}, {J[1,1]:.1f}]")

        # Phase 3: Navigate
        state.set_phase("navigate")
        cur_yaw, cur_pitch = yaw, pitch
        results = []

        for i, (tx, ty, label) in enumerate(pois):
            if state.is_quit():
                break

            logger.info("--- POI %d/%d: %s (%.0f, %.0f) ---",
                        i + 1, len(pois), label, tx, ty)

            poi_xy = (tx, ty)
            state.set_pois(pois, active_idx=i)
            state.set_poi_xy(poi_xy)
            state.set_status(f"Navigating to {label}...",
                             f"yaw={cur_yaw:.2f} pitch={cur_pitch:.2f}")
            state.set_badge(None)

            result = nav.navigate_to(poi_xy, cur_yaw, cur_pitch, jacobian=J)

            reached_str = "OK" if result.reached else "FAIL"
            logger.info("  %s: %s (error=%.1f px, %d iters, yaw=%.2f pitch=%.2f)",
                         label, reached_str, result.pixel_error, result.iterations,
                         result.final_yaw, result.final_pitch)

            status = "REACHED" if result.reached else "MISSED"
            state.set_status(
                f"{label}: {status} err={result.pixel_error:.0f}px iters={result.iterations}",
                f"yaw={result.final_yaw:.2f} pitch={result.final_pitch:.2f}",
                "SPACE = next, Q = abort",
            )
            state.set_badge(label, result.reached)

            results.append((label, result))
            cur_yaw = result.final_yaw
            cur_pitch = result.final_pitch

            # Wait for SPACE or Q
            while not state.is_quit():
                if state.consume_space():
                    break
                time.sleep(0.05)

        # Summary
        ok = sum(1 for _, r in results if r.reached)
        total = len(results)
        logger.info("=== Done: %d/%d POIs reached ===", ok, total)
        for label, r in results:
            s = "OK" if r.reached else "FAIL"
            logger.info("  %s: %s  error=%.1f px", label, s, r.pixel_error)

        state.set_status(f"Done: {ok}/{total} reached -- press Q to exit")
        state.set_poi_xy(None)
        state.set_badge(None)
        state.set_phase("done")

    except Exception as exc:
        logger.error("Worker error: %s", exc)
        state.set_status(f"Error: {exc}", "Press Q to exit")
        state.set_phase("done")


# ---------------------------------------------------------------------------
# POI recomputation after zoom
# ---------------------------------------------------------------------------

def _recompute_pois(detector, fp, state, img_w, img_h, args):
    """Wait for zoom to settle, remeasure board, recompute POIs."""
    time.sleep(0.8)  # let zoom settle + RTSP catch up
    frame, _, _ = fp.get_frame()
    if frame is None:
        return
    corners = detector.detect_corners(frame)
    if corners is None:
        logger.warning("Board not detected after zoom, POIs unchanged")
        return
    half_w, half_h = measure_board_size(corners)
    pois = compute_poi_positions(
        img_w, img_h,
        board_half_w=half_w, board_half_h=half_h,
        positions=args.positions,
    )
    state.set_pois(pois)
    logger.info("Zoom %.0fx: board %.0fx%.0f px, POIs recomputed",
                state.zoom_level, half_w * 2, half_h * 2)


# ---------------------------------------------------------------------------
# Main — display loop on main thread
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(
        description="Visual test for CalibrationNavigator with SIYI gimbal",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--ip", default="192.168.144.25", help="SIYI gimbal IP")
    p.add_argument("--port", type=int, default=37260, help="SIYI gimbal UDP port")
    p.add_argument("--camera", default=None,
                   help="Video source (default: rtsp://{ip}:8554/main.264)")
    p.add_argument("--no-gimbal", action="store_true",
                   help="Preview-only mode without gimbal hardware")
    p.add_argument("--cols", type=int, default=7, help="Checkerboard inner cols")
    p.add_argument("--rows", type=int, default=5, help="Checkerboard inner rows")
    p.add_argument("--margin", type=float, default=0.15,
                   help="POI margin from frame edges (0..0.5)")
    p.add_argument("--tolerance", type=float, default=50.0,
                   help="Convergence tolerance in pixels")
    p.add_argument("--probe-step", type=float, default=3.0,
                   help="Jacobian probe step in degrees")
    p.add_argument("--settle", type=float, default=0.8,
                   help="Settle time after gimbal move (seconds)")
    p.add_argument("--max-iter", type=int, default=10,
                   help="Max steering iterations per POI")
    p.add_argument("--positions", nargs="*", default=None,
                   help="Subset of positions: TL TC TR ML C MR BL BC BR")
    return p.parse_args()


def main():
    args = parse_args()

    if args.camera is not None:
        source = parse_source(args.camera)
    else:
        source = f"rtsp://{args.ip}:8554/main.264"

    gimbal = None
    bg_det = None
    fp = None
    try:
        if args.no_gimbal:
            gimbal = DummyGimbal()
            logger.info("No-gimbal mode: detection preview only")
        else:
            gimbal = connect_gimbal(args.ip, args.port)

        fp = open_stream(source)

        detector = BoardDetector(cols=args.cols, rows=args.rows)

        _, w, h = fp.get_frame()
        if w == 0 or h == 0:
            raise RuntimeError("Cannot determine frame dimensions")

        pois = compute_poi_positions(w, h, margin=args.margin, positions=args.positions)
        logger.info("POIs (%d): %s", len(pois), ", ".join(t[2] for t in pois))

        config = NavigatorConfig(
            probe_step_deg=args.probe_step,
            tolerance_px=args.tolerance,
            max_iterations=args.max_iter,
            settle_time=args.settle,
        )
        nav = CalibrationNavigator(gimbal, fp, detector.detect, config=config, logger=logger)

        # Shared state
        state = OverlayState()
        state.set_pois(pois)
        state.set_status("Searching for checkerboard... Q to quit")

        # Background detector
        bg_det = BackgroundDetector(detector, state)

        # Start worker thread (Jacobian + navigation)
        wt = threading.Thread(
            target=worker_thread,
            args=(nav, state, pois, args.no_gimbal,
                  detector, fp, w, h, args.margin, args.positions),
            daemon=True,
        )
        wt.start()

        # Get SDK for zoom control
        sdk = getattr(gimbal, "_sdk", None) if gimbal else None

        # Main thread: display loop
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        logger.info("Phase: Preview -- point camera at checkerboard, press SPACE to start")
        logger.info("  Arrows=pan  +/-=zoom  SPACE=start  Q=quit")

        while True:
            frame, _, _ = fp.get_frame()
            if frame is not None:
                bg_det.submit(frame)
                overlay = state.snapshot()
                disp = draw_overlay(frame, *overlay)
                cv2.imshow(WINDOW_NAME, disp)

            key = cv2.waitKeyEx(30)
            key_ascii = key & 0xFF
            if key_ascii in (ord("q"), 27):
                state.signal_quit()
                break
            if key_ascii == ord(" "):
                state.signal_space()

            # Arrow keys: pan gimbal
            if key in (_KEY_LEFT, _KEY_RIGHT, _KEY_UP, _KEY_DOWN):
                if key == _KEY_LEFT:
                    state.manual_yaw -= _PAN_STEP_DEG
                elif key == _KEY_RIGHT:
                    state.manual_yaw += _PAN_STEP_DEG
                elif key == _KEY_UP:
                    state.manual_pitch += _PAN_STEP_DEG
                elif key == _KEY_DOWN:
                    state.manual_pitch -= _PAN_STEP_DEG
                gimbal.set_att(Attitude(
                    pitch=state.manual_pitch, yaw=state.manual_yaw, roll=0))
                logger.info("Pan: yaw=%.1f pitch=%.1f",
                            state.manual_yaw, state.manual_pitch)

            # +/- keys: zoom
            if key_ascii in (ord("+"), ord("=")):
                state.zoom_level = min(30.0, state.zoom_level + 1.0)
                if sdk:
                    sdk.requestAbsoluteZoom(state.zoom_level)
                logger.info("Zoom: %.0fx", state.zoom_level)
                _recompute_pois(detector, fp, state, w, h, args)
            if key_ascii == ord("-"):
                state.zoom_level = max(1.0, state.zoom_level - 1.0)
                if sdk:
                    sdk.requestAbsoluteZoom(state.zoom_level)
                logger.info("Zoom: %.0fx", state.zoom_level)
                _recompute_pois(detector, fp, state, w, h, args)

            # c key: center gimbal
            if key_ascii == ord("c"):
                state.manual_yaw = 0.0
                state.manual_pitch = 0.0
                if sdk:
                    sdk.requestCenterGimbal()
                gimbal.set_att(Attitude(pitch=0, yaw=0, roll=0))
                logger.info("Gimbal centered")

            # Update status during preview
            phase = state.get_phase()
            if phase == "preview":
                zoom_str = f"  Zoom: {state.zoom_level:.0f}x" if state.zoom_level > 1 else ""
                pan_str = f"  Pan: ({state.manual_yaw:.1f}, {state.manual_pitch:.1f})"
                hint = "Arrows=pan +/-=zoom C=center SPACE=start Q=quit"
                if state.get_center() is not None:
                    state.set_status(
                        f"Checkerboard detected{zoom_str}{pan_str}",
                        hint)
                else:
                    state.set_status(
                        f"Searching for checkerboard...{zoom_str}{pan_str}",
                        hint)

            if phase == "done" and not wt.is_alive():
                # Show final state, wait for Q
                while True:
                    frame, _, _ = fp.get_frame()
                    if frame is not None:
                        overlay = state.snapshot()
                        disp = draw_overlay(frame, *overlay)
                        cv2.imshow(WINDOW_NAME, disp)
                    key = cv2.waitKey(30) & 0xFF
                    if key in (ord("q"), 27) or key != 255:
                        break
                break

        wt.join(timeout=3.0)

    except KeyboardInterrupt:
        logger.info("Interrupted")
    finally:
        if bg_det is not None:
            bg_det.stop()
        cv2.destroyAllWindows()
        if fp is not None:
            fp.stop()
        if gimbal is not None and hasattr(gimbal, "stop"):
            gimbal.stop()


if __name__ == "__main__":
    main()
