"""Frame overlay, UI, FPS, and diagnostic-counter ownership."""

from __future__ import annotations

import time
from dataclasses import dataclass

import cv2
import numpy as np

from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.frame_provider import FrameProvider
from navpy.modules.vision.multi_object_tracker import TrackedObject
from navpy.modules.vision.real_detector_ports import DetectorOverlayRenderer
from navpy.modules.vision.real_detector_state import (
    DetectorRunState,
    FreshnessPolicy,
    OverlayStore,
    RuntimeMetrics,
)
from navpy.modules.vision.track_identity import TrackIdentityResolver


@dataclass(frozen=True)
class DetectorDebugConfig:
    show: bool = False
    window_name: str = "navpy-detector"
    allow_esc_stop: bool = True


class DetectorDiagnostics:
    """Own overlay reads, debug UI, FPS, and diagnostic counters."""

    def __init__(
        self,
        run_state: DetectorRunState,
        frame_provider: FrameProvider,
        overlays: OverlayStore,
        freshness: FreshnessPolicy,
        metrics: RuntimeMetrics,
        identity: TrackIdentityResolver,
        overlay_renderer: DetectorOverlayRenderer,
        use_target_lock: bool,
        config: DetectorDebugConfig,
        logger: ILogger,
    ) -> None:
        self._run_state = run_state
        self._frame_provider = frame_provider
        self._overlays = overlays
        self._freshness = freshness
        self._metrics = metrics
        self._identity = identity
        self._overlay_renderer = overlay_renderer
        self._use_target_lock = bool(use_target_lock)
        self._config = config
        self._logger = logger
        self._ui_initialized = False

    def get_overlay_tracks(self) -> list[TrackedObject]:
        tracks, _ = self._overlays.snapshot()
        now_s = time.time()
        return [
            track
            for track in tracks
            if track.is_confirmed and self._freshness.is_fresh(track, now_s)
        ]

    @property
    def tracking_fps(self) -> float:
        return self._metrics.tracking_fps

    def get_raw_frame(self) -> np.ndarray | None:
        frame, _, _ = self._frame_provider.get_frame()
        return frame

    def get_debug_frame(self) -> np.ndarray | None:
        frame, _, _ = self._frame_provider.get_frame()
        if frame is None:
            return None
        tracks, locked = self._overlays.snapshot()
        now_s = time.time()
        tracks = [
            track
            for track in tracks
            if self._freshness.is_fresh(track, now_s)
        ]
        if locked is not None and not self._freshness.is_fresh(locked, now_s):
            locked = None
        return self._overlay_renderer.render(
            frame,
            tracks,
            locked,
            fps_est=self._metrics.tracking_fps,
            use_lock=self._use_target_lock,
        )

    def ui_step(self) -> bool:
        if not self._config.show:
            return True
        debug_frame = self.get_debug_frame()
        if debug_frame is None:
            return True
        if not self._ui_initialized:
            try:
                cv2.namedWindow(self._config.window_name, cv2.WINDOW_NORMAL)
                self._ui_initialized = True
            except Exception as error:
                self._logger.error(f"cv2.namedWindow failed: {error}")
                return True
        try:
            cv2.imshow(self._config.window_name, debug_frame)
            key = cv2.waitKey(1) & 0xFF
            if self._config.allow_esc_stop and key == 27:
                self._run_state.request_stop()
                return False
        except Exception as error:
            self._logger.error(f"cv2.imshow/waitKey failed: {error}")
        return True

    def stats(self) -> dict[str, float]:
        result = self._metrics.snapshot()
        for key, value in self._identity.counters.items():
            result[f"id_{key}"] = value
        return result

    def close_ui(self) -> None:
        try:
            cv2.destroyWindow(self._config.window_name)
        except Exception:
            pass
        self._ui_initialized = False


__all__ = ["DetectorDebugConfig", "DetectorDiagnostics"]
