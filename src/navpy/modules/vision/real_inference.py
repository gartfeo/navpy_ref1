"""Primary and deep-search inference loops for the real detector."""

from __future__ import annotations

from collections.abc import Iterable

from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.deep_search import DeepSearchConfig, DeepSearchDetector
from navpy.modules.vision.frame_provider import FrameProvider
from navpy.modules.vision.real_detector_state import (
    DetectorRunState,
    FreshnessPolicy,
    InferenceGeneration,
    InferenceReservation,
    LoopTiming,
    OverlayStore,
    RuntimeMetrics,
    next_loop_deadline,
)
from navpy.modules.vision.real_frame_association import FrameAssociationBuilder
from navpy.modules.vision.poi_lock import PoiLock
from navpy.modules.vision.real_detector_ports import FrameDetector
from navpy.modules.vision.yolo_detector import Detection


class DeepSearchChannel:
    """Selection policy plus frame-keyed deep-search inbox access."""

    def __init__(
            self,
            enabled: bool,
            config: DeepSearchConfig | None,
            poi_lock: PoiLock,
            overlays: OverlayStore,
            freshness: FreshnessPolicy,
            generation: InferenceGeneration,
            timing: LoopTiming = LoopTiming(),
    ) -> None:
        self._enabled = bool(enabled)
        self._config = config
        self._poi_lock = poi_lock
        self._overlays = overlays
        self._freshness = freshness
        self._generation = generation
        self._timing = timing

    def reserve(self) -> InferenceReservation:
        return self._generation.reserve()

    def should_run(self) -> bool:
        if not self._enabled or self._config is None:
            return False
        locked_id = self._poi_lock.locked_id
        if locked_id is None:
            return False
        _, locked = self._overlays.snapshot()
        if locked is None or locked.id != locked_id:
            return True
        if not self._freshness.is_fresh(locked):
            return True
        return int(locked.missed) > 0

    def publish(
            self,
            reservation: InferenceReservation,
            detections: Iterable[Detection],
            *,
            frame_sequence: int | None,
    ) -> bool:
        return self._generation.publish_deep_search(
            reservation,
            detections,
            self._timing.monotonic(),
            frame_sequence,
        )

    def take_for_tracking(self, frame_sequence: int | None) -> list[Detection]:
        if not self.should_run():
            return []
        return self._generation.take_deep_search(
            expected_frame_sequence=frame_sequence,
            stale_seconds=self._config.stale_seconds,
            now_s=self._timing.monotonic(),
        )

    def clear(self) -> None:
        self._generation.clear_deep_search()


class DetectionLoop:
    """Run the frame detector and atomically publish detections with frame state."""

    def __init__(
            self,
            run_state: DetectorRunState,
            period_s: float,
            frame_provider: FrameProvider,
            association_builder: FrameAssociationBuilder,
            detector: FrameDetector,
            generation: InferenceGeneration,
            metrics: RuntimeMetrics,
            logger: ILogger,
            timing: LoopTiming = LoopTiming(),
    ) -> None:
        self._run_state = run_state
        self._period_s = float(period_s)
        self._frame_provider = frame_provider
        self._association_builder = association_builder
        self._detector = detector
        self._generation = generation
        self._metrics = metrics
        self._logger = logger
        self._timing = timing

    def run(self) -> None:
        next_at = self._timing.monotonic()
        while self._run_state.is_running:
            now_s = self._timing.monotonic()
            sleep_s = next_at - now_s
            if sleep_s > 0.0:
                self._timing.sleep(sleep_s)
                continue
            next_at = next_loop_deadline(next_at, self._period_s, now_s)
            reservation = self._generation.reserve()
            snapshot = self._frame_provider.get_frame_snapshot()
            if snapshot.frame is None:
                continue
            try:
                frame = snapshot.frame.copy()
                association = self._association_builder.capture(snapshot, frame)
                detections = self._detector.detect(frame)
                if self._generation.publish_detection(
                        reservation,
                        detections,
                        association,
                ):
                    self._metrics.bump("detect_batches")
            except Exception as error:
                self._logger.error(f"Frame detect error: {error}")


class DeepSearchLoop:
    """Run validation inference and publish to the one-shot channel."""

    def __init__(
            self,
            run_state: DetectorRunState,
            frame_provider: FrameProvider,
            detector: DeepSearchDetector | None,
            config: DeepSearchConfig | None,
            channel: DeepSearchChannel,
            metrics: RuntimeMetrics,
            logger: ILogger,
            timing: LoopTiming = LoopTiming(),
    ) -> None:
        self._run_state = run_state
        self._frame_provider = frame_provider
        self._detector = detector
        self._config = config
        self._channel = channel
        self._metrics = metrics
        self._logger = logger
        self._timing = timing

    def run(self) -> None:
        if self._detector is None or self._config is None:
            return
        next_at = self._timing.monotonic()
        while self._run_state.is_running:
            now_s = self._timing.monotonic()
            sleep_s = next_at - now_s
            if sleep_s > 0.0:
                self._timing.sleep(sleep_s)
                continue
            next_at = next_loop_deadline(next_at, self._config.period, now_s)
            reservation = self._channel.reserve()
            if not self._channel.should_run():
                self._channel.clear()
                continue
            snapshot = self._frame_provider.get_frame_snapshot()
            if snapshot.frame is None:
                continue
            try:
                detections = self._detector.detect(snapshot.frame.copy())
                published = self._channel.publish(
                    reservation,
                    detections,
                    frame_sequence=snapshot.sequence,
                )
                if published:
                    self._metrics.bump("deep_search_runs")
                if published and detections:
                    self._logger.info(
                        f"DeepSearch: validating selected POI with "
                        f"{len(detections)} candidates",
                        key=f"deep_search_candidates_{len(detections)}",
                    )
            except Exception as error:
                self._logger.error(f"DeepSearch detect error: {error}")


__all__ = ["DeepSearchChannel", "DeepSearchLoop", "DetectionLoop"]
