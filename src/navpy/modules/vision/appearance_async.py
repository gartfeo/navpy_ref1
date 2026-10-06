"""Asynchronous mailbox, publication, and lifecycle for appearance batches."""

from __future__ import annotations

from navpy.exception_groups import BaseExceptionGroup

import threading
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from navpy.logger.logger_api import ILogger
from navpy.modules.vision.appearance_worker import AppearanceWorker


class AppearanceBackend(Protocol):
    def embed(
        self,
        frame: np.ndarray,
        boxes_xyxy: Sequence[Sequence[float]],
    ) -> np.ndarray: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class AppearanceSnapshot:
    vectors: dict[int, np.ndarray]
    boxes: dict[int, tuple[float, ...]]
    timestamp: float


def _iou_xyxy(a: Sequence[float], b: Sequence[float]) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    intersection_width = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    intersection_height = max(0.0, min(ay2, by2) - max(ay1, by1))
    intersection = intersection_width * intersection_height
    union = (
        (ax2 - ax1) * (ay2 - ay1)
        + (bx2 - bx1) * (by2 - by1)
        - intersection
    )
    return intersection / union if union > 0 else 0.0


class AppearanceResultStore:
    """Publish and filter immutable appearance-result snapshots."""

    def __init__(
        self,
        clock: Callable[[], float],
        max_age: float,
        min_box_iou: float,
    ) -> None:
        self._clock = clock
        self._max_age = float(max_age)
        self._min_box_iou = float(min_box_iou)
        self._lock = threading.Lock()
        self._snapshot = AppearanceSnapshot({}, {}, -1e9)

    def publish(
        self,
        vectors: dict[int, np.ndarray],
        boxes: dict[int, tuple[float, ...]],
    ) -> None:
        with self._lock:
            self._snapshot = AppearanceSnapshot(vectors, boxes, self._clock())

    def latest(
        self,
        current_boxes: dict[int, Sequence[float]] | None = None,
    ) -> dict[int, np.ndarray]:
        with self._lock:
            snapshot = self._snapshot
            if self._clock() - snapshot.timestamp > self._max_age:
                return {}
            if current_boxes is None:
                return dict(snapshot.vectors)
            return {
                key: vector
                for key, vector in snapshot.vectors.items()
                if self._matches_current_box(key, current_boxes, snapshot)
            }

    def _matches_current_box(
        self,
        key: int,
        current_boxes: dict[int, Sequence[float]],
        snapshot: AppearanceSnapshot,
    ) -> bool:
        current = current_boxes.get(key)
        source = snapshot.boxes.get(key)
        return (
            current is not None
            and source is not None
            and _iou_xyxy(source, current) >= self._min_box_iou
        )


class AppearanceBatchProcessor:
    """Compute one batch and publish it without owning thread lifecycle."""

    def __init__(
        self,
        embedder: AppearanceBackend,
        results: AppearanceResultStore,
        logger: ILogger | None,
        on_batch: Callable[[int], None] | None,
    ) -> None:
        self._embedder = embedder
        self._results = results
        self._logger = logger
        self._on_batch = on_batch

    def compute(
        self,
        frame: np.ndarray,
        boxes: list[Sequence[float]],
        keys: list[int],
    ) -> None:
        try:
            vectors = self._embedder.embed(frame, boxes)
        except Exception as error:  # A ReID failure must not break tracking.
            if self._logger is not None:
                self._logger.error(f"Appearance embed error: {error}")
            return

        result = {
            int(key): vectors[index]
            for index, key in enumerate(keys)
            if index < len(vectors)
        }
        result_boxes = {
            int(key): tuple(boxes[index])
            for index, key in enumerate(keys)
            if index < len(vectors)
        }
        self._results.publish(result, result_boxes)
        if self._on_batch is not None and result:
            self._on_batch(len(result))

    def close(self) -> None:
        self._embedder.close()


class AsyncAppearanceEmbedder:
    """Non-blocking appearance facade with provenance-bounded publication."""

    def __init__(
        self,
        embedder: AppearanceBackend,
        *,
        logger: ILogger | None = None,
        on_batch: Callable[[int], None] | None = None,
        sync: bool = False,
        clock: Callable[[], float] = time.monotonic,
        max_age: float = 0.3,
        min_box_iou: float = 0.45,
    ) -> None:
        try:
            self._sync = bool(sync)
            self._lifecycle = threading.Condition()
            self._state = "open"
            self._close_error: BaseException | None = None
            self._results = AppearanceResultStore(clock, max_age, min_box_iou)
            self._processor = AppearanceBatchProcessor(
                embedder, self._results, logger, on_batch
            )
            self._worker: AppearanceWorker | None = None
            if not self._sync:
                worker = AppearanceWorker(
                    self._processor,
                    logger,
                    self._on_worker_exit,
                )
                self._worker = worker
                worker.start()
        except BaseException as start_error:
            worker = getattr(self, "_worker", None)
            if worker is not None and worker.launch_committed:
                if not worker.close():
                    raise BaseExceptionGroup(
                        "appearance wrapper startup and cleanup failed",
                        [
                            start_error,
                            TimeoutError(
                                "appearance worker startup rollback timed out"
                            ),
                        ],
                    ) from None
            else:
                try:
                    embedder.close()
                except BaseException as cleanup_error:
                    raise BaseExceptionGroup(
                        "appearance wrapper startup and cleanup failed",
                        [start_error, cleanup_error],
                    ) from None
            raise

    def submit(
        self,
        frame: np.ndarray,
        boxes: Sequence[Sequence[float]],
        keys: Sequence[int],
    ) -> None:
        if frame is None or not boxes:
            return
        box_list = list(boxes)
        key_list = list(keys)
        with self._lifecycle:
            if self._state != "open":
                return
            if self._sync:
                self._processor.compute(frame, box_list, key_list)
            elif self._worker is not None:
                self._worker.submit(frame, box_list, key_list)

    def latest(
        self,
        current_boxes: dict[int, Sequence[float]] | None = None,
    ) -> dict[int, np.ndarray]:
        return self._results.latest(current_boxes)

    def close(self) -> None:
        with self._lifecycle:
            if self._state == "closed":
                self._replay_close_error()
                return
            self._state = "closing"
            worker = self._worker
        if worker is not None:
            current_worker = worker.is_current_thread()
            if not worker.close() and not current_worker:
                raise TimeoutError(
                    "appearance worker did not stop; cleanup remains owned"
                )
            if current_worker:
                return
            with self._lifecycle:
                self._replay_close_error()
            return
        self._close_processor()

    def _on_worker_exit(self) -> None:
        with self._lifecycle:
            if self._state == "closed":
                return
            self._state = "closing"
        self._close_processor()

    def _close_processor(self) -> None:
        error: BaseException | None = None
        try:
            self._processor.close()
        except BaseException as caught:
            error = caught
        finally:
            with self._lifecycle:
                self._worker = None
                self._close_error = error
                self._state = "closed"
                self._lifecycle.notify_all()
        if error is not None:
            raise error

    def _replay_close_error(self) -> None:
        if self._close_error is not None:
            raise self._close_error


__all__ = ["AsyncAppearanceEmbedder"]
