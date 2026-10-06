"""Library-backed tracker adapters (boxmot BoT-SORT/StrongSORT).

Kept separate from the lightweight config/factory/custom backend so importing
the public ``tracker_backends`` module stays cheap and one file == one concern.

Input/output conventions (verified against boxmot 19/21):
  - BoT-SORT/StrongSORT ``update`` wants an ``(N, 6)`` array
    ``[x1, y1, x2, y2, conf, cls]`` and the current frame.
"""
from __future__ import annotations

from typing import List, Optional

import numpy as np

from navpy.modules.vision.device import (
    free_cuda_cache,
    DeviceT,
    resolve_reid_device,
    uses_cuda_device,
)
from navpy.modules.vision.multi_object_tracker import TrackedObject
from navpy.modules.vision.track_export_memory import (
    _EMPTY_DETECTION_COAST_SECONDS,
    _TrackExportMemory,
)
from navpy.modules.vision.tracker_config import TrackerBackendConfig
from navpy.modules.vision.yolo_detector import Detection
import time


class BotSortTrackerBackend:
    def __init__(
            self,
            config: TrackerBackendConfig,
            *,
            detector_device: DeviceT,
            logger=None,
    ):
        self._config = config
        self._detector_device = detector_device
        self._logger = logger
        self._reid_device = resolve_reid_device(
            config.reid.device,
            detector_device,
        )
        self._reid_gpu = uses_cuda_device(self._reid_device)
        self._reid = None
        self._closed = False
        self._tracker = self._build_tracker()
        self._memory = _TrackExportMemory(
            max_age_seconds=max(1.0, config.custom.revive_seconds)
        )
        if logger is not None:
            half = config.reid.enabled and config.reid.half and self._reid_gpu
            logger.info(
                f"Tracker backend loaded: botsort with_reid={config.reid.enabled} "
                f"reid_device={self._reid_device} half={half} "
                f"cmc={config.reid.cmc_method}"
            )

    def _build_tracker(self):
        config = self._config
        reid_cls, botsort_cls = _load_botsort_deps()
        reid_model = None
        if config.reid.enabled:
            # boxmot: ReID is a runtime wrapper; the *backend* it returns is the
            # object BoT-SORT calls get_features() on. Passing reid.model (which
            # does not exist) would crash on the first GPU frame.
            self._reid = reid_cls(
                weights=config.reid.weights,
                device=self._reid_device,
                half=config.reid.half and self._reid_gpu,
            )
            reid_model = self._reid.get_backend()
        thresholds = config.association.resolved_botsort_thresholds()
        return botsort_cls(
            reid_model=reid_model,
            with_reid=config.reid.enabled,
            frame_rate=config.runtime.frame_rate,
            track_buffer=config.custom.max_age,
            cmc_method=config.reid.cmc_method,
            fuse_first_associate=config.association.fuse_first_associate,
            **thresholds,
        )

    def update(
            self,
            detections: List[Detection],
            frame_w: int,
            frame_h: int,
            frame: Optional[np.ndarray] = None,
    ) -> List[TrackedObject]:
        if frame is None:
            raise ValueError(
                "BoT-SORT tracker requires the current frame for ReID embeddings "
                "and camera-motion compensation"
            )
        now = time.time()
        if not detections:
            self._memory.prune(set(), now)
            return self._memory.coast(now, max_age_seconds=_EMPTY_DETECTION_COAST_SECONDS)
        rows = np.asarray(self._tracker.update(_detections_to_xyxy_array(detections), frame))
        out: List[TrackedObject] = []
        seen: set[int] = set()
        for row in rows:
            if len(row) < 7:
                continue
            x1, y1, x2, y2 = [float(v) for v in row[:4]]
            track_id = _safe_int_id(row[4])
            confidence = float(row[5])
            class_id = int(row[6])
            seen.add(track_id)
            out.append(
                self._memory.export(track_id, x1, y1, x2, y2, confidence, class_id, True, 0, now)
            )
        self._memory.prune(seen, now)
        return out

    def reset(self) -> None:
        if self._closed:
            return  # do not rebuild GPU resources after close()
        if hasattr(self._tracker, "reset"):
            self._tracker.reset()
        else:
            self._tracker = self._build_tracker()
        self._memory.clear()

    def close(self) -> None:
        self._closed = True
        self._tracker = None
        self._reid = None
        self._memory.clear()
        free_cuda_cache(self._reid_device)


class StrongSortTrackerBackend:
    def __init__(
            self,
            config: TrackerBackendConfig,
            *,
            detector_device: DeviceT,
            logger=None,
    ):
        self._config = config
        self._detector_device = detector_device
        self._logger = logger
        self._reid_device = resolve_reid_device(
            config.reid.device,
            detector_device,
        )
        self._reid_gpu = uses_cuda_device(self._reid_device)
        self._reid = None
        self._closed = False
        self._tracker = self._build_tracker()
        self._memory = _TrackExportMemory(
            max_age_seconds=max(1.0, config.custom.revive_seconds)
        )
        if logger is not None:
            half = config.reid.half and self._reid_gpu
            logger.info(
                f"Tracker backend loaded: strongsort reid_device={self._reid_device} "
                f"half={half} cmc={config.reid.cmc_method}"
            )

    def _build_tracker(self):
        config = self._config
        if not config.reid.enabled:
            raise ValueError("StrongSORT backend requires with_reid=true for appearance memory")
        reid_cls, strongsort_cls = _load_strongsort_deps()
        self._reid = reid_cls(
            weights=config.reid.weights,
            device=self._reid_device,
            half=config.reid.half and self._reid_gpu,
        )
        tracker = strongsort_cls(
            reid_model=self._reid.get_backend(),
            min_conf=float(config.association.detector_conf),
            max_cos_dist=float(config.association.appearance_thresh),
            max_iou_dist=float(config.association.match_thresh),
            n_init=int(config.custom.min_hits),
            nn_budget=100,
            det_thresh=float(config.association.detector_conf),
            max_age=int(config.custom.max_age),
            max_obs=max(50, int(config.custom.max_age)),
            min_hits=int(config.custom.min_hits),
        )
        _configure_strongsort_cmc(tracker, config.reid.cmc_method)
        return tracker

    def update(
            self,
            detections: List[Detection],
            frame_w: int,
            frame_h: int,
            frame: Optional[np.ndarray] = None,
    ) -> List[TrackedObject]:
        if frame is None:
            raise ValueError("StrongSORT tracker requires the current frame for ReID embeddings")
        now = time.time()
        if not detections:
            self._memory.prune(set(), now)
            return self._memory.coast(now, max_age_seconds=_EMPTY_DETECTION_COAST_SECONDS)
        rows = np.asarray(self._tracker.update(_detections_to_xyxy_array(detections), frame))
        out: List[TrackedObject] = []
        seen: set[int] = set()
        for row in rows:
            if len(row) < 7:
                continue
            x1, y1, x2, y2 = [float(v) for v in row[:4]]
            track_id = _safe_int_id(row[4])
            confidence = float(row[5])
            class_id = int(row[6])
            seen.add(track_id)
            out.append(
                self._memory.export(track_id, x1, y1, x2, y2, confidence, class_id, True, 0, now)
            )
        self._memory.prune(seen, now)
        return out

    def reset(self) -> None:
        if self._closed:
            return  # do not rebuild GPU resources after close()
        if hasattr(self._tracker, "reset"):
            self._tracker.reset()
        else:
            self._tracker = self._build_tracker()
        self._memory.clear()

    def close(self) -> None:
        self._closed = True
        self._tracker = None
        self._reid = None
        self._memory.clear()
        free_cuda_cache(self._reid_device)



def _load_botsort_deps():
    try:
        from boxmot.reid.core import ReID
        from boxmot.trackers.bbox.botsort.botsort import BotSort
    except ImportError as exc:
        raise RuntimeError(
            "BoT-SORT backend requires dependency 'boxmot'. "
            "Install with: python -m pip install boxmot"
        ) from exc
    return ReID, BotSort


def _load_strongsort_deps():
    try:
        from boxmot.reid.core import ReID
        from boxmot.trackers.bbox.strongsort.strongsort import StrongSort
    except ImportError as exc:
        raise RuntimeError(
            "StrongSORT backend requires dependency 'boxmot'. "
            "Install with: python -m pip install boxmot"
        ) from exc
    return ReID, StrongSort


def _detections_to_xyxy_array(detections: List[Detection]) -> np.ndarray:
    dets = np.array(
        [[*d.xyxy, d.confidence, d.class_id] for d in detections],
        dtype=np.float32,
    )
    if dets.size == 0:
        return np.empty((0, 6), dtype=np.float32)
    return dets


def _configure_strongsort_cmc(tracker, cmc_method: Optional[str]) -> None:
    if cmc_method is None:
        tracker.cmc = _NoOpCMC()
        return
    try:
        from boxmot.motion.cmc import get_cmc_method
    except ImportError as exc:
        raise RuntimeError(
            "StrongSORT camera-motion compensation requires dependency 'boxmot'. "
            "Install with: python -m pip install boxmot"
        ) from exc
    tracker.cmc = get_cmc_method(cmc_method)()


class _NoOpCMC:
    def apply(self, img, xyxy) -> np.ndarray:
        return np.eye(2, 3, dtype=np.float32)


def _safe_int_id(value) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return abs(hash(str(value))) % 2_000_000_000
