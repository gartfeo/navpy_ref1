"""Request-local selection over simulator detection publications."""

from __future__ import annotations

from collections.abc import Callable
from copy import deepcopy
from typing import List, Optional

from navpy.modules.vision.geometry import iou_cxcywh
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.detection_event_lease import (
    DetectionEventLease,
    DetectionLeaseDispatch,
)
from navpy.modules.vision.models.detection_publication import DetectionPublication
from navpy.modules.vision.models.detect_request import DetectRequest
from navpy.modules.vision.models.detect_response import DetectResponse
from navpy.modules.vision.sim.detection_publication_store import (
    DetectionPublicationStore,
)
from navpy.modules.vision.sim.sim_runtime_ports import (
    FrameOutcomeSink,
    OptionalTextReader,
)
from navpy.modules.vision.target_priority import (
    find_target_by_id,
    prioritize_targets,
)


class DetectionPublicationBuffer:
    """Apply force-lock choice without owning publication transactions."""

    def __init__(
        self,
        store: DetectionPublicationStore,
        *,
        record_outcome: FrameOutcomeSink,
        fallback_source_name: OptionalTextReader,
    ) -> None:
        self._store = store
        self._record_outcome = record_outcome
        self._fallback_source_name = fallback_source_name

    @property
    def source_driven(self) -> bool:
        return self._store.source_driven

    def source_name(
        self,
        detected_targets: List[DetectedObject],
    ) -> Optional[str]:
        for target in detected_targets:
            source_name = target.pixel.source_name
            if isinstance(source_name, str) and source_name:
                return source_name
        return self._fallback_source_name()

    def drain_detection_events(
        self,
        request: DetectRequest,
    ) -> List[DetectionPublication]:
        return self._prepare_publications(request, self._store.drain())

    def open_detection_event_lease(
        self,
        request: DetectRequest,
        reset_handler: Callable[[], None] | None = None,
    ) -> "BufferedDetectionEventLease":
        return BufferedDetectionEventLease(
            self._store.open_event_lease(reset_handler),
            self,
            request,
        )

    def _prepare_publications(
        self,
        request: DetectRequest,
        publications: list[DetectionPublication],
    ) -> List[DetectionPublication]:
        responses = []
        for publication in publications:
            self._record_outcome(
                publication.source_timestamp_s,
                "emitted",
                publication.source_timestamp_s,
            )
            forced = self.resolve_forced_target(
                request,
                list(publication.detected_targets),
            )
            primary = forced or publication.primary_target
            responses.append(publication.with_targets(prioritize_targets(
                publication.detected_targets,
                primary,
            )))
        return responses

    def get_detect_data(self, request: DetectRequest) -> DetectResponse:
        snapshot = self._store.snapshot()
        targets = list(snapshot.detected_targets)
        primary = self.resolve_forced_target(request, targets)
        if primary is None:
            primary = snapshot.primary_target
        ordered = prioritize_targets(targets, primary)
        return DetectResponse(ordered, primary_target=primary)

    def get_latest_detections(self) -> List[DetectedObject]:
        return list(self._store.snapshot().debug_targets)

    @staticmethod
    def resolve_forced_target(
        request: DetectRequest,
        targets: List[DetectedObject],
    ) -> Optional[DetectedObject]:
        if request.force_lock_id is not None:
            return find_target_by_id(targets, request.force_lock_id)
        bbox = request.force_lock_bbox_cxcywh
        if bbox is None:
            return None
        fcx, fcy = float(bbox[0]), float(bbox[1])
        fx1, fy1, fx2, fy2 = (
            fcx - float(bbox[2]) * 0.5,
            fcy - float(bbox[3]) * 0.5,
            fcx + float(bbox[2]) * 0.5,
            fcy + float(bbox[3]) * 0.5,
        )
        best, best_score = None, 0.0
        for target in targets:
            target_bbox = target.tracking.bbox_cxcywh
            if target_bbox is None:
                continue
            inside = (
                fx1 <= float(target_bbox[0]) <= fx2
                and fy1 <= float(target_bbox[1]) <= fy2
            )
            score = iou_cxcywh(bbox, target_bbox) + (0.25 if inside else 0.0)
            if score > best_score:
                best, best_score = target, score
        return best


class BufferedDetectionEventLease:
    """Apply request-local target ordering to one exclusive store lease."""

    def __init__(
        self,
        lease: DetectionEventLease,
        buffer: DetectionPublicationBuffer,
        request: DetectRequest,
    ) -> None:
        self._lease = lease
        self._buffer = buffer
        self._request = DetectRequest(
            request.force_lock_id,
            deepcopy(request.force_lock_bbox_cxcywh),
        )

    @property
    def closed(self) -> bool:
        return self._lease.closed

    def wait_and_dispatch(
        self,
        handler: Callable[[DetectionPublication], bool],
    ) -> bool | None:
        def prepare(publication: DetectionPublication) -> bool:
            prepared = self._buffer._prepare_publications(
                self._request,
                [publication],
            )
            return bool(prepared) and handler(prepared[0])

        return self._lease.wait_and_dispatch(prepare)

    def dispatch_available(
        self,
        handler: Callable[[DetectionPublication], bool],
    ) -> DetectionLeaseDispatch:
        def prepare(publication: DetectionPublication) -> bool:
            prepared = self._buffer._prepare_publications(
                self._request,
                [publication],
            )
            return bool(prepared) and handler(prepared[0])

        return self._lease.dispatch_available(prepare)

    def close(self) -> None:
        self._lease.close()


__all__ = ["BufferedDetectionEventLease", "DetectionPublicationBuffer"]
