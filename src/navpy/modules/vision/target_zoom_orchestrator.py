"""Stable target-zoom API assembled from focused capability facets."""

from __future__ import annotations

from navpy.modules.vision.gimbal_rate_types import GimbalTrackResult
from navpy.modules.vision.target_zoom_composition import (
    TargetZoomParts,
    build_target_zoom,
)
from navpy.modules.vision.target_zoom_geometry import extract_bbox
from navpy.modules.vision.target_zoom_ports import ZoomLogger, ZoomMountPort
from navpy.modules.vision.target_zoom_types import (
    TargetZoomTrackerConfig,
    ZoomStopPlan,
    ZoomTrackResult,
)


class TargetZoomViewFacet:
    _parts: TargetZoomParts

    @property
    def is_supported(self) -> bool:
        return self._parts.view.is_supported

    @property
    def min_zoom(self) -> float:
        return self._parts.view.minimum

    @property
    def max_zoom(self) -> float:
        return self._parts.view.maximum

    @property
    def last_result(self) -> ZoomTrackResult:
        return self._parts.view.last_result

    @property
    def is_zoom_stable(self) -> bool:
        return self._parts.view.is_stable

    @property
    def size_demand(self) -> bool:
        return self._parts.view.size_demand

    @property
    def _absolute_target(self) -> str | None:
        return self._parts.view.absolute_target

    @_absolute_target.setter
    def _absolute_target(self, value: str | None) -> None:
        self._parts.view.absolute_target = value

    def get_target_pixels_for_class(self, class_id: object) -> float:
        return self._parts.view.target_pixels_for_class(class_id)

    @staticmethod
    def _extract_bbox_cxcywh(
        target: object,
    ) -> tuple[float, float, float, float] | None:
        return extract_bbox(target)


class TargetZoomLifecycleFacet:
    _parts: TargetZoomParts

    def set_size_demand(self, enabled: bool) -> None:
        self._parts.lifecycle.set_size_demand(enabled)

    def start_session(self) -> bool:
        return self._parts.lifecycle.start_session()

    def prepare_session_start(self) -> ZoomStopPlan | None:
        return self._parts.lifecycle.prepare_session_start()

    def commit_session_start(self, plan: ZoomStopPlan) -> None:
        self._parts.lifecycle.commit_session_start(plan)

    def reset(self) -> bool:
        return self._parts.lifecycle.reset()

    def reset_to_min(self) -> bool:
        return self._parts.lifecycle.reset_to_min()


class TargetZoomUpdateFacet:
    _parts: TargetZoomParts

    def update(
        self,
        target: object | None,
        now: float | None = None,
        pointing: GimbalTrackResult | None = None,
    ) -> ZoomTrackResult:
        del now  # Detection cadence changes; source/wall time never does.
        return self._parts.updater.update(target, pointing)


class TargetZoomTracker(
    TargetZoomViewFacet,
    TargetZoomLifecycleFacet,
    TargetZoomUpdateFacet,
):
    """One-field compatibility boundary over focused zoom owners."""

    def __init__(
        self,
        mount: ZoomMountPort,
        logger: ZoomLogger,
        config: TargetZoomTrackerConfig | None = None,
        clock: object | None = None,
    ) -> None:
        del clock  # Compatibility only; zoom state never scales source/wall time.
        self._parts = build_target_zoom(mount, logger, config)


__all__ = ["TargetZoomTracker"]
