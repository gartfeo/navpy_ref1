"""Read-only public target-zoom state and compatibility accessors."""

from __future__ import annotations

from navpy.modules.vision.target_zoom_drive import ZoomDrive
from navpy.modules.vision.target_zoom_ports import ZoomCapabilityReader
from navpy.modules.vision.target_zoom_session import TargetZoomSession
from navpy.modules.vision.target_zoom_types import ZoomTrackResult


class TargetZoomPublicView:
    def __init__(
        self,
        capabilities: ZoomCapabilityReader,
        drive: ZoomDrive,
        session: TargetZoomSession,
    ) -> None:
        self._capabilities = capabilities
        self._drive = drive
        self._session = session

    @property
    def is_supported(self) -> bool:
        return self._capabilities.capabilities.supported

    @property
    def minimum(self) -> float:
        return self._capabilities.capabilities.minimum

    @property
    def maximum(self) -> float:
        return self._capabilities.capabilities.maximum

    @property
    def last_result(self) -> ZoomTrackResult:
        return self._session.last_result

    @property
    def is_stable(self) -> bool:
        return self._session.last_result.is_stable

    @property
    def size_demand(self) -> bool:
        return self._session.size_demand

    @property
    def absolute_target(self) -> str | None:
        return self._drive.absolute_target

    @absolute_target.setter
    def absolute_target(self, value: str | None) -> None:
        self._drive.absolute_target = value

    def target_pixels_for_class(self, class_id: object) -> float:
        return self._session.target_pixels_for_class(class_id)


__all__ = ["TargetZoomPublicView"]
