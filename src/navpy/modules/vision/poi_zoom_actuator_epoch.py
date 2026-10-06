"""Reset controller state when the live zoom actuator is replaced."""

from __future__ import annotations

from navpy.modules.vision.poi_zoom_continuous import ContinuousZoomTick
from navpy.modules.vision.poi_zoom_drive import ZoomDrive
from navpy.modules.vision.poi_zoom_ports import ZoomActuatorIdentityReader
from navpy.modules.vision.poi_zoom_session import PoiZoomSession


def _same_identity(left: object, right: object) -> bool:
    if isinstance(left, tuple) and isinstance(right, tuple):
        return len(left) == len(right) and all(
            current is previous
            for current, previous in zip(left, right)
        )
    return left is right


class PoiZoomActuatorEpoch:
    """Invalidate command memory at the live mount replacement boundary."""

    def __init__(
        self,
        identity: ZoomActuatorIdentityReader,
        drive: ZoomDrive,
        continuous: ContinuousZoomTick,
        session: PoiZoomSession,
    ) -> None:
        self._identity_reader = identity
        self._drive = drive
        self._continuous = continuous
        self._session = session
        self._identity = identity.actuator_identity

    def refresh(self) -> bool:
        current = self._identity_reader.actuator_identity
        if _same_identity(current, self._identity):
            return False
        self._identity = current
        self._drive.reset_actuator_source()
        self._continuous.reset()
        self._session.reset_result()
        return True


__all__ = ["PoiZoomActuatorEpoch"]
