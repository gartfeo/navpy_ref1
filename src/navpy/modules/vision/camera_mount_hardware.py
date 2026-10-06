"""Compatibility boundary around optional camera and gimbal capabilities."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from typing import cast

import numpy as np

from navpy.modules.vision.camera_mount_types import ZoomWrite
from navpy.modules.vision.peripheral.camera_abc import CameraAbc
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.worker_failure import require_failure_health


def _provider(owner: object, name: str) -> Callable[..., object] | None:
    candidate = getattr(owner, name, None)
    return cast(Callable[..., object], candidate) if callable(candidate) else None


class FrameStateCapabilityUnavailable(RuntimeError):
    """The gimbal does not publish an atomic frame-state sample."""


class MountCameraHardware:
    """Expose the exact camera operations needed by mount behavior."""

    def __init__(self, source: Callable[[], object]) -> None:
        self._source = source

    @property
    def identity(self) -> object:
        return self._source()

    def get_k(self) -> np.ndarray:
        return self._source().get_k()

    def get_dist(self) -> np.ndarray:
        provider = _provider(self._source(), "get_dist")
        if provider is None:
            return np.zeros(5, dtype=np.float32)
        return provider()

    @property
    def image_width(self) -> int | None:
        return getattr(self._source(), "image_width", None)

    @property
    def image_height(self) -> int | None:
        return getattr(self._source(), "image_height", None)

    def is_valid(self, u: float, v: float) -> bool:
        provider = _provider(self._source(), "is_valid")
        return True if provider is None else provider(u, v)

    def refresh(self) -> None:
        provider = _provider(self._source(), "refresh")
        if provider is not None:
            provider()

    def raise_if_failed(self) -> None:
        require_failure_health(self._source()).raise_if_failed()

    def zoom_key(self) -> str | None:
        camera = self._source()
        if isinstance(camera, CameraAbc):
            return camera.get_zoom_key()
        current = getattr(camera, "__dict__", {}).get("_zoom")
        return None if current is None else str(current)

    def zoom_map(self) -> Mapping[object, object] | None:
        camera = self._source()
        if isinstance(camera, CameraAbc):
            return camera.get_zoom_map()
        value = getattr(camera, "_zoom_map", None)
        return value if isinstance(value, Mapping) else None

    def apply_zoom(self, zoom: str) -> ZoomWrite:
        provider = _provider(self._source(), "set_zoom")
        if provider is None:
            return ZoomWrite(False)
        return ZoomWrite(True, provider(zoom))


class MountGimbalHardware:
    """Expose optional gimbal capabilities without spreading dynamic probes."""

    def __init__(self, source: Callable[[], object]) -> None:
        self._source = source

    @property
    def identity(self) -> object:
        return self._source()

    def get_data(self) -> GimbalData:
        return self._source().get_data()

    def state_is_static(self) -> bool:
        return self._source().state_is_static() is True

    def frame_state_sample(
        self,
    ) -> tuple[GimbalData, object, object, object]:
        provider = _provider(self._source(), "get_frame_state_sample")
        if provider is None:
            raise FrameStateCapabilityUnavailable(
                "missing frame-state sample provider"
            )
        return cast(tuple[GimbalData, object, object, object], provider())

    def start(self) -> None:
        provider = _provider(self._source(), "start")
        if provider is not None:
            provider()

    def stop(self) -> bool:
        provider = _provider(self._source(), "stop")
        if provider is None:
            return True
        result = provider()
        if not isinstance(result, bool):
            raise TypeError("gimbal stop() must return bool")
        return result

    def refresh(self) -> None:
        provider = _provider(self._source(), "refresh")
        if provider is not None:
            provider()

    def raise_if_failed(self) -> None:
        require_failure_health(self._source()).raise_if_failed()

    def apply_zoom(self, zoom: str) -> ZoomWrite:
        provider = _provider(self._source(), "set_zoom")
        if provider is None:
            return ZoomWrite(False)
        return ZoomWrite(True, provider(zoom))


class MountGimbalZoomReadbackHardware:
    """Expose only the optional physical-zoom telemetry surface."""

    def __init__(self, source: Callable[[], object]) -> None:
        self._source = source

    def zoom_level(self) -> object:
        provider = _provider(self._source(), "get_zoom_level")
        return None if provider is None else provider()

    def has_zoom_age(self) -> bool:
        return _provider(self._source(), "get_zoom_level_age_s") is not None

    def zoom_age(self) -> object:
        provider = _provider(self._source(), "get_zoom_level_age_s")
        return None if provider is None else provider()

    def has_zoom_fresh_flag(self) -> bool:
        return _provider(self._source(), "is_zoom_level_fresh") is not None

    def zoom_fresh_flag(self) -> object:
        provider = _provider(self._source(), "is_zoom_level_fresh")
        return None if provider is None else provider()

    def zoom_sample(self) -> object:
        provider = _provider(self._source(), "get_zoom_level_sample")
        return None if provider is None else provider()

    def supports_zoom_readback(self) -> bool:
        provider = _provider(self._source(), "supports_zoom_readback")
        if provider is None:
            return False
        return bool(provider())


class MountGimbalTargetZoomHardware:
    """Expose only continuous and absolute target-zoom capabilities."""

    def __init__(self, source: Callable[[], object]) -> None:
        self._source = source

    def supports_absolute_zoom(self) -> bool:
        provider = _provider(self._source(), "supports_absolute_zoom_control")
        return False if provider is None else bool(provider())

    def supports_continuous_zoom(self) -> bool:
        provider = _provider(self._source(), "supports_continuous_zoom_control")
        return False if provider is None else bool(provider())

    def zoom_in(self) -> bool:
        provider = _provider(self._source(), "zoom_in")
        return False if provider is None else provider() is True

    def zoom_out(self) -> bool:
        provider = _provider(self._source(), "zoom_out")
        return False if provider is None else provider() is True

    def zoom_hold(self) -> bool:
        provider = _provider(self._source(), "zoom_hold")
        return False if provider is None else provider() is True


__all__ = [
    "FrameStateCapabilityUnavailable",
    "MountCameraHardware",
    "MountGimbalHardware",
    "MountGimbalTargetZoomHardware",
    "MountGimbalZoomReadbackHardware",
]
