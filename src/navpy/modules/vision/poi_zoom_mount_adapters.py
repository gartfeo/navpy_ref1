"""Validation adapter over the explicit POI-zoom mount contract."""

from __future__ import annotations

import math
from collections.abc import Callable

from navpy.modules.vision.poi_zoom_ports import ZoomLogger, ZoomMountPort
from navpy.modules.vision.poi_zoom_types import (
    FiniteReading,
    SampleIdReading,
    ZoomCapabilities,
    ZoomGeometry,
    ZoomGeometryReading,
)
from navpy.modules.vision.zoom_tracking_types import ZoomTrackingState


def _validated_levels(values: object) -> tuple[tuple[str, float], ...]:
    parsed: dict[float, str] = {}
    if not isinstance(values, (list, tuple)):
        return ()
    for raw in values:
        if isinstance(raw, bool):
            continue
        try:
            value = float(raw)
        except (TypeError, ValueError):
            continue
        if math.isfinite(value) and value > 0.0:
            parsed.setdefault(value, str(raw))
    return tuple((parsed[value], value) for value in sorted(parsed))


def _positive_values(values: object) -> tuple[float, ...]:
    return tuple(value for _label, value in _validated_levels(values))


def _command_values(
    mount: ZoomMountPort,
    levels: tuple[float, ...],
) -> tuple[float, ...]:
    calibration = mount.zoom_calibration
    if calibration is not None:
        calibrated = _positive_values(
            tuple(entry.commanded for entry in calibration.entries)
        )
        if calibrated:
            return calibrated
    return levels


def _capabilities(mount: ZoomMountPort) -> ZoomCapabilities:
    validated_levels = _validated_levels(mount.get_zoom_levels())
    level_values = tuple(value for _label, value in validated_levels)
    commands = _command_values(mount, level_values)
    values = commands or level_values or (1.0,)
    differences = [
        right - left for left, right in zip(values, values[1:]) if right > left
    ]
    step = round(min(differences), 9) if differences else 1.0
    return ZoomCapabilities(
        continuous=mount.supports_continuous_zoom(),
        absolute=mount.supports_absolute_zoom(),
        levels=tuple(label for label, _value in validated_levels),
        minimum=min(values),
        maximum=max(values),
        command_step=step,
    )


def _finite_reading(
    provider: Callable[[], object],
    logger: ZoomLogger,
    label: str,
) -> FiniteReading:
    try:
        raw = provider()
    except OSError as error:
        logger.warning(f"{label} readback failed: {error}")
        return FiniteReading(None, invalid=True)
    if raw is None:
        return FiniteReading(None)
    if isinstance(raw, bool):
        return FiniteReading(None, invalid=True)
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return FiniteReading(None, invalid=True)
    if not math.isfinite(value) or value <= 0.0:
        return FiniteReading(None, invalid=True)
    return FiniteReading(value)


class MountZoomAdapter:
    """Validate explicit mount readings and expose narrow zoom leaf ports."""

    def __init__(self, mount: ZoomMountPort, logger: ZoomLogger) -> None:
        self._mount = mount
        self._logger = logger

    @property
    def actuator_identity(self) -> object:
        provider = getattr(self._mount, "zoom_actuator_identity", None)
        if callable(provider):
            return provider()
        return getattr(self._mount, "gimbal", self._mount)

    @property
    def capabilities(self) -> ZoomCapabilities:
        try:
            return _capabilities(self._mount)
        except OSError as error:
            self._logger.warning(f"zoom capability discovery failed: {error}")
            return ZoomCapabilities(False, False, (), 1.0, 1.0, 1.0, False)

    def sync_optics(self) -> bool | None:
        try:
            return self._mount.sync_zoom_from_hardware() is not False
        except OSError as error:
            self._logger.warning(f"sync_zoom_from_hardware failed: {error}")
            return None

    def current_command(self) -> FiniteReading:
        return _finite_reading(
            self._mount.get_current_zoom_command,
            self._logger,
            "current zoom command",
        )

    def fresh_command(self) -> FiniteReading:
        return _finite_reading(
            self._mount.get_fresh_zoom_command,
            self._logger,
            "fresh zoom command",
        )

    def current_level(self) -> FiniteReading:
        return _finite_reading(
            self._mount.get_current_zoom,
            self._logger,
            "current zoom level",
        )

    def fresh_sample_id(self) -> SampleIdReading:
        try:
            value = self._mount.get_fresh_zoom_sample_id()
        except OSError as error:
            self._logger.warning(f"fresh zoom sample id readback failed: {error}")
            return SampleIdReading(None, invalid=True)
        if value is None:
            return SampleIdReading(None)
        if isinstance(value, bool) or not isinstance(value, (str, int)):
            return SampleIdReading(None, invalid=True)
        return SampleIdReading(str(value))

    def geometry(self) -> ZoomGeometryReading:
        try:
            width = self._mount.image_width
            height = self._mount.image_height
        except OSError as error:
            self._logger.warning(f"zoom frame geometry readback failed: {error}")
            return ZoomGeometryReading(None, invalid=True)
        if width is None or height is None:
            return ZoomGeometryReading(None)
        if isinstance(width, bool) or isinstance(height, bool):
            return ZoomGeometryReading(None, invalid=True)
        if not isinstance(width, (int, float)) or not isinstance(height, (int, float)):
            return ZoomGeometryReading(None, invalid=True)
        if not math.isfinite(float(width)) or not math.isfinite(float(height)):
            return ZoomGeometryReading(None, invalid=True)
        if width <= 0.0 or height <= 0.0:
            return ZoomGeometryReading(None, invalid=True)
        try:
            raw_geometry = self._mount.get_k()
        except OSError as error:
            self._logger.warning(f"zoom camera geometry readback failed: {error}")
            return ZoomGeometryReading(None, invalid=True)
        try:
            fy = float(raw_geometry[1][1])
        except (IndexError, TypeError, ValueError):
            fy = None
        if fy is not None and (not math.isfinite(fy) or fy <= 0.0):
            fy = None
        return ZoomGeometryReading(ZoomGeometry(float(width), float(height), fy))

    def command_absolute(self, command: str) -> bool:
        try:
            return self._mount.command_zoom(command) is True
        except OSError as error:
            self._logger.warning(f"absolute zoom command failed: {error}")
            return False

    def command_discrete(self, command: str) -> bool:
        try:
            return self._mount.set_zoom(command) is True
        except OSError as error:
            self._logger.warning(f"discrete zoom command failed: {error}")
            return False

    def start_continuous(self, direction: ZoomTrackingState) -> bool:
        try:
            return self._mount.start_continuous_zoom(direction) is True
        except OSError as error:
            self._logger.warning(f"continuous zoom command failed: {error}")
            return False

    def hold(self) -> bool:
        try:
            return self._mount.hold_zoom() is True
        except OSError as error:
            self._logger.warning(f"zoom hold failed: {error}")
            return False


__all__ = ["MountZoomAdapter"]
