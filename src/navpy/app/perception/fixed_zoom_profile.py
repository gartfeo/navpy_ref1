"""Fixed zoom from a camera profile's reading table (HITL step 18).

Built only beside integration timing: a mount whose camera has ``camera.reading_intrinsics`` gets its measured
absolute-zoom steps and a settle evidence; any other mount gets none, and its zoom-capable frames stay refused.
The table must hold exactly one stream mode; a frame of any other size is refused at association.
"""

from __future__ import annotations

from collections.abc import Mapping

from navpy.core.perception.camera.reading_intrinsics import ReadingIntrinsicsTable
from navpy.core.perception.zoom.fixed_step_zoom_drive import FixedStepZoom
from navpy.core.perception.zoom.fixed_zoom_steps import FixedZoomSteps
from navpy.core.perception.zoom.zoom_settle_evidence import ZoomSettleEvidence


def _thaw(value: object) -> object:
    """Profile values are frozen (mapping proxies, tuples); the table parser takes JSON shapes."""
    if isinstance(value, Mapping):
        return {str(key): _thaw(child) for key, child in value.items()}
    if isinstance(value, tuple):
        return [_thaw(child) for child in value]
    return value


def build_fixed_zoom(device: Mapping[str, object]) -> FixedStepZoom | None:
    camera = device.get("camera")
    if not isinstance(camera, Mapping) or "reading_intrinsics" not in camera:
        return None
    table = ReadingIntrinsicsTable.from_profile(_thaw(camera["reading_intrinsics"]))
    streams = table.streams()
    if len(streams) != 1:
        raise ValueError(f"fixed zoom needs exactly one reading_intrinsics stream mode, found {len(streams)}")
    steps = FixedZoomSteps.from_table(table, streams[0])
    _check_command_readings(steps, camera.get("zoom_calibration"))
    return FixedStepZoom(steps, ZoomSettleEvidence(steps))


def _check_command_readings(steps: FixedZoomSteps, calibration: object) -> None:
    """The command-to-reading table the mount commands by must agree with every step's measured reading."""
    if not isinstance(calibration, Mapping):
        return
    by_tenths = {}
    for command, reading in calibration.items():
        try:
            by_tenths[round(float(command) * 10.0)] = round(float(reading) * 10.0)
        except (TypeError, ValueError, OverflowError):
            continue
    for step in steps.steps:
        expected = by_tenths.get(step.command_tenths)
        if expected is not None and expected != step.reading_tenths:
            raise ValueError(
                f"fixed zoom step {step.command}: zoom_calibration reads {expected / 10:.1f}, "
                f"reading_intrinsics {step.optics.reading}"
            )


__all__ = ["build_fixed_zoom"]
