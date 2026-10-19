"""Lens intrinsics keyed by the camera's reported zoom reading.

The SIYI ZR10 reports its zoom as a reading on a 0.1 grid (0x18, ``siyi_sdk.parseCurrentZoomLevelMsg``). The
reading, not the zoom command, names the lens state: the command→reading table is non-monotonic and a reading is
not a linear magnification. This table maps a stream mode and a reading to measured intrinsics, with no
interpolation or extrapolation: any other reading or stream mode is refused, so an unmeasured zoom stays disabled.
A frame whose two bracketing readings differ was exposed during lens travel and is refused too. The command→reading
table (``zoom/zoom_calibration.py``) stays separate and serves commanding only.

A reading is rounded down to its 0.1 step, so one reading can cover several lens states: a held zoom-in that stops
at 1.1 is not where absolute zoom 1.0 lands. Each row therefore names the setting it was measured in (the zoom held
out to its wide stop, or an absolute zoom command left to settle), and a lookup must name the setting that produced
the frame. Settling (travel and autofocus finished) is the caller's to establish; equal bracketing readings only
rule out travel that crosses a reading step.

Profile section ``camera.reading_intrinsics``::

    {"source": "<where the rows were measured>",
     "streams": [{"mode": "main", "image_width": 1920, "image_height": 1080, "crop": [0, 0, 1920, 1080],
                  "readings": {"1.8": {"set_by": {"absolute_zoom": 2.0}, "fx": ..., "fy": ..., "cx": ...,
                                       "cy": ..., "skew": 0.0, "dist": [k1, k2, p1, p2, k3]},
                               "1.0": {"set_by": "wide_stop", ...}}}]}

``crop`` is the x, y, width and height of the camera's stream that the frame shows; together with the frame size it
fixes the pixel scale and the image centre, so a resized or cropped frame needs its own measured stream entry.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np

WIDE_STOP = "wide_stop"
ABSOLUTE_ZOOM = "absolute_zoom"
_SECTION_FIELDS = frozenset({"source", "streams"})
_STREAM_FIELDS = frozenset({"mode", "image_width", "image_height", "crop", "readings"})
_ROW_FIELDS = frozenset({"set_by", "fx", "fy", "cx", "cy", "skew", "dist"})
_GRID_TOLERANCE = 1e-6      # in tenths: a float like 1 + 8/10 from the wire still lands on the 0.1 grid


def zoom_tenths(value: object) -> int | None:
    """A zoom reading or command in tenths, or None unless it is a finite number ≥ 0.1 on the 0.1 grid."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        scaled = float(value) * 10.0
    except OverflowError:       # an int beyond the float range
        return None
    if not math.isfinite(scaled):
        return None
    tenths = round(scaled)
    return tenths if tenths >= 1 and abs(scaled - tenths) <= _GRID_TOLERANCE else None


@dataclass(frozen=True)
class ZoomSetting:
    """How the lens reached its state: held out to the wide stop, or an absolute zoom command (0x0F) that settled."""

    kind: str
    command_tenths: int | None = None

    @classmethod
    def wide_stop(cls) -> "ZoomSetting":
        return cls(WIDE_STOP)

    @classmethod
    def absolute_zoom(cls, command: object) -> "ZoomSetting":
        tenths = zoom_tenths(command)
        if tenths is None:
            raise ValueError(f"absolute zoom command {command!r} is not a number >= 0.1 on the 0.1 grid")
        return cls(ABSOLUTE_ZOOM, tenths)


@dataclass(frozen=True)
class StreamMode:
    """The frame a row was measured on: stream name, frame size and the region of the stream it shows."""

    mode: str
    image_width: int
    image_height: int
    crop: tuple[int, int, int, int]


@dataclass(frozen=True)
class ReadingOptics:
    """Measured intrinsics at one settled zoom reading, reached by one setting."""

    reading: str
    setting: ZoomSetting
    fx: float
    fy: float
    cx: float
    cy: float
    skew: float
    dist: tuple[float, float, float, float, float]

    def k(self) -> np.ndarray:
        return np.array([[self.fx, self.skew, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]], dtype=float)

    def dist_coeffs(self) -> np.ndarray:
        """OpenCV order [k1, k2, p1, p2, k3]."""
        return np.array(self.dist, dtype=float)


class ReadingIntrinsicsTable:
    """Measured intrinsics per stream mode, settled reading and zoom setting; everything else is refused."""

    def __init__(self, rows: Mapping[StreamMode, Mapping[int, ReadingOptics]]) -> None:
        self._rows = {stream: dict(by_reading) for stream, by_reading in rows.items()}

    @classmethod
    def from_profile(cls, section: object) -> "ReadingIntrinsicsTable":
        """Parse a profile's ``camera.reading_intrinsics``; ValueError lists every problem."""
        problems: list[str] = []
        rows = _parse_section(section, problems)
        if problems:
            raise ValueError("camera.reading_intrinsics: " + "; ".join(problems))
        return cls(rows)

    def streams(self) -> tuple[StreamMode, ...]:
        return tuple(self._rows)

    def readings(self, stream: StreamMode) -> tuple[str, ...]:
        """The supported readings of a stream mode, ascending; empty for an unmeasured mode."""
        by_reading = self._rows.get(stream, {})
        return tuple(by_reading[tenths].reading for tenths in sorted(by_reading))

    def optics(self, stream: StreamMode, reading_before: object, reading_after: object,
               setting: ZoomSetting) -> ReadingOptics | None:
        """The intrinsics of a frame exposed between two zoom readings after the given setting, or None (refused)
        when the stream mode is not measured, a reading is not a table row, the readings differ (lens travel) or
        the row was measured after another setting."""
        before, after = zoom_tenths(reading_before), zoom_tenths(reading_after)
        if before is None or before != after:
            return None
        row = self._rows.get(stream, {}).get(before)
        return row if row is not None and row.setting == setting else None


def _is_count(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        number = float(value)
    except OverflowError:
        return None
    return number if math.isfinite(number) else None


def _fields(value: object, allowed: frozenset[str], required: frozenset[str], path: str,
            problems: list[str]) -> bool:
    if not isinstance(value, Mapping):
        problems.append(f"{path} must be a mapping")
        return False
    present = set(value)
    problems.extend(f"{path}.{key} is missing" for key in sorted(required - present))
    problems.extend(f"{path}.{key} is not a known field" for key in sorted(present - allowed, key=str))
    return required <= present


def _parse_section(section: object, problems: list[str]) -> dict[StreamMode, dict[int, ReadingOptics]]:
    rows: dict[StreamMode, dict[int, ReadingOptics]] = {}
    if not _fields(section, _SECTION_FIELDS, frozenset({"streams"}), "section", problems):
        return rows
    streams = section["streams"]
    if not isinstance(streams, list) or not streams:
        problems.append("streams must be a non-empty list")
        return rows
    for index, raw in enumerate(streams):
        path = f"streams[{index}]"
        stream = _parse_stream(raw, path, problems)
        if stream is None:
            continue
        if stream in rows:
            problems.append(f"{path} repeats stream mode {stream}")
            continue
        rows[stream] = _parse_readings(raw["readings"], stream, f"{path}.readings", problems)
    return rows


def _parse_stream(raw: object, path: str, problems: list[str]) -> StreamMode | None:
    if not _fields(raw, _STREAM_FIELDS, _STREAM_FIELDS, path, problems):
        return None
    mode, width, height, crop = raw["mode"], raw["image_width"], raw["image_height"], raw["crop"]
    start = len(problems)
    if not isinstance(mode, str) or not mode:
        problems.append(f"{path}.mode must be a non-empty string")
    if not (_is_count(width) and width > 0 and _is_count(height) and height > 0):
        problems.append(f"{path}.image_width and image_height must be positive integers")
    if not (isinstance(crop, list) and len(crop) == 4 and all(_is_count(v) for v in crop)
            and crop[0] >= 0 and crop[1] >= 0 and crop[2] > 0 and crop[3] > 0):
        problems.append(f"{path}.crop must be [x, y, width, height]: integers, x and y >= 0, sizes > 0")
    if len(problems) > start:
        return None
    return StreamMode(mode, width, height, tuple(crop))


def _parse_readings(raw: object, stream: StreamMode, path: str, problems: list[str]) -> dict[int, ReadingOptics]:
    by_reading: dict[int, ReadingOptics] = {}
    if not isinstance(raw, Mapping) or not raw:
        problems.append(f"{path} must be a non-empty mapping")
        return by_reading
    for key, row in raw.items():
        try:
            tenths = zoom_tenths(float(key)) if isinstance(key, str) else None
        except ValueError:
            tenths = None
        if tenths is None:
            problems.append(f"{path} key {key!r} is not a reading >= 0.1 on the 0.1 grid")
        elif tenths in by_reading:
            problems.append(f"{path} key {key!r} repeats reading {by_reading[tenths].reading}")
        else:
            optics = _parse_row(row, f"{tenths // 10}.{tenths % 10}", stream, f"{path}.{key}", problems)
            if optics is not None:
                by_reading[tenths] = optics
    return by_reading


def _parse_setting(raw: object) -> ZoomSetting | None:
    if raw == WIDE_STOP:
        return ZoomSetting.wide_stop()
    if isinstance(raw, Mapping) and set(raw) == {ABSOLUTE_ZOOM}:
        try:
            return ZoomSetting.absolute_zoom(raw[ABSOLUTE_ZOOM])
        except ValueError:
            return None
    return None


def _parse_row(row: object, reading: str, stream: StreamMode, path: str, problems: list[str]) -> ReadingOptics | None:
    if not _fields(row, _ROW_FIELDS, _ROW_FIELDS, path, problems):
        return None
    setting = _parse_setting(row["set_by"])
    fx, fy, cx, cy, skew = (_number(row[name]) for name in ("fx", "fy", "cx", "cy", "skew"))
    dist = row["dist"]
    start = len(problems)
    if setting is None:
        problems.append(f'{path} set_by must be "{WIDE_STOP}" or {{"{ABSOLUTE_ZOOM}": <command on the 0.1 grid>}}')
    if fx is None or fy is None or fx <= 0.0 or fy <= 0.0:
        problems.append(f"{path} fx and fy must be positive finite numbers")
    if cx is None or cy is None or not (0.0 <= cx <= stream.image_width and 0.0 <= cy <= stream.image_height):
        problems.append(f"{path} cx and cy must lie in the {stream.image_width}x{stream.image_height} frame")
    if skew is None:
        problems.append(f"{path} skew must be a finite number")
    if not (isinstance(dist, list) and len(dist) == 5 and all(_number(v) is not None for v in dist)):
        problems.append(f"{path} dist must be five finite numbers [k1, k2, p1, p2, k3]")
    if len(problems) > start:
        return None
    return ReadingOptics(reading, setting, fx, fy, cx, cy, skew, tuple(float(v) for v in dist))


__all__ = ["ABSOLUTE_ZOOM", "WIDE_STOP", "ReadingIntrinsicsTable", "ReadingOptics", "StreamMode", "ZoomSetting",
           "zoom_tenths"]
