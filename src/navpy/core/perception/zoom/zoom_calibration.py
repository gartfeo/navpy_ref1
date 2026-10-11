"""SIYI-style zoom calibration table.

The SIYI ZR10's commanded zoom and its readback zoom don't match: commanding
``set_zoom(2.0)`` produces readback 1.8; commanding 5.5 produces 5.5; etc.
The drift is deterministic (verified across 3 trials, spread = 0.000 on every
one of 91 measured commands at 0.1 resolution) but non-uniform, so stepping
the zoom tracker by commanded label leads to spurious no-ops and reversals.

This module exposes ``ZoomCalibrationTable`` which ``CameraMount`` uses to
reverse-map a live readback ``actual`` back to the commanded-float value
that produced it, so camera intrinsics can be interpolated in command
space (where the profile's fx/fy is linear by construction).

The table is loaded from the vision profile (``camera.zoom_calibration``)
by ``vision_profiles.build_zoom_calibration`` and wired into production
``CameraMount`` instances by ``build_camera_mounts``.

``scripts/python/zoom_calibrate.py`` is the probe used to generate new
tables when firmware changes or a new mount is added.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ZoomCalibrationEntry:
    commanded: str
    actual: float


@dataclass(frozen=True)
class ZoomCalibrationTable:
    entries: tuple[ZoomCalibrationEntry, ...]

    @classmethod
    def from_profile(cls, cal: dict | None) -> "ZoomCalibrationTable | None":
        """Parse a profile's commanded→actual mapping into a table.

        Returns ``None`` when the mapping is missing, empty, or yields fewer
        than 2 parseable entries (interpolation needs at least two points).
        Caller is expected to fall back to the raw-readback intrinsics path
        in that case.

        Tie-breaking: when multiple commands produce the same actual, the
        entries are ordered primarily by actual ascending and secondarily by
        numeric commanded ascending. ``readback_to_command`` relies on this
        order to pick the lowest-numeric-commanded canonical anchor per
        duplicate-actual group.
        """
        if not cal:
            return None

        parsed: list[ZoomCalibrationEntry] = []
        for cmd_str, actual in cal.items():
            try:
                cmd_float = float(cmd_str)
                actual_f = float(actual)
            except (TypeError, ValueError):
                continue
            if cmd_float <= 0 or actual_f <= 0:
                continue
            parsed.append(ZoomCalibrationEntry(commanded=str(cmd_str), actual=actual_f))

        if len(parsed) < 2:
            return None

        parsed.sort(key=lambda e: (e.actual, float(e.commanded)))
        return cls(entries=tuple(parsed))

    @property
    def actual_min(self) -> float:
        return self.entries[0].actual

    @property
    def actual_max(self) -> float:
        return self.entries[-1].actual

    def readback_to_command(self, actual: float) -> float | None:
        """Predict the commanded-float value whose calibrated actual equals
        ``actual`` via linear interpolation in command space.

        Used by the continuous-zoom path: the lens settles at some readback
        (e.g. 1.5) that may not match any calibrated ``actual``; we reverse-
        lookup the commanded-float value that produces that readback (e.g.
        1.7) so downstream intrinsics can interpolate fx/fy in command
        space — where the existing profile is linear by construction.

        Duplicate-actual groups (e.g. cmd 1.0/1.1/1.2/1.3 all produce actual
        1.1) are collapsed to the LOWEST-numeric-commanded canonical anchor
        before interpolation: duplicate x-values would otherwise make the
        bracketing pair degenerate. For exact-match inputs, returns the
        canonical anchor's cmd. Inputs below/above the measured actual
        range clamp to the nearest end's cmd.

        Returns ``None`` if the table is empty.
        """
        if not self.entries:
            return None

        # Build canonical (actual, cmd_float) list: one entry per unique
        # actual, taking the lowest-numeric command. self.entries is already
        # sorted by (actual, commanded_float) so we just take the first
        # occurrence of each actual.
        canonical: list[tuple[float, float]] = []
        seen: set[float] = set()
        for entry in self.entries:
            if entry.actual in seen:
                continue
            seen.add(entry.actual)
            canonical.append((entry.actual, float(entry.commanded)))

        if not canonical:
            return None

        # Clamp outside the measured range.
        if actual <= canonical[0][0]:
            return canonical[0][1]
        if actual >= canonical[-1][0]:
            return canonical[-1][1]

        # Find bracketing pair and linearly interpolate cmd.
        for i in range(len(canonical) - 1):
            a0, c0 = canonical[i]
            a1, c1 = canonical[i + 1]
            if a0 <= actual <= a1:
                if a1 == a0:
                    return c0  # defensive (shouldn't happen post-dedup)
                t = (actual - a0) / (a1 - a0)
                return c0 + t * (c1 - c0)

        return None  # unreachable given sorted canonical + clamps
