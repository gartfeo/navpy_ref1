"""ZR10 emulator settings: ideal defaults, each value with its source (hitl_final_approach.md section 4).

Runs start ideal (no dead time, no lag, scale 1, instant 0x0F) and add one effect at a time, so a failure is
attributed. A setting that differs from its ideal value is listed by ``non_ideal`` and written to the run's log
header; runs with unmeasured values are exploratory and labelled so by whoever reports them.
"""
from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import asdict, dataclass, fields

STEP_S = 0.02     # the plant step, as navpy's sim (gimbal_siyi_step.py SIYI_PHYSICS_STEP_S)


@dataclass(frozen=True)
class EmulatorConfig:
    slew_rate_deg_s: float = 90.0
    yaw_min_deg: float = -135.0
    yaw_max_deg: float = 132.8
    pitch_min_deg: float = -88.9
    pitch_max_deg: float = 23.9
    rate_dead_time_s: float = 0.0
    rate_deadband: int = 0
    attitude_report_lag_s: float = 0.0
    zoom_report_lag_s: float = 0.0
    yaw_report_scale: float = 1.0
    pitch_report_scale: float = 1.0
    absolute_zoom_speed_per_s: float = math.inf
    manual_zoom_speed_per_s: float = 0.5

    def __post_init__(self) -> None:
        for item in fields(self):
            value = getattr(self, item.name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or math.isnan(value):
                raise ValueError(f"{item.name} must be a number")
        if not (self.yaw_min_deg < 0.0 < self.yaw_max_deg and self.pitch_min_deg < 0.0 < self.pitch_max_deg):
            raise ValueError("limits must contain 0 (the centre)")
        if self.rate_deadband < 0 or self.rate_deadband >= 100 or int(self.rate_deadband) != self.rate_deadband:
            raise ValueError("rate_deadband must be a whole number in 0..99")
        for name in ("slew_rate_deg_s", "yaw_report_scale", "pitch_report_scale", "absolute_zoom_speed_per_s",
                     "manual_zoom_speed_per_s"):
            if not getattr(self, name) > 0.0:
                raise ValueError(f"{name} must be positive")
        for name in ("rate_dead_time_s", "attitude_report_lag_s", "zoom_report_lag_s"):
            value = getattr(self, name)
            if not (0.0 <= value < 10.0):
                raise ValueError(f"{name} must be in 0..10 s")

    def non_ideal(self) -> dict[str, float]:
        """The effects switched on: every value that differs from the ideal run's."""
        return {name: value for name, value in asdict(self).items() if value != IDEAL[name]}


# The ideal run. Slew rate, limits and the manual zoom speed have no "off" value; they are the hardware's.
IDEAL: Mapping[str, float] = asdict(EmulatorConfig())

SOURCES: Mapping[str, str] = {
    "slew_rate_deg_s": "unmeasured: navpy sim plant gimbal_angles.py MAX_SLEW_RATE; aas-hitl uses 145 (TBC)",
    "yaw_min_deg": "rig limit sweep 2026-10-14 (yaw -135.0..+132.8 deg, body-relative)",
    "yaw_max_deg": "rig limit sweep 2026-10-14 (yaw -135.0..+132.8 deg, body-relative)",
    "pitch_min_deg": "rig limit sweep 2026-10-14 (pitch -88.9..+23.9 deg, body-relative)",
    "pitch_max_deg": "rig limit sweep 2026-10-14 (pitch -88.9..+23.9 deg, body-relative)",
    "rate_dead_time_s": "ideal 0; aas-hitl 0.6 s (TBC, zr10.example.toml rate_dead_time_s); rig stop latency "
                        "8.2 ms p99.9 is the send path only",
    "rate_deadband": "ideal 0; aas-hitl bench observation: speeds up to 4 give no motion (zr10.example.toml:26)",
    "attitude_report_lag_s": "ideal 0; unmeasured (aas-hitl's 0.145 s is command-to-motion onset, not "
                             "telemetry lag)",
    "zoom_report_lag_s": "ideal 0; unmeasured",
    "yaw_report_scale": "ideal 1; rig step test 2026-10-08: yaw report 0.92-0.94 x true",
    "pitch_report_scale": "ideal 1; rig step test 2026-10-08: pitch report 0.995 x true",
    "absolute_zoom_speed_per_s": "ideal inf (0x0F jumps); rig sweep 2026-10-08: 2-6 s lens travel, speed over "
                                 "the range unmeasured",
    "manual_zoom_speed_per_s": "aas-hitl zr10.example.toml:34, measured near 1x only; navpy's sim 5 levels/s "
                               "is not used",
}


def zoom_table_from_profile(profile: Mapping[str, object]) -> dict[int, float]:
    """The profile's ``camera.zoom_calibration`` as {0x0F command in tenths: 0x18 reading}.

    One gimbal device with a table is required: the emulator never makes up a command-to-reading map.
    """
    devices = profile.get("devices")
    tables = []
    for device in devices if isinstance(devices, list) else []:
        camera = device.get("camera") if isinstance(device, Mapping) else None
        if isinstance(camera, Mapping) and isinstance(camera.get("zoom_calibration"), Mapping):
            tables.append(camera["zoom_calibration"])
    if len(tables) != 1:
        raise ValueError(f"the profile needs exactly one camera.zoom_calibration table, found {len(tables)}")
    table: dict[int, float] = {}
    for command, reading in tables[0].items():
        tenths = round(float(command) * 10.0)
        value = float(reading)
        if abs(tenths / 10.0 - float(command)) > 1e-9 or not math.isfinite(value) or value < 1.0:
            raise ValueError(f"zoom_calibration row {command!r}: {reading!r} is not a 0.1-step command and reading")
        if abs(round(value * 10.0) / 10.0 - value) > 1e-9:
            raise ValueError(f"zoom_calibration reading {reading!r} is not a multiple of 0.1")
        table[tenths] = round(value * 10.0) / 10.0
    if len(table) < 2:
        raise ValueError("zoom_calibration needs at least two rows")
    return table


def load_zoom_table(profile_name: str) -> dict[int, float]:
    from navpy.core.perception.profile.vision_profile_loader import load_profiles

    profiles, _default, path = load_profiles()
    if profile_name not in profiles:
        raise ValueError(f"vision profile {profile_name!r} not found in {path}")
    return zoom_table_from_profile(profiles[profile_name])


__all__ = ["EmulatorConfig", "IDEAL", "SOURCES", "STEP_S", "load_zoom_table", "zoom_table_from_profile"]
