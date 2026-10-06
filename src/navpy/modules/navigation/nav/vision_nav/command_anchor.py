"""Command anchor state, command clamping, and measurement gates."""

from __future__ import annotations

import math
from dataclasses import dataclass

from navpy.modules.navigation.nav.nav_law import NavCommand, NavCommandMode
from navpy.modules.navigation.nav.vision_nav.frame import FinalApproachVisionFrame
from navpy.modules.navigation.nav.vision_nav.law_config import FinalApproachLawConfig

# The command may not dive past this, whatever the loop asks for.  A converged
# course needs roughly -45 deg at worst; reaching -70 means the loop has already
# failed, and diving steeper destroys the horizontal control authority the
# aircraft needs to hold its ground track against wind.
PITCH_FLOOR_DEG = -70.0
# Flat anti-balloon ceiling.  It does not bind in nominal operation; it exists
# to structurally foreclose the recorded uav-123 climb-away geometry.  A ceiling
# referenced to the LOS elevation was tried instead and rejected: it degrades
# crosswind and can itself command a large nose-up through trajectory feedback.
PITCH_CEILING_DEG = 3.0
ROLL_LIMIT_CAP_DEG = 35.0

# One-interval jumps larger than these are treated as a measurement
# discontinuity rather than a real angular rate: hold the command and reseed the
# differentiators, so a single bad frame cannot inject a spurious rate.
OUTLIER_BEARING_DELTA_DEG = 30.0
OUTLIER_ELEVATION_DELTA_DEG = 15.0


@dataclass(frozen=True)
class FinalApproachCommandAnchor:
    """The command state the next cycle integrates from.

    Holding the previous *command* rather than the measured attitude is what
    makes the fixed point of the recursion the constant-bearing course itself: at zero
    LOS rate the increment is zero and the command stops changing by
    construction.  Anchoring on measured pitch instead makes the airframe's own
    response time part of the loop gain.
    """

    continuity_key: tuple[str, int, int, int]
    timestamp_s: float
    cmd_pitch_deg: float
    cmd_roll_deg: float
    bearing_rad: float
    elevation_rad: float


@dataclass(frozen=True)
class FinalApproachLimits:
    """A roll limit and a pitch range, kept as one thing.

    Used for two DIFFERENT sets that are not interchangeable: the limits the
    autopilot was configured with, and the ones a command is actually clipped
    to once `effective_limits` narrows them by the law's structural caps.
    Auditing against the configured 45 deg roll limit while the law clipped at
    35 produced a standing 10.0 deg "unexplained residual" on every run, which
    is exactly the false alarm that hides a real one.
    """

    roll_limit_deg: float
    pitch_min_deg: float
    pitch_max_deg: float


def effective_limits(
    roll_limit_deg: float,
    pitch_min_deg: float,
    pitch_max_deg: float,
) -> FinalApproachLimits:
    """Configured limits narrowed by the law's own structural caps.

    Takes plain floats rather than a config so the command path and the
    evidence builder can both reach it; the evidence builder holds the limits
    but not the config they came from.
    """
    return FinalApproachLimits(
        roll_limit_deg=min(roll_limit_deg, ROLL_LIMIT_CAP_DEG),
        pitch_min_deg=max(pitch_min_deg, PITCH_FLOOR_DEG),
        pitch_max_deg=min(pitch_max_deg, PITCH_CEILING_DEG),
    )


def clamped_command(
    config: FinalApproachLawConfig,
    raw_roll: float,
    raw_pitch: float,
) -> NavCommand:
    limits = effective_limits(
        config.roll_limit_deg, config.pitch_min_deg, config.pitch_max_deg)
    return NavCommand(
        mode=NavCommandMode.ACTUATOR,
        cmd_roll_deg=clip(
            raw_roll, -limits.roll_limit_deg, limits.roll_limit_deg),
        cmd_pitch_deg=clip(
            raw_pitch, limits.pitch_min_deg, limits.pitch_max_deg),
        cmd_thr=config.throttle,
    )


def clamped_anchor(
    config: FinalApproachLawConfig | None,
    anchor: FinalApproachCommandAnchor,
) -> FinalApproachCommandAnchor:
    """Force an anchor to hold only a command the limits would allow.

    No anchor may ever carry a value the autopilot could not have been sent,
    whichever path produced it -- otherwise the integrator starts from a
    command that was never issued.
    """
    if config is None:
        return anchor
    command = clamped_command(config, anchor.cmd_roll_deg, anchor.cmd_pitch_deg)
    return FinalApproachCommandAnchor(
        anchor.continuity_key,
        anchor.timestamp_s,
        command.cmd_pitch_deg,
        command.cmd_roll_deg,
        anchor.bearing_rad,
        anchor.elevation_rad,
    )


def bootstrap_anchor(frame: FinalApproachVisionFrame) -> FinalApproachCommandAnchor:
    return FinalApproachCommandAnchor(
        frame.continuity_key,
        frame.source_timestamp_s,
        frame.aircraft_pitch_deg,
        0.0,
        frame_bearing(frame),
        frame_elevation(frame),
    )


def advanced_anchor(
    anchor: FinalApproachCommandAnchor,
    frame: FinalApproachVisionFrame,
    bearing: float,
    elevation: float,
) -> FinalApproachCommandAnchor:
    return FinalApproachCommandAnchor(
        anchor.continuity_key,
        frame.source_timestamp_s,
        anchor.cmd_pitch_deg,
        anchor.cmd_roll_deg,
        bearing,
        elevation,
    )


def is_outlier(
    bearing: float,
    elevation: float,
    anchor: FinalApproachCommandAnchor,
) -> bool:
    bearing_delta = abs(
        math.atan2(
            math.sin(bearing - anchor.bearing_rad),
            math.cos(bearing - anchor.bearing_rad),
        )
    )
    elevation_delta = abs(elevation - anchor.elevation_rad)
    return bearing_delta > math.radians(
        OUTLIER_BEARING_DELTA_DEG
    ) or elevation_delta > math.radians(OUTLIER_ELEVATION_DELTA_DEG)


def frame_bearing(frame: FinalApproachVisionFrame) -> float:
    return math.atan2(frame.control_y, frame.control_x)


def frame_elevation(frame: FinalApproachVisionFrame) -> float:
    return math.atan2(
        frame.control_z,
        math.hypot(frame.control_x, frame.control_y),
    )


def clip(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


__all__ = [
    "OUTLIER_BEARING_DELTA_DEG",
    "OUTLIER_ELEVATION_DELTA_DEG",
    "FinalApproachLimits",
    "PITCH_CEILING_DEG",
    "PITCH_FLOOR_DEG",
    "ROLL_LIMIT_CAP_DEG",
    "effective_limits",
    "FinalApproachCommandAnchor",
    "advanced_anchor",
    "bootstrap_anchor",
    "clamped_anchor",
    "clamped_command",
    "clip",
    "frame_bearing",
    "frame_elevation",
    "is_outlier",
]
