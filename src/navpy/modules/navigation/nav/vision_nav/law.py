"""Minimal frame-local final approach law."""

from __future__ import annotations

import math

from navpy.modules.navigation.nav.vision_nav.command_anchor import (
    OUTLIER_BEARING_DELTA_DEG,
    OUTLIER_ELEVATION_DELTA_DEG,
    PITCH_CEILING_DEG,
    PITCH_FLOOR_DEG,
    ROLL_LIMIT_CAP_DEG,
    TerminalCommandAnchor,
    advanced_anchor,
    bootstrap_anchor,
    clamped_anchor,
    clamped_command,
    frame_bearing,
    frame_elevation,
    is_outlier,
)
from navpy.modules.navigation.nav.vision_nav.frame import TerminalVisionFrame
from navpy.modules.navigation.nav.vision_nav.law_config import (
    FixedTerminalLawConfigProvider,
    TerminalLawConfig,
    TerminalLawConfigProvider,
)
from navpy.modules.navigation.nav.vision_nav.law_plan import (
    LATERAL_PN_NAVIGATION_CONSTANT,
    TerminalLawPlan,
    TerminalPlanOrigin,
    VERTICAL_PN_NAVIGATION_CONSTANT,
    held_plan,
    replace_anchor,
)
from navpy.modules.navigation.nav.vision_nav.rate_filter import VerticalRateFilter
from navpy.modules.navigation.nav.vision_nav.lateral_rate import LateralRateFilter
from navpy.modules.navigation.nav.vision_nav.pitch_law import raw_pitch_current
from navpy.utils.math_utils import GRAVITY_MSS

# Own named constant.  This used to be `PTCH2SRV_TCONST`, which the law read for
# two unrelated jobs at once: the rate-filter time constant AND the rate-to-angle
# conversion.  One autopilot parameter silently setting two things is a defect;
# the vertical command no longer reads PTCH2SRV_TCONST at all.  The config field
# survives for diagnostics only.
VERTICAL_RATE_FILTER_TAU_S = 0.10
# The lateral rate feeds a PROPORTIONAL roll command, so an unfiltered two-point
# bearing derivative reaches the bank whole, times N*V/g (~10).
#
# CURRENTLY OFF (None = raw passthrough).  Owner decision 2026-08-20: measured
# noise-free, the filter cuts NO dither at all (mean change per cycle 0.1308 OFF
# vs 0.1302 at tau 0.30), and only softens the lock-on step while still lagging
# a changing LOS rate.  It is a noise tool, so it belongs on once the LOS-rate noise
# SPECTRUM is measured and a tau is fitted against it -- 0.30 never was.  The
# machinery and its tests stay; this is one line to re-enable.
#
# Behaviour pinned both ways: tests/navigation/test_lateral_rate_filter.py.
LATERAL_RATE_FILTER_TAU_S: float | None = None

# Near the control-frame vertical the bearing parameterisation is
# ill-conditioned, and `pixel_observation.py` divides the measured yaw rate by
# cos(pitch) (7.2x amplification at -82 deg).  Inside either guard the lateral
# command holds instead of acting on a number that is mostly conditioning error.
STEEP_LOS_HORIZONTAL_MIN = math.sin(math.radians(20.0))
STEEP_PITCH_DEG = 60.0


class VisionNavLaw:
    """Plan and commit a two-axis visual command without geo state."""

    def __init__(
        self,
        config_provider: TerminalLawConfigProvider,
    ) -> None:
        self._config_provider = config_provider
        self._config = config_provider.read()
        self._rates = VerticalRateFilter()
        self._lateral_rates = LateralRateFilter()
        self._anchor: TerminalCommandAnchor | None = None

    def reset(self) -> None:
        self._config = self._config_provider.read()
        self._rates.reset()
        self._lateral_rates.reset()
        self._anchor = None

    @property
    def available(self) -> bool:
        return self._config is not None

    @property
    def pitch_time_constant_s(self) -> float | None:
        """Diagnostics only. The command path no longer reads this."""
        return None if self._config is None else self._config.pitch_time_constant_s

    @property
    def roll_limit_deg(self) -> float:
        return 0.0 if self._config is None else self._config.roll_limit_deg

    @property
    def pitch_limits_deg(self) -> tuple[float, float]:
        if self._config is None:
            return 0.0, 0.0
        return self._config.pitch_min_deg, self._config.pitch_max_deg

    def seed(self, frame: TerminalVisionFrame, roll_deg: float = 0.0) -> None:
        self._rates.seed(frame)
        self._lateral_rates.seed(frame)
        # `confirmation.py:47-56` seeds WITHOUT issuing a command, so this
        # anchor is what the first real command integrates from.  It must
        # therefore already be clamped: an aircraft seeded at -80 deg pitch
        # against a -55 deg limit would otherwise have its first command
        # integrate from an attitude the autopilot would never have been given.
        #
        # WHICH command the anchor holds is the pitch law's business, not this
        # method's: `bootstrap_anchor` hands over measured aircraft pitch, and
        # the handoff-isolation arm overrides it. Only the `seed` path goes
        # through here -- the bootstrap inside `_plan` is deliberately left
        # alone, because `confirmation.py` previews frames through it before
        # `seed` ever runs, and changing that would move WHETHER a run starts navigation
        # instead of only what it integrates from.
        self._anchor = clamped_anchor(self._config, bootstrap_anchor(frame))
        # Aircraft roll is not command state. ArduPilot owns attitude response.
        del roll_deg

    def preview(self, frame: TerminalVisionFrame) -> TerminalLawPlan | None:
        return self._plan(frame)

    def plan(self, frame: TerminalVisionFrame) -> TerminalLawPlan | None:
        return self._plan(frame)

    def _plan(self, frame: TerminalVisionFrame) -> TerminalLawPlan | None:
        """Compute a command without mutating any state.

        `preview` and `plan` share this body deliberately.  `confirmation.py`
        previews a frame to decide whether it is flyable *before* `seed` has
        ever run, so this must produce a command with no prior command state,
        and must not advance the anchor on that path -- otherwise a rejected
        confirmation would silently integrate.
        """
        config = self._config
        if config is None:
            return None
        rate = self._rates.plan(frame, VERTICAL_RATE_FILTER_TAU_S)
        lateral_rate = self._lateral_rates.plan(frame, LATERAL_RATE_FILTER_TAU_S)
        anchor = self._usable_anchor(frame)
        bearing = frame_bearing(frame)
        elevation = frame_elevation(frame)

        if anchor is None:
            # Bootstrap: no previous command exists (first frame after reset,
            # after `command_reset` cleared held output and law state, or on the
            # preview-before-seed path).  Re-anchor to measured attitude and
            # command a zero increment this cycle.
            return held_plan(
                frame,
                config,
                rate,
                lateral_rate,
                bootstrap_anchor(frame),
                bootstrapped=True,
                reason="bootstrap",
            )

        dt_s = frame.source_timestamp_s - anchor.timestamp_s
        if dt_s <= 0.0:
            return held_plan(
                frame, config, rate, lateral_rate, anchor,
                reason="dt_regressed",
            )
        if is_outlier(bearing, elevation, anchor):
            held = held_plan(
                frame, config, rate, lateral_rate, anchor, reason="outlier",
            )
            # Advance the differentiator reference past the discontinuity, but
            # carry the HELD PLAN's anchor, which already holds the clamped
            # command -- not the raw `anchor`, which may be unclamped.
            return replace_anchor(
                held,
                advanced_anchor(held.next_anchor, frame, bearing, elevation),
                reseed=True,
            )

        raw_pitch = raw_pitch_current(
            anchor.cmd_pitch_deg,
            VERTICAL_PN_NAVIGATION_CONSTANT,
            rate.rate_rad_s,
            dt_s,
        )
        lateral_held = _lateral_is_ill_conditioned(frame)
        if lateral_held:
            raw_roll = anchor.cmd_roll_deg
        else:
            raw_roll = math.degrees(
                math.atan(
                    LATERAL_PN_NAVIGATION_CONSTANT
                    * frame.air_speed_mps
                    * lateral_rate.rate_rad_s
                    / GRAVITY_MSS
                )
            )
        command = clamped_command(config, raw_roll, raw_pitch)
        return TerminalLawPlan(
            command,
            raw_roll,
            raw_pitch,
            rate,
            lateral_rate,
            TerminalCommandAnchor(
                frame.continuity_key,
                frame.source_timestamp_s,
                # Conditional integration: the CLAMPED command becomes the next
                # anchor, so a saturated cycle cannot wind the integrator up.
                # The integrator state and the output are the same variable, so
                # no separate back-calculation term is needed.
                command.cmd_pitch_deg,
                command.cmd_roll_deg,
                bearing,
                elevation,
            ),
            frame,
            lateral_held=lateral_held,
            # A roll-only hold is NOT the same event as a whole-command hold.
            origin=TerminalPlanOrigin(
                reason="lateral_held" if lateral_held else "normal",
                anchor_cmd_roll_deg=anchor.cmd_roll_deg,
                anchor_cmd_pitch_deg=anchor.cmd_pitch_deg,
                dt_s=dt_s,
            ),
        )

    def _usable_anchor(
        self,
        frame: TerminalVisionFrame,
    ) -> TerminalCommandAnchor | None:
        anchor = self._anchor
        if anchor is None or anchor.continuity_key != frame.continuity_key:
            return None
        return anchor

    def commit(self, plan: TerminalLawPlan) -> None:
        if plan.reseed:
            self._rates.seed(plan.frame)
            self._lateral_rates.seed(plan.frame)
        else:
            self._rates.commit(plan.rate)
            self._lateral_rates.commit(plan.lateral_rate)
        self._anchor = plan.next_anchor


def _lateral_is_ill_conditioned(frame: TerminalVisionFrame) -> bool:
    horizontal_norm = math.hypot(frame.control_x, frame.control_y)
    return (
        horizontal_norm < STEEP_LOS_HORIZONTAL_MIN
        or abs(frame.aircraft_pitch_deg) > STEEP_PITCH_DEG
    )


__all__ = [
    "FixedTerminalLawConfigProvider",
    "OUTLIER_BEARING_DELTA_DEG",
    "OUTLIER_ELEVATION_DELTA_DEG",
    "PITCH_CEILING_DEG",
    "PITCH_FLOOR_DEG",
    "ROLL_LIMIT_CAP_DEG",
    "STEEP_LOS_HORIZONTAL_MIN",
    "STEEP_PITCH_DEG",
    "TerminalCommandAnchor",
    "TerminalLawConfig",
    "TerminalLawConfigProvider",
    "TerminalLawPlan",
    "VERTICAL_PN_NAVIGATION_CONSTANT",
    "VERTICAL_RATE_FILTER_TAU_S",
    "LATERAL_PN_NAVIGATION_CONSTANT",
    "VisionNavLaw",
]
