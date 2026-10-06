"""Final-approach law plan record and plan-construction helpers."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.modules.navigation.nav.nav_law import NavCommand
from navpy.modules.navigation.nav.vision_nav.command_anchor import (
    FinalApproachCommandAnchor,
    clamped_command,
)
from navpy.modules.navigation.nav.vision_nav.frame import FinalApproachVisionFrame
from navpy.modules.navigation.nav.vision_nav.law_config import FinalApproachLawConfig
from navpy.modules.navigation.nav.vision_nav.lateral_rate import LateralRatePlan
from navpy.modules.navigation.nav.vision_nav.rate_filter import VerticalRatePlan


# The proportional-navigation gains. They live here, beside the plan that
# records them, so there is exactly one definition: a second copy kept for the
# record could drift from the one the law applies, which is how the previous
# command audit went wrong.
VERTICAL_PN_NAVIGATION_CONSTANT = 4.0
LATERAL_PN_NAVIGATION_CONSTANT = 4.0


@dataclass(frozen=True)
class FinalApproachPlanOrigin:
    """How one cycle formed its command.

    Kept together rather than spread across the plan because they are only
    ever meaningful as a set: a reason without the anchor it held cannot be
    checked, and an anchor without the gains and interval applied to it cannot
    reproduce the command.
    """

    # Which branch ran. `bootstrapped`/`reseed` on the plan cannot separate a
    # regressed-timestamp hold from an outlier hold, and neither says whether a
    # normal cycle held only its roll. One of: normal, bootstrap, dt_regressed,
    # outlier, held, lateral_held.
    reason: str = "normal"
    # The command this cycle integrated FROM, and over what interval. The law
    # is `anchor + increment(rate, dt)`, so without these the record does not
    # contain the terms needed to reproduce its own output.
    anchor_cmd_roll_deg: float | None = None
    anchor_cmd_pitch_deg: float | None = None
    dt_s: float | None = None
    # The gains actually applied, so anything reconstructing the command reads
    # what ran rather than assuming a value.
    lateral_nav_constant: float = LATERAL_PN_NAVIGATION_CONSTANT
    vertical_nav_constant: float = VERTICAL_PN_NAVIGATION_CONSTANT


@dataclass(frozen=True)
class FinalApproachLawPlan:
    command: NavCommand
    raw_roll_deg: float
    raw_pitch_deg: float
    rate: VerticalRatePlan
    lateral_rate: LateralRatePlan
    next_anchor: FinalApproachCommandAnchor
    frame: FinalApproachVisionFrame
    bootstrapped: bool = False
    lateral_held: bool = False
    reseed: bool = False
    origin: FinalApproachPlanOrigin = FinalApproachPlanOrigin()

    @property
    def within_limits(self) -> bool:
        return (
            self.command.cmd_roll_deg == self.raw_roll_deg
            and self.command.cmd_pitch_deg == self.raw_pitch_deg
        )


def held_plan(
    frame: FinalApproachVisionFrame,
    config: FinalApproachLawConfig,
    rate: VerticalRatePlan,
    lateral_rate: LateralRatePlan,
    anchor: FinalApproachCommandAnchor,
    *,
    bootstrapped: bool = False,
    reason: str = "held",
) -> FinalApproachLawPlan:
    """Reissue the anchor's command, clipped by the same limits.

    The returned anchor carries the CLAMPED command, never the raw value.
    Bootstrapping from measured attitude can land outside the limits -- an
    aircraft at -80 deg pitch against a -55 deg limit would otherwise issue
    -55 and then integrate the next cycle from -80, so the loop would not be
    starting from the command it actually gave.  Conditional integration has to
    hold on this path too, not just on the normal one.
    """
    raw_pitch = anchor.cmd_pitch_deg
    raw_roll = anchor.cmd_roll_deg
    command = clamped_command(config, raw_roll, raw_pitch)
    return FinalApproachLawPlan(
        command,
        raw_roll_deg=raw_roll,
        raw_pitch_deg=raw_pitch,
        rate=rate,
        lateral_rate=lateral_rate,
        next_anchor=FinalApproachCommandAnchor(
            anchor.continuity_key,
            anchor.timestamp_s,
            command.cmd_pitch_deg,
            command.cmd_roll_deg,
            anchor.bearing_rad,
            anchor.elevation_rad,
        ),
        frame=frame,
        bootstrapped=bootstrapped,
        lateral_held=True,
        origin=FinalApproachPlanOrigin(
            reason=reason,
            anchor_cmd_roll_deg=anchor.cmd_roll_deg,
            anchor_cmd_pitch_deg=anchor.cmd_pitch_deg,
        ),
    )


def replace_anchor(
    plan: FinalApproachLawPlan,
    anchor: FinalApproachCommandAnchor,
    *,
    reseed: bool,
) -> FinalApproachLawPlan:
    return FinalApproachLawPlan(
        plan.command,
        plan.raw_roll_deg,
        plan.raw_pitch_deg,
        plan.rate,
        plan.lateral_rate,
        anchor,
        plan.frame,
        bootstrapped=plan.bootstrapped,
        lateral_held=plan.lateral_held,
        reseed=reseed,
        # Carried, not defaulted: this is the outlier path, and dropping the
        # origin here would relabel an outlier hold as a normal integration.
        origin=plan.origin,
    )


__all__ = [
    "LATERAL_PN_NAVIGATION_CONSTANT",
    "FinalApproachLawPlan",
    "FinalApproachPlanOrigin",
    "VERTICAL_PN_NAVIGATION_CONSTANT",
    "held_plan",
    "replace_anchor",
]
