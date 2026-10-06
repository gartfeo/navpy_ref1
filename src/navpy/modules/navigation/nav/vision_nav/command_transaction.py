"""Plan/issue/commit terminal command transaction."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Protocol

from navpy.modules.navigation.calc_data import CalcData
from navpy.modules.navigation.nav.vision_nav.frame import TerminalVisionFrame
from navpy.modules.navigation.nav.vision_nav.command_anchor import (
    TerminalLimits,
    effective_limits,
)
from navpy.modules.navigation.nav.vision_nav.law import TerminalLawPlan
from navpy.modules.navigation.nav.vision_nav.law_plan import TerminalPlanOrigin
from navpy.modules.navigation.nav.vision_nav.visual_pass import VisualPassPlan


class TerminalAttitudeActuator(Protocol):
    def issue(
        self,
        roll_deg: float,
        pitch_deg: float,
        throttle: float | None,
    ) -> None: ...


class TerminalCommandLaw(Protocol):
    @property
    def pitch_time_constant_s(self) -> float | None: ...

    @property
    def roll_limit_deg(self) -> float: ...

    @property
    def pitch_limits_deg(self) -> tuple[float, float]: ...

    def plan(self, frame: TerminalVisionFrame) -> TerminalLawPlan | None: ...

    def commit(self, plan: TerminalLawPlan) -> None: ...


class TerminalPassPolicy(Protocol):
    def plan(self, frame: TerminalVisionFrame) -> VisualPassPlan: ...

    def commit(self, plan: VisualPassPlan) -> None: ...


@dataclass(frozen=True)
class TerminalCommandResult:
    calc_data: CalcData | None
    outcome: "TerminalCommandOutcome"
    passed: bool
    evidence: "TerminalLawEvidence | None" = None

    @property
    def issued(self) -> bool:
        return self.outcome is TerminalCommandOutcome.ISSUED


class TerminalCommandOutcome(Enum):
    ISSUED = "issued"
    PASS_SUPPRESSED = "pass_suppressed"
    LAW_UNAVAILABLE = "law_unavailable"


@dataclass(frozen=True)
class TerminalLawEvidence:
    """Frame-local terms that completely explain one terminal command."""

    control_bearing_deg: float
    lateral_rate_deg_s: float
    aircraft_turn_rate_deg_s: float
    raw_inertial_los_rate_deg_s: float
    inertial_los_rate_deg_s: float
    aircraft_roll_deg: float
    aircraft_pitch_deg: float
    air_speed_mps: float
    control_elevation_deg: float
    vertical_rate_deg_s: float
    pitch_time_constant_s: float | None
    # What the autopilot was configured with, and what the command was ACTUALLY
    # clipped to after the law's own structural caps narrowed it. Both, because
    # they differ: auditing against the configured values produced a standing
    # 10.0 deg "unexplained residual" on every run.
    configured_limits: TerminalLimits
    effective_limits: TerminalLimits
    raw_roll_deg: float
    raw_pitch_deg: float
    # Which branch ran, the anchor it integrated from, the interval and the
    # gains -- everything needed to reproduce the command.
    origin: TerminalPlanOrigin


class TerminalCommandTransaction:
    """Mutate law/pass state only at the command boundary."""

    def __init__(
        self,
        law: TerminalCommandLaw,
        visual_pass: TerminalPassPolicy,
        actuator: TerminalAttitudeActuator,
    ) -> None:
        self._law = law
        self._pass = visual_pass
        self._actuator = actuator

    def execute(self, frame: TerminalVisionFrame) -> TerminalCommandResult:
        pass_plan = self._pass.plan(frame)
        if pass_plan.suppress_command:
            self._pass.commit(pass_plan)
            return TerminalCommandResult(
                None,
                TerminalCommandOutcome.PASS_SUPPRESSED,
                pass_plan.next_state.passed,
            )
        law_plan = self._law.plan(frame)
        if law_plan is None:
            return TerminalCommandResult(
                None,
                TerminalCommandOutcome.LAW_UNAVAILABLE,
                pass_plan.next_state.passed,
            )
        command = law_plan.command
        self._actuator.issue(
            command.cmd_roll_deg,
            command.cmd_pitch_deg,
            command.cmd_thr,
        )
        self._law.commit(law_plan)
        self._pass.commit(pass_plan)
        limits = effective_limits(
            self._law.roll_limit_deg,
            self._law.pitch_limits_deg[0],
            self._law.pitch_limits_deg[1],
        )
        control_bearing_deg = math.degrees(
            math.atan2(frame.control_y, frame.control_x)
        )
        control_elevation_deg = math.degrees(
            math.atan2(
                frame.control_z,
                math.hypot(frame.control_x, frame.control_y),
            )
        )
        return TerminalCommandResult(
            CalcData(
                yaw=math.degrees(math.atan2(frame.body_y, frame.body_x)),
                pitch=control_elevation_deg,
                cmd_roll=command.cmd_roll_deg,
                cmd_pitch=command.cmd_pitch_deg,
                cmd_thr=command.cmd_thr,
            ),
            TerminalCommandOutcome.ISSUED,
            pass_plan.next_state.passed,
            TerminalLawEvidence(
                control_bearing_deg=control_bearing_deg,
                lateral_rate_deg_s=math.degrees(
                    law_plan.lateral_rate.visual_rate_rad_s
                ),
                aircraft_turn_rate_deg_s=math.degrees(
                    law_plan.lateral_rate.raw_inertial_rate_rad_s
                    - law_plan.lateral_rate.visual_rate_rad_s
                ),
                raw_inertial_los_rate_deg_s=math.degrees(
                    law_plan.lateral_rate.raw_inertial_rate_rad_s
                ),
                inertial_los_rate_deg_s=math.degrees(
                    law_plan.lateral_rate.rate_rad_s
                ),
                aircraft_roll_deg=frame.aircraft_roll_deg,
                aircraft_pitch_deg=frame.aircraft_pitch_deg,
                air_speed_mps=frame.air_speed_mps,
                control_elevation_deg=control_elevation_deg,
                vertical_rate_deg_s=math.degrees(law_plan.rate.rate_rad_s),
                pitch_time_constant_s=self._law.pitch_time_constant_s,
                configured_limits=TerminalLimits(
                    self._law.roll_limit_deg,
                    self._law.pitch_limits_deg[0],
                    self._law.pitch_limits_deg[1],
                ),
                effective_limits=limits,
                raw_roll_deg=law_plan.raw_roll_deg,
                raw_pitch_deg=law_plan.raw_pitch_deg,
                origin=law_plan.origin,
            ),
        )


__all__ = [
    "TerminalAttitudeActuator",
    "TerminalCommandLaw",
    "TerminalCommandOutcome",
    "TerminalCommandResult",
    "TerminalCommandTransaction",
    "TerminalLawEvidence",
    "TerminalPassPolicy",
]
