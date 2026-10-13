"""Plan/issue/commit final-approach command transaction."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Protocol

from navpy.core.approach.calc_data import CalcData
from navpy.core.ports.frame import FinalApproachVisionFrame
from navpy.plugins.laws.vision_pn.command_anchor import (
    FinalApproachLimits,
    effective_limits,
)
from navpy.plugins.laws.vision_pn.law_plan import FinalApproachLawPlan
from navpy.plugins.laws.vision_pn.law_plan import FinalApproachPlanOrigin
from navpy.core.approach.vision_nav.visual_pass import VisualPassPlan


class FinalApproachAttitudeActuator(Protocol):
    def issue(
        self,
        roll_deg: float,
        pitch_deg: float,
        throttle: float | None,
    ) -> None: ...


class FinalApproachCommandLaw(Protocol):
    @property
    def pitch_time_constant_s(self) -> float | None: ...

    @property
    def roll_limit_deg(self) -> float: ...

    @property
    def pitch_limits_deg(self) -> tuple[float, float]: ...

    def plan(self, frame: FinalApproachVisionFrame) -> FinalApproachLawPlan | None: ...

    def commit(self, plan: FinalApproachLawPlan) -> None: ...


class FinalApproachPassPolicy(Protocol):
    def plan(self, frame: FinalApproachVisionFrame) -> VisualPassPlan: ...

    def commit(self, plan: VisualPassPlan) -> None: ...


class FinalApproachEpisodeOutcome(Protocol):
    @property
    def commands_ended(self) -> bool: ...

    def latch_missed(self) -> bool: ...


@dataclass(frozen=True)
class FinalApproachCommandResult:
    calc_data: CalcData | None
    outcome: "FinalApproachCommandOutcome"
    missed: bool
    evidence: "FinalApproachLawEvidence | None" = None

    @property
    def issued(self) -> bool:
        return self.outcome is FinalApproachCommandOutcome.ISSUED


class FinalApproachCommandOutcome(Enum):
    ISSUED = "issued"
    # An armed dock seen aft: no command; the second aft frame is the miss.
    AFT_SUPPRESSED = "aft_suppressed"
    LAW_UNAVAILABLE = "law_unavailable"
    # The episode already ended (docked or missed): no command, no row.
    TERMINAL_SUPPRESSED = "terminal_suppressed"


@dataclass(frozen=True)
class FinalApproachLawEvidence:
    """Frame-local terms that completely explain one final-approach command."""

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
    configured_limits: FinalApproachLimits
    effective_limits: FinalApproachLimits
    raw_roll_deg: float
    raw_pitch_deg: float
    # Which branch ran, the anchor it integrated from, the interval and the
    # gains -- everything needed to reproduce the command.
    origin: FinalApproachPlanOrigin


class FinalApproachCommandTransaction:
    """Mutate law/aft-gate state only at the command boundary.

    Runs under the runtime lock (slot lease), so it serializes with the
    contact latch: nothing is actuated once the episode docked or missed.
    """

    def __init__(
        self,
        law: FinalApproachCommandLaw,
        visual_pass: FinalApproachPassPolicy,
        actuator: FinalApproachAttitudeActuator,
        episode: FinalApproachEpisodeOutcome,
    ) -> None:
        self._law = law
        self._pass = visual_pass
        self._actuator = actuator
        self._episode = episode

    def terminal_result(self) -> FinalApproachCommandResult | None:
        """The suppression result once the episode ended, before any check.

        The executor asks this before freshness, so a stale frame after
        docking neither latches a liveness failure nor writes a command row.
        """
        if not self._episode.commands_ended:
            return None
        return FinalApproachCommandResult(
            None,
            FinalApproachCommandOutcome.TERMINAL_SUPPRESSED,
            False,
        )

    def execute(self, frame: FinalApproachVisionFrame) -> FinalApproachCommandResult:
        terminal = self.terminal_result()
        if terminal is not None:
            return terminal
        pass_plan = self._pass.plan(frame)
        if pass_plan.suppress_command:
            self._pass.commit(pass_plan)
            missed = pass_plan.next_state.missed and self._episode.latch_missed()
            return FinalApproachCommandResult(
                None,
                FinalApproachCommandOutcome.AFT_SUPPRESSED,
                missed,
            )
        law_plan = self._law.plan(frame)
        if law_plan is None:
            return FinalApproachCommandResult(
                None,
                FinalApproachCommandOutcome.LAW_UNAVAILABLE,
                False,
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
        return FinalApproachCommandResult(
            CalcData(
                yaw=math.degrees(math.atan2(frame.body_y, frame.body_x)),
                pitch=control_elevation_deg,
                cmd_roll=command.cmd_roll_deg,
                cmd_pitch=command.cmd_pitch_deg,
                cmd_thr=command.cmd_thr,
            ),
            FinalApproachCommandOutcome.ISSUED,
            False,
            FinalApproachLawEvidence(
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
                configured_limits=FinalApproachLimits(
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
    "FinalApproachAttitudeActuator",
    "FinalApproachCommandLaw",
    "FinalApproachCommandOutcome",
    "FinalApproachCommandResult",
    "FinalApproachCommandTransaction",
    "FinalApproachEpisodeOutcome",
    "FinalApproachLawEvidence",
    "FinalApproachPassPolicy",
]
