"""NavLaw ABC, NavContext, and NavCommand contract.

Introduced in Step 1 of the observable-only navigation redesign. Existing
RollL1PitchNav is wrapped as a legacy actuator-form NavLaw in
roll_l1_pitch_nav_law.py without behavior change. Later steps add
acceleration-form laws (Kim-Grider, APN) that emit NavCommand.ACCELERATION
and rely on a NavAllocator for actuator conversion.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np

from navpy.modules.common.models.location import Location


class NavCommandMode(Enum):
    """Representation mode of a NavCommand.

    ACTUATOR and ACCELERATION are mutually exclusive; INVALID carries only a reason.
    """

    ACTUATOR = "ACTUATOR"
    ACCELERATION = "ACCELERATION"
    INVALID = "INVALID"


class NavInvalidReason(Enum):
    """Enumerated reasons for NavCommandMode.INVALID.

    The outer controller (NavController) owns the policy decision (coast,
    abort, reacquire, reset) based on this reason code.
    """

    V_NAVIGATION_INVALID = "V_NAVIGATION_INVALID"
    LOS_RATE_INVALID = "LOS_RATE_INVALID"
    GAMMA_INVALID = "GAMMA_INVALID"
    ZOOM_SLEWING = "ZOOM_SLEWING"
    EKF_DIVERGED = "EKF_DIVERGED"
    DETECTION_LOST = "DETECTION_LOST"
    TARGET_VELOCITY_INVALID = "TARGET_VELOCITY_INVALID"


@dataclass
class NavCommand:
    """Output of NavLaw.calc().

    Exactly one representation is active per mode:
      - ACTUATOR: cmd_roll_deg and cmd_pitch_deg required; cmd_thr optional.
      - ACCELERATION: a_cmd_ned required, shape (3,) [N, E, D] in m/s**2.
      - INVALID: reason required, no command fields populated.

    Invariants are enforced in __post_init__. valid is a derived property.
    """

    mode: NavCommandMode
    cmd_roll_deg: Optional[float] = None
    cmd_pitch_deg: Optional[float] = None
    cmd_thr: Optional[float] = None
    a_cmd_ned: Optional[np.ndarray] = None
    reason: Optional[NavInvalidReason] = None

    def __post_init__(self) -> None:
        has_actuator = self.cmd_roll_deg is not None or self.cmd_pitch_deg is not None
        has_accel = self.a_cmd_ned is not None

        if self.mode == NavCommandMode.ACTUATOR:
            if self.cmd_roll_deg is None or self.cmd_pitch_deg is None:
                raise ValueError(
                    "ACTUATOR NavCommand requires cmd_roll_deg and cmd_pitch_deg"
                )
            if has_accel:
                raise ValueError("ACTUATOR NavCommand must not set a_cmd_ned")
            if self.reason is not None:
                raise ValueError("ACTUATOR NavCommand must not set reason")
        elif self.mode == NavCommandMode.ACCELERATION:
            if not has_accel:
                raise ValueError("ACCELERATION NavCommand requires a_cmd_ned")
            if self.a_cmd_ned.shape != (3,):
                raise ValueError(
                    f"a_cmd_ned must have shape (3,), got {self.a_cmd_ned.shape}"
                )
            if has_actuator or self.cmd_thr is not None:
                raise ValueError(
                    "ACCELERATION NavCommand must not set actuator fields"
                )
            if self.reason is not None:
                raise ValueError("ACCELERATION NavCommand must not set reason")
        elif self.mode == NavCommandMode.INVALID:
            if self.reason is None:
                raise ValueError("INVALID NavCommand requires reason")
            if has_actuator or has_accel or self.cmd_thr is not None:
                raise ValueError("INVALID NavCommand must not set command fields")
        else:
            raise ValueError(f"Unknown NavCommandMode: {self.mode}")

    @property
    def valid(self) -> bool:
        return self.mode != NavCommandMode.INVALID


@dataclass
class NavContext:
    """Inputs to NavLaw.calc().

    Mirrors the current RollL1PitchNav.calc() kwargs. Extensible: later steps
    add fields (observables, EKF state) without breaking existing laws.
    """

    prev_loc: Location
    current_loc: Location
    next_loc: Location
    target_bearing_cd: float
    target_ned: np.ndarray
    pitch_error: float
    distance: Optional[float]


class NavLaw(ABC):
    """Navigation-law interface.

    Implementations may emit ACTUATOR-form or ACCELERATION-form NavCommands.
    An ACCELERATION NavCommand is decomposed into actuator commands by a
    separate NavAllocator (introduced in Step 9).
    """

    @abstractmethod
    def calc(self, ctx: NavContext) -> NavCommand:
        raise NotImplementedError

    @abstractmethod
    def reset(self) -> None:
        raise NotImplementedError

    def handoff_from(self, prev: "NavLaw") -> None:
        """Bumpless transfer hook. Default no-op; laws override as needed."""
        return None
