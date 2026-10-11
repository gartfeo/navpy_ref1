"""NavCommand contract: the navigation law's output and its modes."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np


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
    POI_VELOCITY_INVALID = "POI_VELOCITY_INVALID"


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
