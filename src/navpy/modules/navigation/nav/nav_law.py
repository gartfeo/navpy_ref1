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
from typing import Optional

import numpy as np

from navpy.core.kernel.location import Location
from navpy.core.ports.nav_command import NavCommand


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
    poi_ned: np.ndarray
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
