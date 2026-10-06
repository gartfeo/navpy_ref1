"""Compatibility exports for Roll-L1 navigation and legacy constructors."""

from __future__ import annotations

from navpy.args.navigation_args import NavigationArgs
from navpy.modules.navigation.nav.roll_l1_core import (
    PitchController,
    PitchLockPolicy,
    PitchPidController,
    PitchPnController,
    RollL1PitchNav,
    create_pitch_controller,
)
from navpy.modules.vehicle.vehicle_interface import IVehicle


class _LegacyRollL1Facade:
    def __init__(
        self,
        vehicle: IVehicle,
        args: NavigationArgs,
        controller_name: str,
    ) -> None:
        from navpy.modules.navigation.nav.roll_l1_composition import (
            compose_roll_l1_nav,
        )

        self._inner = compose_roll_l1_nav(
            vehicle,
            args,
            controller_name=controller_name,
        )

    def calc(
        self,
        *args: object,
        **kwargs: object,
    ) -> tuple[float, float, float | None]:
        return self._inner.calc(*args, **kwargs)

    def reset(self) -> None:
        self._inner.reset()


class RollL1PitchPidNav(_LegacyRollL1Facade):
    def __init__(self, vehicle: IVehicle, args: NavigationArgs) -> None:
        super().__init__(vehicle, args, "pid")


class RollL1PitchPnNav(_LegacyRollL1Facade):
    def __init__(self, vehicle: IVehicle, args: NavigationArgs) -> None:
        super().__init__(vehicle, args, "pn")


__all__ = [
    "PitchController",
    "PitchLockPolicy",
    "PitchPidController",
    "PitchPnController",
    "RollL1PitchNav",
    "RollL1PitchPidNav",
    "RollL1PitchPnNav",
    "create_pitch_controller",
]
