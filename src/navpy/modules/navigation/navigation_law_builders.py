"""Algorithm-specific law construction at the vehicle composition boundary."""

from __future__ import annotations

from collections.abc import Callable, Mapping

from navpy.args.navigation_args import NavigationAlgorithm, NavigationArgs
from navpy.modules.navigation.nav.nav_law_factory import NavigationLawLifecycle
from navpy.modules.navigation.nav.roll_l1_composition import compose_roll_l1_law
from navpy.modules.navigation.nav.final_approach_airframe_config import (
    FinalApproachAirframeConfigProvider,
    FinalApproachParameterPort,
)
from navpy.modules.navigation.nav.vision_nav.law import VisionNavLaw
from navpy.modules.vehicle.vehicle_interface import IVehicle


LawBuilder = Callable[[], NavigationLawLifecycle]


def compose_law_builders(
    vehicle: IVehicle,
    args: NavigationArgs,
) -> Mapping[NavigationAlgorithm, LawBuilder]:
    final_approach_config = FinalApproachAirframeConfigProvider(
        FinalApproachParameterPort(
            lambda name: vehicle.get_parameter(name, quiet=True)
        ),
        lambda: args.final_approach_throttle,
    )
    return {
        NavigationAlgorithm.PID: lambda: compose_roll_l1_law(vehicle, args),
        NavigationAlgorithm.PN: lambda: compose_roll_l1_law(vehicle, args),
        NavigationAlgorithm.VISION_NAV_PN: (
            lambda: VisionNavLaw(final_approach_config)
        ),
    }


__all__ = ["LawBuilder", "compose_law_builders"]
