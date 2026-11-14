"""Algorithm-specific law construction at the vehicle composition boundary."""

from __future__ import annotations

from collections.abc import Callable, Mapping

from navpy.core.kernel.config.navigation_args import NavigationAlgorithm, NavigationArgs
from navpy.core.approach.nav_law_factory import NavigationLawLifecycle
from navpy.app.approach.roll_l1_composition import compose_roll_l1_law
from navpy.plugins.vehicle.ardupilot.guidance_airframe_config import (
    GuidanceAirframeConfigProvider,
    GuidanceParameterPort,
)
from navpy.plugins.laws.vision_pn.law import VisionNavLaw
from navpy.core.ports.vehicle_contract import IVehicle


LawBuilder = Callable[[], NavigationLawLifecycle]


def compose_law_builders(
    vehicle: IVehicle,
    args: NavigationArgs,
) -> Mapping[NavigationAlgorithm, LawBuilder]:
    guidance_config = GuidanceAirframeConfigProvider(
        GuidanceParameterPort(
            lambda name: vehicle.get_parameter(name, quiet=True)
        ),
        lambda: args.guidance_throttle,
    )
    return {
        NavigationAlgorithm.PID: lambda: compose_roll_l1_law(vehicle, args),
        NavigationAlgorithm.PN: lambda: compose_roll_l1_law(vehicle, args),
        NavigationAlgorithm.VISION_NAV_PN: (
            lambda: VisionNavLaw(guidance_config)
        ),
    }


__all__ = ["LawBuilder", "compose_law_builders"]
