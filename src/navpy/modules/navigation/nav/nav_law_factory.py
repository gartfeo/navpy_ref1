"""Navigation-algorithm metadata registry."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Protocol

from navpy.args.navigation_args import NavigationAlgorithm, NavigationArgs
from navpy.modules.navigation.nav.terminal_airframe_config import (
    VehicleTerminalLawConfigProvider,
)


class NavigationLawLifecycle(Protocol):
    def reset(self) -> None: ...


KpProvider = Callable[[NavigationArgs], Optional[float]]


@dataclass(frozen=True)
class NavAlgorithmSpec:
    algorithm: NavigationAlgorithm
    log_name: str
    log_message: str
    kp_provider: KpProvider


def _pitch_kp(args: NavigationArgs) -> Optional[float]:
    return float(args.pitch_args.kp)


def _no_kp(_args: NavigationArgs) -> Optional[float]:
    return None


NAV_ALGORITHM_SPECS = {
    NavigationAlgorithm.PID: NavAlgorithmSpec(
        algorithm=NavigationAlgorithm.PID,
        log_name=NavigationAlgorithm.PID.value,
        log_message="Using L1 roll + PID pitch",
        kp_provider=_pitch_kp,
    ),
    NavigationAlgorithm.PN: NavAlgorithmSpec(
        algorithm=NavigationAlgorithm.PN,
        log_name=NavigationAlgorithm.PN.value,
        log_message="Using L1 roll + PN pitch",
        kp_provider=_pitch_kp,
    ),
    NavigationAlgorithm.VISION_NAV_PN: NavAlgorithmSpec(
        algorithm=NavigationAlgorithm.VISION_NAV_PN,
        log_name=NavigationAlgorithm.VISION_NAV_PN.value,
        log_message="Using vision nav + PN pitch",
        kp_provider=_no_kp,
    ),
}


def get_nav_algorithm_spec(
    algorithm: NavigationAlgorithm | str,
) -> NavAlgorithmSpec:
    resolved = (
        algorithm
        if isinstance(algorithm, NavigationAlgorithm)
        else NavigationAlgorithm(str(algorithm))
    )
    return NAV_ALGORITHM_SPECS[resolved]


__all__ = [
    "NavigationLawLifecycle",
    "NavAlgorithmSpec",
    "VehicleTerminalLawConfigProvider",
    "get_nav_algorithm_spec",
]
