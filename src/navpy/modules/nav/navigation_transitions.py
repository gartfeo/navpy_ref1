"""Compatibility exports for focused navigation transition owners."""

from navpy.modules.nav.nav_transition import (
    NavExitOutcome,
    NavTransition,
)
from navpy.modules.nav.navigation_transition_handler import (
    NavigationTransitionHandler,
    TransitionOutcome,
    TransitionPorts,
)
from navpy.modules.nav.navigation_zoom import ZoomController

__all__ = [
    "NavExitOutcome",
    "NavTransition",
    "NavigationTransitionHandler",
    "TransitionOutcome",
    "TransitionPorts",
    "ZoomController",
]
