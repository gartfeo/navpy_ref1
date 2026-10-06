"""Navigation module - Final-approach navigation and path following.

This module owns ALL navigation-related functionality:
- Approach geometry for a selected delivery reference
- Trajectory/path following
- Geo-reference math (via navigation.geo submodule)
- Terrain utilities (via navigation.geo submodule)
- Navigation algorithms (L1 control, PID)
- Mission planning

Generic target/setpoint and standard control terms retain their engineering
meaning. Approach geometry is not an approved physical handover procedure.

The public API is through this module:
    from navpy.modules.navigation import Navigation, GeoRefCalc, ZcUtil
    
Or access the geo submodule directly:
    from navpy.modules.navigation.geo import GeoRefCalc, ZcUtil
"""
# Core navigation
from navpy.modules.navigation.navigation import Navigation, ClosestSnap
from navpy.modules.navigation.navigation_data import NavigationData
from navpy.modules.navigation.mission_planner import MissionPlanner, MissionPlannerArgs
from navpy.modules.navigation.gimbal_navigation import GimbalNavigation

# Navigation algorithms
from navpy.modules.navigation.nav.roll_l1_pitch_nav import RollL1PitchNav
from navpy.modules.navigation.nav.roll_l1_pitch_nav import RollL1PitchPidNav, RollL1PitchPnNav

# Geo-reference (convenience re-export)
from navpy.modules.navigation.geo import GeoRefCalc, ZcUtil, DemData

__all__ = [
    # Core navigation
    "Navigation",
    "ClosestSnap",
    "NavigationData",
    "MissionPlanner",
    "MissionPlannerArgs",
    "GimbalNavigation",
    # Navigation
    "RollL1PitchNav",
    "RollL1PitchPidNav",
    "RollL1PitchPnNav",
    # Geo/terrain
    "GeoRefCalc",
    "ZcUtil",
    "DemData",
]

