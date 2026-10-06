"""Geo-reference and terrain utilities for the navigation module.

This module owns all geo-reference calculations and terrain utilities:
- GeoRefCalc: Coordinate transformations between camera/gimbal/UAS/NED frames
- ZcUtil: Terrain intersection and DEM data queries
- DemData: Digital Elevation Model data handling
- Rotation utilities for Euler angle conversions

Usage:
    from navpy.modules.navigation.geo import GeoRefCalc, ZcUtil
"""
# Import from existing locations (will be moved here in future)
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.navigation.geo.dem_data import DemData
from navpy.modules.navigation.geo.zc_util import ZcUtil
from navpy.modules.navigation.geo.rotation_utils import (
    get_euler_rotation_angles,
    calculate_yaw_pitch,
    normalize,
    calculate_euler_angles,
)

__all__ = [
    "GeoRefCalc",
    "ZcUtil",
    "DemData",
    "get_euler_rotation_angles",
    "calculate_yaw_pitch",
    "normalize",
    "calculate_euler_angles",
]

