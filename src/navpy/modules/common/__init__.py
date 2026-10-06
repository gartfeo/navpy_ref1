"""Common shared types used across modules.

This module contains data types that are shared between multiple modules
but don't belong to any single module's domain.
"""
from navpy.modules.common.models.location import Location
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.wind import Wind

__all__ = [
    "Location",
    "Attitude", 
    "Wind",
]

