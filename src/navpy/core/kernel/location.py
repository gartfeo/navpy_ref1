"""Geographic location model."""
from __future__ import annotations
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pymavlink.mavutil import location

EARTH_RADIUS_M = 6371000

@dataclass
class Location:
    """Geographic location with latitude, longitude, altitude.
    
    Attributes:
        lat: Latitude in degrees
        lng: Longitude in degrees  
        alt: Altitude in meters
        heading: Heading in degrees (0-360)
        is_absolute: True if altitude is absolute (AMSL), False if relative to home
    """
    lat: float
    lng: float
    alt: float
    heading: float = 0.0
    is_absolute: bool = False
    
    @classmethod
    def create(cls, loc: "location") -> "Location":
        """Create from pymavlink location object."""
        return cls(
            lat=loc.lat,
            lng=loc.lng, 
            alt=loc.alt,
            heading=loc.heading,
            is_absolute=True
        )
    
    def __str__(self) -> str:
        return f"{self.lat:.6f}, {self.lng:.6f}, {self.alt:.1f}"
    
    def distance_to(self, other: "Location") -> float:
        """Calculate approximate distance to another location in meters.
        
        Uses haversine formula for great-circle distance.
        """
        import math

        lat1 = math.radians(self.lat)
        lat2 = math.radians(other.lat)
        dlat = math.radians(other.lat - self.lat)
        dlng = math.radians(other.lng - self.lng)
        
        a = (math.sin(dlat / 2) ** 2 +
             math.cos(lat1) * math.cos(lat2) * math.sin(dlng / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return EARTH_RADIUS_M * c

