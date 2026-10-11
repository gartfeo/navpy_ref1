"""Wind model."""


class Wind:
    """Wind information.
    
    Attributes:
        direction: Wind direction in degrees (where wind is coming FROM), 0-360
        speed: Wind speed in m/s
        speed_z: Vertical wind speed in m/s (positive = upward)
    """
    
    def __init__(self, direction: float, speed: float, speed_z: float = 0.0):
        # Normalize direction to 0-360
        d = float(direction)
        if d < 0:
            d += 360
        self.direction = d
        self.speed = speed
        self.speed_z = speed_z
    
    def __repr__(self) -> str:
        return f"Wind(direction={self.direction}, speed={self.speed}, speed_z={self.speed_z})"
    
    def __str__(self) -> str:
        return f"dir={self.direction:.0f}°, spd={self.speed:.1f}m/s"
    
    @property
    def heading(self) -> float:
        """Wind heading (direction wind is going TO)."""
        return (self.direction + 180) % 360

