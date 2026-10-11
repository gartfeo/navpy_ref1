"""Attitude (orientation) model."""
from dataclasses import dataclass


@dataclass
class Attitude:
    """Vehicle attitude (orientation) in Euler angles.
    
    Represents the orientation of a vehicle using pitch, yaw, and roll angles.
    
    .. figure:: http://upload.wikimedia.org/wikipedia/commons/thumb/c/c1/Yaw_Axis_Corrected.svg/500px-Yaw_Axis_Corrected.svg.png
        :width: 400px
        :alt: Diagram showing Pitch, Roll, Yaw
        
    Attributes:
        pitch: Pitch angle in degrees (nose up/down)
        yaw: Yaw angle in degrees (heading)
        roll: Roll angle in degrees (bank left/right)
    """
    pitch: float
    yaw: float
    roll: float
    
    def __str__(self) -> str:
        return f"p={self.pitch:.1f}, y={self.yaw:.1f}, r={self.roll:.1f}"
    
    def to_radians(self) -> "Attitude":
        """Return a new Attitude with angles in radians."""
        import math
        return Attitude(
            pitch=math.radians(self.pitch),
            yaw=math.radians(self.yaw),
            roll=math.radians(self.roll)
        )
    
    @classmethod
    def from_radians(cls, pitch: float, yaw: float, roll: float) -> "Attitude":
        """Create Attitude from radian values."""
        import math
        return cls(
            pitch=math.degrees(pitch),
            yaw=math.degrees(yaw),
            roll=math.degrees(roll)
        )

