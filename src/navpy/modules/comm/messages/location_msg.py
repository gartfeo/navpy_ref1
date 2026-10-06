from dataclasses import dataclass
from typing import Dict, Any

from navpy.modules.comm.messages.msg_abc import MsgDataAbc


@dataclass
class LocationMsgData(MsgDataAbc):
    """
    Data class representing location message data.
    """
    lat: float
    lng: float
    alt: float

    def __str__(self):
        return f"Location: {self.lat:.6f}, {self.lng:.6f}, {self.alt:.1f}"

    def to_dict(self) -> Dict[str, Any]:
        return {
            'lat': self.lat,
            'lng': self.lng,
            'a': self.alt,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'LocationMsgData':
        return cls(float(data['lat']), float(data['lng']), float(data['a']))
