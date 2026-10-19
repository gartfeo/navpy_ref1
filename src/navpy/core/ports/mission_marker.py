"""A mission marker, decoded without giving it a meaning."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from navpy.core.kernel.location import Location


@dataclass(frozen=True)
class MissionMarker:
    """One position marker of the uploaded mission: its code, place and site type.

    The vehicle plugin decodes every marker; a mission module decides which
    codes mean something to it.
    """

    code: int
    location: Location
    location_type: Optional[str]


__all__ = ["MissionMarker"]
