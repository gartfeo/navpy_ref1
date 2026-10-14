from __future__ import annotations

from navpy.core.kernel.location import Location


class SimulationObject:
    """Positioned simulator POI."""

    def __init__(self, uid: int, loc_global: Location, height: int,
                 location_type: str | None = None) -> None:
        self.uid = uid
        self.g_loc = loc_global
        self.height = height
        self.location_type = location_type


class _Frozen:
    """Refuses attribute writes once constructed (``object.__setattr__`` builds it)."""

    def __setattr__(self, name: str, value: object) -> None:
        raise AttributeError(f"{type(self).__name__} is frozen: cannot set {name!r}")

    def __delattr__(self, name: str) -> None:
        raise AttributeError(f"{type(self).__name__} is frozen: cannot delete {name!r}")


class FrozenLocation(_Frozen, Location):
    """A published POI's location: a copy no reader can change.

    Equal to any :class:`Location` with the same fields.
    """

    def __init__(self, lat: float, lng: float, alt: float, heading: float = 0.0,
                 is_absolute: bool = False) -> None:
        object.__setattr__(self, "lat", lat)
        object.__setattr__(self, "lng", lng)
        object.__setattr__(self, "alt", alt)
        object.__setattr__(self, "heading", heading)
        object.__setattr__(self, "is_absolute", is_absolute)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Location):
            return NotImplemented
        return (self.lat, self.lng, self.alt, self.heading, self.is_absolute) == (
            other.lat, other.lng, other.alt, other.heading, other.is_absolute)

    __hash__ = None  # type: ignore[assignment]  # as Location's: equal by value, unhashable


class FrozenSimulationObject(_Frozen, SimulationObject):
    """A published simulator POI: a frozen copy, its location a :class:`FrozenLocation`."""

    def __init__(self, uid: int, loc_global: Location, height: float,
                 location_type: str | None = None) -> None:
        location = FrozenLocation(loc_global.lat, loc_global.lng, loc_global.alt,
                                  loc_global.heading, loc_global.is_absolute)
        object.__setattr__(self, "uid", uid)
        object.__setattr__(self, "g_loc", location)
        object.__setattr__(self, "height", height)
        object.__setattr__(self, "location_type", location_type)


__all__ = ["FrozenLocation", "FrozenSimulationObject", "SimulationObject"]
