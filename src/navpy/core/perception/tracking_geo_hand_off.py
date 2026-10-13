"""Visual tracking start that hands off from geo pointing on every camera."""

from __future__ import annotations

from collections.abc import Callable

from navpy.core.perception.detector_ports import TrackingCommandPort


class GeoReleasingTracking:
    """Start visual tracking, then stop the geo sessions of all cameras.

    ``start_tracking`` clears the geo session of the tracking camera only;
    with several cameras the others would stay geo-pointed at the reported
    POI. The tracking camera's session is already clear, so its geo stop
    returns without moving it. Stop errors propagate to the caller.
    """

    def __init__(
        self,
        tracking: TrackingCommandPort,
        stop_geo_tracking: Callable[[], None],
    ) -> None:
        self._tracking = tracking
        self._stop_geo_tracking = stop_geo_tracking

    def start_tracking(self, obj_id: int) -> None:
        self._tracking.start_tracking(obj_id)
        self._stop_geo_tracking()

    def stop_tracking(self, to_neutral: bool = True) -> None:
        self._tracking.stop_tracking(to_neutral=to_neutral)


__all__ = ["GeoReleasingTracking"]
