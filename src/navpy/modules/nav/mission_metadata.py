"""Read metadata items encoded in mission waypoints by waypoint_builder."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, List

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_DO_SET_ROI_LOCATION,
    MAV_CMD_NAV_WAYPOINT,
)

from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.common.models.location import Location
from navpy.modules.nav.mission_encoding import (
    META_FALLBACK_DELIVERY_LOCATION,
    decode_meta_z, decode_location_type_from_z,
    DOCK_CLASS_TO_DETECT_ID,
)


@dataclass
class MissionMetadata:
    """Decoded mission metadata."""
    search_pattern: str = "distributed"
    dock_classes: List[str] = field(default_factory=list)
    waypoint_altitudes: List[float] = field(default_factory=list)
    # Scan/zone (navigation task) altitude relative to home — the per-class
    # ``altitude_m`` band the drone orbits and dives from, distinct from
    # the (often higher) corridor/transit band. See _resolve_scan_altitude.
    scan_altitude_rel: Optional[float] = None
    fallback_delivery_location: Optional[Location] = None
    fallback_delivery_location_type: Optional[str] = None

    @property
    def detect_class_ids(self) -> List[int]:
        """Map mission dock class names to detection class IDs."""
        return [DOCK_CLASS_TO_DETECT_ID.get(tc, 0) for tc in self.dock_classes]


def read_fallback_delivery_location(vehicle: IVehicle) -> Optional[Location]:
    """Scan mission items for a META_FALLBACK_DELIVERY_LOCATION metadata entry.

    Returns the fallback delivery location Location or None if not present.
    """
    for seq in range(vehicle.mission_items_count):
        wp = vehicle.get_mission_item(seq)
        if wp is None:
            continue
        if wp.command == MAV_CMD_DO_SET_ROI_LOCATION and int(wp.param1) == META_FALLBACK_DELIVERY_LOCATION:
            return Location(wp.x / 1e7, wp.y / 1e7, 0)
    return None


def _resolve_scan_altitude(
    post_marker_alts: set[float], pre_marker_alts: set[float],
) -> Optional[float]:
    """Resolve the scan/zone (navigation task) altitude relative to home.

    A mission carries two altitude bands by construction (see
    ``waypoint_builder.build_mission``): corridor waypoints at the
    transit altitude *before* the metadata marker block, and scan/zone
    waypoints at the per-class ``altitude_m`` *after* it. The drone
    orbits and dives from the SCAN band, so that — never the (often
    higher) corridor band — is the navigation task altitude. Taking the
    corridor altitude oversizes the orbit past recognition range
    (SITL run 174040: 898 m, mission stalled in CONFIRM_WAIT).

    Scan waypoints share a single ``altitude_m`` per upload, so ``min``
    of the post-marker band returns it (heterogeneous scan altitudes
    would need a richer mission metadata model).

    When the mission has NO metadata marker there is no authoritative
    corridor/scan boundary. If every NAV_WAYPOINT shares one altitude the
    band is unambiguous and is returned; if the markerless mission spans
    multiple distinct altitudes the scan band cannot be identified
    (``min`` would guess the corridor band for a scan-above-corridor
    mission, ``max`` the reverse), so ``None`` is returned and callers
    fall back to legacy camera-range sizing rather than guess — the same
    source-of-truth discipline that motivates this whole change. Returns
    ``None`` when the mission has no altitude-bearing NAV_WAYPOINT.
    """
    if post_marker_alts:
        return min(post_marker_alts)
    if len(pre_marker_alts) == 1:
        return next(iter(pre_marker_alts))
    return None


def read_mission_metadata(vehicle: IVehicle) -> MissionMetadata:
    """Read search_pattern, dock classes, altitudes, and fallback delivery location from mission.

    Scans all mission items for:
    - Metadata markers (DO_SET_ROI_LOCATION): search_pattern + dock classes from last z
    - NAV_WAYPOINT items: collects distinct altitudes, split into the
      corridor band (before the first marker) and the scan/zone band
      (after it) to resolve ``scan_altitude_rel``
    - Fallback delivery location location
    """
    last_meta_z = 0.0
    altitudes: set[float] = set()
    pre_marker_alts: set[float] = set()
    post_marker_alts: set[float] = set()
    seen_corridor_marker = False
    fallback_delivery_location: Optional[Location] = None
    fallback_delivery_location_type: Optional[str] = None

    for seq in range(vehicle.mission_items_count):
        wp = vehicle.get_mission_item(seq)
        if wp is None:
            continue

        if wp.command == MAV_CMD_DO_SET_ROI_LOCATION:
            # The metadata block is inserted between the corridor and the
            # scan track, so the first marker is the corridor/scan boundary.
            seen_corridor_marker = True
            meta_type = int(wp.param1)
            if meta_type == META_FALLBACK_DELIVERY_LOCATION:
                fallback_delivery_location = Location(wp.x / 1e7, wp.y / 1e7, 0)
                fallback_delivery_location_type = decode_location_type_from_z(wp.z)
            if wp.z != 0:
                last_meta_z = wp.z

        elif wp.command == MAV_CMD_NAV_WAYPOINT and wp.z > 0:
            alt = float(wp.z)
            altitudes.add(alt)
            if seen_corridor_marker:
                post_marker_alts.add(alt)
            else:
                pre_marker_alts.add(alt)

    search_pattern, dock_classes = decode_meta_z(last_meta_z)

    return MissionMetadata(
        search_pattern=search_pattern,
        dock_classes=dock_classes,
        waypoint_altitudes=sorted(altitudes),
        scan_altitude_rel=_resolve_scan_altitude(post_marker_alts, pre_marker_alts),
        fallback_delivery_location=fallback_delivery_location,
        fallback_delivery_location_type=fallback_delivery_location_type,
    )
