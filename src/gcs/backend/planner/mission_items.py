"""Convert track points to MAVLink mission items for VehicleMav upload."""
from __future__ import annotations


from pymavlink.mavwp import MAVWPLoader
from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_DO_JUMP,
    MAV_CMD_NAV_WAYPOINT,
    MAV_CMD_NAV_TAKEOFF,
    MAV_FRAME_GLOBAL_RELATIVE_ALT,
    MAV_FRAME_MISSION,
)

# Shared mission metadata encoding — single source of truth
from navpy.modules.nav.mission_encoding import CORRIDOR_END_MARKER, META_POLYGON_VERTEX, META_CORRIDOR_VERTEX, META_LAUNCH_POINT, META_DEFAULT_DELIVERY_HUB, encode_meta_z, encode_location_type_into_z


def build_mission(
    track_latlon: list[dict],
    altitude_m: float,
    target_system: int = 1,
    corridor_count: int = 0,
    search_pattern: str = "distributed",
    polygon: list[dict] | None = None,
    corridor_backbone: list[dict] | None = None,
    launch_point: dict | None = None,
    corridor_altitude_m: float | None = None,
    default_delivery_hub: dict | None = None,
    takeoff_altitude_m: float | None = None,
) -> MAVWPLoader:
    """Build a MAVWPLoader from track lat/lon points.

    Args:
        track_latlon: list of {"lat": float, "lon": float}
        altitude_m: relative altitude in meters
        target_system: MAVLink system ID
        corridor_count: number of leading waypoints that form the
            corridor (corridor points only, no launch point). A DO marker
            is inserted after these to allow safe round-trip parsing.
        search_pattern: search pattern name ("distributed", "corridor")
        launch_point: separate launch/home position {"lat": float, "lon": float}.
            If provided, Home and Takeoff use these coords instead of track[0].
        takeoff_altitude_m: safe climb-out height (m, relative) that ends the
            NAV_TAKEOFF phase. When None, the takeoff waypoint keeps the legacy
            behavior of climbing to the full first-leg altitude.

    Returns:
        MAVWPLoader ready for upload_mission()
    """
    wp_loader = MAVWPLoader(target_system, 0)

    # Home/Takeoff location: use launch_point if provided, else first track point
    home_loc = launch_point if launch_point else (track_latlon[0] if track_latlon else None)

    # Home waypoint (seq 0)
    if home_loc:
        wp_loader.add_latlonalt(
            home_loc["lat"], home_loc["lon"], 0,
        )
        # Set frame for home
        wp = wp_loader.wp(0)
        wp.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT
        wp.command = MAV_CMD_NAV_WAYPOINT
        wp.autocontinue = 1
        # INT format for mission_item_int_send
        wp.x = int(home_loc["lat"] * 1e7)
        wp.y = int(home_loc["lon"] * 1e7)

    corr_alt = corridor_altitude_m if corridor_altitude_m is not None else altitude_m

    # Takeoff waypoint (seq 1). NAV_TAKEOFF altitude is the climb target that
    # ends the takeoff phase before the aircraft proceeds to the first leg, so
    # it should be a low safe height — not the mission altitude. The first-leg
    # altitude (corridor altitude when a corridor exists, else zone altitude) is
    # the ceiling: clamp to it so we never command a climb above the first leg
    # followed by a descent, and never overshoot the corridor approach altitude.
    first_leg_alt = corr_alt if corridor_count > 0 else altitude_m
    if takeoff_altitude_m is not None:
        takeoff_alt = min(takeoff_altitude_m, first_leg_alt)
    else:
        takeoff_alt = first_leg_alt
    if home_loc:
        wp_loader.add_latlonalt(
            home_loc["lat"], home_loc["lon"], takeoff_alt,
        )
        wp = wp_loader.wp(1)
        wp.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT
        wp.command = MAV_CMD_NAV_TAKEOFF
        wp.param1 = 15.0  # pitch angle
        wp.autocontinue = 1
        wp.x = int(home_loc["lat"] * 1e7)
        wp.y = int(home_loc["lon"] * 1e7)

    # Track waypoints (with metadata DO items between corridor and track)
    # Collect all metadata items: (meta_type, point_dict)
    meta_items: list[tuple[int, dict]] = []
    for pv in (polygon or []):
        meta_items.append((META_POLYGON_VERTEX, pv))
    for cv in (corridor_backbone or []):
        meta_items.append((META_CORRIDOR_VERTEX, cv))
    if launch_point:
        meta_items.append((META_LAUNCH_POINT, launch_point))
    if default_delivery_hub:
        meta_items.append((META_DEFAULT_DELIVERY_HUB, default_delivery_hub))

    def _insert_skip_jump() -> None:
        # ArduPilot executes DO_SET_ROI_LOCATION in AUTO (Plane points the
        # primary mount at it), so AUTO must never reach the metadata block.
        # A forward DO_JUMP to the item right after the block makes
        # AP_Mission skip it for both nav and DO execution. Repeat -1 means
        # "no limit", so the skip also holds on mission restarts.
        seq = wp_loader.count()
        wp_loader.add_latlonalt(0, 0, 0)
        wp = wp_loader.wp(seq)
        wp.frame = MAV_FRAME_MISSION
        wp.command = MAV_CMD_DO_JUMP
        wp.param1 = float(seq + 1 + len(meta_items))
        wp.param2 = -1.0
        wp.autocontinue = 1
        wp.x = 0
        wp.y = 0
        wp.z = 0

    def _insert_metadata(followed: bool) -> None:
        # With nothing after the block the jump target would be past the
        # last item; AP would then just end the mission, but we do not rely
        # on its invalid-target handling and emit no jump instead.
        if followed:
            _insert_skip_jump()
        for idx, (meta_type, mp) in enumerate(meta_items):
            seq = wp_loader.count()
            wp_loader.add_latlonalt(mp["lat"], mp["lon"], 0)
            wp = wp_loader.wp(seq)
            wp.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT
            wp.command = CORRIDOR_END_MARKER
            wp.param1 = float(meta_type)
            wp.autocontinue = 1
            wp.x = int(mp["lat"] * 1e7)
            wp.y = int(mp["lon"] * 1e7)
            # Only the last metadata item carries search_pattern
            if idx == len(meta_items) - 1:
                wp.z = encode_meta_z(search_pattern)
            else:
                wp.z = 0
            # Location type encoded into z bits 8-10
            if meta_type == META_DEFAULT_DELIVERY_HUB:
                wp.z = encode_location_type_into_z(wp.z, mp.get("type"))

    meta_inserted = False
    for i, pt in enumerate(track_latlon):
        # Insert metadata between corridor and track waypoints
        if i == corridor_count and meta_items:
            # A track waypoint always follows a block inserted in-loop.
            _insert_metadata(followed=True)
            meta_inserted = True

        # Corridor approach waypoints use corridor altitude (base altitude
        # for all UAVs); scanning track waypoints use zone-specific altitude.
        wp_alt = corr_alt if i < corridor_count else altitude_m
        seq = wp_loader.count()
        wp_loader.add_latlonalt(
            pt["lat"], pt["lon"], wp_alt,
        )
        wp = wp_loader.wp(seq)
        wp.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT
        wp.command = MAV_CMD_NAV_WAYPOINT
        wp.autocontinue = 1
        wp.x = int(pt["lat"] * 1e7)
        wp.y = int(pt["lon"] * 1e7)

    # If corridor_count >= len(track_latlon) (e.g. a set whose scan track is
    # empty), the in-loop `i == corridor_count` guard never fires — emit the
    # metadata block now so search_pattern/polygon are never dropped and
    # the companion can still recover them from the mission.
    if not meta_inserted and meta_items:
        # Only the default-delivery-hub waypoint can still follow the block.
        _insert_metadata(followed=default_delivery_hub is not None)

    # Append default delivery hub as the last NAV_WAYPOINT
    if default_delivery_hub:
        seq = wp_loader.count()
        wp_loader.add_latlonalt(default_delivery_hub["lat"], default_delivery_hub["lon"], altitude_m)
        wp = wp_loader.wp(seq)
        wp.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT
        wp.command = MAV_CMD_NAV_WAYPOINT
        wp.autocontinue = 1
        wp.x = int(default_delivery_hub["lat"] * 1e7)
        wp.y = int(default_delivery_hub["lon"] * 1e7)

    return wp_loader
