"""Shared mission metadata encoding constants and helpers.

Used by both the GCS waypoint builder and the companion-side mission reader.
Metadata is stored in MAV_CMD_DO_SET_ROI_LOCATION (195) items inserted
between corridor and track waypoints in the mission.

Layout per metadata DO item:
  param1 = meta_type, x = lat*1e7, y = lon*1e7, z = 0 (except last)
Only the LAST metadata item's z is non-zero:
  z = (location_type << 8) | search_pattern      [location_type:3 | search_pattern:8]
location_type is only set on a default-delivery-hub item (and is 0
otherwise). Every zone targets the single ``dock`` class, so no dock class
is encoded on the wire.

ArduPlane executes DO_SET_ROI_LOCATION in AUTO (it points the primary mount
at the item), so the GCS planner puts a forward MAV_CMD_DO_JUMP right before
the block whenever an item follows it; readers must skip that jump.
"""
from __future__ import annotations

from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_DO_SET_ROI_LOCATION

# Marker command for metadata items
CORRIDOR_END_MARKER = MAV_CMD_DO_SET_ROI_LOCATION

# Search pattern bitmask (bits 0-7 of z)
SEARCH_PATTERN_IDS = {"distributed": 1, "corridor": 4}
SEARCH_PATTERN_NAMES = {v: k for k, v in SEARCH_PATTERN_IDS.items()}
_SEARCH_PATTERN_MASK = 0xFF

# Location type field (bits 8-10 of z)
_LOCATION_TYPE_SHIFT = 8
_LOCATION_TYPE_MASK = 0x7

# Compact location type IDs stored in bits 8-10 of the default-delivery-hub item's z.
# 0 is reserved for "not encoded".
LOCATION_TYPE_IDS = {
    "building": 1,
    "vehicle": 2,
    "antenna": 3,
    "operations_site": 4,
    "bridge": 5,
    "fuel": 6,
    "other": 7,
}
LOCATION_TYPE_NAMES = {v: k for k, v in LOCATION_TYPE_IDS.items()}

# Metadata item types (param1)
META_POLYGON_VERTEX = 0
META_CORRIDOR_VERTEX = 1
META_LAUNCH_POINT = 2
META_DEFAULT_DELIVERY_HUB = 3

def encode_meta_z(search_pattern: str) -> float:
    """Encode the search pattern into a metadata z value (bits 0-7)."""
    return float(SEARCH_PATTERN_IDS.get(search_pattern, SEARCH_PATTERN_IDS["distributed"]))


def decode_meta_z(z: float) -> str:
    """Decode the search pattern name from a metadata z value."""
    return SEARCH_PATTERN_NAMES.get(int(z) & _SEARCH_PATTERN_MASK, "distributed")


def encode_location_type_into_z(z: float, type_name: str | None) -> float:
    """Encode a location type into bits 8-10 of a z value.

    Layout: [location_type:3 | search_pattern:8] (11 bits max), small enough
    for ArduPilot to accept as a DO item param.
    """
    location_bits = LOCATION_TYPE_IDS.get(type_name or "", 0) & _LOCATION_TYPE_MASK
    search_bits = int(z) & _SEARCH_PATTERN_MASK
    return float((location_bits << _LOCATION_TYPE_SHIFT) | search_bits)


def decode_location_type_from_z(z: float) -> str | None:
    """Decode location type from bits 8-10 of a z value."""
    return LOCATION_TYPE_NAMES.get((int(z) >> _LOCATION_TYPE_SHIFT) & _LOCATION_TYPE_MASK)
