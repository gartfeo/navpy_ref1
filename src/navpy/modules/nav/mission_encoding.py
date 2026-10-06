"""Shared mission metadata encoding constants and helpers.

Used by both the GCS waypoint builder and the companion-side mission reader.
Metadata is stored in MAV_CMD_DO_SET_ROI_LOCATION (195) items inserted
between corridor and track waypoints in the mission.

Layout per metadata DO item:
  param1 = meta_type, x = lat*1e7, y = lon*1e7, z = 0 (except last)
  Location type is encoded in z bits 11-13 (same item as search_pattern when fallback delivery location is last)
Only the LAST metadata item's z encodes search_pattern + dock classes:
  z = (dock_class_bitmask << 8) | search_pattern_bitmask
"""
from __future__ import annotations

from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_DO_SET_ROI_LOCATION
from navpy.modules.vision.vision_class_profile import DOCK_CLASS_TO_DETECT_ID

# Marker command for metadata items
CORRIDOR_END_MARKER = MAV_CMD_DO_SET_ROI_LOCATION

# Search pattern bitmask (bits 0-7 of z)
SEARCH_PATTERN_IDS = {"distributed": 1, "corridor": 4}
SEARCH_PATTERN_NAMES = {v: k for k, v in SEARCH_PATTERN_IDS.items()}

# Dock class IDs — bit (id-1) set in dock-class bitmask (bits 8+ of z)
DOCK_CLASS_IDS = {"small": 1, "medium": 2, "large": 3}
DOCK_CLASS_NAMES = {v: k for k, v in DOCK_CLASS_IDS.items()}

# Compact location type IDs stored in bits 11-13 of the fallback-location item's z.
# Keep 0 reserved for "not encoded" so older missions remain distinguishable.
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
META_FALLBACK_DELIVERY_LOCATION = 3

def encode_meta_z(search_pattern: str, dock_classes: list[str] | None = None) -> float:
    """Encode search_pattern + dock classes into a single z value."""
    search_pattern_bits = SEARCH_PATTERN_IDS.get(search_pattern, 1)
    dock_bits = 0
    for tc in (dock_classes or []):
        cid = DOCK_CLASS_IDS.get(tc)
        if cid:
            dock_bits |= 1 << (cid - 1)
    return float((dock_bits << 8) | search_pattern_bits)


def decode_meta_z(z: float) -> tuple[str, list[str]]:
    """Decode search_pattern + dock classes from a z value.

    Returns (search_pattern_name, sorted list of dock class names).
    """
    val = int(z)
    search_pattern = SEARCH_PATTERN_NAMES.get(val & 0xFF, "distributed")
    dock_bits = val >> 8
    classes = sorted(
        [name for cid, name in DOCK_CLASS_NAMES.items() if dock_bits & (1 << (cid - 1))],
        key=lambda n: DOCK_CLASS_IDS[n],
    )
    return search_pattern, classes


def encode_location_type_into_z(z: float, type_name: str | None) -> float:
    """Encode a location type into bits 11-13 of a z value.

    Layout: [location_type:3 | dock_classes:3 | search_pattern:8] (14 bits max)
    Keeps z small enough for ArduPilot to accept as a DO item param.
    """
    val = int(z)
    location_bits = LOCATION_TYPE_IDS.get(type_name or "", 0) & 0x7
    return float((location_bits << 11) | (val & 0x7FF))


def decode_location_type_from_z(z: float) -> str | None:
    """Decode location type from bits 11-13 of a z value."""
    return LOCATION_TYPE_NAMES.get((int(z) >> 11) & 0x7)
