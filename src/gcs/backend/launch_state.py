"""State and fixed physical constants shared by launch orchestration."""
from __future__ import annotations

import enum

# Fixed relative-altitude floor (m) above which a vehicle is considered already
# flying, so its ESP32 launch trigger is skipped. Intentionally independent of
# the operator-tunable airborne-confirmation threshold (which may be set very
# low or zero) so a grounded vehicle can never falsely skip its launch.
_ALREADY_AIRBORNE_MIN_ALT_M = 10.0

# A launch container physically holds this many UAVs. Vehicles are grouped into
# containers by ascending sys_id. Must match the frontend UAVS_PER_CONTAINER
# (src/gcs/frontend/src/utils/containers.js).
UAVS_PER_CONTAINER = 3


def _container_map(sys_ids: list[int], uavs_per_container: int) -> dict[int, int]:
    """Map each sys_id to a 0-based container index, grouped by ascending sys_id.

    Empty when the group size is unset (container gaps disabled).
    """
    if uavs_per_container <= 0:
        return {}
    return {sid: i // uavs_per_container for i, sid in enumerate(sorted(sys_ids))}


class VehicleState(str, enum.Enum):
    idle = "idle"
    queued = "queued"
    arming = "arming"
    armed = "armed"
    launching = "launching"
    airborne = "airborne"
    failed = "failed"
