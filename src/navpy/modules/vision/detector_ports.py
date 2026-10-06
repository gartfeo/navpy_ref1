"""Narrow structural capabilities exposed by detector implementations."""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING, Protocol, Sequence

from navpy.modules.vision.worker_failure import FailureHealthPort

if TYPE_CHECKING:
    from navpy.modules.common.models.attitude import Attitude
    from navpy.modules.common.models.location import Location
    from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
    from navpy.modules.vision.camera_mount import CameraMount
    from navpy.modules.vision.models.detect_data import DetectedObject
    from navpy.modules.vision.models.detection_event_lease import DetectionEventLease
    from navpy.modules.vision.models.detection_publication import DetectionPublication
    from navpy.modules.vision.models.detect_request import DetectRequest
    from navpy.modules.vision.models.detect_response import DetectResponse
    from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
    from navpy.modules.vision.poi_zoom_types import ZoomTrackResult


class DetectionSnapshotPort(Protocol):
    def get_detect_data(self, request: "DetectRequest") -> "DetectResponse": ...


class DetectionEventPort(Protocol):
    @property
    def has_source_driven_detection_events(self) -> bool: ...

    def drain_detection_events(
        self,
        request: "DetectRequest",
    ) -> Sequence["DetectionPublication"]: ...

    def open_detection_event_lease(
        self,
        request: "DetectRequest",
        reset_handler: Callable[[], None] | None = None,
    ) -> "DetectionEventLease | None": ...

    def poi_uses_source_driven_events(
        self,
        poi: "DetectedObject",
    ) -> bool | None: ...


class TrackingCommandPort(Protocol):
    def start_tracking(self, obj_id: int) -> None: ...

    def stop_tracking(self, to_neutral: bool = True) -> None: ...


class TrackingStatusPort(Protocol):
    @property
    def is_detection_armed(self) -> bool: ...

    @property
    def loss_hold_sec(self) -> float | None: ...


class PoiIdentityPort(Protocol):
    def rebind_task_id(self, task_id: int, poi: "DetectedObject") -> bool: ...


class GeoPointingPort(Protocol):
    def start_geo_tracking(
        self,
        poi_loc: "Location",
        geo_ref: "GeoRefCalc",
    ) -> None: ...

    def update_geo(self, uav_loc: "Location", uav_att: "Attitude") -> None: ...

    def prepare_geo_acquisition(
        self,
        uav_loc: "Location",
        uav_att: "Attitude",
        class_id: int,
        min_pixels: float,
    ) -> bool: ...

    def stop_geo_tracking(self) -> None: ...

    @property
    def is_geo_armed(self) -> bool: ...


class ZoomControlPort(Protocol):
    @property
    def is_zoom_stable(self) -> bool: ...

    def get_zoom_result(
        self,
        obj_id: int | None = None,
    ) -> "ZoomTrackResult | None": ...

    def set_zoom_size_demand(self, enabled: bool) -> None: ...

    def freeze_final_approach_zoom_at_min(self) -> bool: ...


class MountCatalogPort(Protocol):
    @property
    def mounts(self) -> Sequence["CameraMount"]: ...


class SimulationControlPort(Protocol):
    @property
    def is_simulation(self) -> bool: ...

    def set_sim_poi(
        self,
        command_index: int,
        location: "Location",
        location_type: str | None = None,
    ) -> None: ...


class DetectorResetPort(Protocol):
    def refresh(self) -> None: ...


class DetectorLifecyclePort(FailureHealthPort, Protocol):
    def start(self) -> None: ...

    def stop(self) -> bool: ...

    @property
    def is_quiescent(self) -> bool: ...


class SchedulerCadence(Protocol):
    def wall_period_for_scheduler_period(self, scheduler_period_s: float) -> float: ...


class SourceIdentity(Protocol):
    @property
    def source_name(self) -> str: ...


class GimbalDataReader(Protocol):
    def __call__(self) -> "GimbalData": ...


def source_name_from_gimbal(read_gimbal_data: GimbalDataReader) -> str:
    """Read the routing identity without performing construction-time I/O."""
    name = getattr(read_gimbal_data(), "name", None)
    if not isinstance(name, str) or not name:
        raise RuntimeError("detector gimbal data must report a non-empty name")
    return name


class GimbalSourceIdentity:
    """Lazy source identity backed by the mount's current gimbal data."""

    def __init__(self, read_gimbal_data: GimbalDataReader) -> None:
        self._read_gimbal_data = read_gimbal_data

    @property
    def source_name(self) -> str:
        return source_name_from_gimbal(self._read_gimbal_data)


class PollingDetectionEvents:
    """Explicit null event capability for polling detectors."""

    @property
    def has_source_driven_detection_events(self) -> bool:
        return False

    def drain_detection_events(
        self,
        request: "DetectRequest",
    ) -> tuple["DetectionPublication", ...]:
        del request
        return ()

    def open_detection_event_lease(
        self,
        request: "DetectRequest",
        reset_handler: Callable[[], None] | None = None,
    ) -> None:
        del request, reset_handler
        return None

    def poi_uses_source_driven_events(
        self,
        poi: "DetectedObject",
    ) -> bool:
        del poi
        return False


class IdentitySchedulerCadence:
    """Default cadence capability that leaves scheduler frequency unchanged."""

    def wall_period_for_scheduler_period(self, scheduler_period_s: float) -> float:
        return scheduler_period_s


class NonSimulationControls:
    """Explicit null capability for hardware-backed detectors."""

    @property
    def is_simulation(self) -> bool:
        return False

    def set_sim_poi(
        self,
        command_index: int,
        location: "Location",
        location_type: str | None = None,
    ) -> None:
        del command_index, location, location_type


class DetectorFleetMember(
    DetectionSnapshotPort,
    DetectionEventPort,
    TrackingCommandPort,
    TrackingStatusPort,
    GeoPointingPort,
    ZoomControlPort,
    MountCatalogPort,
    SimulationControlPort,
    DetectorResetPort,
    DetectorLifecyclePort,
    SchedulerCadence,
    SourceIdentity,
    Protocol,
):
    """Full detector intersection used only at the fleet composition boundary."""


__all__ = [
    "DetectionEventPort",
    "DetectionSnapshotPort",
    "DetectorFleetMember",
    "DetectorLifecyclePort",
    "DetectorResetPort",
    "GeoPointingPort",
    "GimbalDataReader",
    "GimbalSourceIdentity",
    "IdentitySchedulerCadence",
    "MountCatalogPort",
    "NonSimulationControls",
    "PollingDetectionEvents",
    "SchedulerCadence",
    "SimulationControlPort",
    "SourceIdentity",
    "PoiIdentityPort",
    "TrackingCommandPort",
    "TrackingStatusPort",
    "ZoomControlPort",
    "source_name_from_gimbal",
]
