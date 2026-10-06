"""Validated camera, mount, and commanded-zoom factory functions."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import TYPE_CHECKING

from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.peripheral.gimbal_abc import GimbalAbc, GimbalData
from navpy.modules.vision.peripheral.camera_intrinsics import CameraIntrinsics
from navpy.modules.vision.vision_camera_calibration import parse_zoom_calibration
from navpy.modules.vision.vision_detector_profile import get_detector_settings
from navpy.modules.vision.vision_gimbal_profile import (
    build_device_gimbal_metadata,
    build_gimbal_data,
)
from navpy.modules.vision.vision_profile_loader import get_devices
from navpy.modules.vision.vision_profile_types import CameraMountSpec
from navpy.modules.vision.zoom_calibration import ZoomCalibrationTable

if TYPE_CHECKING:
    from navpy.modules.common.scheduler_cadence import SchedulerCadence
    from navpy.modules.vehicle.vehicle_interface import IVehicle


def _positive_integer(value: object, name: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a positive integer")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a positive integer") from exc
    if not math.isfinite(number) or number <= 0.0 or not number.is_integer():
        raise ValueError(f"{name} must be a positive integer")
    return int(number)


def _camera_config(device: Mapping[str, object]) -> Mapping[str, object]:
    if not isinstance(device, Mapping):
        raise ValueError("Vision profile device must be a mapping")
    camera = device.get("camera")
    if not isinstance(camera, Mapping):
        raise ValueError("Vision profile device camera must be a mapping")
    return camera


def _zoom_map(camera: Mapping[str, object]) -> Mapping[object, object]:
    intrinsics = camera.get("intrinsics")
    if not isinstance(intrinsics, Mapping):
        raise ValueError("camera.intrinsics must be a mapping")
    zooms = intrinsics.get("zooms")
    if not isinstance(zooms, Mapping):
        raise ValueError("camera.intrinsics.zooms must be a mapping")
    parse_zoom_calibration(zooms)
    return zooms


def build_camera_model(
    device: Mapping[str, object],
    logger: ILogger,
    *,
    log: bool = True,
) -> CameraIntrinsics:
    camera_config = _camera_config(device)
    zooms = _zoom_map(camera_config)
    width = _positive_integer(camera_config.get("image_width"), "camera.image_width")
    height = _positive_integer(
        camera_config.get("image_height"),
        "camera.image_height",
    )
    camera = CameraIntrinsics(
        zoom_map=dict(zooms),
        image_width=width,
        image_height=height,
    )
    if log:
        first_zoom = next(iter(zooms))
        logger.info(f"Camera model: intrinsics (zoom={first_zoom})")
    return camera


def build_zoom_calibration(
    device: Mapping[str, object],
    logger: ILogger,
) -> ZoomCalibrationTable | None:
    """Build the optional commanded-to-actual zoom calibration table."""
    from navpy.modules.vision.zoom_calibration import ZoomCalibrationTable

    camera = _camera_config(device)
    raw_calibration = camera.get("zoom_calibration")
    if raw_calibration is not None and not isinstance(raw_calibration, Mapping):
        raise ValueError("camera.zoom_calibration must be a mapping")
    table = ZoomCalibrationTable.from_profile(raw_calibration)
    if table is None:
        if raw_calibration:
            logger.warning(
                "zoom_calibration present but unusable (<2 parseable entries); "
                "falling back to raw-readback intrinsics"
            )
        return None
    logger.info(
        f"Zoom calibration loaded: {len(table.entries)} entries, "
        f"actual [{table.actual_min:.2f}, {table.actual_max:.2f}]"
    )
    return table


def _build_gimbal(
    device: Mapping[str, object],
    gimbal_data: GimbalData,
    vehicle: "IVehicle",
    logger: ILogger,
    *,
    use_sim_gimbal: bool,
    scheduler_cadence: "SchedulerCadence | None",
) -> GimbalAbc:
    from navpy.args.uas_args import UasArgs
    from navpy.modules.vision.peripheral.fixed_gimbal import FixedGimbal
    from navpy.modules.vision.sim.gimbal_sim import GimbalSim

    raw_gimbal = device.get("gimbal", {})
    if not isinstance(raw_gimbal, Mapping):  # validated by build_gimbal_data
        raise ValueError("Vision profile device gimbal must be a mapping")
    gimbal_type = raw_gimbal.get("type", "auto")
    is_stabilized = gimbal_data.roll_stabilize or gimbal_data.pitch_stabilize

    if gimbal_type == "siyi" and not use_sim_gimbal:
        from navpy.modules.vision.peripheral.gimbal_siyi import GimbalSiyi

        return GimbalSiyi(
            gimbal_data,
            raw_gimbal.get("siyi_ip", "192.168.144.25"),
            raw_gimbal.get("siyi_port", 37260),
            logger,
        )
    if gimbal_type == "siyi" and use_sim_gimbal:
        from navpy.modules.vision.gimbal_attitude_reader import VehicleAttitudeReader
        from navpy.modules.vision.peripheral.siyi.sim import GimbalSiyiSim

        return GimbalSiyiSim(
            gimbal_data,
            VehicleAttitudeReader(vehicle),
            logger,
            scheduler_cadence=scheduler_cadence,
        )
    if use_sim_gimbal and is_stabilized:
        from navpy.modules.vision.gimbal_attitude_reader import VehicleAttitudeReader

        return GimbalSim(
            gimbal_data,
            VehicleAttitudeReader(vehicle),
            UasArgs().uas_seq,
            scheduler_cadence=scheduler_cadence,
        )
    return FixedGimbal(gimbal_data)


def build_camera_mounts(
    profile: Mapping[str, object],
    vehicle: "IVehicle",
    logger: ILogger,
    *,
    use_sim_gimbal: bool = True,
    scheduler_cadence: "SchedulerCadence | None" = None,
) -> list[CameraMountSpec]:
    """Build mounts paired with immutable profile-device snapshots."""
    from navpy.modules.vision.camera_mount import CameraMount
    from navpy.modules.vision.peripheral.fixed_gimbal import FixedGimbal

    devices = get_devices(profile)
    detector_settings = get_detector_settings(profile)
    mounts: list[CameraMountSpec] = []
    used_gimbal_device_ids: set[int] = set()

    for index, device in enumerate(devices):
        name = device.get("name", f"mount_{index}")
        if not isinstance(name, str) or not name:
            raise ValueError("Vision profile device name must be a non-empty string")
        metadata = build_device_gimbal_metadata(
            device,
            index,
            used_ids=used_gimbal_device_ids,
        )
        camera = build_camera_model(device, logger, log=True)
        gimbal_data = build_gimbal_data(device, index, detector_settings)
        if gimbal_data is None:
            logger.warning(f"Device '{name}' has no gimbal config, skipping")
            continue

        gimbal = _build_gimbal(
            device,
            gimbal_data,
            vehicle,
            logger,
            use_sim_gimbal=use_sim_gimbal,
            scheduler_cadence=scheduler_cadence,
        )
        mount = CameraMount(
            name=name,
            camera=camera,
            gimbal=gimbal,
            zoom_calibration=build_zoom_calibration(device, logger),
        )
        mounts.append(
            CameraMountSpec(
                mount=mount,
                device=device,
                gimbal_device_id=metadata["gimbal_device_id"],
                profile_device_index=index,
            )
        )
        logger.info(
            f"Mount '{name}': {camera.image_width}x{camera.image_height}, "
            f"pitch={gimbal_data.att.pitch}, "
            f"fixed={isinstance(gimbal, FixedGimbal)}"
        )
    return mounts


__all__ = [
    "build_camera_model",
    "build_camera_mounts",
    "build_zoom_calibration",
]
