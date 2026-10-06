"""Resolve one vision profile and its camera-mount assembly."""

from __future__ import annotations

from dataclasses import dataclass
from collections.abc import Mapping
from typing import cast

from navpy.args.uas_args import UasArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.navigation.approach_strategy import ApproachKind, classify_approach
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.vision_camera_factory import build_camera_mounts
from navpy.modules.vision.vision_detector_profile import get_detector_settings
from navpy.modules.vision.vision_profile_loader import resolve_profile
from navpy.modules.vision.vision_profile_types import CameraMountSpec, VisionProfile
from navpy.modules.vision.vision_tracking_composition import build_tracking_config


@dataclass(frozen=True)
class VisionProfileAssembly:
    name: str
    profile: VisionProfile
    mount_specs: tuple[CameraMountSpec, ...]
    detector_settings: VisionProfile
    geo_ref: GeoRefCalc
    approach_kind: ApproachKind


def build_vision_profile(
    profile_name: str,
    vehicle: IVehicle,
    logger: ILogger,
    scheduler_cadence: SchedulerCadence | None,
) -> VisionProfileAssembly:
    resolved_name, raw_profile, _ = resolve_profile(
        profile_name,
        logger,
    )
    profile = cast(VisionProfile, raw_profile)
    mount_specs = tuple(
        build_camera_mounts(
            raw_profile,
            vehicle,
            logger,
            use_sim_gimbal=True,
            scheduler_cadence=scheduler_cadence,
        )
    )
    if not mount_specs:
        raise ValueError(
            f"No camera mounts configured in profile '{resolved_name}'"
        )
    mounts = [spec.mount for spec in mount_specs]
    devices = [spec.device for spec in mount_specs]
    return VisionProfileAssembly(
        name=resolved_name,
        profile=profile,
        mount_specs=mount_specs,
        detector_settings=cast(
            VisionProfile,
            get_detector_settings(raw_profile),
        ),
        geo_ref=GeoRefCalc(UasArgs()),
        approach_kind=classify_approach(mounts, devices),
    )


__all__ = [
    "VisionProfileAssembly",
    "build_tracking_config",
    "build_vision_profile",
]
