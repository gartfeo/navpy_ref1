"""Resolve one vision profile and its camera-mount assembly."""

from __future__ import annotations

import os
from dataclasses import dataclass
from collections.abc import Mapping
from typing import TYPE_CHECKING, cast

from navpy.core.kernel.config.uas_args import UasArgs
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.kernel.scheduler_cadence import SchedulerCadence
from navpy.core.skills.acquire.approach_strategy import ApproachKind, classify_approach
from navpy.core.perception.geo.geo_ref_calc import GeoRefCalc
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.app.perception.vision_camera_factory import build_camera_mounts
from navpy.core.perception.profile.vision_detector_profile import get_detector_settings
from navpy.core.perception.profile.vision_profile_loader import resolve_profile
from navpy.core.perception.profile.vision_profile_types import CameraMountSpec, VisionProfile
from navpy.app.perception.sim_video_test_source import with_sim_video_stream
from navpy.app.perception.vision_tracking_composition import build_tracking_config

if TYPE_CHECKING:
    from navpy.plugins.gimbal.siyi.hardware.command_fence import SiyiCommandFence


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
    *,
    use_sim_gimbal: bool,
    siyi_fence: SiyiCommandFence | None = None,
) -> VisionProfileAssembly:
    """use_sim_gimbal: False for a real run, which then never gets a simulated gimbal (design section 4, B3)."""
    resolved_name, raw_profile, _ = resolve_profile(
        profile_name,
        logger,
    )
    if use_sim_gimbal:  # simulated companions only; real aircraft use their profile
        raw_profile = with_sim_video_stream(raw_profile, os.environ)
    profile = cast(VisionProfile, raw_profile)
    mount_specs = tuple(
        build_camera_mounts(
            raw_profile,
            vehicle,
            logger,
            use_sim_gimbal=use_sim_gimbal,
            scheduler_cadence=scheduler_cadence,
            siyi_fence=siyi_fence,
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
