"""Exact public dependency bundle for :class:`VisionController`."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from navpy.core.skills.acquire.approach_strategy import ApproachKind
from navpy.core.perception.geo.geo_ref_calc import GeoRefCalc
from navpy.core.perception.camera.camera_mount import CameraMount
from navpy.core.perception.detection_coordinator import DetectionCoordinator
from navpy.core.perception.detector_ports import DetectorFleetMember
from navpy.core.perception.vision_lifecycle import VisionLifecycle
from navpy.core.perception.profile.vision_profile_types import VisionProfile


class VisionUiStepPort(Protocol):
    def ui_step(self) -> bool: ...


@dataclass(frozen=True)
class VisionControllerPorts:
    coordinator: DetectionCoordinator
    detectors: list[DetectorFleetMember]
    mounts: list[CameraMount]
    geo_ref: GeoRefCalc
    profile_name: str
    profile: VisionProfile
    approach_kind: ApproachKind
    lifecycle: VisionLifecycle
    ui: VisionUiStepPort


__all__ = ["VisionControllerPorts", "VisionUiStepPort"]
