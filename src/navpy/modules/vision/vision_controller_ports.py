"""Exact public dependency bundle for :class:`VisionController`."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from navpy.modules.navigation.approach_strategy import ApproachKind
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.detection_coordinator import DetectionCoordinator
from navpy.modules.vision.detector_ports import DetectorFleetMember
from navpy.modules.vision.vision_lifecycle import VisionLifecycle
from navpy.modules.vision.vision_profile_types import VisionProfile


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
