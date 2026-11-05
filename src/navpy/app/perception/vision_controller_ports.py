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
from navpy.core.ports.dock_contact import DockContactSensor
from navpy.app.approach.law_airspeed import LawAirspeed


class VisionUiStepPort(Protocol):
    def ui_step(self) -> bool: ...


class ClosingLogPort(Protocol):
    def close(self) -> None: ...


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
    dock_contact: DockContactSensor | None = None
    # HITL only: closed once vision has stopped quiescent, never before.
    integration_log: ClosingLogPort | None = None
    # HITL only (step 20): the eligible airspeed and its write guard, for navigation.
    law_airspeed: LawAirspeed | None = None


__all__ = ["ClosingLogPort", "VisionControllerPorts", "VisionUiStepPort"]
