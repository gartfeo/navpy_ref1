"""Typed factory and UI outputs for one configured vision detector."""

from __future__ import annotations

from navpy.exception_groups import ExceptionGroup

from dataclasses import dataclass
from typing import Optional, Protocol

from navpy.modules.vision.detector_ports import DetectorFleetMember
from navpy.modules.vision.vision_ui_ports import (
    DetectorUiStepPort,
    SimDebugSource,
)
from navpy.modules.vision.vision_profile_types import CameraMountSpec, VisionProfile


class DetectorConstructionCleanupPort(Protocol):
    def stop(self) -> None: ...


@dataclass(frozen=True)
class VisionDetectorNode:
    detector: DetectorFleetMember
    cleanup: DetectorConstructionCleanupPort
    real_ui: Optional[DetectorUiStepPort]
    sim_debug: Optional[SimDebugSource]


class VisionDetectorFactory(Protocol):
    def create(
        self,
        mount_spec: CameraMountSpec,
        detector_settings: VisionProfile,
        mount_index: int,
    ) -> VisionDetectorNode: ...


def build_detector_nodes(
    factory: VisionDetectorFactory,
    mount_specs: tuple[CameraMountSpec, ...],
    detector_settings: VisionProfile,
) -> tuple[VisionDetectorNode, ...]:
    nodes: list[VisionDetectorNode] = []
    try:
        for index, spec in enumerate(mount_specs):
            nodes.append(factory.create(spec, detector_settings, index))
    except Exception as creation_error:
        cleanup_errors = cleanup_detector_nodes(nodes)
        if cleanup_errors:
            raise ExceptionGroup(
                "detector construction and rollback failed",
                [creation_error, *cleanup_errors],
            ) from None
        raise
    return tuple(nodes)


def cleanup_detector_nodes(
    nodes: tuple[VisionDetectorNode, ...] | list[VisionDetectorNode],
) -> list[Exception]:
    errors: list[Exception] = []
    for node in reversed(nodes):
        try:
            node.cleanup.stop()
        except Exception as cleanup_error:
            errors.append(cleanup_error)
    return errors


def cleanup_mount_specs(
    mount_specs: tuple[CameraMountSpec, ...],
) -> list[Exception]:
    errors: list[Exception] = []
    for spec in reversed(mount_specs):
        try:
            spec.mount.stop()
        except Exception as cleanup_error:
            errors.append(cleanup_error)
    return errors


__all__ = [
    "DetectorConstructionCleanupPort",
    "DetectorUiStepPort",
    "VisionDetectorFactory",
    "VisionDetectorNode",
    "build_detector_nodes",
    "cleanup_detector_nodes",
    "cleanup_mount_specs",
]
