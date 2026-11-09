"""The rfdetr backend's profile settings: the model block and the acquisition gates (HITL step 23b)."""
from __future__ import annotations

from collections.abc import Mapping

from navpy.app.perception.fixed_zoom_profile import build_fixed_zoom
from navpy.core.kernel.config.detector_backend import RFDETR_BACKEND
from navpy.core.skills.geo_approach.orbit_centre_frame import OrbitCentreFrame
from navpy.core.perception.profile.vision_class_profile import DOCK_DETECT_CLASS_ID, DOCK_PRESET_NAME
from navpy.core.perception.tracking.dock_acquisition_gate import (
    ACQUISITION_KEY,
    DockAcquisitionSettings,
    acquisition_block,
)
from navpy.plugins.detection.rfdetr.rfdetr_settings import RfDetrSettings, rfdetr_settings

# The navpy classes a COCO class may map to: only the dock.
NAVPY_CLASSES = {DOCK_PRESET_NAME: DOCK_DETECT_CLASS_ID}


def whole_dock_settings(detector: Mapping[str, object]) -> RfDetrSettings:
    """The profile's rfdetr block, its classes mapped to navpy's dock class."""
    return rfdetr_settings(detector, NAVPY_CLASSES)


def rfdetr_model_settings(backend: str, detector: Mapping[str, object]) -> RfDetrSettings | None:
    """The profile's rfdetr block for the rfdetr backend (required there), None for every other backend."""
    return whole_dock_settings(detector) if backend == RFDETR_BACKEND else None


def dock_acquisition(backend: str, detector: Mapping[str, object]) -> DockAcquisitionSettings | None:
    """The whole-dock gates: required with the rfdetr backend, refused with any other."""
    block = acquisition_block(detector)
    if backend != RFDETR_BACKEND:
        if block is not None:
            raise ValueError(f"detector.{ACQUISITION_KEY} is the rfdetr backend's; not for {backend}")
        return None
    if block is None:
        raise ValueError(f"the rfdetr backend needs the profile's detector.{ACQUISITION_KEY} block")
    model = whole_dock_settings(detector)
    return DockAcquisitionSettings(
        min_lock_s=float(block["min_lock_s"]),
        min_box_px=float(block["min_box_px"]),
        min_confidence=float(detector.get("conf", 0.35)),
        dock_class_id=DOCK_DETECT_CLASS_ID,
        input_scale=model.input_scale,
    )


def orbit_centre_frame(profile: Mapping[str, object]) -> OrbitCentreFrame | None:
    """The whole-dock detector's centre-frame rule (the acquisition floor at the widest measured zoom step), or None
    for a profile without the detector's acquisition block."""
    detector = profile.get("detector") if isinstance(profile, Mapping) else None
    block = acquisition_block(detector) if isinstance(detector, Mapping) else None
    if block is None:
        return None
    built = [zoom for zoom in (build_fixed_zoom(device) for device in profile.get("devices", ())) if zoom]
    if len(built) != 1:
        raise ValueError(f"detector.{ACQUISITION_KEY} needs exactly one mount with measured zoom steps, "
                         f"found {len(built)}")
    widest = built[0].steps.steps[0].optics
    return OrbitCentreFrame(float(block["min_box_px"]), float(widest.fx), float(widest.fy),
                            whole_dock_settings(detector).input_scale)


__all__ = ["NAVPY_CLASSES", "dock_acquisition", "orbit_centre_frame", "rfdetr_model_settings",
           "whole_dock_settings"]
