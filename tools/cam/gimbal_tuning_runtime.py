"""Resource lifecycle and interactive loop for the SIYI gimbal tuning tool."""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

import cv2

from navpy.modules.common.resource_cleanup import CleanupStack
from navpy.modules.vision import DetectRequest
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.detector import Detector

from .gimbal_tuning_args import MAX_ZOOM, MIN_ZOOM, parse_args
from .gimbal_tuning_assembly import (
    ConsoleLogger,
    apply_detector_overrides,
    apply_zoom,
    build_detector,
    build_mount,
    capture_calibrated_intrinsics,
    center_gimbal,
    default_model_path,
    parse_source,
)
from .gimbal_tuning_geometry import (
    ANCHOR_NAMES,
    TrackingIntrinsics,
    build_bbox_tracking_command,
    zoom_sequence,
)
from .gimbal_tuning_overlay import HotkeyAction, decode_hotkey, draw_overlay
from .gimbal_tuning_sample import (
    TuningTrackerPort,
    build_overlay_poi,
    tick_gimbal_tracker,
    tracking_bbox,
)
from .gimbal_tuning_tracking import build_tuning_tracker


WINDOW_NAME = "Gimbal Controller"


@dataclass
class TuningState:
    zoom: float
    anchor: int
    intrinsics: TrackingIntrinsics


def main() -> None:
    run(parse_args(), ConsoleLogger())


def run(args: argparse.Namespace, logger: ConsoleLogger) -> None:
    if not MIN_ZOOM <= args.zoom <= MAX_ZOOM:
        raise ValueError(f"zoom must be between {MIN_ZOOM:g} and {MAX_ZOOM:g}")
    logger.info("Loading profile '%s' ...", args.profile)
    mount, detector_settings, default_source = build_mount(
        args.profile,
        args.ip,
        args.port,
        logger,
    )
    with CleanupStack() as cleanup:
        cleanup.push(mount.stop)
        calibrated = capture_calibrated_intrinsics(mount)
        levels = zoom_sequence(calibrated)
        settings = apply_detector_overrides(
            detector_settings,
            args.conf,
            args.device,
        )
        tracker = build_tuning_tracker(
            mount.gimbal,
            logger,
            args.max_rate,
        )
        cleanup.push(tracker.stop)
        mount.start()
        if not mount.gimbal.is_connected():
            raise RuntimeError("Failed to connect to SIYI gimbal")
        center_gimbal(mount.gimbal, logger)
        time.sleep(1.0)
        intrinsics = apply_zoom(mount, calibrated, args.zoom, logger)
        if intrinsics is None:
            raise RuntimeError(f"Failed to set zoom to {args.zoom:g}x")
        state = TuningState(float(args.zoom), args.anchor, intrinsics)
        source = (
            parse_source(args.camera)
            if args.camera is not None
            else default_source
        )
        detector = build_detector(
            mount,
            source,
            args.model or default_model_path(),
            settings,
            logger,
        )
        cleanup.push(detector.stop)
        detector.start()
        cleanup.push(cv2.destroyAllWindows)
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        logger.info("Ready. 1-9 anchor  +/- zoom  c=center  q=quit")
        _run_loop(detector, mount, tracker, state, calibrated, levels, logger)
    logger.info("Done.")


def _run_loop(
    detector: Detector,
    mount: CameraMount,
    tracker: TuningTrackerPort,
    state: TuningState,
    calibrated: dict[float, TrackingIntrinsics],
    levels: list[float],
    logger: ConsoleLogger,
) -> None:
    while True:
        response = detector.get_detect_data(DetectRequest())
        poi = response.primary_poi
        if poi is None and response.detected_pois:
            poi = response.detected_pois[0]
        command = (
            None
            if poi is None
            else build_bbox_tracking_command(
                tracking_bbox(poi),
                state.intrinsics,
                state.anchor,
                mount.image_width or 1920,
                mount.image_height or 1080,
            )
        )
        result = tick_gimbal_tracker(tracker, command, poi)
        frame = detector.get_debug_frame()
        if frame is not None:
            display = draw_overlay(
                frame,
                build_overlay_poi(poi),
                command,
                result,
                state.anchor,
                state.zoom,
                state.intrinsics,
                mount.gimbal.get_data().att,
            )
            cv2.imshow(WINDOW_NAME, display)
        action = decode_hotkey(cv2.waitKey(1) & 0xFF, state.zoom, levels)
        if action is not None and _apply_action(
            action,
            state,
            calibrated,
            mount,
            tracker,
            logger,
        ):
            return
        time.sleep(0.005)


def _apply_action(
    action: HotkeyAction,
    state: TuningState,
    calibrated: dict[float, TrackingIntrinsics],
    mount: CameraMount,
    tracker: TuningTrackerPort,
    logger: ConsoleLogger,
) -> bool:
    if action.name == "quit":
        return True
    if action.name == "center":
        center_gimbal(mount.gimbal, logger)
        tracker.stop()
    elif action.name == "anchor" and isinstance(action.value, int):
        state.anchor = action.value
        logger.info(
            "Anchor mode: %s [%d]",
            ANCHOR_NAMES[action.value],
            action.value,
        )
    elif action.name == "zoom" and action.value is not None:
        zoom = float(action.value)
        if zoom != state.zoom:
            intrinsics = apply_zoom(mount, calibrated, zoom, logger)
            if intrinsics is not None:
                state.zoom = zoom
                state.intrinsics = intrinsics
                tracker.stop()
    return False


__all__ = ["TuningState", "WINDOW_NAME", "main", "run"]
