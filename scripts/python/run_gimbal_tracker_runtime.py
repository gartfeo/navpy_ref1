"""Interactive lifecycle and source-timestamped navigation loop."""

from __future__ import annotations

import queue
import time
from dataclasses import dataclass, field

import cv2

from navpy.modules.common.resource_cleanup import CleanupStack
from navpy.modules.navigation.gimbal_navigation import GimbalNavigation
from navpy.modules.vision import DetectRequest
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.detector import Detector
from navpy.modules.vision.focus_monitor import FocusMonitor, laplacian_focus

from scripts.python.run_gimbal_tracker_args import (
    DETECTOR_PRESETS,
    parse_args,
)
from scripts.python.run_gimbal_tracker_assembly import (
    ConsoleLogger,
    assemble_runner,
    build_detector,
    build_navigation,
    initialize_gimbal,
    resolve_model_path,
)
from scripts.python.run_gimbal_tracker_commands import (
    RunnerControlState,
    TrackSnapshot,
    build_anchor_k,
    drain_commands,
    start_command_reader,
)
from scripts.python.run_gimbal_tracker_profile import apply_detector_overrides


@dataclass
class LoopState:
    controls: RunnerControlState
    last_source_timestamp_s: float | None = None

    @classmethod
    def from_anchor(cls, anchor: int) -> LoopState:
        return cls(RunnerControlState(anchor))


def feed_navigation(
    state: LoopState,
    detector: Detector,
    navigation: GimbalNavigation,
    mount: CameraMount,
) -> None:
    force_bbox = state.controls.click_bbox
    state.controls.click_bbox = None
    response = detector.get_detect_data(
        DetectRequest(force_lock_bbox_cxcywh=force_bbox)
    )
    if not response.detected_pois:
        navigation.update(None, now=time.time())
        return
    poi = response.detected_pois[0]
    state.controls.last_locked_id = poi.identity.obj_id
    bbox = poi.tracking.bbox_cxcywh or poi.confirmation.bbox_cxcywh
    half_size = 0.0 if bbox is None else max(bbox[2], bbox[3]) / 2.0
    frame = detector.get_raw_frame()
    height, width = (
        frame.shape[:2]
        if frame is not None
        else (mount.image_height or 1080, mount.image_width or 1920)
    )
    anchored = build_anchor_k(
        poi.pixel.calibration.matrix(),
        mount.get_dist(),
        state.controls.anchor,
        width,
        height,
        half_size,
        half_size,
    )
    timestamp_s = poi.timing.detection_timestamp_s
    if timestamp_s is None or timestamp_s == state.last_source_timestamp_s:
        navigation.update(None, now=time.time())
        return
    state.last_source_timestamp_s = float(timestamp_s)
    navigation.update(
        poi,
        now=float(timestamp_s),
        principal_point=(float(anchored[0, 2]), float(anchored[1, 2])),
    )


def run_loop(
    detector: Detector,
    navigation: GimbalNavigation,
    mount: CameraMount,
    focus: FocusMonitor,
    commands: queue.Queue[str],
    state: LoopState,
    logger: ConsoleLogger,
) -> None:
    while True:
        frame = detector.get_debug_frame()
        if frame is not None:
            cv2.imshow("Gimbal Tracker", frame)
        zoom = mount.get_current_zoom_command() or mount.get_current_zoom()
        focus.tick(detector.get_raw_frame(), zoom)
        cv2.waitKey(1)
        tracks = [
            TrackSnapshot(
                int(track.id),
                float(track.cx),
                float(track.cy),
                float(track.w),
                float(track.h),
            )
            for track in detector.get_overlay_tracks()
        ]
        drain_commands(
            commands,
            state.controls,
            tracks,
            mount.gimbal.request_autofocus,
            mount.set_zoom,
            logger.warning,
        )
        feed_navigation(state, detector, navigation, mount)
        time.sleep(0.005)


def main() -> None:
    args = parse_args()
    logger = ConsoleLogger()
    preset_model, classes = DETECTOR_PRESETS[args.preset]
    assembly = assemble_runner(args, logger)
    with CleanupStack() as cleanup:
        cleanup.push(assembly.mount.stop)
        settings = apply_detector_overrides(args, assembly.detector_settings)
        initialize_gimbal(args, assembly.mount, logger)
        navigation = build_navigation(
            args,
            assembly.mount,
            logger,
            assembly.tracking_setup,
            assembly.zoom_config,
        )
        cleanup.push(navigation.stop_tracking)
        detector = build_detector(
            args,
            assembly.mount,
            logger,
            settings,
            resolve_model_path(args.model, preset_model),
            classes,
            args.stream or assembly.default_stream,
        )
        cleanup.push(detector.stop)
        cleanup.push(
            lambda: logger.info("Session stats: %s", detector.stats())
        )
        detector.start()
        cleanup.push(cv2.destroyAllWindows)
        cv2.namedWindow("Gimbal Tracker", cv2.WINDOW_NORMAL)
        commands: queue.Queue[str] = queue.Queue()
        state = LoopState.from_anchor(args.anchor)
        start_command_reader(commands, logger.warning)
        focus = FocusMonitor(
            assembly.mount.gimbal.request_autofocus,
            focus_metric=laplacian_focus,
        )
        try:
            run_loop(
                detector,
                navigation,
                assembly.mount,
                focus,
                commands,
                state,
                logger,
            )
        except KeyboardInterrupt:
            logger.info("Interrupted")


__all__ = ["LoopState", "feed_navigation", "main", "run_loop"]
