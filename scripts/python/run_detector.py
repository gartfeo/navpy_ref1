# ============================================================
# Demo: detector with RTSP or webcam video
#
# Usage:
#   python scripts/python/run_detector.py                                    # webcam 1
#   python scripts/python/run_detector.py --camera rtsp://192.168.144.25:8554/main.264  # SIYI
#   python scripts/python/run_detector.py --camera 0                         # webcam 0
# ============================================================
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


if not __package__:
    repository_root = Path(__file__).resolve().parents[2]
    for import_root in (repository_root, repository_root / "src"):
        import_path = str(import_root)
        while import_path in sys.path:
            sys.path.remove(import_path)
        sys.path.insert(0, import_path)

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision import DetectRequest
from navpy.modules.vision.detector import (
    Detector,
    DetectorDebugConfig,
    DetectorDependencies,
    DetectorModelConfig,
    DetectorPipelineConfig,
    RealDetectorConfig,
)
from navpy.modules.vision.vision_profiles import (
    resolve_profile, build_camera_mounts, get_detector_settings,
)


class _DummyVehicle:
    @property
    def attitude(self):
        return Attitude(0, 0, 0)

    def location(self, _):
        return None


class _Log:
    def info(self, msg, *a): print(msg % a if a else msg)
    def warning(self, msg, *a): print(msg % a if a else msg)
    def error(self, msg, *a): print(msg % a if a else msg)
    def debug(self, msg, *a): pass


def _parse_source(raw: str):
    """Convert camera arg to int (webcam) or str (RTSP/file)."""
    try:
        return int(raw)
    except ValueError:
        return raw


def main():
    parser = argparse.ArgumentParser(description="Run detector demo")
    parser.add_argument("--profile", default="siyi_zr10", help="Vision profile name (default: from json)")
    parser.add_argument("--camera", default="rtsp://192.168.144.25:8554/main.264", help="Video source: RTSP URL or webcam index (default: SIYI RTSP)")
    parser.add_argument("--device", default="cpu", help="YOLO device: cpu, 0, auto")
    parser.add_argument("--model", default=None, help="YOLO model path")
    args = parser.parse_args()

    current_path = Path(__file__).resolve().parents[2]
    model = args.model or str((current_path / ".models" / "yolov8n-face-lindevs.pt").resolve())

    vehicle = _DummyVehicle()
    logger = _Log()

    # Load vision profile and build mount
    key, profile, _ = resolve_profile(args.profile, logger)
    det_settings = get_detector_settings(profile)
    mount_pairs = build_camera_mounts(profile, vehicle, logger, use_sim_gimbal=False)
    if not mount_pairs:
        logger.error("No mounts built from profile '%s'", key)
        return
    mount, _device = mount_pairs[0]

    # Detector with frame_source — no external capture thread needed
    det = Detector(
        DetectorDependencies(vehicle, mount, logger),
        RealDetectorConfig(
            model=DetectorModelConfig(
                model_path=model,
                imgsz=det_settings.get("imgsz", 640),
                conf=det_settings.get("conf", 0.35),
                device=args.device,
            ),
            pipeline=DetectorPipelineConfig(
                detect_hz=det_settings.get("detect_hz", 10.0),
                track_hz=det_settings.get("track_hz", 60.0),
                reference_height_m=det_settings.get("reference_height_m", 2.0),
                frame_source=_parse_source(args.camera),
                use_target_lock=True,
                output_mode="all",
            ),
            debug=DetectorDebugConfig(
                show=True,
                window_name="Detector Debug",
                allow_esc_stop=True,
            ),
        ),
    )

    det.start()

    try:
        last_print = 0.0
        while True:
            if not det.ui_step():
                break

            now = time.time()
            if now - last_print > 0.5:
                last_print = now
                resp = det.get_detect_data(DetectRequest())
                if resp.detected_targets:
                    print("targets:",
                          [(t.identity.obj_id, t.classification.class_id,
                            float(round(t.pixel.u_px, 1)), float(round(t.pixel.v_px, 1)))
                           for t in resp.detected_targets])

            time.sleep(0.005)

    finally:
        det.stop()
        try:
            import cv2
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    main()
