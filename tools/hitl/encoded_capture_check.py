"""Live check of the encoded capture (hitl_final_approach.md section 9, stage 1 step 9).

Runs navpy's real detector and tracker on the RTSP stream, through the same factory a flight uses, and scores what came
out against the publisher's own records (aas-hitl aas_hitl/video/gstreamer_encoder.py, encoded-NNN.csv):

- 0 fallbacks: the factory opened the GStreamer backend once, and no FFmpeg or OpenCV capture;
- every decoded PTS is unique, and its interval from the first frame matches the sender's within one RTP tick per side;
- 0 sender frames missing between the first and the last decoded frame;
- 0 duplicate LOS updates: no picture and no measurement time reached the measurement port twice;
- with --expect-disconnect: the stream's end fails the capture, and it never opens again.

Inside the WSL distro or on the Orin, from the repository root, with a Python that has navpy's dependencies and sees the
distro's python3-gi:

    PYTHONPATH=src:. python -m tools.hitl.encoded_capture_check --seconds 600 --publisher-run <dir> --out <dir>

The aircraft and the gimbal are stubs: this checks the frame path to the measurement port, not LOS values, the capture
clock (step 10), integration timing (step 11) or any command. It proves nothing about physical docking.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Sequence

from tools.hitl.encoded_capture_score import read_sender_pts, rescore, score_intervals, score_los
from tools.hitl.encoded_capture_stubs import stub_mount, stub_vehicle

DISCONNECT_REASONS = ("stream ended", "no frame for")     # the capture's own transport ends


class _MeasurementRecorder:
    """Stands in for the gimbal's measurement port behind the real TrackingNavigationSink."""

    def __init__(self, rows: list[tuple[int, float]]) -> None:
        self.rows = rows
        self.tracking_obj_id: int | None = None
        self.picture: int | None = None

    def update(self, poi: Any) -> None:
        if poi is not None:
            self.rows.append((int(self.picture or 0), float(poi.timing.detection_timestamp_s)))


def _instrument(los_rows: list[tuple[int, float]], frame_rows: list[tuple[str, int, int, float]]) -> dict[str, Any]:
    """Wires the recorders in at the factory and the tracking sink; the capture and detector code stay real."""
    from navpy.app.perception import frame_capture_factory as factory
    from navpy.core.perception.detection.poi_priority import get_poi_task_id
    from navpy.core.perception.pipeline import real_tracking, real_tracking_batch
    from navpy.plugins.camera.gstreamer.encoded_frame_gate import EncodedFrameGate, EncodedStreamReset
    from navpy.plugins.camera.gstreamer.frame_capture_gstreamer import GstreamerFrameCapture

    facts: dict[str, Any] = {"opens": [], "fallback_opens": 0, "refused_frames": []}
    recorder = _MeasurementRecorder(los_rows)

    def observe(identity: Any, read_at_s: float) -> None:
        frame_rows.append((identity.open_id, identity.sequence, identity.pts_ns, read_at_s))

    class _ObservedCapture:
        @staticmethod
        def open(source: str, logger: Any) -> Any:
            capture = GstreamerFrameCapture.open(source, logger, observe)
            facts["opens"].append(capture)
            if capture is not None:
                capture.run = _watched(capture.run)
            return capture

    def _watched(run: Any) -> Any:
        def watched(*args: Any) -> Any:
            try:
                return run(*args)
            except EncodedStreamReset as error:
                facts["capture_reset"] = (str(error), time.perf_counter())
                raise
        return watched

    def refused(*_args: Any, **_kwargs: Any) -> None:
        facts["fallback_opens"] += 1

    gate_admit = EncodedFrameGate.admit

    def admit(self: Any, frame: Any) -> Any:
        sequence = gate_admit(self, frame)
        if sequence is None:
            facts["refused_frames"].append((frame.pts_ns, frame.corrupted, time.perf_counter()))
        return sequence

    EncodedFrameGate.admit = admit
    factory.GstreamerFrameCapture = _ObservedCapture
    factory.FfmpegFrameCapture = SimpleNamespace(open=refused)
    factory.open_source_capture = refused
    factory.open_jetson_capture = refused

    sink_init = real_tracking.TrackingNavigationSink.__init__
    sink_update = real_tracking.TrackingNavigationSink.update
    process = real_tracking_batch.TrackingBatchProcessor.process

    def init(self: Any, _navigation: Any, freshness: Any) -> None:
        sink_init(self, recorder, freshness)

    def update(self: Any, pois: Any) -> None:
        recorder.tracking_obj_id = get_poi_task_id(pois[0]) if pois else None
        sink_update(self, pois)

    def process_batch(self: Any, batch: Any, now_s: float) -> None:
        if batch.association is not None:
            recorder.picture = batch.association.frame_sequence
        process(self, batch, now_s)

    real_tracking.TrackingNavigationSink.__init__ = init
    real_tracking.TrackingNavigationSink.update = update
    real_tracking_batch.TrackingBatchProcessor.process = process_batch
    return facts


def _build_detector(profile_name: str, logger: Any) -> tuple[Any, str]:
    from navpy.core.perception.pipeline.detector import Detector
    from navpy.core.perception.pipeline.real_detector_config import (
        DetectorDependencies,
        DetectorModelConfig,
        DetectorPipelineConfig,
        RealDetectorConfig,
    )
    from navpy.core.perception.profile.vision_profile_loader import resolve_profile
    from navpy.plugins.detection.charuco.charuco_board import (
        charuco_board_spec_from_settings,
        charuco_detection_settings_from_settings,
    )

    _key, profile, _path = resolve_profile(profile_name, logger)
    settings = profile["detector"]
    source = settings["camera_index"]
    config = RealDetectorConfig(
        model=DetectorModelConfig(
            model_path="",
            backend="charuco",
            charuco_board=charuco_board_spec_from_settings(settings),
            charuco_detection=charuco_detection_settings_from_settings(settings),
            conf=float(settings.get("conf", 0.35)),
            tracker=settings.get("tracker"),
        ),
        pipeline=DetectorPipelineConfig(
            detect_hz=float(settings["detect_hz"]),
            track_hz=float(settings["track_hz"]),
            reference_height_m=float(settings["reference_height_m"]),
            frame_source=source,
        ),
    )
    return Detector(DetectorDependencies(stub_vehicle(), stub_mount(), logger), config), source


class _Log:
    def __init__(self) -> None:
        self.lines: list[str] = []

    def _add(self, message: str, *_args: Any, **_kwargs: Any) -> None:
        self.lines.append(f"{time.time():.3f} {message}")
        print(message, flush=True)

    info = warning = error = debug = _add


def _stream_span_s(frame_rows: Sequence[tuple[str, int, int, float]]) -> float:
    return (frame_rows[-1][2] - frame_rows[0][2]) / 1e9 if len(frame_rows) > 1 else 0.0


def run(args: argparse.Namespace) -> dict[str, Any]:
    los_rows: list[tuple[int, float]] = []
    frame_rows: list[tuple[str, int, int, float]] = []
    facts = _instrument(los_rows, frame_rows)
    log = _Log()
    detector, source = _build_detector(args.profile, log)
    failure = None
    started = time.monotonic()
    detector.start()
    try:
        while _stream_span_s(frame_rows) < args.seconds and (
            frame_rows or time.monotonic() - started < args.first_frame_wait_s
        ):
            time.sleep(0.5)
            try:
                detector.raise_if_failed()
            except Exception as error:
                failure = str(error)
                time.sleep(args.after_failure_s)    # the stream may come back: nothing may reopen it
                break
    finally:
        detector.stop()
    args.out.mkdir(parents=True, exist_ok=True)
    with (args.out / "frames.csv").open("w", newline="", encoding="utf-8") as file:
        csv.writer(file).writerows([("open_id", "sequence", "pts_ns", "read_at_s"), *frame_rows])
    with (args.out / "los.csv").open("w", newline="", encoding="utf-8") as file:
        csv.writer(file).writerows([("picture", "detection_timestamp_s"), *los_rows])
    with (args.out / "refused.csv").open("w", newline="", encoding="utf-8") as file:
        csv.writer(file).writerows([("pts_ns", "corrupted", "at_s"), *facts["refused_frames"]])
    (args.out / "log.txt").write_text("\n".join(log.lines), encoding="utf-8")
    return verdict(args, source, facts, frame_rows, los_rows, failure, time.monotonic() - started)


def _disconnect_proven(facts: dict[str, Any], failure: str | None, frame_rows: Sequence[Any]) -> bool:
    """The capture itself ended on a transport end, that ended the run, and no frame came after it."""
    reset = facts.get("capture_reset")
    if reset is None or failure is None or reset[0] not in failure or not reset[0].startswith(DISCONNECT_REASONS):
        return False
    return not [row for row in frame_rows if row[3] > reset[1]]


def verdict(
    args: argparse.Namespace,
    source: str,
    facts: dict[str, Any],
    frame_rows: Sequence[tuple[str, int, int, float]],
    los_rows: Sequence[tuple[int, float]],
    failure: str | None,
    elapsed_s: float,
) -> dict[str, Any]:
    opens = facts["opens"]
    capture = opens[0] if opens else None
    gate = None if capture is None else capture.gate
    decoded = [row[2] for row in frame_rows]
    intervals = score_intervals(decoded, read_sender_pts(args.publisher_run)) if args.publisher_run else None
    los = score_los(los_rows)
    span_s = _stream_span_s(frame_rows)
    checks = {
        "one_gstreamer_open": len(opens) == 1 and capture is not None,
        "zero_fallbacks": facts["fallback_opens"] == 0,
        "one_open_identity": len({row[0] for row in frame_rows}) == 1,
        "no_duplicate_frames": gate is not None and gate.duplicates == 0,
        "no_duplicate_los": los["los_updates"] > 0 and not (los["duplicate_pictures"] or los["duplicate_times"]),
        "intervals_kept": intervals is not None and intervals.ok,
        "ended_as_expected": _disconnect_proven(facts, failure, frame_rows) if args.expect_disconnect else failure is None,
        "long_enough": args.expect_disconnect or span_s >= args.seconds,
    }
    return {
        "pass": all(checks.values()),
        "checks": checks,
        "source": source,
        "settings": None if capture is None else capture.recorded,
        "frames": len(decoded),
        "stream_span_s": span_s,
        "unique_fps": (len(decoded) - 1) / span_s if span_s > 0 else 0.0,
        "gate": None if gate is None else {"duplicates": gate.duplicates, "corrupted": gate.corrupted},
        "connect_attempts": None if capture is None else capture.connect_attempts,
        "fallback_opens": facts["fallback_opens"],
        "intervals": None if intervals is None else asdict(intervals),
        "los": los,
        "failure": failure,
        "capture_reset": facts.get("capture_reset"),
        "elapsed_s": elapsed_s,
        "last_frame_to_end_s": time.perf_counter() - frame_rows[-1][3] if frame_rows else None,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--profile", default="siyi_zr10_hitl_encoded")
    parser.add_argument("--seconds", type=float, default=600.0)
    parser.add_argument("--first-frame-wait-s", type=float, default=280.0, help="give up if no frame came by then")
    parser.add_argument("--publisher-run", type=Path, help="the publisher's output folder")
    parser.add_argument("--expect-disconnect", action="store_true")
    parser.add_argument("--after-failure-s", type=float, default=0.0, help="keep watching after the run failed")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--rescore", action="store_true", help="only score the intervals of a finished run again")
    args = parser.parse_args(argv)
    result = rescore(args.out, args.publisher_run) if args.rescore else run(args)
    (args.out / "result.json").write_text(json.dumps(result, indent=1), encoding="utf-8")
    print(json.dumps(result, indent=1), flush=True)
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
