"""Commissioning injector: delay or lose decoded frames in one SITL run (hitl_final_approach.md section 9, step 15).

A bounded ``FrameCaptureBackend`` wrapper after decode. navpy's factory takes it as a dependency
(``open_capture_backend(wrap=)``, encoded GStreamer sources only); product code never imports this module. The
approach runner's explicit ``--commission-fault`` flag writes the fault into the run record, and then starts navpy
through this launcher instead of ``navpy.main``:

    python -s -m tools.hitl.media_fault_injector --fault-log FILE -- <navpy.main arguments>

The launcher refuses unless navpy runs the real detector with ``--integration-timing`` naming a valid run record whose
mode may commission (media_fault_spec.COMMISSIONING_MODES) and whose ``commissioning`` section holds a fault; it puts
the wrapper on navpy's parsed arguments and runs ``navpy.main.main``. navpy admits a wrapper only beside integration
timing, which admits only pc-sitl, so never in flight or Qualification.

Every frame keeps the ``CaptureStamp`` its backend made (PTS, open identity, read time): a late frame is published
with its own stamp, never a new one, so the delay shows as age. A held frame older than the spec's ``max_hold_s`` on
that stamp is discarded instead of published; frames still held when the capture ends are never published. A frame
without a source identity fails the capture. The fault log has one JSON line per decoded frame (what happened to it
and, for a delay, which frame was released in its place) and a closing summary.
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import deque
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any, TextIO

import numpy as np

from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.frame_capture_ports import (CaptureLogger, CaptureRunState, CaptureStamp, FrameCaptureBackend,
                                                  FramePublisher)
from tools.hitl.media_fault_spec import DELAY, LOSS, FaultSpec

PASS, HOLD, DROP = "pass", "hold", "drop"
RELEASED, DISCARDED_AGED = "released", "discarded_aged"


class _FaultLog:
    """One JSON line per event, flushed at once: a run that dies keeps what it injected."""

    def __init__(self, path: Path) -> None:
        self._file: TextIO = open(path, "x", encoding="utf-8")      # one log per run

    def write(self, record: dict[str, Any]) -> None:
        if not self._file.closed:
            self._file.write(json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n")
            self._file.flush()

    def close(self) -> None:
        self._file.close()


def _identity(capture: CaptureStamp) -> dict[str, Any]:
    source = capture.source
    assert source is not None
    return {"open_id": source.open_id, "sequence": source.sequence, "pts_ns": source.pts_ns,
            "read_s": capture.captured_at_s}


class FaultInjectingCapture:
    """Wraps one opened backend; see the module docstring."""

    def __init__(self, inner: FrameCaptureBackend, spec: FaultSpec, log: _FaultLog,
                 clock: Callable[[], float] = rt.perf_counter) -> None:
        if not spec.active or spec.problems():
            raise ValueError(f"no injectable fault: {spec.describe()} {spec.problems()}")
        self._inner, self._spec, self._log, self._clock = inner, spec, log, clock
        self._held: deque[tuple[np.ndarray | None, CaptureStamp]] = deque(maxlen=spec.delay_frames + 1)
        self.counts = {"decoded": 0, PASS: 0, HOLD: 0, DROP: 0, RELEASED: 0, DISCARDED_AGED: 0}
        log.write({"event": "open", **spec.record()})

    def run(self, publish: FramePublisher, is_running: CaptureRunState) -> None:
        def inject(frame: np.ndarray | None, capture: CaptureStamp | None) -> None:
            self._on_frame(publish, frame, capture)
        self._inner.run(inject, is_running)

    def _on_frame(self, publish: FramePublisher, frame: np.ndarray | None, capture: CaptureStamp | None) -> None:
        if capture is None or capture.source is None:
            raise ValueError("the commissioning injector needs each frame's source identity (encoded capture)")
        self.counts["decoded"] += 1
        record: dict[str, Any] = {"event": "frame", "at_s": self._clock(), **_identity(capture)}
        if self.counts["decoded"] <= self._spec.after_frames:
            record["action"] = PASS
            publish(frame, capture)
            self._count_and_log(record)
            return
        if self._spec.kind == LOSS:
            record["action"] = DROP
            self._count_and_log(record)
            return
        assert self._spec.kind == DELAY
        record["action"] = HOLD
        self._held.append((frame, capture))
        if len(self._held) <= self._spec.delay_frames:
            self._count_and_log(record)
            return
        late_frame, late_capture = self._held.popleft()
        age_s = record["at_s"] - late_capture.captured_at_s
        outcome = DISCARDED_AGED if age_s > self._spec.max_hold_s else RELEASED
        record[outcome] = {**_identity(late_capture), "age_s": age_s}
        self.counts[outcome] += 1
        if outcome == RELEASED:                      # published at once on its age check: no log I/O between
            publish(late_frame, late_capture)        # the stamp it was read with: never retimestamped
        self._count_and_log(record)

    def _count_and_log(self, record: dict[str, Any]) -> None:
        self.counts[record["action"]] += 1
        self._log.write(record)

    def request_stop(self) -> bool:
        return self._inner.request_stop()

    def close(self) -> None:
        try:
            self._inner.close()
        finally:
            try:
                self._log.write({"event": "close", **self.counts, "left_held": len(self._held)})
            finally:                                 # a failed summary still frees the frames and the file
                self._held.clear()
                self._log.close()


class MediaFaultWrapper:
    """navpy's ``CaptureBackendWrapper``: wraps the run's one open; a second open is refused (no reopen in a run)."""

    def __init__(self, spec: FaultSpec, log_path: Path, clock: Callable[[], float] = rt.perf_counter) -> None:
        problems = spec.problems()
        if not spec.active or problems:
            raise ValueError(f"no injectable fault: {spec.describe()} {problems}")
        self._spec, self._log_path, self._clock = spec, Path(log_path), clock
        self.wrapped: FaultInjectingCapture | None = None

    def __call__(self, backend: FrameCaptureBackend, logger: CaptureLogger) -> FrameCaptureBackend:
        if self.wrapped is not None:
            raise RuntimeError("the commissioning injector wraps one capture open per run")
        self.wrapped = FaultInjectingCapture(backend, self._spec, _FaultLog(self._log_path), self._clock)
        logger.warning(f"COMMISSIONING: injected media fault: {self._spec.describe()} (fault log {self._log_path})")
        return self.wrapped


def launch_problems(navpy_args: argparse.Namespace) -> tuple[FaultSpec | None, list[str]]:
    """The run record's fault, or why navpy may not run with the injector."""
    from tools.hitl import approach_records                      # its record spec imports media_fault_spec only
    from tools.hitl.json_checks import RecordError
    from tools.hitl.media_fault_spec import from_record, mode_problems

    problems = []
    if getattr(navpy_args, "detector_type", "") != "real":
        problems.append("the injector needs navpy's real detector (--detector-type real)")
    if not getattr(navpy_args, "integration_timing", "") or not getattr(navpy_args, "integration_log", ""):
        problems.append("the injector needs --integration-timing and --integration-log (an approach run)")
        return None, problems
    try:
        record = approach_records.read_record(Path(navpy_args.integration_timing))
    except (OSError, RecordError, ValueError) as exc:
        return None, [*problems, f"run record refused: {exc}"]
    spec = from_record(record["commissioning"])
    if not spec.active:
        problems.append("the run record names no commissioning fault (the runner's --commission-fault)")
    problems += mode_problems(spec, record["run"]["mode"])
    return spec, problems


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--fault-log", type=Path, required=True, help="a new JSONL file for the injected faults")
    parser.add_argument("navpy", nargs=argparse.REMAINDER, help="-- then navpy.main's arguments")
    args = parser.parse_args(argv)
    navpy_argv = args.navpy[1:] if args.navpy[:1] == ["--"] else args.navpy

    from navpy.app.navpy_argparse import make_parser
    from navpy.main import main as navpy_main

    navpy_args = make_parser(description="NavPy Control System (commissioning injector)").parse_args(navpy_argv)
    spec, problems = launch_problems(navpy_args)
    if problems or spec is None:
        print("error: commissioning injector refused: " + "; ".join(problems), file=sys.stderr, flush=True)
        return 2
    navpy_args.capture_wrapper = MediaFaultWrapper(spec, args.fault_log)
    return navpy_main(navpy_args)


if __name__ == "__main__":
    sys.exit(main())
