"""The grid check on a recorded serial-step firmware frame sequence (step 9g).

``tests/fixtures/lockstep/serial_step_frames.csv`` holds the first 1200
frames' ``(tick, source_us)`` of a real lockstep slice run (seed 1, the
firmware in ``scripts/lockstep_firmware.lock.json``; boot, then mission).
The manifest's Δ_max (``lockstep_wsl_sitl.MAX_FRAME_GAP_US``) is set from it:
the largest recorded gap, so every recorded frame passes and a longer gap
or a stall fails.
"""

from __future__ import annotations

import csv
from pathlib import Path

import pytest

from navpy.sim.lockstep.coordinator_frames import FrameReader
from navpy.sim.lockstep.coordinator_types import CoordinatorFault
from navpy.sim.lockstep.step_wire import Frame, PortState
from scripts.lockstep_wsl_sitl import MAX_FRAME_GAP_US

RECORDING = Path(__file__).resolve().parents[2] / "fixtures" / "lockstep" / "serial_step_frames.csv"
_PORTS = (PortState(0, 0, 0, b""), PortState(0, 0, 0, b""))


def recorded_frames() -> list[tuple[int, int]]:
    with RECORDING.open(encoding="utf-8", newline="") as stream:
        return [(int(row["tick"]), int(row["source_us"])) for row in csv.DictReader(stream)]


def _check(frames: list[tuple[int, int]], max_gap_us: int) -> None:
    reader = FrameReader(link=None, max_frame_gap_us=max_gap_us)
    for tick, source_us in frames:
        reader.check(Frame((0, 0), tick, source_us, 0, True, _PORTS))


def test_the_recording_is_contiguous_and_its_gaps_are_whole_physics_frames():
    frames = recorded_frames()
    assert len(frames) == 1200 and [tick for tick, _ in frames] == list(range(1, 1201))
    gaps = {b - a for (_, a), (_, b) in zip(frames, frames[1:])}
    assert gaps == {19_992, 20_825}  # 24 and 25 SITL physics steps of 833 us


def test_the_manifest_max_frame_gap_is_the_recorded_maximum_and_the_recording_passes():
    frames = recorded_frames()
    assert MAX_FRAME_GAP_US == max(b - a for (_, a), (_, b) in zip(frames, frames[1:]))
    _check(frames, MAX_FRAME_GAP_US)


def test_a_tighter_bound_fails_at_the_first_long_gap():
    frames = recorded_frames()
    first_long = next(b_tick for (_, a), (b_tick, b) in zip(frames, frames[1:])
                      if b - a == MAX_FRAME_GAP_US)
    with pytest.raises(CoordinatorFault, match=f"grid check at tick {first_long}"):
        _check(frames, MAX_FRAME_GAP_US - 1)


@pytest.mark.parametrize("change", ["stall", "skipped_frame"])
def test_a_stall_or_a_skipped_physics_frame_fails(change):
    frames = recorded_frames()[:50]
    tick, source_us = frames[30]
    previous = frames[29][1]
    frames[30] = (tick, previous if change == "stall" else previous + MAX_FRAME_GAP_US + 833)
    with pytest.raises(CoordinatorFault, match=f"grid check at tick {tick}"):
        _check(frames, MAX_FRAME_GAP_US)
