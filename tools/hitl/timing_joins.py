"""Step 12's joins: navpy frames to the frames Unreal rendered, and navpy's gimbal readbacks to emulator plant times.

Every join is by identity, never by subtracting one host's clock from another's:
- navpy frame -> publisher frame by PTS. navpy's decoded PTS has its own zero (rtspsrc running time), so the join
  is by PTS delta from one anchor, within one RTP tick per side; every frame must fit, and of the anchors that fit,
  exactly one may imply the fewest sender frames navpy never got.
- publisher frame -> Unreal sent slot by frame index and board time, both equal: the represented board time.
- Unreal slot -> emulator steps by gimbal session and the two step sequences it rendered between, and their weight:
  the rendered plant time on the emulator's device clock.
- navpy gimbal readback -> emulator 0x0D frame by SIYI wire sequence: both sides unwrapped in their own order; the
  one 65536 offset that puts every readback on a sent 0x0D frame to one destination must be unique.

A join that finds nothing is missing, one that finds more than one is ambiguous; both raise :class:`JoinError`.
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from navpy.plugins.gimbal.siyi.siyi_message import COMMAND
from tools.hitl.encoded_capture_score import INTERVAL_TOLERANCE_NS
from tools.hitl.timing_inputs import EmulatorLog, GimbalReceipt, PublisherFrame, RenderedSlot, SiyiOut

SIYI_SEQ_MODULUS = 65536
NS_PER_US = 1000
# Navpy frames tried against every anchor before the whole series is: enough to reject almost every wrong anchor.
ANCHOR_PROBE_FRAMES = 64


class JoinError(ValueError):
    def __init__(self, kind: str, message: str) -> None:
        super().__init__(f"{kind}: {message}")
        self.kind = kind


MISSING = "join_missing"
AMBIGUOUS = "join_ambiguous"
INVALID = "join_invalid"


@dataclass(frozen=True)
class NavpyFrame:
    sequence: int
    pts_ns: int


def _nearest(sorted_pts: np.ndarray, targets: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Index of the nearest sender PTS to each target and its distance."""
    right = np.clip(np.searchsorted(sorted_pts, targets), 0, len(sorted_pts) - 1)
    left = np.clip(right - 1, 0, len(sorted_pts) - 1)
    right_error = np.abs(sorted_pts[right] - targets)
    left_error = np.abs(sorted_pts[left] - targets)
    pick = np.where(left_error <= right_error, left, right)
    return pick, np.minimum(left_error, right_error)


@dataclass(frozen=True)
class PublisherJoin:
    frames: dict[int, PublisherFrame]      # navpy frame sequence -> the publisher frame it is
    lost: int                              # sender frames between joined ones that navpy never logged a frame for


def join_publisher(frames: Sequence[NavpyFrame], publisher: Sequence[PublisherFrame]) -> PublisherJoin:
    """navpy frame sequence -> the one publisher frame it is, by PTS delta from one unique anchor."""
    if not frames:
        raise JoinError(MISSING, "no navpy frames")
    if not publisher:
        raise JoinError(MISSING, "no pushed publisher frames")
    frames = sorted(frames, key=lambda frame: frame.sequence)
    for earlier, later in zip(frames, frames[1:]):
        if later.pts_ns <= earlier.pts_ns:
            raise JoinError(INVALID, f"navpy PTS does not rise with sequence at {later.sequence}")
    sender = np.asarray([frame.pts_ns for frame in publisher], dtype=np.int64)
    if np.any(np.diff(sender) <= 0):
        raise JoinError(INVALID, "publisher PTS does not rise in its log")
    deltas = np.asarray([frame.pts_ns - frames[0].pts_ns for frame in frames], dtype=np.int64)
    sequences = np.asarray([frame.sequence for frame in frames], dtype=np.int64)
    probe = deltas[:ANCHOR_PROBE_FRAMES]
    fits = []
    for anchor in range(len(sender)):
        _, errors = _nearest(sender, sender[anchor] + probe)
        if np.any(errors > INTERVAL_TOLERANCE_NS):
            continue
        rows, errors = _nearest(sender, sender[anchor] + deltas)
        if np.any(errors > INTERVAL_TOLERANCE_NS):
            continue
        # One sender frame per navpy frame, and never more navpy frames than sender frames between two of them.
        extra = np.diff(rows) - np.diff(sequences)
        if np.any(extra < 0):
            continue
        fits.append((int(extra.sum()), rows))
    if not fits:
        raise JoinError(MISSING, "no publisher anchor fits every navpy frame's PTS")
    # An anchor may also fit by putting a gap of navpy's on sender frames navpy never got. The true anchor implies
    # the fewest such lost frames; a tie cannot be told apart.
    fewest = min(lost for lost, _ in fits)
    best = [rows for lost, rows in fits if lost == fewest]
    if len(best) > 1:
        raise JoinError(AMBIGUOUS, f"{len(best)} publisher anchors fit every navpy frame's PTS with {fewest} "
                                   f"sender frames navpy never got")
    return PublisherJoin({int(sequence): publisher[int(row)] for sequence, row in zip(sequences, best[0])}, fewest)


def join_slots(frames: dict[int, PublisherFrame], slots: Sequence[RenderedSlot]) -> dict[int, RenderedSlot]:
    """navpy frame sequence -> the Unreal sent slot of its publisher frame (same frame index and board time).

    The slots joined must share one camera run and pose generation, and each must have rendered a valid stamped
    pose and a valid gimbal bracket with a weight in [0, 1].
    """
    by_frame: dict[int, list[RenderedSlot]] = {}
    for slot in slots:
        by_frame.setdefault(slot.frame, []).append(slot)
    joined: dict[int, RenderedSlot] = {}
    for sequence, frame in frames.items():
        matches = [slot for slot in by_frame.get(frame.frame_index, []) if slot.board_us == frame.board_time_us]
        if not matches:
            raise JoinError(MISSING, f"no Unreal sent slot for frame {frame.frame_index} at {frame.board_time_us} us")
        if len(matches) > 1:
            raise JoinError(AMBIGUOUS, f"{len(matches)} Unreal sent slots for frame {frame.frame_index}")
        slot = matches[0]
        if not (slot.pose_valid and slot.clock_mapped and slot.gimbal_valid
                and math.isfinite(slot.gimbal_weight) and 0.0 <= slot.gimbal_weight <= 1.0):
            raise JoinError(INVALID, f"Unreal slot of frame {frame.frame_index} has no valid pose/clock/gimbal")
        joined[sequence] = slot
    streams = {(slot.run, slot.gen, slot.sys_id) for slot in joined.values()}
    if len(streams) > 1:
        raise JoinError(AMBIGUOUS, f"joined slots come from {len(streams)} camera runs/generations")
    return joined


def rendered_plant_time(slot: RenderedSlot, emulator: EmulatorLog) -> float:
    """The plant time (emulator device clock) of the gimbal state Unreal rendered for ``slot``."""
    if slot.gimbal_session != emulator.session:
        raise JoinError(MISSING, f"slot gimbal session {slot.gimbal_session} is not the emulator's")
    try:
        older = emulator.step_device_t[slot.gimbal_older]
        newer = emulator.step_device_t[slot.gimbal_newer]
    except KeyError as exc:
        raise JoinError(MISSING, f"emulator has no step {exc.args[0]}") from exc
    if slot.gimbal_newer < slot.gimbal_older or newer < older:
        raise JoinError(INVALID, f"slot gimbal bracket {slot.gimbal_older},{slot.gimbal_newer} runs backwards")
    return older + slot.gimbal_weight * (newer - older)


def _unwrap(seqs: Sequence[int]) -> list[int]:
    unwrapped = [seqs[0]]
    for seq in seqs[1:]:
        unwrapped.append(unwrapped[-1] + (seq - unwrapped[-1]) % SIYI_SEQ_MODULUS)
    return unwrapped


@dataclass(frozen=True)
class GimbalJoin:
    """Each navpy readback (one session, log order) and the plant time its 0x0D frame reported."""

    session: int
    samples: tuple[GimbalReceipt, ...]
    plant_t: tuple[float, ...]
    destination: tuple[str, int]


def join_gimbal(receipts: Sequence[GimbalReceipt], emulator: EmulatorLog) -> GimbalJoin:
    if not receipts:
        raise JoinError(MISSING, "no navpy gimbal receipts")
    sessions = {sample.session for sample in receipts}
    if len(sessions) > 1:
        raise JoinError(AMBIGUOUS, f"navpy gimbal reconnected ({len(sessions)} sessions); one per check")
    for earlier, later in zip(receipts, receipts[1:]):
        if later.receipt_s <= earlier.receipt_s:
            raise JoinError(INVALID, "navpy gimbal receipts do not rise in log order")
    sent = emulator.siyi_out
    if not sent:
        raise JoinError(MISSING, "the emulator log has no SIYI frames")
    # The emulator's encoder numbers every frame it builds: the whole outgoing stream unwraps step by step.
    emulator_seq = _unwrap([frame.seq for frame in sent])
    if any(later - earlier != 1 for earlier, later in zip(emulator_seq, emulator_seq[1:])):
        raise JoinError(INVALID, "the emulator's SIYI frames are not numbered one by one (frames missing)")
    attitude: dict[int, SiyiOut] = {
        seq: frame for seq, frame in zip(emulator_seq, sent) if frame.cmd == COMMAND.ACQUIRE_GIMBAL_ATT and frame.sent
    }
    navpy_seq = _unwrap([sample.seq for sample in receipts])
    fits = []
    for candidate in attitude:
        if (candidate - navpy_seq[0]) % SIYI_SEQ_MODULUS:
            continue
        offset = candidate - navpy_seq[0]
        frames = [attitude.get(seq + offset) for seq in navpy_seq]
        if all(frame is not None for frame in frames) and len({frame.dest for frame in frames}) == 1:
            fits.append(frames)
    if not fits:
        raise JoinError(MISSING, "no emulator 0x0D frames carry every navpy readback's sequence")
    if len(fits) > 1:
        raise JoinError(AMBIGUOUS, f"{len(fits)} sequence offsets fit every navpy readback")
    plant = tuple(frame.report_t for frame in fits[0])
    if any(later < earlier for earlier, later in zip(plant, plant[1:])):
        raise JoinError(INVALID, "joined readbacks report plant times out of order")
    return GimbalJoin(receipts[0].session, tuple(receipts), plant, fits[0][0].dest)


__all__ = [
    "AMBIGUOUS",
    "ANCHOR_PROBE_FRAMES",
    "GimbalJoin",
    "INVALID",
    "JoinError",
    "MISSING",
    "NavpyFrame",
    "PublisherJoin",
    "SIYI_SEQ_MODULUS",
    "join_gimbal",
    "join_publisher",
    "join_slots",
    "rendered_plant_time",
]
