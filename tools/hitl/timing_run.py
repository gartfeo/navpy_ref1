"""One receiver's step 12 logs joined into one run (tools/hitl/check_integration_timing.py).

Every navpy frame that mapped joins to its publisher frame, Unreal slot and rendered plant time, and navpy's gimbal
readbacks join to the emulator's 0x0D frames (timing_joins.py). One capture open and one run id per check; map
refusals after the first mapped frame are interruptions.
"""
from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
from functools import cached_property

from navpy.core.perception.camera.integration_timing import (
    STATUS_ABORTED,
    STATUS_UNAVAILABLE,
    STATUS_UNKNOWN_FRAME,
    STATUS_UNSETTLED,
    IntegrationTimingConfig,
)
from tools.hitl.timing_inputs import Association, EmulatorLog, NavpyLog, PublisherFrame, RenderedSlot
from tools.hitl.timing_joins import (
    AMBIGUOUS,
    INVALID,
    MISSING,
    GimbalJoin,
    JoinError,
    NavpyFrame,
    join_gimbal,
    join_publisher,
    join_slots,
    rendered_plant_time,
)

CORRECTION_MATCH_S = 1e-9            # a logged lookup - mapped equals the frozen C to float precision

# Refusals a map gives before it has settled; after the first mapped frame any of them is an interruption.
_MAP_REFUSALS = frozenset({STATUS_UNSETTLED, STATUS_UNKNOWN_FRAME, STATUS_UNAVAILABLE, STATUS_ABORTED})


@dataclass
class Joined:
    """One receiver's logs with every mapped frame joined to its render stamp."""

    config: IntegrationTimingConfig
    log: NavpyLog
    frames: list[Association]                     # one association per navpy frame sequence, log order
    board_s: dict[int, float]                     # sequence -> represented board time B
    rendered_t: dict[int, float]                  # sequence -> rendered plant time T_r
    gimbal: GimbalJoin
    sender_frames_unlogged: int                   # pushed frames between joined ones without a navpy frame
    interrupted: list[str] = field(default_factory=list)

    @cached_property
    def gimbal_receipts(self) -> list[float]:
        return [sample.receipt_s for sample in self.gimbal.samples]

    @cached_property
    def gimbal_index(self) -> dict[float, int]:
        """Receipt -> position in the joined gimbal samples (receipts rise strictly, so each is unique)."""
        return {sample.receipt_s: index for index, sample in enumerate(self.gimbal.samples)}


def _frames(log: NavpyLog, config: IntegrationTimingConfig) -> tuple[list[Association], list[str]]:
    """The mapped attempts, one per frame sequence, and the map interruptions after the first one."""
    opens = {item.get("open_id") for item in log.associations if item.get("open_id") is not None}
    if len(opens) > 1:
        raise JoinError(AMBIGUOUS, f"{len(opens)} capture opens in one navpy log; one per check")
    if log.run_ids != {config.run_id}:
        raise JoinError(INVALID, f"navpy log run ids {sorted(log.run_ids)} are not the record's {config.run_id!r}")
    frames: dict[int, Association] = {}
    interrupted: list[str] = []
    mapped_once = False
    for item in log.associations:
        status = item.status
        if status in _MAP_REFUSALS and (mapped_once or status == STATUS_ABORTED):
            interrupted.append(f"log line {item.order}: {status}")
        if item.get("mapped_s") is None:
            continue
        mapped_once = True
        sequence = item.get("frame_sequence")
        if not isinstance(sequence, int) or not isinstance(item.get("pts_ns"), int):
            raise JoinError(INVALID, f"log line {item.order}: mapped frame without sequence/PTS")
        known = frames.get(sequence)
        if known is not None:
            if known.get("pts_ns") != item.get("pts_ns") or known.get("mapped_s") != item.get("mapped_s"):
                raise JoinError(AMBIGUOUS, f"frame {sequence} logged with two PTS or mapped times")
            continue
        frames[sequence] = item
    if not frames:
        raise JoinError(MISSING, "no frame ever mapped")
    return list(frames.values()), interrupted


def join_run(config: IntegrationTimingConfig, log: NavpyLog, publisher: Sequence[PublisherFrame],
             slots: Sequence[RenderedSlot], emulator: EmulatorLog) -> Joined:
    frames, interrupted = _frames(log, config)
    navpy = [NavpyFrame(int(item.get("frame_sequence")), int(item.get("pts_ns"))) for item in frames]
    published = join_publisher(navpy, publisher)
    rendered = join_slots(published.frames, slots)
    board_s = {sequence: slot.board_us / 1e6 for sequence, slot in rendered.items()}
    rendered_t = {sequence: rendered_plant_time(slot, emulator) for sequence, slot in rendered.items()}
    gimbal = join_gimbal(log.gimbal, emulator)
    return Joined(config, log, frames, board_s, rendered_t, gimbal, published.lost, interrupted)


__all__ = ["CORRECTION_MATCH_S", "Joined", "join_run"]
