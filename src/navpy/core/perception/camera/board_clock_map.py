"""Board (autopilot) clock to ATTITUDE receipt wall clock, from ATTITUDE itself.

A simulated camera frame (Unreal, NPYC v2) carries the board time of the
pose it was rendered from: the HIL ``time_usec`` of the bridge frame, the
same autopilot clock as ATTITUDE ``time_boot_ms``. Capture stamps live on
the ATTITUDE receipt wall clock (``CaptureStamp``), so the board time is
mapped with the ATTITUDE samples navpy already keeps:

    wall(board_t) = board_t + min_i(receipt_i - time_boot_i)

over the newest run of samples of one boot. The minimum is the lower
envelope of the link: the receipt time an ATTITUDE generated at board_t
would have had at the smallest delay observed in the window. The window is
recent (a few seconds at most), so board/host clock drift (tens of ppm)
stays far below a millisecond; ``time_boot_ms`` quantisation adds up to
1 ms. Consequence for the capture-lookup calibration of such a source:
``attitude_link_delay_s`` is the typical ATTITUDE delay ABOVE that minimum
(near zero on a local link), not the full link delay.

No mapping (None) while too few samples exist, for a board time too far
from the newest ATTITUDE, or, when the window shows a reboot, for a board
time before the new boot's first sample (it may belong to the old boot):
such a frame never gets an EXPOSURE stamp. The mapping reads timing only,
never a pose.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import Any

# How many recent ATTITUDE samples define the envelope. The vehicle keeps a
# longer history; 64 matches the frame association's window
# (real_frame_association.ATTITUDE_HISTORY_COUNT): ~1.3 s at 50 Hz.
BOARD_CLOCK_HISTORY_COUNT = 64
# Fewer samples than this make the minimum a poor envelope (one late sample
# can be the minimum): ~0.2 s at 50 Hz before frames are EXPOSURE-stamped.
BOARD_CLOCK_MIN_SAMPLES = 10
# A frame's board time this far from the newest ATTITUDE is not mapped. The
# HITL bridge treats a board-time jump over 2 s as a new boot
# (hitl_bridge.board_clock.MAX_FORWARD_JUMP_US); a frame that old or that
# far ahead belongs to another boot or a stalled stream.
BOARD_CLOCK_MAX_GAP_S = 2.0
US_PER_S = 1_000_000

AttitudeHistoryReader = Callable[[int], Sequence[Any]]


class AttitudeBoardClock:
    """``BoardClock`` from the vehicle's ATTITUDE history (time_boot_s, receipt_time_s)."""

    def __init__(
        self,
        history: AttitudeHistoryReader,
        count: int = BOARD_CLOCK_HISTORY_COUNT,
    ) -> None:
        self._history = history
        self._count = int(count)

    def wall_time_s(self, board_time_us: int) -> float | None:
        run, rebooted = _newest_boot_run(self._history(self._count))
        if len(run) < BOARD_CLOCK_MIN_SAMPLES:
            return None
        board_s = board_time_us / US_PER_S
        if abs(board_s - run[-1][0]) > BOARD_CLOCK_MAX_GAP_S:
            return None
        if rebooted and board_s < run[0][0]:
            return None
        return board_s + min(receipt - boot for boot, receipt in run)


def _newest_boot_run(samples: Sequence[Any]) -> tuple[list[tuple[float, float]], bool]:
    """(boot, receipt) pairs after the last backwards step of the boot clock, and whether one was seen."""
    run: list[tuple[float, float]] = []
    rebooted = False
    for sample in samples:
        try:
            boot = float(sample.time_boot_s)
            receipt = float(sample.receipt_time_s)
        except (AttributeError, TypeError, ValueError):
            continue
        if run and boot < run[-1][0]:
            run = []
            rebooted = True
        run.append((boot, receipt))
    return run, rebooted


__all__ = [
    "AttitudeBoardClock",
    "BOARD_CLOCK_HISTORY_COUNT",
    "BOARD_CLOCK_MAX_GAP_S",
    "BOARD_CLOCK_MIN_SAMPLES",
]
