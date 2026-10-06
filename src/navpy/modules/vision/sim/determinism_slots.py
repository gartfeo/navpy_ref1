"""Integer-slot algebra on the autopilot clock, and the frame payload digest.

Split from the recorder because this is the KEY, not the log: every landing of
the one-clock work compares samples by these values, so they have to be exact
and testable on their own.

Everything here is integer. Floats are never bucketed, because
``time_boot_ms * 1e-3`` and ``time_us * 1e-6`` round differently for ~40% of
tick values -- the reason ``truth_pose_time_axis.SOURCE_CLOCK_QUANTUM_S``
exists at all. That helper returns whole-microsecond-valued FLOATS, so slot
keys are re-derived here as real ints rather than reused.
"""

from __future__ import annotations

import struct
from typing import Any


MICROSECONDS_PER_SECOND = 1_000_000
# Presence markers for an optional command channel. Distinct bytes, because a
# commanded 0.0 and a commanded nothing are different commands.
# A digest of exactly these bytes means the payload was PRESENT but could
# not be read. Distinct from None, which means there was no payload to
# read: two runs whose frames were both undigestable have not been shown
# to agree, whereas two runs that both staged nothing have.
PAYLOAD_UNREADABLE = b"unreadable-payload"
ABSENT_CHANNEL = bytes((0,))
PRESENT_CHANNEL = bytes((1,))


def source_microseconds(seconds: Any) -> int | None:
    """Whole autopilot microseconds, or None when the stamp is unusable.

    ``bool`` is rejected explicitly: it is an ``int`` subclass, and a True
    that silently became slot 0 would corrupt every key derived from it.
    """
    if seconds is None or isinstance(seconds, bool):
        return None
    try:
        value = float(seconds)
    except (TypeError, ValueError):
        return None
    if value != value or value in (float("inf"), float("-inf")):
        return None
    return int(round(value * MICROSECONDS_PER_SECOND))


def slot_index(microseconds: int | None, period_us: int) -> int | None:
    """Half-open [k*P, (k+1)*P) slot, in integer arithmetic only."""
    if microseconds is None or period_us <= 0:
        return None
    return microseconds // period_us


def scheduler_slot_period_us(scheduler_rate_hz: Any) -> int:
    """The grid period in whole microseconds, from the autopilot loop rate.

    The grid is the AUTOPILOT SCHEDULER period, not the pose period: pose runs
    at 0.8x the scheduler rate (pose_streams.py), so roughly one slot in five
    is legitimately empty and deterministically holds. Keying on the pose
    period would hide exactly that.

    Returns 0 for an unusable rate, which disables slot keying rather than
    inventing a period.
    """
    if isinstance(scheduler_rate_hz, bool):
        return 0
    try:
        rate = float(scheduler_rate_hz)
    except (TypeError, ValueError):
        return 0
    if rate != rate or rate <= 0.0 or rate == float("inf"):
        return 0
    return int(round(MICROSECONDS_PER_SECOND / rate))


def source_seconds_of(sample: Any) -> Any:
    """The autopilot source stamp of an associated pose or a rendered frame.

    One extractor for both shapes because the same sample is discarded at two
    different points in the flow, and the trace has to key them identically.
    """
    if sample is None:
        return None
    stamp = getattr(sample, "attitude_timestamp_s", None)
    if stamp is not None:
        return stamp
    pixel = getattr(sample, "pixel", None)
    return None if pixel is None else getattr(pixel, "source_timestamp_s", None)


def frame_digest(frame: Any) -> bytes | None:
    """Exact bits of the payload a delivered frame hands the navigation law.

    Bytes and not a float tuple, because float equality is the wrong relation
    for a digest in both directions: it equates -0.0 with 0.0, and it makes two
    bit-identical NaNs compare unequal. The question being asked is "did two
    runs produce the SAME command input", so the answer must be bitwise.

    Every answer comes from what the read ACTUALLY DID. There is no attempt to
    decide first whether a payload was "declared", because Python cannot answer
    that without running code: a payload reachable only through ``__getattr__``
    appears in no class dict and no instance dict, and a probe that asks the
    type invokes descriptors and metaclasses that can raise. Such a probe was
    tried here and produced three defects at once -- it reported a READABLE
    dynamic payload as absent, reported a RAISING one as absent, and let a
    raising metaclass escape the function into the recorder.

    ``None`` means nothing was there to digest: no frame at all, or a frame
    whose pixel read back as ``None``. Both are observations, and two runs that
    both staged nothing HAVE been shown to agree.

    ``PAYLOAD_UNREADABLE`` means the attempt to read RAISED -- a hole in the
    evidence, not an outcome, so the caller invalidates the run on it. This
    includes a frame carrying no ``pixel`` at all. That is deliberate, and it is
    the conservative direction: the only frame a real run legitimately has
    nothing to digest for is ``None``, answered above without touching the
    object. Every non-None frame on this path is a ``DetectedObject`` built by
    ``IdealTargetProjector`` (direct_pixel_render.py:59 constructs it; an earlier
    version of this docstring named ``finite_target_projector``, which is a
    different path), and it always carries a pixel. So a frame that yields no
    readable payload is malformed -- and a malformed frame must invalidate a
    verdict rather than pass as a clean absence.
    """
    if frame is None:
        return None
    try:
        # The whole read is inside the try, including REACHING the attribute:
        # on an object the recorder does not own, arriving at ``pixel`` can
        # raise just as reading its fields can.
        pixel = frame.pixel
        if pixel is None:
            return None
        return struct.pack(
            "<ddddd",
            float(pixel.u_px),
            float(pixel.v_px),
            float(pixel.aircraft_pitch_deg),
            float(pixel.aircraft_roll_deg),
            float(pixel.source_timestamp_s),
        )
    except Exception:  # noqa: BLE001
        # ANY failure reading a foreign object means "cannot digest this
        # payload". Naming a tuple of types here was a defect:
        # OverflowError is an ArithmeticError, so float(10**400) escaped a
        # digest whose whole contract is to answer instead of raise. But
        # answering None was ALSO wrong: the caller used to latch on the
        # raise, and None is what "nothing to digest" already means, so
        # returning it here silently turned a hole into a clean run.
        # A BaseException still propagates, and deleting the declaration
        # probe made that NEWLY REACHABLE: a dynamic getter raising
        # SystemExit used to be skipped as "not declared" and now runs. It
        # is left propagating on purpose -- swallowing KeyboardInterrupt or
        # SystemExit in a library is worse than an instrument raising -- and
        # production cannot reach it, because a ``DetectedObject`` pixel is
        # a plain slot read with no code behind it.
        return PAYLOAD_UNREADABLE


def command_digest(command: Any) -> bytes | None:
    """Exact bits of the command a worker iteration actually issued.

    Takes the object ``execute_work`` RETURNED. Nothing here recomputes a
    command or re-reads a control input: a recorder that recomputed would be
    measuring its own arithmetic, and one that re-read would sample inputs the
    command never saw.

    ``None`` for an optional channel is a DISTINCT outcome from 0.0, so each
    optional field carries a presence byte ahead of its value. Packing 0.0 for
    an absent channel would equate "commanded level" with "commanded nothing".
    """
    if command is None:
        return None
    parts = [b"cmd"]
    try:
        for name in ("yaw", "pitch"):
            parts.append(struct.pack("<d", float(getattr(command, name))))
        for name in ("cmd_roll", "cmd_pitch", "cmd_thr"):
            value = getattr(command, name)
            if value is None:
                parts.append(ABSENT_CHANNEL)
                continue
            parts.append(PRESENT_CHANNEL + struct.pack("<d", float(value)))
    except Exception:  # noqa: BLE001
        # See frame_digest: undigestable is an ANSWER, not a fault.
        return None
    return b"".join(parts)


__all__ = [
    "MICROSECONDS_PER_SECOND",
    "PAYLOAD_UNREADABLE",
    "command_digest",
    "frame_digest",
    "scheduler_slot_period_us",
    "slot_index",
    "source_microseconds",
    "source_seconds_of",
]
