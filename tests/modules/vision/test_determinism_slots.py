"""Unit tests for the determinism trace's slot algebra and payload digest.

These pin the KEY, not the log: every landing of the one-clock work compares
samples by these values, so a rounding or type slip here would silently
corrupt every downstream verdict.
"""

from __future__ import annotations

import math
import struct
from types import SimpleNamespace

from navpy.modules.vision.sim.determinism_slots import (
    PAYLOAD_UNREADABLE,
    frame_digest,
    scheduler_slot_period_us,
    slot_index,
    source_microseconds,
    source_seconds_of,
)


PERIOD_US = 20_000  # 50 Hz autopilot scheduler period.


def _frame(
    source_timestamp_s: float,
    *,
    u_px: float = 1.5,
    v_px: float = -2.5,
    pitch_deg: float = 3.0,
    roll_deg: float = 4.0,
) -> SimpleNamespace:
    return SimpleNamespace(
        pixel=SimpleNamespace(
            u_px=u_px,
            v_px=v_px,
            aircraft_pitch_deg=pitch_deg,
            aircraft_roll_deg=roll_deg,
            source_timestamp_s=source_timestamp_s,
        )
    )


def test_source_microseconds_rejects_unusable_stamps() -> None:
    assert source_microseconds(None) is None
    # bool is an int subclass: a True that became slot 0 would corrupt keys.
    assert source_microseconds(True) is None
    assert source_microseconds(False) is None
    assert source_microseconds(float("nan")) is None
    assert source_microseconds(float("inf")) is None
    assert source_microseconds(float("-inf")) is None
    assert source_microseconds("not a number") is None


def test_source_microseconds_are_whole_microseconds_as_real_ints() -> None:
    micros = source_microseconds(12.5)
    assert micros == 12_500_000
    assert type(micros) is int
    # The float product 0.007 vs 0.006999999999999999 is exactly why the key
    # is re-derived as an integer instead of reusing the float quantum.
    assert source_microseconds(7 * 1e-3) == source_microseconds(7000 * 1e-6)


def test_slot_index_is_half_open_and_integer() -> None:
    assert slot_index(0, PERIOD_US) == 0
    assert slot_index(PERIOD_US - 1, PERIOD_US) == 0
    assert slot_index(PERIOD_US, PERIOD_US) == 1
    assert type(slot_index(PERIOD_US, PERIOD_US)) is int
    assert slot_index(None, PERIOD_US) is None
    # An unusable period disables keying rather than inventing one.
    assert slot_index(PERIOD_US, 0) is None


def test_scheduler_slot_period_us_from_loop_rate() -> None:
    assert scheduler_slot_period_us(50.0) == 20_000
    assert scheduler_slot_period_us(400.0) == 2_500
    assert scheduler_slot_period_us(True) == 0
    assert scheduler_slot_period_us(0.0) == 0
    assert scheduler_slot_period_us(-50.0) == 0
    assert scheduler_slot_period_us(float("nan")) == 0
    assert scheduler_slot_period_us(float("inf")) == 0
    assert scheduler_slot_period_us("fifty") == 0


def test_source_seconds_of_reads_both_carriers_of_one_stamp() -> None:
    # The same frame is discarded at two points in the flow, so both shapes
    # must key identically.
    associated = SimpleNamespace(attitude_timestamp_s=12.5)
    assert source_seconds_of(associated) == 12.5
    assert source_seconds_of(_frame(12.5)) == 12.5
    assert source_seconds_of(None) is None
    assert source_seconds_of(SimpleNamespace()) is None


# Digest bytes written out as LITERALS, and that is the whole point of them.
#
# Two rounds of review broke assertions here that compared one ``frame_digest``
# result against another, or against a value the test built with the same
# ``struct.pack`` the digest uses. Both are satisfied by a degenerate answer: a
# packer that returns PAYLOAD_UNREADABLE makes expected and actual agree, and a
# reviewer did exactly that and watched every test pass. A digest test that
# shares its expectation with the code under test cannot fail when that code is
# wrong.
#
# These bytes are little-endian IEEE-754 doubles in the order frame_digest packs
# them -- u_px, v_px, pitch, roll, source timestamp. Regenerate with:
#     struct.pack("<ddddd", 1.0, 2.0, 3.0, 4.0, 5.0)
# and if the FORMAT ever changes, these must be regenerated deliberately, which
# is the intended cost: the digest is a comparison key that other landings rely
# on, so changing its bytes should never be quiet.
# The five fields as 1.0 through 5.0.
#     struct.pack("<ddddd", 1.0, 2.0, 3.0, 4.0, 5.0)
DIGEST_ONE_TO_FIVE = (
    b"\x00\x00\x00\x00\x00\x00\xf0\x3f"
    b"\x00\x00\x00\x00\x00\x00\x00\x40"
    b"\x00\x00\x00\x00\x00\x00\x08\x40"
    b"\x00\x00\x00\x00\x00\x00\x10\x40"
    b"\x00\x00\x00\x00\x00\x00\x14\x40"
)
# What ``_frame(1.0, u_px=math.nan)`` produces: a quiet NaN in u_px, then
# the helper's defaults v_px=-2.5, pitch=3.0, roll=4.0, and timestamp 1.0.
#     struct.pack("<ddddd", math.nan, -2.5, 3.0, 4.0, 1.0)
DIGEST_NAN_U_PX = (
    b"\x00\x00\x00\x00\x00\x00\xf8\x7f"
    b"\x00\x00\x00\x00\x00\x00\x04\xc0"
    b"\x00\x00\x00\x00\x00\x00\x08\x40"
    b"\x00\x00\x00\x00\x00\x00\x10\x40"
    b"\x00\x00\x00\x00\x00\x00\xf0\x3f"
)


def test_frame_digest_separates_signed_zero_and_matches_identical_nan() -> None:
    # A float tuple gets both of these wrong for digest purposes: it equates
    # -0.0 with 0.0, and it makes two identical NaNs compare unequal.
    assert frame_digest(_frame(1.0, u_px=0.0)) != frame_digest(
        _frame(1.0, u_px=-0.0)
    )
    # Named bytes, not "is not None" and not one digest against another. A
    # review made frame_digest return PAYLOAD_UNREADABLE for a NaN u_px and this
    # test passed, so the NaN comparison it claims to make never happened.
    nan_digest = frame_digest(_frame(1.0, u_px=math.nan))
    assert nan_digest == DIGEST_NAN_U_PX
    assert nan_digest == frame_digest(_frame(1.0, u_px=math.nan))
    assert frame_digest(None) is None


def test_a_getter_that_raises_attribute_error_is_still_a_hole() -> None:
    """The one case ``getattr`` with a default silently got wrong.

    ``getattr(frame, "pixel", None)`` cannot tell "this object has no pixel"
    from "its pixel property raised AttributeError". Both came back as None,
    which is the answer that means "nothing to digest", so a payload that
    existed and could not be read reported a COMPLETE run.

    An intermediate version answered this by settling off the TYPE, and the
    claim written here was that doing so "does not invoke the getter". That was
    false for a custom descriptor or metaclass, which the type probe invokes like
    any other, and the probe was removed. The read itself is the answer now:
    what raises is a hole, whatever raised it.
    """

    class RaisesAttributeError:
        @property
        def pixel(self) -> object:
            raise AttributeError("pixel is not available")

    class RaisesSomethingElse:
        @property
        def pixel(self) -> object:
            raise RuntimeError("no")

    assert frame_digest(RaisesAttributeError()) == PAYLOAD_UNREADABLE
    assert frame_digest(RaisesSomethingElse()) == PAYLOAD_UNREADABLE


def test_a_declared_but_unset_slot_is_a_hole_not_an_absence() -> None:
    """Production frames are ``DetectedObject``, which uses ``__slots__``.

    Reading an unset slot raises AttributeError, and the descriptor is on the
    class, so "declared" is true and the answer is a hole. A frame that is
    SUPPOSED to carry a payload and does not is not the same as a stage with no
    frame at all.
    """

    class Slotted:
        __slots__ = ("pixel",)

    assert frame_digest(Slotted()) == PAYLOAD_UNREADABLE


def test_no_payload_and_an_unreadable_payload_are_different_answers() -> None:
    """This assertion CHANGED TWICE, and both old versions were wrong.

    Version one required ``None`` for a pixel object whose fields cannot be
    read. That made "there was nothing to digest" and "there was something and I
    could not read it" indistinguishable, and the consequence was measured: an
    undigestable frame reported a COMPLETE run, so two runs with different
    unreadable payloads left identical evidence and a clean verdict.

    Version two, below, is the line that changed: an object carrying no
    ``pixel`` was required to answer ``None``. Deciding that needed a probe that
    ran code, and the probe was wrong three ways at once (see
    ``test_a_payload_reachable_only_through_getattr_is_read_not_guessed``). The
    answer now comes only from what the read did, so an object with no pixel
    raises and lands on the HOLE side.

    That is a move TOWARD strictness, not away from it: an input travelled from
    the side that reports a clean run to the side that invalidates it. The only
    frame a real run legitimately has nothing to digest for is ``None``, which
    is answered without touching the object at all.
    """
    assert frame_digest(None) is None
    assert frame_digest(SimpleNamespace(pixel=None)) is None

    assert frame_digest(SimpleNamespace()) == PAYLOAD_UNREADABLE
    assert frame_digest(SimpleNamespace(pixel=SimpleNamespace())) == (
        PAYLOAD_UNREADABLE
    )
    assert frame_digest(_frame(1.0, u_px=10**400)) == PAYLOAD_UNREADABLE


def test_a_payload_reachable_only_through_getattr_is_read_not_guessed() -> None:
    """The three defects a "was it declared?" probe caused, all at once.

    The probe asked ``hasattr(type(frame), "pixel")`` plus the instance
    ``__dict__``. A payload served by ``__getattr__`` is in NEITHER, so:

    - a READABLE dynamic payload was reported ABSENT. That is the worst of the
      three and it is not a hole, it is a false clean: two runs handing the
      navigation law DIFFERENT payloads both digested to ``None`` and compared
      equal.
    - a RAISING dynamic payload was reported ABSENT -- the hole-reads-clean
      defect this whole marker exists to prevent, reopened for a new shape.
    - the probe itself ran code, so a metaclass or descriptor that raises while
      being asked escaped ``frame_digest`` entirely, into a recorder whose one
      hard requirement is that it cannot perturb the run it measures.

    Reading the attribute answers all three without asking anything first.
    """

    class DynamicReadable:
        def __getattr__(self, name: str) -> object:
            if name == "pixel":
                return SimpleNamespace(
                    u_px=1.0,
                    v_px=2.0,
                    aircraft_pitch_deg=3.0,
                    aircraft_roll_deg=4.0,
                    source_timestamp_s=5.0,
                )
            raise AttributeError(name)

    class DynamicRaising:
        def __getattr__(self, name: str) -> object:
            if name == "pixel":
                raise RuntimeError("the payload exists and cannot be read")
            raise AttributeError(name)

    class ProbeRaises(type):
        def __getattribute__(cls, name: str) -> object:
            if name == "pixel":
                raise RuntimeError("asking the TYPE is not a free question")
            return type.__getattribute__(cls, name)

    class AsksTheTypeAtOwnRisk(metaclass=ProbeRaises):
        pixel = SimpleNamespace(
            u_px=1.0,
            v_px=2.0,
            aircraft_pitch_deg=3.0,
            aircraft_roll_deg=4.0,
            source_timestamp_s=5.0,
        )

    # The bytes are named OUTRIGHT, and this is the correction of a real defect
    # in this very test. It first asserted only that two digests were EQUAL and
    # that one was "not None" -- and PAYLOAD_UNREADABLE is not None, so both
    # assertions passed with every digest failing. Measured: the test passed with
    # struct.pack raising unconditionally, and passed with frame_digest stubbed
    # to return the marker. That is the fifth appearance of the pattern this
    # whole file exists to stop, this time inside the test written to catch it.
    # A digest comparison can only ever be one half of the claim; the other half
    # is that the digest is real command bytes.
    assert frame_digest(DynamicReadable()) == DIGEST_ONE_TO_FIVE
    assert frame_digest(AsksTheTypeAtOwnRisk()) == DIGEST_ONE_TO_FIVE

    assert frame_digest(DynamicRaising()) == PAYLOAD_UNREADABLE
