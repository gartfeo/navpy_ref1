"""Exact float32 parsing and ArduPilot parameter storage conversion."""
from __future__ import annotations

import math
import re
import struct
from decimal import Decimal
from fractions import Fraction

# Parameter storage types vary -- EK3_HGT_DELAY is AP_Int16, others are
# AP_Float -- but every value reaches us through a float32 store and a float32
# MAVLink field.  So compare AFTER converting both sides the way the vehicle
# stores them, then require exact equality.  A threshold fails in both
# directions: an absolute one rejects a correctly stored 12345.678 (float32
# gives 12345.677734375, off by 2.6e-4) while accepting a requested 0.00009
# that read back as 0, and a relative one calls ARMING_CHECK bit 20 equal to
# bit 0.


def _f32(value: float) -> float:
    """Round to what a float32 parameter store can actually hold.

    Overflow saturates to infinity rather than raising, which is what the C
    conversion does: strtof("1e39") returns HUGE_VALF.  Raising here would
    turn a bad row in a parameter file into a traceback from struct.
    """
    try:
        return struct.unpack("<f", struct.pack("<f", float(value)))[0]
    except OverflowError:
        return float("-inf") if value < 0 else float("inf")


# MAV_PARAM_TYPE integer classes, with the range AP_Param constrains to.
# These three are the only integer types a vehicle can report: mav_param_type
# maps AP_PARAM_INT8/16/32 to 2/4/6 and everything else to REAL32
# (GCS_MAVLink.cpp:101).  set_float has branches for exactly the same three
# plus float, so a fourth entry here would model a conversion ArduPilot does
# not perform -- for any other type it applies nothing at all.
_INTEGER_LIMITS: dict[int, tuple[int, int]] = {
    2: (-128, 127),
    4: (-32768, 32767),
    6: (-2147483648, 2147483647),
}


def as_stored(value: float, param_type: int | None = None) -> float:
    """Return the value the vehicle would hold after storing this request.

    An integer parameter does not keep what you asked for.  AP_Param's
    set_float (AP_Param.cpp:2179) adds 0.01 signed to match, constrains to the
    type's range, then casts -- the addition existing precisely so a file
    saying 119.99999 lands on 120.  EK3_HGT_DELAY is AP_Int16, so a
    near-integer request is CORRECT, not a mismatch.

    The addition runs in float32, as the C does.  In double precision it falls
    short: 120.99 stays under 121 and truncates to 120, where the vehicle
    stores 121 -- which would both accept a stale 120 and reject the correct
    121.

    The clamp is applied to the CAST result, where the C applies it to the
    float before casting.  Those agree for every finite float32: truncation is
    toward zero, so it never carries a value across a bound in either
    direction.  Clamping after also stays defined where the C does not --
    constrain_float converts INT32_MAX to float too, giving 2147483648.0f, and
    casting that to int32 is undefined in C++.  So this is EXACT for AP_Int8
    and AP_Int16, and for AP_Int32 within its range; a request past the
    AP_Int32 bound is outside what this check can certify, and the value here
    is the sane reading rather than a prediction.

    With no type -- comparing a readback whose type the vehicle did not report
    -- fall back to the float32 round, which is what a float parameter does.
    """
    rounded = _f32(value)
    if not math.isfinite(rounded):
        # set_float returns without applying a non-finite value
        # (AP_Param.cpp:2181), so there is no stored value to report -- for a
        # float parameter just as much as for an integer one.
        raise ValueError(f"{value!r} is not finite; ArduPilot would not store it")
    limits = _INTEGER_LIMITS.get(param_type)
    if limits is None:
        return rounded
    addition = _f32(0.01) if rounded >= 0 else _f32(-0.01)
    shifted = _f32(rounded + addition)
    low, high = limits
    # An out-of-range request stores the limit rather than overflowing.
    return float(min(max(int(shifted), low), high))


def same_value(left: float, right: float, param_type: int | None = None) -> bool:
    """True when a request and a readback agree once stored.

    Exact equality after the storage conversion, with no tolerance on top.
    The conversion already absorbs representation loss, and a tolerance beyond
    it hides differences that are real: an AP_Int32 bitmask distinguishes
    1048576 from 1048577 (ARMING_CHECK bit 20 against bit 0), and both are
    exactly representable.

    Two integers above 2**24 share a float32, so equality here certifies the
    wire value, not the underlying store.  That is a limit of what a MAVLink
    readback can prove, not something a tolerance would repair.
    """
    return as_stored(left, param_type) == as_stored(right, param_type)


_NUMERIC = re.compile(
    # The hex branch allows a leading point ("0x.8p1" is 1.0 to strtof); a
    # form the regex rejects is not skipped but silently read as something
    # else, so the missing case cost a real override its verification.
    r"[+-]?(?:0[xX](?:[0-9a-fA-F]+(?:\.[0-9a-fA-F]*)?|\.[0-9a-fA-F]+)"
    r"(?:[pP][+-]?[0-9]+)?"
    r"|(?:[0-9]+\.?[0-9]*|\.[0-9]+)(?:[eE][+-]?[0-9]+)?)"
)


def _step_f32(value: float, upward: bool) -> float:
    """The adjacent float32, toward +inf or -inf."""
    bits = struct.unpack("<I", struct.pack("<f", value))[0]
    if value == 0.0:
        bits = 1 if upward else (1 << 31) | 1
    elif (value > 0.0) == upward:
        bits += 1
    else:
        bits -= 1
    return struct.unpack("<f", struct.pack("<I", bits))[0]


# The largest finite float32, and the exact value at which IEEE rounding sends
# a result to infinity instead of it: 2**128 - 2**103, the midpoint between the
# two.  Anything strictly below rounds back down to the finite value, however
# close it gets, so the boundary has to be tested exactly rather than inferred
# from a binary64 approximation that has already overflowed.
_F32_MAX = struct.unpack("<f", struct.pack("<I", 0x7F7FFFFF))[0]
_F32_OVERFLOW = Fraction(2) ** 128 - Fraction(2) ** 103


def _nearest_f32(exact: Fraction) -> float:
    """The float32 nearest an exact value, ties to even.

    Ties-to-even is the default IEEE rounding mode, which is what the SITL
    binary runs under; glibc's strtof honours whatever mode is active, so this
    matches it under that default and nothing here changes the mode.

    float(text) then _f32() rounds TWICE, and the two disagree: the decimal
    1.0000000596046448 is a hair above the float32 midpoint above 1.0, so C
    strtof answers 1.0000001192092896, while binary64 lands exactly ON the
    midpoint and the second rounding takes it to 1.0.  That difference is not
    academic here -- it decided whether a parameter was verified at all.
    """
    if abs(exact) >= _F32_OVERFLOW:
        return float("-inf") if exact < 0 else float("inf")
    try:
        start = _f32(float(exact))
    except OverflowError:
        start = math.copysign(_F32_MAX, -1.0 if exact < 0 else 1.0)
    if not math.isfinite(start):
        # Below the overflow boundary but over binary64's own rounding of it:
        # 2**128 - 2**103 - 1 rounds UP to the boundary as a double and then
        # narrows to infinity, where the correct answer is the largest finite
        # float32.  Start from that instead and let the search below settle it.
        start = math.copysign(_F32_MAX, -1.0 if exact < 0 else 1.0)
    best = None
    for candidate in (_step_f32(start, False), start, _step_f32(start, True)):
        if not math.isfinite(candidate):
            continue
        distance = abs(Fraction(candidate) - exact)
        even = struct.unpack("<I", struct.pack("<f", candidate))[0] % 2 == 0
        # Ties go to the even mantissa, so the key breaks them the same way.
        key = (distance, not even)
        if best is None or key < best[0]:
            best = (key, candidate)
    return start if best is None else best[1]


def _bounded_exponent(exponent: int, base: int, scale: int) -> int:
    """Pull an exponent back to one that can be computed, changing no answer.

    The pattern accepts any run of digits, so a file can say ``1e-9999999999``
    and no arithmetic on that is possible -- ``Decimal`` refuses it outright.
    It is also unnecessary.  A mantissa written with N digits lies strictly
    between ``base**-scale`` and ``base**scale``, so once the exponent passes
    ``scale`` by the margin below, the result is beyond float32 in that
    direction whatever the mantissa is, and clamping cannot change where it
    rounds to.  The margin covers float32's own range with room to spare:
    ``10**60`` is far past the largest float32 (3.4e38) and ``10**-60`` far
    under the smallest subnormal (1.4e-45); ``2**200`` and ``2**-200`` likewise.
    """
    margin = 60 if base == 10 else 200
    return max(-(margin + scale), min(margin + scale, exponent))


def _exact_value(text: str) -> Fraction:
    """The exact value of a numeric token, losing nothing on the way."""
    body = text.lstrip("+-")
    sign = -1 if text[0] == "-" else 1
    if body[:2].lower() != "0x":
        digits, _, power = body.replace("E", "e").partition("e")
        exponent = _bounded_exponent(int(power or 0), 10, len(digits))
        return sign * Fraction(Decimal(digits or "0")) * Fraction(10) ** exponent
    body = body[2:]
    exponent = 0
    if "p" in body.lower():
        digits, _, power = body.replace("P", "p").partition("p")
        exponent = int(power or 0)
    else:
        digits = body
    whole, _, fraction = digits.partition(".")
    scaled = int((whole + fraction) or "0", 16)
    exponent = _bounded_exponent(exponent, 2, 4 * len(digits))
    return sign * Fraction(scaled, 16 ** len(fraction)) * Fraction(2) ** exponent


def strtof(token: str) -> float | None:
    """Convert a parameter-file value the way ArduPilot's strtof does.

    Not float(): ArduPilot reads shipped defaults such as ``0xF0`` -- four of
    the 75 files under Tools/autotest/default_params use hex -- and it stops
    at the first character it cannot use, so ``120;note`` is 120.  Python
    rejects that hex and reads ``1_20`` as 120 where strtof yields 1.

    Returns None when there is no numeric prefix at all.  strtof would store 0
    there; refusing is the safer reading, since such a token is a mistake
    rather than a request for zero.

    The result is a float32, which is what C strtof returns and what the
    vehicle stores.  Checked against glibc -- the library the SITL binary
    links, not the host's -- over 9525 tokens including exact midpoints,
    subnormals, the overflow boundary and hex forms.  Converting to binary64
    first and narrowing afterwards rounds twice and does not always agree --
    see _nearest_f32.  An overflow
    saturates to infinity, as strtof's HUGE_VALF does, and is refused by the
    caller rather than raising out of struct or float.fromhex.
    """
    match = _NUMERIC.match(token.strip())
    if not match:
        return None
    text = match.group(0)
    value = _nearest_f32(_exact_value(text))
    if value == 0.0 and text[0] == "-":
        # A Fraction has no signed zero, so the sign is lost on the way
        # through.  C keeps it: strtof("-0") is -0.0.  Nothing here tells the
        # two apart -- -0.0 == 0.0 -- but the conversion claims to be what the
        # C does, and this is the one input where it would not be.
        return -0.0
    return value
