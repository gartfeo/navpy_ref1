"""Pure parser for the exact defaults file loaded by ArduPilot."""
from __future__ import annotations

import math
from scripts.eval_param_float32 import _f32, strtof

# The firmware reads a defaults row into ``char line[100]`` and calls fgets
# with ``sizeof(line)-1`` (AP_Param.cpp:2304); that fgets loops ``i < buflen-1``
# reading one byte at a time (AP_Filesystem.cpp:270), so it takes at most 98
# bytes per call.  A longer row is CUT at 98 and its tail is read as the next
# row -- the firmware then stores a value this file never spells out.
_FIRMWARE_ROW_BYTES = 98


def parse_parm(text: str) -> dict[str, float]:
    """Parse ArduPilot's ``NAME VALUE`` parameter-file format."""
    values: dict[str, float] = {}
    for raw in text.splitlines():
        if len(raw.encode("utf-8")) > _FIRMWARE_ROW_BYTES:
            # Reading the whole row would certify a value the vehicle never
            # saw.  "SIM_X 1.0000000596046448" padded past the limit reads as
            # 1.0000001192092896 here and as 1.0 on the vehicle, and the
            # difference is invisible in the run's own output.
            raise ValueError(
                f"a row is {len(raw.encode('utf-8'))} bytes, over the "
                f"{_FIRMWARE_ROW_BYTES} the firmware reads at once "
                "(AP_Filesystem.cpp:270). It would be cut, and the value "
                "stored would not be the one written here."
            )
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        # ArduPilot accepts "NAME VALUE", "NAME,VALUE" and "NAME=VALUE"
        # (AP_Param.cpp), so a file using any of them is legal input. Treating
        # only whitespace as a separator would silently parse to nothing, and
        # an empty override set skips verification altogether -- the exact
        # silent failure this module exists to prevent.
        parts = line.replace(",", " ").replace("=", " ").split()
        if len(parts) < 2:
            continue
        name, token = parts[0].upper(), parts[1]
        converted = strtof(token)
        if converted is None:
            # Skipping the row would drop an override from verification
            # without a word, which is the gap this module exists to close.
            raise ValueError(
                f"{name}: cannot read value {token!r}. Skipping the row would "
                "leave that override unverified."
            )
        if name in values and values[name] != converted:
            # ArduPilot has no single answer for this: set_float is called for
            # each row in turn, so RAM holds the last, while the override
            # lookup returns the first match (AP_Param.cpp:2560). Keeping
            # either would mean verifying a value that may not be the one the
            # vehicle flew.
            raise ValueError(
                f"{name}: given twice, as {values[name]:g} and {converted:g}. "
                "Which one ArduPilot applies is not well defined, so the "
                "flight could not be certified either way."
            )
        if not math.isfinite(_f32(converted)):
            # set_float returns without applying a non-finite value
            # (AP_Param.cpp:2181), so the vehicle simply keeps what it had.
            # Accepting the row would certify a value that was never applied:
            # infinity clamps to the type limit here, and would then match a
            # vehicle sitting at that limit for entirely unrelated reasons.
            raise ValueError(
                f"{name}: {token!r} is not finite as a float32. ArduPilot "
                "would not apply it, so the flight cannot test it."
            )
        values[name] = converted
    return values
