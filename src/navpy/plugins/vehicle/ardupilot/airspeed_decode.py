"""AIRSPEED (295) packets on the shared bus (hitl_guidance.md section 9, step 19).

The bus decodes with pinned pymavlink's ``ardupilotmega`` dialect (navlink included), which has no
AIRSPEED.  pymavlink hands such a packet over as ``MAVLink_unknown``: the original bytes, but no
source identity (system 0, so the bus would fan it out to every receiver) and no CRC check (the
dialect does not know the CRC extra).  This module validates and decodes those original bytes with
the same pin's generated ``all`` dialect, and only for message id 295: every other packet, known or
unknown, keeps the shared dialect's result unchanged.  A 295 packet that fails validation (CRC,
length, header) is refused, never passed on as an unknown.

The decoded packet keeps its original bytes, sequence, source system/component and the sensor's
``id`` and ``flags`` (UNHEALTHY, USING) together.  The bus hands it only to the receiver of its own
source system (``MavBusMembership.receivers_of``), never to the fallback for unattached sources.
Which source and flags may feed the approach law is decided downstream (step 20), not here.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import pymavlink.dialects.v20.all as all_dialect
from pymavlink.dialects.v20.ardupilotmega import (
    PROTOCOL_MARKER_V2,
    MAVLink_message,
    MAVLink_unknown,
)

AIRSPEED_MSG_ID = all_dialect.MAVLINK_MSG_ID_AIRSPEED  # 295
_V2_HEADER_LEN = 10


def unknown_msg_id(message: MAVLink_message) -> int | None:
    """The message id in an unknown packet's original MAVLink 2 header, else None."""
    if not isinstance(message, MAVLink_unknown):
        return None
    raw = message.get_msgbuf()
    if len(raw) < _V2_HEADER_LEN or raw[0] != PROTOCOL_MARKER_V2:
        return None  # MAVLink 1 ids are 8-bit: 295 cannot be one
    return raw[7] | (raw[8] << 8) | (raw[9] << 16)


@dataclass
class AirspeedDecoder:
    """Turns the shared dialect's unknown 295 packets into checked AIRSPEED messages.

    One per bus reader; used only on its reader thread.
    """

    decoded: int = 0
    refused: int = 0
    _parser: all_dialect.MAVLink = field(
        default_factory=lambda: all_dialect.MAVLink(None), repr=False
    )

    def admit(self, message: MAVLink_message) -> MAVLink_message | None:
        """The message to dispatch: ``message`` itself, its AIRSPEED decode, or None (refused)."""
        if unknown_msg_id(message) != AIRSPEED_MSG_ID:
            return message
        decoded = self._decode(bytes(message.get_msgbuf()))
        if decoded is None:
            self.refused += 1
            return None
        self.decoded += 1
        return decoded

    def _decode(self, raw: bytes) -> MAVLink_message | None:
        try:
            # decode() checks the marker, the length against the header and the CRC with
            # AIRSPEED's CRC extra; it keeps the buffer and the header.
            message = self._parser.decode(bytearray(raw))
        except all_dialect.MAVError:
            return None
        if not isinstance(message, all_dialect.MAVLink_airspeed_message):
            return None
        return message
