"""MAVLink bytes without ``mavutil.mavfile``: an encoder sink and a stream decoder.

The bus adapter contract: participants exchange raw bytes per tick, so a
message may be split across ticks. The decoder keeps its parser state
between :meth:`StreamDecoder.feed` calls; the encoder keeps each source's
sequence number, so the bytes are the ones a real link would carry.
"""

from __future__ import annotations

from pymavlink.dialects.v20 import ardupilotmega as mavlink


class ByteSink:
    """File-like target for ``MAVLink(file=...)``; collects one tick's bytes."""

    def __init__(self) -> None:
        self._chunks: list[bytes] = []

    def write(self, data: bytes) -> None:
        self._chunks.append(bytes(data))

    def drain(self) -> bytes:
        data, self._chunks = b"".join(self._chunks), []
        return data


def encoder(src_system: int, src_component: int) -> tuple[mavlink.MAVLink, ByteSink]:
    sink = ByteSink()
    link = mavlink.MAVLink(sink, srcSystem=src_system, srcComponent=src_component)
    link.robust_parsing = False
    return link, sink


class StreamDecoder:
    """Robust like ``mavfile``: bytes that are no MAVLink frame are skipped and
    counted (an autopilot prints boot text on its serial ports)."""

    def __init__(self) -> None:
        self._parser = mavlink.MAVLink(None)
        self._parser.robust_parsing = True
        self.skipped_bytes = 0

    def feed(self, data: bytes) -> list[mavlink.MAVLink_message]:
        messages = []
        for byte in data:
            message = self._parser.parse_char(bytes((byte,)))
            if message is None:
                continue
            if message.get_type() == "BAD_DATA":
                self.skipped_bytes += len(message.get_msgbuf())
                continue
            messages.append(message)
        return messages


__all__ = ["ByteSink", "StreamDecoder", "encoder"]
