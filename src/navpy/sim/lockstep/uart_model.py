"""Reference model of the serial-step UART contract (the firmware's oracle).

Per port and direction (``docs/design/lockstep_contracts.md``, UART):

- **Ingress.** Bytes the coordinator sends at tick k go to a per-port FIFO
  that is never dropped. Each io tick moves ``min(len(fifo), space, 10000)``
  bytes into the read buffer, as the socket path does (``UARTDriver.cpp``
  ``handle_reading_from_device_to_readbuffer``: ``max_bytes = 10000``, the
  baud limiter off as by default); the autopilot's MAVLink parser reads from
  that buffer. So a burst larger than the buffer arrives whole, over several
  io ticks, in order.
- **Egress.** A write takes at most ``txspace()`` bytes and returns how many
  it took (a short write); the rest is the writer's to drop, and the drop is
  counted. At tick end the coordinator drains every byte written this tick.
- **Header.** Each tick reports the FIFO length, the read-buffer fill and the
  egress drop counter per port, so a backlog is visible in the journal.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass


#: Usable bytes of SITL's 16 KiB UART ring buffers (``UARTDriver.h``); a
#: ``ByteBuffer`` keeps one byte free to tell full from empty.
READ_BUFFER_BYTES = 16384 - 1
WRITE_BUFFER_BYTES = 16384 - 1
#: Bytes one io tick may admit (``max_bytes`` in the socket read path).
MAX_ADMIT_PER_IO_TICK = 10000


@dataclass(frozen=True)
class PortHeader:
    fifo_bytes: int
    read_buffer_bytes: int
    egress_dropped_bytes: int


class StepUart:
    def __init__(
        self,
        read_buffer_bytes: int = READ_BUFFER_BYTES,
        write_buffer_bytes: int = WRITE_BUFFER_BYTES,
    ) -> None:
        self._fifo: deque[int] = deque()
        self._read: deque[int] = deque()
        self._read_size = read_buffer_bytes
        self._write = bytearray()
        self._write_size = write_buffer_bytes
        self._dropped = 0

    # coordinator side
    def enqueue(self, data: bytes) -> None:
        self._fifo.extend(data)

    def drain(self) -> bytes:
        data, self._write = bytes(self._write), bytearray()
        return data

    def header(self) -> PortHeader:
        return PortHeader(len(self._fifo), len(self._read), self._dropped)

    # autopilot side
    def io_tick(self) -> int:
        """Admit what the read buffer has room for; returns bytes admitted."""
        admitted = min(
            len(self._fifo), self._read_size - len(self._read), MAX_ADMIT_PER_IO_TICK
        )
        for _ in range(admitted):
            self._read.append(self._fifo.popleft())
        return admitted

    def read(self, count: int) -> bytes:
        count = min(count, len(self._read))
        return bytes(self._read.popleft() for _ in range(count))

    def txspace(self) -> int:
        return self._write_size - len(self._write)

    def write(self, data: bytes) -> int:
        taken = min(len(data), self.txspace())
        self._write.extend(data[:taken])
        return taken

    def record_drop(self, count: int) -> None:
        """The writer dropped ``count`` bytes it could not place."""
        self._dropped += count


__all__ = [
    "MAX_ADMIT_PER_IO_TICK",
    "PortHeader",
    "READ_BUFFER_BYTES",
    "StepUart",
    "WRITE_BUFFER_BYTES",
]
