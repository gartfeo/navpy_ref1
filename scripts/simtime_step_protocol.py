"""Identity-only Linux SITL stepping protocol; never carries vehicle commands."""

from __future__ import annotations

import struct
from dataclasses import dataclass


WIRE = struct.Struct("<8sQQQQII")
MAGIC = b"NVSTEP01"
WINDOW_SIZE = 1000


@dataclass(frozen=True)
class StepRequest:
    boot: tuple[int, int]
    step: int
    source_us: int
    vehicle: int
    tick: int


def decode(packet: bytes) -> StepRequest:
    if len(packet) != WIRE.size:
        raise ValueError("wrong frame length")
    magic, boot0, boot1, step, source_us, vehicle, tick = WIRE.unpack(packet)
    if magic != MAGIC:
        raise ValueError("wrong protocol version")
    if not (boot0 or boot1) or not 1 <= step <= WINDOW_SIZE:
        raise ValueError("invalid boot or step")
    if source_us == 0 or tick == 0 or not 1 <= vehicle <= 255:
        raise ValueError("invalid source identity")
    return StepRequest((boot0, boot1), step, source_us, vehicle, tick)


class StepSequence:
    """Validate one finite boot/vehicle window before echoing a continue."""

    def __init__(self) -> None:
        self.previous: StepRequest | None = None

    def accept(self, packet: bytes) -> StepRequest:
        current = decode(packet)
        previous = self.previous
        if previous is None:
            if current.step != 1:
                raise ValueError("window starts late")
        elif (
            current.boot != previous.boot
            or current.vehicle != previous.vehicle
            or current.step != previous.step + 1
            or current.tick != previous.tick + 1
            or current.source_us <= previous.source_us
        ):
            raise ValueError("discontinuous step stream")
        self.previous = current
        return current
