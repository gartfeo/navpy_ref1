"""Versioned, local-only control-boundary protocol for the SITL experiment."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct

MAGIC = b"NVGUID03"
HEADER = struct.Struct("<8sQQQQII")
STATE = struct.Struct("<QQIIIIQQII17dii16H")
COMMAND = struct.Struct("<QQII5f")
SNAPSHOT_SIZE = HEADER.size + STATE.size
REPLY_SIZE = HEADER.size + COMMAND.size
NONE, ATTITUDE, TAKEOFF_ARM, GUIDED = range(4)
WINDOW_SIZE = 3000
DRAIN_STEPS = 2
# Seven is coprime with the five-tick 40Hz camera cycle. All phases are perturbed.
HOST_DELAYS_S = (0.0, 0.002, 0.004, 0.0, 0.006, 0.001, 0.003)


@dataclass(frozen=True)
class Identity:
    boot: tuple[int, int]
    step: int
    source_us: int
    vehicle: int
    tick: int

    def pack(self) -> bytes:
        return HEADER.pack(MAGIC, *self.boot, self.step, self.source_us,
                           self.vehicle, self.tick)


@dataclass(frozen=True)
class ControlObservation:
    """Only estimated quantities permitted to accompany measured pixels."""

    roll_rad: float
    pitch_rad: float
    rates_rad_s: tuple[float, float, float]
    airspeed_mps: float


@dataclass(frozen=True)
class RenderTruth:
    latitude: float
    longitude: float
    altitude: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass(frozen=True)
class ApplicationReceipt:
    before_us: int
    after_us: int
    log_disarmed: bool
    logger_ready: bool


@dataclass(frozen=True)
class Snapshot:
    identity: Identity
    truth_us: int
    applied_step: int
    applied_kind: int
    mode: int
    armed: bool
    status: int
    application: ApplicationReceipt
    observation: ControlObservation
    truth: RenderTruth
    limits: tuple[float, float, float, float, float]
    previous_targets: tuple[int, int]
    previous_servos: tuple[int, ...]

    @classmethod
    def decode(cls, packet: bytes) -> Snapshot:
        if len(packet) != SNAPSHOT_SIZE:
            raise ValueError("snapshot length")
        magic, b0, b1, step, source, vehicle, tick = HEADER.unpack_from(packet)
        if magic != MAGIC or not (b0 or b1) or step < 1 or not 1 <= vehicle <= 255:
            raise ValueError("snapshot identity/version")
        v = STATE.unpack_from(packet, HEADER.size)
        if not all(math.isfinite(x) for x in v[10:27]):
            raise ValueError("nonfinite snapshot")
        if v[4] not in (0, 1) or v[5] != 0:
            raise ValueError("snapshot armed/status")
        if v[6] != v[7] or v[7] != source or v[8] not in (0, 1) or v[9] not in (0, 1):
            raise ValueError("invalid application receipt")
        return cls(Identity((b0, b1), step, source, vehicle, tick),
                   *v[:4], bool(v[4]), v[5],
                   ApplicationReceipt(v[6], v[7], bool(v[8]), bool(v[9])),
                   ControlObservation(v[10], v[11], tuple(v[12:15]), v[15]),
                   RenderTruth(*v[16:22]), tuple(v[22:27]),
                   tuple(v[27:29]), tuple(v[29:]))


@dataclass(frozen=True)
class StepCommand:
    kind: int = NONE
    mask: int = 0
    quaternion: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    throttle: float = 0.0

    def reply(self, identity: Identity) -> bytes:
        if self.kind not in (NONE, ATTITUDE, TAKEOFF_ARM, GUIDED):
            raise ValueError("unknown command")
        if not all(math.isfinite(v) for v in (*self.quaternion, self.throttle)):
            raise ValueError("nonfinite command")
        return identity.pack() + COMMAND.pack(
            identity.step, identity.step + 1, self.kind, self.mask,
            *self.quaternion, self.throttle)


class SnapshotSequence:
    """Reject discontinuities and missing application acknowledgements."""

    def __init__(self) -> None:
        self.previous: Snapshot | None = None
        self.expected = StepCommand()

    def accept(self, packet: bytes) -> Snapshot:
        current = Snapshot.decode(packet)
        i = current.identity
        previous = self.previous
        if previous is None:
            if i.step != 1 or current.applied_step or current.applied_kind:
                raise ValueError("invalid first snapshot")
        else:
            p = previous.identity
            if (i.boot, i.vehicle, i.step, i.tick) != (
                    p.boot, p.vehicle, p.step + 1, p.tick + 1) or i.source_us <= p.source_us:
                raise ValueError("nonconsecutive control snapshot")
            if (current.applied_step, current.applied_kind) != (p.step, self.expected.kind):
                raise ValueError("missing application acknowledgement")
            if current.truth_us - i.source_us != previous.truth_us - p.source_us:
                raise ValueError("truth/estimator boundary offset changed")
            if current.limits != previous.limits:
                raise ValueError("law configuration changed")
        self.previous = current
        return current

    def reply(self, command: StepCommand) -> bytes:
        if self.previous is None:
            raise ValueError("reply without snapshot")
        self.expected = command
        return command.reply(self.previous.identity)


class CameraSchedule:
    """Capture on actual control ticks, anchored at phase entry."""

    def __init__(self, numerator: int = 4, denominator: int = 5) -> None:
        if not 0 < numerator <= denominator:
            raise ValueError("invalid camera fraction")
        self.numerator, self.denominator = numerator, denominator
        self.phase = denominator - numerator

    def due(self) -> bool:
        self.phase += self.numerator
        if self.phase >= self.denominator:
            self.phase -= self.denominator
            return True
        return False
