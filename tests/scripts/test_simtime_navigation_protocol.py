"""Causal identity and sampling checks for the opt-in simulator protocol."""

from dataclasses import fields
import math

import pytest

from scripts.simtime_navigation_protocol import (
    ATTITUDE, CameraSchedule, ControlObservation, HEADER, Identity, MAGIC,
    Snapshot, SnapshotSequence, STATE, StepCommand,
)


def packet(step: int = 1, **changes: object) -> bytes:
    values = dict(truth_us=40_000_000 + step * 20_000, applied=step - 1,
                  kind=0, mode=15, armed=1, status=0)
    values.update(changes)
    return HEADER.pack(MAGIC, 1, 2, step, 40_000_000 + step * 20_000, 201, 2000 + step) + STATE.pack(
        values["truth_us"], values["applied"], values["kind"], values["mode"],
        values["armed"], values["status"],
        40_000_000 + step * 20_000, 40_000_000 + step * 20_000, 1, 1,
        0., 0., 0., 0., 0., 22., 40., 44., 1400., 0., 0., 0.,
        -20., 25., 65., .25, .5, 0, 0, *([1500] * 16))


def test_acknowledgement_proves_the_previous_command_was_applied() -> None:
    seq = SnapshotSequence()
    seq.accept(packet())
    seq.reply(StepCommand(ATTITUDE, 132, (1., 0., 0., 0.), .5))
    with pytest.raises(ValueError, match="acknowledgement"):
        seq.accept(packet(2))
    assert seq.accept(packet(2, kind=ATTITUDE)).applied_step == 1


@pytest.mark.parametrize("step,changes,reason", [
    (3, {}, "nonconsecutive"), (2, {"truth_us": 40_040_001}, "offset"),
    (2, {"status": 1}, "status"), (2, {"armed": 2}, "armed"),
])
def test_bad_snapshot_invalidates(step: int, changes: dict, reason: str) -> None:
    seq = SnapshotSequence()
    seq.accept(packet())
    with pytest.raises(ValueError, match=reason):
        seq.accept(packet(step, **changes))


def test_snapshot_lengths_and_finite_values() -> None:
    good = packet()
    for bad in (good[:-1], good + b"x"):
        with pytest.raises(ValueError, match="length"):
            Snapshot.decode(bad)
    damaged = bytearray(good)
    import struct
    struct.pack_into("<d", damaged, HEADER.size + 56, math.nan)
    with pytest.raises(ValueError, match="nonfinite"):
        Snapshot.decode(bytes(damaged))


def test_control_observation_has_no_truth_or_compass_fields() -> None:
    assert {f.name for f in fields(ControlObservation)} == {
        "roll_rad", "pitch_rad", "rates_rad_s", "airspeed_mps"}


def test_camera_phase_is_source_tick_based_and_captures_actual_boundaries() -> None:
    camera = CameraSchedule(4, 5)
    assert [tick for tick in range(11) if camera.due()] == [0, 2, 3, 4, 5, 7, 8, 9, 10]
    assert all(CameraSchedule(1, 1).due() for _ in range(20))


def test_reply_declares_exact_one_tick_latency() -> None:
    from scripts.simtime_navigation_protocol import COMMAND
    identity = Identity((1, 2), 7, 140_000, 201, 700)
    reply = StepCommand().reply(identity)
    assert reply[:HEADER.size] == identity.pack()
    assert COMMAND.unpack_from(reply, HEADER.size)[:2] == (7, 8)


def test_physics_quantization_does_not_get_replaced_with_an_ideal_clock() -> None:
    import struct
    seq = SnapshotSequence()
    seq.accept(packet())
    quantized = bytearray(packet(2))
    struct.pack_into("<Q", quantized, 32, 40_039_992)
    struct.pack_into("<Q", quantized, HEADER.size, 40_039_992)
    struct.pack_into("<QQ", quantized, HEADER.size + 32, 40_039_992, 40_039_992)
    assert seq.accept(bytes(quantized)).identity.source_us == 40_039_992


def test_delay_schedule_exercises_every_held_command_phase() -> None:
    from scripts.simtime_navigation_protocol import HOST_DELAYS_S
    camera = CameraSchedule(4, 5)
    held_delays = {HOST_DELAYS_S[index % len(HOST_DELAYS_S)]
                   for index in range(751, 751 + 70) if not camera.due()}
    assert held_delays == set(HOST_DELAYS_S)
