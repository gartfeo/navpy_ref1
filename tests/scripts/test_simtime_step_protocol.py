"""Independent literal wire fixture and faulty stream checks for the prototype."""

import importlib.util
from pathlib import Path
import sys

import pytest


SCRIPTS = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(SCRIPTS))
from simtime_step_protocol import StepSequence, decode, WIRE  # noqa: E402
from simtime_step_peer import reply_packet  # noqa: E402


def frame(*, step=1, stamp=100000, tick=50, boot=1, vehicle=121):
    return WIRE.pack(b"NVSTEP01", boot, 2, step, stamp, vehicle, tick)


def test_independent_wire_fixture():
    packet = bytes.fromhex(
        "4e56535445503031 0100000000000000 0200000000000000 "
        "0100000000000000 a086010000000000 79000000 32000000"
    )
    decoded = decode(packet)
    assert (decoded.boot, decoded.step, decoded.source_us,
            decoded.vehicle, decoded.tick) == ((1, 2), 1, 100000, 121, 50)


@pytest.mark.parametrize("fault", ["boot", "step", "version", "truncated", "oversized"])
def test_fault_injection_changes_echo(fault):
    assert reply_packet(frame(), fault) != frame()


@pytest.mark.parametrize("changes", [
    {"step": 1}, {"step": 3}, {"stamp": 100000}, {"stamp": 99999},
    {"tick": 50}, {"tick": 52}, {"boot": 3}, {"vehicle": 122},
])
def test_invalid_stream_does_not_advance_state(changes):
    sequence = StepSequence()
    first = sequence.accept(frame())
    next_fields = {"step": 2, "stamp": 120000, "tick": 51} | changes
    with pytest.raises(ValueError):
        sequence.accept(frame(**next_fields))
    assert sequence.previous == first


def test_complete_window_and_no_extra_step():
    sequence = StepSequence()
    for i in range(1, 1001):
        assert sequence.accept(frame(step=i, stamp=i*20000, tick=i)).step == i
    with pytest.raises(ValueError):
        sequence.accept(frame(step=1001, stamp=20020000, tick=1001))


@pytest.mark.parametrize("packet", [b"", frame()[:-1], frame()+b"x",
                                    reply_packet(frame(), "version")])
def test_invalid_packet(packet):
    with pytest.raises(ValueError):
        decode(packet)
