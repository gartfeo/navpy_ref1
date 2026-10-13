"""Per-frame timing of the Windows relay (tools/tuning/hitl_bridge/relay_timing.py)."""
import io

from pymavlink.dialects.v20 import common as mavlink2

from tools.tuning.hitl_bridge.relay_timing import (DOWN, STALL_MS, UP, FrameSplitter, FrameTimer)


def _codec(sysid: int) -> mavlink2.MAVLink:
    return mavlink2.MAVLink(io.BytesIO(), srcSystem=sysid, srcComponent=1)


def actuator_frame(time_usec: int) -> bytes:
    return _codec(1).hil_actuator_controls_encode(time_usec, [1500.0] * 16, 0, 0).pack(_codec(1))


def state_frame(time_usec: int) -> bytes:
    codec = _codec(42)
    return codec.hil_state_quaternion_encode(time_usec, [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.0, -353632610, 1491652300,
                                             585000, 0, 0, 0, 0, 0, 0, 0, -1000).pack(codec)


def test_the_splitter_finds_frames_and_their_board_time_across_any_chunking():
    stream = b"\x00junk" + actuator_frame(5_000) + state_frame(5_000) + actuator_frame(10_000)
    for size in (1, 7, 64, len(stream)):
        splitter = FrameSplitter()
        frames = []
        for i in range(0, len(stream), size):
            frames += splitter.feed(stream[i:i + size])
        assert frames == [(93, 5_000), (115, 5_000), (93, 10_000)], size


def test_a_trimmed_payload_still_gives_its_time():
    # time_usec 0 trims the whole payload's leading zeros only at the end: the first field reads as 0
    assert FrameSplitter().feed(actuator_frame(0)) == [(93, 0)]


def test_the_round_trip_on_the_pc_pairs_each_state_with_its_actuator_frame():
    out = io.StringIO()
    timer = FrameTimer(out)
    timer.note(UP, actuator_frame(5_000), read_s=1.000, written_s=1.0001)
    timer.note(UP, actuator_frame(10_000), read_s=1.005, written_s=1.0051)
    timer.note(DOWN, state_frame(5_000), read_s=1.0019, written_s=1.002)
    timer.note(DOWN, state_frame(10_000), read_s=1.0089, written_s=1.009)
    assert [round(v, 3) for v in timer.stats.pc_round_trip_ms] == [2.0, 4.0]
    assert [round(v, 3) for v in timer.stats.up_gaps_ms] == [5.0]
    lines = out.getvalue().splitlines()
    assert lines[0] == "direction,time_usec,read_s,written_s" and len(lines) == 5
    assert "2 frames answered" in timer.summary()


def test_a_stall_from_the_board_is_counted_and_unanswered_frames_are_dropped_from_the_pairing():
    timer = FrameTimer()
    timer.note(UP, actuator_frame(5_000), read_s=0.0, written_s=0.0)
    timer.note(UP, actuator_frame(10_000), read_s=(STALL_MS + 30) / 1e3, written_s=0.05)
    for k in range(3, 40):
        timer.note(UP, actuator_frame(k * 5_000), read_s=0.05 + k * 0.005, written_s=0.05 + k * 0.005)
    timer.note(DOWN, state_frame(39 * 5_000), read_s=0.3, written_s=0.3)
    assert timer.stats.unanswered == 39 - 32         # 39 frames before the answer, 32 kept
    assert "1 gaps over" in timer.summary()
