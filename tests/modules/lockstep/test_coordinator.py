"""The coordinator's tick loop and journal, against a scripted SITL link."""

from __future__ import annotations

import hashlib

import pytest

from gcs.backend import instance_ports as ip
from navpy.sim.lockstep.baton import LockstepFault
from navpy.sim.lockstep.coordinator import Journal, run
from navpy.sim.lockstep.scripted_vehicle import mission_items
from navpy.sim.lockstep.step_wire import Frame, PortState, pack_frame, unpack_reply


def _frame(tick: int, *, advances: int = 0, egress: bytes = b"") -> Frame:
    return Frame((7, 1), tick, 20_000 * tick, advances, True,
                 (PortState(0, 0, 0, b""), PortState(0, 0, 0, egress)))


class FakeLink:
    """Plays a SITL: one frame per tick, records every reply."""

    def __init__(self, frames: list[Frame]) -> None:
        self._frames = [pack_frame(frame) for frame in frames]
        self.replies: list[bytes] = []

    def recv_frame(self) -> bytes:
        return self._frames[len(self.replies)]

    def send_reply(self, reply: bytes) -> None:
        self.replies.append(reply)


class EchoParticipant:
    def on_tick(self, frame: Frame) -> tuple[bytes, bytes]:
        return b"", frame.ports[1].egress[::-1]


def test_each_tick_answers_its_frame_and_chains_the_journal():
    frames = [_frame(1, egress=b"ab"), _frame(2), _frame(3, egress=b"xyz")]
    link = FakeLink(frames)
    journal, wall = run(link, EchoParticipant(), ticks=3)

    assert [unpack_reply(reply, expected_tick=k + 1) for k, reply in enumerate(link.replies)] == [
        (b"", b"ba"), (b"", b""), (b"", b"zyx"),
    ]
    assert len(wall) == 2  # cycles between successive frames
    chain = bytes(32)
    for frame, reply in zip(frames, link.replies):
        chain = hashlib.sha256(
            chain + frame.tick.to_bytes(8, "little")
            + hashlib.sha256(pack_frame(frame)).digest() + hashlib.sha256(reply).digest()
        ).digest()
    assert journal.digest == chain.hex()


def test_the_journal_changes_with_one_byte():
    first, _ = run(FakeLink([_frame(1, egress=b"a")]), EchoParticipant(), ticks=1)
    second, _ = run(FakeLink([_frame(1, egress=b"b")]), EchoParticipant(), ticks=1)
    assert first.digest != second.digest


def test_an_out_of_loop_advance_or_a_skipped_tick_stops_the_run():
    with pytest.raises(LockstepFault, match="out-of-loop"):
        run(FakeLink([_frame(1, advances=1)]), EchoParticipant(), ticks=1)
    with pytest.raises(ValueError, match="missing or duplicate"):
        run(FakeLink([_frame(1), _frame(3)]), EchoParticipant(), ticks=2)


def test_host_delays_do_not_change_the_journal():
    frames = [_frame(k, egress=bytes([k])) for k in range(1, 6)]
    plain, _ = run(FakeLink(frames), EchoParticipant(), ticks=5)
    delayed, _ = run(FakeLink(frames), EchoParticipant(), ticks=5, host_delay_seed=3)
    assert plain.digest == delayed.digest


def test_the_journal_csv_has_one_row_per_tick(tmp_path):
    journal = Journal()
    frame = _frame(1)
    journal.add(frame, pack_frame(frame), b"reply")
    journal.write_csv(tmp_path / "journal.csv")
    lines = (tmp_path / "journal.csv").read_text(encoding="utf-8").splitlines()
    assert lines[0] == "tick,source_us,helpers_quiet,frame_sha256,reply_sha256,chain"
    assert lines[1].startswith("1,20000,1,") and lines[1].endswith(journal.digest)


def test_relay_ports_are_slot_derived_and_never_shared():
    ports = {ip.lockstep_relay_port(n, k) for n in range(ip.EVAL_CHAT_MIN, ip.MAX_CHAT_INDEX + 1)
             for k in range(ip.VEHICLES_PER_CHAT)}
    assert len(ports) == (ip.MAX_CHAT_INDEX + 1 - ip.EVAL_CHAT_MIN) * ip.VEHICLES_PER_CHAT
    assert min(ports) > ip.esp32_sim_port(ip.MAX_CHAT_INDEX)
    with pytest.raises(ValueError):
        ip.lockstep_relay_port(40, 3)


def test_the_scripted_mission_is_home_takeoff_waypoints_loiter():
    items = mission_items("40.1,44.2,500,0", 1)
    assert [item.command for item in items] == [16, 22, 16, 16, 16, 17]
    assert [item.seq for item in items] == list(range(6))
    assert items[2].x == round(40.104 * 1e7) and items[2].y == round(44.2 * 1e7)
