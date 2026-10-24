"""Radio byte loss and what it broke (todo 114 fix 1, ``radio_link``).

``docs/design/swarm-task-assignment-ack.md``, "Fix 1" and its Checks: the
loss rate and its draws, chunk independence, a clean serial0, the router
neither routing nor learning from a broken frame, and the position matching
of lossy emissions to clean packets (broken, false, genuine).
"""

from __future__ import annotations

import random

import pytest

from navpy.sim.lockstep.coordinator_journal import sha256
from navpy.sim.lockstep.radio_link import ByteLoss, RadioLinks, RadioStream, journal_radio_loss_pct, \
    radio_events
from navpy.sim.lockstep.router_entries import load_entry_table
from navpy.sim.lockstep.router_framer import RouterFramer
from navpy.sim.lockstep.sim_profile import model_stream
from tests.modules.lockstep.router_doubles import BACKEND, ROUTER, PortsHarness, Sources, serial0, serial1

TABLE = load_entry_table()
DEVICE = "sitl1.serial1"


class FixedLoss:
    """Loses the bytes at the given offsets of the whole stream."""

    def __init__(self, *lost: int) -> None:
        self._lost = set(lost)
        self._fed = 0

    def drop(self, data: bytes) -> tuple[bytes, list[int]]:
        lost = [i for i in range(len(data)) if self._fed + i in self._lost]
        self._fed += len(data)
        return bytes(b for i, b in enumerate(data) if i not in lost), lost


class CountingRandom(random.Random):
    def __init__(self) -> None:
        super().__init__(1)
        self.calls = 0

    def random(self) -> float:
        self.calls += 1
        return super().random()


class Lossy:
    """A radio stream with its lossy router framer, driven chunk by chunk."""

    def __init__(self, loss, device: str = DEVICE) -> None:
        self.radio = RadioStream(device, loss, TABLE)
        self.framer = RouterFramer(TABLE)
        self.emitted: list[tuple[list, bytes]] = []
        self.events: list[list] = []

    def send(self, tick: int, data: bytes) -> None:
        kept = self.radio.transmit(tick, data)
        emissions = []
        for segment in self.framer.feed(kept):
            if segment.packet is not None:
                packet_id = [1, 1, DEVICE, len(self.emitted) + 1]
                self.emitted.append((packet_id, segment.data))
                emissions.append((packet_id, segment.start, len(segment.data)))
        self.events += self.radio.observe(emissions, self.framer.held_start)

    def end(self) -> None:
        self.events += self.radio.settle()

    def kinds(self) -> list[tuple]:
        return [(e[0], tuple(e[3])) if e[0] == "broken" else (e[0], e[2][3]) for e in self.events]


def _frames(count: int) -> tuple[list[bytes], list[tuple[int, int]]]:
    src = Sources()
    frames = [src.heartbeat(1, 1) if n % 2 else src.param((255, 190), 2, 1) for n in range(count)]
    starts, at = [], 0
    for frame in frames:
        starts.append((at, at + len(frame)))
        at += len(frame)
    return frames, starts


# Draws


def test_the_loss_rate_is_near_b_over_many_bytes():
    loss = ByteLoss(0.01, model_stream(1, "uart_loss", "radio.sitl1"))
    kept, lost = loss.drop(bytes(200_000))
    assert len(kept) + len(lost) == 200_000
    assert 1800 < len(lost) < 2200


def test_the_lost_bytes_do_not_depend_on_the_chunks():
    data = bytes(random.Random(3).randrange(256) for _ in range(20_000))
    whole = ByteLoss(0.02, model_stream(5, "uart_loss", "radio.sitl1")).drop(data)
    chunked = ByteLoss(0.02, model_stream(5, "uart_loss", "radio.sitl1"))
    kept, lost, at, cuts = b"", [], 0, random.Random(4)
    while at < len(data):
        size = cuts.randrange(0, 300)
        part_kept, part_lost = chunked.drop(data[at:at + size])
        kept += part_kept
        lost += [at + index for index in part_lost]
        at += size
    assert (kept, lost) == whole and lost


def test_zero_loss_draws_nothing_and_keeps_every_byte():
    draws = CountingRandom()
    assert ByteLoss(0.0, draws).drop(b"\xfd" * 500) == (b"\xfd" * 500, [])
    assert draws.calls == 0
    for rate in (-0.1, 1.0):
        with pytest.raises(ValueError):
            ByteLoss(rate, draws)


# Matching: genuine, broken, false


def test_a_byte_lost_in_a_frames_crc_breaks_it_and_the_next_frame_by_resync():
    frames, ranges = _frames(3)
    a, b, c = frames
    assert 0xFD not in b[1:] and 0xFE not in b[1:]  # B's rest has no marker: the parser skips it
    stream = Lossy(FixedLoss(len(a) - 1))
    stream.send(7, a + b + c)
    assert stream.kinds() == [("broken", ranges[0]), ("broken", ranges[1])]
    assert [data for _id, data in stream.emitted] == [c]  # recovery at C, genuine
    broken = stream.events[0]
    assert broken[:3] == ["broken", DEVICE, 7] and broken[4] == sha256(a)


def test_a_byte_lost_in_a_frames_header_breaks_only_what_the_parser_could_not_recover():
    frames, ranges = _frames(3)
    stream = Lossy(FixedLoss(1))  # A's length byte
    stream.send(1, b"".join(frames))
    emitted = [data for _id, data in stream.emitted]
    broken = [tuple(e[3]) for e in stream.events if e[0] == "broken"]
    assert ranges[0] in broken and frames[2] in emitted
    # Every clean packet is either broken or emitted unchanged, never both, never neither.
    for frame, span in zip(frames, ranges):
        assert (span in broken) != (frame in emitted)
    # What the parser made of the broken header is no clean packet: recorded false.
    false = [e[2] for e in stream.events if e[0] == "false"]
    assert false == [packet_id for packet_id, data in stream.emitted if data not in frames]


def test_a_frame_embedded_in_a_broken_one_is_false_and_the_real_ones_are_genuine():
    src = Sources()
    f = src.heartbeat(1, 1)
    a = src.send((1, 1), "encapsulated_data", 7, [0] * 4 + list(f) + [0] * (253 - 4 - len(f)))
    b = src.param((255, 190), 2, 1)
    assert a.find(b"\xfd", 1) == a.find(f)  # the first marker after A's own is F's
    stream = Lossy(FixedLoss(0))  # A's start marker
    stream.send(3, a + b + f)
    assert [data for _id, data in stream.emitted] == [f, b, f]
    false_id = stream.emitted[0][0]
    assert stream.events == [["false", DEVICE, false_id],
                             ["broken", DEVICE, 3, [0, len(a)], sha256(a), 131, [1, 1], [-1, -1], {}]]


def test_a_frame_the_loss_shortened_by_exactly_its_lost_byte_is_false():
    """An id outside the table has no CRC check: P's lost length byte makes its header declare
    one byte less, so the parse spans P's own bytes with a gap: false, and P broken."""
    unknown = (2895918).to_bytes(3, "little")
    p = bytes([0xFD, 1, 0, 0, 5, 1, 1]) + unknown + bytes([0x50, 0x11, 0x22])
    assert TABLE.get(2895918) is None and TABLE.get(int.from_bytes(unknown[1:] + b"P", "little")) is None
    q = Sources().heartbeat(1, 1)
    stream = Lossy(FixedLoss(1))
    stream.send(1, p + q)
    assert [data for _id, data in stream.emitted] == [p[:1] + p[2:], q]
    assert stream.kinds() == [("false", 1), ("broken", (0, len(p)))]


def test_an_intact_packet_inside_a_frame_the_router_still_holds_at_the_end_is_broken():
    """A's lost byte breaks A and B; a marker in B's payload starts a frame longer than the rest of
    the run, so the router holds C forever. C lost no byte, but only the loss kept it from the
    router: broken too."""
    src = Sources()
    a, c = src.heartbeat(1, 1), src.heartbeat(1, 1)
    data = [1] * 180
    data[170:172] = [0xFD, 200]
    b = src.send((255, 190), "gps_rtcm_data", 0, 180, bytes(data))
    stream = Lossy(FixedLoss(len(a) - 1))
    stream.send(1, a + b + c)
    stream.end()
    assert stream.emitted == [] and stream.framer.held
    end = len(a) + len(b)
    assert stream.kinds() == [("broken", (0, len(a))), ("broken", (len(a), end)),
                              ("broken", (end, end + len(c)))]


def test_byte_identical_repeats_are_matched_by_position():
    frames, _ranges = _frames(1)
    frame, size = frames[0], len(frames[0])
    stream = Lossy(FixedLoss(2 * size - 1))  # the second copy's last byte; resync takes the third
    stream.send(1, frame * 4)
    assert [data for _id, data in stream.emitted] == [frame, frame]
    assert stream.kinds() == [("broken", (size, 2 * size)), ("broken", (2 * size, 3 * size))]


@pytest.mark.parametrize("cut", [1, 5, 17, 40])
def test_the_same_traffic_split_across_chunks_and_ticks_gives_the_same_evidence(cut):
    frames, _ranges = _frames(6)
    data = b"".join(frames)
    lost = (3, len(frames[0]) + len(frames[1]) + 10, len(data) - 2)
    whole = Lossy(FixedLoss(*lost))
    whole.send(1, data)
    whole.end()
    split = Lossy(FixedLoss(*lost))
    for at in range(0, len(data), cut):
        split.send(at // cut + 1, data[at:at + cut])
    split.end()
    untimed = [[*e[:2], *e[3:]] if e[0] == "broken" else e for e in split.events]
    assert untimed == [[*e[:2], *e[3:]] if e[0] == "broken" else e for e in whole.events]
    # A broken packet carries the tick of the chunk its last clean byte came in.
    assert [e[2] for e in split.events if e[0] == "broken"] ==         [(e[3][1] - 1) // cut + 1 for e in split.events if e[0] == "broken"]
    assert [d for _i, d in split.emitted] == [d for _i, d in whole.emitted]


def test_a_broken_packet_is_written_at_the_tick_it_completed():
    frames, ranges = _frames(3)
    data = b"".join(frames)
    stream = Lossy(FixedLoss(len(frames[0]) - 1))
    stream.send(1, data[:5])
    stream.send(2, data[5:len(frames[0])])
    stream.send(3, data[len(frames[0]):])
    assert [(e[2], tuple(e[3])) for e in stream.events] == [(2, ranges[0]), (3, ranges[1])]


def test_at_the_end_a_packet_only_the_clean_stream_completed_is_broken():
    frames, ranges = _frames(2)
    a, b = frames
    held = Lossy(FixedLoss())
    held.send(1, a + b[:-1])  # B incomplete on both sides: the router still holds it
    held.end()
    assert held.events == []
    cut = Lossy(FixedLoss(len(a) + len(b) - 1))
    cut.send(1, a + b)  # B's last byte lost: complete in the clean stream, held short in the lossy one
    assert cut.events == []
    cut.end()
    assert cut.kinds() == [("broken", ranges[1])]


def test_random_loss_accounts_for_every_clean_packet_exactly_once():
    src = Sources()
    rng = random.Random(9)
    frames = [rng.choice((src.heartbeat(1, 1), src.param((255, 190), 2, 1), src.blob((255, 190), n)))
              for n in range(300)]
    stream = Lossy(ByteLoss(0.004, model_stream(2, "uart_loss", "radio.sitl1")))
    clean, data = [], b"".join(frames)
    for tick, at in enumerate(range(0, len(data), 97), start=1):
        stream.send(tick, data[at:at + 97])
    stream.end()
    at = 0
    for frame in frames:
        clean.append((at, at + len(frame), frame))
        at += len(frame)
    broken = {tuple(e[3]) for e in stream.events if e[0] == "broken"}
    false = {tuple(e[2]) for e in stream.events if e[0] == "false"}
    genuine = [data for packet_id, data in stream.emitted if tuple(packet_id) not in false]
    assert broken and len(genuine) + len(broken) == len(frames)
    assert genuine == [frame for start, end, frame in clean if (start, end) not in broken]


# The coordinator's radios


def test_each_radio_loses_its_own_bytes_from_the_seed():
    data = bytes(random.Random(6).randrange(256) for _ in range(5000))
    endpoints = {"sitl1.serial1": "sitl1", "sitl2.serial1": "sitl2"}

    def kept(seed: int) -> list[bytes]:
        links = RadioLinks(5.0, seed, endpoints, TABLE)
        return [links.transmit(("sitl", 1, device), 1, data) for device in endpoints]

    first = kept(3)
    assert first == kept(3) and first != kept(4)
    assert first[0] != first[1] and all(len(k) < len(data) for k in first)


def test_two_streams_lose_independently_and_only_the_lossy_one_has_evidence():
    frames, _ranges = _frames(4)
    links = RadioLinks(1.0, 1, {"sitl1.serial1": "sitl1", "sitl2.serial1": "sitl2"}, TABLE)
    links._losses["sitl1.serial1"] = FixedLoss(2)
    links._losses["sitl2.serial1"] = FixedLoss()
    data = b"".join(frames)
    for device in ("sitl1.serial1", "sitl2.serial1"):
        key = ("sitl", 1, device)
        framer = RouterFramer(TABLE)
        kept = links.transmit(key, 1, data)
        emissions = [([device, n], s.start, len(s.data)) for n, s in enumerate(framer.feed(kept))
                     if s.packet is not None]
        links.observe(key, emissions, framer.held_start)
    links.settle()
    assert {e[1] for e in links.take()} == {"sitl1.serial1"}
    assert not links.lossy("lockstep:serial0.1")


def test_serial0_is_clean_and_serial1_and_the_uplink_lose_bytes():
    run = PortsHarness(radio_loss_pct=50.0)
    src = Sources()
    navpy = run.roster.add(1, serial0(1))
    backend = run.roster.add(BACKEND, ROUTER)
    status, companion = src.heartbeat(1, 1) * 20, src.heartbeat(1, 191) * 20
    delivered, ingress = run.tick(1, egress={serial0(1): status, serial1(1): src.heartbeat(1, 1) * 20},
                                  admitted=[navpy, backend],
                                  done=[(navpy, serial0(1), companion),
                                        (backend, ROUTER, src.heartbeat(255, 190) * 20)])
    assert delivered[serial0(1)] == status and ingress[serial0(1)] == companion
    radio = run.records[-1]["radio"]
    assert {e[1] for e in radio if e[0] == "broken"} == {serial1(1), ROUTER}
    assert run.journal().problems() == []


def test_a_broken_frame_is_never_routed_and_never_teaches_the_router_its_source(monkeypatch):
    """Todo 102 at both lossy ingresses: SITL 2's broken heartbeat leaves system 2 unknown, so the
    backend's request to 2/1 reaches nobody; a clean one is learned and routed."""
    def request_routed(lost: tuple[int, ...]) -> bool:
        run = PortsHarness(radio_loss_pct=1.0)
        monkeypatch.setitem(run.ports.radio._losses, serial1(2), FixedLoss(*lost))
        monkeypatch.setitem(run.ports.radio._losses, ROUTER, FixedLoss())
        src = Sources()
        backend = run.roster.add(BACKEND, ROUTER)
        hb = src.heartbeat(2, 1)
        run.tick(1, egress={serial1(2): hb}, admitted=[backend],
                 done=[(backend, ROUTER, src.heartbeat(255, 190))])
        _delivered, ingress = run.tick(2, admitted=[backend],
                                       done=[(backend, ROUTER, src.param((255, 190), 2, 1))])
        assert run.journal().problems() == []
        return len(ingress[serial1(2)]) > 0 and ingress[serial1(2)] != b""
    assert request_routed(())
    assert not request_routed((len(Sources().heartbeat(2, 1)) - 1,))


def test_the_backends_unknown_ids_are_filtered_not_routed():
    """The uplink takes in only ids whose CRC the router checks, as a SITL endpoint does."""
    run = PortsHarness()
    src = Sources()
    backend = run.roster.add(BACKEND, ROUTER)
    frame = bytearray(src.heartbeat(255, 190))
    frame[7:10] = (2895918).to_bytes(3, "little")  # an id outside the table: no CRC check
    assert TABLE.get(2895918) is None
    run.tick(1, admitted=[backend], done=[(backend, ROUTER, bytes(frame))])
    assert [e[0] for _k, e in run.events("router_filtered")] == ["router_filtered"]
    assert run.events("routed") == []


def test_a_clean_radio_writes_no_radio_evidence_and_the_journal_readers_follow_the_grid():
    run = PortsHarness()
    src = Sources()
    backend = run.roster.add(BACKEND, ROUTER)
    run.tick(1, egress={serial1(1): src.heartbeat(1, 1)}, admitted=[backend])
    assert "radio" not in run.records[-1]
    assert journal_radio_loss_pct([{"grid": {}}]) == 0.0
    assert journal_radio_loss_pct([{"grid": {"radio_loss": {"loss_pct": 0.3}}}]) == 0.3
    records = [{"radio": [["false", "d", [1]]]}, {"parked": {"radio": [["false", "d", [2]]]}},
               {"mavlink_counts": {}, "radio": [["false", "d", [3]]]}]
    assert [e[2] for e in radio_events(records)] == [[1], [2], [3]]


def test_parking_settles_the_sitls_radios():
    run = PortsHarness(radio_loss_pct=1.0)
    src = Sources()
    hb = src.heartbeat(1, 1)
    run.ports.radio._losses[serial1(1)] = FixedLoss(len(hb) + len(hb) - 1)
    run.tick(1, egress={serial1(1): hb})
    parked = run.park(2, egress={serial1(1): hb})
    assert [e[0] for e in parked["radio"]] == ["broken"]
    assert parked["radio"][0][2] == 2 and parked["radio"][0][3] == [len(hb), 2 * len(hb)]
    assert run.journal().problems() == []


@pytest.mark.parametrize("lost", [False, True])
def test_a_broken_backend_frame_is_never_routed_and_is_recorded_on_the_router_device(monkeypatch, lost):
    """The uplink is a radio too: the backend's request with a lost byte reaches no SITL, and the
    radio records it broken on the backend's router device; intact, it is routed."""
    run = PortsHarness(radio_loss_pct=1.0)
    src = Sources()
    hb, request = src.heartbeat(255, 190), src.param((255, 190), 2, 1)
    monkeypatch.setitem(run.ports.radio._losses, serial1(2), FixedLoss())
    monkeypatch.setitem(run.ports.radio._losses, ROUTER,
                        FixedLoss(len(hb) + len(request) - 2) if lost else FixedLoss())
    backend = run.roster.add(BACKEND, ROUTER)
    run.tick(1, egress={serial1(2): src.heartbeat(2, 1)}, admitted=[backend], done=[(backend, ROUTER, hb)])
    _delivered, ingress = run.tick(2, admitted=[backend], done=[(backend, ROUTER, request + hb)])
    run.tick(3, admitted=[backend], done=[(backend, ROUTER, hb)])
    assert (request in ingress[serial1(2)]) is not lost
    broken = [(e[1], e[2], e[4]) for record in run.records for e in record.get("radio", []) if e[0] == "broken"]
    assert broken[:1] == ([(ROUTER, 2, sha256(request))] if lost else [])
    assert run.journal().problems() == []
