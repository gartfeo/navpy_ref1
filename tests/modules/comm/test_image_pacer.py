"""The paced confirmation-image sender (``radio_budget.md`` round 7, todo 116)."""

import threading
import time

import pytest
from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink_data_transmission_handshake_message,
    MAVLink_encapsulated_data_message,
)

from navpy.plugins.radio.mavlink import network_mavlink
from navpy.plugins.radio.mavlink.image_pacer import PacedImageSender, wire_bytes
from navpy.plugins.radio.mavlink.network_mavlink import NetworkMavlink

CHUNK_S = 0.03  # the test's wire time of one chunk


def _chunk(seq: int, tag: int = 0) -> MAVLink_encapsulated_data_message:
    return MAVLink_encapsulated_data_message(seqnr=seq, data=[tag] * 253)


class _Recorder:
    def __init__(self) -> None:
        self.sent: list[tuple[float, int, int]] = []  # (time, tag, seq)
        self.done = threading.Event()
        self.expect = 0

    def send(self, frame) -> None:
        self.sent.append((time.perf_counter(), frame.data[0], frame.seqnr))  # monotonic ticks 15.6 ms on Windows
        if len(self.sent) >= self.expect:
            self.done.set()


@pytest.fixture
def recorder():
    return _Recorder()


@pytest.fixture
def sender(recorder):
    pacer = PacedImageSender(recorder.send, wire_bytes(_chunk(0)) / CHUNK_S)
    yield pacer
    pacer.close()


def _admit(send) -> bool:
    send()
    return True


def test_a_chunk_is_its_payload_and_mavlink2_framing_on_the_wire():
    assert wire_bytes(_chunk(0)) == 255 + 12
    handshake = MAVLink_data_transmission_handshake_message(0, 0, 0, 0, 0, 0, 0)
    assert wire_bytes(handshake) == 13 + 12


def test_the_radio_image_rate_is_a_quarter_of_57600_baud():
    assert network_mavlink.IMAGE_BYTES_PER_S == pytest.approx(5760 * 0.25)
    assert wire_bytes(_chunk(0)) / network_mavlink.IMAGE_BYTES_PER_S == pytest.approx(0.1854, abs=1e-4)


def test_each_frame_is_followed_by_its_wire_time(sender, recorder):
    recorder.expect = 5
    sender.queue(1, [_chunk(seq) for seq in range(5)], _admit)
    assert recorder.done.wait(5)
    gaps = [b[0] - a[0] for a, b in zip(recorder.sent, recorder.sent[1:])]
    assert all(gap >= CHUNK_S * 0.95 for gap in gaps), gaps


def test_images_go_one_at_a_time_in_queue_order_and_share_the_budget(sender, recorder):
    recorder.expect = 6
    sender.queue(1, [_chunk(seq, 1) for seq in range(3)], _admit)
    sender.queue(2, [_chunk(seq, 2) for seq in range(3)], _admit)
    assert recorder.done.wait(5)
    assert [(tag, seq) for _t, tag, seq in recorder.sent] == [(1, 0), (1, 1), (1, 2), (2, 0), (2, 1), (2, 2)]
    assert recorder.sent[3][0] - recorder.sent[2][0] >= CHUNK_S * 0.95


def test_a_refused_frame_ends_that_image_and_the_next_one_goes(sender, recorder):
    calls = []

    def refuse_second(send) -> bool:
        calls.append(1)
        if len(calls) == 2:
            return False
        send()
        return True

    recorder.expect = 3
    sender.queue(1, [_chunk(seq, 1) for seq in range(4)], refuse_second)
    sender.queue(2, [_chunk(seq, 2) for seq in range(2)], _admit)
    assert recorder.done.wait(5)
    assert [(tag, seq) for _t, tag, seq in recorder.sent] == [(1, 0), (2, 0), (2, 1)]
    assert len(calls) == 2


def test_the_wait_between_frames_is_outside_the_admit(sender, recorder):
    fence = threading.Lock()
    first_sent = threading.Event()

    def fenced(send) -> bool:
        with fence:
            send()
            first_sent.set()
            return True

    sender.queue(1, [_chunk(seq) for seq in range(2)], fenced)
    assert first_sent.wait(5)
    time.sleep(CHUNK_S / 3)  # inside the wait after frame 0
    assert fence.acquire(timeout=CHUNK_S / 3), "the fence is held through the wait"
    fence.release()


def test_a_newer_image_replaces_the_one_waiting_for_its_poi(sender, recorder):
    release = threading.Event()

    def held(send) -> bool:
        release.wait(5)
        send()
        return True

    recorder.expect = 1 + 2
    sender.queue(9, [_chunk(0, 9)], held)  # occupies the sender
    sender.queue(1, [_chunk(seq, 1) for seq in range(2)], _admit)
    sender.queue(1, [_chunk(seq, 3) for seq in range(2)], _admit)
    release.set()
    assert recorder.done.wait(5)
    time.sleep(CHUNK_S * 3)
    assert [(tag, seq) for _t, tag, seq in recorder.sent] == [(9, 0), (3, 0), (3, 1)]


def test_a_new_image_for_the_poi_being_sent_waits_behind_it(sender, recorder):
    recorder.expect = 4
    sender.queue(1, [_chunk(seq, 1) for seq in range(2)], _admit)
    time.sleep(CHUNK_S / 3)  # the first image is being sent
    sender.queue(1, [_chunk(seq, 2) for seq in range(2)], _admit)
    assert recorder.done.wait(5)
    assert [(tag, seq) for _t, tag, seq in recorder.sent] == [(1, 0), (1, 1), (2, 0), (2, 1)]


def test_a_send_error_ends_only_that_image(sender, recorder):
    def failing(send) -> bool:
        raise OSError("link down")

    recorder.expect = 2
    sender.queue(1, [_chunk(seq, 1) for seq in range(3)], failing)
    sender.queue(2, [_chunk(seq, 2) for seq in range(2)], _admit)
    assert recorder.done.wait(5)
    assert [(tag, seq) for _t, tag, seq in recorder.sent] == [(2, 0), (2, 1)]


def test_close_ends_the_image_being_sent(recorder):
    pacer = PacedImageSender(recorder.send, wire_bytes(_chunk(0)) / 1.0)  # 1 s per chunk
    pacer.queue(1, [_chunk(seq) for seq in range(5)], _admit)
    time.sleep(0.1)
    started = time.monotonic()
    pacer.close()
    assert time.monotonic() - started < 0.5
    assert len(recorder.sent) == 1


class _Subscription:
    def cancel(self) -> None:
        pass


class _Vehicle:
    def __init__(self) -> None:
        self.sent = []

    def on_message(self, name, callback):
        return _Subscription()

    def send_mavlink_message(self, message, *, source_component=None) -> None:
        self.sent.append(message)


def test_network_mavlink_queues_the_image_and_returns_at_once(monkeypatch):
    import base64

    monkeypatch.setattr(network_mavlink, "IMAGE_BYTES_PER_S", wire_bytes(_chunk(0)) / CHUNK_S)
    vehicle = _Vehicle()
    network = NetworkMavlink(1, vehicle, logger=None)
    try:
        image = base64.b64encode(bytes(range(256)) * 4).decode()  # 1024 bytes: 5 chunks
        assert network.send_image(7, image) == 5
        assert len(vehicle.sent) <= 1, "send_image waited for its frames"
        deadline = time.monotonic() + 5
        while len(vehicle.sent) < 6 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert [type(m).__name__ for m in vehicle.sent] == (
            ["MAVLink_data_transmission_handshake_message"] + ["MAVLink_encapsulated_data_message"] * 5)
        assert vehicle.sent[0].width == 7 and vehicle.sent[0].packets == 5
    finally:
        network.close()
