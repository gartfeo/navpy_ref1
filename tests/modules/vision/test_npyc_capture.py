"""Unreal native camera frames (NPYC v1/v2) through the npyc:// frame-capture backend, and the board clock."""

from __future__ import annotations

import socket
import threading
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from navpy.app.perception.frame_capture_factory import open_capture_backend
from navpy.core.perception.camera.board_clock_map import (
    BOARD_CLOCK_MAX_GAP_S,
    BOARD_CLOCK_MIN_SAMPLES,
    AttitudeBoardClock,
)
from navpy.core.ports.frame_capture_ports import CaptureStampKind
from navpy.plugins.camera.unreal.frame_capture_npyc import NpycFrameCapture, parse_npyc_source
from navpy.plugins.camera.unreal.npyc_codec import (
    NPYC_PIXEL_FORMAT_BGR8,
    NPYC_PIXEL_FORMAT_BGRA8,
    NPYC_V1_HEADER_SIZE,
    NPYC_V2_HEADER_SIZE,
    POSE_EXACT,
    POSE_INTERPOLATED,
    POSE_STAMP_VALID,
    NpycHeader,
    decode_header,
    encode_header,
)

from tests.modules.vision.test_frame_provider import _Logger

WAIT_S = 5.0        # generous bound for loopback delivery; the tests wait on events, never poll
W, H = 4, 3


def header(version=2, flags=POSE_STAMP_VALID | POSE_EXACT, board_time_us=7_000_000, index=0,
           pixel_format=NPYC_PIXEL_FORMAT_BGR8):
    bpp = 3 if pixel_format == NPYC_PIXEL_FORMAT_BGR8 else 4
    sequence = 42 if version == 2 else 0        # v1 carries no pose fields
    return NpycHeader(version, W, H, pixel_format, index, 123.25, W * H * bpp, board_time_us, sequence, flags)


def frame_bytes(h: NpycHeader) -> bytes:
    pixels = (np.arange(h.payload_size) % 251).astype(np.uint8).tobytes()
    return encode_header(h) + pixels


# --- codec ------------------------------------------------------------------------------------

@pytest.mark.parametrize("version, size", [(1, NPYC_V1_HEADER_SIZE), (2, NPYC_V2_HEADER_SIZE)])
def test_header_round_trip_for_both_versions(version, size):
    h = header(version, board_time_us=2 ** 40 + 3 if version == 2 else 0,
               flags=POSE_STAMP_VALID | POSE_EXACT if version == 2 else 0)
    raw = encode_header(h)
    assert len(raw) == size == h.header_size
    assert decode_header(raw) == h


def test_v1_layout_is_the_plugins_original_36_bytes():
    # NavPyGimbalCameraActor.cpp v1 writer: magic, version, size, w, h, format, reserved, index, time, payload.
    import struct
    raw = struct.pack("<4sHHHHHHQdI", b"NPYC", 1, 36, W, H, NPYC_PIXEL_FORMAT_BGR8, 0, 9, 1.5, W * H * 3)
    decoded = decode_header(raw)
    assert (decoded.version, decoded.frame_index, decoded.capture_time_s) == (1, 9, 1.5)
    assert not decoded.exposure_on_board_clock


@pytest.mark.parametrize("flags, exposure", [(POSE_STAMP_VALID | POSE_EXACT, True),
                                             (POSE_STAMP_VALID | POSE_INTERPOLATED, True), (POSE_STAMP_VALID, False),
                                             (POSE_EXACT, False), (POSE_INTERPOLATED, False), (0, False)])
def test_only_a_valid_exact_or_interpolated_pose_stamp_is_an_exposure_on_the_board_clock(flags, exposure):
    assert header(flags=flags).exposure_on_board_clock is exposure


@pytest.mark.parametrize("mutate, message", [
    (lambda raw: b"XXXX" + raw[4:], "magic"),
    (lambda raw: raw[:4] + (3).to_bytes(2, "little") + raw[6:], "version"),
    (lambda raw: raw[:6] + (36).to_bytes(2, "little") + raw[8:], "header size"),
    (lambda raw: raw[:-1], "wrong size"),
])
def test_malformed_headers_are_refused(mutate, message):
    with pytest.raises(ValueError, match=message):
        decode_header(mutate(encode_header(header())))


def test_payload_size_must_match_the_dimensions():
    with pytest.raises(ValueError, match="payload size"):
        encode_header(NpycHeader(2, W, H, NPYC_PIXEL_FORMAT_BGR8, 0, 0.0, W * H * 4))


# --- board clock ------------------------------------------------------------------------------

def samples(pairs):
    return [SimpleNamespace(time_boot_s=boot, receipt_time_s=receipt) for boot, receipt in pairs]


def clock_over(pairs):
    return AttitudeBoardClock(lambda count: samples(pairs)[-count:])


def test_board_time_maps_with_the_minimum_attitude_link_offset():
    # receipt - boot: 1000.010, 1000.004 (the minimum), 1000.030 ...
    pairs = [(t * 0.02, 1000.0 + t * 0.02 + d) for t, d in
             enumerate([0.010, 0.004, 0.030, 0.012, 0.009, 0.020, 0.006, 0.011, 0.015, 0.008])]
    wall = clock_over(pairs).wall_time_s(150_000)
    assert wall == pytest.approx(0.15 + 1000.004)


def test_no_mapping_with_too_few_samples_or_far_from_the_newest_attitude():
    pairs = [(t * 0.02, 50.0 + t * 0.02) for t in range(BOARD_CLOCK_MIN_SAMPLES)]
    assert clock_over(pairs[:-1]).wall_time_s(100_000) is None
    clock = clock_over(pairs)
    newest = pairs[-1][0]
    assert clock.wall_time_s(int((newest - BOARD_CLOCK_MAX_GAP_S + 0.01) * 1e6)) is not None
    assert clock.wall_time_s(int((newest - BOARD_CLOCK_MAX_GAP_S - 0.01) * 1e6)) is None
    assert clock.wall_time_s(int((newest + BOARD_CLOCK_MAX_GAP_S + 0.01) * 1e6)) is None


def test_a_reboot_in_the_history_keeps_only_the_new_boot():
    old_boot = [(100.0 + t * 0.02, 10.0 + t * 0.02) for t in range(20)]      # offset -90
    new_boot = [(t * 0.02, 20.0 + t * 0.02) for t in range(BOARD_CLOCK_MIN_SAMPLES)]   # offset +20
    assert clock_over(old_boot + new_boot).wall_time_s(100_000) == pytest.approx(20.1)
    assert clock_over(old_boot + new_boot[:3]).wall_time_s(20_000) is None
    # A frame from a short old boot, inside the 2 s window but before the new boot's first sample.
    short_old = [(1.0 + t * 0.02, 30.0 + t * 0.02) for t in range(10)]
    later_new = [(0.5 + t * 0.02, 40.0 + t * 0.02) for t in range(BOARD_CLOCK_MIN_SAMPLES)]
    clock = clock_over(short_old + later_new)
    assert clock.wall_time_s(400_000) is None
    assert clock.wall_time_s(600_000) == pytest.approx(0.6 + 39.5)


def test_the_mapping_reads_only_the_two_clock_fields_never_a_pose():
    class TimingOnly:
        def __init__(self, boot, receipt):
            self.time_boot_s, self.receipt_time_s = boot, receipt

        def __getattr__(self, name):          # attitude, rates, anything else
            raise AssertionError(f"board clock read {name}")

    history = [TimingOnly(t * 0.02, 5.0 + t * 0.02) for t in range(BOARD_CLOCK_MIN_SAMPLES)]
    assert AttitudeBoardClock(lambda count: history).wall_time_s(0) == pytest.approx(5.0)


# --- backend over loopback TCP ----------------------------------------------------------------

class FixedClock:
    def __init__(self, wall):
        self.wall = wall
        self.asked = []

    def wall_time_s(self, board_time_us):
        self.asked.append(board_time_us)
        return self.wall


def free_port():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


class Harness:
    """The backend running in a thread; publications land in a list, each one sets an event."""

    def __init__(self, board_clock=None):
        self.source = f"npyc://127.0.0.1:{free_port()}"
        self.backend = NpycFrameCapture.open(self.source, _Logger(), board_clock)
        assert self.backend is not None
        self.published = []
        self.arrived = threading.Semaphore(0)
        self.running = True
        self.thread = threading.Thread(target=self.backend.run, args=(self.publish, lambda: self.running))
        self.thread.start()

    def publish(self, frame, stamp):
        self.published.append((frame, stamp))
        self.arrived.release()

    def connect(self):
        return socket.create_connection(parse_npyc_source(self.source), timeout=WAIT_S)

    def wait_for(self, count):
        for _ in range(count):
            assert self.arrived.acquire(timeout=WAIT_S), "frame not published"

    def stop(self):
        self.running = False
        self.backend.close()
        self.thread.join(WAIT_S)
        assert not self.thread.is_alive()


def test_a_v2_frame_with_an_exact_pose_is_published_with_the_mapped_exposure_stamp():
    clock = FixedClock(1234.5)
    rig = Harness(clock)
    try:
        h = header(board_time_us=7_000_000)
        with rig.connect() as unreal:
            unreal.sendall(frame_bytes(h))
            rig.wait_for(1)
        frame, stamp = rig.published[0]
        assert (stamp.kind, stamp.captured_at_s) == (CaptureStampKind.EXPOSURE, 1234.5)
        assert clock.asked == [7_000_000]
        assert frame.shape == (H, W, 3)
        assert frame.tobytes() == frame_bytes(h)[NPYC_V2_HEADER_SIZE:]
    finally:
        rig.stop()


@pytest.mark.parametrize("h, clock", [
    (header(version=1, flags=0, board_time_us=0), FixedClock(1.0)),       # v1: no board time at all
    (header(flags=POSE_STAMP_VALID), FixedClock(1.0)),                    # interpolated pose
    (header(), FixedClock(None)),                                         # no mapping yet
    (header(), None),                                                     # no board clock given
])
def test_every_other_frame_gets_a_read_stamp_never_a_synthesized_exposure(h, clock):
    rig = Harness(clock)
    try:
        with rig.connect() as unreal:
            unreal.sendall(frame_bytes(h))
            rig.wait_for(1)
        assert rig.published[0][1].kind is CaptureStampKind.READ
    finally:
        rig.stop()


def test_bgra_frames_are_published_as_bgr():
    rig = Harness()
    try:
        h = header(pixel_format=NPYC_PIXEL_FORMAT_BGRA8)
        with rig.connect() as unreal:
            unreal.sendall(frame_bytes(h))
            rig.wait_for(1)
        frame = rig.published[0][0]
        bgra = np.frombuffer(frame_bytes(h)[NPYC_V2_HEADER_SIZE:], np.uint8).reshape(H, W, 4)
        assert np.array_equal(frame, bgra[:, :, :3])
    finally:
        rig.stop()


def test_unreal_reconnecting_after_a_drop_or_a_protocol_error_is_accepted_again():
    rig = Harness()
    try:
        with rig.connect() as first:
            first.sendall(frame_bytes(header(index=0)))
            rig.wait_for(1)
        with rig.connect() as garbage:
            garbage.sendall(b"JUNK" * 20)
            try:                                    # the backend dropped this connection: EOF, or a reset
                assert garbage.recv(1) == b""       # for the bytes it left unread
            except ConnectionResetError:
                pass
        with rig.connect() as second:
            second.sendall(frame_bytes(header(index=1)) + frame_bytes(header(index=2)))
            rig.wait_for(2)
        assert len(rig.published) == 3 and rig.backend.connections == 3
    finally:
        rig.stop()


def test_stop_unblocks_a_backend_waiting_for_unreal():
    rig = Harness()
    rig.stop()                  # asserts the run thread ended while blocked in accept


def test_a_busy_port_is_reported_and_no_backend_opens():
    rig = Harness()
    try:
        assert NpycFrameCapture.open(rig.source, _Logger()) is None
    finally:
        rig.stop()


def test_the_factory_routes_npyc_sources_with_the_board_clock():
    clock = FixedClock(0.0)
    with patch("navpy.app.perception.frame_capture_factory.NpycFrameCapture.open", return_value="b") as opened:
        assert open_capture_backend("npyc://127.0.0.1:18110", _Logger(), clock) == "b"
    opened.assert_called_once_with("npyc://127.0.0.1:18110", opened.call_args.args[1], clock)


@pytest.mark.parametrize("source", ["npyc://127.0.0.1", "npyc://:1", "npyc://h:0", "rtsp://h:1"])
def test_bad_npyc_sources_are_refused(source):
    with pytest.raises(ValueError):
        parse_npyc_source(source)
