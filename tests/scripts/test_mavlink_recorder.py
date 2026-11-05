"""The verify-port recorder: whole frames, a readable tlog, a status file, isolation."""

from __future__ import annotations

import socket
import struct
import sys
import threading
import time
from pathlib import Path

import pytest
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))

import mavlink_recorder as rec  # noqa: E402


def _frame(system: int, text: bytes = b"hello", mavlink1: bool = False) -> bytes:
    link = mavlink2.MAVLink(None, srcSystem=system, srcComponent=1)
    if mavlink1:
        heartbeat = link.heartbeat_encode(1, 3, 0, 0, 0)
        link.WIRE_PROTOCOL_VERSION = "1.0"
        return heartbeat.pack(link, force_mavlink1=True)
    return link.statustext_encode(6, text).pack(link)


class Bound:
    """Binds the real exclusive socket on a free port and remembers it."""

    def __init__(self):
        self.port = None

    def __call__(self, _port):
        sock = rec.bind_exclusive(0)
        self.port = sock.getsockname()[1]
        return sock


def _send(port: int, *datagrams: bytes) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as out:
        for datagram in datagrams:
            out.sendto(datagram, ("127.0.0.1", port))


def _status(tlog: Path) -> dict:
    return rec.read_status(tlog)


def _wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_split_keeps_whole_v1_v2_and_signed_frames():
    signed = bytearray(_frame(2))
    signed[2] |= 0x01
    signed += bytes(13)
    datagram = _frame(1) + _frame(3, mavlink1=True) + bytes(signed)
    frames, rest = rec.split_frames(datagram)
    assert [len(frame) for frame in frames] == [len(_frame(1)), len(_frame(3, mavlink1=True)), len(signed)]
    assert b"".join(frames) == datagram and rest == 0


@pytest.mark.parametrize("tail", [b"\x00\x01", _frame(1)[:-1], b"\xfd"])
def test_split_counts_what_is_not_a_whole_frame(tail):
    frames, rest = rec.split_frames(_frame(1) + tail)
    assert frames == [_frame(1)] and rest == len(tail)


def test_stamp_is_big_endian_microseconds_with_the_low_bits_clear():
    (stamp,) = struct.unpack(">Q", rec.tlog_stamp(1.000007))
    assert stamp == 1000004 and stamp & 3 == 0


def test_records_whole_frames_into_a_tlog_pymavlink_reads(tmp_path, monkeypatch):
    monkeypatch.setattr(rec, "WRITE_INTERVAL_S", 0.05)
    tlog = tmp_path / "run" / "mavlink.tlog"
    bound = Bound()
    recorder = rec.MavlinkRecorder(tlog, 16550, bind=bound)
    recorder.start()
    _send(bound.port, _frame(1, b"one") + _frame(2, b"two"), _frame(3, b"three"), b"junk")
    # Written within a write interval, before any stop: a forced kill loses at most that.
    assert _wait_for(lambda: _status(tlog)["records"] == 3 and _status(tlog)["bytes"] > 0)
    assert _status(tlog)["stop_reason"] is None
    assert recorder.stop() is True

    status = _status(tlog)
    assert status["stop_reason"] == "supervisor exit" and status["error"] is None
    assert status["bytes"] == tlog.stat().st_size and status["unframed_bytes"] == 4
    reader = mavutil.mavlink_connection(str(tlog))
    texts = []
    while (message := reader.recv_match(type="STATUSTEXT")) is not None:
        texts.append((message.get_srcSystem(), message.text))
    assert texts == [(1, "one"), (2, "two"), (3, "three")]


def test_a_failed_bind_is_in_the_status_file_and_raised(tmp_path):
    tlog = tmp_path / "mavlink.tlog"

    def refused(_port):
        raise OSError(10048, "address in use")

    with pytest.raises(OSError):
        rec.MavlinkRecorder(tlog, 16551, bind=refused).start()
    status = _status(tlog)
    assert status["stop_reason"] == "not started"
    assert "could not bind verify port 16551" in status["error"]
    assert not tlog.exists()


@pytest.mark.skipif(not hasattr(socket, "SO_EXCLUSIVEADDRUSE"), reason="Windows socket option")
def test_the_bind_is_exclusive_both_ways():
    with rec.bind_exclusive(0) as held:
        port = held.getsockname()[1]
        # A manual udpin reader (SO_REUSEADDR) can no longer take its packets.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reader:
            reader.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            with pytest.raises(OSError):
                reader.bind(("0.0.0.0", port))
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reader:
        reader.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        reader.bind(("0.0.0.0", 0))
        # A reader left open makes the recorder's bind fail instead of sharing.
        with pytest.raises(OSError):
            rec.bind_exclusive(reader.getsockname()[1])


def test_the_receive_buffer_is_asked_for():
    with rec.bind_exclusive(0) as sock:
        assert sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF) >= rec.RECEIVE_BUFFER_BYTES


def test_stops_at_the_size_cap_and_keeps_only_whole_records(tmp_path, monkeypatch):
    monkeypatch.setattr(rec, "WRITE_INTERVAL_S", 0.05)
    record = 8 + len(_frame(1))
    tlog = tmp_path / "mavlink.tlog"
    bound = Bound()
    recorder = rec.MavlinkRecorder(tlog, 16550, bind=bound, max_bytes=2 * record + 1)
    recorder.start()
    _send(bound.port, _frame(1), _frame(1), _frame(1))
    assert _wait_for(lambda: _status(tlog)["stop_reason"] is not None)
    status = _status(tlog)
    assert status["stop_reason"].startswith("size cap")
    assert tlog.stat().st_size == status["bytes"] == 2 * record
    assert recorder.stop() is True


def test_a_write_failure_ends_the_record_not_the_caller(tmp_path):
    tlog = tmp_path / "mavlink.tlog"
    tlog.mkdir()  # the file cannot be opened
    recorder = rec.MavlinkRecorder(tlog, 16550, bind=Bound())
    recorder.start()
    assert recorder.stop() is True
    status = _status(tlog)
    assert status["stop_reason"] == "error" and status["error"]


def test_records_nothing_unless_the_environment_names_a_file(tmp_path):
    assert rec.start_from_environment(16550, {}) is None
    assert rec.start_from_environment(16550, {rec.TLOG_ENV: ""}) is None
    bound = Bound()
    recorder = rec.start_from_environment(16552, {rec.TLOG_ENV: str(tmp_path / "x.tlog")}, bind=bound)
    try:
        assert recorder.tlog == tmp_path / "x.tlog" and recorder.port == 16552
    finally:
        assert recorder.stop() is True


def test_the_supervisor_hook_records_the_chats_verify_port(monkeypatch):
    seen = []
    monkeypatch.setattr(rec, "start_from_environment", lambda port: seen.append(port))
    rec.start_for_chat(7)
    assert seen == [rec.ip.verify_port(7)]


def test_a_wall_clock_step_back_does_not_hold_the_writes_back(tmp_path, monkeypatch):
    monkeypatch.setattr(rec, "WRITE_INTERVAL_S", 0.05)
    wall = iter([1.8e9] + [1.8e9 - 300.0] * 100000)  # stepped back five minutes after the start
    tlog = tmp_path / "mavlink.tlog"
    bound = Bound()
    recorder = rec.MavlinkRecorder(tlog, 16550, bind=bound, clock=lambda: next(wall))
    recorder.start()
    _send(bound.port, _frame(1))
    try:
        assert _wait_for(lambda: _status(tlog)["bytes"] > 0, timeout=2.0)
    finally:
        recorder.stop()


def test_an_existing_file_counts_toward_the_cap(tmp_path, monkeypatch):
    monkeypatch.setattr(rec, "WRITE_INTERVAL_S", 0.05)
    record = 8 + len(_frame(1))
    tlog = tmp_path / "mavlink.tlog"
    tlog.write_bytes(bytes(record))
    bound = Bound()
    recorder = rec.MavlinkRecorder(tlog, 16550, bind=bound, max_bytes=2 * record + 1)
    recorder.start()
    _send(bound.port, _frame(1), _frame(1))
    assert _wait_for(lambda: _status(tlog)["stop_reason"] is not None)
    assert tlog.stat().st_size == _status(tlog)["bytes"] == 2 * record
    assert recorder.stop() is True


def test_a_status_read_waits_out_a_replace_in_progress(tmp_path, monkeypatch):
    tlog = tmp_path / "mavlink.tlog"
    rec.status_path(tlog).write_text('{"records": 1}', encoding="utf-8")
    real_read = Path.read_text
    # What Windows raises mid-replace: a refused open, then the name missing.
    refusals = iter([PermissionError(13, "Permission denied"), FileNotFoundError(2, "No such file")])

    def read_text(path, *args, **kwargs):
        error = next(refusals, None)
        if error is not None:
            raise error
        return real_read(path, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", read_text)
    assert rec.read_status(tlog) == {"records": 1}


@pytest.mark.parametrize("error", [PermissionError, FileNotFoundError])
def test_a_status_read_still_fails_when_the_error_lasts(tmp_path, monkeypatch, error):
    tlog = tmp_path / "mavlink.tlog"
    calls = []

    def refused(path, *args, **kwargs):
        calls.append(path)
        raise error()

    monkeypatch.setattr(Path, "read_text", refused)
    monkeypatch.setattr(rec, "STATUS_REPLACE_PAUSE_S", 0.0)
    with pytest.raises(error):
        rec.read_status(tlog)
    assert len(calls) == rec.STATUS_READ_ATTEMPTS


def test_status_reads_and_replaces_in_a_loop_never_fail(tmp_path):
    """The recorder's real writer replaces the file while a reader polls it.

    Before read_status retried, reads here failed with PermissionError on
    Windows dozens of times a second.
    """
    tlog = tmp_path / "mavlink.tlog"
    recorder = rec.MavlinkRecorder(tlog, 16550, bind=Bound())
    recorder._write_status()
    done = threading.Event()

    def write_forever():
        while not done.is_set():
            recorder._status["records"] += 1
            recorder._write_status()

    writer = threading.Thread(target=write_forever)
    writer.start()
    reads = 0
    try:
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            assert rec.read_status(tlog)["stop_reason"] is None
            reads += 1
    finally:
        done.set()
        writer.join()
    assert reads > 0 and recorder._status["records"] > 0
