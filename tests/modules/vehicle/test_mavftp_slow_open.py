"""MAVFTP param.pck fetch over a slow or lossy link.

The fake autopilot follows ArduPilot's GCS_FTP.cpp: one global session, a
cached last reply replayed for a repeated request, open refused while a file
is open, and burst reads. A simulated clock lets the test model an autopilot
that takes longer than the client's 1 s open retry to answer, as under load.
"""
from __future__ import annotations

import struct
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from navpy.plugins.vehicle.ardupilot._mavftp import _upstream_mavftp as upstream
from navpy.plugins.vehicle.ardupilot._mavftp._upstream_mavftp import MAVFTP
from navpy.plugins.vehicle.ardupilot._mavftp._upstream_mavftp_op import (
    FTP_OP,
    OP_Ack,
    OP_BurstReadFile,
    OP_Nack,
    OP_OpenFileRO,
    OP_ResetSessions,
    OP_TerminateSession,
)
from navpy.plugins.vehicle.ardupilot._mavftp.vehicle_ftp import VehicleFtp

ERR_FAIL = 1
ERR_INVALID_SESSION = 4
SESSION_TIMEOUT_S = 3.0  # FTP_SESSION_TIMEOUT in GCS_FTP.cpp
FILE = bytes(range(256)) * 3 + bytes(range(100))  # 868 bytes, not a burst multiple


class _Clock:
    def __init__(self) -> None:
        self.now = 1000.0

    def time(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


def _parse(payload) -> FTP_OP:
    seq, session, opcode, size, req_opcode, burst_complete, _pad, offset = (
        struct.unpack("<HBBBBBBI", bytes(payload[:12]))
    )
    return FTP_OP(seq, session, opcode, size, req_opcode, burst_complete, offset,
                  bytearray(payload[12:12 + size]))


class _FakeAutopilot:
    """Single-threaded FTP server with link delay and per-opcode work time."""

    def __init__(self, clock: _Clock, *, link_delay: float, open_work: float,
                 drop_requests=(), drop_replies=(), silent: bool = False) -> None:
        self.clock = clock
        self.link_delay = link_delay
        self.open_work = open_work
        self.drop_requests = set(drop_requests)  # indexes of OpenFileRO requests
        self.drop_replies = set(drop_replies)    # indexes of OpenFileRO replies
        self.silent = silent
        self.busy_until = 0.0
        self.session = None  # session holding the open file, None when closed
        self.last_activity = 0.0
        self.cached = None   # (reply seq, session, replies)
        self.opens_seen = 0
        self.open_replies = 0
        self.inbox: list[tuple[float, SimpleNamespace]] = []
        self.source_system = 255
        self.source_component = 0
        self.mav = SimpleNamespace(file_transfer_protocol_send=self._receive)

    # client -> autopilot
    def _receive(self, _network, _target_system, _target_component, payload) -> None:
        op = _parse(payload)
        if op.opcode == OP_OpenFileRO:
            index = self.opens_seen
            self.opens_seen += 1
            if index in self.drop_requests:
                return
        if self.silent:
            return
        arrival = self.clock.now + self.link_delay
        start = max(arrival, self.busy_until)
        replies, work = self._handle(op, start)
        self.busy_until = start + work
        for reply in replies:
            if reply.req_opcode == OP_OpenFileRO:
                index = self.open_replies
                self.open_replies += 1
                if index in self.drop_replies:
                    continue
            self._deliver(self.busy_until + self.link_delay, reply)

    def _handle(self, op: FTP_OP, now: float):
        cached = self.cached
        if (cached is not None and cached[1] == op.session
                and (op.seq + 1) % 65536 == cached[0]):
            return cached[2], 0.001  # ftp_worker replays the last reply
        self.last_activity = now
        replies, work = self._process(op, now)
        if replies:
            self.cached = (replies[-1].seq, op.session, replies)
        return replies, work

    def _reply(self, op: FTP_OP, opcode: int, *, payload=None, offset=0,
               burst_complete=0) -> FTP_OP:
        payload = bytearray(payload or b"")
        return FTP_OP((op.seq + 1) % 65536, op.session, opcode, len(payload),
                      op.opcode, burst_complete, offset, payload)

    def _nack(self, op: FTP_OP, error: int) -> FTP_OP:
        return self._reply(op, OP_Nack, payload=bytes([error]))

    def _process(self, op: FTP_OP, now: float):
        if op.opcode == OP_ResetSessions:
            self.session = None
            return [self._reply(op, OP_Ack)], 0.001
        if op.opcode == OP_TerminateSession:
            if op.session == self.session:
                self.session = None
            return [self._reply(op, OP_Ack)], 0.001
        if op.opcode == OP_OpenFileRO:
            if self.session is not None:
                if op.session == self.session:
                    return [self._nack(op, ERR_FAIL)], 0.001
                if now - self.last_activity < SESSION_TIMEOUT_S:
                    return [self._nack(op, ERR_INVALID_SESSION)], 0.001
            self.session = op.session
            size = struct.pack("<I", len(FILE))
            return [self._reply(op, OP_Ack, payload=size)], self.open_work
        if op.opcode == OP_BurstReadFile:
            if op.session != self.session:
                return [self._nack(op, ERR_INVALID_SESSION)], 0.001
            replies = []
            offset = op.offset
            while offset < len(FILE):
                chunk = FILE[offset:offset + op.size]
                last = offset + len(chunk) >= len(FILE)
                replies.append(self._reply(op, OP_Ack, payload=chunk, offset=offset,
                                           burst_complete=int(last)))
                offset += len(chunk)
            return replies, 0.001 * len(replies)
        raise AssertionError(f"unexpected opcode {op.opcode}")

    def _deliver(self, when: float, reply: FTP_OP) -> None:
        payload = reply.pack()
        message = SimpleNamespace(
            get_type=lambda: "FILE_TRANSFER_PROTOCOL",
            target_system=self.source_system,
            target_component=self.source_component,
            payload=payload,
        )
        self.inbox.append((when, message))
        self.inbox.sort(key=lambda item: item[0])

    # autopilot -> client
    def recv_match(self, *, type=None, timeout: float = 0.1, **_kwargs):
        if self.inbox and self.inbox[0][0] <= self.clock.now + timeout:
            when, message = self.inbox.pop(0)
            self.clock.now = max(self.clock.now, when)
            return message
        self.clock.now += timeout
        return None


class TestMavFtpSlowOpen(unittest.TestCase):
    def _fetch(self, **autopilot_kwargs):
        clock = _Clock()
        autopilot = _FakeAutopilot(clock, **autopilot_kwargs)
        with patch.object(upstream, "time", clock):
            adapter = VehicleFtp.__new__(VehicleFtp)
            adapter.ftp = MAVFTP(autopilot, target_system=1, target_component=1)
            start = clock.now
            try:
                return adapter.fetch_param_pck(timeout=30.0), autopilot
            finally:
                self.elapsed = clock.now - start

    def test_open_slower_than_retry_interval_still_completes(self):
        """An open answered after the client's 1 s retry is not abandoned."""
        data, autopilot = self._fetch(link_delay=0.15, open_work=1.2)
        self.assertEqual(data, FILE)
        self.assertGreaterEqual(autopilot.opens_seen, 2)  # the retry happened

    def test_lost_open_request_is_retried(self):
        data, _ = self._fetch(link_delay=0.05, open_work=0.02, drop_requests={0})
        self.assertEqual(data, FILE)

    def test_lost_open_reply_is_replayed(self):
        data, _ = self._fetch(link_delay=0.05, open_work=0.02, drop_replies={0})
        self.assertEqual(data, FILE)

    def test_fast_link_opens_once(self):
        data, autopilot = self._fetch(link_delay=0.01, open_work=0.01)
        self.assertEqual(data, FILE)
        self.assertEqual(autopilot.opens_seen, 1)

    def test_silent_autopilot_fails_bounded(self):
        with self.assertRaisesRegex(RuntimeError, r"MAVFTP get .* failed"):
            self._fetch(link_delay=0.01, open_work=0.01, silent=True)
        self.assertLess(self.elapsed, 30.0)


if __name__ == "__main__":
    unittest.main()
