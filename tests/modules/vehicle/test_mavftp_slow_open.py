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
ERR_EOF = 6
ERR_FILE_NOT_FOUND = 10
SESSION_TIMEOUT_S = 3.0  # FTP_SESSION_TIMEOUT in GCS_FTP.cpp
BURST_PACKETS = 500      # transfer_size in GCS_FTP.cpp
PUSH_S = 0.001           # time to push one reply
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
    """GCS_FTP.cpp ftp_worker: one request at a time, link delay each way.

    ``last_send`` is the time of the last reply pushed (``ftp.last_send_ms``,
    zeroed after a TerminateSession reply), and only the last reply is kept
    for a re-request.
    """

    def __init__(self, clock: _Clock, *, link_delay: float, open_work: float,
                 drop_requests=(), drop_replies=(), silent: bool = False) -> None:
        self.clock = clock
        self.link_delay = link_delay
        self.open_work = open_work
        self.drop_requests = set(drop_requests)  # indexes of OpenFileRO requests
        self.drop_replies = set(drop_replies)    # indexes of OpenFileRO replies
        self.silent = silent
        self.busy_until = 0.0
        self.fd_open = False
        self.current_session = None
        self.last_send = 0.0
        self.reply = None  # the last reply pushed
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
        self.busy_until = max(self.clock.now + self.link_delay, self.busy_until)
        reply = self.reply
        if (reply is not None and reply.session == op.session
                and (op.seq + 1) % 65536 == reply.seq):
            self._push(reply)  # a re-request: send the last reply again
            return
        self._dispatch(op, self.busy_until)

    def _push(self, reply: FTP_OP) -> None:
        self.busy_until += PUSH_S
        self.last_send = self.busy_until
        if reply.req_opcode == OP_TerminateSession:
            self.last_send = 0.0
        self.reply = reply
        if reply.req_opcode == OP_OpenFileRO:
            index = self.open_replies
            self.open_replies += 1
            if index in self.drop_replies:
                return
        self._deliver(self.busy_until + self.link_delay, reply)

    def _reply(self, op: FTP_OP, opcode: int, *, payload=b"", offset=0,
               burst_complete=0, seq=None) -> FTP_OP:
        payload = bytearray(payload)
        seq = (op.seq + 1) % 65536 if seq is None else seq
        return FTP_OP(seq, op.session, opcode, len(payload), op.opcode,
                      burst_complete, offset, payload)

    def _nack(self, op: FTP_OP, error: int, **kwargs) -> FTP_OP:
        return self._reply(op, OP_Nack, payload=bytes([error]), **kwargs)

    def _close(self) -> None:
        self.fd_open = False
        self.current_session = None

    def _dispatch(self, op: FTP_OP, now: float) -> None:
        other_session = op.session != self.current_session
        if other_session and op.opcode in (OP_TerminateSession, OP_ResetSessions):
            self._push(self._reply(op, OP_Ack))
            return
        if self.fd_open and other_session and now - self.last_send < SESSION_TIMEOUT_S:
            self._push(self._nack(op, ERR_INVALID_SESSION))
            return
        if self.fd_open and other_session:
            self._close()  # the old session idled out
        if op.opcode in (OP_TerminateSession, OP_ResetSessions):
            self._close()
            self._push(self._reply(op, OP_Ack))
        elif op.opcode == OP_OpenFileRO:
            if self.fd_open and now - self.last_send > SESSION_TIMEOUT_S:
                self._close()
            if self.fd_open:
                self._push(self._nack(op, ERR_FAIL))
                return
            self.fd_open = True
            self.current_session = op.session
            self.busy_until += self.open_work
            self._push(self._reply(op, OP_Ack, payload=struct.pack("<I", len(FILE))))
        elif op.opcode == OP_BurstReadFile:
            if not self.fd_open:
                self._push(self._nack(op, ERR_FILE_NOT_FOUND))
                return
            seq = (op.seq + 1) % 65536
            offset = op.offset
            for i in range(BURST_PACKETS):
                chunk = FILE[offset:offset + op.size]
                if not chunk:
                    self._push(self._nack(op, ERR_EOF, offset=offset, seq=seq))
                    return
                self._push(self._reply(op, OP_Ack, payload=chunk, offset=offset,
                                       burst_complete=int(i == BURST_PACKETS - 1), seq=seq))
                offset += len(chunk)
                seq = (seq + 1) % 65536
        else:
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

    def test_captured_file_wins_over_a_late_error_status(self):
        """The result is the status of the last packet processed; a reply
        discarded after the read completed must not fail a complete file."""

        class LateStatusFtp:
            def cmd_get(self, _args, callback, progress_callback_bytes=None):
                callback(SimpleNamespace(read=lambda: FILE))

            def process_ftp_reply(self, _operation, timeout):
                return SimpleNamespace(error_code=ERR_INVALID_SESSION,
                                       operation_name="mavlink_packet")

        adapter = VehicleFtp.__new__(VehicleFtp)
        adapter.ftp = LateStatusFtp()
        self.assertEqual(adapter.fetch_param_pck(), FILE)

    def test_silent_autopilot_fails_bounded(self):
        with self.assertRaisesRegex(RuntimeError, r"MAVFTP get .* failed"):
            self._fetch(link_delay=0.01, open_work=0.01, silent=True)
        self.assertLess(self.elapsed, 30.0)


if __name__ == "__main__":
    unittest.main()
