"""Regression tests for NavPy's MAVFTP callback download path."""
from __future__ import annotations

import time
import threading
import unittest
from io import BytesIO
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from pymavlink.dialects.v20.ardupilotmega import MAV_TYPE_ONBOARD_CONTROLLER

from navpy.modules.vehicle._mavftp._upstream_mavftp import FtpError, MAVFTP
from navpy.modules.vehicle._mavftp._upstream_mavftp_op import (
    FTP_OP,
    OP_Ack,
    OP_BurstReadFile,
    OP_Nack,
    OP_ResetSessions,
)
from navpy.modules.vehicle._mavftp.vehicle_ftp import VehicleFtp, _BridgeMaster


class TestMavFtpCallbackDownload(unittest.TestCase):
    def test_failure_callback_preserves_transport_error(self):
        """A failed transfer reports its MAVFTP error instead of dereferencing None."""

        class FailedFtp:
            def cmd_get(self, _args, callback, progress_callback_bytes=None):
                callback(None)

            def process_ftp_reply(self, _operation, timeout):
                return SimpleNamespace(
                    error_code=FtpError.Fail,
                    operation_name="BurstReadFile",
                )

        adapter = VehicleFtp.__new__(VehicleFtp)
        adapter.ftp = FailedFtp()

        with self.assertRaisesRegex(
            RuntimeError,
            r"MAVFTP get .* failed: BurstReadFile error=1",
        ):
            adapter.fetch_param_pck()

    def test_callback_download_skips_local_write_for_query_path(self):
        """`@PARAM/param.pck?withdefaults=1` is a remote path, not a file name.

        The full-param editor uses a callback to keep the downloaded blob in
        memory. On Windows, writing the callback result back to a derived local
        name like `param.pck?withdefaults=1` raises `OSError: invalid argument`.
        """
        data = b"param-pck-bytes"
        captured = {}

        ftp = MAVFTP.__new__(MAVFTP)
        ftp.fh = BytesIO(data)
        ftp.op_start = time.time() - 0.1
        ftp.read_gaps = []
        ftp.reached_eof = True
        ftp.read_total = len(data)
        ftp.requested_size = len(data)
        ftp.requested_offset = 0
        ftp.callback = lambda fh: captured.setdefault("data", fh.read())
        ftp.filename = "param.pck?withdefaults=1"
        ftp.temp_filename = "unused-temp-file"
        ftp.get_result = None
        ftp._MAVFTP__terminate_session = lambda: None

        with patch("builtins.open") as open_mock:
            finished = ftp._MAVFTP__check_read_finished()

        self.assertTrue(finished)
        self.assertEqual(captured["data"], data)
        self.assertEqual(ftp.get_result, data)
        open_mock.assert_not_called()


class TestMavFtpEstimatedParamSize(unittest.TestCase):
    def _ftp_for_read(self, initial_size: int) -> MAVFTP:
        ftp = MAVFTP.__new__(MAVFTP)
        ftp.fh = BytesIO()
        ftp.op_start = time.time() - 0.1
        ftp.read_gaps = []
        ftp.read_gap_times = {}
        ftp.reached_eof = False
        ftp.read_total = 0
        ftp.requested_size = initial_size
        ftp.requested_offset = 0
        ftp.remote_file_size = initial_size
        ftp.callback = lambda _fh: None
        ftp.callback_progress = None
        ftp.callback_progress_bytes = None
        ftp.filename = "-"
        ftp.get_result = None
        ftp.burst_size = 80
        ftp.session = 0
        ftp.duplicates = 0
        ftp.ftp_settings = SimpleNamespace(pkt_loss_tx=0, debug=0)
        ftp._MAVFTP__terminate_session = lambda: None
        ftp._MAVFTP__check_read_send = lambda: None
        return ftp

    def test_short_final_burst_resets_estimated_size_to_eof(self):
        data = b"x" * 63
        ftp = self._ftp_for_read(initial_size=120)
        events = []
        ftp.callback_progress_bytes = events.append
        op = FTP_OP(
            seq=0,
            session=0,
            opcode=OP_Ack,
            size=len(data),
            req_opcode=OP_BurstReadFile,
            burst_complete=1,
            offset=0,
            payload=bytearray(data),
        )

        with patch("logging.warning") as warning_mock:
            result = ftp._MAVFTP__handle_burst_read(op, None)

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(ftp.requested_size, len(data))
        self.assertEqual(ftp.get_result, data)
        self.assertGreaterEqual(len(events), 2)
        self.assertEqual(events[0]["bytes_read"], len(data))
        self.assertEqual(events[0]["size_estimate"], 120)
        self.assertFalse(events[0]["done"])
        self.assertIn(
            {"bytes_read": len(data), "size_estimate": 120, "done": False, "total_bytes": len(data)},
            events,
        )
        self.assertEqual(events[-1]["bytes_read"], len(data))
        self.assertEqual(events[-1]["total_bytes"], len(data))
        self.assertTrue(events[-1]["done"])
        warning_mock.assert_not_called()

    def test_clean_eof_nack_resets_estimated_size_to_file_position(self):
        data = b"x" * 80
        ftp = self._ftp_for_read(initial_size=120)
        events = []
        ftp.callback_progress_bytes = events.append
        ftp.fh.write(data)
        ftp.read_total = len(data)
        op = FTP_OP(
            seq=0,
            session=0,
            opcode=OP_Nack,
            size=1,
            req_opcode=OP_BurstReadFile,
            burst_complete=1,
            offset=len(data),
            payload=bytearray([FtpError.EndOfFile]),
        )

        with patch("logging.warning") as warning_mock:
            result = ftp._MAVFTP__handle_burst_read(op, None)

        self.assertEqual(result.error_code, FtpError.Success)
        self.assertEqual(ftp.requested_size, len(data))
        self.assertEqual(ftp.get_result, data)
        self.assertIn(
            {"bytes_read": len(data), "size_estimate": 120, "done": False, "total_bytes": len(data)},
            events,
        )
        self.assertEqual(events[-1]["bytes_read"], len(data))
        self.assertEqual(events[-1]["total_bytes"], len(data))
        self.assertTrue(events[-1]["done"])
        warning_mock.assert_not_called()


class TestMavFtpBridgeMaster(unittest.TestCase):
    def test_bridge_exposes_connection_source_identity(self):
        from navpy.modules.vehicle.mav_transport import MavTransport
        from navpy.modules.vehicle.vehicle_identity import VehicleIdentity

        connection = SimpleNamespace(
            mav=SimpleNamespace(srcSystem=255, srcComponent=0),
        )
        transport = MavTransport(connection, threading.RLock())
        identity = VehicleIdentity(3, 3, 191, MAV_TYPE_ONBOARD_CONTROLLER)

        master = _BridgeMaster(transport, identity, target_component=1)

        self.assertEqual(master.target_system, 3)
        self.assertEqual(master.target_component, 1)
        self.assertEqual(master.source_system, 255)
        self.assertEqual(master.source_component, 0)

    def test_reset_sessions_ack_targeted_to_local_source_is_accepted(self):
        ftp = MAVFTP.__new__(MAVFTP)
        ftp.master = SimpleNamespace(source_system=255, source_component=0)
        ftp.ftp_settings = SimpleNamespace(debug=0, pkt_loss_rx=0)
        ftp.session = 0
        ftp.last_op_time = time.time()
        ftp.last_op = FTP_OP(
            seq=0,
            session=0,
            opcode=OP_ResetSessions,
            size=0,
            req_opcode=0,
            burst_complete=0,
            offset=0,
            payload=None,
        )

        msg = SimpleNamespace(
            target_system=255,
            target_component=0,
            payload=FTP_OP(
                seq=0,
                session=0,
                opcode=OP_Ack,
                size=0,
                req_opcode=OP_ResetSessions,
                burst_complete=0,
                offset=0,
                payload=None,
            ).pack(),
            get_type=lambda: "FILE_TRANSFER_PROTOCOL",
        )

        result = ftp._MAVFTP__mavlink_packet(msg)

        self.assertEqual(result.operation_name, "ResetSessions")
        self.assertEqual(result.error_code, FtpError.Success)


if __name__ == "__main__":
    unittest.main()
