"""A torn with-defaults param.pck is fetched again, a bounded number of times.

ArduPilot packs param.pck again for each read. With defaults, a record holds
its default only while the value differs from it, so when a value leaves its
default between a burst and a gap read, the joined bytes mix two layouts.
"""
from __future__ import annotations

import struct
import unittest
from types import SimpleNamespace
from unittest import mock

from navpy.core.kernel import runtime_time as rt
from navpy.plugins.vehicle.ardupilot._mavftp._upstream_mavftp import MAVFTP

from navpy.plugins.vehicle.ardupilot._mavftp.param_pck import (
    AP_TYPE_FLOAT,
    AP_TYPE_INT8,
    FLAG_HAS_DEFAULT,
    PCK_MAGIC_WITH_DEFAULTS,
    ParamPckError,
    parse_param_pck,
)
from navpy.plugins.vehicle.ardupilot._mavftp.vehicle_ftp import VehicleFtp
from navpy.plugins.vehicle.ardupilot.parameter_snapshot import (
    TORN_PCK_FETCHES,
    ParameterSnapshotClient,
)

NAMES = [b"ARSPD_RATIO", b"BATT_CAP", b"CRUISE_SPD", b"DOCK_ALT", b"ELEV_REV"]


def _pck(first_value: float) -> bytes:
    """Five float records; the first carries its default 2.0 only when moved off it."""
    out = struct.pack("<HHH", PCK_MAGIC_WITH_DEFAULTS, len(NAMES), len(NAMES))
    for i, name in enumerate(NAMES):
        value = first_value if i == 0 else float(i)
        moved = i == 0 and value != 2.0
        flags = FLAG_HAS_DEFAULT if moved else 0
        out += bytes([(flags << 4) | AP_TYPE_FLOAT, (len(name) - 1) << 4]) + name
        out += struct.pack("<f", value) + (struct.pack("<f", 2.0) if moved else b"")
    return out


def _torn() -> bytes:
    """Bytes 0..19 read at the default, the rest re-read after the value moved."""
    before, after = _pck(2.0), _pck(2.5)
    return before[:20] + after[20:]


def _real_ftp() -> VehicleFtp:
    """A VehicleFtp over the real MAVFTP client, with a silent link."""
    master = SimpleNamespace(
        mav=SimpleNamespace(file_transfer_protocol_send=lambda *_a, **_k: None),
        recv_match=lambda **_k: None,
    )
    adapter = VehicleFtp.__new__(VehicleFtp)
    adapter.ftp = MAVFTP(master, target_system=1, target_component=1)
    return adapter


SHORTEST_TIMEOUT_S = _real_ftp().shortest_timeout_s


class _StubFtp:
    shortest_timeout_s = SHORTEST_TIMEOUT_S

    def __init__(self, blobs) -> None:
        self.blobs = list(blobs)
        self.fetches = 0

        self.timeouts: list[float] = []
        self.now = 0.0
        self.cost = 0.0

    def fetch_param_pck(self, *, timeout: float, **_kwargs) -> bytes:
        self.fetches += 1
        # The real client refuses this (process_ftp_reply's assert).
        assert timeout > self.shortest_timeout_s, timeout
        self.timeouts.append(timeout)
        self.now += self.cost
        return self.blobs.pop(0)


def _client(blobs) -> tuple[ParameterSnapshotClient, _StubFtp]:
    client = ParameterSnapshotClient(None, SimpleNamespace(target_system=1), None)
    client._ftp = _StubFtp(blobs)
    return client, client._ftp


class TestTornParamPck(unittest.TestCase):
    def test_fixture_is_torn(self):
        for good in (_pck(2.0), _pck(2.5)):
            parse_param_pck(good, defaults_requested=True)
        with self.assertRaises(ParamPckError):
            parse_param_pck(_torn(), defaults_requested=True)

    def test_torn_blob_is_fetched_again(self):
        client, ftp = _client([_torn(), _pck(2.5)])
        snapshot = client.fetch(timeout=30.0)
        self.assertEqual(ftp.fetches, 2)
        first = snapshot.pck.records[0]
        self.assertEqual((first.value, first.default), (2.5, 2.0))

    def test_refetch_is_bounded(self):
        client, ftp = _client([_torn()] * TORN_PCK_FETCHES)
        with self.assertRaises(ParamPckError):
            client.fetch(timeout=30.0)
        self.assertEqual(ftp.fetches, TORN_PCK_FETCHES)

    def test_every_head_tail_join_is_refused_or_one_whole_table(self):
        # One layout's head joined to the other's tail at any cut. An interior
        # gap (old head, new middle, old tail) can parse as a mixed table:
        # todo 272 keeps that open.
        a, b = _pck(2.0), _pck(2.5)
        wholes = {parse_param_pck(x, defaults_requested=True).records for x in (a, b)}
        for before, after in ((a, b), (b, a)):
            for cut in range(1, max(len(before), len(after))):
                joined = before[:cut] + after[cut:]
                try:
                    records = parse_param_pck(joined, defaults_requested=True).records
                except ParamPckError:
                    continue
                self.assertIn(records, wholes, f"cut {cut} accepted a mixed table")

    def test_refetches_share_one_timeout(self):
        client, ftp = _client([_torn()] * TORN_PCK_FETCHES)
        ftp.cost = 4.0
        with mock.patch.object(rt, "monotonic", lambda: ftp.now):
            with self.assertRaises(ParamPckError):
                client.fetch(timeout=20.0)
        self.assertEqual(ftp.timeouts, [20.0, 16.0, 12.0])

    def test_no_refetch_when_the_rest_is_too_short_for_the_client(self):
        client, ftp = _client([_torn()] * TORN_PCK_FETCHES)
        ftp.cost = 10.0 - SHORTEST_TIMEOUT_S  # leaves exactly the shortest
        with mock.patch.object(rt, "monotonic", lambda: ftp.now):
            with self.assertRaises(ParamPckError):
                client.fetch(timeout=10.0)
        self.assertEqual(ftp.fetches, 1)

    def test_the_timeout_checked_is_the_timeout_passed(self):
        # Time that passes before the refetch (here: while the warning is
        # logged) must not shrink the value handed to the client below its
        # bound: the stub asserts as the real client does.
        client, ftp = _client([_torn(), _pck(2.5)])
        ftp.cost = 10.0 - SHORTEST_TIMEOUT_S - 0.1  # 0.1 s above the bound after it
        slow_log = mock.patch(
            "navpy.plugins.vehicle.ardupilot.parameter_snapshot.log.warning",
            lambda *_a: setattr(ftp, "now", ftp.now + 0.2),
        )
        with mock.patch.object(rt, "monotonic", lambda: ftp.now), slow_log:
            with self.assertRaises(ParamPckError):
                client.fetch(timeout=10.0)
        self.assertEqual(ftp.fetches, 1)

    def test_the_real_client_refuses_the_shortest_timeout(self):
        adapter = _real_ftp()
        with self.assertRaisesRegex(AssertionError, "idle_detection_time"):
            adapter.ftp.process_ftp_reply("OpenFileRO", timeout=adapter.shortest_timeout_s)

    def test_good_blob_is_fetched_once(self):
        client, ftp = _client([_pck(2.0)])
        client.fetch(timeout=30.0)
        self.assertEqual(ftp.fetches, 1)


if __name__ == "__main__":
    unittest.main()
