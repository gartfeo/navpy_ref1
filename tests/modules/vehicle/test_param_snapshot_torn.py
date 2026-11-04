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

from navpy.plugins.vehicle.ardupilot._mavftp.param_pck import (
    AP_TYPE_FLOAT,
    AP_TYPE_INT8,
    FLAG_HAS_DEFAULT,
    PCK_MAGIC_WITH_DEFAULTS,
    ParamPckError,
    parse_param_pck,
)
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


class _StubFtp:
    def __init__(self, blobs) -> None:
        self.blobs = list(blobs)
        self.fetches = 0

        self.timeouts: list[float] = []
        self.now = 0.0
        self.cost = 0.0

    def fetch_param_pck(self, *, timeout: float, **_kwargs) -> bytes:
        self.fetches += 1
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
        snapshot = client.fetch(timeout=1.0)
        self.assertEqual(ftp.fetches, 2)
        first = snapshot.pck.records[0]
        self.assertEqual((first.value, first.default), (2.5, 2.0))

    def test_refetch_is_bounded(self):
        client, ftp = _client([_torn()] * TORN_PCK_FETCHES)
        with self.assertRaises(ParamPckError):
            client.fetch(timeout=1.0)
        self.assertEqual(ftp.fetches, TORN_PCK_FETCHES)

    def test_every_layout_join_is_refused_or_one_whole_table(self):
        # A gap read joins one layout's head to the other's tail at some cut.
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
                client.fetch(timeout=10.0)
        self.assertEqual(ftp.timeouts, [10.0, 6.0, 2.0])

    def test_no_refetch_once_the_timeout_is_spent(self):
        client, ftp = _client([_torn()] * TORN_PCK_FETCHES)
        ftp.cost = 10.0
        with mock.patch.object(rt, "monotonic", lambda: ftp.now):
            with self.assertRaises(ParamPckError):
                client.fetch(timeout=10.0)
        self.assertEqual(ftp.fetches, 1)

    def test_good_blob_is_fetched_once(self):
        client, ftp = _client([_pck(2.0)])
        client.fetch(timeout=1.0)
        self.assertEqual(ftp.fetches, 1)


if __name__ == "__main__":
    unittest.main()
