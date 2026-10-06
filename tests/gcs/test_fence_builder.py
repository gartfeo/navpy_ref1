"""Tests for fence_builder — polygon fence item construction and upload retry."""
import unittest

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
    MAV_CMD_NAV_FENCE_RETURN_POINT,
    MAV_FRAME_GLOBAL,
    MAV_MISSION_TYPE_FENCE,
    MAVLink_mission_item_int_message,
)

from gcs.backend.planner.fence_builder import (
    build_fence_items,
    parse_fence_items,
    upload_fence_with_retry,
    MIN_FENCE_VERTICES,
)


def _triangle():
    return [
        {"lat": 32.0, "lon": 34.0},
        {"lat": 32.01, "lon": 34.0},
        {"lat": 32.0, "lon": 34.01},
    ]


class TestBuildFenceItems(unittest.TestCase):
    def test_count_matches_vertices(self):
        items = build_fence_items(_triangle(), target_system=5)
        self.assertEqual(len(items), 3)

    def test_command_and_mission_type(self):
        items = build_fence_items(_triangle(), target_system=5)
        for it in items:
            self.assertEqual(it.command, MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION)
            self.assertEqual(it.mission_type, MAV_MISSION_TYPE_FENCE)
            self.assertEqual(it.frame, MAV_FRAME_GLOBAL)

    def test_param1_is_vertex_count_on_every_item(self):
        """ArduPilot groups a polygon by param1 = total vertex count on each item."""
        verts = _triangle()
        items = build_fence_items(verts, target_system=5)
        for it in items:
            self.assertEqual(it.param1, float(len(verts)))

    def test_latlon_scaled_to_1e7(self):
        items = build_fence_items(_triangle(), target_system=5)
        self.assertEqual(items[0].x, int(round(32.0 * 1e7)))
        self.assertEqual(items[0].y, int(round(34.0 * 1e7)))
        self.assertEqual(items[1].x, int(round(32.01 * 1e7)))

    def test_seq_is_sequential(self):
        items = build_fence_items(_triangle(), target_system=5)
        self.assertEqual([it.seq for it in items], [0, 1, 2])


def _square(lat0=33.0, lon0=35.0):
    return [
        {"lat": lat0, "lon": lon0},
        {"lat": lat0, "lon": lon0 + 0.01},
        {"lat": lat0 + 0.01, "lon": lon0 + 0.01},
        {"lat": lat0 + 0.01, "lon": lon0},
    ]


class TestBuildFenceItemsExclusions(unittest.TestCase):
    def test_inclusion_then_exclusion_runs(self):
        """Inclusion ring first, then each exclusion ring; commands + param1
        delimit the runs the way ArduPilot's loader expects."""
        items = build_fence_items(_triangle(), target_system=5, exclusions=[_square()])
        self.assertEqual(len(items), 7)  # 3 inclusion + 4 exclusion
        self.assertEqual([it.seq for it in items], [0, 1, 2, 3, 4, 5, 6])
        inc, exc = items[:3], items[3:]
        for it in inc:
            self.assertEqual(it.command, MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION)
            self.assertEqual(it.param1, 3.0)
        for it in exc:
            self.assertEqual(it.command, MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION)
            self.assertEqual(it.param1, 4.0)
            self.assertEqual(it.mission_type, MAV_MISSION_TYPE_FENCE)

    def test_multiple_exclusions_each_delimited(self):
        items = build_fence_items(
            _triangle(), target_system=5, exclusions=[_square(), _triangle()],
        )
        self.assertEqual(len(items), 3 + 4 + 3)
        self.assertEqual(items[7].param1, 3.0)  # second exclusion run count

    def test_degenerate_exclusion_skipped(self):
        """A keep-out with < 3 vertices can't be enforced, so it's dropped."""
        items = build_fence_items(
            _triangle(), target_system=5, exclusions=[[{"lat": 33.0, "lon": 35.0}]],
        )
        self.assertEqual(len(items), 3)  # only the inclusion ring


def _fence_item(command, lat, lon, count, seq=0):
    """Build one raw FENCE MISSION_ITEM_INT (as a vehicle would send back)."""
    return MAVLink_mission_item_int_message(
        target_system=5,
        target_component=0,
        seq=seq,
        frame=MAV_FRAME_GLOBAL,
        command=command,
        current=0,
        autocontinue=1,
        param1=float(count),
        param2=0.0,
        param3=0.0,
        param4=0.0,
        x=int(round(lat * 1e7)),
        y=int(round(lon * 1e7)),
        z=0.0,
        mission_type=MAV_MISSION_TYPE_FENCE,
    )


def _assert_ring_matches(tc, ring, verts):
    tc.assertEqual(len(ring), len(verts))
    for got, want in zip(ring, verts):
        tc.assertAlmostEqual(got["lat"], want["lat"], places=6)
        tc.assertAlmostEqual(got["lon"], want["lon"], places=6)


class TestParseFenceItems(unittest.TestCase):
    def test_single_inclusion_roundtrip(self):
        """build_fence_items -> parse_fence_items recovers the inclusion ring."""
        verts = _triangle()
        items = build_fence_items(verts, target_system=5)
        parsed = parse_fence_items(items)
        _assert_ring_matches(self, parsed["vertices"], verts)
        self.assertEqual(parsed["exclusions"], [])

    def test_inclusion_plus_two_exclusions_roundtrip(self):
        verts = _triangle()
        exclusions = [_square(), _square(lat0=40.0, lon0=45.0)]
        items = build_fence_items(verts, target_system=5, exclusions=exclusions)
        parsed = parse_fence_items(items)
        _assert_ring_matches(self, parsed["vertices"], verts)
        self.assertEqual(len(parsed["exclusions"]), 2)
        _assert_ring_matches(self, parsed["exclusions"][0], exclusions[0])
        _assert_ring_matches(self, parsed["exclusions"][1], exclusions[1])

    def test_return_point_item_skipped(self):
        """A FENCE_RETURN_POINT another GCS may add is skipped, not parsed."""
        items = build_fence_items(_triangle(), target_system=5)
        return_point = _fence_item(MAV_CMD_NAV_FENCE_RETURN_POINT, 33.0, 35.0, count=0, seq=99)
        parsed = parse_fence_items([return_point] + items)
        _assert_ring_matches(self, parsed["vertices"], _triangle())
        self.assertEqual(parsed["exclusions"], [])

    def test_empty_list_returns_empty(self):
        parsed = parse_fence_items([])
        self.assertEqual(parsed, {"vertices": [], "exclusions": []})

    def test_malformed_run_tolerated(self):
        """A run whose param1 count exceeds the vertices actually present must
        not raise; the parser keeps the valid vertices and stops that run."""
        v = _triangle()
        # Header claims 5 vertices, but only 2 inclusion items exist before a
        # return point interrupts the run.
        malformed = [
            _fence_item(MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, v[0]["lat"], v[0]["lon"], count=5),
            _fence_item(MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, v[1]["lat"], v[1]["lon"], count=5),
            _fence_item(MAV_CMD_NAV_FENCE_RETURN_POINT, 0.0, 0.0, count=0),
        ]
        parsed = parse_fence_items(malformed)
        # Kept the 2 valid inclusion vertices; did not raise.
        self.assertEqual(len(parsed["vertices"]), 2)
        self.assertAlmostEqual(parsed["vertices"][0]["lat"], v[0]["lat"], places=6)
        self.assertEqual(parsed["exclusions"], [])


class _FakeVehicle:
    """Minimal VehicleMav stand-in for fence upload."""

    def __init__(self, upload_ok=True, fail_times=0):
        self.target_system = 9
        self.mission_lock = _NullLock()
        self._upload_ok = upload_ok
        self._fail_times = fail_times
        self.clear_calls = 0
        self.upload_calls = 0
        self.last_items = None

    def clear_fence(self):
        self.clear_calls += 1

    def upload_fence(self, items):
        self.upload_calls += 1
        self.last_items = items
        if self._fail_times > 0:
            self._fail_times -= 1
            return False
        return self._upload_ok


class _NullLock:
    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


class TestUploadFenceWithRetry(unittest.TestCase):
    def test_rejects_too_few_vertices(self):
        v = _FakeVehicle()
        res = upload_fence_with_retry(v, _triangle()[:2])
        self.assertFalse(res.success)
        self.assertEqual(v.upload_calls, 0)
        self.assertIn(str(MIN_FENCE_VERTICES), res.error)

    def test_success_clears_then_uploads(self):
        v = _FakeVehicle(upload_ok=True)
        res = upload_fence_with_retry(v, _triangle())
        self.assertTrue(res.success)
        self.assertEqual(res.vertex_count, 3)
        self.assertEqual(v.clear_calls, 1)
        self.assertEqual(v.upload_calls, 1)
        self.assertEqual(len(v.last_items), 3)

    def test_success_trusts_ack_without_param_read(self):
        """Success is the ACK — no FENCE_TOTAL read-back (it would be stale)."""
        v = _FakeVehicle(upload_ok=True)
        # A get_parameter call here would be a regression; ensure none happens.
        v.get_parameter = lambda name: (_ for _ in ()).throw(AssertionError("read-back"))
        res = upload_fence_with_retry(v, _triangle())
        self.assertTrue(res.success)

    def test_retries_until_success(self):
        v = _FakeVehicle(upload_ok=True, fail_times=1)
        res = upload_fence_with_retry(v, _triangle(), max_retries=3)
        self.assertTrue(res.success)
        self.assertEqual(v.upload_calls, 2)

    def test_gives_up_after_max_retries(self):
        v = _FakeVehicle(upload_ok=False)
        res = upload_fence_with_retry(v, _triangle(), max_retries=2)
        self.assertFalse(res.success)
        self.assertEqual(v.upload_calls, 2)

    def test_exclusions_counted_in_total(self):
        """vertex_count (→ FENCE_TOTAL) includes inclusion + exclusion items."""
        v = _FakeVehicle(upload_ok=True)
        res = upload_fence_with_retry(v, _triangle(), exclusions=[_square()])
        self.assertTrue(res.success)
        self.assertEqual(res.vertex_count, 7)
        self.assertEqual(len(v.last_items), 7)


if __name__ == "__main__":
    unittest.main()
