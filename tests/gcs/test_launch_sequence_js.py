"""Tests for applyLaunchOrder / orderVehiclesByLaunch in launchSequence.js."""
import json
import os
from tests.gcs.js_runner import run_node
import unittest


_UTIL_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
    "launchSequence.js",
))

_raw = open(_UTIL_PATH, encoding="utf-8").read()
_JS_SRC = _raw.replace("export function ", "function ")


def _run_js(script):
    result = run_node(_JS_SRC + "\n" + script, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


def _order(sys_ids, launch_order):
    out = _run_js(
        f"console.log(JSON.stringify(applyLaunchOrder({json.dumps(sys_ids)}, {json.dumps(launch_order)})));"
    )
    return json.loads(out)


class TestApplyLaunchOrder(unittest.TestCase):
    def test_empty_order_keeps_input(self):
        self.assertEqual(_order([1, 2, 3], []), [1, 2, 3])

    def test_null_order_keeps_input(self):
        self.assertEqual(_order([1, 2, 3], None), [1, 2, 3])

    def test_full_reorder(self):
        self.assertEqual(_order([1, 2, 3], [3, 1, 2]), [3, 1, 2])

    def test_partial_order_listed_first(self):
        # Only 3 listed → it leads, rest keep original order
        self.assertEqual(_order([1, 2, 3], [3]), [3, 1, 2])

    def test_unknown_ids_in_order_ignored(self):
        self.assertEqual(_order([1, 2], [9, 2, 5, 1]), [2, 1])

    def test_duplicates_in_order_dedup(self):
        self.assertEqual(_order([1, 2, 3], [2, 2, 1]), [2, 1, 3])

    def test_missing_vehicle_from_order_dropped(self):
        # 2 not currently present → skipped; 3 and 1 ordered, then rest
        self.assertEqual(_order([1, 3, 4], [2, 3, 1]), [3, 1, 4])


def _resolve(channel_map, sys_ids):
    out = _run_js(
        f"console.log(JSON.stringify(resolveChannelMap({json.dumps(channel_map)}, {json.dumps(sys_ids)})));"
    )
    return json.loads(out)


class TestResolveChannelMap(unittest.TestCase):
    """Frontend channel resolution must match backend _resolve_channel_map."""

    def test_out_of_range_sysids_auto_assigned(self):
        # sys_ids 161/162/163 -> channels 1/2/3
        self.assertEqual(_resolve({}, [163, 161, 162]), {"161": 1, "162": 2, "163": 3})

    def test_explicit_wins_and_is_skipped(self):
        r = _resolve({"161": 5}, [161, 162, 163])
        self.assertEqual(r["161"], 5)
        self.assertEqual(r["162"], 1)
        self.assertEqual(r["163"], 2)

    def test_empty_string_value_treated_as_unmapped(self):
        r = _resolve({"1": ""}, [1, 2])
        self.assertEqual(r["1"], 1)
        self.assertEqual(r["2"], 2)

    def test_more_vehicles_than_channels(self):
        r = _resolve({}, [1, 2, 3, 4, 5, 6, 7])
        self.assertEqual(sorted(r.values()), [1, 2, 3, 4, 5, 6])
        self.assertNotIn("7", r)


class TestOrderVehiclesByLaunch(unittest.TestCase):
    def test_orders_objects(self):
        out = _run_js(
            "console.log(JSON.stringify(orderVehiclesByLaunch("
            "[{\"sys_id\":1,\"n\":\"a\"},{\"sys_id\":2,\"n\":\"b\"},{\"sys_id\":3,\"n\":\"c\"}],"
            "[3,1])));"
        )
        result = json.loads(out)
        self.assertEqual([v["sys_id"] for v in result], [3, 1, 2])
        self.assertEqual(result[0]["n"], "c")


def _select(vehicles, is_container, planned_count):
    out = _run_js(
        "console.log(JSON.stringify(selectLaunchVehicles("
        f"{json.dumps(vehicles)}, {json.dumps(is_container)}, {json.dumps(planned_count)}"
        ")));"
    )
    return json.loads(out)


class TestSelectLaunchVehicles(unittest.TestCase):
    def test_container_uses_all_connected_vehicles_not_plan_count(self):
        vehicles = [{"sys_id": 161}, {"sys_id": 162}, {"sys_id": 163}]
        self.assertEqual(
            [v["sys_id"] for v in _select(vehicles, True, 2)],
            [161, 162, 163],
        )

    def test_container_does_not_invent_disconnected_vehicles(self):
        vehicles = [{"sys_id": 161}, {"sys_id": 162}]
        self.assertEqual(
            [v["sys_id"] for v in _select(vehicles, True, 3)],
            [161, 162],
        )

    def test_non_container_keeps_plan_sized_roster(self):
        vehicles = [{"sys_id": 1}, {"sys_id": 2}, {"sys_id": 3}]
        self.assertEqual(
            [v["sys_id"] for v in _select(vehicles, False, 2)],
            [1, 2],
        )


def _prepared_order(requested, prepared, launch_order):
    out = _run_js(
        "console.log(JSON.stringify(resolvePreparedLaunchOrder("
        f"{json.dumps(requested)}, {json.dumps(prepared)}, {json.dumps(launch_order)}"
        ")));"
    )
    return json.loads(out)


class TestResolvePreparedLaunchOrder(unittest.TestCase):
    def test_backend_roster_restores_omitted_connected_uav(self):
        self.assertEqual(
            _prepared_order([161, 162], [161, 162, 163], None),
            [161, 162, 163],
        )

    def test_backend_roster_still_uses_operator_launch_order(self):
        self.assertEqual(
            _prepared_order([161, 162], [161, 162, 163], [163, 161]),
            [163, 161, 162],
        )

    def test_older_backend_falls_back_to_requested_roster(self):
        self.assertEqual(
            _prepared_order([1, 2, 3], None, [3, 1]),
            [3, 1, 2],
        )


if __name__ == "__main__":
    unittest.main()
