"""Tests for containers.js — grouping UAVs into launch containers (3 by sys_id)."""
import unittest
from tests.gcs.js_runner import run_node
import json
import os
import re


_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils", "containers.js",
))


def _strip_es_modules(src):
    out = []
    for line in src.split("\n"):
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


_JS = _strip_es_modules(open(_FILE, encoding="utf-8").read())


def _run_js(script):
    result = run_node(_JS + "\n" + script, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


class TestGroupVehiclesByContainer(unittest.TestCase):
    def test_empty(self):
        self.assertEqual(json.loads(_run_js(
            "console.log(JSON.stringify(groupVehiclesByContainer([])));")), [])

    def test_null(self):
        self.assertEqual(json.loads(_run_js(
            "console.log(JSON.stringify(groupVehiclesByContainer(null)));")), [])

    def test_default_is_three(self):
        self.assertEqual(_run_js("console.log(UAVS_PER_CONTAINER);"), "3")

    def test_single_container_for_three_or_fewer(self):
        out = _run_js("""
            const g = groupVehiclesByContainer([{sys_id:1},{sys_id:2},{sys_id:3}]);
            console.log(JSON.stringify(g.map(x => [x.container, x.items.length])));
        """)
        self.assertEqual(json.loads(out), [[1, 3]])

    def test_chunks_of_three(self):
        out = _run_js("""
            const list = [1,2,3,4,5,6,7].map(sys_id => ({sys_id}));
            const g = groupVehiclesByContainer(list);
            console.log(JSON.stringify(g.map(x => [x.container, x.items.map(i=>i.v.sys_id)])));
        """)
        self.assertEqual(json.loads(out), [
            [1, [1, 2, 3]],
            [2, [4, 5, 6]],
            [3, [7]],
        ])

    def test_assignment_by_ascending_sys_id(self):
        # Unsorted input is assigned to containers by sys_id order.
        out = _run_js("""
            const list = [{sys_id:12},{sys_id:3},{sys_id:7},{sys_id:1}];
            const g = groupVehiclesByContainer(list);
            console.log(JSON.stringify(g.map(x => x.items.map(i=>i.v.sys_id))));
        """)
        self.assertEqual(json.loads(out), [[1, 3, 7], [12]])

    def test_global_index_preserved(self):
        # Each item keeps its position in the ORIGINAL list (for zone alignment),
        # even though grouping sorts by sys_id.
        out = _run_js("""
            const list = [{sys_id:30},{sys_id:10},{sys_id:20},{sys_id:5}];
            const g = groupVehiclesByContainer(list);
            const flat = g.flatMap(x => x.items.map(i => [i.v.sys_id, i.index]));
            console.log(JSON.stringify(flat));
        """)
        # sorted by sys_id: 5(idx3),10(idx1),20(idx2) -> container 1; 30(idx0) -> container 2
        self.assertEqual(json.loads(out), [[5, 3], [10, 1], [20, 2], [30, 0]])

    def test_custom_per_container(self):
        out = _run_js("""
            const list = [1,2,3,4,5].map(sys_id => ({sys_id}));
            const g = groupVehiclesByContainer(list, 2);
            console.log(JSON.stringify(g.map(x => x.items.length)));
        """)
        self.assertEqual(json.loads(out), [2, 2, 1])


class TestContainersFromSysIds(unittest.TestCase):
    def test_empty(self):
        self.assertEqual(json.loads(_run_js(
            "console.log(JSON.stringify(containersFromSysIds([])));")), [])

    def test_chunks_of_three_sorted(self):
        out = _run_js(
            "console.log(JSON.stringify(containersFromSysIds([12,3,7,1,9,5,11])));")
        self.assertEqual(json.loads(out), [[1, 3, 5], [7, 9, 11], [12]])

    def test_custom_size(self):
        out = _run_js(
            "console.log(JSON.stringify(containersFromSysIds([1,2,3,4,5], 2)));")
        self.assertEqual(json.loads(out), [[1, 2], [3, 4], [5]])


if __name__ == "__main__":
    unittest.main()
