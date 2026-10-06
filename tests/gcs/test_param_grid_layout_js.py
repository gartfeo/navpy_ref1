"""Tests for src/gcs/frontend/src/utils/paramGridLayout.js.

Locks the single-source-of-truth column math so the sticky header and the
scrolling body of VirtualParamGrid can never diverge when the compare File
column (or, later, the selection checkbox) is added.
"""
import json
import os
from tests.gcs.js_runner import run_node
import unittest


_UTIL = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils", "paramGridLayout.js",
))

_JS_SRC = (
    open(_UTIL, encoding="utf-8").read()
    .replace("export function ", "function ")
    .replace("export const ", "const ")
)


def _layout(**opts):
    expr = "computeGridLayout(%s)" % json.dumps(opts)
    code = _JS_SRC + f"\nconsole.log(JSON.stringify({expr}));"
    result = run_node(code, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return json.loads(result.stdout.strip())


NAME, VAL, FILE, CHECK = 220, 124, 96, 34


class TestComputeGridLayout(unittest.TestCase):
    def test_normal_mode_has_no_file_or_checkbox(self):
        lay = _layout(vehicleCount=3)
        self.assertEqual(lay["fileWidth"], 0)
        self.assertEqual(lay["checkboxWidth"], 0)
        self.assertEqual(lay["totalWidth"], NAME + 3 * VAL)  # 592

    def test_compare_mode_adds_file_column_only(self):
        lay = _layout(vehicleCount=3, mode="compare")
        self.assertEqual(lay["fileWidth"], FILE)
        self.assertEqual(lay["checkboxWidth"], 0)
        self.assertEqual(lay["totalWidth"], NAME + FILE + 3 * VAL)  # 688

    def test_compare_selectable_adds_checkbox_and_file(self):
        lay = _layout(vehicleCount=3, mode="compare", selectable=True)
        self.assertEqual(lay["checkboxWidth"], CHECK)
        self.assertEqual(lay["fileWidth"], FILE)
        self.assertEqual(lay["totalWidth"], CHECK + NAME + FILE + 3 * VAL)  # 722

    def test_harmonize_selectable_has_checkbox_no_file(self):
        lay = _layout(vehicleCount=3, mode="harmonize", selectable=True)
        self.assertEqual(lay["checkboxWidth"], CHECK)
        self.assertEqual(lay["fileWidth"], 0)  # harmonize has no File column
        self.assertEqual(lay["totalWidth"], CHECK + NAME + 3 * VAL)  # 626

    def test_selectable_without_special_mode_has_no_checkbox(self):
        # checkbox only exists in compare/harmonize modes.
        lay = _layout(vehicleCount=2, selectable=True)
        self.assertEqual(lay["checkboxWidth"], 0)
        self.assertEqual(lay["fileWidth"], 0)
        self.assertEqual(lay["totalWidth"], NAME + 2 * VAL)

    def test_zero_vehicles(self):
        lay = _layout(vehicleCount=0, mode="compare")
        self.assertEqual(lay["totalWidth"], NAME + FILE)
        self.assertEqual(lay["vehicleCount"], 0)

    def test_negative_or_nan_vehicle_count_clamped(self):
        self.assertEqual(_layout(vehicleCount=-5)["vehicleCount"], 0)
        self.assertEqual(_layout(vehicleCount="x")["vehicleCount"], 0)

    def test_dims_passthrough(self):
        lay = _layout(vehicleCount=1)
        self.assertEqual(lay["rowHeight"], 28)
        self.assertEqual(lay["headerHeight"], 32)
        self.assertEqual(lay["nameWidth"], NAME)
        self.assertEqual(lay["valWidth"], VAL)


if __name__ == "__main__":
    unittest.main()
