"""Tests for the CONFIRMATION_PARAM_SECTION export in paramSections.js.

Locks in:
    - shape: title plus four fields keyed nav_auto_cm, nav_cwt, nav_cm_fl,
      nav_cgt (in that render order).
    - boolean option values for the two select fields (the existing
      `select` rendering preserves o.value via String(o.value) lookup, so
      a regression to numeric/string-coerced values would break the AAS
      bool roundtrip end-to-end).
    - rowVisibleWhen predicates on nav_cwt and nav_cm_fl: the manual-only
      rows render when AT LEAST ONE selected vehicle has nav_auto_cm ===
      false, and collapse only when every selected vehicle is in
      automatic mode. nav_cgt has no visibility predicate (always shown).
"""
import json
import os
from tests.gcs.js_runner import run_node
import unittest


_UTIL_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components", "settings",
    "paramSections.js",
))

_raw = open(_UTIL_PATH, encoding="utf-8").read()
_JS_SRC = (
    _raw
    .replace("export function ", "function ")
    .replace("export const ", "const ")
)


def _run_js(script):
    code = _JS_SRC + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


def _section():
    """Read the CONFIRMATION_PARAM_SECTION shape (functions stripped)."""
    return json.loads(_run_js(
        # JSON.stringify drops functions, which is exactly what we want
        # for the shape assertion.
        "console.log(JSON.stringify(CONFIRMATION_PARAM_SECTION));"
    ))


def _row_visible_when(field_key, snapshots_json):
    """Evaluate the rowVisibleWhen predicate of a named field."""
    return json.loads(_run_js(
        f"const f = CONFIRMATION_PARAM_SECTION.fields.find((x) => x.key === {json.dumps(field_key)});"
        f"console.log(JSON.stringify({{v: f.rowVisibleWhen({snapshots_json})}}));"
    ))["v"]


def _cell_disabled_when(field_key, vehicle_draft_json):
    """Evaluate the cellDisabledWhen predicate of a named field for one vehicle."""
    return json.loads(_run_js(
        f"const f = CONFIRMATION_PARAM_SECTION.fields.find((x) => x.key === {json.dumps(field_key)});"
        f"console.log(JSON.stringify({{v: f.cellDisabledWhen({vehicle_draft_json})}}));"
    ))["v"]


class TestSectionShape(unittest.TestCase):
    def test_section_has_title_and_four_fields_in_render_order(self):
        s = _section()
        self.assertEqual(s["title"], "Confirmation")
        self.assertEqual(
            [f["key"] for f in s["fields"]],
            ["nav_auto_cm", "nav_cwt", "nav_cm_fl", "nav_cgt"],
        )

    def test_nav_auto_cm_select_with_bool_values(self):
        s = _section()
        f = next(x for x in s["fields"] if x["key"] == "nav_auto_cm")
        self.assertEqual(f["type"], "select")
        self.assertEqual([o["value"] for o in f["options"]], [True, False])
        self.assertEqual([o["label"] for o in f["options"]], ["Automatic", "Manual"])

    def test_nav_cm_fl_select_with_bool_values(self):
        s = _section()
        f = next(x for x in s["fields"] if x["key"] == "nav_cm_fl")
        self.assertEqual(f["type"], "select")
        self.assertEqual([o["value"] for o in f["options"]], [True, False])
        self.assertEqual([o["label"] for o in f["options"]], ["Approve", "Deny"])

    def test_nav_cwt_is_number_field(self):
        s = _section()
        f = next(x for x in s["fields"] if x["key"] == "nav_cwt")
        self.assertEqual(f["type"], "number")
        self.assertEqual(f["step"], 1)
        self.assertEqual(f["min"], 0)

    def test_nav_cgt_is_unconditional_number_field(self):
        # nav_cgt (image-quality timeout) is read regardless of mode in
        # NavPy navigation, so it must NOT carry rowVisibleWhen.
        s = _section()
        f = next(x for x in s["fields"] if x["key"] == "nav_cgt")
        self.assertEqual(f["type"], "number")
        self.assertEqual(f["step"], 1)
        self.assertEqual(f["min"], 0)
        self.assertEqual(f["label"], "Quality image timeout (s)")
        self.assertNotIn("rowVisibleWhen", f)


class TestRowVisibleWhen(unittest.TestCase):
    def test_nav_auto_cm_row_has_no_visibility_predicate(self):
        s = _section()
        f = next(x for x in s["fields"] if x["key"] == "nav_auto_cm")
        # JSON.stringify drops functions; absence in JSON output IS the assertion.
        self.assertNotIn("rowVisibleWhen", f)

    def test_nav_cwt_row_visible_when_any_vehicle_manual(self):
        # Single-vehicle scenarios.
        self.assertTrue(_row_visible_when("nav_cwt", '{"1": {"nav_auto_cm": false}}'))
        self.assertFalse(_row_visible_when("nav_cwt", '{"1": {"nav_auto_cm": true}}'))
        # Multi-vehicle: row visible if at least one is manual.
        self.assertTrue(_row_visible_when(
            "nav_cwt", '{"1": {"nav_auto_cm": true}, "2": {"nav_auto_cm": false}}'))
        # All automatic -> hidden.
        self.assertFalse(_row_visible_when(
            "nav_cwt", '{"1": {"nav_auto_cm": true}, "2": {"nav_auto_cm": true}}'))
        # No vehicles selected -> hidden (no manual to show).
        self.assertFalse(_row_visible_when("nav_cwt", '{}'))

    def test_nav_cm_fl_row_visible_when_any_vehicle_manual(self):
        self.assertTrue(_row_visible_when("nav_cm_fl", '{"1": {"nav_auto_cm": false}}'))
        self.assertFalse(_row_visible_when("nav_cm_fl", '{"1": {"nav_auto_cm": true}}'))
        self.assertTrue(_row_visible_when(
            "nav_cm_fl", '{"1": {"nav_auto_cm": true}, "2": {"nav_auto_cm": false}}'))
        self.assertFalse(_row_visible_when(
            "nav_cm_fl", '{"1": {"nav_auto_cm": true}, "2": {"nav_auto_cm": true}}'))

    def test_row_visibility_strict_equal_false(self):
        # Strict === false; "false", 0, null, undefined per-vehicle entries
        # must NOT count as manual-mode triggers.
        self.assertFalse(_row_visible_when("nav_cwt", '{"1": {"nav_auto_cm": "false"}}'))
        self.assertFalse(_row_visible_when("nav_cwt", '{"1": {"nav_auto_cm": 0}}'))
        self.assertFalse(_row_visible_when("nav_cwt", '{"1": {"nav_auto_cm": null}}'))
        # Missing key in a vehicle's draft -> not manual.
        self.assertFalse(_row_visible_when("nav_cwt", '{"1": {}}'))
        # Null draft for a vehicle -> not manual.
        self.assertFalse(_row_visible_when("nav_cwt", '{"1": null}'))


class TestCellDisabledWhen(unittest.TestCase):
    """Within a visible manual-only row, cells for vehicles that are
    themselves in automatic mode are disabled so the operator does not
    get the impression that nav_cwt / nav_cm_fl can be edited per-cell
    for an automatic-mode vehicle."""

    def test_nav_cwt_cell_enabled_only_when_vehicle_manual(self):
        self.assertFalse(_cell_disabled_when("nav_cwt", '{"nav_auto_cm": false}'))
        self.assertTrue(_cell_disabled_when("nav_cwt", '{"nav_auto_cm": true}'))
        self.assertTrue(_cell_disabled_when("nav_cwt", '{}'))
        self.assertTrue(_cell_disabled_when("nav_cwt", "null"))

    def test_nav_cm_fl_cell_enabled_only_when_vehicle_manual(self):
        self.assertFalse(_cell_disabled_when("nav_cm_fl", '{"nav_auto_cm": false}'))
        self.assertTrue(_cell_disabled_when("nav_cm_fl", '{"nav_auto_cm": true}'))
        self.assertTrue(_cell_disabled_when("nav_cm_fl", '{}'))
        self.assertTrue(_cell_disabled_when("nav_cm_fl", "null"))

    def test_nav_auto_cm_has_no_cell_disabled_predicate(self):
        s = _section()
        f = next(x for x in s["fields"] if x["key"] == "nav_auto_cm")
        self.assertNotIn("cellDisabledWhen", f)

    def test_nav_cgt_has_no_cell_disabled_predicate(self):
        s = _section()
        f = next(x for x in s["fields"] if x["key"] == "nav_cgt")
        self.assertNotIn("cellDisabledWhen", f)


if __name__ == "__main__":
    unittest.main()
