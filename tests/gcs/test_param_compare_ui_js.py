"""Source-level wiring guards for the Compare-parameters UI (Step 4).

The React tab/grid aren't unit-tested via Node, so these lock the
behaviour-critical wiring the Codex gate-07 review called out:
    - compare mode never stages drafts (load-into-compare is separate),
    - the grid is read-only in compare mode and shows current values,
    - the sidebar/row universe is the compare file's params,
    - the prefix resets on enter/exit, and parse errors surface in the banner.
The diff math itself is covered by test_param_compare_js.py / _grid_layout_js.py.
"""
import os
import unittest


def _read(*parts):
    return open(os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", *parts)),
                encoding="utf-8").read()


_SETTINGS = ("src", "gcs", "frontend", "src", "components", "settings")
_TAB = _read(*_SETTINGS, "ParametersTab.jsx")
_GRID = _read(*_SETTINGS, "VirtualParamGrid.jsx")
_BAR = _read(*_SETTINGS, "ParamCompareBar.jsx")
_CELLS = _read(*_SETTINGS, "ParamCompareCells.jsx")


class TestParametersTabWiring(unittest.TestCase):
    def test_uses_compare_model_and_bar(self):
        self.assertIn("computeParamCompare", _TAB)
        self.assertIn("ParamCompareBar", _TAB)
        self.assertIn("compareMode = !!compareFile", _TAB)

    def test_loading_compare_file_does_not_stage_drafts(self):
        # The compare-file handler region must not call setVehicleDraft — loading
        # a compare file is separate from the draft/write path.
        start = _TAB.index("onCompareFileSelected")
        end = _TAB.index("onRefresh")
        region = _TAB[start:end]
        self.assertIn("setCompareFile(", region)
        self.assertNotIn("setVehicleDraft", region)

    def test_prefix_reset_on_enter_and_exit(self):
        start = _TAB.index("onCompareFileSelected")
        end = _TAB.index("onRefresh")
        region = _TAB[start:end]
        # The compare load + exit handlers reset the prefix (sync enter/exit,
        # which live in the same region, reset it too).
        self.assertGreaterEqual(region.count("setSelectedPrefix('(all)')"), 2)

    def test_sidebar_and_rows_use_compare_order(self):
        self.assertIn("sidebarSource", _TAB)
        self.assertIn("compareModel?.order", _TAB)

    def test_grid_is_readonly_in_compare_mode(self):
        self.assertIn("editable={!paramsRefreshing && !compareMode && !harmonizeMode}", _TAB)
        self.assertIn("mode={harmonizeMode ? 'harmonize' : compareMode ? 'compare' : 'normal'}", _TAB)
        self.assertIn("compareRowsByName={compareRowsByName}", _TAB)

    def test_write_changed_hidden_in_special_modes(self):
        self.assertIn("{!compareMode && !harmonizeMode && (", _TAB)

    def test_banner_surfaces_parse_errors(self):
        self.assertIn("parseErrors={compareFile.parseErrors}", _TAB)


class TestGridCompareWiring(unittest.TestCase):
    def test_uses_shared_layout_helper(self):
        self.assertIn("computeGridLayout", _GRID)
        # The scattered constants/inline totalWidth are gone.
        self.assertNotIn("const NAME_COL_WIDTH = 220", _GRID)
        self.assertNotIn("NAME_COL_WIDTH + vehicles.length * VAL_COL_WIDTH", _GRID)

    def test_renders_compare_columns(self):
        self.assertIn("FileValueCell", _GRID)
        self.assertIn("CompareNameTags", _GRID)
        self.assertIn("CompareValueCell", _GRID)
        # Compare mode renders the read-only cell, not the editor.
        self.assertIn("if (compareMode) {", _GRID)


class TestSelectionAndApplyWiring(unittest.TestCase):
    def test_tab_wires_selection_and_apply(self):
        self.assertIn("collectCompareChanges", _TAB)
        self.assertIn("defaultCompareSelection", _TAB)
        self.assertIn("reconcileCompareSelection", _TAB)
        self.assertIn("performApply", _TAB)
        self.assertIn("selectable={compareMode || harmonizeMode}", _TAB)
        self.assertIn("onToggleRow={harmonizeMode ? toggleHarmonizeRow : toggleCompareRow}", _TAB)
        self.assertIn("compareCellErrorsByVehicle", _TAB)

    def test_nothing_preselected_on_reset(self):
        # Operator feedback: rows must never be pre-ticked. On a reset trigger
        # (new file / skip toggle / a vehicle finishing load) the selection
        # clears to empty; "Select all" (scopedDefaultNames) is the only way to
        # bulk-select the safe/recommended subset, and it's an explicit click.
        start = _TAB.index("if (isReset) {")
        end = _TAB.index("} else {", start)
        region = _TAB[start:end]
        self.assertIn("setSelectedNames(new Set());", region)
        self.assertNotIn("defaultCompareSelection(compareModel.rows)", region)

    def test_apply_uses_numeric_sysids_not_object_keys(self):
        start = _TAB.index("const performApply")
        end = _TAB.index("// Long-press gate")
        region = _TAB[start:end]
        self.assertIn("writeChangedToVehicles(sysIds, { changesByVehicle })", region)
        # The write/arm-token ids must come from vehicle state, not the
        # string-keyed changes map. (An Object.keys length check is fine.)
        self.assertNotIn("writeChangedToVehicles(Object.keys", region)
        self.assertNotIn("of Object.keys(changesByVehicle)", region)

    def test_longpress_cancelled_on_mode_change(self):
        # Cancel effect depends on compareMode (plus selection/model for the
        # stale-closure guard); harmonize state is included too.
        self.assertIn("cancelLongPress(); }, [", _TAB)
        self.assertIn("compareMode, selectedNames, compareModel", _TAB)

    def test_grid_renders_checkbox_column(self):
        self.assertIn("masterRef", _GRID)
        self.assertIn("indeterminate = masterIndeterminate", _GRID)
        self.assertIn("left: CHECKBOX_COL_WIDTH", _GRID)
        self.assertIn("onToggleRow", _GRID)

    def test_compare_cell_renders_apply_error(self):
        self.assertIn("error = null", _CELLS)
        self.assertIn("colors.error", _CELLS)


class TestCompareCellsReadOnly(unittest.TestCase):
    def test_compare_value_cell_ignores_drafts(self):
        # The compare value cell renders the record's current value via
        # formatValueForDisplay; it has no draft/onChange machinery.
        self.assertIn("formatValueForDisplay(rec)", _CELLS)
        self.assertNotIn("onChange", _CELLS)
        self.assertNotIn("setVehicleDraft", _CELLS)

    def test_bar_has_differs_toggle_only(self):
        # Focus moved to the sidebar groups, so the bar no longer carries the
        # broad "only actionable" hide toggle — just "differs only" + exit.
        self.assertIn("differs_only", _BAR)
        self.assertNotIn("cmp_only_actionable", _BAR)
        self.assertNotIn("onToggleSkip", _BAR)
        self.assertIn("onExit", _BAR)


class TestCompareGroupSidebar(unittest.TestCase):
    def test_tab_uses_priority_group_sidebar(self):
        # Compare mode replaces the prefix tree with functional priority groups
        # (Option A) driven by summarizeCompareGroups + a selectedGroup focus.
        self.assertIn("summarizeCompareGroups", _TAB)
        self.assertIn("selectedGroup", _TAB)
        self.assertIn("setSelectedGroup", _TAB)
        self.assertIn("compareGroups", _TAB)

    def test_tab_filters_grid_by_selected_group(self):
        # Selecting a group filters the grid; 'all' shows every group EXCEPT the
        # view-only read-only bucket; other rows bucket via groupBucketForRow.
        self.assertIn("selectedGroup === 'all'", _TAB)
        self.assertIn("groupBucketForRow(r) === selectedGroup", _TAB)
        self.assertIn("!compareRowsByName[n]?.readOnly", _TAB)

    def test_tab_no_longer_hides_non_actionable_rows(self):
        # The blunt hide filter is gone — grouping deprioritizes instead of hiding.
        self.assertNotIn("!r.safetyTag && r.status !== 'file-only'", _TAB)

    def test_select_all_scoped_to_focused_group(self):
        # Master "select all" only touches the focused group's default rows.
        self.assertIn("scopedDefaultNames", _TAB)
        self.assertIn("groupBucketForRow(compareRowsByName?.[n]) === selectedGroup", _TAB)


if __name__ == "__main__":
    unittest.main()
