"""Source-level wiring guards for the Sync-UAVs (harmonize) UI.

The React tab/grid aren't Node-unit-testable, so these lock the behaviour-critical
wiring: harmonize never stages drafts, mutual exclusion with compare, the scoped
apply passes numeric sysIds and gates on allLoaded, and the grid renders the
clickable harmonize cells. The model math is covered by test_param_harmonize_js.py.
"""
import os
import unittest


def _read(*parts):
    return open(os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", *parts)),
                encoding="utf-8").read()


_S = ("src", "gcs", "frontend", "src", "components", "settings")
_TAB = _read(*_S, "ParametersTab.jsx")
_GRID = _read(*_S, "VirtualParamGrid.jsx")
_HZBAR = _read(*_S, "ParamHarmonizeBar.jsx")
_HZCELLS = _read(*_S, "ParamHarmonizeCells.jsx")
_CMPCELLS = _read(*_S, "ParamCompareCells.jsx")
_LAYOUT = _read("src", "gcs", "frontend", "src", "utils", "paramGridLayout.js")


class TestHarmonizeTabWiring(unittest.TestCase):
    def test_uses_harmonize_model_and_bar(self):
        self.assertIn("computeFleetHarmonize", _TAB)
        self.assertIn("ParamHarmonizeBar", _TAB)
        self.assertIn("collectHarmonizeChanges", _TAB)
        self.assertIn("harmonizeMode", _TAB)

    def test_mutually_exclusive_with_compare(self):
        # Entering sync clears the compare file; loading a compare file exits sync.
        enter = _TAB.index("const onEnterHarmonize")
        region = _TAB[enter:enter + 300]
        self.assertIn("setCompareFile(null)", region)
        self.assertIn("setHarmonizeMode(true)", region)
        sel = _TAB.index("const onCompareFileSelected")
        self.assertIn("setHarmonizeMode(false)", _TAB[sel:sel + 500])

    def test_apply_scoped_write_numeric_and_gated(self):
        start = _TAB.index("const performHarmonizeApply")
        end = _TAB.index("// Long-press gate")
        region = _TAB[start:end]
        self.assertIn("harmonizeModel.allLoaded", region)          # gated on full fleet
        self.assertIn("writeChangedToVehicles(sysIds, { changesByVehicle })", region)
        self.assertNotIn("of Object.keys(changesByVehicle)", region)
        self.assertNotIn("setVehicleDraft", region)                # never stages drafts

    def test_grid_gets_harmonize_props(self):
        self.assertIn("harmonizeRowsByName={harmonizeRowsByName}", _TAB)
        self.assertIn("winnersByName={harmonizeWinners}", _TAB)
        self.assertIn("onPickWinner={onPickWinner}", _TAB)

    def test_pick_winner_stores_base_sysid(self):
        # A pick records the clicked UAV alongside the value so the grid can badge
        # the base cell (not every cell that shares the winning value).
        start = _TAB.index("const onPickWinner")
        region = _TAB[start:start + 400]
        self.assertIn("[name]: { value, sysId }", region)

    def test_nothing_preselected_on_reset(self):
        # Operator feedback (same fix as compare): rows must never be pre-ticked.
        # On a reset trigger the selection clears to empty; "Select all"
        # (hzDefaultSelection) is the only way to bulk-select the safe subset.
        start = _TAB.index("Nothing is pre-selected")
        end = _TAB.index("} else {", start)
        region = _TAB[start:end]
        self.assertIn("setHarmonizeSelected(new Set());", region)
        self.assertNotIn("defaultHarmonizeSelection(harmonizeModel.rows)", region)

    def test_deselecting_a_row_forgets_its_manual_winner(self):
        # Codex gate 15 (blocking): a manually-picked winner used to survive
        # deselection, so a later reselect (checkbox or "select all") could
        # silently reapply a value the operator no longer sees chosen. Scenario:
        # pick minority value -> row selects; click the same (now winner) cell
        # again -> row deselects; toggleHarmonizeRow must then drop that row's
        # entry from harmonizeWinners so a future reselect starts fresh at the
        # modal/default winner, not the stale manual pick.
        start = _TAB.index("const toggleHarmonizeRow")
        end = _TAB.index("const hzDefaultSelection", start)
        region = _TAB[start:end]
        self.assertIn("const wasSelected = harmonizeSelected.has(name);", region)
        self.assertIn("if (wasSelected) {", region)
        self.assertIn("setHarmonizeWinners((prev) => {", region)
        self.assertIn("const { [name]: _dropped, ...rest } = prev;", region)

    def test_select_all_off_forgets_winners_for_every_cleared_row(self):
        # The same staleness bug applies to the master "select all" toggling
        # OFF (a mass deselect) — every row it clears must also lose its
        # manual winner, not just the single-row checkbox/click path above.
        start = _TAB.index("const onToggleAllHarmonize")
        end = _TAB.index("const harmonizeChanges", start)
        region = _TAB[start:end]
        self.assertIn("const cleared = harmonizeSelected;", region)
        self.assertIn("setHarmonizeSelected(new Set());", region)
        self.assertIn("setHarmonizeWinners((prev) => {", region)
        self.assertIn("for (const n of cleared)", region)

    def test_recompute_forgets_winner_for_rows_dropped_by_reconcile(self):
        # Codex gate-15 follow-up (PR #90, deferred non-blocking): a PLAIN
        # recompute (e.g. post-write/refresh, not an operator toggle) can also
        # drop a row out of the selection via reconcileHarmonizeSelection — that
        # row's manual pick must be forgotten too, so a LATER divergence of the
        # same param can't silently resurface a stale pick via effectiveWinner.
        # This is the third path; the two operator-driven ones are covered by
        # the tests above.
        start = _TAB.index("} else {", _TAB.index("Nothing is pre-selected"))
        end = _TAB.index("const onPickWinner", start)
        region = _TAB[start:end]
        self.assertIn("reconcileHarmonizeSelection(", region)
        self.assertIn("droppedHarmonizeWinnerNames(harmonizeSelected, reconciled)", region)
        self.assertIn("setHarmonizeWinners((prev) => {", region)
        self.assertIn("if (n in next) { delete next[n]; changed = true; }", region)


class TestHarmonizeGridWiring(unittest.TestCase):
    def test_grid_uses_mode_enum(self):
        self.assertIn("const harmonizeMode = mode === 'harmonize'", _GRID)
        self.assertIn("const compareMode = mode === 'compare'", _GRID)

    def test_grid_renders_harmonize_cell(self):
        self.assertIn("HarmonizeValueCell", _GRID)
        self.assertIn("harmonizeCellStates", _GRID)
        self.assertIn("if (harmonizeMode) {", _GRID)

    def test_grid_threads_base_sysid_and_click(self):
        # The grid resolves which cell is the base and threads the clicked UAV's
        # sys_id into the pick, so effectiveBaseSysId badges exactly that cell.
        self.assertIn("effectiveBaseSysId", _GRID)
        self.assertIn("isBase={harmonizeRowSelected && harmonizeBaseSid === v.sys_id}", _GRID)
        self.assertIn("onPickWinner(name, value, v.sys_id)", _GRID)

    def test_layout_supports_harmonize_mode(self):
        self.assertIn("isHarmonize", _LAYOUT)
        # File column is compare-only.
        self.assertIn("const fileWidth = isCompare ?", _LAYOUT)

    def test_sync_colors_gated_on_row_selection(self):
        # Operator feedback: winner/outlier colors (and the base badge) must only
        # render for a SELECTED row — an unselected row shows plain values.
        self.assertIn("harmonizeRowSelected", _GRID)
        self.assertIn(
            "state={harmonizeRowSelected && harmonizeStates ? harmonizeStates[v.sys_id] : null}",
            _GRID,
        )

    def test_click_current_winner_cell_unselects_row(self):
        # Clicking a cell picks + selects; clicking the cell that's ALREADY this
        # row's shown winner (only true once selected) unselects instead of
        # re-picking — "click again to unselect".
        start = _GRID.index("onPick={onPickWinner && !harmonizeRow?.readOnly ? (value) => {")
        end = _GRID.index("} : null}", start)
        region = _GRID[start:end]
        self.assertIn("const isCurrentWinnerCell = harmonizeBaseSid === v.sys_id;", region)
        self.assertIn("if (isCurrentWinnerCell && harmonizeRowSelected) {", region)
        self.assertIn("if (onToggleRow) onToggleRow(name);", region)
        self.assertIn("if (!harmonizeRowSelected && onToggleRow) onToggleRow(name);", region)

    def test_grid_passes_harmonize_mode_to_name_tags(self):
        # CompareNameTags needs to know it's rendering a harmonize row to show
        # the no-majority/type-mismatch/partial tags (see TestDisabledRowTags).
        self.assertIn("<CompareNameTags row={tagRow} harmonizeMode={harmonizeMode} />", _GRID)


class TestDisabledRowTags(unittest.TestCase):
    """A harmonize row's checkbox is disabled whenever there's no effective
    winner (no majority / type mismatch) — without a tag that reads as an
    unexplained 'read-only' row, identical in appearance to a genuinely
    unpickable one. These tags close that gap (operator report: 'RTL_ALT is
    readonly why?' — RTL_ALTITUDE simply has three distinct values, no majority,
    and nothing said so)."""

    def test_no_majority_tag_present(self):
        self.assertIn("row.defaultWinner === null", _CMPCELLS)
        self.assertIn("tag_no_majority", _CMPCELLS)

    def test_type_mismatch_tag_takes_precedence(self):
        # typeMismatch rows always have defaultWinner === null too (see
        # paramHarmonize.js); the type-mismatch tag must win so the operator
        # gets the more specific reason, not a redundant/misleading pair.
        start = _CMPCELLS.index("if (harmonizeMode) {")
        end = _CMPCELLS.index("if (tags.length === 0)", start)
        region = _CMPCELLS[start:end]
        self.assertIn("if (row.typeMismatch)", region)
        self.assertIn("} else if (row.defaultWinner === null)", region)
        self.assertIn("tag_type_mismatch", region)

    def test_partial_tag_independent_of_majority(self):
        # Partial can co-occur with either outcome, so it's a separate branch.
        self.assertIn("if (row.partial)", _CMPCELLS)
        self.assertIn("tag_partial", _CMPCELLS)

    def test_tags_only_apply_in_harmonize_mode(self):
        # Compare-mode rows never carry these fields; the branch must be gated
        # on harmonizeMode so a compare row can't accidentally match.
        gate_idx = _CMPCELLS.index("if (harmonizeMode) {")
        type_mismatch_idx = _CMPCELLS.index("row.typeMismatch")
        self.assertGreater(type_mismatch_idx, gate_idx)

    def test_tags_render_as_icons_not_text_pills(self):
        # Operator-caught bug: a row with TWO text-pill tags (identity/calib +
        # no-majority, e.g. SYSID_THISMAV / STAT_*) overran the 220px Name
        # column and squeezed the param name to 0 width. Fix: render each tag as
        # a compact icon (label + reason in the hover title / aria-label), so the
        # name always stays visible. Guard the icon rendering so a regression to
        # wide text pills is caught.
        self.assertIn("const TAG_ICON = {", _CMPCELLS)
        for icon_key in ("icon: 'lock'", "icon: 'plug'", "icon: 'file'",
                         "icon: 'warn'", "icon: 'neq'", "icon: 'half'"):
            self.assertIn(icon_key, _CMPCELLS)
        self.assertIn("aria-label={tag.label}", _CMPCELLS)
        self.assertIn("{TAG_ICON[tag.icon]}", _CMPCELLS)
        # The full label + reason still reaches the operator on hover.
        self.assertIn("title={`${tag.label}", _CMPCELLS)

    def test_name_span_truncates_before_tags(self):
        # Companion to the icon fix: the name span must be the flex item that
        # shrinks/truncates (flex 1 1 auto + minWidth 0), never the fixed-width
        # tag icons — otherwise the name could still be squeezed away.
        start = _GRID.index("title={name}")
        end = _GRID.index("<CompareNameTags", start)
        region = _GRID[start:end]
        self.assertIn("flex: '1 1 auto',", region)
        self.assertIn("minWidth: 0,", region)


class TestReadOnlyGroup(unittest.TestCase):
    """Read-only (view-only firmware) params get a dedicated "Read-only" sidebar
    bucket (operator: "let also user to see the readonly" — Option B). They render
    view-only: a lock tag, no pick, disabled checkbox, and are excluded from every
    other view. These guard the wiring across tab / grid / cells."""

    def test_tab_has_readonly_sidebar_item(self):
        # Harmonize prefix-tree gets a muted "Read-only" item toggling a sentinel.
        self.assertIn("grp_read_only", _TAB)
        self.assertIn("'(readonly)'", _TAB)
        self.assertIn("readOnlyCount", _TAB)

    def test_tab_excludes_readonly_from_prefix_tree(self):
        # sidebarSource (the harmonize prefix tree) filters read-only rows out so
        # they surface ONLY under their own bucket.
        self.assertIn("filter((r) => !r.readOnly)", _TAB)
        self.assertIn("readOnlyNames", _TAB)

    def test_tab_readonly_view_shows_only_readonly_names(self):
        start = _TAB.index("const visibleNames")
        end = _TAB.index("const staleByVehicle", start)
        region = _TAB[start:end]
        # The read-only view draws from readOnlyNames; the "All" compare view drops
        # read-only rows; other compare groups match via groupBucketForRow.
        self.assertIn("selectedPrefix === '(readonly)'", region)
        self.assertIn("filterCanonicalNames(readOnlyNames", region)
        self.assertIn("!compareRowsByName[n]?.readOnly", region)
        self.assertIn("groupBucketForRow(r) === selectedGroup", region)

    def test_grid_gates_pick_off_for_readonly_rows(self):
        # A read-only harmonize row is never pickable (no winner to choose).
        self.assertIn("onPickWinner && !harmonizeRow?.readOnly", _GRID)

    def test_grid_disables_checkbox_for_readonly_rows(self):
        # The checkbox reads inert for a read-only row too, even in the rare
        # writable->read-only refresh case where a stale winner would otherwise
        # keep it enabled (cosmetic; the write path is already guarded).
        self.assertIn("harmonizeRow.readOnly", _GRID)

    def test_name_tag_renders_readonly_lock_only(self):
        # CompareNameTags short-circuits read-only rows to a single lock badge —
        # no misleading no-majority/type-mismatch/partial tags.
        start = _CMPCELLS.index("export function CompareNameTags")
        end = _CMPCELLS.index("const tags = [];", start)
        region = _CMPCELLS[start:end]
        self.assertIn("if (row.readOnly) {", region)
        self.assertIn("tag_read_only", region)
        self.assertIn("TAG_ICON.lock", region)


class TestHarmonizeCells(unittest.TestCase):
    def test_harmonize_cell_is_pick_only_no_drafts(self):
        self.assertIn("formatValueForDisplay(rec)", _HZCELLS)
        self.assertIn("onPick", _HZCELLS)
        self.assertNotIn("onChange", _HZCELLS)
        self.assertNotIn("setVehicleDraft", _HZCELLS)

    def test_harmonize_cell_renders_base_badge(self):
        # The selected base cell is distinguished from other same-value winner
        # cells by an icon + its own tooltip, so a pick isn't visually ambiguous.
        self.assertIn("isBase", _HZCELLS)
        self.assertIn("hz_base_cell", _HZCELLS)
        self.assertIn("<svg", _HZCELLS)

    def test_bar_has_skip_toggle_and_exit(self):
        self.assertIn("skip_uav_specific", _HZBAR)
        self.assertIn("exit_sync", _HZBAR)
        self.assertIn("onExit", _HZBAR)


if __name__ == "__main__":
    unittest.main()
