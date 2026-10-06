// Pure compare model for the "Compare parameters" feature. No React, no fetch.
//
// Turns parsed .param rows + per-UAV snapshots into a per-parameter diff that
// the grid renders and "apply" consumes. The row universe is the FILE's params
// (the actionable set): applying a file can neither add nor remove a vehicle's
// own params, so vehicle-only params are surfaced as a count, not rows.
//
// Diff fidelity: a row is "differs" iff the value the autopilot would STORE for
// the file value (coerceValueForRecord — truncate+clamp ints) is not equal to
// the current value (valuesEqual). This is exactly what selectChangedNames /
// writeChangedToVehicles use, so a row shown as differing is precisely what
// apply submits, and an int file value that coerces equal to current shows as
// "same" (e.g. file 1.9 vs current int 1).
//
// Tested via tests/gcs/test_param_compare_js.py (Node subprocess).

import { coerceValueForRecord, valuesEqual } from './fullParams';
import { matchesBusPortKeep, matchesIdentitySkip } from './paramSkipList';

// 'identity-cal-skip' → per-board identity/calibration (skip by default).
// 'bus-port-keep'     → wiring/port/protocol config (kept). Keep wins on overlap.
// null                → ordinary tune/config param.
function safetyTagFor(name) {
  if (matchesBusPortKeep(name)) return 'bus-port-keep';
  if (matchesIdentitySkip(name)) return 'identity-cal-skip';
  return null;
}

/**
 * Build the compare model.
 *
 *   fileRows           ordered [{name, value}] from parseParamFile
 *   snapshotsByVehicle { [sysId]: normalisedSnapshot } (paramsByName + nameOrder)
 *   sysIds             connected vehicle ids, column order
 *
 * Default selection (`row.defaultSelected`, consumed by defaultCompareSelection):
 * plain tuning/config diffs and bus/port wiring are always candidates; per-board
 * identity/calibration is NEVER a candidate, unconditionally (copying it across
 * airframes corrupts the target) — only an explicit per-row tick applies it.
 * There is no "show all" toggle here: every row is always present in the model
 * and grid, the priority-group sidebar just changes what's in view (Codex gate
 * 15 — this used to also gate on a skipUavSpecific flag shared with Sync UAVs'
 * own toggle, which let that unrelated mode's state leak into this one).
 */
export function computeParamCompare({
  fileRows, snapshotsByVehicle, sysIds,
} = {}) {
  const ids = Array.isArray(sysIds) ? sysIds : [];
  const snaps = snapshotsByVehicle || {};
  const isLoaded = (sid) => !!(snaps[sid] && snaps[sid].paramsByName);
  const allLoaded = ids.length > 0 && ids.every(isLoaded);

  // 1. Dedup file rows: first-occurrence order, last value wins. Non-finite
  // values are dropped (parseParamFile already rejects them, but keep the pure
  // helper honest if called directly) and counted, never rendered as a row.
  const order = [];
  const fileValueByName = new Map();
  const duplicateNames = [];
  let invalid = 0;
  for (const r of (Array.isArray(fileRows) ? fileRows : [])) {
    if (!r || typeof r.name !== 'string') continue;
    const v = Number(r.value);
    if (!Number.isFinite(v)) { invalid += 1; continue; }
    if (fileValueByName.has(r.name)) {
      if (!duplicateNames.includes(r.name)) duplicateNames.push(r.name);
    } else {
      order.push(r.name);
    }
    fileValueByName.set(r.name, v);
  }

  // 2. Per-row compare state.
  const rows = [];
  const counts = {
    total: 0, differ: 0, same: 0, fileOnly: 0, pending: 0,
    vehicleOnly: 0, readOnly: 0, skippedDiffer: 0, busPortDiffer: 0,
    defaultSelectedRows: 0, defaultWriteCells: 0,
    duplicates: duplicateNames.length, invalid,
  };
  for (const name of order) {
    const fileValue = fileValueByName.get(name);
    const safetyTag = safetyTagFor(name);
    const isUavSpecific = safetyTag === 'identity-cal-skip';
    const perVehicle = {};
    const writableSysIds = [];
    let anyLoaded = false;
    let anyPresent = false;
    let readOnlyParam = false;
    for (const sid of ids) {
      const loaded = isLoaded(sid);
      if (loaded) anyLoaded = true;
      const rec = loaded ? snaps[sid].paramsByName[name] : undefined;
      const present = !!rec;
      let value = null;
      let fileStored = null;
      let differs = false;
      let coerced = false;
      if (present) {
        anyPresent = true;
        if (rec.read_only) readOnlyParam = true;
        value = rec.value;
        fileStored = coerceValueForRecord(rec, fileValue);
        if (fileStored !== null) {
          differs = !valuesEqual(fileStored, rec.value, rec);
          coerced = Number(fileValue) !== fileStored;
        }
        if (differs) writableSysIds.push(sid);
      }
      perVehicle[sid] = { loaded, present, value, fileStored, differs, coerced };
    }

    // Read-only params (firmware-maintained: STAT_* counters, *_DEVID, MIS_TOTAL,
    // *_GND_PRESS, ...) can never be written — the firmware ignores/overwrites the
    // value. They're kept as a VIEW-ONLY row (readOnly:true) so the operator can
    // still inspect the per-UAV values, but never written: no diff highlight, no
    // writable cells, never selectable or default-selected, and bucketed into the
    // dedicated 'read-only' sidebar group (excluded from every other group / All).
    if (readOnlyParam) {
      counts.readOnly += 1;
      for (const sid of ids) { if (perVehicle[sid]) perVehicle[sid].differs = false; }
      rows.push({
        name, fileValue, safetyTag: null, isUavSpecific: false,
        perVehicle, anyLoaded, anyPresent, anyDiffers: false, status: 'read-only',
        writableSysIds: [], actionable: false, defaultSelected: false, readOnly: true,
      });
      continue;
    }

    const anyDiffers = writableSysIds.length > 0;
    let status;
    if (!anyLoaded) status = 'pending';
    else if (anyDiffers) status = 'differ';
    else if (anyPresent) status = 'same';
    else status = allLoaded ? 'file-only' : 'pending';

    // A row is "actionable" only when it's a plain tuning/config diff present on
    // the vehicle: not per-board identity/cal, not bus/port wiring, not file-only.
    const actionable = anyDiffers && safetyTag === null;
    // Identity/calibration is NEVER auto-selected (copying per-board cal corrupts
    // the target) — it applies only by an explicit per-row tick, unconditionally,
    // independent of skipUavSpecific. Bus/port is safe shared config, so it's
    // always a default-selection candidate too: the compare view no longer has
    // its own "show all" toggle (superseded by the priority-group sidebar, where
    // navigating into a group is itself the operator's explicit choice to review
    // it), so gating it on skipUavSpecific would leak Sync UAVs' toggle state
    // into compare's selection (Codex gate 15, nice-to-have #1). Plain diffs are
    // always selected.
    const defaultSelected = actionable || safetyTag === 'bus-port-keep';

    rows.push({
      name, fileValue, safetyTag, isUavSpecific,
      perVehicle, anyLoaded, anyPresent, anyDiffers, status,
      writableSysIds, actionable, defaultSelected,
    });

    counts.total += 1;
    if (status === 'differ') counts.differ += 1;
    else if (status === 'same') counts.same += 1;
    else if (status === 'file-only') counts.fileOnly += 1;
    else counts.pending += 1;
    if (anyDiffers && isUavSpecific) counts.skippedDiffer += 1;
    if (anyDiffers && safetyTag === 'bus-port-keep') counts.busPortDiffer += 1;
    if (defaultSelected) {
      counts.defaultSelectedRows += 1;
      counts.defaultWriteCells += writableSysIds.length;
    }
  }

  // 3. vehicle-only: unique names on any loaded vehicle but not in the file.
  const seen = new Set();
  for (const sid of ids) {
    const snap = snaps[sid];
    if (!snap || !Array.isArray(snap.nameOrder)) continue;
    for (const n of snap.nameOrder) {
      if (fileValueByName.has(n) || seen.has(n)) continue;
      seen.add(n);
      counts.vehicleOnly += 1;
    }
  }

  return { rows, order, duplicateNames, counts };
}

/**
 * Names selected by default: differing rows not excluded by the skip toggle
 * (row.defaultSelected). Returned as an array; the caller wraps it in a Set.
 */
export function defaultCompareSelection(rows) {
  const out = [];
  for (const row of (Array.isArray(rows) ? rows : [])) {
    if (row && row.defaultSelected) out.push(row.name);
  }
  return out;
}

/**
 * Reconcile a prior selection against a recomputed model: keep selected names
 * that are still writable (anyDiffers), drop the rest. Preserves a manually
 * selected identity/calib row as long as it still differs, and drops rows that
 * a write just made equal (so applied rows fall out of the selection). Used on
 * snapshot/post-write recompute — NOT on file-load/skip-toggle, which reset to
 * defaults instead.
 */
export function reconcileCompareSelection(selectedNames, rows) {
  const writable = new Set();
  for (const row of (Array.isArray(rows) ? rows : [])) {
    if (row && row.anyDiffers) writable.add(row.name);
  }
  const sel = selectedNames instanceof Set ? selectedNames : new Set(selectedNames || []);
  const out = [];
  for (const name of sel) if (writable.has(name)) out.push(name);
  return out;
}

/**
 * Collect the changes to write for the operator's selected rows. Returns
 * `{ [sysId]: [{name, value}] }` containing ONLY cells that are present and
 * actually differ on that vehicle; the written value is that vehicle's
 * coerced fileStored. Selecting a row that is same/absent on some UAV simply
 * contributes nothing for those UAVs.
 *
 * NOTE: object keys are stringified sys_ids. A caller that iterates
 * `Object.keys(...)` must `Number()` them before matching numeric vehicle ids
 * or requesting armed tokens.
 */
export function collectCompareChanges(rows, selectedNames) {
  const out = {};
  if (!Array.isArray(rows)) return out;
  const selected = selectedNames instanceof Set
    ? selectedNames
    : new Set(selectedNames || []);
  for (const row of rows) {
    // Read-only rows are view-only — never written, even if a stray selection
    // names one (defense-in-depth; writableSysIds is already empty for them).
    if (!row || row.readOnly || !selected.has(row.name)) continue;
    for (const sid of (row.writableSysIds || [])) {
      const cell = row.perVehicle ? row.perVehicle[sid] : null;
      if (!cell || !cell.differs || cell.fileStored === null) continue;
      if (!out[sid]) out[sid] = [];
      out[sid].push({ name: row.name, value: cell.fileStored });
    }
  }
  return out;
}
