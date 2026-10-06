// Pure model for the "Sync UAVs / Harmonize" feature. No React, no fetch.
//
// Finds parameters where the CONNECTED UAVs disagree, and lets the operator pick
// a winning value per row to push to the UAVs that differ (the outliers). Only
// PRESENT UAVs are compared and written — you can't write a param to a UAV that
// doesn't have it. The winner is coerced and compared PER TARGET (a UAV's ap_type
// may differ), reusing the write-path coercion (coerceValueForRecord + valuesEqual)
// so a row's highlighted outliers are exactly what the scoped writer submits.
//
// Identity/calibration params are shown but excluded from the default selection —
// they are supposed to differ per board (same skip-list as compare).
//
// Tested via tests/gcs/test_param_harmonize_js.py (Node subprocess).

import { coerceValueForRecord, valuesEqual } from './fullParams';
import { matchesBusPortKeep, matchesIdentitySkip } from './paramSkipList';

function safetyTagFor(name) {
  if (matchesBusPortKeep(name)) return 'bus-port-keep';
  if (matchesIdentitySkip(name)) return 'identity-cal-skip';
  return null;
}

// Group present UAV cells by stored-value equality. Each cell is {sid, rec}.
// Returns [{ value, apType, sysIds:[...], count }] sorted by count desc, then by
// first-seen order (stable) so ties are deterministic.
function groupPresentValues(cells) {
  const groups = [];
  for (const { sid, rec } of cells) {
    const g = groups.find((gr) => valuesEqual(gr.value, rec.value, { ap_type: gr.apType }));
    if (g) { g.sysIds.push(sid); g.count += 1; } else {
      groups.push({ value: rec.value, apType: rec.ap_type, sysIds: [sid], count: 1 });
    }
  }
  // Stable sort by count desc (Array.sort is stable in modern engines/Node).
  groups.sort((a, b) => b.count - a.count);
  return groups;
}

/**
 * Build the harmonize model over the connected UAVs.
 *
 *   snapshotsByVehicle { [sysId]: normalisedSnapshot }
 *   sysIds             connected vehicle ids (column order)
 *   skipUavSpecific    when true (default), identity/cal rows are not selected by
 *                      default (still shown, still individually selectable)
 *
 * Returns { rows, counts }. A row is included only when >= 2 present UAVs disagree.
 */
export function computeFleetHarmonize({
  snapshotsByVehicle, sysIds, skipUavSpecific = true,
} = {}) {
  const ids = Array.isArray(sysIds) ? sysIds : [];
  const snaps = snapshotsByVehicle || {};
  const isLoaded = (sid) => !!(snaps[sid] && snaps[sid].paramsByName);
  const loadedIds = ids.filter(isLoaded);
  // Only auto-select when the WHOLE connected fleet is loaded — otherwise the
  // divergence is computed over a subset and could push a value that the still-
  // loading UAV already agrees with (or would contradict).
  const allLoaded = ids.length > 0 && loadedIds.length === ids.length;

  // Row universe: union of param names across LOADED snapshots (stable order).
  const order = [];
  const seen = new Set();
  for (const sid of loadedIds) {
    for (const n of (snaps[sid].nameOrder || [])) {
      if (!seen.has(n)) { seen.add(n); order.push(n); }
    }
  }

  const rows = [];
  const counts = {
    divergent: 0, skippedDivergent: 0, noMajority: 0, typeMismatch: 0, partial: 0,
    readOnly: 0, defaultSelectedRows: 0,
  };

  for (const name of order) {
    const perVehicle = {};
    const presentCells = [];
    for (const sid of ids) {
      const loaded = isLoaded(sid);
      const rec = loaded ? snaps[sid].paramsByName[name] : undefined;
      const present = !!rec;
      perVehicle[sid] = {
        loaded,
        present,
        value: present ? rec.value : null,
        apType: present ? rec.ap_type : null,
      };
      if (present) presentCells.push({ sid, rec });
    }
    // Need at least two UAVs that have the param to compare/harmonize.
    if (presentCells.length < 2) continue;
    const groups = groupPresentValues(presentCells);
    if (groups.length < 2) continue; // all present UAVs agree — not divergent
    // Read-only params (firmware-maintained: STAT_* counters, *_DEVID, *_GND_PRESS,
    // MIS_TOTAL, ...) can never be written — the firmware ignores/overwrites the
    // value. A divergent read-only param is kept as a VIEW-ONLY row (readOnly:true,
    // defaultWinner:null so it's inert in every write/selection path) so the
    // operator can still inspect the per-UAV values in the dedicated 'read-only'
    // sidebar group, but it's never harmonizable. Checked after the agreement test
    // so only would-be-divergent read-only params become rows (identical ones are
    // dropped by the agreement test — nothing to show). read_only is backend-set,
    // name-derived and uniform across the fleet.
    if (presentCells.some(({ rec }) => rec.read_only)) {
      counts.readOnly += 1;
      rows.push({
        name, safetyTag: null, isUavSpecific: false, perVehicle,
        distinctValues: groups.map((g) => ({ value: g.value, sysIds: g.sysIds, count: g.count })),
        presentSysIds: presentCells.map((c) => c.sid),
        modalValue: null, defaultWinner: null, typeMismatch: false, partial: false,
        status: 'read-only', defaultSelected: false, readOnly: true,
      });
      continue;
    }

    const apTypes = new Set(presentCells.map((c) => c.rec.ap_type));
    const typeMismatch = apTypes.size > 1;
    const partial = presentCells.length < loadedIds.length;

    // The modal value = the unique MOST-COMMON value (its group's count strictly
    // exceeds the runner-up). This is plurality, not strict majority: a 4-UAV
    // 2-1-1 split still yields a modal winner (the "2"); a 2-2 tie yields none.
    const modal = groups[0].count > (groups[1] ? groups[1].count : 0) ? groups[0] : null;
    const modalValue = modal ? modal.value : null;
    // Don't default a winner when types disagree — grouping isn't trustworthy.
    const defaultWinner = (modal && !typeMismatch) ? modalValue : null;

    const safetyTag = safetyTagFor(name);
    const isUavSpecific = safetyTag === 'identity-cal-skip';
    // Auto-select only a clean case: a modal winner, not identity/cal (with skip
    // on), the param present on every loaded UAV (not partial), and the whole
    // fleet loaded. Everything else is shown but requires an explicit tick.
    const defaultSelected = defaultWinner !== null
      && !(skipUavSpecific && isUavSpecific)
      && !partial
      && allLoaded;

    rows.push({
      name,
      safetyTag,
      isUavSpecific,
      perVehicle,
      distinctValues: groups.map((g) => ({ value: g.value, sysIds: g.sysIds, count: g.count })),
      presentSysIds: presentCells.map((c) => c.sid),
      modalValue,
      defaultWinner,
      typeMismatch,
      partial,
      status: 'divergent',
      defaultSelected,
    });

    counts.divergent += 1;
    if (isUavSpecific && skipUavSpecific) counts.skippedDivergent += 1;
    if (!modal) counts.noMajority += 1;
    if (typeMismatch) counts.typeMismatch += 1;
    if (partial) counts.partial += 1;
    if (defaultSelected) counts.defaultSelectedRows += 1;
  }

  return { rows, counts, allLoaded };
}

/**
 * Per-present-UAV cell state for the grid, computed with the SAME coercion as the
 * writer so highlighting can't drift from what apply would do. Returns
 * `{ [sid]: { isWinner, isOutlier, wouldWrite } }` — `isWinner` cells already hold
 * the (coerced) winner value; `isOutlier` cells differ and would be written.
 */
export function harmonizeCellStates(row, winner) {
  const out = {};
  if (!row) return out;
  const w = winner === undefined ? row.defaultWinner : winner;
  for (const sid of (row.presentSysIds || [])) {
    const cell = row.perVehicle[sid];
    const rec = { ap_type: cell.apType };
    let isWinner = false;
    let isOutlier = false;
    if (w !== null && w !== undefined) {
      const stored = coerceValueForRecord(rec, w);
      if (stored !== null) {
        if (valuesEqual(stored, cell.value, rec)) isWinner = true;
        else isOutlier = true;
      }
    }
    out[sid] = { isWinner, isOutlier, wouldWrite: isOutlier };
  }
  return out;
}

// An operator pick is stored as { value, sysId } — the value chosen AND the cell
// the operator clicked (the "base"). Legacy/plain values are still accepted so the
// model stays tolerant. These helpers read either shape.
function pickValueOf(entry) {
  return (entry && typeof entry === 'object' && !Array.isArray(entry)) ? entry.value : entry;
}
function pickSysIdOf(entry) {
  return (entry && typeof entry === 'object' && !Array.isArray(entry)) ? entry.sysId : undefined;
}

// The winner value in effect for a row: an explicit operator choice overrides the
// default (modal) winner — but ONLY while that choice is still a value some present
// UAV holds. A stale pick (the row changed / disappeared and reappeared with new
// values) self-heals back to the modal winner, so apply can never write a value the
// operator didn't pick in the current view. Returns null when there's no winner yet.
export function effectiveWinner(row, winnersByName) {
  if (row && winnersByName && Object.prototype.hasOwnProperty.call(winnersByName, row.name)) {
    const w = pickValueOf(winnersByName[row.name]);
    for (const sid of (row.presentSysIds || [])) {
      const cell = row.perVehicle[sid];
      const rec = { ap_type: cell.apType };
      const stored = coerceValueForRecord(rec, w);
      if (stored !== null && valuesEqual(stored, cell.value, rec)) return w;
    }
  }
  return row ? row.defaultWinner : null;
}

// Does a present UAV currently hold the (coerced) winner value?
function cellHoldsWinner(row, sid, winner) {
  const cell = row.perVehicle[sid];
  if (!cell || !cell.present) return false;
  const rec = { ap_type: cell.apType };
  const stored = coerceValueForRecord(rec, winner);
  return stored !== null && valuesEqual(stored, cell.value, rec);
}

// The UAV cell to badge as the "base" — the source of the winning value. Honors the
// operator's clicked cell while it still holds the winner; otherwise falls back to
// the first present UAV that holds the winner (a stable representative source for the
// default/modal winner, so exactly ONE green cell is marked as the base). Returns
// null when the row has no effective winner.
export function effectiveBaseSysId(row, winnersByName) {
  if (!row) return null;
  const w = effectiveWinner(row, winnersByName);
  if (w === null || w === undefined) return null;
  const entry = (winnersByName && Object.prototype.hasOwnProperty.call(winnersByName, row.name))
    ? winnersByName[row.name] : undefined;
  const pickedSid = pickSysIdOf(entry);
  if (pickedSid !== undefined && pickedSid !== null && cellHoldsWinner(row, pickedSid, w)) {
    return pickedSid;
  }
  for (const sid of (row.presentSysIds || [])) {
    if (cellHoldsWinner(row, sid, w)) return sid;
  }
  return null;
}

/**
 * Present UAVs whose current stored value differs from the (effective) winner —
 * i.e. the ones a harmonize apply would write. Uses per-target coercion so it
 * matches the scoped writer exactly.
 */
export function outlierSysIds(row, winner) {
  if (!row) return [];
  const w = winner === undefined ? row.defaultWinner : winner;
  if (w === null || w === undefined) return [];
  const out = [];
  for (const sid of (row.presentSysIds || [])) {
    const cell = row.perVehicle[sid];
    if (!cell || !cell.present) continue;
    const rec = { ap_type: cell.apType };
    const stored = coerceValueForRecord(rec, w);
    if (stored === null) continue;
    if (!valuesEqual(stored, cell.value, rec)) out.push(sid);
  }
  return out;
}

/**
 * Collect the changes to write for the operator's selected rows. For each
 * selected row with a chosen winner, push the winner (coerced to the target's
 * type) to the PRESENT UAVs whose current value differs. Returns
 * `{ [sysId]: [{name, value}] }`. Rows with no winner (no majority + no manual
 * pick) contribute nothing.
 *
 * NOTE: object keys are stringified sys_ids — the apply caller must pass numeric
 * sysIds from vehicle state to writeChangedToVehicles, not Object.keys(...).
 */
export function collectHarmonizeChanges(rows, selectedNames, winnersByName) {
  const out = {};
  if (!Array.isArray(rows)) return out;
  const selected = selectedNames instanceof Set ? selectedNames : new Set(selectedNames || []);
  for (const row of rows) {
    // Read-only rows are view-only — never written, even if a stray selection or
    // winner entry names one (defense-in-depth; the UI already gates picking).
    if (!row || row.readOnly || !selected.has(row.name)) continue;
    const w = effectiveWinner(row, winnersByName);
    if (w === null || w === undefined) continue;
    for (const sid of (row.presentSysIds || [])) {
      const cell = row.perVehicle[sid];
      if (!cell || !cell.present) continue;
      const rec = { ap_type: cell.apType };
      const stored = coerceValueForRecord(rec, w);
      if (stored === null) continue;
      if (valuesEqual(stored, cell.value, rec)) continue; // already equals winner
      if (!out[sid]) out[sid] = [];
      out[sid].push({ name: row.name, value: stored });
    }
  }
  return out;
}

/** Names selected by default: divergent rows with a default winner, not skipped. */
export function defaultHarmonizeSelection(rows) {
  const out = [];
  for (const row of (Array.isArray(rows) ? rows : [])) {
    if (row && row.defaultSelected) out.push(row.name);
  }
  return out;
}

/**
 * Reconcile a prior selection against a recomputed model: keep selected names
 * that are still divergent rows AND still have an effective winner (default modal
 * or a still-valid manual pick). Applied rows that became equal drop out of the
 * model; rows that lost their winner (e.g. now a tie with a stale pick) drop out
 * of the selection so there's no checked-but-unwritable row.
 */
export function reconcileHarmonizeSelection(selectedNames, rows, winnersByName) {
  const byName = new Map();
  for (const r of (Array.isArray(rows) ? rows : [])) byName.set(r.name, r);
  const sel = selectedNames instanceof Set ? selectedNames : new Set(selectedNames || []);
  const out = [];
  for (const name of sel) {
    const row = byName.get(name);
    if (!row) continue;
    const w = effectiveWinner(row, winnersByName);
    if (w === null || w === undefined) continue;
    out.push(name);
  }
  return out;
}

/**
 * Names to forget from the manual-pick winner map because
 * reconcileHarmonizeSelection just dropped their row from the selection during a
 * plain recompute (no longer divergent, or lost its effective winner). Same
 * "unselected rows must not retain a manual winner" rule already enforced for the
 * two operator-driven deselect paths (toggling a row off, "select all" off) — this
 * closes the third path: a post-write/refresh recompute that drops a row without an
 * explicit operator action. Without this, a stale pick could resurface via
 * effectiveWinner if the same param name diverges again later and the picked value
 * happens to still be held by some present UAV.
 */
export function droppedHarmonizeWinnerNames(priorSelectedNames, reconciledNames) {
  const prior = priorSelectedNames instanceof Set
    ? priorSelectedNames : new Set(priorSelectedNames || []);
  const kept = new Set(reconciledNames || []);
  const out = [];
  for (const name of prior) {
    if (!kept.has(name)) out.push(name);
  }
  return out;
}
