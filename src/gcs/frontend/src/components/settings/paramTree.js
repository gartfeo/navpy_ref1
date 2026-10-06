// Pure helpers for the Parameters tab's prefix sidebar + filter logic.
// No React, no fetch. Tested via tests/gcs/test_param_tree_js.py.

/**
 * Canonical merge of every vehicle's `nameOrder` for `sysIds`.
 *
 *   - Walk vehicles in `sysIds` order.
 *   - For the first vehicle, take its `nameOrder` verbatim.
 *   - For each subsequent vehicle, append names not already seen.
 *
 * This way the row order in the grid is deterministic across reloads
 * and stays close to backend (param.pck) order — first vehicle wins for
 * ordering, later vehicles only contribute previously-unseen names.
 */
export function canonicalNameOrder(snapshotsByVehicle, sysIds) {
  const seen = new Set();
  const order = [];
  for (const sid of sysIds) {
    const snap = snapshotsByVehicle?.[sid];
    if (!snap || !Array.isArray(snap.nameOrder)) continue;
    for (const name of snap.nameOrder) {
      if (!seen.has(name)) {
        seen.add(name);
        order.push(name);
      }
    }
  }
  return order;
}

/**
 * Take the canonical name list and group by `_`-delimited prefix.
 *
 * Returns an alphabetically-sorted list of `{prefix, count, children}`.
 * Children are second-level prefixes (`INS_POS1`) for names with at least
 * two underscores. Names with no underscore remain available under All
 * and search, but do not create a visible sidebar group.
 */
export function buildPrefixSidebar(canonicalNames) {
  const groups = new Map();
  const ensureGroup = (prefix) => {
    if (!groups.has(prefix)) {
      groups.set(prefix, { prefix, count: 0, children: new Map() });
    }
    return groups.get(prefix);
  };
  for (const name of canonicalNames) {
    const parts = name.split('_');
    if (parts.length < 2) {
      continue;
    }
    const prefix = parts[0];
    const group = ensureGroup(prefix);
    group.count += 1;
    if (parts.length > 2) {
      const childPrefix = `${parts[0]}_${parts[1]}`;
      group.children.set(childPrefix, (group.children.get(childPrefix) || 0) + 1);
    }
  }
  return Array.from(groups.values())
    .map((group) => ({
      prefix: group.prefix,
      count: group.count,
      children: Array.from(group.children.entries())
        .map(([prefix, count]) => ({ prefix, count }))
        .sort((a, b) => a.prefix.localeCompare(b.prefix)),
    }))
    .sort((a, b) => a.prefix.localeCompare(b.prefix));
}

/**
 * Filter a canonical name list by the active sidebar prefix and a
 * case-insensitive substring search. Preserves canonical order.
 *
 *   `selectedPrefix === null` means "all".
 *   `searchText === ''` means "no substring filter".
 *   `(root)` matches names with no underscore for compatibility with
 *   older internal callers; the sidebar no longer exposes it.
 */
export function filterCanonicalNames(canonicalNames, { selectedPrefix, searchText }) {
  const search = (searchText || '').trim().toLowerCase();
  const out = [];
  for (const name of canonicalNames) {
    if (selectedPrefix && selectedPrefix !== '(all)') {
      if (selectedPrefix === '(root)') {
        if (name.includes('_')) continue;
      } else if (!name.startsWith(`${selectedPrefix}_`)) {
        continue;
      }
    }
    if (search && !name.toLowerCase().includes(search)) continue;
    out.push(name);
  }
  return out;
}

/**
 * Pure virtualisation maths: which row indices are inside the viewport
 * (with overscan)? Used by VirtualParamGrid; extracted so the math is
 * testable without rendering.
 *
 * Always returns `start <= end` even when `scrollTop` is stale after a
 * filter has shrunk `rowCount` (per Codex post-step finding 4 for Step 5).
 */
export function computeVisibleRange(scrollTop, viewportHeight, rowHeight, rowCount, overscan = 6) {
  if (rowHeight <= 0 || rowCount <= 0) {
    return { start: 0, end: 0 };
  }
  const safeScroll = Math.max(0, scrollTop || 0);
  const safeViewport = Math.max(0, viewportHeight || 0);
  const firstVisible = Math.min(
    rowCount - 1,
    Math.max(0, Math.floor(safeScroll / rowHeight)),
  );
  const lastVisible = Math.ceil((safeScroll + safeViewport) / rowHeight);
  const start = Math.max(0, firstVisible - overscan);
  const end = Math.max(start, Math.min(rowCount, lastVisible + overscan));
  return { start, end };
}

/**
 * Tolerant value-equality used by the grid for "all same" detection.
 * Mirrors the rule from utils/fullParams.js so partial-missing surfaces
 * as not-all-same and float drift doesn't false-positive a mixed badge.
 */
function _valuesEqualTolerant(a, b, isInt) {
  if (a == null || b == null) return a === b;
  const na = Number(a);
  const nb = Number(b);
  if (!Number.isFinite(na) || !Number.isFinite(nb)) return false;
  if (isInt) return Math.round(na) === Math.round(nb);
  const tol = 1e-5 * Math.max(Math.abs(na), Math.abs(nb), 1.0);
  return Math.abs(na - nb) <= tol;
}

/**
 * Returns true when every vehicle in `records` (in matching order) has
 * the parameter present AND every value matches within float tolerance.
 *
 *   - Any missing record → false (partial-missing is a mismatch).
 *   - Single-vehicle case → true (no comparison needed).
 */
export function recordsAllSame(records) {
  if (!Array.isArray(records) || records.length === 0) return true;
  if (records.some((r) => r == null)) return false;
  if (records.length === 1) return true;
  const first = records[0];
  const isInt = first.ap_type >= 1 && first.ap_type <= 3;
  for (let i = 1; i < records.length; i++) {
    if (!_valuesEqualTolerant(first.value, records[i].value, isInt)) {
      return false;
    }
  }
  return true;
}

/**
 * Set of parameter names whose value differs across the LOADED UAVs (or is
 * present on some loaded UAVs and missing on others). Compares only vehicles
 * that actually have a snapshot, so a still-downloading or failed UAV doesn't
 * make every parameter look "different". Needs >= 2 loaded snapshots — returns
 * an empty set otherwise. Reuses `recordsAllSame`'s tolerant comparison.
 */
export function differingParamNames(snapshotsByVehicle, sysIds, names) {
  const out = new Set();
  const loaded = (Array.isArray(sysIds) ? sysIds : [])
    .filter((sid) => snapshotsByVehicle?.[sid]?.paramsByName);
  if (loaded.length < 2) return out;
  for (const name of names || []) {
    const records = loaded.map((sid) => snapshotsByVehicle[sid].paramsByName[name]);
    if (!recordsAllSame(records)) out.add(name);
  }
  return out;
}

/**
 * Format a parameter value for read-only display. Integer-typed values
 * render without trailing zeros; floats render with up to 6 significant
 * digits to keep columns tight while still distinguishing close values.
 */
export function formatValueForDisplay(record) {
  if (record == null || record.value == null) return '';
  const isInt = record.ap_type >= 1 && record.ap_type <= 3;
  const v = record.value;
  if (isInt) return String(Math.round(Number(v)));
  if (typeof v !== 'number') return String(v);
  if (!Number.isFinite(v)) return String(v);
  // 6 significant digits, then trim insignificant zeros.
  return Number(v.toPrecision(6)).toString();
}
