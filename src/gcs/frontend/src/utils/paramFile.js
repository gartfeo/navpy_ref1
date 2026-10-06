// Mission Planner-style `.param` file format helpers.
//
// File grammar (lenient — Mission Planner accepts both):
//   - lines starting with `#` are comments and are ignored
//   - blank lines are ignored
//   - `NAME,VALUE` (CSV) or `NAME VALUE` (whitespace) per param
//   - param names are ASCII (A-Z, 0-9, _), max 16 chars
//   - values are decimal numbers (int or float)
//
// Save direction: serialise from a snapshot's records. We only emit
// values (no defaults / metadata) — matches Mission Planner output.
//
// Load direction: parse text → return one of three buckets per row:
//   { accepted: [{name, value, rawValue?, coerced?}], skipped: [{name, reason}],
//     rejected: [{name, reason}] }
//
// "skipped" = name unknown to this vehicle (per-vehicle filter applied later by
// the caller). "rejected" = value is not a finite number. Integer rows that
// need truncation/clamping to fit storage are ACCEPTED with the value the
// autopilot would store and a `coerced` flag (honest-permissive — matches the
// inline editor; the caller surfaces the adjustment in the import preview).

import { coerceValueForRecord } from './fullParams';

const NAME_RE = /^[A-Z0-9_]{1,16}$/;

/**
 * Parse `.param` text. Returns `{ rows, comments, errors }` where
 *   - rows is an ordered list of `{name, value}` (raw — caller must validate
 *     against per-vehicle snapshots before applying)
 *   - errors is a list of `{ line, raw, reason }` for malformed lines
 */
export function parseParamFile(text) {
  if (typeof text !== 'string') return { rows: [], errors: [] };
  const rows = [];
  const errors = [];
  const lines = text.split(/\r?\n/);
  for (let i = 0; i < lines.length; i++) {
    const raw = lines[i];
    const trimmed = raw.trim();
    if (!trimmed) continue;
    if (trimmed.startsWith('#')) continue;
    // Try comma first, fall back to whitespace.
    const comma = trimmed.split(',');
    const tokens = comma.length >= 2 ? comma : trimmed.split(/\s+/);
    if (tokens.length < 2) {
      errors.push({ line: i + 1, raw, reason: 'expected NAME,VALUE' });
      continue;
    }
    const name = tokens[0].trim().toUpperCase();
    const valueStr = tokens.slice(1).join(',').trim();
    if (!NAME_RE.test(name)) {
      errors.push({ line: i + 1, raw, reason: `invalid name "${name}"` });
      continue;
    }
    const n = Number(valueStr);
    if (!Number.isFinite(n)) {
      errors.push({ line: i + 1, raw, reason: `non-numeric value "${valueStr}"` });
      continue;
    }
    rows.push({ name, value: n });
  }
  return { rows, errors };
}

/**
 * Validate parsed `.param` rows against ONE vehicle's snapshot.
 * Returns three buckets so the UI can show the operator exactly what
 * will be staged before we modify any draft.
 */
export function validateRowsForVehicle(rows, snapshot) {
  const accepted = [];
  const skipped = [];
  const rejected = [];
  if (!Array.isArray(rows)) return { accepted, skipped, rejected };
  if (!snapshot || !snapshot.paramsByName) {
    // No snapshot for this vehicle — everything is skipped pending fetch.
    for (const r of rows) skipped.push({ name: r.name, reason: 'no snapshot' });
    return { accepted, skipped, rejected };
  }
  for (const r of rows) {
    const rec = snapshot.paramsByName[r.name];
    if (!rec) {
      skipped.push({ name: r.name, reason: 'unknown on this vehicle' });
      continue;
    }
    const coerced = coerceValueForRecord(rec, r.value);
    if (coerced === null) {
      rejected.push({ name: r.name, reason: 'invalid number' });
      continue;
    }
    // Honest-permissive: an int row that doesn't land exactly on a storable
    // value is accepted as the value the autopilot would store (truncate +
    // clamp) and flagged so the preview surfaces the adjustment, never hides it.
    const row = { name: r.name, value: coerced };
    if (Number(r.value) !== coerced) {
      row.rawValue = r.value;
      row.coerced = true;
    }
    accepted.push(row);
  }
  return { accepted, skipped, rejected };
}

/**
 * Convert a snapshot to `.param` text. Mission Planner uses CSV
 * (`NAME,VALUE`) by default with a leading `# Mission Planner ...`
 * comment; we mirror that format so files round-trip with their tools.
 */
export function snapshotToParamFile(snapshot, { vehicleName, exportedAtIso } = {}) {
  if (!snapshot || !Array.isArray(snapshot.records)) return '';
  const lines = [];
  lines.push('# NavPy Parameter File');
  if (vehicleName) lines.push(`# Vehicle: ${vehicleName}`);
  if (exportedAtIso) lines.push(`# Exported: ${exportedAtIso}`);
  lines.push('# NAME,VALUE');
  for (const rec of snapshot.records) {
    if (!rec || typeof rec.name !== 'string') continue;
    let valStr;
    if (rec.ap_type >= 1 && rec.ap_type <= 3) {
      valStr = String(Math.round(Number(rec.value)));
    } else {
      const n = Number(rec.value);
      valStr = Number.isFinite(n) ? n.toPrecision(8).replace(/\.?0+$/, '') : String(rec.value);
    }
    lines.push(`${rec.name},${valStr}`);
  }
  return lines.join('\n') + '\n';
}
