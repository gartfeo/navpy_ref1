// Pure helpers for full-parameter state. No React, no fetch.
// Tested via tests/gcs/test_full_params_js.py (Node subprocess).

/**
 * Normalize a backend snapshot response into the shape useFullParams stores.
 *   `records`: ordered array, preserves backend order (Step 5 tree/search).
 *   `paramsByName`: O(1) lookup map.
 *   `nameOrder`: list of names for stable iteration without re-walking records.
 */
export function normaliseSnapshot(rawSnapshot) {
  const records = Array.isArray(rawSnapshot?.params) ? rawSnapshot.params : [];
  const paramsByName = {};
  const nameOrder = [];
  for (const r of records) {
    if (!r || typeof r.name !== 'string') continue;
    paramsByName[r.name] = r;
    nameOrder.push(r.name);
  }
  return {
    records,
    paramsByName,
    nameOrder,
    numParams: rawSnapshot?.num_params ?? records.length,
    totalParams: rawSnapshot?.total_params ?? records.length,
    fetchedAtUnixS: rawSnapshot?.fetched_at_unix_s ?? null,
    stale: !!rawSnapshot?.stale,
  };
}

/** Fetch a fleet concurrently and publish each vehicle as soon as it settles. */
export async function fetchFullParamFleet(
  sysIds,
  fetchOne,
  onSuccess = () => {},
  onSettled = () => {},
) {
  return Promise.all((sysIds || []).map(async (sysId) => {
    let outcome;
    try {
      const data = await fetchOne(sysId);
      if (!data) throw new Error('download failed');
      onSuccess(sysId, data);
      outcome = { sysId, ok: true, data, error: null };
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error || 'download failed');
      outcome = { sysId, ok: false, data: null, error: message };
    }
    onSettled(sysId, outcome);
    return outcome;
  }));
}

/** True only while a refresh still owns the current connection for this UAV. */
export function ownsFullParamRefresh(generations, liveSysIds, sysId, generation) {
  return liveSysIds?.has?.(sysId) && generations?.[sysId] === generation;
}

/**
 * Convert useFullParams' refresh status into real byte progress state.
 */
export function fullParamLoadProgress(status) {
  if (!status || !status.counts) return null;
  if (status.kind !== 'downloading' && status.op !== 'refresh') return null;
  const counts = status.counts || {};
  const total = Math.max(0, Math.floor(Number(counts.uavTotal) || 0));
  const done = Math.min(
    total,
    Math.max(0, Math.floor(Number(counts.uavDone) || 0)),
  );
  const ok = Math.min(
    done,
    Math.max(0, Math.floor(Number(counts.uavOk) || 0)),
  );
  const byVehicle = counts.byVehicle || {};
  const vehicleProgress = Object.keys(byVehicle)
    .sort((a, b) => Number(a) - Number(b))
    .map((sid) => {
      const p = byVehicle[sid] || {};
      const bytesRead = Math.max(0, Math.floor(Number(p.bytesRead) || 0));
      const sizeEstimate = Math.max(0, Math.floor(Number(p.sizeEstimate) || 0));
      const totalBytes = Math.max(0, Math.floor(Number(p.totalBytes) || 0));
      const denominator = totalBytes || sizeEstimate;
      const percent = denominator > 0
        ? Math.min(100, Math.round((bytesRead / denominator) * 100))
        : (p.done ? 100 : 0);
      // No bytes and no known size yet → MAVFTP setup phase: the header shows
      // an indeterminate spinner rather than a ring stuck at 0%.
      const hasBytes = bytesRead > 0 || sizeEstimate > 0 || totalBytes > 0;
      return {
        sysId: Number(sid),
        bytesRead,
        sizeEstimate,
        totalBytes,
        percent,
        hasBytes,
        done: !!p.done,
        error: p.error || null,
      };
    });
  const bytesRead = vehicleProgress.reduce((acc, p) => acc + p.bytesRead, 0);
  const sizeEstimate = vehicleProgress.reduce((acc, p) => acc + p.sizeEstimate, 0);
  const totalBytes = vehicleProgress.reduce(
    (acc, p) => acc + (p.totalBytes || (p.done ? p.bytesRead : 0)),
    0,
  );
  const observedVehicles = vehicleProgress.length;
  const knownTotal = totalBytes > 0
    && total > 0
    && observedVehicles >= total
    && vehicleProgress.every((p) => p.done);
  const missingVehicles = Math.max(0, total - observedVehicles);
  const percent = total > 0
    ? Math.min(
      100,
      Math.round(
        (
          vehicleProgress.reduce((acc, p) => acc + p.percent, 0)
          + (done > observedVehicles ? (done - observedVehicles) * 100 : 0)
        ) / (observedVehicles + missingVehicles),
      ),
    )
    : 0;
  return {
    done,
    ok,
    total,
    bytesRead,
    sizeEstimate,
    totalBytes,
    percent,
    knownTotal,
    hasBytes: bytesRead > 0 || sizeEstimate > 0 || totalBytes > 0,
    vehicleProgress,
  };
}

/**
 * Decide what a grid column header shows for one vehicle. Pure (no React).
 *
 *   'active' -> still downloading: ring + percent
 *   'error'  -> the download failed: ✕
 *   'done'   -> finished (progress reported done) OR a snapshot is loaded: check
 *   'idle'   -> connected but nothing fetched yet: no indicator
 *
 * `done` deliberately keys off progress.done as well as `loaded` so a UAV that
 * finishes early keeps an explicit success state while slower peers continue.
 */
export function paramHeaderStatus(progress, loaded) {
  if (progress && !progress.done && !progress.error) return 'active';
  if (progress && progress.error) return 'error';
  if (loaded || (progress && progress.done)) return 'done';
  return 'idle';
}

/**
 * Return names whose current per-vehicle value differs from a known default.
 * Unknown defaults are not classified.
 */
export function selectNonDefaultNames(
  snapshotsByVehicle,
  sysIds,
  candidateNames,
  draftsByVehicle = {},
) {
  const out = [];
  for (const name of candidateNames || []) {
    let nonDefault = false;
    for (const sid of sysIds || []) {
      const rec = snapshotsByVehicle?.[sid]?.paramsByName?.[name];
      if (!rec || !rec.default_known) continue;
      const draft = draftsByVehicle?.[sid];
      const hasDraft = draft && Object.prototype.hasOwnProperty.call(draft, name);
      let value = rec.value;
      if (hasDraft) {
        const coerced = coerceValueForRecord(rec, draft[name]);
        value = coerced === null ? draft[name] : coerced;
      }
      if (!valuesEqual(value, rec.default, rec)) {
        nonDefault = true;
        break;
      }
    }
    if (nonDefault) out.push(name);
  }
  return out;
}

// AP packed storage types: 1=INT8, 2=INT16, 3=INT32, 4=FLOAT.
// Signed storage range per AP integer type. `param.pck` carries the storage
// type only — not logical @Range metadata — so this is the storage range.
const AP_INT_BOUNDS = {
  1: [-128, 127],
  2: [-32768, 32767],
  3: [-2147483648, 2147483647],
};

const AP_INT_LABEL = { 1: 'int8', 2: 'int16', 3: 'int32' };

// Coerce a finite number to the integer an AP int param would STORE: truncate
// toward zero, then clamp to the signed storage range. Mirrors the autopilot's
// AP_Param::set_float (trunc + constrain), so the editor and the .param
// importer stage exactly what the vehicle will hold.
function toStoredInt(bounds, n) {
  return Math.min(bounds[1], Math.max(bounds[0], Math.trunc(n)));
}

/**
 * Describe an AP packed type for display: a readable label and (for ints) the
 * signed storage range. Returns `{ label, isInt, min, max }`.
 */
export function describeParamType(apType) {
  const bounds = AP_INT_BOUNDS[apType];
  if (bounds) {
    return { label: AP_INT_LABEL[apType], isInt: true, min: bounds[0], max: bounds[1] };
  }
  if (apType === 4) return { label: 'float', isInt: false, min: null, max: null };
  return { label: `ap_type ${apType}`, isInt: false, min: null, max: null };
}

/**
 * Coerce a value (possibly a numeric string from an editor) to the value the
 * record's param would STORE. Returns `null` only when the value is not a
 * finite number.
 *
 * AP packed types: 1=INT8, 2=INT16, 3=INT32, 4=FLOAT. Integer params truncate
 * toward zero and clamp to their storage range (honest-permissive: stage what
 * the autopilot will actually hold, e.g. 2.1 -> 2, 200 -> 127 for int8). Float
 * params pass the finite value through.
 */
export function coerceValueForRecord(record, value) {
  if (record == null) return null;
  if (typeof value === 'string') {
    const trimmed = value.trim();
    if (trimmed === '') return null;
  }
  const n = Number(value);
  if (!Number.isFinite(n)) return null;
  const bounds = AP_INT_BOUNDS[record.ap_type];
  if (bounds) return toStoredInt(bounds, n);
  return n;
}

/**
 * Compare two values for "equal as a parameter would be stored". Integer
 * params compare by their stored integer (truncate + clamp), so a draft that
 * coerces to the current stored value counts as no change. Floats use a
 * relative tolerance because draft-from-string and confirmed-from-float may
 * differ in trailing precision.
 */
export function valuesEqual(a, b, record) {
  if (a == null || b == null) return a === b;
  const bounds = record && AP_INT_BOUNDS[record.ap_type];
  const na = Number(a);
  const nb = Number(b);
  if (!Number.isFinite(na) || !Number.isFinite(nb)) return false;
  if (bounds) return toStoredInt(bounds, na) === toStoredInt(bounds, nb);
  const tol = 1e-5 * Math.max(Math.abs(na), Math.abs(nb), 1.0);
  return Math.abs(na - nb) <= tol;
}

/**
 * Compare an editor draft against a typed submitted/snapshot value using
 * the same coercion rules as the write path.
 */
export function draftValueMatchesSubmitted(record, draftValue, submittedValue) {
  const coerced = coerceValueForRecord(record, draftValue);
  if (coerced === null) return false;
  return valuesEqual(coerced, submittedValue, record);
}

/**
 * Diff a draft against the cached snapshot for one vehicle. Returns an
 * ordered list of `{name, value}` for every name whose draft value has
 * changed (and coerces correctly).
 *
 * Names not present in the snapshot are dropped — Step 7's `.param` load
 * is responsible for surfacing those as rejected rows BEFORE staging
 * them into the draft.
 */
export function selectChangedNames(draft, snapshot) {
  const out = [];
  if (!draft || !snapshot || !snapshot.paramsByName) return out;
  for (const name of Object.keys(draft)) {
    const rec = snapshot.paramsByName[name];
    if (!rec) continue;
    const coerced = coerceValueForRecord(rec, draft[name]);
    if (coerced === null) continue;
    if (!valuesEqual(coerced, rec.value, rec)) {
      out.push({ name, value: coerced });
    }
  }
  return out;
}

/**
 * Resolve what to PUT per vehicle, applying the SAME snapshot gate to both the
 * default draft path and an explicit scoped-changes path (e.g. compare-apply):
 * names absent from the current snapshot are dropped, values are coerced to the
 * stored value, non-finite values are dropped, and values already equal are
 * dropped. Returns `{ [sid]: [{name, value}] }`.
 *
 *   ids                vehicle ids to resolve (active/column order)
 *   changesByVehicle   optional explicit `{ [sid]: [{name, value}] }`. When
 *                      provided, ONLY these names are considered (drafts are
 *                      ignored) so a scoped apply can't push unrelated manual
 *                      edits; still re-validated against the live snapshot.
 *                      When omitted/null, the per-vehicle draft is diffed
 *                      (current "Write Changed" behavior).
 *   draftsByVehicle    `{ [sid]: { [name]: value } }`
 *   snapshotsByVehicle `{ [sid]: normalisedSnapshot }`
 */
export function resolveSubmissions(ids, {
  changesByVehicle, draftsByVehicle, snapshotsByVehicle,
} = {}) {
  const out = {};
  for (const sid of (Array.isArray(ids) ? ids : [])) {
    const snap = snapshotsByVehicle ? snapshotsByVehicle[sid] : null;
    let source;
    if (changesByVehicle) {
      // Build a draft-shaped map from the explicit changes, then run it through
      // the same gate as drafts so semantics are identical.
      source = {};
      for (const ch of (changesByVehicle[sid] || [])) {
        if (ch && typeof ch.name === 'string') source[ch.name] = ch.value;
      }
    } else {
      source = draftsByVehicle ? draftsByVehicle[sid] : null;
    }
    out[sid] = selectChangedNames(source, snap);
  }
  return out;
}

/**
 * Restrict a `selectChangedNames` diff to a given set of names. Used by the
 * curated Failsafe tab so a per-group "Save" writes only that group's params
 * even though the draft store is shared across the whole parameter view.
 * `names` may be an array or a Set; a nullish `names` is a no-op pass-through.
 */
export function filterChangesByNames(changes, names) {
  if (names == null) return changes || [];
  const set = names instanceof Set ? names : new Set(names);
  return (changes || []).filter((c) => set.has(c.name));
}

/**
 * Aggregate per-vehicle PUT results into UI-friendly counters. Takes
 * `submittedByVehicle` so we can tell "network failure with unsent
 * changes" apart from "vehicle had no changes to submit".
 */
export function aggregateBatchResults(perVehicleResults, submittedByVehicle) {
  const sysIds = Object.keys(submittedByVehicle || {});
  let uavOk = 0;
  let uavFail = 0;
  let cellOk = 0;
  let cellFail = 0;
  const errors = [];
  for (const sid of sysIds) {
    const submitted = submittedByVehicle[sid] || [];
    if (submitted.length === 0) continue;  // nothing attempted; skip from counts
    const results = perVehicleResults?.[sid];
    if (!results) {
      // Network failure / no response — every submitted cell counts as fail.
      uavFail += 1;
      cellFail += submitted.length;
      for (const change of submitted) {
        errors.push({ sysId: sid, name: change.name, error: 'no response' });
      }
      continue;
    }
    let vehicleOk = true;
    for (const change of submitted) {
      const r = results[change.name];
      if (r && r.ok) {
        cellOk += 1;
      } else {
        cellFail += 1;
        vehicleOk = false;
        errors.push({
          sysId: sid,
          name: change.name,
          error: (r && r.error) || 'no result',
        });
      }
    }
    if (vehicleOk) uavOk += 1; else uavFail += 1;
  }
  return { uavOk, uavFail, cellOk, cellFail, errors };
}

/**
 * Multi-vehicle consensus for one parameter name. Distinguishes "vehicle's
 * snapshot not fetched yet" (loading) from "name absent on all" (unknown).
 *
 *   loading  — at least one vehicle in `sysIds` has no snapshot loaded.
 *   unknown  — all snapshots loaded but none contain the name.
 *   ready    — all loaded snapshots have the same value for the name.
 *   mixed    — loaded snapshots disagree on the value.
 */
export function consensusByName(snapshotsByVehicle, sysIds, name) {
  if (!Array.isArray(sysIds) || sysIds.length === 0) {
    return { kind: 'unknown' };
  }
  let anyLoaded = false;
  let anyKnown = false;
  let anyMissing = false;
  let firstValue;
  let firstRecord;
  let mixed = false;
  for (const sid of sysIds) {
    const snap = snapshotsByVehicle?.[sid];
    if (!snap || !snap.paramsByName) {
      return { kind: 'loading' };
    }
    anyLoaded = true;
    const rec = snap.paramsByName[name];
    if (!rec) {
      anyMissing = true;
      continue;
    }
    if (!anyKnown) {
      firstValue = rec.value;
      firstRecord = rec;
      anyKnown = true;
    } else if (!valuesEqual(firstValue, rec.value, firstRecord || rec)) {
      mixed = true;
    }
  }
  if (!anyLoaded) return { kind: 'loading' };
  if (!anyKnown) return { kind: 'unknown' };
  // Partial-missing: some vehicles have the param, others don't — surface
  // as 'mixed' so the UI shows the operator that vehicles disagree (per
  // Codex post-step finding 4 for Step 4).
  if (mixed || anyMissing) return { kind: 'mixed' };
  return { kind: 'ready', value: firstValue, record: firstRecord };
}

/**
 * Prune a `{[sysId]: ...}` map down to the live set, returning the same
 * reference when nothing changed (so React state setters can no-op).
 */
export function pruneToSet(map, liveSysIds) {
  if (!map) return map;
  let changed = false;
  const next = {};
  const live = liveSysIds instanceof Set ? liveSysIds : new Set(liveSysIds || []);
  for (const k of Object.keys(map)) {
    if (live.has(Number(k)) || live.has(k)) {
      next[k] = map[k];
    } else {
      changed = true;
    }
  }
  return changed ? next : map;
}

/**
 * `armedToken` validity check — backend issues a 30s TTL nonce; we drop it
 * client-side as soon as it would be useless.
 */
export function isArmedTokenLive(token, nowUnixS) {
  if (!token || typeof token.nonce !== 'string') return false;
  if (typeof token.expires_at_unix_s !== 'number') return false;
  return nowUnixS < token.expires_at_unix_s;
}
