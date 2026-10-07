/**
 * Pure helpers and defaults for the per-vehicle AAS parameter store.
 *
 * The canonical source of truth for AAS values is the autopilot. These
 * helpers operate on the GCS-side cache: per-vehicle "confirmed" values
 * (downloaded or PARAM_VALUE-acked truth), per-vehicle staged drafts,
 * and a session draft used only when no vehicle is connected.
 *
 * No React, no fetch -- every input/output is a plain JS value so the
 * helpers can be unit-tested directly via Node.
 */


/**
 * Frontend session defaults for pre-connect editing. Used only when no
 * vehicle is connected so the planning sidebar and settings modal can
 * render reasonable initial values; on vehicle connect, the autopilot's
 * downloaded values become the displayed baseline. Defaults are never
 * pushed to a vehicle automatically. Locked by test_defaults_match_expected.
 */
export const AAS_DEFAULTS = {
  del_pitch: 0,
  del_thr: -1,
  del_dir: false,
  del_p_kp: 1.5,
  del_pld: 100,
  del_plrd: 2,
  del_ctrl: 2,
  use_trn: true,
  targ_wps: 16,
  targ_alt: 150,
  nav_last_wp: 3,
  nav_min_alt: 150,
  nav_cwt: 30,
  nav_cgt: 15,
  nav_auto_cm: true,
  // MISS-04: REJECT on confirm-window expiry unless explicitly re-enabled.
  nav_cm_fl: false,
  nav_oneshot: false,
  log_defer: false,
  log_rate: 2,
};


/**
 * Compute the cross-vehicle consensus for a single AAS key.
 *
 * `confirmedByVehicle` is `{ [sysId]: { key: value, ... } }`. A vehicle
 * is "loaded" once it has an entry in this map (its initial download has
 * completed); a sys_id with no entry is "pending."
 *
 * State semantics:
 *   - session: no sys_ids supplied -- caller should use sessionDraft + AAS_DEFAULTS.
 *   - loading: at least one sys_id has no entry yet.
 *   - mixed:   loaded vehicles disagree, OR some loaded vehicles have a
 *              defined value and others have undefined for this key
 *              (partial-missing).
 *   - ready:   every loaded vehicle reports the same defined value.
 *   - unknown: every loaded vehicle reports undefined for this key.
 *
 * Always returns a plain object so the result survives JSON round-trips
 * in Node-subprocess tests.
 *
 * @param {Object} confirmedByVehicle - `{ [sysId]: { key: value } }`.
 * @param {Array<number|string>} sysIds - Vehicles to consider.
 * @param {string} key - AAS field name.
 * @returns {{state: string, value?: any, valuesBySysId?: Object}}
 */
export function getConsensus(confirmedByVehicle, sysIds, key) {
  if (!sysIds || sysIds.length === 0) return { state: 'session' };

  const cbv = confirmedByVehicle || {};

  for (const sid of sysIds) {
    if (!Object.prototype.hasOwnProperty.call(cbv, sid)) {
      return { state: 'loading' };
    }
  }

  const valuesBySysId = {};
  let firstDefined;
  let firstDefinedSet = false;
  let anyDefined = false;
  let anyUndefined = false;
  let mixed = false;
  for (const sid of sysIds) {
    const v = cbv[sid] ? cbv[sid][key] : undefined;
    // JSON.stringify drops undefined, so coerce to null for round-trip safety.
    valuesBySysId[sid] = v === undefined ? null : v;
    if (v === undefined) {
      anyUndefined = true;
    } else {
      anyDefined = true;
      if (!firstDefinedSet) {
        firstDefined = v;
        firstDefinedSet = true;
      } else if (v !== firstDefined) {
        mixed = true;
      }
    }
  }

  if (!anyDefined) return { state: 'unknown' };
  if (anyUndefined) mixed = true;

  if (mixed) return { state: 'mixed', valuesBySysId };
  return { state: 'ready', value: firstDefined };
}


/**
 * Decide what value the UI should display for a key, given a consensus
 * result. Returns null when the caller should render a placeholder
 * (mixed / loading / unknown / no default available).
 *
 * Kept separate from getConsensus by design -- consensus is about cross-
 * vehicle agreement; display policy is about which fallback to use when
 * there is no consensus.
 *
 * @param {Object} consensus - Result of getConsensus.
 * @param {Object} sessionDraft - Pre-connect edits, `{ key: value }`.
 * @param {string} key - AAS field name.
 * @returns {*} Value to display, or null for placeholder.
 */
export function pickDisplayValue(consensus, sessionDraft, key) {
  if (!consensus) return null;
  if (consensus.state === 'ready') return consensus.value;
  if (consensus.state === 'session') {
    const sd = sessionDraft || {};
    if (Object.prototype.hasOwnProperty.call(sd, key)) return sd[key];
    if (Object.prototype.hasOwnProperty.call(AAS_DEFAULTS, key)) return AAS_DEFAULTS[key];
    return null;
  }
  return null;
}


/**
 * Select changed fields from a draft relative to a baseline. Optional
 * keyList narrows the comparison so a caller (e.g. ConfirmSection) can
 * upload only a specific subset of fields rather than every drift it
 * has accumulated.
 *
 * keyList semantics:
 *   - undefined / not passed: compare every key in draft.
 *   - array (including empty): compare only the listed keys. An empty
 *     array therefore returns {} -- a "no valid keys" caller never
 *     accidentally uploads every pending draft field.
 *
 * Loose equality (`!=`) so numeric strings round-trip cleanly with
 * downloaded numeric values, matching the existing paramUpload helper.
 *
 * @param {Object} draft - Edited values, `{ key: value }`.
 * @param {Object} baseline - Last-known confirmed values, `{ key: value }`.
 * @param {Array<string>} [keyList] - Optional whitelist of keys to consider.
 * @returns {Object} `{ key: value }` of changed fields only.
 */
export function selectChangedFields(draft, baseline, keyList) {
  const out = {};
  if (!draft) return out;
  const base = baseline || {};
  const keys = Array.isArray(keyList) ? keyList : Object.keys(draft);
  for (const k of keys) {
    if (!Object.prototype.hasOwnProperty.call(draft, k)) continue;
    // eslint-disable-next-line eqeqeq
    if (draft[k] != base[k]) out[k] = draft[k];
  }
  return out;
}


/**
 * Merge per-field acks from a single PUT into that vehicle's confirmed
 * baseline. ONLY fields the autopilot acked (results[k] === true) are
 * advanced, and the merged value is taken from the `submitted` snapshot
 * -- the exact payload sent over the wire -- never from live draft state.
 *
 * Failed ack means the baseline is left unchanged for that field. This
 * helper does NOT touch draft state; reverting a draft to baseline after
 * a failed upload is the caller's (hook's) responsibility.
 *
 * @param {Object} baseline - Existing per-vehicle confirmed values.
 * @param {Object} submitted - The exact payload that was PUT.
 * @param {Object} results - Per-field bool ack map from the backend.
 * @returns {Object} New baseline (does not mutate inputs).
 */
export function mergeAckedFieldsIntoBaseline(baseline, submitted, results) {
  const next = { ...(baseline || {}) };
  if (!results) return next;
  for (const [k, ok] of Object.entries(results)) {
    if (ok === true && submitted && Object.prototype.hasOwnProperty.call(submitted, k)) {
      next[k] = submitted[k];
    }
  }
  return next;
}


/**
 * Aggregate per-vehicle PUT results into counters suitable for status
 * messages ("Updated 2/3 UAVs", "5 not confirmed").
 *
 * `submittedByVehicle` is required so the helper can distinguish a real
 * fanout failure (the network call itself returned nothing) from a
 * vehicle that simply had no changes to submit. A sys_id present in
 * submittedByVehicle but absent from perVehicleResults -- or whose
 * results payload is null/undefined -- counts every submitted key for
 * that vehicle as failed.
 *
 * @param {Object} perVehicleResults - `{ [sysId]: { key: bool } | null }`.
 * @param {Object} submittedByVehicle - `{ [sysId]: { key: value } }`.
 * @returns {{
 *   uavOk: number, uavFail: number,
 *   fieldOk: number, fieldFail: number,
 *   perKeyOk: Object, perKeyFail: Object,
 * }}
 */
export function aggregateFanoutResults(perVehicleResults, submittedByVehicle) {
  const pvr = perVehicleResults || {};
  const sub = submittedByVehicle || {};
  let uavOk = 0;
  let uavFail = 0;
  let fieldOk = 0;
  let fieldFail = 0;
  const perKeyOk = {};
  const perKeyFail = {};

  for (const sid of Object.keys(sub)) {
    const submittedKeys = Object.keys(sub[sid] || {});
    if (submittedKeys.length === 0) continue;

    const results = pvr[sid];
    if (!results) {
      uavFail += 1;
      for (const k of submittedKeys) {
        fieldFail += 1;
        perKeyFail[k] = (perKeyFail[k] || 0) + 1;
      }
      continue;
    }

    let anyFail = false;
    for (const k of submittedKeys) {
      if (results[k] === true) {
        fieldOk += 1;
        perKeyOk[k] = (perKeyOk[k] || 0) + 1;
      } else {
        fieldFail += 1;
        perKeyFail[k] = (perKeyFail[k] || 0) + 1;
        anyFail = true;
      }
    }
    if (anyFail) uavFail += 1;
    else uavOk += 1;
  }

  return { uavOk, uavFail, fieldOk, fieldFail, perKeyOk, perKeyFail };
}
