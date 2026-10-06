/**
 * Pure helpers for the AAS parameter upload flow. Kept framework-free so
 * they can be unit-tested directly from Node.
 */

/**
 * Select the changed-value subset of a vehicle's params relative to its
 * downloaded baseline. Loose equality so numeric strings round-trip cleanly.
 *
 * @param {Object} vp - Current vehicle params (edited values).
 * @param {Object} dl - Downloaded baseline params.
 * @returns {Object} Map of changed keys → current values.
 */
export function selectChangedParams(vp, dl) {
  const out = {};
  if (!vp) return out;
  const baseline = dl || {};
  for (const k of Object.keys(vp)) {
    // eslint-disable-next-line eqeqeq
    if (vp[k] != baseline[k]) out[k] = vp[k];
  }
  return out;
}

/**
 * Apply per-field MAVLink acks to a baseline. Only fields the autopilot
 * confirmed are merged in, and the merged value is taken from the
 * `submitted` snapshot (what was sent over the wire) — NOT from the live
 * edit state — so a re-edit while the upload is in flight cannot be
 * silently baselined by an ack for the older value.
 *
 * @param {Object} baseline - Existing per-vehicle baseline (downloadedRef[sid]).
 * @param {Object} submitted - The exact payload that was PUT to the backend.
 * @param {Object} results - Per-field bool ack map from the backend.
 * @returns {Object} New baseline (does not mutate inputs).
 */
export function mergeUploadResults(baseline, submitted, results) {
  const next = { ...(baseline || {}) };
  if (!results) return next;
  for (const [k, ok] of Object.entries(results)) {
    if (ok === true && submitted && Object.prototype.hasOwnProperty.call(submitted, k)) {
      next[k] = submitted[k];
    }
  }
  return next;
}
