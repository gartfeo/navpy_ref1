import { autoAssignDeliveryHubs } from './deliveryHubAssignment';

/**
 * Build the per-zone label "S{set+1}U{nthZoneInSet}" used in
 * "missing delivery hub" upload messages. Mirrors the labeling that used to live
 * inline in useMissionUpload.handleUpload.
 *
 * @param {Array<object>} zones - plan zones (each may carry .set_index)
 * @param {number} i - zone index
 * @returns {string}
 */
function zoneLabel(zones, i) {
  const si = zones[i]?.set_index ?? 0;
  let uavNum = 0;
  for (let j = 0; j <= i; j++) {
    if ((zones[j]?.set_index ?? 0) === si) uavNum++;
  }
  return `S${si + 1}U${uavNum}`;
}

/**
 * Resolve the delivery hub assignment to upload for each zone — synchronously, from the
 * plan that is actually being uploaded, never from render-stale React state.
 *
 * This is the core of the first-click upload fix: handleUpload used to validate
 * and build its payload from the closure-captured `deliveryHubAssignments`, which lags
 * the plan that `localGenerate` (re)produces in the same handler. Deriving the
 * assignment here from the active plan removes that dependency on a setState
 * having committed.
 *
 * Which assignment "wins" for a zone:
 * - Manual mode (`manualDeliveryHubEdit`): the user's current assignment is authoritative
 *   and preserved — but normalized to the zone count and validated; an index that
 *   is null or out of range (`>= deliveryHubs.length`, or negative) is treated as missing
 *   AND nulled so a stale index can't produce a `default_delivery_hub: null`.
 * - Auto mode, plan NOT regenerated this upload (`regenerated === false`) and the
 *   current assignments are complete+valid: preserve them. This keeps assignments
 *   restored from a downloaded plan's `default_delivery_hub` metadata (or a prior
 *   auto-assign) — "what you see is what you upload" — rather than recomputing and
 *   possibly diverging.
 * - Auto mode otherwise (freshly regenerated plan, or current assignments
 *   missing/incomplete): (re)derive from the plan via `autoAssignDeliveryHubs` so the
 *   assignments match the zones actually being uploaded (the first-click fix).
 *
 * A zone whose resolved assignment is null (e.g. a track-less zone, for which
 * `autoAssignDeliveryHubs` cannot infer a destination) is reported in `missingLabels` rather
 * than silently uploaded without a delivery hub.
 *
 * @param {Array<object>} zones - plan zones (with .track, optional .set_index)
 * @param {Array<object>} deliveryHubs - default delivery hubs
 * @param {Array<number|null>|undefined} currentAssignments - current per-zone delivery hub indices
 * @param {boolean} manualDeliveryHubEdit - whether the user has manually edited assignments
 * @param {boolean} regenerated - whether the plan was just regenerated for this upload
 *   (true => current assignments may not match the new zones, so recompute)
 * @returns {{ assignments: Array<number|null>, missingLabels: string[] }}
 */
export function resolveUploadAssignments(zones, deliveryHubs, currentAssignments, manualDeliveryHubEdit, regenerated) {
  const n = zones?.length || 0;
  const m = deliveryHubs?.length || 0;
  const assignments = new Array(n).fill(null);

  if (n === 0) return { assignments, missingLabels: [] };

  // Normalize current assignments to in-range-or-null for these zones.
  const cur = new Array(n).fill(null);
  for (let i = 0; i < n; i++) {
    const idx = currentAssignments?.[i];
    if (idx != null && idx >= 0 && idx < m) cur[i] = idx;
  }
  const curComplete = currentAssignments?.length === n && cur.every((x) => x != null);

  let source;
  if (manualDeliveryHubEdit) {
    source = cur; // manual edits are authoritative (already validated into cur)
  } else if (!regenerated && curComplete) {
    source = cur; // existing plan: keep the displayed/downloaded assignments
  } else {
    source = autoAssignDeliveryHubs(zones, deliveryHubs); // regenerated, or no usable current -> (re)derive
  }

  for (let i = 0; i < n; i++) {
    const idx = source?.[i];
    // Keep only valid in-range indices; null / out-of-range -> unassigned.
    if (idx != null && idx >= 0 && idx < m) assignments[i] = idx;
  }

  const missingLabels = [];
  for (let i = 0; i < n; i++) {
    if (assignments[i] == null) missingLabels.push(zoneLabel(zones, i));
  }
  return { assignments, missingLabels };
}
