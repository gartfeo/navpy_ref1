/**
 * Reindex zone-keyed maps (simDockWps, detectAfterWps) after zones are
 * removed.  keptOldIndices is an ordered list of old zone indices that
 * survived; position in the array is the new index.
 *
 * @param {Object} map       – e.g. { 0: [1,3], 1: [2], 2: [0] }
 * @param {number[]} keptOldIndices – e.g. [0, 2]  (zone 1 was removed)
 * @param {'array'|'scalar'} mode – 'array' keeps non-empty arrays,
 *                                   'scalar' keeps non-null scalars
 * @returns {Object} reindexed map – e.g. { 0: [1,3], 1: [0] }
 */
export function reindexZoneMap(map, keptOldIndices, mode = 'array') {
  const out = {};
  for (let newIdx = 0; newIdx < keptOldIndices.length; newIdx++) {
    const oldIdx = keptOldIndices[newIdx];
    const val = map[oldIdx];
    if (mode === 'array') {
      if (val?.length > 0) out[newIdx] = val;
    } else {
      if (val != null) out[newIdx] = val;
    }
  }
  return out;
}

/**
 * Build targ_wps bitmask updates for all zones.
 * Always produces an entry for every zone (including bitmask 0) so that
 * previously set POIs are explicitly cleared on the vehicle.
 *
 * @param {Object|null} simDockWps – { zoneIndex: [trackWpIndex, ...] }
 * @param {Object[]} zones          – plan zones (need set_index)
 * @param {Object[]} vehicles       – [{ sys_id }, ...]
 * @param {boolean}  isCorridor     – corridor search pattern flag
 * @param {number[]} corridorLens   – per-zone corridor lengths
 * @returns {{ updates: Object, calls: Array }}
 */
export function buildTargWpsUpdates(simDockWps, zones, vehicles, isCorridor, corridorLens) {
  const updates = {};
  const calls = [];
  for (let i = 0; i < zones.length; i++) {
    const v = vehicles[i];
    if (!v) continue;
    const corridorLen = isCorridor ? 0 : (corridorLens[i] || 0);
    let bitmask = 0;
    for (const wi of ((simDockWps || {})[i] || [])) {
      const wpNum = corridorLen + wi + 1;
      bitmask |= (1 << (wpNum - 1));
    }
    updates[v.sys_id] = bitmask;
    calls.push({ sys_id: v.sys_id, targ_wps: bitmask });
  }
  return { updates, calls };
}

/**
 * Build nav_last_wp updates for all zones.
 * When detectAfterWps has no entry for a zone, produces 0 to clear
 * any previously set value on the vehicle.
 *
 * @param {Object|null} detectAfterWps – { zoneIndex: trackWpIndex }
 * @param {Object[]} zones             – plan zones
 * @param {Object[]} vehicles          – [{ sys_id }, ...]
 * @param {boolean}  isCorridor        – corridor search pattern flag
 * @param {number[]} corridorLens      – per-zone corridor lengths
 * @returns {{ updates: Object, calls: Array }}
 */
export function buildNavLastWpUpdates(detectAfterWps, zones, vehicles, isCorridor, corridorLens) {
  const updates = {};
  const calls = [];
  for (let i = 0; i < zones.length; i++) {
    const v = vehicles[i];
    if (!v) continue;
    const wi = (detectAfterWps || {})[i];
    let navLastWp = 0;
    if (wi != null) {
      const corridorLen = isCorridor ? 0 : (corridorLens[i] || 0);
      navLastWp = corridorLen + wi + 1;
    }
    updates[v.sys_id] = navLastWp;
    calls.push({ sys_id: v.sys_id, nav_last_wp: navLastWp });
  }
  return { updates, calls };
}
