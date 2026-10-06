/**
 * Compute per-vehicle differences for a parameter key.
 *
 * @param {string} key          - The param key to check (e.g. "del_pitch")
 * @param {*} editValue         - Current edit value in the draft
 * @param {Object} vehicleParams - { sysId: { key: value, ... }, ... }
 * @param {number[]} selectedIds - Array of selected sys_ids
 * @param {Object} [idToIndex]  - Optional { sysId: colorIndex } map
 * @returns {Array<{label: string, value: *, colorIdx: number}>|undefined}
 *   Array of {label, value, colorIdx} for vehicles whose value differs from
 *   editValue, or undefined if fewer than 2 selected or no differences found.
 */
export function computeParamDiffs(key, editValue, vehicleParams, selectedIds, idToIndex) {
  if (selectedIds.length < 2) return undefined;
  const entries = [];
  for (const sid of selectedIds) {
    const vp = vehicleParams[sid];
    if (!vp || vp[key] === undefined) continue;
    // eslint-disable-next-line eqeqeq
    if (vp[key] != editValue) {
      entries.push({
        label: `UAV ${sid}`,
        value: vp[key],
        colorIdx: idToIndex ? (idToIndex[sid] ?? 0) : 0,
      });
    }
  }
  return entries.length > 0 ? entries : undefined;
}
