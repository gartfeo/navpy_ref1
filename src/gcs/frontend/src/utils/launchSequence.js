/**
 * Launch-sequence ordering helpers. Pure utilities — no React dependencies.
 */

/**
 * Order a list of sys_ids by a saved launch order.
 *
 * sys_ids listed in `launchOrder` come first, in that order; any sys_ids not
 * listed keep their original relative order at the end. Entries in
 * `launchOrder` that aren't currently present are ignored. An empty/missing
 * order returns the input order unchanged.
 *
 * @param {number[]} sysIds - currently-available sys_ids (in discovery order)
 * @param {number[]|null|undefined} launchOrder - saved preferred order
 * @returns {number[]} reordered sys_ids
 */
export function applyLaunchOrder(sysIds, launchOrder) {
  const ids = Array.isArray(sysIds) ? [...sysIds] : [];
  if (!Array.isArray(launchOrder) || launchOrder.length === 0) return ids;
  const present = new Set(ids);
  const ordered = [];
  const seen = new Set();
  for (const id of launchOrder) {
    if (present.has(id) && !seen.has(id)) {
      ordered.push(id);
      seen.add(id);
    }
  }
  const rest = ids.filter((id) => !seen.has(id));
  return [...ordered, ...rest];
}

/**
 * Order a list of vehicle objects (with `sys_id`) by a saved launch order.
 * @param {Array<{sys_id:number}>} vehicles
 * @param {number[]|null|undefined} launchOrder
 * @returns {Array<{sys_id:number}>}
 */
export function orderVehiclesByLaunch(vehicles, launchOrder) {
  const list = Array.isArray(vehicles) ? vehicles : [];
  const order = applyLaunchOrder(list.map((v) => v.sys_id), launchOrder);
  const bySysId = new Map(list.map((v) => [v.sys_id, v]));
  return order.map((id) => bySysId.get(id)).filter(Boolean);
}

/**
 * Select the authoritative launch roster.
 *
 * A container launch operates on the connected fleet: plan-zone count is a
 * mission-planning detail and must never hide a connected vehicle from the
 * physical launch sequence. Non-container launch keeps the existing
 * plan-sized roster behavior.
 *
 * @param {Array<{sys_id:number}>} vehicles - currently connected vehicles
 * @param {boolean} isContainer - whether container launch mode is active
 * @param {number} plannedCount - number of UAVs represented by plan zones
 * @returns {Array<{sys_id:number}>} selected launch vehicles
 */
export function selectLaunchVehicles(vehicles, isContainer, plannedCount) {
  const list = Array.isArray(vehicles) ? vehicles : [];
  if (isContainer) return [...list];
  const count = Math.max(0, Math.trunc(Number(plannedCount) || 0));
  return list.slice(0, count);
}

/**
 * Resolve the trigger order after the backend prepares a container session.
 * The backend-returned roster is authoritative; the requested roster is only
 * a compatibility fallback for older backends.
 */
export function resolvePreparedLaunchOrder(requestedSysIds, preparedSysIds, launchOrder) {
  const authoritative = Array.isArray(preparedSysIds)
    ? preparedSysIds
    : requestedSysIds;
  return applyLaunchOrder(authoritative, launchOrder);
}

/**
 * Resolve sys_id -> ESP32 channel, mirroring the backend `_resolve_channel_map`
 * (control.py): explicit mappings win; any vehicle without one is auto-assigned
 * the next free channel (1..N) in ascending sys_id order, skipping used
 * channels. Lets the UI show the channel an "Auto" vehicle will actually trigger.
 * @param {object|null|undefined} channelMap - explicit "sysId" -> channel
 * @param {number[]} sysIds - currently-present sys_ids
 * @param {number} [numChannels=6]
 * @returns {Record<number, number>} sys_id -> resolved channel
 */
export function resolveChannelMap(channelMap, sysIds, numChannels = 6) {
  const resolved = {};
  for (const [k, v] of Object.entries(channelMap || {})) {
    const ch = Number(v);
    if (v !== '' && v != null && Number.isFinite(ch)) resolved[Number(k)] = ch;
  }
  const used = new Set(Object.values(resolved));
  const free = [];
  for (let c = 1; c <= numChannels; c += 1) if (!used.has(c)) free.push(c);
  let fi = 0;
  for (const sid of [...(sysIds || [])].sort((a, b) => a - b)) {
    if (sid in resolved) continue;
    if (fi >= free.length) continue; // more vehicles than channels
    resolved[sid] = free[fi];
    fi += 1;
  }
  return resolved;
}
