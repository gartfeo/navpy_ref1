// Group connected UAVs into launch containers. A container physically holds a
// fixed number of UAVs (UAVS_PER_CONTAINER); vehicles are assigned by ascending
// sys_id, so Container 1 holds the lowest sys_ids, Container 2 the next, etc.
//
// This is the single source of truth for container membership — the UAV Status
// display and (once wired) the staggered launch both group from it.
//
// Each vehicle keeps its GLOBAL index (position in the original vehicleList) so
// per-vehicle data indexed by list position (zone distances, colours) stays
// aligned regardless of the sys_id sort.
//
// Returns: [{ container, items: [{ v, index }] }]  (container is 1-based)
// Tested via tests/gcs/test_containers_js.py.

export const UAVS_PER_CONTAINER = 3;

// Chunk a flat list of sys_ids into containers by ascending sys_id — the launch
// order that matches the container grouping shown in UAV Status. Returns an
// array of sys_id arrays: [[c1...], [c2...], ...].
export function containersFromSysIds(sysIds, perContainer = UAVS_PER_CONTAINER) {
  const sorted = [...(sysIds || [])].sort((a, b) => a - b);
  const size = perContainer > 0 ? perContainer : 1;
  const out = [];
  for (let i = 0; i < sorted.length; i += size) out.push(sorted.slice(i, i + size));
  return out;
}

export function groupVehiclesByContainer(vehicleList, perContainer = UAVS_PER_CONTAINER) {
  const withIndex = (vehicleList || []).map((v, index) => ({ v, index }));
  // Assign containers by ascending sys_id (deterministic, independent of the
  // order telemetry happened to arrive in).
  withIndex.sort((a, b) => (a.v && a.v.sys_id != null ? a.v.sys_id : 0)
                         - (b.v && b.v.sys_id != null ? b.v.sys_id : 0));
  const size = perContainer > 0 ? perContainer : 1;
  const groups = [];
  withIndex.forEach((item, i) => {
    const container = Math.floor(i / size) + 1;
    let grp = groups[container - 1];
    if (!grp) {
      grp = { container, items: [] };
      groups[container - 1] = grp;
    }
    grp.items.push(item);
  });
  return groups;
}
