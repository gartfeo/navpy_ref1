/**
 * Assemble downloaded mission results into a plan structure.
 * Filters errors, builds zones and metadata from successful downloads.
 *
 * @param {Array} missionResults - Array of { waypoints, sys_id, error?, ... }
 * @returns {{ zones, missionFallbackLocations, altitude, searchPattern, polygon, corridorBackbone, launchPoint, dockClasses } | null}
 */
export function assembleMissionsFromResults(missionResults) {
  const missions = [];
  for (const r of missionResults) {
    if (r.error) {
      console.warn(`Download failed for vehicle ${r.sys_id}: ${r.error}`);
    } else {
      missions.push(r);
    }
  }
  const zones = [];
  const missionFallbackLocations = [];
  let altitude = 100;
  let searchPattern = 'distributed';
  let polygon = null;
  let corridorBackbone = null;
  let launchPoint = null;
  let dockClasses = null;
  for (const m of missions) {
    if (m?.waypoints?.length) {
      zones.push({
        zone_index: zones.length,
        polygon: [],
        track: m.waypoints.map((wp) => ({ lat: wp.lat, lon: wp.lon, ...(wp.alt != null && { alt: wp.alt }) })),
        sys_id: m.sys_id,
        corridor_end_index: m.corridor_end_index ?? 0,
        altitude_m: m.altitude_m,
      });
      missionFallbackLocations.push(m.fallback_delivery_location || null);
      altitude = m.altitude_m || altitude;
      if (m.search_pattern) searchPattern = m.search_pattern;
      if (!polygon && m.polygon?.length >= 3) polygon = m.polygon;
      if (!corridorBackbone && m.corridor_backbone?.length > 0) corridorBackbone = m.corridor_backbone;
      if (!launchPoint && m.launch_point) launchPoint = m.launch_point;
      if (!dockClasses && m.dock_classes?.length > 0) dockClasses = m.dock_classes;
    }
  }
  if (zones.length === 0) return null;
  return { zones, missionFallbackLocations, altitude, searchPattern, polygon, corridorBackbone, launchPoint, dockClasses };
}

/**
 * Merge a single downloaded mission into an existing plan's zones.
 *
 * Replaces any existing zone for the same vehicle (idempotent re-download) and
 * appends otherwise, then re-indexes zone_index. Pure: callers thread the
 * result back through their plan ref synchronously so concurrent single-vehicle
 * downloads accumulate instead of racing on a stale read (last-write-wins would
 * silently drop a UAV's zone).
 *
 * @param {Array} existingZones - current plan.zones (may be undefined)
 * @param {object} mission - a downloaded mission ({ waypoints, corridor_end_index, altitude_m })
 * @param {number} sysId - the vehicle the mission belongs to
 * @returns {Array} the new zones array
 */
export function mergeDownloadedMissionZones(existingZones, mission, sysId) {
  const track = (mission?.waypoints || []).map((wp) => ({
    lat: wp.lat, lon: wp.lon, ...(wp.alt != null && { alt: wp.alt }),
  }));
  const newZone = {
    zone_index: 0,
    polygon: [],
    track,
    sys_id: sysId,
    corridor_end_index: mission?.corridor_end_index ?? 0,
    altitude_m: mission?.altitude_m,
  };
  const kept = (existingZones || []).filter((z) => z.sys_id !== sysId);
  return [...kept, newZone].map((z, i) => ({ ...z, zone_index: i }));
}
