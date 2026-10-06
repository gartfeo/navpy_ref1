import { decodeBitmaskArray } from '../../../utils/bitmask';

/**
 * True while a zone still holds the raw downloaded mission track — the corridor
 * prefix is part of `track`, marked by `corridor_end_index`.
 *
 * A mission download publishes the raw plan first and trims the prefix in a
 * later post-processing pass (usePlanPersistence.derivePlanPolygon), which also
 * resets the flag to 0. Between those two renders `corridorPointsArr` may
 * already hold the corridor from an earlier publication, so prepending it to a
 * raw track counts the corridor twice and shifts every resolved waypoint back
 * by corridorLen. Index a raw track directly; prepend only once it is trimmed.
 */
function isRawTrack(zone) {
  return (zone?.corridor_end_index ?? 0) > 0;
}

/**
 * Resolve simulation target positions from plan data and per-vehicle targ_wps.
 *
 * Waypoint ordering per vehicle mission:
 *   - Non-corridor: [...corridorPts, ...zoneTrack]
 *   - Corridor:     zoneTrack
 *   - Raw (untrimmed) zone track: zoneTrack, which already starts with the
 *     corridor — see isRawTrack.
 *
 * targ_wps bit (i-1) = WP i = wps[i-1] in the above array.
 *
 * @param {object|null} plan - Plan with zones[].track, zones[].set_index, zones[].sys_id
 * @param {Array<Array<{lat,lon}>>} corridorPointsArr - Corridor points per set
 * @param {string} searchPattern - "corridor" | "distributed"
 * @param {Object<number,number>} vehicleTargWps - { sys_id: bitmask } per-vehicle target WPs
 * @param {Array<{sys_id}>} vehicleList - Ordered vehicle list (zone i → vehicleList[i])
 * @returns {Array<{lat, lon, zoneIndex, wpNumber}>}
 */
export function resolveSimDocks(plan, corridorPointsArr, searchPattern, vehicleTargWps, vehicleList) {
  if (!plan?.zones?.length) return [];
  if (!vehicleTargWps || Object.keys(vehicleTargWps).length === 0) return [];

  const targets = [];

  for (let zi = 0; zi < plan.zones.length; zi++) {
    const zone = plan.zones[zi];
    const zoneTrack = zone.track || [];
    const setIdx = zone.set_index ?? 0;

    // Determine which vehicle owns this zone
    const sysId = zone.sys_id ?? (vehicleList || [])[zi]?.sys_id;
    if (sysId == null) continue;

    const bitmask = vehicleTargWps[sysId];
    if (!bitmask) continue;

    const wpNumbers = decodeBitmaskArray(bitmask);
    if (wpNumbers.length === 0) continue;

    let wps;
    if (searchPattern === 'corridor' || isRawTrack(zone)) {
      wps = zoneTrack;
    } else {
      const corridorPts = (corridorPointsArr || [])[setIdx] || [];
      wps = [...corridorPts, ...zoneTrack];
    }

    for (const wpNum of wpNumbers) {
      const idx = wpNum - 1;
      if (idx >= 0 && idx < wps.length) {
        const pt = wps[idx];
        targets.push({
          lat: pt.lat,
          lon: pt.lon ?? pt.lng,
          zoneIndex: zone.zone_index ?? 0,
          wpNumber: wpNum,
          sys_id: sysId,
        });
      }
    }
  }

  return targets;
}

/**
 * Resolve sim target positions from plan data and user-selected track waypoints
 * (planning mode — before upload, no vehicle bitmask needed).
 *
 * @param {object|null} plan - Plan with zones[].track
 * @param {Object<number, number[]>} simDockWps - { zoneIndex: [wpIndex, ...] } (0-based)
 * @returns {Array<{lat, lon, zoneIndex, wpNumber}>}
 */
export function resolveSimDocksFromPlan(plan, simDockWps) {
  if (!plan?.zones?.length || !simDockWps) return [];
  const targets = [];
  for (let zi = 0; zi < plan.zones.length; zi++) {
    const wpIndices = simDockWps[zi];
    if (!wpIndices?.length) continue;
    const track = plan.zones[zi].track || [];
    for (const wi of wpIndices) {
      if (wi >= 0 && wi < track.length) {
        targets.push({
          lat: track[wi].lat,
          lon: track[wi].lon ?? track[wi].lng,
          zoneIndex: plan.zones[zi].zone_index ?? zi,
          wpNumber: wi + 1,
        });
      }
    }
  }
  return targets;
}

/**
 * Resolve detection start positions from plan data and user-selected detect-after waypoints
 * (planning mode — before upload, no vehicle params needed).
 *
 * @param {object|null} plan - Plan with zones[].track
 * @param {Object<number, number>} detectAfterWps - { zoneIndex: wpIndex } (0-based track index)
 * @returns {Array<{lat, lon, zoneIndex, wpNumber}>}
 */
export function resolveDetectionStartFromPlan(plan, detectAfterWps) {
  if (!plan?.zones?.length || !detectAfterWps) return [];
  const points = [];
  for (let zi = 0; zi < plan.zones.length; zi++) {
    const wi = detectAfterWps[zi];
    if (wi == null) continue;
    const track = plan.zones[zi].track || [];
    if (wi >= 0 && wi < track.length) {
      points.push({
        lat: track[wi].lat,
        lon: track[wi].lon ?? track[wi].lng,
        alt: plan.zones[zi].altitude_m || plan.altitude_m || 150,
        zoneIndex: plan.zones[zi].zone_index ?? zi,
        wpNumber: wi + 1,
      });
    }
  }
  return points;
}

/**
 * Resolve the first detection waypoint per zone.
 * Detection starts at WP (nav_last_wp + 1), i.e. the waypoint AFTER the last
 * navigation waypoint.
 *
 * @param {object|null} plan - Plan with zones[].track, zones[].set_index, zones[].sys_id
 * @param {Array<Array<{lat,lon}>>} corridorPointsArr - Corridor points per set
 * @param {string} searchPattern - "corridor" | "distributed"
 * @param {Object<number,number>} vehicleNavLastWp - { sys_id: nav_last_wp } per-vehicle
 * @param {Array<{sys_id}>} vehicleList - Ordered vehicle list (zone i → vehicleList[i])
 * @returns {Array<{lat, lon, zoneIndex, wpNumber}>}
 */
export function resolveDetectionStart(plan, corridorPointsArr, searchPattern, vehicleNavLastWp, vehicleList) {
  if (!plan?.zones?.length) return [];
  if (!vehicleNavLastWp || Object.keys(vehicleNavLastWp).length === 0) return [];

  const points = [];

  for (let zi = 0; zi < plan.zones.length; zi++) {
    const zone = plan.zones[zi];
    const zoneTrack = zone.track || [];
    const setIdx = zone.set_index ?? 0;

    const sysId = zone.sys_id ?? (vehicleList || [])[zi]?.sys_id;
    if (sysId == null) continue;

    const navLast = vehicleNavLastWp[sysId];
    if (!navLast || navLast <= 0) continue;

    // Detection starts at WP navLast, which is index (navLast - 1) in 0-based array
    const detIdx = navLast - 1;

    let wps;
    if (searchPattern === 'corridor' || isRawTrack(zone)) {
      wps = zoneTrack;
    } else {
      const corridorPts = (corridorPointsArr || [])[setIdx] || [];
      wps = [...corridorPts, ...zoneTrack];
    }

    if (detIdx >= 0 && detIdx < wps.length) {
      const pt = wps[detIdx];
      points.push({
        lat: pt.lat,
        lon: pt.lon ?? pt.lng,
        alt: zone.altitude_m || plan.altitude_m || 150,
        zoneIndex: zone.zone_index ?? 0,
        wpNumber: navLast,
      });
    }
  }

  return points;
}
