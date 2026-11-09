/**
 * The static sim docks to draw: while planning only, and only the docks this
 * plan picks (simDockWps). A UAV's stored dock parameter belongs to the mission
 * on that UAV, not to the plan in the editor: drawn onto a new plan it shows
 * docks that do not exist. A downloaded plan already carries the UAVs' docks
 * (the download turns their parameters into simDockWps). In monitoring the map
 * shows only what the swarm knows (dock location rule), so no dock shows from
 * waypoint data before a detection or peer message.
 */
export function plannedDocksToShow(phase, plan, simDockWps) {
  return phase === 'PLANNING' ? resolveSimDocksFromPlan(plan, simDockWps) : [];
}

/**
 * The detection start markers to draw. The UAVs' own nav_last_wp parameters
 * describe the plan on the UAVs, so they are drawn only while the map shows
 * that plan (monitoring a synced plan). Otherwise the plan's own selection
 * (detectAfterWps) is drawn.
 */
export function detectionStartsToShow({
  phase, planOnVehicles, plan, detectAfterWps,
  corridorPointsArr, searchPattern, vehicleNavLastWp, vehicleList,
}) {
  if (phase === 'PLANNING' || !planOnVehicles) return resolveDetectionStartFromPlan(plan, detectAfterWps);
  return resolveDetectionStart(plan, corridorPointsArr, searchPattern, vehicleNavLastWp, vehicleList);
}

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
 * The zone's mission waypoints in vehicle order (targ_wps bit (i-1) = WP i =
 * wps[i-1]): the set's corridor then the track, or the track alone for
 * corridor search and for a raw downloaded track (see isRawTrack).
 */
export function zoneMissionWaypoints(zone, corridorPointsArr, searchPattern) {
  const zoneTrack = zone?.track || [];
  if (searchPattern === 'corridor' || isRawTrack(zone)) return zoneTrack;
  const corridorPts = (corridorPointsArr || [])[zone?.set_index ?? 0] || [];
  return [...corridorPts, ...zoneTrack];
}

/**
 * Resolve sim POI positions from plan data and user-selected track waypoints
 * (planning mode — before upload, no vehicle bitmask needed).
 *
 * @param {object|null} plan - Plan with zones[].track
 * @param {Object<number, number[]>} simDockWps - { zoneIndex: [wpIndex, ...] } (0-based)
 * @returns {Array<{lat, lon, zoneIndex, wpNumber}>}
 */
export function resolveSimDocksFromPlan(plan, simDockWps) {
  if (!plan?.zones?.length || !simDockWps) return [];
  const pois = [];
  for (let zi = 0; zi < plan.zones.length; zi++) {
    const wpIndices = simDockWps[zi];
    if (!wpIndices?.length) continue;
    const track = plan.zones[zi].track || [];
    for (const wi of wpIndices) {
      if (wi >= 0 && wi < track.length) {
        pois.push({
          lat: track[wi].lat,
          lon: track[wi].lon ?? track[wi].lng,
          zoneIndex: plan.zones[zi].zone_index ?? zi,
          wpNumber: wi + 1,
        });
      }
    }
  }
  return pois;
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

    const sysId = zone.sys_id ?? (vehicleList || [])[zi]?.sys_id;
    if (sysId == null) continue;

    const navLast = vehicleNavLastWp[sysId];
    if (!navLast || navLast <= 0) continue;

    // Detection starts at WP navLast, which is index (navLast - 1) in 0-based array
    const detIdx = navLast - 1;

    const wps = zoneMissionWaypoints(zone, corridorPointsArr, searchPattern);

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
