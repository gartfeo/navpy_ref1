/**
 * Client-side planning algorithms — JS port of coverage.py.
 * Orchestrator: delegates to focused sub-modules.
 */

// Re-export public APIs from sub-modules for backward compatibility
export { configurePlanner, getCameraConfig, getConfig, computeSets, getUavsPerSet, setDeviceConfigs, getDeviceConfigs, clearDeviceConfigs, computeMaxDetectDist, setTrackSpacingOverride } from './plannerConfig.js';
export { R, DEG2RAD, RAD2DEG, toMeters, toLatLon, polygonToMeters, polygonToLatLon } from './projection.js';
export { polygonAreaM2, polygonCentroid, insetPolygon, longestEdgeAngle, lineIntersect, offsetPolyline } from './geometry.js';
export { clipPolygonByLine, clipPolygonByStrip, projectAlongPerp, partitionPolygon } from './clipping.js';
export { computeTrackSpacing, generateZigzagTrack, generateStrips, assignStripsEqualDistance, trackDistance } from './trackGenerator.js';
export { computeLaunchZone } from './launchZone.js';

import { getConfig } from './plannerConfig.js';
import { DEG2RAD } from './projection.js';
import { toMeters, toLatLon, polygonToMeters, polygonToLatLon } from './projection.js';
import { polygonAreaM2, polygonCentroid, longestEdgeAngle, offsetPolyline } from './geometry.js';
import { clipPolygonByStrip, partitionPolygon, projectAlongPerp } from './clipping.js';
import { computeTrackSpacing, generateZigzagTrack, generateStrips, assignStripsEqualDistance, trackDistance } from './trackGenerator.js';
import { computeLaunchZone } from './launchZone.js';

// ---- Helpers ----

function _round(v, decimals) {
  const f = 10 ** decimals;
  return Math.round(v * f) / f;
}

function _orientTrackToLaunchPerp(track, lpM, partitionAngle) {
  if (!lpM || track.length < 2) return;
  const perpX = -Math.sin(partitionAngle), perpY = Math.cos(partitionAngle);
  const lpPerp = lpM[0] * perpX + lpM[1] * perpY;
  const fmx = (track[0][0] + track[1][0]) / 2;
  const fmy = (track[0][1] + track[1][1]) / 2;
  const lmx = (track[track.length - 2][0] + track[track.length - 1][0]) / 2;
  const lmy = (track[track.length - 2][1] + track[track.length - 1][1]) / 2;
  if (Math.abs(lpPerp - (lmx * perpX + lmy * perpY)) <
      Math.abs(lpPerp - (fmx * perpX + fmy * perpY))) {
    track.reverse();
  }
  const d0 = Math.hypot(track[0][0] - lpM[0], track[0][1] - lpM[1]);
  const d1 = Math.hypot(track[1][0] - lpM[0], track[1][1] - lpM[1]);
  if (d1 < d0) {
    for (let j = 0; j < track.length - 1; j += 2) {
      [track[j], track[j + 1]] = [track[j + 1], track[j]];
    }
  }
}

// ---- Public helpers ----

export function autoPartitionAngle(polygonObj) {
  const polygonLL = polygonObj.map((p) => [p.lat, p.lon]);
  const [ptsM] = polygonToMeters(polygonLL);
  return longestEdgeAngle(ptsM);
}

// ---- High-level API ----

/**
 * Analyze polygon for mission planning stats.
 */
export function analyzeArea(polygonObj, dockClasses, uavCount) {
  const cfg = getConfig();
  const polygonLL = polygonObj.map((p) => [p.lat, p.lon]);
  const [ptsM, refLat] = polygonToMeters(polygonLL);
  const areaKm2 = polygonAreaM2(ptsM) / 1e6;

  const [spacing] = computeTrackSpacing(dockClasses, cfg);
  const angleDeg = longestEdgeAngle(ptsM);
  const angleRad = angleDeg * DEG2RAD;

  const [track, stripCount] = generateZigzagTrack(ptsM, spacing, angleRad);
  const totalDistM = trackDistance(track);
  const totalDistKm = totalDistM / 1000;
  const estTimeMin = (totalDistM / cfg.CRUISE_SPEED_MS) / 60;

  const minUavs = totalDistKm <= 0 ? 1 : Math.max(1, Math.ceil(totalDistKm / cfg.USABLE_FLIGHT_KM));
  const sets = Math.max(1, Math.ceil(minUavs / cfg.UAVS_PER_SET));
  const requiredUavs = sets * cfg.UAVS_PER_SET;

  const bufferUavs = uavCount || requiredUavs;
  const perUavDistKm = bufferUavs > 0 ? totalDistKm / bufferUavs : 0;
  const remainingKm = Math.max(cfg.MIN_LAUNCH_ZONE_BUFFER_KM, cfg.FLIGHT_BUDGET_KM - cfg.SAFETY_RESERVE_KM - perUavDistKm);
  const [lzPts, lzBuffer] = computeLaunchZone(polygonLL, remainingKm * 1000);

  const minLzBufferM = cfg.MIN_LAUNCH_ZONE_BUFFER_KM * 1000;
  const minLzOutwardM = Math.max(100, lzBuffer - minLzBufferM);
  const [minLzPts] = computeLaunchZone(polygonLL, minLzOutwardM);

  return {
    area_km2: _round(areaKm2, 2),
    min_uavs: minUavs,
    max_uavs: Math.max(1, stripCount),
    required_uavs: requiredUavs,
    sets,
    track_spacing_m: _round(spacing, 1),
    strip_count: stripCount,
    total_distance_km: _round(totalDistKm, 1),
    estimated_time_min: _round(estTimeMin, 1),
    grid_angle_deg: _round(angleDeg, 1),
    launch_zone: lzPts.map((p) => ({ lat: _round(p[0], 6), lon: _round(p[1], 6) })),
    launch_zone_buffer_m: Math.round(lzBuffer),
    min_launch_zone: minLzPts.map((p) => ({ lat: _round(p[0], 6), lon: _round(p[1], 6) })),
    min_launch_zone_buffer_m: Math.round(minLzOutwardM),
  };
}

// ---- Search pattern handlers ----

function generateCorridorPlan(polygonLL, spacing, altitude, uavCount, lp, corridorWaypointsObj, cfg) {
  const cwObj = corridorWaypointsObj || [];
  if (cwObj.length < 2) {
    const minLzBufferM = cfg.MIN_LAUNCH_ZONE_BUFFER_KM * 1000;
    const [minLzPts] = computeLaunchZone(polygonLL, minLzBufferM);
    return {
      zones: [], all_feasible: true, total_distance_km: 0, strip_count: 0,
      estimated_time_min: 0, grid_angle_deg: 0,
      altitude_m: _round(altitude, 1), altitude_separation_m: cfg.ALTITUDE_SEPARATION_M,
      launch_zone: [], launch_zone_buffer_m: 0,
      min_launch_zone: minLzPts.map((p) => ({ lat: _round(p[0], 6), lon: _round(p[1], 6) })),
      min_launch_zone_buffer_m: Math.round(minLzBufferM),
    };
  }
  const cw = cwObj.map((p) => [p.lat, p.lon]);
  const cwRefLat = cw.reduce((s, p) => s + p[0], 0) / cw.length;
  const cwM = cw.map((p) => toMeters(p[0], p[1], cwRefLat));
  const zones = [];
  const n = Math.max(1, uavCount);
  for (let i = 0; i < n; i++) {
    const off = (i - (n - 1) / 2) * spacing;
    let trackM;
    if (Math.abs(off) < 1e-9) {
      trackM = [...cwM];
    } else {
      trackM = offsetPolyline(cwM, Math.abs(off), off >= 0 ? 1 : -1);
    }
    const trackLL = trackM.map((p) => toLatLon(p[0], p[1], cwRefLat));
    const distM = trackDistance(trackM);
    const distKm = distM / 1000;
    zones.push({
      zone_index: i, set_index: 0, polygon: [],
      track: trackLL.map((p) => ({ lat: p[0], lon: p[1] })),
      total_distance_km: _round(distKm, 1), strip_count: 0,
      feasible: distKm <= cfg.USABLE_FLIGHT_KM,
      estimated_time_min: _round(distM > 0 ? (distM / cfg.CRUISE_SPEED_MS) / 60 : 0, 1),
    });
  }
  zones.forEach((z, i) => {
    z.altitude_m = _round(altitude + (zones.length - 1 - i) * cfg.ALTITUDE_SEPARATION_M, 1);
  });
  const minLzBufferM = cfg.MIN_LAUNCH_ZONE_BUFFER_KM * 1000;
  const [minLzPts] = computeLaunchZone(polygonLL, minLzBufferM);
  return {
    zones, all_feasible: zones.every((z) => z.feasible),
    total_distance_km: _round(zones.reduce((s, z) => s + z.total_distance_km, 0), 1),
    strip_count: 0,
    estimated_time_min: _round(Math.max(...zones.map((z) => z.estimated_time_min), 0), 1),
    grid_angle_deg: 0,
    altitude_m: _round(altitude, 1), altitude_separation_m: cfg.ALTITUDE_SEPARATION_M,
    launch_zone: [], launch_zone_buffer_m: 0,
    min_launch_zone: minLzPts.map((p) => ({ lat: _round(p[0], 6), lon: _round(p[1], 6) })),
    min_launch_zone_buffer_m: Math.round(minLzBufferM),
  };
}

function generateDistributedPlan(ptsM, refLat, polygonLL, spacing, altitude, angleRad, lpM, uavCount, setLaunchPoints, cfg) {
  const numSets = Math.max(1, Math.ceil(uavCount / cfg.UAVS_PER_SET));
  const subPolygons = partitionPolygon(ptsM, numSets, angleRad);

  const zones = [];
  let globalZoneIdx = 0;
  for (let si = 0; si < numSets; si++) {
    const setPoly = subPolygons[si] || subPolygons[subPolygons.length - 1];

    const setSlp = setLaunchPoints?.[si];
    const setLpM = setSlp ? toMeters(setSlp.lat, setSlp.lon, refLat) : null;
    let setAngle;
    if (setLpM) {
      const [cx, cy] = polygonCentroid(setPoly);
      setAngle = Math.atan2(setLpM[1] - cy, setLpM[0] - cx);
    } else {
      setAngle = angleRad;
    }

    const setStrips = generateStrips(setPoly, spacing, setAngle);
    if (!setStrips.length) {
      const zoneLL = polygonToLatLon(setPoly, refLat);
      zones.push({
        zone_index: globalZoneIdx++, set_index: si,
        polygon: zoneLL.map((p) => ({ lat: p[0], lon: p[1] })),
        track: [], total_distance_km: 0, strip_count: 0,
        feasible: true, estimated_time_min: 0,
      });
      continue;
    }

    const zonesInSet = si < numSets - 1 ? cfg.UAVS_PER_SET : uavCount - si * cfg.UAVS_PER_SET;
    const groups = assignStripsEqualDistance(setStrips, zonesInSet);
    const perpX = -Math.sin(setAngle), perpY = Math.cos(setAngle);
    const stripPerps = setStrips.map((strip) => {
      const mx = strip.reduce((s, p) => s + p[0], 0) / strip.length;
      const my = strip.reduce((s, p) => s + p[1], 0) / strip.length;
      return mx * perpX + my * perpY;
    });
    // Zone borders tile the set polygon exactly: split adjacent zones at the
    // midpoint between their edge lanes, and run the outermost zones out to
    // the area border itself — no spacing-derived padding.
    const setProjs = projectAlongPerp(setPoly, setAngle);
    const setPerpMin = Math.min(...setProjs), setPerpMax = Math.max(...setProjs);

    for (const [s, e] of groups) {
      const track = [];
      for (let k = s; k < e; k++) {
        const pts = [...setStrips[k]];
        if ((k - s) % 2 === 0) pts.reverse();
        track.push(...pts);
      }

      const zonePerpMin = s === 0
        ? setPerpMin : (stripPerps[s - 1] + stripPerps[s]) / 2;
      const zonePerpMax = e === setStrips.length
        ? setPerpMax : (stripPerps[e - 1] + stripPerps[e]) / 2;
      const zoneM = clipPolygonByStrip(setPoly, setAngle, zonePerpMin, zonePerpMax);

      const distM = trackDistance(track);
      const distKm = distM / 1000;
      const trackLL = polygonToLatLon(track, refLat);
      const zoneLL = zoneM.length ? polygonToLatLon(zoneM, refLat) : [];
      zones.push({
        zone_index: globalZoneIdx++, set_index: si,
        polygon: zoneLL.map((p) => ({ lat: p[0], lon: p[1] })),
        track: trackLL.map((p) => ({ lat: p[0], lon: p[1] })),
        total_distance_km: _round(distKm, 1), strip_count: e - s,
        feasible: distKm <= cfg.USABLE_FLIGHT_KM,
        estimated_time_min: _round((distM / cfg.CRUISE_SPEED_MS) / 60, 1),
      });
    }
  }
  return zones;
}

// ---- Search pattern dispatch ----

const SEARCH_PATTERN_HANDLERS = {
  corridor: generateCorridorPlan,
  distributed: generateDistributedPlan,
};

/**
 * Generate full mission plan with zones and tracks.
 */
export function generatePlan(polygonObj, dockClasses, searchPattern, uavCount, launchPointObj, corridorWaypointsObj, setLaunchPoints, partitionAngleDeg) {
  const cfg = getConfig();
  const [spacing, altitude] = computeTrackSpacing(dockClasses, cfg);
  const polygonLL = polygonObj.map((p) => [p.lat, p.lon]);
  const lp = launchPointObj ? [launchPointObj.lat, launchPointObj.lon] : null;

  // Corridor search pattern has its own coordinate system
  if (searchPattern === 'corridor') {
    return {
      ...generateCorridorPlan(polygonLL, spacing, altitude, uavCount, lp, corridorWaypointsObj, cfg),
      dock_classes: dockClasses || [],
    };
  }

  // Distributed shares polygon projection
  const [ptsM, refLat] = polygonToMeters(polygonLL);
  const autoAngleDeg = longestEdgeAngle(ptsM);
  const angleDeg = partitionAngleDeg != null ? partitionAngleDeg : autoAngleDeg;
  const angleRad = angleDeg * DEG2RAD;
  const lpM = lp ? toMeters(lp[0], lp[1], refLat) : null;

  const zones = generateDistributedPlan(ptsM, refLat, polygonLL, spacing, altitude, angleRad, lpM, uavCount, setLaunchPoints, cfg);

  // Assign per-zone altitude with separation
  const nz = zones.length;
  zones.forEach((z, i) => {
    z.altitude_m = _round(altitude + (nz - 1 - i) * cfg.ALTITUDE_SEPARATION_M, 1);
  });

  const maxUavDistKm = Math.max(...zones.map((z) => z.total_distance_km), 0);
  const remainingKm = Math.max(cfg.MIN_LAUNCH_ZONE_BUFFER_KM, cfg.FLIGHT_BUDGET_KM - cfg.SAFETY_RESERVE_KM - maxUavDistKm);
  const [lzPts, lzBuffer] = computeLaunchZone(polygonLL, remainingKm * 1000);
  const minLzBufferM = cfg.MIN_LAUNCH_ZONE_BUFFER_KM * 1000;
  const minLzOutwardM = Math.max(100, lzBuffer - minLzBufferM);
  const [minLzPts] = computeLaunchZone(polygonLL, minLzOutwardM);

  return {
    zones,
    all_feasible: zones.every((z) => z.feasible),
    total_distance_km: _round(zones.reduce((s, z) => s + z.total_distance_km, 0), 1),
    strip_count: zones.reduce((s, z) => s + z.strip_count, 0),
    estimated_time_min: _round(Math.max(...zones.map((z) => z.estimated_time_min), 0), 1),
    grid_angle_deg: _round(angleDeg, 1),
    altitude_m: _round(altitude, 1),
    altitude_separation_m: cfg.ALTITUDE_SEPARATION_M,
    launch_zone: lzPts.map((p) => ({ lat: _round(p[0], 6), lon: _round(p[1], 6) })),
    launch_zone_buffer_m: Math.round(lzBuffer),
    min_launch_zone: minLzPts.map((p) => ({ lat: _round(p[0], 6), lon: _round(p[1], 6) })),
    min_launch_zone_buffer_m: Math.round(minLzOutwardM),
    dock_classes: dockClasses || [],
  };
}
