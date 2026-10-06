/**
 * Zigzag track generation, strip assignment, and track distance calculation.
 */

import { projectAlongPerp } from './clipping.js';

export function computeTrackSpacing(dockClasses, config) {
  const { DOCK_PRESETS, FOV_HORIZONTAL_DEG, FOV_VERTICAL_DEG, CAMERA_PITCH_DEG,
          OVERLAP_FRACTION, DEG2RAD } = config;
  if (!dockClasses || !dockClasses.length) dockClasses = ['small'];
  const alt = Math.min(
    ...dockClasses.map((tc) => (DOCK_PRESETS[tc] || DOCK_PRESETS.small).altitude_m)
  );
  const hfovRad = FOV_HORIZONTAL_DEG * DEG2RAD;
  const vfovRad = FOV_VERTICAL_DEG * DEG2RAD;
  const nadirOffset = (90 + CAMERA_PITCH_DEG) * DEG2RAD;
  const cosOff = Math.cos(nadirOffset);
  const sinOff = Math.sin(nadirOffset);
  const bzNear = sinOff * Math.tan(vfovRad / 2) + cosOff;
  // Guard: bz_near <= 0 means camera looks above horizon — no valid ground swath
  if (bzNear <= 0) return [0, alt, 0];
  const halfSwath = alt * Math.tan(hfovRad / 2) / bzNear;
  const spacing = 2 * halfSwath * (1 - OVERLAP_FRACTION);
  if (spacing <= 0) return [0, alt, 0];
  // Demo-mode manual route offset overrides the camera-derived spacing.
  const override = config.TRACK_SPACING_OVERRIDE_M;
  return [override > 0 ? override : spacing, alt, halfSwath];
}

function scanLineIntersections(polygon, angleRad, offset) {
  const perpX = -Math.sin(angleRad), perpY = Math.cos(angleRad);
  const dirX = Math.cos(angleRad), dirY = Math.sin(angleRad);
  const intersections = [];
  const n = polygon.length;
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    const [ax, ay] = polygon[i];
    const [bx, by] = polygon[j];
    const da = ax * perpX + ay * perpY - offset;
    const db = bx * perpX + by * perpY - offset;
    if (Math.abs(da - db) < 1e-12) continue;
    if ((da >= 0 && db <= 0) || (da <= 0 && db >= 0)) {
      const t = da / (da - db);
      intersections.push([ax + t * (bx - ax), ay + t * (by - ay)]);
    }
  }
  intersections.sort((a, b) => (a[0] * dirX + a[1] * dirY) - (b[0] * dirX + b[1] * dirY));
  return intersections.length >= 2
    ? [intersections[0], intersections[intersections.length - 1]]
    : intersections;
}

export function generateZigzagTrack(polygon, spacing, angleRad) {
  // Single boustrophedon track built from the coverage lanes (see generateStrips),
  // alternating direction each lane.
  const strips = generateStrips(polygon, spacing, angleRad);
  const track = [];
  strips.forEach((linePts, i) => {
    track.push(...(i % 2 === 1 ? [...linePts].reverse() : linePts));
  });
  return [track, strips.length];
}

export function generateStrips(polygon, spacing, angleRad) {
  if (!polygon || polygon.length < 3 || spacing <= 0) return [];
  const projs = projectAlongPerp(polygon, angleRad);
  const pMin = Math.min(...projs), pMax = Math.max(...projs);
  const width = pMax - pMin;
  // Tile the cross-track width into n equal bands of pitch g <= spacing and run
  // one lane down the center of each: every route gap is exactly g and each
  // border gets g/2 — a uniform grid whose border standoff comes from the
  // tiling, never from the camera swath. Coverage stays complete because
  // computeTrackSpacing guarantees spacing <= 2*halfSwath, so g/2 <= halfSwath
  // and the outermost swaths image past both borders. (Previously the first
  // lane sat halfSwath inside the border — an overlap-dependent dead margin —
  // and the last lane was pinned at pMax - halfSwath, leaving one narrower
  // remainder gap.)
  const n = Math.max(1, Math.ceil(width / spacing - 1e-9));
  const g = width / n;
  const strips = [];
  for (let i = 0; i < n; i++) {
    const linePts = scanLineIntersections(polygon, angleRad, pMin + (i + 0.5) * g);
    if (linePts.length >= 2) strips.push(linePts);
  }
  return strips;
}

export function assignStripsEqualDistance(strips, nZones) {
  const n = strips.length;
  if (n <= nZones) return strips.map((_, i) => [i, i + 1]);

  const stripDists = strips.map((strip, i) => {
    let d = trackDistance(strip);
    if (i > 0) {
      const prev = strips[i - 1];
      d += Math.hypot(strip[0][0] - prev[prev.length - 1][0],
                      strip[0][1] - prev[prev.length - 1][1]);
    }
    return d;
  });

  const totalDist = stripDists.reduce((s, d) => s + d, 0);
  let target = totalDist / nZones;
  const assignments = [];
  let start = 0;

  for (let z = 0; z < nZones - 1; z++) {
    let cum = 0, best = start + 1;
    for (let j = start; j < n; j++) {
      cum += stripDists[j];
      if (cum >= target) {
        const without = cum - stripDists[j];
        best = (j > start && Math.abs(without - target) < Math.abs(cum - target)) ? j : j + 1;
        break;
      }
      best = j + 1;
    }
    const remainingStrips = n - best;
    const remainingZones = nZones - z - 1;
    if (remainingStrips < remainingZones) best = n - remainingZones;

    assignments.push([start, best]);
    start = best;
    const remDist = stripDists.slice(start).reduce((s, d) => s + d, 0);
    if (remainingZones > 0) target = remDist / remainingZones;
  }

  assignments.push([start, n]);
  return assignments;
}

export function trackDistance(track) {
  let total = 0;
  for (let i = 0; i < track.length - 1; i++) {
    total += Math.hypot(track[i + 1][0] - track[i][0], track[i + 1][1] - track[i][1]);
  }
  return total;
}
